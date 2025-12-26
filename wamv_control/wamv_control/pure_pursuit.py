#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, Imu
import math
import tf2_ros
from tf2_geometry_msgs import PoseStamped  
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
R_EARTH = 6378137.0 
ORIGIN_LAT = -33.722768664380965
ORIGIN_LON = 150.67399101080872
class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')


        self.lookAD = 2.0
        self.max_linear_speed = 2.0
        self.max_angular_speed = 1.8
        self.wheelbase = 2.06
        self.waypoint_tolerance = 0.7
        self.waypoints = np.array([[4.0, 4.0], [6.0, 6.0], [8.0, 8.0],[10.0, 10.0],[13.0, 13.0],[18.0, 15.0], [24.0, 16.0],[32.0, 16.0]])

        #WAMV
        self.current_pose_x= 0.0
        self.current_pose_y= 0.0
        self.current_yaw = 0.0
        self.wap_index = 0
        self.gps_data_received = False
        self.imu_data_received = False

        #Subscriptores
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data',self.imu_callback,10)       
        #Publicadores
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel', 10)

        self.left_thruster_pub  = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/pure_pursuit/current_position', 10)

        # Publisher para markers de waypoints (esferas verdes en RViz)
        self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/waypoints', 10)
     
        #Timer
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Nodo Pure pursuit iniciado'
                               )
        
     
    def latlon_to_meters(self, lat, lon):
        """
        Convierte Latitud/Longitud a coordenadas cartesianas (x, y) en metros.
        X (Este) se corresponde a Longitud, Y (Norte) a Latitud.
        """
        # Distancia en Y (Latitud)
        y = R_EARTH * math.radians(lat - ORIGIN_LAT)
        
        # Factor de correcci para la Longitud (coseno de la latitud)
        cos_lat = math.cos(math.radians(ORIGIN_LAT))
        
        # Distancia en X (Longitud)
        x = R_EARTH * math.radians(lon - ORIGIN_LON) * cos_lat
        
        return x, y    
    
    def gps_callback(self, msg):
        self.current_pose_x, self.current_pose_y = self.latlon_to_meters(
            msg.latitude,
            msg.longitude
        )
        self.gps_data_received = True

    def imu_callback(self, msg):
        q = msg.orientation
        self.current_orientation = q
        quaternion = [q.x, q.y, q.z, q.w]

        siny_cosp = 2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        cosy_cosp = 1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_data_received = True

    def control_loop(self):
            self.get_logger().info(f"POSICIÓN ACTUAL DEL BARCO → X: {self.current_pose_x:+.3f} m | Y: {self.current_pose_y:+.3f} m | Yaw: {math.degrees(self.current_yaw):+.1f}°")
            if not self.gps_data_received or not self.imu_data_received:
                self.get_logger().warn('Esperando datos de GPS o IMU')
                return
        
                
            if self.wap_index >= len(self.waypoints):
                        self.publish_thrusters(0.0, 0.0)    
                        self.get_logger().info('¡Ruta completada!')
                        return

            # Pure Pursuit: 
            target = self.waypoints[self.wap_index]      
            
            xg, yg = target[0], target[1]

            
            dx = xg - self.current_pose_x
            dy = yg - self.current_pose_y
            distance = math.sqrt(dx*dx + dy*dy)

            x_local =  dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)   
            y_local = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)   

        
            D2 = x_local*x_local + y_local*y_local
            D  = math.sqrt(D2)

            self.get_logger().info(
            f"WP {self.wap_index+1}/{len(self.waypoints)} → "
            f"Dist: {distance:5.1f} m | "
            f"Local: ({x_local:+5.1f}, {y_local:+5.1f}) | "
            f"Yaw: {math.degrees(self.current_yaw):+6.1f}°"
        )
            
            if D < self.waypoint_tolerance:          # ← mejor usar D que D2
                self.get_logger().info(f'Waypoint {self.wap_index + 1} alcanzado!')
                self.wap_index += 1
                # Si era el último, paramos
                if self.wap_index >= len(self.waypoints):
                    self.publish_thrusters(0.0, 0.0)
                    self.get_logger().info('¡Ruta completada!')
                    return
                else:
                    return
                
            #Cogemos lookahead o distancia segun estemos cerca o no
            ld = max(D, self.lookAD)    
            curvature = 2.0 * y_local / (ld * ld)

            linear = 0.5  
            angular = linear * curvature
            #self.publish_thrusters(linear, angular)    

            self.publish_cmd(linear, angular)
                       # === Publicar posición actual para plotting y RViz (sin orientación) ===
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "wamv/wamv/base_link"
            pose_msg.pose.position.x = self.current_pose_x
            pose_msg.pose.position.y = self.current_pose_y
            pose_msg.pose.position.z = 0.0

                     # === Orientación directa desde la IMU ===
            if self.current_orientation is not None:
                 pose_msg.pose.orientation = self.current_orientation
            else:
             # Valor por defecto si aún no ha llegado IMU
                pose_msg.pose.orientation.w = 1.0
                pose_msg.pose.orientation.x = pose_msg.pose.orientation.y = pose_msg.pose.orientation.z = 0.0

            self.position_pub.publish(pose_msg)

            # === Publicar markers de waypoints ===
            marker_array = MarkerArray()
            for i, wp in enumerate(self.waypoints):
                marker = Marker()
                marker.header.frame_id = "wamv/wamv/base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "waypoints"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = wp[0]
                marker.pose.position.y = wp[1]
                marker.pose.position.z = 0.5
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0 if i == self.wap_index else 0.3
                marker.color.b = 0.0
                marker_array.markers.append(marker)

            # Marker especial para el waypoint actual (más grande y brillante)
            current_marker = Marker()
            current_marker.header.frame_id = "wamv/wamv/base_link"
            current_marker.header.stamp = self.get_clock().now().to_msg()
            current_marker.ns = "waypoints"
            current_marker.id = len(self.waypoints)
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            current_marker.pose.position.x = self.waypoints[self.wap_index][0]
            current_marker.pose.position.y = self.waypoints[self.wap_index][1]
            current_marker.pose.position.z = 1.0
            current_marker.pose.orientation.w = 1.0
            current_marker.scale.x = current_marker.scale.y = current_marker.scale.z = 2.0
            current_marker.color.a = 1.0
            current_marker.color.r = 1.0
            current_marker.color.g = 1.0
            current_marker.color.b = 0.0
            marker_array.markers.append(current_marker)

            self.marker_pub.publish(marker_array)
        
    def publish_cmd(self, linear, angular):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'wamv/wamv/base_link'
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def publish_thrusters(self, linear_vel, angular_vel):
        """
        Convierte velocidad lineal (m/s) y angular (rad/s) en empuje diferencial
        para los dos motores del WAM-V.
        Fórmula clásica: 
            thrust_right = V + (ω × L/2)
            thrust_left  = V - (ω × L/2)
        donde L = distancia entre motores ≈ 2.06 m (tu wheelbase)
        """
        V = linear_vel
        omega = angular_vel
        L = 2.06  # distancia entre thrusters en metros (exacto para WAM-V 16')

        # Empuje base + diferencial
        thrust_right = V + (omega * L / 2.0)
        thrust_left  = V - (omega * L / 2.0)

        # Saturamos (el simulador acepta hasta ~150 N sin problemas)
        thrust_right = np.clip(thrust_right * 300.0, -2000, 2000)  # ×60 para que avance bien
        thrust_left  = np.clip(thrust_left  * 300.0, -2000, 2000)

        # Publicamos
        msg_left = Float64()
        msg_right = Float64()
        msg_left.data  = float(thrust_left)
        msg_right.data = float(thrust_right)

        self.left_thruster_pub.publish(msg_left)
        self.right_thruster_pub.publish(msg_right)    


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


