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
        self.waypoints = np.array([[10.0, 0.0], [20.0, 10.0], [30.0, 0.0]])

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

            linear = 0.9  
            angular = linear * curvature
            #self.publish_thrusters(linear, angular)    

            self.publish_cmd(linear, angular)
        
    def publish_cmd(self, linear, angular):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'wamv/wamv/base_link'
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    # def publish_thrusters(self, linear_vel, angular_vel):
    #     """
    #     Convierte velocidad lineal (m/s) y angular (rad/s) en empuje diferencial
    #     para los dos motores del WAM-V.
    #     Fórmula clásica: 
    #         thrust_right = V + (ω × L/2)
    #         thrust_left  = V - (ω × L/2)
    #     donde L = distancia entre motores ≈ 2.06 m (tu wheelbase)
    #     """
    #     V = linear_vel
    #     omega = angular_vel
    #     L = 2.06  # distancia entre thrusters en metros (exacto para WAM-V 16')

    #     # Empuje base + diferencial
    #     thrust_right = V + (omega * L / 2.0)
    #     thrust_left  = V - (omega * L / 2.0)

    #     # Saturamos (el simulador acepta hasta ~150 N sin problemas)
    #     thrust_right = np.clip(thrust_right * 500.0, -2000, 2000)  # ×60 para que avance bien
    #     thrust_left  = np.clip(thrust_left  * 500.0, -2000, 2000)

    #     # Publicamos
    #     msg_left = Float64()
    #     msg_right = Float64()
    #     msg_left.data  = float(thrust_left)
    #     msg_right.data = float(thrust_right)

    #     self.left_thruster_pub.publish(msg_left)
    #     self.right_thruster_pub.publish(msg_right)    


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


