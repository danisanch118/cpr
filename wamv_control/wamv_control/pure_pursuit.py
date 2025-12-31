#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import math
import tf2_ros
from tf2_geometry_msgs import PoseStamped  
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')


        self.lookAD = 1.0
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
        self.data_odom_received = False

        #Subscriptores
        self.create_subscription(Odometry, '/odometry/filtered',self.odom_callback,10)   
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
        
      
    
    def odom_callback(self,msg):
         self.current_pose_x = msg.pose.pose.position.x
         self.current_pose_y = msg.pose.pose.position.y
         q = msg.pose.pose.orientation
        # Fórmula estándar para yaw (rotación alrededor de Z)
         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

         self.data_odom_received = True

    

    def control_loop(self):
            self.get_logger().info(f"POSICIÓN ACTUAL DEL BARCO → X: {self.current_pose_x:+.3f} m | Y: {self.current_pose_y:+.3f} m | Yaw: {math.degrees(self.current_yaw):+.1f}°")
            if not self.data_odom_received:
                self.get_logger().warn('Esperando datos de odometry/filtered')
                return
        
                
            if self.wap_index >= len(self.waypoints):
                        self.publish_cmd(0.0, 0.0)  
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
                    self.publish_cmd(0.0, 0.0)
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
            # pose_msg = PoseStamped()
            # pose_msg.header.stamp = self.get_clock().now().to_msg()
            # pose_msg.header.frame_id = "odom"
            # pose_msg.pose.position.x = self.current_pose_x
            # pose_msg.pose.position.y = self.current_pose_y
            # pose_msg.pose.position.z = 0

            
        

            # self.position_pub.publish(pose_msg)

            # === Publicar markers de waypoints ===
            marker_array = MarkerArray()
            for i, wp in enumerate(self.waypoints):
                marker = Marker()
                marker.header.frame_id = "odom"
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
            current_marker.header.frame_id = "odom"
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



def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


