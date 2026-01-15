#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import math
from std_msgs.msg import Float64
import numpy as np
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy # <--- Importante

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.lookAD = 1.0
        self.waypoint_tolerance = 1.0 # Tolerancia para pasar al siguiente punto
        self.waypoints = []
        
        # QoS Latched (Debe coincidir con el RRT)
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Suscripciones
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)   
        self.create_subscription(Path, '/rrt/path', self.path_callback, qos_latched)

        # WAMV States
        self.current_pose_x= 0.0
        self.current_pose_y= 0.0
        self.current_yaw = 0.0
        self.wap_index = 0
        self.data_odom_received = False

        # Publicadores
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel', 10)
        self.left_thruster_pub  = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/waypoints', 10)
     
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Pure Pursuit Reactivo Iniciado')

    def odom_callback(self,msg):
         self.current_pose_x = msg.pose.pose.position.x
         self.current_pose_y = msg.pose.pose.position.y
         q = msg.pose.pose.orientation
         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
         self.data_odom_received = True

    def path_callback(self, msg):
        new_waypoints = []
        for pose in msg.poses:
            new_waypoints.append([pose.pose.position.x, pose.pose.position.y])
        
        self.waypoints = np.array(new_waypoints)
        self.wap_index = 0 
        self.get_logger().info(f"Nueva ruta recibida con {len(self.waypoints)} puntos")

    def control_loop(self):
        if not self.data_odom_received:
            return

        if len(self.waypoints) == 0:
            self.publish_cmd(0.0, 0.0) 
            return

        # --- Lógica Anti-Spinning (Parar al llegar) ---
        last_wp = self.waypoints[-1]
        dist_to_goal = math.hypot(last_wp[0] - self.current_pose_x, last_wp[1] - self.current_pose_y)

        if dist_to_goal < 2.0:
             self.get_logger().info(f"En meta ({dist_to_goal:.1f}m). Motores OFF.", throttle_duration_sec=2)
             self.publish_cmd(0.0, 0.0)
             return
        # ---------------------------------------------

        # Chequear indice
        if self.wap_index >= len(self.waypoints):
            # Si se nos acabó la lista pero seguimos lejos, apuntamos al último
            self.wap_index = len(self.waypoints) - 1

        # Pure Pursuit
        target = self.waypoints[self.wap_index]      
        dx = target[0] - self.current_pose_x
        dy = target[1] - self.current_pose_y
        
        # Transformación local
        x_local =  dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)   
        y_local = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)   
        D = math.sqrt(x_local**2 + y_local**2)
        
        # Check waypoint alcanzado (solo si NO es el último)
        if D < self.waypoint_tolerance and self.wap_index < len(self.waypoints) - 1:
            self.wap_index += 1
            return 

        # Cálculo curvatura
        ld = max(D, self.lookAD)    
        curvature = 2.0 * y_local / (ld * ld)

        linear = 0.8 # Velocidad constante (puedes subirla)
        angular = linear * curvature

        # Limites
        angular = max(min(angular, 1.0), -1.0)

        self.publish_cmd(linear, angular)
        self.publish_markers()

    def publish_markers(self):
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
            marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 0.5
            marker.color.a = 1.0
            
            if i == self.wap_index: # Verde el actual
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
                marker.scale.x = 1.0; marker.scale.y = 1.0; marker.scale.z = 1.0
            else: # Rojo el resto
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
                marker.color.a = 0.3

            marker_array.markers.append(marker)
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
