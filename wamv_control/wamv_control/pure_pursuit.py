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
from nav_msgs.msg import Odometry, Path 
from visualization_msgs.msg import Marker, MarkerArray

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.lookAD = 1.0
        self.max_linear_speed = 2.0
        self.max_angular_speed = 1.8
        self.wheelbase = 2.06
        self.waypoint_tolerance = 0.7
        
        # Inicializamos vacio para esperar al RRT
        self.waypoints = []
        
        # --- Suscripciones ---
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)   
        self.create_subscription(Path, '/rrt/path', self.path_callback, 10)

        #WAMV States
        self.current_pose_x= 0.0
        self.current_pose_y= 0.0
        self.current_yaw = 0.0
        self.wap_index = 0
        self.data_odom_received = False

        # --- Publicadores ---
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel', 10)
        self.left_thruster_pub  = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/pure_pursuit/current_position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/waypoints', 10)
     
        #Timer
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Nodo Pure pursuit iniciado')

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
        # 1. Verificaciones iniciales
        if not self.data_odom_received:
            self.get_logger().warn('Esperando datos de odometry/filtered...', throttle_duration_sec=2)
            return

        if len(self.waypoints) == 0:
            self.get_logger().info("Esperando ruta del RRT...", throttle_duration_sec=2)
            self.publish_cmd(0.0, 0.0) # Por seguridad, parar si no hay ruta
            return

        # --- CORRECCIÓN 2: Todo esto debe estar FUERA del 'if len == 0' ---
        
        self.get_logger().info(f"POSICIÓN: X: {self.current_pose_x:.2f} | Y: {self.current_pose_y:.2f}")

        # Chequear si hemos terminado
        if self.wap_index >= len(self.waypoints):
            self.publish_cmd(0.0, 0.0)  
            self.get_logger().info('¡Ruta completada!', throttle_duration_sec=2)
            return

        # 2. Lógica Pure Pursuit
        target = self.waypoints[self.wap_index]      
        xg, yg = target[0], target[1]
        
        dx = xg - self.current_pose_x
        dy = yg - self.current_pose_y
        distance = math.sqrt(dx*dx + dy*dy)

        # Transformación al frame local del barco
        x_local =  dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)   
        y_local = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)   

        # Distancia euclídea al waypoint
        D = math.sqrt(x_local**2 + y_local**2)

        # Log de depuración
        # self.get_logger().info(f"WP {self.wap_index} Dist: {distance:.1f} LocalY: {y_local:.1f}")
        
        # Check de tolerancia
        if D < self.waypoint_tolerance:
            self.get_logger().info(f'Waypoint {self.wap_index} alcanzado!')
            self.wap_index += 1
            return # Esperamos al siguiente ciclo para calcular el nuevo target

        # Cálculo de control (Curvatura)
        ld = max(D, self.lookAD)    
        curvature = 2.0 * y_local / (ld * ld)

        linear = 0.5  
        angular = linear * curvature

        # 3. Publicar comando
        self.publish_cmd(linear, angular)

        # 4. Visualización (Markers)
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
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.a = 1.0
            # Verde si es el actual, rojo apagado si no
            if i == self.wap_index:
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
                marker.scale.x = 2.0; marker.scale.y = 2.0; marker.scale.z = 2.0
            else:
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
                marker.color.a = 0.3 # Transparente

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

if __name__ == '__main__':
    main()
