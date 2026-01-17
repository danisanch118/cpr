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
from rclpy.qos import QoSProfile, DurabilityPolicy

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')


        self.lookAD = 6.0
        self.max_linear_speed = 2.0
        self.max_angular_speed = 1.8
        self.wheelbase = 2.06
        self.waypoint_tolerance = 1.0
        #self.waypoints = np.array([[1.5,1],[2.5, 2.5],[4.0, 4.0], [6.0, 6.0], [8.0, 8.0],[10.0, 10.0],[13.0, 13.0],[18.0, 15.0], [24.0, 16.0],[32.0, 16.0]])
        #self.waypoints = np.array([[1.0,1.0],[8.0, 8.0], [24.0, 16.0],[40.0, 26.0]])
        self.waypoints =[]
        self.finish = False
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        #WAMV
        self.current_pose_x= 0.0
        self.current_pose_y= 0.0
        self.current_yaw = 0.0
        self.wap_index = 0
        self.last_found_index = 0
        self.data_odom_received = False

        #Subscriptores
        self.create_subscription(Odometry, '/odometry/filtered',self.odom_callback,10) 
        self.create_subscription(Path, '/rrt/path', self.path_callback, qos_latched) 
        #Publicadores
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel', 10)

        self.left_thruster_pub  = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
       # self.position_pub = self.create_publisher(PoseStamped, '/pure_pursuit/current_position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/debug', 10)
        # Publisher para markers de waypoints (esferas verdes en RViz)
        #self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/waypoints', 10)
     
        #Timer
        self.timer = self.create_timer(0.1, self.control_loop)

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

    def path_callback(self, msg):
        new_waypoints = []
        for pose in msg.poses:
            new_waypoints.append([pose.pose.position.x, pose.pose.position.y])
        
        self.waypoints = np.array(new_waypoints)
        
        # --- CORRECCIONES ---
        self.wap_index = 0
        self.last_found_index = 0   
        self.finish = False         
     
        
        self.get_logger().info(f"Nueva ruta recibida con {len(self.waypoints)} puntos")

    def get_target_point(self, lookahead):
        """
        Busca el punto de intersección entre el círculo de radio 'lookahead'
        y los segmentos del camino.
        Devuelve: (goal_x, goal_y)
        """
        goal_point = None
        
        # Buscamos desde el último índice conocido hacia adelante
        for i in range(self.last_found_index, len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i+1]
            
            # Vectores
            V = p2 - p1
            rob_pos = np.array([self.current_pose_x, self.current_pose_y])
            Q = p1 - rob_pos
            
            # Ecuación cuadrática para intersección Línea-Círculo:
            # |(P1 + t*V) - Robot|^2 = R^2
            a = np.dot(V, V)
            b = 2 * np.dot(Q, V)
            c = np.dot(Q, Q) - lookahead**2
            
            discriminant = b**2 - 4*a*c
            
            if discriminant >= 0:
                # Solucionamos t
                sqrt_disc = math.sqrt(discriminant)
                t1 = (-b - sqrt_disc) / (2*a)
                t2 = (-b + sqrt_disc) / (2*a)
                
                # Buscamos la intersección válida más adelantada en el segmento (t entre 0 y 1)
                t = -1.0
                if 0 <= t2 <= 1:
                    t = t2
                elif 0 <= t1 <= 1:
                    t = t1
                
                if t >= 0:
                    goal_point = p1 + t * V
                    dx = goal_point[0] - self.current_pose_x
                    dy = goal_point[1] - self.current_pose_y
                    
                    # Rotamos para ver si la X local es positiva (delante)
                    local_x = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
                    
                    if local_x > 0:
                        # punto delante
                        self.last_found_index = i
                        return goal_point
                    else:
                        # El punto está DETRÁS
                        
                        continue

        # Si no encontramos intersección (estamos muy lejos o al final),
        # apuntamos al último waypoint
        return self.waypoints[-1]

    def control_loop(self):
            self.get_logger().info(f"POSICIÓN ACTUAL DEL BARCO → X: {self.current_pose_x:+.3f} m | Y: {self.current_pose_y:+.3f} m | Yaw: {math.degrees(self.current_yaw):+.1f}°")
            if not self.data_odom_received:
                self.get_logger().warn('Esperando datos de odometry/filtered')
                return
            
            if len(self.waypoints) == 0:
              self.get_logger().info("Esperando ruta del RRT...", throttle_duration_sec=2)
              self.publish_cmd(0.0, 0.0) 
              return
                
            target_point = self.get_target_point(self.lookAD)     
            
            xg, yg = target_point[0], target_point[1]

            
            dx = xg - self.current_pose_x
            dy = yg - self.current_pose_y
            

            x_local =  dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)   
            y_local = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)   

            curvature = 2.0 * y_local / (self.lookAD**2)
        
            dist_to_last = np.linalg.norm([self.current_pose_x - self.waypoints[-1][0], 
                                        self.current_pose_y - self.waypoints[-1][1]])
            
            if dist_to_last < self.waypoint_tolerance:
                self.get_logger().info('¡Ruta completada!')
                self.finish = True
               

            if self.finish:
                self.publish_cmd(0.0,0.0)
                return    
                

            linear = 1.2 / (1 + abs(curvature) * 2.0)  
            angular = linear * curvature
            #self.publish_thrusters(linear, angular)    

            self.publish_cmd(linear, angular)
            self.publish_debug_markers(target_point)
                  
        
    def publish_cmd(self, linear, angular):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'wamv/wamv/base_link'
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def publish_debug_markers(self, target):
        marker_array = MarkerArray()
        
        
        delete_marker = Marker()
        delete_marker.header.frame_id = "odom"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = "odom"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3 
            m.color.a = 1.0; m.color.g = 1.0 # Verde
            m.pose.position.x = wp[0]
            m.pose.position.y = wp[1]
            m.pose.position.z = 0.5
            marker_array.markers.append(m)

        
        t = Marker()
        t.header.frame_id = "odom"
        t.id = 9999 
        t.type = Marker.SPHERE
        t.action = Marker.ADD
        t.scale.x = 0.6; t.scale.y = 0.6; t.scale.z = 0.6
        t.color.a = 1.0; t.color.r = 1.0 # Rojo
        t.pose.position.x = target[0]
        t.pose.position.y = target[1]
        t.pose.position.z = 0.8
        marker_array.markers.append(t)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


