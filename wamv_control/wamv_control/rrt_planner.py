#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2
import tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import math

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # --- PARÁMETROS ---
        self.declare_parameter('goal_x', -47.0)
        self.declare_parameter('goal_y', 90.0)
        
        gx = self.get_parameter('goal_x').get_parameter_value().double_value
        gy = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal = [gx, gy]
        self.add_on_set_parameters_callback(self.parameters_callback)

        # CONFIGURACIÓN RRT
        self.step_size = 3.0       
        self.max_iter = 5000       
        self.search_radius = 80.0
        self.collision_radius = 4.0 

        # Estado
        self.robot_pos = None
        self.obstacles = [] 
        self.current_path = [] 
        
        # TF Buffer 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS Latched
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Suscriptores y Publicadores
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/rrt/path', qos_latched)
        
        # Timer 
        self.create_timer(0.5, self.plan_path)
        self.get_logger().info("RRT Planner LISTO - Esperando Odometría y LiDAR...")

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'goal_x':
                self.goal[0] = param.value
                self.current_path = [] 
            if param.name == 'goal_y':
                self.goal[1] = param.value
                self.current_path = [] 
        self.get_logger().info(f"Nuevo Goal: {self.goal}")
        return_result = SetParametersResult()
        return_result.successful = True
        return return_result
    
    def odom_callback(self, msg):
        self.robot_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def goal_callback(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]
        self.current_path = [] 
        self.get_logger().info(f"Nuevo Goal recibido por tópico: {self.goal}")

    def lidar_callback(self, msg):
        try:
            if self.robot_pos is None: return

            trans = self.tf_buffer.lookup_transform(
                'odom', 
                msg.header.frame_id, 
                rclpy.time.Time(nanoseconds=0)
            )
            
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            q = trans.transform.rotation
            
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            new_obstacles = []
            
            for p in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
                
                if not math.isfinite(p[0]) or not math.isfinite(p[1]): 
                    continue

                # Transformación manual
                x_odom = p[0] * math.cos(yaw) - p[1] * math.sin(yaw) + tx
                y_odom = p[0] * math.sin(yaw) + p[1] * math.cos(yaw) + ty
                
                # Calcular distancia al robot
                dist = math.hypot(x_odom - self.robot_pos[0], y_odom - self.robot_pos[1])

                
                if 2.0 < dist < 50.0:
                    new_obstacles.append([x_odom, y_odom])
            
            # Guardamos 1 de cada 10 para velocidad
            self.obstacles = new_obstacles[::10] 

        except Exception as e:
            pass

    def check_collision(self, p1, p2):
        if not self.obstacles or len(self.obstacles) < 5: 
            return False
        
        obs_array = np.array(self.obstacles)
        dist_seg = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        steps = int(dist_seg / 0.5) + 1 
        
        for i in range(steps):
            t = i / steps
            curr_x = p1[0] + (p2[0] - p1[0]) * t
            curr_y = p1[1] + (p2[1] - p1[1]) * t
            
            distances_sq = np.sum((obs_array - np.array([curr_x, curr_y]))**2, axis=1)
            if np.any(distances_sq < self.collision_radius**2):
                return True 
        return False

    def is_path_safe(self):
        if not self.current_path or len(self.current_path) < 2: return False 
        for i in range(len(self.current_path) - 1):
            if self.check_collision(self.current_path[i], self.current_path[i+1]):
                self.get_logger().warn("¡Ruta bloqueada! Re-planificando...")
                return False 
        return True

    def smooth_path(self, path):
        if len(path) < 3: return path
        smoothed_path = [path[0]] 
        j = 1 
        N = len(path)
        while j < N - 1:
            ws = smoothed_path[-1]
            w_plus = path[j+1]
            if self.check_collision(ws, w_plus):
                w = path[j]
                smoothed_path.append(w)
            j += 1
        smoothed_path.append(path[-1])
        return smoothed_path

    def densify_path(self, path, resolution=2.0):
        if not path or len(path) < 2: return path
        dense_path = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i+1])
            dist = np.linalg.norm(p2 - p1)
            num_steps = int(dist / resolution)
            if num_steps == 0:
                dense_path.append(path[i])
                continue
            for j in range(num_steps):
                alpha = j / num_steps
                new_p = p1 + (p2 - p1) * alpha
                dense_path.append(new_p.tolist())
        dense_path.append(path[-1])
        return dense_path            

    def publish_path_msg(self, points):
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            msg.poses.append(pose)
        self.path_pub.publish(msg)
        self.get_logger().info(f"--> RUTA ENVIADA: {len(points)} puntos")

    def plan_path(self):
        if not self.robot_pos: 
            return
        
        # Validar si el propio robot está bloqueado al inicio
        if self.obstacles:
             obs_array = np.array(self.obstacles)
             dists = np.sum((obs_array - np.array(self.robot_pos))**2, axis=1)
             if np.any(dists < self.collision_radius**2):
                 self.get_logger().error(f"¡ESTOY ATASCADO! Obstáculos a menos de {self.collision_radius}m", once=True)
                 # Intentamos planificar igual, pero avisando

        if math.hypot(self.goal[0]-self.robot_pos[0], self.goal[1]-self.robot_pos[1]) < 3.0: 
            return

        if self.current_path and self.is_path_safe(): 
            return 
        
        # --- RRT ---
        tree = [self.robot_pos] 
        parents = {0: None}

        for i in range(self.max_iter):
            if np.random.rand() < 0.15: rand_pt = np.array(self.goal)
            else: rand_pt = np.array(self.robot_pos) + np.random.uniform(-self.search_radius, self.search_radius, 2)

            dists = [np.linalg.norm(np.array(node) - rand_pt) for node in tree]
            nearest_idx = np.argmin(dists)
            nearest_node = tree[nearest_idx]

            direction = rand_pt - nearest_node
            length = np.linalg.norm(direction)
            if length == 0: continue
            new_node = nearest_node + (direction / length) * min(self.step_size, length)

            if not self.check_collision(nearest_node, new_node):
                tree.append(new_node)
                new_idx = len(tree) - 1
                parents[new_idx] = nearest_idx
                
                if np.linalg.norm(new_node - np.array(self.goal)) < self.step_size:
                    path = [self.goal, new_node]
                    curr = nearest_idx
                    while curr is not None:
                        path.append(tree[curr])
                        curr = parents[curr]
                    path.reverse()
                    
                    smoothed_path = self.smooth_path(path)
                   
                    final_path = self.densify_path(smoothed_path, resolution=2.0)
                    
                    self.current_path = final_path 
                    self.publish_path_msg(final_path)
                    return
        
        # Si llega aquí, es que falló
        self.get_logger().warn(f"RRT Falló: No encontré camino tras {self.max_iter} intentos", once=True)

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()