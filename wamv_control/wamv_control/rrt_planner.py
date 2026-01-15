#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import math

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # --- Configuración RRT ---
        self.goal = [-5, 40.0]  
        self.step_size = 3.0       
        self.max_iter = 3000       
        self.search_radius = 80.0  
        self.collision_radius = 5.0 

        # Estado
        self.robot_pos = None
        self.obstacles = [] 
        self.current_path = [] 
        
        # TF Buffer 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS Latched (Memoria)
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Suscriptores
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidar/lidar_wamv/points', self.lidar_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publicador
        self.path_pub = self.create_publisher(Path, '/rrt/path', qos_latched)
        
        self.create_timer(1.0, self.plan_path)
        self.get_logger().info("RRT Planner con Smoothing Iniciado")

    # --- ALGORITMO DE LA DIAPOSITIVA (Path Smoothing) ---
    def smooth_path(self, path):
        """
        Implementación exacta del algoritmo 'smoothRRT' de la imagen subida.
        Trata de saltarse nodos intermedios si no hay colisión directa.
        """
        if len(path) < 3:
            return path
            
        # 1. Inicializar Ws con el primer nodo
        smoothed_path = [path[0]] 
        
        # Iterador 'j' del algoritmo (empieza mirando al siguiente del siguiente)
        # Ajustamos índices porque Python empieza en 0
        j = 1 
        N = len(path)

        while j < N - 1:
            # ws: último nodo añadido a la ruta suavizada
            ws = smoothed_path[-1]
            # w+: el candidato para saltar (j+1)
            w_plus = path[j+1]

            # existFeasiblePath(T, ws, w+) == FALSE ?
            # (Mi check_collision devuelve True si HAY colisión, es decir, si NO es feasible)
            if self.check_collision(ws, w_plus):
                # Si hay colisión al intentar saltar, no podemos hacer el atajo.
                # Añadimos el nodo intermedio 'w' (path[j]) a la ruta suavizada.
                w = path[j]
                smoothed_path.append(w)
            
            # Avanzamos j
            j += 1
        
        # Añadir el último nodo (wN)
        smoothed_path.append(path[-1])
        
        return smoothed_path
    # ----------------------------------------------------

    def odom_callback(self, msg):
        self.robot_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def goal_callback(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]
        self.current_path = [] 
        self.plan_path()       

    def lidar_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
            cloud_out = do_transform_cloud(msg, trans)
            points = []
            for p in point_cloud2.read_points(cloud_out, field_names=("x", "y"), skip_nans=True):
                if self.robot_pos:
                    dist = math.hypot(p[0] - self.robot_pos[0], p[1] - self.robot_pos[1])
                    if dist < 50.0: 
                        points.append([p[0], p[1]])
            self.obstacles = points 
        except: pass

    def check_collision(self, p1, p2):
        if not self.obstacles: return False
        dist_seg = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        steps = int(dist_seg / 1.0) + 1 
        for i in range(steps):
            t = i / steps
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            for obs in self.obstacles:
                if abs(obs[0] - x) > self.collision_radius + 1.0: continue 
                if math.hypot(x - obs[0], y - obs[1]) < self.collision_radius: return True 
        return False

    def is_path_safe(self):
        if not self.current_path or len(self.current_path) < 2: return False 
        for i in range(len(self.current_path) - 1):
            if self.check_collision(self.current_path[i], self.current_path[i+1]): return False 
        return True

    def plan_path(self):
        if not self.robot_pos: return
        if math.hypot(self.goal[0]-self.robot_pos[0], self.goal[1]-self.robot_pos[1]) < 3.0: return
        if self.current_path and self.is_path_safe(): return 
        
        # --- ALGORITMO RRT (Diapositiva 1) ---
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
                    # Reconstruir camino crudo (W)
                    path = [self.goal, new_node]
                    curr = nearest_idx
                    while curr is not None:
                        path.append(tree[curr])
                        curr = parents[curr]
                    path.reverse()
                    
                    # --- APLICAR SMOOTHING (Diapositiva 2) ---
                    final_path = self.smooth_path(path)
                    
                    self.current_path = final_path 
                    self.publish_path_msg(final_path)
                    return

    def publish_path_msg(self, points):
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for p in points:
            pose = PoseStamped()
            
            # --- ARREGLO DEL FALLO "BUFFER EXCEEDED" ---
            # Es obligatorio poner header a cada pose individual
            pose.header.frame_id = "odom"
            pose.header.stamp = msg.header.stamp
            # -------------------------------------------
            
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            msg.poses.append(pose)
        
        self.path_pub.publish(msg)
        self.get_logger().info(f"¡Ruta publicada ({len(points)} nodos)!")

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
