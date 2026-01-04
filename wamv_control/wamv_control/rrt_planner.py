import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import math

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # --- Configuración RRT ---
        self.goal = [10.0, 5.0]  # Ajusta esto a una meta libre en tu mapa
        self.step_size = 3.0       # Aumentado para dar pasos más largos
        self.max_iter = 2000       
        self.search_radius = 80.0  
        
        # ### CAMBIO 1: Aumentar radio de seguridad (Inflation)
        # El WAM-V es grande y tiene inercia. 2.5m es poco. 
        # Ponemos 5.0m o 6.0m para que se aleje de la costa.
        self.collision_radius = 6.0 

        # Estado
        self.robot_pos = None
        self.obstacles = [] 
        self.current_path = [] # ### NUEVO: Guardamos la ruta actual para validarla
        
        # TF Buffer 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Suscriptores
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidar/lidar_wamv/points', self.lidar_callback, 10)

        # Publicador
        self.path_pub = self.create_publisher(Path, '/rrt/path', 10)
        
        # ### CAMBIO 2: Timer más lento
        # Antes estaba a 1.0s. Lo subimos a 2.0s, pero además añadiremos lógica 
        # para NO calcular si no hace falta.
        self.create_timer(2.0, self.plan_path)

        self.get_logger().info("RRT Planner con 'Lazy Replanning' Iniciado.")

    def odom_callback(self, msg):
        self.robot_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def lidar_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
            cloud_out = do_transform_cloud(msg, trans)
            
            points = []
            for p in point_cloud2.read_points(cloud_out, field_names=("x", "y"), skip_nans=True):
                if self.robot_pos:
                    # Solo guardamos obstáculos en un radio relevante (ej. 40m)
                    dist = math.hypot(p[0] - self.robot_pos[0], p[1] - self.robot_pos[1])
                    if dist < 40.0: 
                        points.append([p[0], p[1]])
            
            self.obstacles = points 

        except Exception as e:
            # Es normal que falle al principio hasta que llegue la TF
            pass

    def check_collision(self, p1, p2):
        """ Verifica si el segmento p1-p2 choca con obstáculos """
        if not self.obstacles: 
            return False
            
        # Distancia entre puntos
        dist_seg = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        steps = int(dist_seg / 0.5) + 1 # Chequear cada medio metro
        
        for i in range(steps):
            t = i / steps
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            
            # Optimización: Solo chequear obstáculos cercanos al punto intermedio
            # (Aquí hacemos fuerza bruta por simplicidad, pero con cuidado)
            for obs in self.obstacles:
                # Chequeo rápido de caja (bounding box) para no calcular hipotenusa siempre
                if abs(obs[0] - x) > self.collision_radius or abs(obs[1] - y) > self.collision_radius:
                    continue
                
                if math.hypot(x - obs[0], y - obs[1]) < self.collision_radius:
                    return True 
        return False

    def is_path_safe(self):
        """ ### NUEVO: Revisa si la ruta actual sigue siendo válida """
        if not self.current_path or len(self.current_path) < 2:
            return False
            
        # Revisamos cada segmento de la ruta actual contra los obstáculos nuevos
        for i in range(len(self.current_path) - 1):
            p1 = self.current_path[i]
            p2 = self.current_path[i+1]
            if self.check_collision(p1, p2):
                self.get_logger().warn("¡Obstáculo detectado en la ruta actual! Recalculando...")
                return False # La ruta ya no es segura
        
        return True # La ruta sigue limpia

    def plan_path(self):
        if not self.robot_pos:
            return

        # 1. Verificar si llegamos a la meta
        dist_to_goal = math.hypot(self.goal[0]-self.robot_pos[0], self.goal[1]-self.robot_pos[1])
        if dist_to_goal < 3.0:
            self.get_logger().info("Meta alcanzada. Esperando nueva meta...", throttle_duration_sec=2)
##            self.current_path = [] # Limpiamos ruta para evitar re-publicar meta vieja
            return

        self.get_logger().info("Calculando nueva ruta RRT...")
        
        # --- ALGORITMO RRT ---
        tree = [self.robot_pos]
        parents = {0: None}

        for i in range(self.max_iter):
            # Sampleo sesgado
            if np.random.rand() < 0.15: # 15% chance de ir directo a la meta
                rand_pt = np.array(self.goal)
            else:
                rand_pt = np.array(self.robot_pos) + np.random.uniform(-self.search_radius, self.search_radius, 2)

            # Nearest
            dists = [np.linalg.norm(np.array(node) - rand_pt) for node in tree]
            nearest_idx = np.argmin(dists)
            nearest_node = tree[nearest_idx]

            # Steer
            direction = rand_pt - nearest_node
            length = np.linalg.norm(direction)
            if length == 0: continue
            
            # Normalizar y avanzar step_size
            new_node = nearest_node + (direction / length) * min(self.step_size, length)

            # Check Collision
            if not self.check_collision(nearest_node, new_node):
                tree.append(new_node)
                new_idx = len(tree) - 1
                parents[new_idx] = nearest_idx

                # Check Goal
                if np.linalg.norm(new_node - np.array(self.goal)) < self.step_size:
                    # Reconstruir camino
                    path = [self.goal, new_node]
                    curr = nearest_idx
                    while curr is not None:
                        path.append(tree[curr])
                        curr = parents[curr]
                    path.reverse()
                    
                    # Guardamos y publicamos
                    self.current_path = path 
                    self.publish_path_msg(path)
                    return

    def publish_path_msg(self, points):
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for p in points:
            pose = PoseStamped()
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            msg.poses.append(pose)
        
        self.path_pub.publish(msg)
        self.get_logger().info(f"¡Nueva ruta publicada con {len(points)} nodos!")

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
