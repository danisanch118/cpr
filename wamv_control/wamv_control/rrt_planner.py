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
        self.goal = [30.0, -5.0]  
        self.step_size = 3.0       
        self.max_iter = 3000       
        self.search_radius = 80.0  
        self.collision_radius = 5.0 
        self.robot_pos = None
        self.obstacles = [] 
        self.current_path = [] 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidar/lidar_wamv/points', self.lidar_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/rrt/path', qos_latched)
        self.create_timer(1.0, self.plan_path)
        self.get_logger().info("RRT Planner Iniciado")

    # --- FUNCIÓN DE SUAVIZADO (Mínimo cambio) ---
    def smooth_path(self, path):
        if len(path) < 3: return path
        smoothed = [path[0]]
        curr = 0
        while curr < len(path) - 1:
            for next_p in range(len(path) - 1, curr, -1):
                if not self.check_collision(path[curr], path[next_p]):
                    smoothed.append(path[next_p])
                    curr = next_p
                    break
        return smoothed

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
                    if dist < 50.0: points.append([p[0], p[1]])
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
        
        tree = [self.robot_pos] 
        parents = {0: None}
        for i in range(self.max_iter):
            rand_pt = np.array(self.goal) if np.random.rand() < 0.15 else np.array(self.robot_pos) + np.random.uniform(-self.search_radius, self.search_radius, 2)
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
                    # --- Cambio aplicado aquí ---
                    path = self.smooth_path(path) 
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

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
