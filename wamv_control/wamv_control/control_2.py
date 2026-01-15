#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

# Tipos de mensajes
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

# --- CONSTANTES DEL MODELO (SIMULINK) ---
KP_V = 226.74
KI_V = 1582.7
KP_W = 1504.0
KI_W = 2974.9
K_COUPLING = 1.0 / 1.027135
SATURATION_LIMIT = 2000.0 
R_EARTH = 6378137.0 

# --- CONSTANTES DEL PREFILTRO (SUAVIZADO) ---
TAU_V = 2.0  # Segundos para alcanzar 63% velocidad
TAU_W = 1.0  # Segundos para alcanzar 63% giro

class BoatController(Node):
    def __init__(self):
        super().__init__('boat_pid_controller')

        # --- Variables de Estado ---
        self.current_v = 0.0
        self.current_w = 0.0
        
        # --- Variables del Prefiltro ---
        self.target_v = 0.0     # Input RAW (Escalón)
        self.target_w = 0.0
        
        self.ref_v_smooth = 0.0 # Input FILTERED (Curva)
        self.ref_w_smooth = 0.0

        # Integradores
        self.integral_error_v = 0.0
        self.integral_error_w = 0.0

        # GPS Aux
        self.last_gps_time = None
        self.last_x = None
        self.last_y = None
        self.origin_lat = None
        self.origin_lon = None
        self.first_fix = True

        # --- Publishers (Motores) ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # --- Publishers (Debug / Visualización) ---
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel_act', 10) # Velocidad Real
        
        # ¡NUEVOS TOPICS! Para comparar en gráficas
        self.pub_debug_raw = self.create_publisher(TwistStamped, '/wamv/debug/ref_raw', 10)
        self.pub_debug_filtered = self.create_publisher(TwistStamped, '/wamv/debug/ref_filtered', 10)

        # --- Subscribers ---
        self.create_subscription(TwistStamped, '/wamv/cmd_vel', self.ref_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)

        # --- Configuración Bucle ---
        self.dt = 0.02
        self.alpha_ref_v = self.dt / (TAU_V + self.dt)
        self.alpha_ref_w = self.dt / (TAU_W + self.dt)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Controlador Listo. Debug topics activos.")

    def ref_callback(self, msg):
        self.target_v = msg.twist.linear.x
        self.target_w = msg.twist.angular.z

    def imu_callback(self, msg):
        self.current_w = msg.angular_velocity.z

    def gps_callback(self, msg):
        current_time = time.time()
        if self.first_fix:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            x, y = self.latlon_to_meters(msg.latitude, msg.longitude)
            self.last_x = x
            self.last_y = y
            self.last_gps_time = current_time
            self.first_fix = False
            return

        x, y = self.latlon_to_meters(msg.latitude, msg.longitude)
        dt_gps = current_time - self.last_gps_time
        
        if dt_gps > 0.001:
            dx = x - self.last_x
            dy = y - self.last_y
            dist = math.hypot(dx, dy)
            self.current_v = dist / dt_gps 
            self.last_x = x
            self.last_y = y
            self.last_gps_time = current_time

    def latlon_to_meters(self, lat, lon):
        if self.origin_lat is None: return 0.0, 0.0
        y = R_EARTH * math.radians(lat - self.origin_lat)
        cos_lat = math.cos(math.radians(self.origin_lat))
        x = R_EARTH * math.radians(lon - self.origin_lon) * cos_lat
        return x, y
        
    def publish_twist(self, publisher, lin, ang):
        """Helper para publicar TwistStamped rápido"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'wamv/wamv/base_link'
        cmd.twist.linear.x = float(lin)
        cmd.twist.angular.z = float(ang)
        publisher.publish(cmd)

    def control_loop(self):
        # 1. PREFILTRADO
        self.ref_v_smooth = (self.alpha_ref_v * self.target_v) + ((1.0 - self.alpha_ref_v) * self.ref_v_smooth)
        self.ref_w_smooth = (self.alpha_ref_w * self.target_w) + ((1.0 - self.alpha_ref_w) * self.ref_w_smooth)

        # 2. PUBLICAR DEBUG (Antes del PID)
        # Publicamos la referencia "cruda" y la "suavizada"
        self.publish_twist(self.pub_debug_raw, self.target_v, self.target_w)
        self.publish_twist(self.pub_debug_filtered, self.ref_v_smooth, self.ref_w_smooth)

        # 3. ERRORES (Usando Ref Suave)
        error_v = self.ref_v_smooth - self.current_v
        error_w = self.ref_w_smooth - self.current_w

        # 4. PID
        self.integral_error_v += error_v * self.dt
        self.integral_error_w += error_w * self.dt
        
        E = (KP_V * error_v) + (KI_V * self.integral_error_v)
        M = (KP_W * error_w) + (KI_W * self.integral_error_w)

        # 5. MEZCLA
        M_coupled = M * K_COUPLING
        input_L = 0.5 * (E - M_coupled)
        input_R = 0.5 * (E + M_coupled)

        # 6. SATURACION Y SALIDA
        cmd_L = max(min(input_L, SATURATION_LIMIT), -SATURATION_LIMIT)
        cmd_R = max(min(input_R, SATURATION_LIMIT), -SATURATION_LIMIT)

        msg_L = Float64()
        msg_R = Float64()
        msg_L.data = float(cmd_L)
        msg_R.data = float(cmd_R)

        self.pub_left.publish(msg_L)
        self.pub_right.publish(msg_R)
        
        # Publicar velocidad real
        self.publish_twist(self.cmd_pub, self.current_v, self.current_w)

def main(args=None):
    rclpy.init(args=args)
    node = BoatController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Float64()
        stop_msg.data = 0.0
        node.pub_left.publish(stop_msg)
        node.pub_right.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
