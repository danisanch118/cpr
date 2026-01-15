#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

# Tipos de mensajes
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

# --- CONSTANTES DEL MODELO PID ---
KP_V = 226.74
KI_V = 1582.7
KP_W = 1504.0
KI_W = 2974.9
K_COUPLING = 1.0 / 1.027135
SATURATION_LIMIT = 2000.0 
R_EARTH = 6378137.0 

# ==========================================
#      CONFIGURACIÓN DE FILTROS (TAUs)
# ==========================================
# Referencia: Qué tan suave es la aceleración que pides
TAU_REF_V = 2.0  
TAU_REF_W = 1.0  

# Sensores: Qué tanto ruido quitamos (Cuidado con ponerlos muy altos)
TAU_SENSOR_V = 0.5   
TAU_SENSOR_W = 0.1   

class BoatController(Node):
    def __init__(self):
        super().__init__('boat_pid_controller')

        # --- Variables de Estado ---
        # "Current": Valor FILTRADO usado por el PID
        self.current_v = 0.0
        self.current_w = 0.0

        # "Raw": Valor SUCIO directo del sensor (Solo para debug)
        self.raw_v = 0.0
        self.raw_w = 0.0
        
        # --- Variables de Referencia ---
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

        # --- Publishers ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # --- DEBUG PUBLISHERS ---
        # 1. Velocidad Real FILTRADA (La que usa el PID)
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel_act', 10) 
        
        # 2. Velocidad Real CRUDA/RAW (La que tiene ruido) ¡NUEVO!
        self.pub_debug_sensors_raw = self.create_publisher(TwistStamped, '/wamv/debug/sensors_raw', 10)

        # 3. Referencias (Lo que pides vs Lo que se aplica)
        self.pub_debug_ref_raw = self.create_publisher(TwistStamped, '/wamv/debug/ref_raw', 10)
        self.pub_debug_ref_filtered = self.create_publisher(TwistStamped, '/wamv/debug/ref_filtered', 10)

        # --- Subscribers ---
        self.create_subscription(TwistStamped, '/wamv/cmd_vel', self.ref_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)

        # --- Filtros ---
        self.dt = 0.02
        self.alpha_ref_v = self.dt / (TAU_REF_V + self.dt)
        self.alpha_ref_w = self.dt / (TAU_REF_W + self.dt)
        self.alpha_sensor_v = self.dt / (TAU_SENSOR_V + self.dt)
        self.alpha_sensor_w = self.dt / (TAU_SENSOR_W + self.dt)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Controlador Iniciado. Publicando raw vs filtered.")

    def ref_callback(self, msg):
        self.target_v = msg.twist.linear.x
        self.target_w = msg.twist.angular.z

    def imu_callback(self, msg):
        # 1. Guardamos el dato RAW (Sucio)
        self.raw_w = msg.angular_velocity.z
        
        # 2. Filtramos
        self.current_w = (self.alpha_sensor_w * self.raw_w) + \
                         ((1.0 - self.alpha_sensor_w) * self.current_w)

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
            
            # 1. Guardamos dato RAW (Sucio)
            self.raw_v = dist / dt_gps 
            
            # 2. Filtramos
            self.current_v = (self.alpha_sensor_v * self.raw_v) + \
                             ((1.0 - self.alpha_sensor_v) * self.current_v)
            
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
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'wamv/wamv/base_link'
        cmd.twist.linear.x = float(lin)
        cmd.twist.angular.z = float(ang)
        publisher.publish(cmd)

    def control_loop(self):
        # 1. PREFILTRADO DE REFERENCIA
        self.ref_v_smooth = (self.alpha_ref_v * self.target_v) + ((1.0 - self.alpha_ref_v) * self.ref_v_smooth)
        self.ref_w_smooth = (self.alpha_ref_w * self.target_w) + ((1.0 - self.alpha_ref_w) * self.ref_w_smooth)

        # 2. PUBLICAR DEBUG DE REFERENCIAS
        self.publish_twist(self.pub_debug_ref_raw, self.target_v, self.target_w)
        self.publish_twist(self.pub_debug_ref_filtered, self.ref_v_smooth, self.ref_w_smooth)
        
        # 3. PUBLICAR DEBUG DE SENSORES (Raw vs Filtered)
        self.publish_twist(self.pub_debug_sensors_raw, self.raw_v, self.raw_w) # <-- EL RUIDO
        self.publish_twist(self.cmd_pub, self.current_v, self.current_w)       # <-- LO LIMPIO

        # 4. PID
        error_v = self.ref_v_smooth - self.current_v
        error_w = self.ref_w_smooth - self.current_w

        self.integral_error_v += error_v * self.dt
        self.integral_error_w += error_w * self.dt
        
        E = (KP_V * error_v) + (KI_V * self.integral_error_v)
        M = (KP_W * error_w) + (KI_W * self.integral_error_w)

        M_coupled = M * K_COUPLING
        input_L = 0.5 * (E - M_coupled)
        input_R = 0.5 * (E + M_coupled)

        cmd_L = max(min(input_L, SATURATION_LIMIT), -SATURATION_LIMIT)
        cmd_R = max(min(input_R, SATURATION_LIMIT), -SATURATION_LIMIT)

        msg_L = Float64()
        msg_R = Float64()
        msg_L.data = float(cmd_L)
        msg_R.data = float(cmd_R)

        self.pub_left.publish(msg_L)
        self.pub_right.publish(msg_R)

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
