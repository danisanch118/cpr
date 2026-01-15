import rclpy
from rclpy.node import Node
import math

# Tipos de mensajes estándar
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

# --- CONSTANTES PID ---
KP_V = 200.0   
KI_V = 160.0   # Ojo: Si sigue oscilando lento, baja esto a 50.0
KP_W = 200.0
KI_W = 160.0

K_COUPLING = 1.0 / 1.027135
SATURATION_LIMIT = 2000.0 
R_EARTH = 6378137.0 
INTEGRAL_LIMIT_V = 1000.0 
INTEGRAL_LIMIT_W = 1000.0

# --- NUEVO: CONSTANTE DE RAMPA (SLEW RATE) ---
# Unidades de fuerza por segundo.
# Ejemplo: Si es 500.0, tardará 4 segundos en ir de 0 a 2000 (Full Power).
# Ajusta esto: 
#   - 500.0 : Muy suave (Barco pesado)
#   - 2000.0 : Respuesta media
#   - 10000.0 : Casi instantáneo (sin limite práctico)
MAX_SLEW_RATE = 800.0 

class BoatController(Node):
    def __init__(self):
        super().__init__('boat_pid_controller')

        # --- Estados ---
        self.current_v = 0.0
        self.current_w = 0.0
        self.current_yaw = 0.0
        
        # --- Referencias ---
        self.ref_v = 0.0
        self.ref_w = 0.0

        # --- Integradores ---
        self.integral_error_v = 0.0
        self.integral_error_w = 0.0

        # --- Auxiliares GPS ---
        self.last_gps_time = None
        self.last_x = None
        self.last_y = None
        self.origin_lat = None
        self.origin_lon = None
        self.first_fix = True

        # --- Filtro EMA ---
        self.alpha_v = 0.2 
        self.filtered_v = 0.0 

        # --- NUEVO: Memoria para el Slew Rate Limiter ---
        self.last_cmd_L = 0.0
        self.last_cmd_R = 0.0

        # --- Publishers ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel_act', 10)

        # --- Subscribers ---
        self.create_subscription(TwistStamped, '/wamv/cmd_vel', self.ref_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)

        self.dt = 0.05 
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Controlador PID con Slew Rate Limiter Iniciado")

    def ref_callback(self, msg):
        self.ref_v = msg.twist.linear.x
        self.ref_w = msg.twist.angular.z

    def imu_callback(self, msg):
        self.current_w = msg.angular_velocity.z
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def gps_callback(self, msg):
        current_time_nanos = self.get_clock().now().nanoseconds
        current_time = current_time_nanos / 1e9 

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
            raw_speed = dist / dt_gps

            move_angle = math.atan2(dy, dx)
            delta_angle = move_angle - self.current_yaw
            while delta_angle > math.pi: delta_angle -= 2*math.pi
            while delta_angle < -math.pi: delta_angle += 2*math.pi

            if abs(delta_angle) > (math.pi / 2):
                raw_v_signed = -raw_speed
            else:
                raw_v_signed = raw_speed

            self.filtered_v = (self.alpha_v * raw_v_signed) + ((1.0 - self.alpha_v) * self.filtered_v)
            self.current_v = self.filtered_v
            self.last_x = x
            self.last_y = y
            self.last_gps_time = current_time

    def latlon_to_meters(self, lat, lon):
        if self.origin_lat is None: return 0.0, 0.0
        y = R_EARTH * math.radians(lat - self.origin_lat)
        cos_lat = math.cos(math.radians(self.origin_lat))
        x = R_EARTH * math.radians(lon - self.origin_lon) * cos_lat
        return x, y
    
    def apply_slew_rate(self, target, current):
        """
        Limita la pendiente de cambio de la señal.
        """
        # Calcular cuánto queremos cambiar
        diff = target - current
        
        # Máximo cambio permitido en este paso de tiempo (dt)
        max_change = MAX_SLEW_RATE * self.dt
        
        # Si el cambio es positivo y mayor al límite, lo limitamos
        if diff > max_change:
            return current + max_change
        # Si el cambio es negativo (frenada) y excede el límite, lo limitamos
        elif diff < -max_change:
            return current - max_change
        # Si está dentro de los límites, dejamos pasar la señal tal cual
        else:
            return target

    def publish_cmd_debug(self, linear, angular):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'wamv/wamv/base_link'
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        error_v = self.ref_v - self.current_v
        error_w = self.ref_w - self.current_w

        self.integral_error_v += error_v * self.dt
        self.integral_error_v = max(min(self.integral_error_v, INTEGRAL_LIMIT_V), -INTEGRAL_LIMIT_V)
        
        self.integral_error_w += error_w * self.dt
        self.integral_error_w = max(min(self.integral_error_w, INTEGRAL_LIMIT_W), -INTEGRAL_LIMIT_W)

        E = (KP_V * error_v) + (KI_V * self.integral_error_v)
        M = (KP_W * error_w) + (KI_W * self.integral_error_w)

        M_coupled = M * K_COUPLING
        input_L = 0.5 * (E - M_coupled)
        input_R = 0.5 * (E + M_coupled)

        # 1. Saturación estática (Hard Limit)
        cmd_L_saturated = max(min(input_L, SATURATION_LIMIT), -SATURATION_LIMIT)
        cmd_R_saturated = max(min(input_R, SATURATION_LIMIT), -SATURATION_LIMIT)
        
        # 2. NUEVO: Slew Rate Limiter (Soft Limit / Rampa)
        # Tomamos el valor saturado y limitamos lo rápido que llegamos a él
        final_L = self.apply_slew_rate(cmd_L_saturated, self.last_cmd_L)
        final_R = self.apply_slew_rate(cmd_R_saturated, self.last_cmd_R)

        # Actualizamos la memoria para el siguiente ciclo
        self.last_cmd_L = final_L
        self.last_cmd_R = final_R

        msg_L = Float64()
        msg_R = Float64()
        msg_L.data = float(final_L)
        msg_R.data = float(final_R)

        self.pub_left.publish(msg_L)
        self.pub_right.publish(msg_R)
        self.publish_cmd_debug(self.current_v, self.current_w)

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
