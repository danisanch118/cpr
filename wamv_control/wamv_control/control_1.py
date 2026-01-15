import rclpy
from rclpy.node import Node
import math
import time

# Tipos de mensajes
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

# --- CONSTANTES DEL MODELO (SIMULINK) ---
# Controlador Velocidad Lineal (C_E)
KP_V = 226.74
KI_V = 1582.7

# Controlador Velocidad Angular (C_M)
KP_W = 1504.0
KI_W = 2974.9

# Ganancia de acoplamiento (Triangulo pequeño antes de la mezcla)
# En simulink es 1 / 1.027135
K_COUPLING = 1.0 / 1.027135

# Límites de saturación (Ajustar según tus propulsores, ej: 1.0, 100.0, 1000.0)
SATURATION_LIMIT = 2000.0 

# Constantes Geográficas
R_EARTH = 6378137.0  # Radio de la tierra en metros

class BoatController(Node):
    def __init__(self):
        super().__init__('boat_pid_controller')

        # --- Variables de Estado ---
        self.current_v = 0.0
        self.current_w = 0.0
        
        # Referencias (Setpoints)
        self.ref_v = 0.0
        self.ref_w = 0.0

        # Integradores para los PIs
        self.integral_error_v = 0.0
        self.integral_error_w = 0.0

        # Variables auxiliares para cálculo de velocidad GPS
        self.last_gps_time = None
        self.last_x = None
        self.last_y = None
        self.origin_lat = None
        self.origin_lon = None
        self.first_fix = True

        # --- Publishers (Salida a motores) ---
        # Ajustar nombres de topics según la configuración de tu WAM-V
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/wamv/cmd_vel_act', 10)
        # --- Subscribers ---
        # 1. Referencias de velocidad (cmd_vel)
        self.create_subscription(TwistStamped, '/wamv/cmd_vel', self.ref_callback, 10)
        
        # 2. IMU (Para velocidad angular actual)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        
        # 3. GPS (Para calcular velocidad lineal actual)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)

        # --- Timer del Bucle de Control ---
        # Ejecutamos el PID a 50Hz (0.02s)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Nodo de Control PID del Barco Iniciado")

    def ref_callback(self, msg):
        """Recibe la referencia deseada (ej: teleop o path planner)"""
        self.ref_v = msg.twist.linear.x
        self.ref_w = msg.twist.angular.z

    def imu_callback(self, msg):
        """Lee la velocidad angular actual (Yaw Rate)"""
        self.current_w = msg.angular_velocity.z

    def gps_callback(self, msg):
        """Calcula la velocidad lineal basándose en incrementos de posición"""
        current_time = time.time() # O usar msg.header.stamp si está sincronizado

        # 1. Inicializar origen en el primer mensaje
        if self.first_fix:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            x, y = self.latlon_to_meters(msg.latitude, msg.longitude)
            self.last_x = x
            self.last_y = y
            self.last_gps_time = current_time
            self.first_fix = False
            return

        # 2. Calcular posición actual en metros
        x, y = self.latlon_to_meters(msg.latitude, msg.longitude)

        # 3. Calcular velocidad (v = distancia / tiempo)
        dt_gps = current_time - self.last_gps_time
        
        if dt_gps > 0.001: # Evitar división por cero
            dx = x - self.last_x
            dy = y - self.last_y
            dist = math.hypot(dx, dy)
            
            # Dirección simple: si nos movemos, asumimos forward positivo por ahora.
            # (Para detectar marcha atrás se requeriría comparar el vector movimiento con el heading del compás)
            self.current_v = dist / dt_gps 
            
            # Actualizar valores antiguos
            self.last_x = x
            self.last_y = y
            self.last_gps_time = current_time

    def latlon_to_meters(self, lat, lon):
        """
        Convierte Latitud/Longitud a coordenadas cartesianas (x, y) en metros.
        X (Este) se corresponde a Longitud, Y (Norte) a Latitud.
        """
        if self.origin_lat is None:
            return 0.0, 0.0

        # Distancia en Y (Latitud)
        y = R_EARTH * math.radians(lat - self.origin_lat)
        
        # Factor de corrección para la Longitud (coseno de la latitud)
        cos_lat = math.cos(math.radians(self.origin_lat))
        
        # Distancia en X (Longitud)
        x = R_EARTH * math.radians(lon - self.origin_lon) * cos_lat
        
        return x, y
    def publish_cmd(self, linear, angular):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'wamv/wamv/base_link'
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        """Ejecuta la lógica del diagrama de bloques Simulink"""
        
        # 1. Calcular Errores
        error_v = self.ref_v - self.current_v
        error_w = self.ref_w - self.current_w

        # 2. Controladores PI (Discretizados)
        # Integral = Integral_anterior + error * dt
        self.integral_error_v += error_v * self.dt
        self.integral_error_w += error_w * self.dt
        
        # Anti-windup simple (opcional, resetea si el error cruza cero)
        # if (error_v > 0 and self.last_error_v < 0) or (error_v < 0 and self.last_error_v > 0): ...

        # Salida PI Velocidad (C_E del diagrama)
        # E = Kp*error + Ki*integral
        E = (KP_V * error_v) + (KI_V * self.integral_error_v)

        # Salida PI Angular (C_M del diagrama)
        M = (KP_W * error_w) + (KI_W * self.integral_error_w)

        # 3. Mezcla y Acoplamiento (Lógica central del diagrama)
        
        # Ganancia triangular 1/1.027135 aplicada a M
        M_coupled = M * K_COUPLING

        # Nodos de suma/resta antes de saturación
        # El diagrama muestra división por 2 (bloques triangulares 1/2)
        # Camino superior (Izquierda): Suma +E y Resta -M_coupled
        input_L = 0.5 * (E - M_coupled)
        
        # Camino inferior (Derecha): Suma +E y Suma +M_coupled
        input_R = 0.5 * (E + M_coupled)

        # 4. Saturación
        cmd_L = max(min(input_L, SATURATION_LIMIT), -SATURATION_LIMIT)
        cmd_R = max(min(input_R, SATURATION_LIMIT), -SATURATION_LIMIT)

        # 5. Publicar a los motores
        msg_L = Float64()
        msg_R = Float64()
        msg_L.data = float(cmd_L)
        msg_R.data = float(cmd_R)

        self.pub_left.publish(msg_L)
        self.pub_right.publish(msg_R)
        self.publish_cmd(self.current_v, self.current_w)

def main(args=None):
    rclpy.init(args=args)
    node = BoatController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar motores al salir
        stop_msg = Float64()
        stop_msg.data = 0.0
        node.pub_left.publish(stop_msg)
        node.pub_right.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()