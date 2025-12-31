import rclpy
from rclpy.node import Node
import math
import time

# Tipos de mensajes
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# --- CONSTANTES DEL MODELO (SIMULINK) ---
# Controlador Velocidad Lineal (C_E)
KP_V = 226.74 #226.74
KI_V = 1582.7 #1582.7

# Controlador Velocidad Angular (C_M)
KP_W = 1504.0
KI_W = 2974.9

# Ganancia de acoplamiento (Triangulo pequeño antes de la mezcla)
# En simulink es 1 / 1.027135
K_COUPLING = 1.0 / 1.027135

# Límites de saturación (Ajustar según tus propulsores, ej: 1.0, 100.0, 1000.0)
SATURATION_LIMIT = 2000.0 



class BoatController(Node):
    def __init__(self):
        super().__init__('boat_pid_controller')

        # --- Variables de Estado ---
        self.current_v = 0.0
        self.current_w = 0.0
        self.v_filtered = 0.0
        self.w_filtered = 0.0
        self.alpha_w = 0.8  
        self.alpha_v = 0.6
        
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
        
        self.create_subscription(Odometry, '/odometry/filtered',self.odom_callback,10)   

        # --- Timer del Bucle de Control ---
        # Ejecutamos el PID a 50Hz (0.02s)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Nodo de Control PID del Barco Iniciado")

    def odom_callback(self,msg):
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z    

    def ref_callback(self, msg):
        """Recibe la referencia deseada (ej: teleop o path planner)"""
        self.ref_v = msg.twist.linear.x
        self.ref_w = msg.twist.angular.z
        
        
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