import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select
from time import sleep

# --- CONFIGURACIÓN DEL WAM-V ---
THRUST_INCREMENT = 350.0  # Cuánto aumenta el empuje con cada pulsación
ANGULAR_INCREMENT = 150.0 # Cuánto aumenta la fuerza de giro
MAX_THRUST = 4000.0       # Límite máximo de empuje (ajusta según tu simulación)
LINEAR_KEY = 'i'         # Tecla para avanzar
ANGULAR_LEFT_KEY = 'j'   # Tecla para girar izquierda
ANGULAR_RIGHT_KEY = 'l'  # Tecla para girar derecha
STOP_KEY = 'k'           # Tecla para detener
# ------------------------------

class WAMVKeydrive(Node):
    def __init__(self):
        super().__init__('wamv_keydrive_translator')
        
        # Publicadores a los tópicos de empuje (Float64)
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # Variables de control
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        self.MAX_THRUST = MAX_THRUST
        
        # Timer para publicación constante (20 Hz)
        self.timer = self.create_timer(0.02, self.publish_thrust_commands)
        
        self.get_logger().info('Nodo WAMV Keydrive iniciado.')
        self.print_instructions()

    def print_instructions(self):
        print("\n==============================================")
        print("  Control de WAM-V por Teclado (Jazzy)")
        print("==============================================")
        print(f"  [{LINEAR_KEY}]  : Aumentar Empuje Lineal (Adelante)")
        print(f"[{ANGULAR_LEFT_KEY}] : Aumentar Empuje Angular (Girar Izquierda)")
        print(f"[{ANGULAR_RIGHT_KEY}] : Aumentar Empuje Angular (Girar Derecha)")
        print(f"  [{STOP_KEY}]  : Detener/Resetear (Empuje a 0)")
        print("==============================================")
        print("Presione una tecla para comenzar el control...")

    def publish_thrust_commands(self):
        """Calcula el empuje diferencial y lo publica."""
        
        # La lógica diferencial (Barco tipo Skid-steer)
        # linear_speed controla avance/retroceso
        # angular_speed controla el giro (positivo = izquierda, negativo = derecha)
        
        left_thrust = self.linear_speed - self.angular_speed
        right_thrust = self.linear_speed + self.angular_speed
        
        # Aplicar límites máximos
        left_thrust = max(min(left_thrust, self.MAX_THRUST), -self.MAX_THRUST)
        right_thrust = max(min(right_thrust, self.MAX_THRUST), -self.MAX_THRUST)
        
        # Crear y publicar mensajes
        msg_l = Float64(data=left_thrust)
        msg_r = Float64(data=right_thrust)
        
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def key_listener(self, key):
        """Ajusta las variables de control basadas en la tecla pulsada."""
        
        if key == LINEAR_KEY:
            self.linear_speed += THRUST_INCREMENT
        elif key == ANGULAR_LEFT_KEY:
            self.angular_speed += ANGULAR_INCREMENT
        elif key == ANGULAR_RIGHT_KEY:
            self.angular_speed -= ANGULAR_INCREMENT
        elif key == STOP_KEY:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
        
        # Aplicar límite al empuje lineal para evitar que crezca indefinidamente
        self.linear_speed = max(min(self.linear_speed, self.MAX_THRUST), -self.MAX_THRUST)
        self.angular_speed = max(min(self.angular_speed, self.MAX_THRUST), -self.MAX_THRUST)
        
        # Mostrar el estado actual
        self.get_logger().info(
            f'Lin: {self.linear_speed:.2f} | Ang: {self.angular_speed:.2f}'
            f' (L_Thrust: {self.linear_speed - self.angular_speed:.2f}, R_Thrust: {self.linear_speed + self.angular_speed:.2f})', 
            throttle_duration_sec=0.5
        )

def get_key(settings):
    """Función que maneja la lectura de una sola tecla."""
    try:
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            # Revertir a la configuración original para que la consola funcione normalmente
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key
    except Exception as e:
        print(f"Error leyendo la tecla: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return None

# ==============================================================================
# IMPORTACIONES (Asegúrate de que estas sean las únicas que uses para el teclado)
# ==============================================================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select 
# ==============================================================================


# ... (Todo el código de la CLASE WAMVKeydrive va aquí, sin cambios) ...
# ... (Los métodos __init__, print_instructions, publish_thrust_commands, key_listener) ...


# ==============================================================================
# FUNCIONES DE LECTURA Y BUCLE PRINCIPAL (REEMPLAZAR LA VERSIÓN ANTERIOR)
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    
    # 1. Guardar y configurar la terminal (NECESARIO)
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    
    node = WAMVKeydrive()
    node.get_logger().info('Listo para recibir comandos de teclado. CTRL+C para salir.')

    try:
        # El bucle principal de lectura:
        while rclpy.ok():
            
            # Verificamos si hay una tecla disponible para leer
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                
                # Procesa la tecla
                node.key_listener(key)
                
                if key == '\x03':  # CTRL+C para salir
                    break
            
            # Gira el nodo ROS para procesar el timer y la publicación
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        node.get_logger().error(f"Fallo en el bucle principal: {e}")
        
    finally:
        # 2. Restaurar la terminal al estado original
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # Detener el barco
        node.linear_speed = 0.0
        node.angular_speed = 0.0
        node.publish_thrust_commands()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
