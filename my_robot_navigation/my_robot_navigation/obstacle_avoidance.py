import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Suscriptor a datos del LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10 
        )

        # Suscriptor a datos del IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Publicador de comandos de movimiento a los motores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer del control loop
        timer_period = 0.02  
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        # Parámetros fisicos y de navegacion
        self.safe_distance = 0.5  # Distancia mínima permitida (en metros)
        self.linear_speed = 0.2   # Velocidad de avance (m/s)
        self.turn_speed = 0.5     # Velocidad de giro (rad/s)

        self.critical_distance = 0.20 # Límite de colisión
        self.reverse_speed = -0.15 # Velocidad de marcha atrás
        
        self.target_angle = math.pi / 2.0 # 90 grados en radianes
        
        # Variables de estado
        self.state = 'AVANZAR'
        self.start_yaw = 0.0

        self.current_yaw = 0.0
        self.latest_scan = None

    def imu_callback(self, msg):
        q = msg.orientation # Cuaternión de la IMU
        
        # Fórmula para convertir a Yaw (rotación Z)
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(t3, t4)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def control_loop(self):
        
        if self.latest_scan is None:
            return

        cmd = Twist()
        msg = self.latest_scan
        
        # Lectura global 360° del LIDAR para maniobras de emergencia
        entorno_completo = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > msg.range_min]
        distancia_minima_global = min(entorno_completo) if entorno_completo else float('inf')

        # ==========================================
        #           AVANZAR EN LÍNEA RECTA
        # ==========================================
        if self.state == 'AVANZAR':
            
            grados_apertura = 30.0 # Cono de visión deseado en grados
            apertura_rad = math.radians(grados_apertura) # Transformado a radianes
            
            
            indices_por_cono = int(apertura_rad / msg.angle_increment) # Indices del arreglo equivalen a esos grados
            # msg.angle_increment es la distancia en radianes entre cada rayo
            
            indice_central = int(abs(msg.angle_min) / msg.angle_increment) # Indice que equivale a 0 grados
            
            # Recortamos el arreglo desde el centro hacia la izquierda y derecha
            inicio = indice_central - indices_por_cono
            fin = indice_central + indices_por_cono
            front_rays = msg.ranges[inicio:fin] # Extraemos solo los rayos frontales
            
            # Filtramos valores inválidos (infinitos, ceros o nulos)
            valid_ranges = [
                r for r in front_rays 
                if not math.isinf(r) and not math.isnan(r) and r > msg.range_min and r < msg.range_max
            ]
            
            if valid_ranges: 
                min_distance = min(valid_ranges) # Distancia al bstáculo más cercano
                # self.get_logger().info(f'Distancia al frente: {min_distance:.2f} m')
                
                if min_distance < self.safe_distance:
                    self.start_yaw = self.current_yaw
                    self.state = 'GIRANDO_90'
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.turn_speed
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0
            else:
                
                # Avanzar
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
        
        # ==========================================
        #            GIRANDO 90 GRADOS
        # ==========================================
        elif self.state == 'GIRANDO_90':
            # Si se encuentra con un obstáculo demasiado cercano durante el giro cambia de estado
            if distancia_minima_global < self.critical_distance:
                self.get_logger().error('¡COLISIÓN INMINENTE DURANTE GIRO! Abortando a modo ESCAPE.')
                self.state = 'ESCAPAR'
    
                # Frenar
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            else:
                # Calcular angulo de giro acumulado con el IMU
                yaw_diff = self.current_yaw - self.start_yaw
                yaw_diff_normalized = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
                giro_actual = abs(yaw_diff_normalized)
                
                error = self.target_angle - giro_actual # cuánto falta para los 90 grados
                
                # Si el error es menor a esto, terminamos el giro
                tolerancia = 0.02 # en radianes
                if error < tolerancia:
                    self.state = 'AVANZAR'
                    
                    # Frenar
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                else:
                    # CONTROL PROPORCIONAL
                    Kp = 0.8  # Constante proporcional
                    
                    # La velocidad se reduce matemáticamente a medida que el error se achica
                    velocidad_dinamica = Kp * error 
                    
                    # Fricción estática: Forzamos una velocidad mínima (0.15 rad/s) para que 
                    # los motores tengan fuerza suficiente para completar el último grado
                    if velocidad_dinamica < 0.15:
                        velocidad_dinamica = 0.15
                        
                    # Girar
                    cmd.linear.x = 0.0
                    cmd.angular.z = velocidad_dinamica

        # ==========================================
        #                  ESCAPAR
        # ==========================================
        elif self.state == 'ESCAPAR':
            # retrocedemos hasta tener al menos 35cm de margen (0.35m).
            if distancia_minima_global < 0.35:

                # Retroceder
                cmd.linear.x = self.reverse_speed
                cmd.angular.z = 0.0
                # self.get_logger().warn('Retrocediendo para liberar espacio...')
            else:
                self.get_logger().info('Espacio liberado. Reevaluando entorno.')
                self.state = 'AVANZAR'

        
        # Publica el comando de velocidad a los motores del robot    
        self.cmd_pub.publish(cmd)
        return


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()