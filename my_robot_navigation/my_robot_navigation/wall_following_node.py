import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WallFollowingNode(Node):

    def __init__(self):
        super().__init__('wall_following_node')

        # Parametros
        self.distancia_pared    = 0.25
        self.distancia_frontal  = 0.25
        self.umbral_lateral     = 0.40
        self.vel_lineal         = 0.18
        self.vel_angular        = 0.9

        # Estado de la maquina de maniobras
        self.estado = 'AVANZAR'
        self.contador_maniobra = 0
        self.lado_verificacion = 'DER'
        self.error_anterior = 0.0
        self.ultimo_giro = None
        self.pasos_rectos = 0
        self.dist_retroceso_u = 0.12   # cuanto retroceder antes de girar en U

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        # Suscriptor a la odometria para conocer la orientacion (yaw)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Yaw actual del robot en radianes (se actualiza en odom_callback)
        self.yaw_actual = 0.0
        # Yaw de referencia: se captura en la primera lectura de odometria
        # Las cardinales se calculan relativas a esta referencia inicial
        self.yaw_referencia = None
        self.yaw_objetivo = 0.0   # yaw que el robot debe alcanzar al girar

        # Avance previo antes de doblar a la derecha (esquina exterior)
        self.umbral_pre_giro = 0.30   # avanzar hasta que el frente quede a esta distancia
        self.umbral_cruce = 0.50   # frente mas abierto que esto => es un cruce: doblar ya, sin avanzar
        self.dist_max_avance = 0.15   # tope de avance (cruces sin pared frontal)

        # Posicion del robot (odometria) para medir distancia recorrida
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.x_inicio = 0.0
        self.y_inicio = 0.0

        self.get_logger().info('Nodo de navegacion iniciado')

    def odom_callback(self, msg):
        # Convertir cuaternion a yaw (rotacion en el plano)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw_actual = math.atan2(siny_cosp, cosy_cosp)

        # Guardar posicion para medir distancia en maniobras
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        # En la primera lectura, capturar la referencia inicial
        if self.yaw_referencia is None:
            self.yaw_referencia = self.yaw_actual
            self.get_logger().info(
                f'Referencia de yaw capturada: {math.degrees(self.yaw_referencia):.1f} grados')

        # Calcular el error respecto a la cardinal mas cercana
        error_card = self.error_a_cardinal()
        self.get_logger().info(
            f'Yaw: {math.degrees(self.yaw_actual):.1f}  '
            f'Error a cardinal: {math.degrees(error_card):.1f} grados',
            throttle_duration_sec=2.0)

    def normalizar_angulo(self, angulo):
        """Normaliza un angulo al rango [-pi, pi] (camino mas corto)."""
        return math.atan2(math.sin(angulo), math.cos(angulo))

    def error_a_cardinal(self):
        """Devuelve el error angular (rad) entre el yaw actual y la
        cardinal mas cercana, relativa a la referencia inicial."""
        # Yaw relativo a la referencia
        rel = self.normalizar_angulo(self.yaw_actual - self.yaw_referencia)
        # Las cardinales relativas son 0, pi/2, pi, -pi/2
        # Redondeamos rel al multiplo de pi/2 mas cercano
        paso = math.pi / 2.0
        cardinal_mas_cercana = round(rel / paso) * paso
        # Error = cuanto falta para llegar a esa cardinal
        return self.normalizar_angulo(cardinal_mas_cercana - rel)

    def scan_callback(self, msg):
        ranges = [r if math.isfinite(r) else 12.0 for r in msg.ranges]

        adelante  = min(ranges[175:180] + ranges[0:5])
        derecha   = min(ranges[125:145])
        izquierda = min(ranges[35:55])

        der_libre = derecha   > self.umbral_lateral
        izq_libre = izquierda > self.umbral_lateral
        frente_libre = adelante > self.distancia_frontal

        cmd = Twist()

        if self.estado != 'AVANZAR':
            self.ejecutar_maniobra(cmd, adelante, frente_libre)
            self.cmd_pub.publish(cmd)
            return

        if der_libre and self.ultimo_giro != 'DER':
            self.estado = 'AVANCE_PREVIO_DER'
            self.contador_maniobra = 0
            self.ultimo_giro = 'DER'
            self.pasos_rectos = 0
            self.get_logger().info('Decision: doblar DERECHA (avanzo antes)', throttle_duration_sec=1.0)

        elif frente_libre:
            self.pasos_rectos += 1
            if self.pasos_rectos >= 2:
                self.ultimo_giro = None

            # --- Error LIDAR (centrado lateral) ---
            der_hay = derecha   < self.umbral_lateral
            izq_hay = izquierda < self.umbral_lateral
            if der_hay and izq_hay:
                error_lidar = derecha - izquierda      # centrado entre paredes
                modo = 'centrado'
            elif der_hay:
                error_lidar = derecha - self.distancia_pared
                modo = 'sigue-der'
            elif izq_hay:
                error_lidar = self.distancia_pared - izquierda
                modo = 'sigue-izq'
            else:
                error_lidar = 0.0
                modo = 'libre'

            # --- Error YAW (alineacion a cardinal) ---
            # error_a_cardinal devuelve cuanto girar para alinearse.
            # Lo negamos para que el signo coincida con la convencion
            # del error_lidar (correccion = -k*error)
            error_yaw = -self.error_a_cardinal()

            # --- Peso adaptativo segun cercania a pared ---
            distancia_minima = min(derecha, izquierda)
            if distancia_minima > 0.15:
                # Zona segura: priorizar yaw (ir derecho)
                peso_yaw, peso_lidar = 0.9, 0.1
            else:
                # Cerca de pared: priorizar lidar (alejarse)
                peso_yaw, peso_lidar = 0.3, 0.7

            # --- Combinar ambos errores ---
            kp_lidar = 2.0
            kp_yaw   = 1.5
            correccion_lidar = -kp_lidar * error_lidar
            correccion_yaw   = -kp_yaw * error_yaw
            salida = peso_lidar * correccion_lidar + peso_yaw * correccion_yaw

            correccion = max(min(salida, self.vel_angular * 0.5),
                             -self.vel_angular * 0.5)
            cmd.linear.x  = self.vel_lineal
            cmd.angular.z = correccion
            self.get_logger().info(
                f'Recto [{modo}] der:{derecha:.2f} izq:{izquierda:.2f} '
                f'e_lidar:{error_lidar:.2f} e_yaw:{math.degrees(error_yaw):.1f}',
                throttle_duration_sec=1.0)

        elif izq_libre and self.ultimo_giro != 'IZQ':
            self.estado = 'GIRO_IZQ'
            self.contador_maniobra = 0
            self.ultimo_giro = 'IZQ'
            self.pasos_rectos = 0
            self.get_logger().info('Decision: doblar IZQUIERDA', throttle_duration_sec=1.0)

        else:
            # Las tres salidas bloqueadas => callejon confirmado: giro en U directo.
            self.estado = 'GIRO_U'
            self.contador_maniobra = 0
            self.lado_verificacion = 'IZQ' if izquierda > derecha else 'DER'
            self.get_logger().info(
                f'Decision: giro en U — adel:{adelante:.2f} der:{derecha:.2f} izq:{izquierda:.2f}',
                throttle_duration_sec=1.0)
        self.cmd_pub.publish(cmd)

    def ejecutar_maniobra(self, cmd, adelante, frente_libre):
        """Ejecuta la maniobra en curso paso a paso."""

        if self.estado == 'AVANCE_PREVIO_DER':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                self.x_inicio = self.pos_x
                self.y_inicio = self.pos_y
            # Avanzar recto, enderezando hacia la cardinal mientras avanza
            err_card = self.error_a_cardinal()
            cmd.linear.x  = self.vel_lineal
            cmd.angular.z = max(min(1.0 * err_card,
                                    self.vel_angular * 0.4),
                                -self.vel_angular * 0.4)
            # Distancia recorrida desde el inicio del avance
            dist = math.hypot(self.pos_x - self.x_inicio,
                              self.pos_y - self.y_inicio)
            self.get_logger().info(
                f'Avance previo: dist:{dist:.2f} adelante:{adelante:.2f}',
                throttle_duration_sec=0.5)
            # Doblar cuando el frente esta cerca (esquina) o si llego al tope
            if (adelante < self.umbral_pre_giro
                    or adelante > self.umbral_cruce
                    or dist >= self.dist_max_avance):
                self.estado = 'GIRO_DER'
                self.contador_maniobra = 0

        elif self.estado == 'GIRO_DER':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                # Fijar el yaw objetivo: 90 grados a la derecha
                self.yaw_objetivo = self.normalizar_angulo(
                    self.yaw_actual - math.pi / 2.0)
            
            # Error: cuanto falta para llegar al objetivo (rad)
            err = self.normalizar_angulo(self.yaw_objetivo - self.yaw_actual)

            # Velocidad PROPORCIONAL al error: rapido lejos, lento al acercarse.
            # Asi no se pasa de largo aunque el loop evalue en saltos grandes.
            kp_giro = 1.5
            vel = max(min(kp_giro * err, self.vel_angular), -self.vel_angular)
            cmd.linear.x  = 0.0
            cmd.angular.z = vel

            # Llegamos cuando el error es chico
            if abs(err) < math.radians(2):
                self.estado = 'POST_GIRO'
                self.contador_maniobra = 0
            # Tope de seguridad (no deberia activarse nunca)
            if self.contador_maniobra >= 80:
                self.estado = 'POST_GIRO'
                self.contador_maniobra = 0

        elif self.estado == 'POST_GIRO':
            self.contador_maniobra += 1
            cmd.linear.x  = self.vel_lineal * 0.5
            cmd.angular.z = 0.0
            if self.contador_maniobra >= 3:
                self.estado = 'AVANZAR'

        elif self.estado == 'GIRO_IZQ':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                self.yaw_objetivo = self.normalizar_angulo(
                    self.yaw_actual + math.pi / 2.0)

            err = self.normalizar_angulo(self.yaw_objetivo - self.yaw_actual)

            kp_giro = 1.5
            vel = max(min(kp_giro * err, self.vel_angular), -self.vel_angular)
            cmd.linear.x  = 0.0
            cmd.angular.z = vel

            if abs(err) < math.radians(2):
                self.estado = 'POST_GIRO'
                self.contador_maniobra = 0
            if self.contador_maniobra >= 80:
                self.estado = 'POST_GIRO'
                self.contador_maniobra = 0

        elif self.estado == 'VERIFICAR_U':
            self.contador_maniobra += 1
            if self.lado_verificacion == 'IZQ':
                cmd.angular.z = self.vel_angular
            else:
                cmd.angular.z = -self.vel_angular
            cmd.linear.x = 0.0
            if self.contador_maniobra >= 2:
                if adelante > self.distancia_frontal:
                    self.estado = 'AVANZAR'
                else:
                    self.estado = 'GIRO_U'
                    self.contador_maniobra = 0

        elif self.estado == 'GIRO_U':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                # Girar hacia la pared CERCANA (lado con menor distancia):
                # la cola barre hacia el lado libre y el cuerpo se aleja de
                # la pared cercana al retroceder, en vez de trabarse contra ella.
                # lado='IZQ' => izq>der => pared cercana = DERECHA => giro CW (-1)
                self.giro_u_dir = -1.0 if self.lado_verificacion == 'IZQ' else 1.0
                err_card = self.error_a_cardinal()
                cardinal_actual = self.normalizar_angulo(self.yaw_actual + err_card)
                self.yaw_objetivo = self.normalizar_angulo(
                    cardinal_actual + math.radians(180))
            err = self.normalizar_angulo(self.yaw_objetivo - self.yaw_actual)
            # Sentido FORZADO hacia la pared cercana: constante mientras esta
            # lejos, proporcional cerca del objetivo para no pasarse de largo.
            if abs(err) < math.radians(45):
                cmd.angular.z = max(min(1.5 * err, self.vel_angular),
                                    -self.vel_angular)
            else:
                cmd.angular.z = self.giro_u_dir * self.vel_angular
            # 1ra mitad del giro: retroceder (despeja la cola del rincon).
            # 2da mitad: avanzar para completar los 180°.
            if abs(err) > math.radians(90):
                cmd.linear.x = -self.vel_lineal * 0.3
            else:
                cmd.linear.x =  self.vel_lineal * 0.3
            if abs(err) < math.radians(5):
                self.estado = 'AVANZAR'

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()