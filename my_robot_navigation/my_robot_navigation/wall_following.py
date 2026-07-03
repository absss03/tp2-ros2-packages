import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class WallFollowingNode(Node):

    def __init__(self):
        super().__init__('wall_following')

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
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation_markers', 10)

        # Yaw actual del robot en radianes (se actualiza en odom_callback)
        self.yaw_actual = 0.0
        # Yaw de referencia: se captura en la primera lectura de odometria
        # Las cardinales se calculan relativas a esta referencia inicial
        self.yaw_referencia = None
        self.yaw_objetivo = 0.0   # yaw que el robot debe alcanzar al girar

        # Avance previo antes de doblar a la derecha (esquina exterior)
        self.umbral_pre_giro = 0.20   # avanzar hasta que el frente quede a esta distancia
        self.umbral_cruce = 0.50   # frente mas abierto que esto => es un cruce: doblar ya, sin avanzar
        self.dist_max_avance = 0.3   # tope de avance (cruces sin pared frontal)

        # Avance previo antes de doblar a la IZQUIERDA (esquina interior)
        # El giro a la izq se decide con pared al frente: el robot se mete
        # hasta la pared frontal y se endereza antes de pivotar.
        self.umbral_frontal_giro = 0.23   # avanzar hasta tener el frente a esta distancia
        self.dist_max_avance_izq = 0.08   # tope de avance del giro izq (8 cm)
        self.tol_alineacion = math.radians(3)  # alinear a +-3 grados antes de pivotar
        self.fase_giro_izq = 'AVANZAR'    # sub-estado: 'AVANZAR' o 'ALINEAR'

        # --- POST_GIRO contra paredes ---
        # Despues de un giro, el robot se endereza usando las PAREDES REALES
        # (no la cardinal, que pudo derivar). Rayos simetricos por lado:
        #   Derecha: perpendicular = indice 135 -> rayos 125 (atras) y 145 (adelante)
        #   Izquierda: perpendicular = indice 45 -> rayos 35 (adelante) y 55 (atras)
        # Paralelo a la pared => los dos rayos del lado miden igual.
        self.kp_paralelo   = 5.0          # autoridad para ponerse paralelo a la pared
        self.kp_centro_pg  = 0.6          # autoridad (suave) para centrarse en el POST_GIRO
        self.tol_paralelo  = 0.015        # |dif de rayos| < esto => paralelo (~5 grados)
        self.tol_centro_pg = 0.05         # |der-izq| < esto => centrado
        self.umbral_ve_pared = 0.60       # un rayo "ve pared" si mide menos que esto
        self.post_giro_tope  = 50         # tope de seguridad (ticks) del POST_GIRO

        # Posicion del robot (odometria) para medir distancia recorrida
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.x_inicio = 0.0
        self.y_inicio = 0.0

        # Datos para markers
        self.trayectoria = []
        self.MAX_TRAYECTORIA = 10000
        self.decisiones = []
        self.id_decision = 0

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

        # Registrar punto de trayectoria para el marker
        p = Point(x=self.pos_x, y=self.pos_y, z=0.05)
        self.trayectoria.append(p)
        if len(self.trayectoria) > self.MAX_TRAYECTORIA:
            self.trayectoria.pop(0)

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
        if self.yaw_referencia is None:
            return 0.0
        # Yaw relativo a la referencia
        rel = self.normalizar_angulo(self.yaw_actual - self.yaw_referencia)
        # Las cardinales relativas son 0, pi/2, pi, -pi/2
        # Redondeamos rel al multiplo de pi/2 mas cercano
        paso = math.pi / 2.0
        cardinal_mas_cercana = round(rel / paso) * paso
        # Error = cuanto falta para llegar a esa cardinal
        return self.normalizar_angulo(cardinal_mas_cercana - rel)

    def cardinal_mas_cercana(self):
        """Devuelve el yaw absoluto (rad) de la cardinal mas cercana.
        Sirve para que los giros partan SIEMPRE de una cardinal exacta,
        sin arrastrar el error con el que el robot llego al giro."""
        return self.normalizar_angulo(self.yaw_actual + self.error_a_cardinal())

    def alinear_con_paredes(self, ranges):
        """Calcula la correccion angular para ponerse PARALELO al pasillo
        usando dos rayos por pared (la pared real, no la cardinal).
        Devuelve (err_paralelo, der_ok, izq_ok):
          - err_paralelo: correccion angular ya con signo (None si no hay
            ninguna pared confiable). >0 gira a izq, <0 gira a der.
          - der_ok/izq_ok: si cada lado tiene lectura confiable de pared.
        Paralelo a un lado => sus dos rayos miden igual."""
        rd_ade = ranges[145]   # adelante-derecha
        rd_atr = ranges[125]   # atras-derecha
        ri_ade = ranges[35]    # adelante-izquierda
        ri_atr = ranges[55]    # atras-izquierda

        der_ok = rd_ade < self.umbral_ve_pared and rd_atr < self.umbral_ve_pared
        izq_ok = ri_ade < self.umbral_ve_pared and ri_atr < self.umbral_ve_pared

        # Correccion con signo verificado: si la nariz rota a la izq (CCW),
        # en la pared DER el rayo de adelante se alarga (rd_ade-rd_atr>0) y hay
        # que rotar a la der (correccion negativa) -> corr = -(rd_ade-rd_atr).
        # En la pared IZQ es al reves -> corr = +(ri_ade-ri_atr).
        corr_der = -(rd_ade - rd_atr)
        corr_izq =  (ri_ade - ri_atr)

        if der_ok and izq_ok:
            return 0.5 * corr_der + 0.5 * corr_izq, der_ok, izq_ok
        elif der_ok:
            return corr_der, der_ok, izq_ok
        elif izq_ok:
            return corr_izq, der_ok, izq_ok
        else:
            return None, der_ok, izq_ok

    def scan_callback(self, msg):
        ranges = [r if math.isfinite(r) else 12.0 for r in msg.ranges]

        adelante  = min(ranges[175:180] + ranges[0:5])
        derecha   = min(ranges[125:145])
        izquierda = min(ranges[35:55])

        der_libre = derecha   > self.umbral_lateral
        izq_libre = izquierda > self.umbral_lateral
        frente_libre = adelante > self.distancia_frontal

        self.get_logger().info(f'Estado: {self.estado} ultimo_giro: {self.ultimo_giro} der:{derecha:.2f}')

        cmd = Twist()

        if self.estado != 'AVANZAR':
            self.ejecutar_maniobra(cmd, adelante, frente_libre, ranges, derecha, izquierda)
            self.cmd_pub.publish(cmd)
            self._publicar_markers(adelante, derecha, izquierda)
            return

        if der_libre and self.ultimo_giro != 'DER':
            self.estado = 'AVANCE_PREVIO_DER'
            self.contador_maniobra = 0
            self.ultimo_giro = 'DER'
            self.pasos_rectos = 0
            self._registrar_decision('DER')
           # self.get_logger().info('Decision: doblar DERECHA (avanzo antes)', throttle_duration_sec=1.0)

        elif frente_libre:
            self.pasos_rectos += 1
            if self.pasos_rectos >= 15:
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
            if distancia_minima > 0.20:
                # Zona segura: priorizar yaw (ir derecho)
                peso_yaw, peso_lidar = 0.9, 0.1
            else:
                # Cerca de pared: priorizar lidar (alejarse)
                peso_yaw, peso_lidar = 0.1, 0.9

            # --- Combinar ambos errores ---
            kp_lidar = 1.5
            kp_yaw   = 2
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
            # Igual que la derecha: primero avanza/se endereza, despues pivota
            self.estado = 'AVANCE_PREVIO_IZQ'
            self.contador_maniobra = 0
            self.ultimo_giro = 'IZQ'
            self.pasos_rectos = 0
            self._registrar_decision('IZQ')
            self.get_logger().info('Decision: doblar IZQUIERDA (avanzo antes)', throttle_duration_sec=1.0)

        else:
            # Las tres salidas bloqueadas => callejon confirmado: giro en U directo.
            self.estado = 'GIRO_U'
            self.contador_maniobra = 0
            self.lado_verificacion = 'IZQ' if izquierda > derecha else 'DER'
            self._registrar_decision('U')
            self.get_logger().info(
                f'Decision: giro en U — adel:{adelante:.2f} der:{derecha:.2f} izq:{izquierda:.2f}',
                throttle_duration_sec=1.0)
        self.cmd_pub.publish(cmd)
        self._publicar_markers(adelante, derecha, izquierda)

    def ejecutar_maniobra(self, cmd, adelante, frente_libre, ranges, derecha, izquierda):
        """Ejecuta la maniobra en curso paso a paso."""

        if self.estado == 'AVANCE_PREVIO_DER':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                self.x_inicio = self.pos_x
                self.y_inicio = self.pos_y
            # Avanzar recto, enderezando hacia la cardinal mientras avanza
            err_card = self.error_a_cardinal()
            cmd.linear.x  = self.vel_lineal * 0.5
            cmd.angular.z = max(min(1.0 * err_card,
                                    self.vel_angular * 0.4),
                                -self.vel_angular * 0.4)
            # Distancia recorrida desde el inicio del avance
            dist = math.hypot(self.pos_x - self.x_inicio,
                              self.pos_y - self.y_inicio)
            self.get_logger().info(
                f'Avance previo DER: dist:{dist:.2f} adelante:{adelante:.2f}',
                throttle_duration_sec=0.5)
            # Doblar cuando el frente esta cerca (esquina) o si llego al tope
            if (adelante < self.umbral_pre_giro
                    or adelante > self.umbral_cruce
                    or dist >= self.dist_max_avance):
                self.estado = 'GIRO_DER'
                self.contador_maniobra = 0

        elif self.estado == 'AVANCE_PREVIO_IZQ':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                self.x_inicio = self.pos_x
                self.y_inicio = self.pos_y
                self.fase_giro_izq = 'AVANZAR'

            err_card = self.error_a_cardinal()

            if self.fase_giro_izq == 'AVANZAR':
                # FASE 1: meterse LENTO hasta la pared frontal, enderezando.
                cmd.linear.x  = self.vel_lineal * 0.4
                cmd.angular.z = max(min(1.0 * err_card,
                                        self.vel_angular * 0.4),
                                    -self.vel_angular * 0.4)
                dist = math.hypot(self.pos_x - self.x_inicio,
                                  self.pos_y - self.y_inicio)
                self.get_logger().info(
                    f'Avance previo IZQ [avanza]: dist:{dist:.2f} adelante:{adelante:.2f}',
                    throttle_duration_sec=0.5)
                # Llego a la pared frontal (0.23) o al tope (8cm): pasar a alinear
                if (adelante < self.umbral_frontal_giro
                        or dist >= self.dist_max_avance_izq):
                    self.fase_giro_izq = 'ALINEAR'

            else:  # FASE 2: plantado (sin avanzar), enderezar a la cardinal
                cmd.linear.x  = 0.0
                cmd.angular.z = max(min(1.5 * err_card, self.vel_angular),
                                    -self.vel_angular)
                self.get_logger().info(
                    f'Avance previo IZQ [alinea]: e_card:{math.degrees(err_card):.1f}',
                    throttle_duration_sec=0.5)
                # Alineado a +-3 grados (o tope de seguridad): ejecutar el giro
                if abs(err_card) < self.tol_alineacion or self.contador_maniobra >= 100:
                    self.estado = 'GIRO_IZQ'
                    self.contador_maniobra = 0

        elif self.estado == 'GIRO_DER':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                # Objetivo = cardinal mas cercana - 90, NO yaw_actual - 90.
                # Asi el giro termina EXACTO sobre una cardinal, sin arrastrar
                # el error con el que el robot llego al giro.
                self.yaw_objetivo = self.normalizar_angulo(
                    self.cardinal_mas_cercana() - math.pi / 2.0)

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
            # Avanza LENTO y se endereza/centra contra las PAREDES REALES.
            # Al quedar paralelo, RESINCRONIZA la cardinal (borra la deriva
            # de odometria para que el recto no lo vuelva a torcer).
            self.contador_maniobra += 1
            err_paralelo, der_ok, izq_ok = self.alinear_con_paredes(ranges)
            # Centrar SOLO si hay pared a los dos lados. Si un lado es apertura
            # (izq o der = lejos), centrar lo tiraria hacia el hueco -> no centrar.
            hay_dos_paredes = (derecha < self.umbral_lateral
                               and izquierda < self.umbral_lateral)
            err_centro = (derecha - izquierda) if hay_dos_paredes else 0.0

            if err_paralelo is None:
                # Ninguna pared confiable para enderezar: no arriesgar, seguir.
                self.estado = 'AVANZAR'
                self.contador_maniobra = 0
            else:
                # Enderezar (prioridad) + centrar (suave)
                giro = (self.kp_paralelo * err_paralelo
                        - self.kp_centro_pg * err_centro)
                cmd.angular.z = max(min(giro, self.vel_angular * 0.4),
                                    -self.vel_angular * 0.4)
                # Avanzar lento; frenar el avance si hay algo cerca al frente
                cmd.linear.x = self.vel_lineal * 0.25
                if adelante < 0.20:
                    cmd.linear.x = 0.0

                self.get_logger().info(
                    f'POST_GIRO [endereza] e_par:{err_paralelo:.3f} e_cen:{err_centro:.2f} '
                    f'der_ok:{der_ok} izq_ok:{izq_ok}',
                    throttle_duration_sec=0.3)

                paralelo_ok = abs(err_paralelo) < self.tol_paralelo
                centrado_ok = abs(err_centro)   < self.tol_centro_pg

                if (paralelo_ok and centrado_ok) or self.contador_maniobra >= self.post_giro_tope:
                    # Si quedo PARALELO a la pared real, resincronizar la cardinal:
                    # esta orientacion pasa a ser exactamente la cardinal mas cercana.
                    if paralelo_ok:
                        self.yaw_referencia = self.normalizar_angulo(
                            self.yaw_referencia - self.error_a_cardinal())
                        self.get_logger().info(
                            'POST_GIRO: cardinal resincronizada con la pared real')
                    self.estado = 'AVANZAR'
                    self.contador_maniobra = 0

        elif self.estado == 'GIRO_IZQ':
            self.contador_maniobra += 1
            if self.contador_maniobra == 1:
                # Objetivo = cardinal mas cercana + 90 (mismo criterio que GIRO_DER)
                self.yaw_objetivo = self.normalizar_angulo(
                    self.cardinal_mas_cercana() + math.pi / 2.0)

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
                self.yaw_objetivo = self.normalizar_angulo(
                    self.cardinal_mas_cercana() + math.radians(180))
            err = self.normalizar_angulo(self.yaw_objetivo - self.yaw_actual)
            # Sentido FORZADO hacia la pared cercana: constante mientras esta
            # lejos, proporcional cerca del objetivo para no pasarse de largo.
            if abs(err) < math.radians(45):
                cmd.angular.z = max(min(1.5 * err, self.vel_angular),
                                    -self.vel_angular)
            else:
                cmd.angular.z = self.giro_u_dir * self.vel_angular
            # 1ra mitad del giro: retroceder (despeja la cola del rincon).
            # 2da mitad: avanzar para completar los 180.
            if abs(err) > math.radians(90):
                cmd.linear.x = -self.vel_lineal * 0.3
            else:
                cmd.linear.x =  self.vel_lineal * 0.3
            if abs(err) < math.radians(5):
                self.estado = 'AVANZAR'

    def _registrar_decision(self, tipo):
        self.decisiones.append({
            'x': self.pos_x, 'y': self.pos_y,
            'tipo': tipo, 'id': self.id_decision
        })
        self.id_decision += 1

    def _color_decision(self, tipo):
        colores = {
            'DER': (0.0, 0.5, 1.0),
            'IZQ': (1.0, 0.5, 0.0),
            'U':   (1.0, 0.0, 0.0),
            'STOP':(1.0, 1.0, 0.0),
        }
        return colores.get(tipo, (1.0, 1.0, 1.0))

    def _publicar_markers(self, adelante, derecha, izquierda):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        if len(self.trayectoria) >= 2:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'trayectoria'
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            m.pose.orientation.w = 1.0
            m.points = list(self.trayectoria)
            array.markers.append(m)

        for dec in self.decisiones:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'decisiones'
            m.id = dec['id']
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = dec['x']
            m.pose.position.y = dec['y']
            m.pose.position.z = 0.10
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            r, g, b = self._color_decision(dec['tipo'])
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            array.markers.append(m)

        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'estado'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = self.pos_x
        m.pose.position.y = self.pos_y
        m.pose.position.z = 0.40
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f'{self.estado}\nF:{adelante:.2f} D:{derecha:.2f} I:{izquierda:.2f}'
        array.markers.append(m)

        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'orientacion'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        cola = Point(x=self.pos_x, y=self.pos_y, z=0.05)
        punta = Point(
            x=self.pos_x + 0.25 * math.cos(self.yaw_actual),
            y=self.pos_y + 0.25 * math.sin(self.yaw_actual),
            z=0.05)
        m.points = [cola, punta]
        m.scale.x, m.scale.y, m.scale.z = 0.04, 0.08, 0.0
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
        array.markers.append(m)

        array.markers.append(self._marker_grilla())
        self.marker_pub.publish(array)

    def _marker_grilla(self, tamaño=5.0, paso=0.5):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'grilla'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.01
        m.color.r = m.color.g = m.color.b = 0.5
        m.color.a = 0.5
        m.pose.orientation.w = 1.0

        pasos = int(tamaño / paso)
        for i in range(-pasos, pasos + 1):
            c = i * paso
            m.points += [Point(x=c, y=-tamaño, z=0.0), Point(x=c, y=tamaño, z=0.0)]
            m.points += [Point(x=-tamaño, y=c, z=0.0), Point(x=tamaño, y=c, z=0.0)]
        return m


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()