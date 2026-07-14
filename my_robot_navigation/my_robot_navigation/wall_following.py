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
        self.distancia_pared    = 0.20   # seguir la pared de referencia a 20 cm
        self.distancia_frontal  = 0.25
        self.umbral_lateral     = 0.40   # lado LIBRE (apertura) si mide mas
        # Centrarse entre paredes SOLO si AMBAS estan a menos de esto.
        # Con una pared izquierda a mas de 0.30, NO centrar: seguir la
        # derecha a 0.20 (una pared lejana no debe arrastrar al robot).
        self.umbral_centrado    = 0.30
        self.vel_lineal         = 0.18
        self.vel_angular        = 0.9
        # Peso adaptativo del control recto (ver bloque en scan_callback)
        self.dist_peligro       = 0.15   # a menos de esto: evasion
        self.umbral_yaw_torcido = 7.0    # grados; mas que esto: enderezar

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
        # Suscriptor a la odometria FILTRADA (EKF: ruedas + IMU).
        # El yaw de /odometry/filtered esta anclado por el giroscopo de la
        # IMU, que no depende del patinaje de las ruedas: mayor precision
        # angular que el /odom crudo del diff-drive.
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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

        # --- Grilla de visitas (Tremaux) + metricas de mision ---
        # Celda = constante de red del laberinto: 0.45 m (paredes del world en multiplos de 0.45).
        # El robot spawnea en un punto de la red y la IMU ancla los ejes de odom paralelos al mundo: 
        # round(pos/0.45) alinea la grilla EXACTA con las celdas reales.
        self.tam_celda = 0.45
        self.visitas = {}            # {(i,j): cantidad de entradas}
        self.celda_actual = None
        self.dist_recorrida = 0.0    # metros acumulados
        self.pos_anterior = None
        self.t_inicio = None         # instante del primer callback

        # --- Meta (coordenadas, a priori permitidas por la catedra) ---
        # Mundo (-0.45, 2.70) -> odom (4.95, -6.75) -> celda (11, -15)
        self.meta_x = 4.95
        self.meta_y = -6.75
        self.radio_meta = 0.15   # < media celda (0.225), > radio del disco (0.099)
        self.mision_cumplida = False

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

        # --- Metricas: tiempo y distancia ---
        if self.t_inicio is None:
            self.t_inicio = self.get_clock().now()
        if self.pos_anterior is not None:
            self.dist_recorrida += math.hypot(
                self.pos_x - self.pos_anterior[0],
                self.pos_y - self.pos_anterior[1])
        self.pos_anterior = (self.pos_x, self.pos_y)

        # --- Grilla: marcar la celda al ENTRAR (no por tick) ---
        celda = (round(self.pos_x / self.tam_celda),
                 round(self.pos_y / self.tam_celda))
        if celda != self.celda_actual:
            self.celda_actual = celda
            self.visitas[celda] = self.visitas.get(celda, 0) + 1
            t = (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9
            self.get_logger().info(
                f'Celda {celda} visitas:{self.visitas[celda]} '
                f'exploradas:{len(self.visitas)} '
                f'dist:{self.dist_recorrida:.2f} m  t:{t:.0f} s')

        # --- Detector de meta ---
        if not self.mision_cumplida and self.t_inicio is not None:
            if math.hypot(self.pos_x - self.meta_x,
                          self.pos_y - self.meta_y) < self.radio_meta:
                self.mision_cumplida = True
                t = (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9
                self.get_logger().info(
                    f'*** META ALCANZADA ***  distancia: {self.dist_recorrida:.2f} m  '
                    f'tiempo: {t:.1f} s ({t/60.0:.1f} min)  '
                    f'celdas exploradas: {len(self.visitas)}')

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

    def rumbo_grilla(self):
        """Rumbo cardinal actual como paso de grilla (di, dj).
        El yaw (odom) se cuantiza al multiplo de 90 grados mas cercano."""
        k = round(self.yaw_actual / (math.pi / 2.0)) % 4
        return [(1, 0), (0, 1), (-1, 0), (0, -1)][k]

    def celda_valida(self, celda):
        """True si la celda esta DENTRO del laberinto (9x9 m => en la
        grilla de odom: i 0..20, j -19..1). El exterior se EXCLUYE de
        las opciones: frontera virtual solo para la decision; el
        wall following y el LIDAR no cambian."""
        i, j = celda
        return 0 <= i <= 20 and -19 <= j <= 1

    def visitas_vecina(self, lado):
        """Visitas de la celda vecina hacia 'RECTO', 'DER' o 'IZQ',
        o None si la vecina queda fuera del laberinto."""
        if self.celda_actual is None:
            return 0
        di, dj = self.rumbo_grilla()
        if lado == 'DER':
            di, dj = dj, -di      # rotar rumbo -90
        elif lado == 'IZQ':
            di, dj = -dj, di      # rotar rumbo +90
        vecina = (self.celda_actual[0] + di, self.celda_actual[1] + dj)
        if not self.celda_valida(vecina):
            return None
        return self.visitas.get(vecina, 0)

    def arbitro(self, der_libre, frente_libre, izq_libre):
        """Tremaux ACOTADO: entre las direcciones libres (LIDAR) cuya
        vecina este dentro del laberinto, ir a la menos visitada.
        Empate -> mano derecha. None => sin opciones (giro en U)."""
        opciones = []
        for libre, bloqueo, prioridad, nombre in (
                (der_libre,    self.ultimo_giro == 'DER', 0, 'DER'),
                (frente_libre, False,                     1, 'RECTO'),
                (izq_libre,    self.ultimo_giro == 'IZQ', 2, 'IZQ')):
            if libre and not bloqueo:
                v = self.visitas_vecina(nombre)
                if v is not None:
                    opciones.append((v, prioridad, nombre))
        if not opciones:
            return None
        visitas_min, _, eleccion = min(opciones)
        prioritaria = min(opciones, key=lambda o: o[1])
        if eleccion != prioritaria[2]:
            self.get_logger().info(
                f'Arbitro: {prioritaria[2]} tiene {prioritaria[0]} visitas; '
                f'voy {eleccion} ({visitas_min})')
        return eleccion

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

        if self.mision_cumplida:
            self.cmd_pub.publish(Twist())   # mision terminada: quieto
            return

        ranges = [r if math.isfinite(r) else 12.0 for r in msg.ranges]

        adelante  = min(ranges[175:180] + ranges[0:5])
        derecha   = min(ranges[125:145])
        izquierda = min(ranges[35:55])

        der_libre = derecha   > self.umbral_lateral
        izq_libre = izquierda > self.umbral_lateral
        frente_libre = adelante > self.distancia_frontal

        #self.get_logger().info(f'Estado: {self.estado} ultimo_giro: {self.ultimo_giro} der:{derecha:.2f}')

        cmd = Twist()

        if self.estado != 'AVANZAR':
            self.ejecutar_maniobra(cmd, adelante, frente_libre, ranges, derecha, izquierda)
            self.cmd_pub.publish(cmd)
            return

        eleccion = self.arbitro(der_libre, frente_libre, izq_libre)

        if eleccion == 'DER':
            self.estado = 'AVANCE_PREVIO_DER'
            self.contador_maniobra = 0
            self.ultimo_giro = 'DER'
            self.pasos_rectos = 0
           # self.get_logger().info('Decision: doblar DERECHA (avanzo antes)', throttle_duration_sec=1.0)

        elif eleccion == 'RECTO':
            self.pasos_rectos += 1
            if self.pasos_rectos >= 15:
                self.ultimo_giro = None

            # --- Error LIDAR (seguimiento lateral) ---
            # Centrarse SOLO en pasillo angosto (ambas paredes < 0.30).
            # Si la izquierda esta lejos (>0.30), la referencia es la pared
            # derecha a 0.20: el carril no se corre por una pared lejana.
            der_hay = derecha   < self.umbral_lateral
            izq_hay = izquierda < self.umbral_lateral
            if derecha < self.umbral_centrado and izquierda < self.umbral_centrado:
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

            # --- Peso adaptativo segun cercania a pared Y error de yaw ---
            # Modelo linealizado del seguimiento (d = error de distancia,
            # theta = error de yaw):  d'' + k_th*d' + v*k_d*d = 0
            #   k_d  = peso_lidar*kp_lidar ; k_th = peso_yaw*kp_yaw
            #   zeta = k_th / (2*sqrt(v*k_d))   con v = 0.18
            # regimen (0.4/0.6) -> zeta=0.99 critico: converge sin serpentear
            # torcido (0.5/0.5) -> zeta=1.36 sobreamortiguado: endereza
            # evasion (0.1/0.9) -> zeta=0.20 agresivo: solo pegado (<0.15)
            distancia_minima = min(derecha, izquierda)
            error_yaw_grados = abs(math.degrees(self.error_a_cardinal()))
            if distancia_minima < self.dist_peligro and \
                    error_yaw_grados <= self.umbral_yaw_torcido:
                # Pegado con yaw casi bien: despegarse rapido (evasion)
                peso_yaw, peso_lidar = 0.1, 0.9
            elif error_yaw_grados > self.umbral_yaw_torcido:
                # Torcido: enderezar antes de que el error crezca
                peso_yaw, peso_lidar = 0.5, 0.5
            else:
                # Regimen de seguimiento/centrado: amortiguacion critica
                peso_yaw, peso_lidar = 0.4, 0.6

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

        elif eleccion == 'IZQ':
            # Igual que la derecha: primero avanza/se endereza, despues pivota
            self.estado = 'AVANCE_PREVIO_IZQ'
            self.contador_maniobra = 0
            self.ultimo_giro = 'IZQ'
            self.pasos_rectos = 0
            self.get_logger().info('Decision: doblar IZQUIERDA (avanzo antes)', throttle_duration_sec=1.0)

        else:
            # Las tres salidas bloqueadas => callejon confirmado: giro en U directo.
            self.estado = 'GIRO_U'
            self.contador_maniobra = 0
            self.lado_verificacion = 'IZQ' if izquierda > derecha else 'DER'
            self.get_logger().info(
                f'Decision: giro en U — adel:{adelante:.2f} der:{derecha:.2f} izq:{izquierda:.2f}',
                throttle_duration_sec=1.0)
        self.cmd_pub.publish(cmd)

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
            # Avance minimo obligatorio antes de doblar en un cruce abierto.
            # Sin esto, "adelante > umbral_cruce" disparaba el giro con dist=0.00
            # y el robot doblaba pegado a la pared del cruce (chocaba).
            avance_minimo = 0.10
            corte_cruce = adelante > self.umbral_cruce and dist >= avance_minimo
            # Doblar si: pared frontal cerca (esquina), cruce abierto pero ya
            # avanzo el minimo, o llego al tope de avance (seguridad).
            if (adelante < self.umbral_pre_giro
                    or corte_cruce
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
            hay_dos_paredes = (derecha < self.umbral_centrado
                               and izquierda < self.umbral_centrado)
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
                self.estado = 'POST_GIRO'
                self.contador_maniobra = 0

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()