import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy

class GridMapperNode(Node):

    def __init__(self):
        super().__init__('grid_mapper_node')

        # --- Parametros ROS2 (ajustables sin modificar el codigo) ---
        # Posicion de spawn del robot en el mundo de Gazebo
        self.declare_parameter('spawn_x',      0.0)
        self.declare_parameter('spawn_y',      0.0)
        # Limites del laberinto en coordenadas del mundo
        self.declare_parameter('world_min_x', -3.825)
        self.declare_parameter('world_max_x',  3.825)
        self.declare_parameter('world_min_y', -3.150)
        self.declare_parameter('world_max_y',  3.150)
        # Resolucion de la grilla y margen extra alrededor del laberinto
        self.declare_parameter('resolucion',   0.225)
        self.declare_parameter('margen',       1.0)

        spawn_x     = self.get_parameter('spawn_x').value
        spawn_y     = self.get_parameter('spawn_y').value
        world_min_x = self.get_parameter('world_min_x').value
        world_max_x = self.get_parameter('world_max_x').value
        world_min_y = self.get_parameter('world_min_y').value
        world_max_y = self.get_parameter('world_max_y').value
        self.resolucion = self.get_parameter('resolucion').value
        margen      = self.get_parameter('margen').value

        # --- Parametros de la grilla en coordenadas de odometria ---
        # La odometria arranca en (0,0) cuando el robot esta en (spawn_x, spawn_y).
        # Para pasar de coordenadas del mundo a odom: x_odom = x_mundo - spawn_x
        # El borde inferior-izquierdo del laberinto en odom es:
        #   world_min - spawn - margen
        self.origen_x = (world_min_x - spawn_x) - margen
        self.origen_y = (world_min_y - spawn_y) - margen

        # Tamaño de la grilla: cubre el laberinto completo mas margen en cada lado
        # REVISAR
        ancho = (world_max_x - world_min_x) + 2.0 * margen
        alto  = (world_max_y - world_min_y) + 2.0 * margen
        self.cols  = int(ancho / self.resolucion) + 1
        self.filas = int(alto  / self.resolucion) + 1

        # --- Matriz de contadores internos ---
        # Cada celda almacena cuantas veces paso el robot por ella.
        # Se inicializa en 0 (no visitado).
        self.contadores = [[0] * self.cols for _ in range(self.filas)]
        self.ultima_celda = None

        # --- ROS2: suscripcion y publicacion ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # QoS Transient Local: RViz recibe el ultimo mensaje al conectarse
        self.grid_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/visited_grid', self.grid_qos)

        # Publicar grilla inicial vacia
        self.publicar_grilla()

        self.get_logger().info(
            f'GridMapper iniciado — grilla {self.filas}x{self.cols} '
            f'@ {self.resolucion}m/celda | '
            f'origen odom ({self.origen_x:.3f}, {self.origen_y:.3f}) | '
            f'spawn ({spawn_x:.3f}, {spawn_y:.3f})')

    # ------------------------------------------------------------------
    # Conversion coordenadas odom -> indices de celda
    # ------------------------------------------------------------------

    def mundo_a_celda(self, x, y):
        """Convierte una posicion (x, y) en frame odom a (fila, col) de la grilla.
        Devuelve None si la posicion esta fuera de los limites."""
        col  = int((x - self.origen_x) / self.resolucion)
        fila = int((y - self.origen_y) / self.resolucion)

        if 0 <= fila < self.filas and 0 <= col < self.cols:
            return fila, col
        return None

    # ------------------------------------------------------------------
    # Callback de odometria: registrar posicion
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        # Posicion directamente en frame odom (sin conversion)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        celda = self.mundo_a_celda(x, y)
        if celda is not None:
            fila, col = celda
            # Solo actualizar y publicar si el robot cambio de celda
            if celda != self.ultima_celda:
                self.contadores[fila][col] += 1
                self.ultima_celda = celda
                self.publicar_grilla()
        else:
            self.get_logger().warn(
                f'Posicion fuera de grilla: odom ({x:.2f}, {y:.2f})',
                throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Conversion contadores -> valores OccupancyGrid
    # ------------------------------------------------------------------

    def contador_a_ocupancia(self, count):
        """Mapea el numero de visitas a un valor de OccupancyGrid [0-100].

        Escala de grises por frecuencia de paso:
          0 visitas  ->  -1  (desconocido, gris en RViz2)
          1 visita   ->  25  (gris claro)
          2 visitas  ->  50  (gris medio)
          3 visitas  ->  75  (gris oscuro)
          4+ visitas -> 100  (negro, muy transitado)
        """
        if count == 0:
            return -1
        elif count == 1:
            return 25
        elif count == 2:
            return 50
        elif count == 3:
            return 75
        else:
            return 100

    # ------------------------------------------------------------------
    # Publicacion de la grilla
    # ------------------------------------------------------------------

    def publicar_grilla(self):
        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.resolucion
        msg.info.width  = self.cols
        msg.info.height = self.filas

        # Origen de la grilla en frame odom
        msg.info.origin.position.x = self.origen_x
        msg.info.origin.position.y = self.origen_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Lista plana row-major (fila 0 = y minimo)
        datos = []
        for fila in range(self.filas):
            for col in range(self.cols):
                datos.append(
                    self.contador_a_ocupancia(self.contadores[fila][col]))

        msg.data = datos
        self.grid_pub.publish(msg)

        celdas_visitadas = sum(
            1 for fila in self.contadores for v in fila if v > 0)
        self.get_logger().info(
            f'Grilla publicada — celdas visitadas: {celdas_visitadas} '
            f'/ {self.filas * self.cols}',
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GridMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()