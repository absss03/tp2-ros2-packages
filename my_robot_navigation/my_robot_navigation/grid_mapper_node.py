import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy
import math

class GridMapperNode(Node):

    def __init__(self):
        super().__init__('grid_mapper_node')

        # --- Parametros de la grilla ---
        # Resolucion: mitad de una celda de pared del laberinto (0.45m / 2)
        self.resolucion = 0.225   # metros por celda

        # Tamanio: 40x40 celdas -> cubre 9m x 9m
        self.filas = 40
        self.cols  = 40

        # Origen de la grilla en coordenadas del mundo Gazebo.
        # Con origen en (-4.5, -4.5) la grilla cubre de -4.5 a +4.5
        # en ambos ejes, centrada en el origen del mundo.
        self.origen_x = -4.5   # metros (esquina inferior izquierda)
        self.origen_y = -4.5   # metros (esquina inferior izquierda)

        # --- Matriz de contadores internos ---
        # Cada celda almacena cuantas veces paso el robot por ella.
        # Se inicializa en 0 (no visitado).
        self.contadores = [[0] * self.cols for _ in range(self.filas)]
        self.ultima_celda = None  # Almacena (fila, col) de la ultima posicion del robot

        # --- ROS2: suscripcion y publicacion ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Usar QoS Transient Local para que RViz reciba la grilla correctamente
        self.grid_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/visited_grid', self.grid_qos)

        # Publicar la grilla inicial vacia
        self.publicar_grilla()

        self.get_logger().info(
            f'GridMapper iniciado — grilla {self.filas}x{self.cols} '
            f'@ {self.resolucion}m/celda, '
            f'origen ({self.origen_x}, {self.origen_y})')

    # ------------------------------------------------------------------
    # Conversion coordenadas mundo -> indices de celda
    # ------------------------------------------------------------------

    def mundo_a_celda(self, x, y):
        """Convierte una posicion (x, y) del mundo a (fila, col) de la grilla.
        Devuelve None si la posicion esta fuera de los limites de la grilla."""
        col  = int((x - self.origen_x) / self.resolucion)
        fila = int((y - self.origen_y) / self.resolucion)

        if 0 <= fila < self.filas and 0 <= col < self.cols:
            return fila, col
        return None

    # ------------------------------------------------------------------
    # Callback de odometria: registrar posicion
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        celda = self.mundo_a_celda(x, y)
        if celda is not None:
            fila, col = celda
            # Solo actualizar y publicar si el robot se ha movido a una nueva celda
            if celda != self.ultima_celda:
                self.contadores[fila][col] += 1
                self.ultima_celda = celda
                self.publicar_grilla()
        else:
            self.get_logger().warn(
                f'Posicion fuera de grilla: ({x:.2f}, {y:.2f})',
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

        Esto permite distinguir visualmente zonas exploradas una sola vez
        de zonas recorridas repetidamente (giros en U, callejones, etc.).
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

        # Encabezado: mismo frame que la odometria
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        # Metadata de la grilla
        msg.info.resolution = self.resolucion
        msg.info.width  = self.cols
        msg.info.height = self.filas

        # Origen de la grilla en el frame 'odom'
        msg.info.origin.position.x = self.origen_x
        msg.info.origin.position.y = self.origen_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0   # sin rotacion

        # Convertir la matriz de contadores a lista plana (row-major).
        # OccupancyGrid espera los datos fila por fila, de abajo hacia arriba
        # (fila 0 = y minimo).
        datos = []
        for fila in range(self.filas):
            for col in range(self.cols):
                datos.append(
                    self.contador_a_ocupancia(self.contadores[fila][col]))

        msg.data = datos
        self.grid_pub.publish(msg)

        # Log periodico con estadisticas basicas
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