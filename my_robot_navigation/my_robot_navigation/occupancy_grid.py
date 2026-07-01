import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy
import math

class PathGridNode(Node):
    def __init__(self):
        super().__init__('path_grid_node')

        # 1. Suscriptor a la odometría de Gazebo
        self.subscription = self.create_subscription(
            Odometry,
            '/real_odom',
            self.odom_callback,
            10)

        self.grid_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 2. Publicador de la grilla de ocupación
        self.publisher = self.create_publisher(
            OccupancyGrid, 
            '/path_grid', 
            self.grid_qos)

        # 3. Configuración de la grilla
        self.resolution = 0.25  # 1 metro por celda
        self.width = 200       # 200 metros de ancho
        self.height = 200      # 200 metros de alto

        # Centrar el origen del mapa para que el robot empiece en el medio (0,0)
        self.origin_x = - (self.width * self.resolution) / 2.0
        self.origin_y = - (self.height * self.resolution) / 2.0

        # Inicializar el mapa: -1 = desconocido, 0 = libre, 100 = ocupado/visitado
        self.grid_data = [-1] * (self.width * self.height)

        # 4. Timer para publicar el mapa a 1Hz
        self.timer = self.create_timer(1.0, self.publish_grid)
        
        self.get_logger().info("Nodo PathGrid iniciado. Resolución: 1m/celda.")

    def odom_callback(self, msg):
        # Extraer la posición (x, y) actual del diferencial
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Mapear coordenadas del mundo (metros) a índices de la matriz
        grid_x = int(math.floor((x - self.origin_x) / self.resolution))
        grid_y = int(math.floor((y - self.origin_y) / self.resolution))

        # Asegurarse de que el robot no se haya salido de los límites de la grilla
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            # Calcular el índice en el arreglo 1D (Row-major order)
            index = grid_y * self.width + grid_x
            
            # Marcar la celda como visitada (100)
            self.grid_data[index] = 100 

    def publish_grid(self):
        grid_msg = OccupancyGrid()

        # Configurar el Header
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'odom' # Asegúrate de que coincida con tu árbol TF
    

        # Configurar la Metadata (MapMetaData)
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Configurar el origen
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # Asignar los datos
        grid_msg.data = self.grid_data

        # Publicar
        self.publisher.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()