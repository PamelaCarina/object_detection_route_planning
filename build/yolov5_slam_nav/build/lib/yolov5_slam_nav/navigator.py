import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import yaml
import os

class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')

        # Publicador del mapa
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Cargar y publicar el mapa
        self.load_and_publish_map()

    def load_and_publish_map(self):
        """Carga el mapa desde archivo y lo publica en /map"""
        map_path = os.path.expanduser("~/map.pgm")
        yaml_path = os.path.expanduser("~/map.yaml")

        if not os.path.exists(map_path) or not os.path.exists(yaml_path):
            self.get_logger().warn("No se encontraron archivos de mapa.")
            return
        
        # Cargar configuración desde YAML
        with open(yaml_path, 'r') as file:
            map_metadata = yaml.safe_load(file)

        resolution = map_metadata["resolution"]
        origin = map_metadata["origin"]

        # Cargar imagen del mapa
        image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        height, width = image.shape

        # Convertir la imagen a datos de OccupancyGrid
        data = np.array(image, dtype=np.int8)
        data = np.where(data == 254, 0, data)   # Libre (blanco en PGM → 0 en ROS)
        data = np.where(data == 0, 100, data)   # Ocupado (negro en PGM → 100 en ROS)
        data = np.where(data == 205, -1, data)  # Desconocido (gris en PGM → -1 en ROS)

        # Crear mensaje OccupancyGrid
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "map"
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = origin[0]
        map_msg.info.origin.position.y = origin[1]
        map_msg.info.origin.position.z = 0.0
        map_msg.data = data.flatten().tolist()

        # Publicar mapa
        self.map_pub.publish(map_msg)
        self.get_logger().info("Mapa cargado y publicado en /map.")

def main(args=None):
    rclpy.init(args=args)
    node = MapLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

