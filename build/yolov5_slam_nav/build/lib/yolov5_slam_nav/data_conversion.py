import rclpy
from rclpy.node import Node
import json
import tf2_ros
import tf_transformations
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

class DataConversionNode(Node):
    def __init__(self):
        super().__init__('data_conversion')

        # Suscribirse al tópico de detecciones de YOLOv5
        self.subscription = self.create_subscription(
            String,
            'detecciones',
            self.detections_callback,
            10
        )

        # Publicador de marcadores en RViz
        self.marker_pub = self.create_publisher(MarkerArray, 'yolo_markers', 10)

        # Buffer de TF2 para transformaciones
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Nodo de conversión de datos iniciado")

    def detections_callback(self, msg):
        """ Procesa las detecciones de YOLOv5 y las transforma a coordenadas del mapa. """
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Error al decodificar JSON")
            return

        marker_array = MarkerArray()

        # Definir el frame de la cámara basado en RViz
        camera_frame = "camera_link"  # Si no aparece en RViz, usar "base_link"

        for i, detection in enumerate(detections):
            class_id = detection["class"]
            confidence = detection["confidence"]

            # Suponemos que la detección está a 1 metro de la cámara
            z_distance = 1.0  

            # Crear un punto en el frame de la cámara
            point_camera = PointStamped()
            point_camera.header.frame_id = camera_frame
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.point.x = z_distance
            point_camera.point.y = (detection["xmin"] + detection["xmax"]) / 2 / 640.0 - 0.5
            point_camera.point.z = (detection["ymin"] + detection["ymax"]) / 2 / 480.0 - 0.5

            # Esperar a que la transformación esté disponible
            try:
                if not self.tf_buffer.can_transform("map", camera_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)):
                    self.get_logger().warn(f"Esperando transformación de {camera_frame} a map...")
                    return

                transform = self.tf_buffer.lookup_transform("map", camera_frame, rclpy.time.Time())
                point_map = do_transform_point(point_camera, transform)

                # Crear un marcador en RViz
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "yolo_detections"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point_map.point.x
                marker.pose.position.y = point_map.point.y
                marker.pose.position.z = 0.1
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0 if class_id == 0 else 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0 if class_id != 0 else 0.0

                marker_array.markers.append(marker)

            except Exception as e:
                self.get_logger().error(f"No se pudo transformar coordenadas: {e}")

        # Publicar los marcadores en RViz
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DataConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

