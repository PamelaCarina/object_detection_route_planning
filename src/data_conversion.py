import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import json
import tf2_ros
import tf_transformations
from sensor_msgs.msg import PointCloud2, PointField
import struct

class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')

        # Suscripción a detecciones de YOLOv5
        self.subscription = self.create_subscription(
            String, 'detecciones', self.detection_callback, 10)

        # Publicadores
        self.point_pub = self.create_publisher(PointStamped, 'object_map', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'object_points_cloud', 10)

        # Buffer de transformaciones
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def detection_callback(self, msg):
        detections = json.loads(msg.data)

        # Obtener la transformación de la cámara al mapa
        try:
            trans = self.tf_buffer.lookup_transform("map", "camera_link", rclpy.time.Time())

            points = []

            for det in detections:
                if det['confidence'] < 0.7:
                    continue  # Filtrar detecciones con baja confianza

                # Crear punto en el frame de la cámara
                point = PointStamped()
                point.header.frame_id = "camera_link"
                point.point.x = det['x']  # Asumiendo que YOLO ya da coordenadas 3D
                point.point.y = det['y']
                point.point.z = det.get('z', 0)  # Si no hay 'z', asumir 0

                # Transformar al frame del mapa
                point_transformed = self.tf_buffer.transform(point, "map")

                # Publicar punto individual
                self.point_pub.publish(point_transformed)
                
                points.append([point_transformed.point.x, point_transformed.point.y, point_transformed.point.z])

                self.get_logger().info(f"Objeto detectado en mapa: {point_transformed.point.x}, {point_transformed.point.y}")

            # Publicar en PointCloud2
            self.publish_pointcloud(points)

        except Exception as e:
            self.get_logger().warn(f"Error en transformación TF: {e}")

    def publish_pointcloud(self, points):
        """ Publica una nube de puntos con los objetos detectados """
        cloud = PointCloud2()
        cloud.header.frame_id = "map"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b"".join([struct.pack("fff", *p) for p in points])

        self.cloud_pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

