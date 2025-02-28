import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import tf2_ros
import tf_transformations

class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')

        # Suscribirse a detecciones de YOLOv5
        self.subscription = self.create_subscription(
            String, 'detecciones', self.detection_callback, 10)

        # Publicador de objetos en el mapa
        self.marker_pub = self.create_publisher(PoseStamped, 'object_map', 10)

        # Buffer de transformaciones
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def detection_callback(self, msg):
        detections = json.loads(msg.data)

        # Obtener la posición actual del robot en el mapa
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            for det in detections:
                # Aquí se debe calcular la posición real del objeto en el mapa
                obj_x = robot_x + 1.0  # Suponiendo 1 metro adelante (Ejemplo)
                obj_y = robot_y

                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = obj_x
                pose.pose.position.y = obj_y
                pose.pose.orientation.w = 1.0

                self.marker_pub.publish(pose)
                self.get_logger().info(f"Objeto detectado en mapa: {obj_x}, {obj_y}")

        except Exception as e:
            self.get_logger().warn(f"Error en transformación TF: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

