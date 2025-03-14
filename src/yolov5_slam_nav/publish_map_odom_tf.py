import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class MapOdomTFPublisher(Node):
    def __init__(self):
        super().__init__('map_odom_tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)  # Publicar cada 0.1s

    def broadcast_tf(self):
        t = TransformStamped()
        
        # 🔹 Usamos el tiempo actual del sistema
        #t.header.stamp = self.get_clock().now().to_msg()
        #t.header.stamp = rclpy.time.Time().to_msg()
        now = self.get_clock().now()
        t.header.stamp = rclpy.time.Time(seconds=now.nanoseconds * 1e-9).to_msg()
        self.get_logger().info(f"Publicando TF map → odom en el tiempo: {t.header.stamp.sec}.{t.header.stamp.nanosec}")

        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

