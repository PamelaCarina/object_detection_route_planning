import rclpy
from rclpy.node import Node
import socket
import json
from std_msgs.msg import String

class YoloReceiver(Node):
    def __init__(self):
        super().__init__('yolo_receiver')

        # Crear un publicador
        self.publisher_ = self.create_publisher(String, 'detecciones', 10)

        # Configurar socket UDP
        self.UDP_IP = "192.168.1.96"  # IP del host (Jetson Nano)
        self.UDP_PORT = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))

        self.get_logger().info("Nodo de YOLOv5 escuchando en UDP")

        # Loop para recibir datos
        self.listen()

    def listen(self):
        while rclpy.ok():
            data, _ = self.sock.recvfrom(4096)  # Recibir datos
            detections = json.loads(data.decode())

            # Publicar detecciones en ROS 2
            msg = String()
            msg.data = json.dumps(detections)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Detecciones recibidas: {detections}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
