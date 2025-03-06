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
        self.UDP_IP = "0.0.0.0"  # Escucha en todas las interfaces
        self.UDP_PORT = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))

        self.get_logger().info(f"Escuchando en {self.UDP_IP}:{self.UDP_PORT}")

        self.listen()

    def listen(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(4096)  # Recibir datos
                self.get_logger().info(f"Datos recibidos de {addr}: {data.decode()}")
                
                detections = json.loads(data.decode())

                # Publicar detecciones en ROS 2
                msg = String()
                msg.data = json.dumps(detections)
                self.publisher_.publish(msg)
                self.get_logger().info(f"Detecciones publicadas: {detections}")

            except Exception as e:
                self.get_logger().error(f"Error recibiendo datos: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

