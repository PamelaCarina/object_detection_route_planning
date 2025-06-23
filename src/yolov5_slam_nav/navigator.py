import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import os

class ObjectNavigator(Node):
    def __init__(self):
        super().__init__('object_navigator')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.objects_db = self.load_objects_from_file()

    def load_objects_from_file(self):
        file_path = os.path.expanduser("/home/jetson/object_detection_route_planning/src/yolov5_slam_nav/maps/objects_db.json")
        if not os.path.exists(file_path):
            self.get_logger().warn(f"No se encontró el archivo {file_path}. Ejecuta map_updater.py primero.")
            return {}

        with open(file_path, "r") as file:
            objects_db = json.load(file)

        self.get_logger().info(f"Objetos disponibles en la base de datos: {list(objects_db.keys())}")
        return objects_db

    def navigate_to_object(self, obj_name):
        if obj_name not in self.objects_db:
            self.get_logger().warn(f"El objeto '{obj_name}' no fue encontrado en la base de datos.")
            return

        x, y = self.objects_db[obj_name]

        # Orientación fija (mirando al este)
        yaw = 0.0
        q = quaternion_from_euler(0, 0, yaw)

        self.get_logger().info(f"Navegando hacia '{obj_name}' en ({x}, {y}) con orientación fija yaw={yaw:.2f} rad")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.action_client.wait_for_server()
        self.get_logger().info("Servidor de acciones de navegación encontrado")

        try:
            self.send_goal_future = self.action_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f"Error enviando goal: {e}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('La meta fue rechazada por el servidor de acciones.')
            return

        self.get_logger().info('Meta aceptada, esperando resultado...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info('Meta alcanzada exitosamente')
        else:
            self.get_logger().warn('Error al alcanzar la meta')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectNavigator()

    try:
        while rclpy.ok():
            print("\nObjetos detectados disponibles para navegación:")
            for obj in node.objects_db.keys():
                print(f" - {obj}")
            obj_name = input("\nIngrese el nombre del objeto al que desea navegar (o 'salir'): ").strip()
            if obj_name.lower() in ['salir', 'exit', 'q']:
                break
            node.navigate_to_object(obj_name)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
