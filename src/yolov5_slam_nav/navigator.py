import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import json
import os
import math

class ObjectNavigator(Node):
    def __init__(self):
        super().__init__('object_navigator')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Cargar la base de datos de objetos detectados
        self.objects_db = self.load_objects_from_file()

        #Transformaciones TF para saber posición actual del robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def load_objects_from_file(self):
        """Carga los objetos desde el archivo JSON guardado por map_updater.py."""
        file_path = os.path.expanduser("/home/jetson/object_detection_route_planning/src/yolov5_slam_nav/maps/objects_db.json")
        if not os.path.exists(file_path):
            self.get_logger().warn(f"No se encontró el archivo {file_path}. Asegúrate de ejecutar map_updater.py primero.")
            return {}

        with open(file_path, "r") as file:
            objects_db = json.load(file)

        self.get_logger().info(f"Objetos disponibles en la base de datos: {list(objects_db.keys())}")
        return objects_db

    def navigate_to_object(self, obj_name):
        """Envía una meta de navegación al objeto si está en la base de datos."""
        if obj_name not in self.objects_db:
            self.get_logger().warn(f"El objeto '{obj_name}' no fue encontrado en la base de datos.")
            return

        x, y = self.objects_db[obj_name]  # Extraer coordenadas del JSON
        self.get_logger().info(f"Navegando hacia '{obj_name}' en ({x}, {y}), orientacion=({q[0]}, {q[1]}, {q[2]}, {q[3]})")

        # Obtener la posición actual del robot
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=Duration(seconds=1.0)
            )
            x_robot = trans.transform.translation.x
            y_robot = trans.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la pose actual del robot: {e}")
            return

        # Calcular el yaw para mirar hacia el objeto
        dx = x_goal - x_robot
        dy = y_goal - y_robot
        yaw = math.atan2(dy, dx)
        q = quaternion_from_euler(0, 0, yaw)

        # Crear y enviar el goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x_goal
        goal_msg.pose.pose.position.y = y_goal
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # Crear el mensaje de meta para la acción NavigateToPose
        #goal_msg = NavigateToPose.Goal()
        #goal_msg.pose.header.frame_id = 'map'
        #goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        #goal_msg.pose.pose.position.x = x
        #goal_msg.pose.pose.position.y = y

        #orientación del objetivo "mirando al este" (0 radianes)
        #q = quaternion_from_euler(0, 0, 0)
        #goal_msg.pose.pose.orientation.x = q[0]
        #goal_msg.pose.pose.orientation.y = q[1]
        #goal_msg.pose.pose.orientation.z = q[2]
        #goal_msg.pose.pose.orientation.w = q[3]

        # Esperar a que el servidor de acciones esté disponible
        self.action_client.wait_for_server()
        self.get_logger().info("Servidor de acciones de navegación encontrado")

        # Enviar la meta
        #self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        #self.send_goal_future.add_done_callback(self.goal_response_callback)

        try:
            self.send_goal_future = self.action_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f"Error enviando goal: {e}")

    def goal_response_callback(self, future):
        """Maneja la respuesta de Nav2 al objetivo de navegación."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('La meta fue rechazada por el servidor de acciones.')
            return

        self.get_logger().info('Meta aceptada, esperando resultado...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Maneja el resultado de la navegación."""
        result = future.result()
        if result:
            self.get_logger().info('Meta alcanzada exitosamente')
        else:
            self.get_logger().warn('Error al alcanzar la meta')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectNavigator()

    # Mostrar los objetos disponibles
    print("\nObjetos detectados disponibles para navegación:")
    for obj in node.objects_db.keys():
        print(f" - {obj}")

    # Pedir el nombre del objeto a navegar
    obj_name = input("\nIngrese el nombre del objeto al que desea navegar: ").strip()
    
    node.navigate_to_object(obj_name)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()