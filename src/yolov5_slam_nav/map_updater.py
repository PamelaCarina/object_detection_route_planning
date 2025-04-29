import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf2_ros
import json
import numpy as np
import os
import tf_transformations
import math
#from yolov5_slam_nav.srv import NavigateToObject
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Vector3Stamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_vector3

class MapUpdater(Node):
    def __init__(self):
        super().__init__('map_updater')

        # Suscripción al mapa de SLAM Toolbox
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

        # Suscripción a las detecciones de YOLOv5
        self.yolo_sub = self.create_subscription(String, 'detecciones', self.yolo_callback, 2)

        # Suscripción al LiDAR
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 5)

        # Publicador del mapa actualizado
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 5)

        # Publicador de marcadores en RViz2
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 5)

        # Servicio para compartir la base de datos de objetos detectados
        #self.get_object_position_srv = self.create_service(
        #    NavigateToObject, 'get_object_position', self.get_object_position
        #)

        # Transformaciones TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_map = None
        self.lidar_ranges = []
        self.objects_db = {} # Diccionario de objetos y coordenadas


    def map_callback(self, msg):
        """Almacena el mapa actual."""
        self.current_map = msg

    def lidar_callback(self, msg):
        """Almacena los datos del LiDAR."""
        self.lidar_ranges = msg.ranges

    def save_objects_to_file(self):
        """Guarda los objetos detectados en un archivo JSON."""
        script_dir = os.path.dirname(os.path.realpath(__file__))  # Ruta del script actual
        maps_dir = os.path.join(script_dir, "maps")  # Carpeta "maps"
        os.makedirs(maps_dir, exist_ok=True)  # Crea la carpeta si no existe

        file_path = os.path.join(maps_dir, "objects_db.json")  # Guarda en 'maps/'
        with open(file_path, "w") as file:
            json.dump(self.objects_db, file)
        self.get_logger().info(f"Objetos guardados en {file_path}")

    def yolo_callback(self, msg):
        """Coloca los objetos detectados en el mapa con su nombre."""
        if self.current_map is None:
            self.get_logger().warn("Mapa no recibido aún, esperando...")
            return

        detections = json.loads(msg.data)
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        for detection in detections:
            obj_class = detection["class"]
            obj_name = self.get_coco_label(obj_class)

            # Obtener coordenadas del centro del objeto, se calcula punto medio de la cajita
            obj_x_pixel = (detection["xmin"] + detection["xmax"]) / 2
            obj_y_pixel = (detection["ymin"] + detection["ymax"]) / 2

            #Entrego coordenadas del centro del objeto y me devuelve las coordenadas del objeto con respecto al robot
            obj_x_robot, obj_y_robot = self.image_to_robot(obj_x_pixel, obj_y_pixel, detection["distance_m"])
            
            # #Entrego coordenadas el objeto con respecto al robot y me devuelve coordenadas que objeto en el mapa
            # timestamp_float = detection["time"]  # por ejemplo: 1712847643.123456

            # # Separar segundos y nanosegundos
            # secs = int(timestamp_float)
            # nsecs = int((timestamp_float - secs) * 1e9)

            # # Crear objeto Time
            # ros_time = Time(seconds=secs, nanoseconds=nsecs)
            obj_x_mapa, obj_y_mapa = self.object_to_map(obj_x_robot, obj_y_robot)
            if obj_x_mapa is None or obj_y_mapa is None:
                continue  # Si la transformación falló, no colocar el marcador

            # Convertir a índice del mapa
            #map_x = int((obj_x_mapa - origin_x) / resolution)
            #map_y = int((obj_y_mapa - origin_y) / resolution)
            #index = map_y * width + map_x

            # Marcar en el mapa si está dentro de los límites
            #if 0 <= index < len(self.current_map.data):
            #    self.current_map.data[index] = 100  # Marca como ocupado

            # Publicar el marcador en RViz2
            self.publish_marker(obj_x_mapa, obj_y_mapa, obj_name)
            #Guarda las coordenadas del objeto en la base de datos
            self.objects_db[obj_name] = (obj_x_mapa, obj_y_mapa)

        self.save_objects_to_file()
        self.map_pub.publish(self.current_map)
        self.get_logger().info("Mapa actualizado con objetos detectados.")


    def publish_marker(self, x, y, name):
        """Publica un marcador en RViz2 con el nombre del objeto."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "objects"
        marker.id = hash(name) % 1000  # ID único para cada tipo de objeto
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Altura para que sea visible
        marker.scale.z = 0.5  # Tamaño del texto
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Transparencia
        marker.text = name  # Nombre del objeto detectado
        
        self.get_logger().info(f"Publicando marcador: {name} en ({x}, {y})")
        self.marker_pub.publish(marker)

    def image_to_robot(self, x_pixel, y_pixel, distance):
        """Convierte coordenadas de imagen a coordenadas del robot."""
        cam_fov = 90
        cam_width = 1280
        cam_height = 720

        #angulo es de izquierda a derecha de 0 a 90
        #luego se deja de -45 a 45 ya que necesitamos que se adapte a las coordenadas del robot (0,0)
        #desde el robot la y va hacia adelante y atras y la x de izquierda a derecha
        angle = (x_pixel / cam_width) * cam_fov - (cam_fov / 2)
        #distance = min(self.lidar_ranges) if self.lidar_ranges else 1.0

        #si el angulo es menor que 0 se multiplica por -1 porque esta hacia el lado izquierdo del robot
        #el cos del angulo se mueve entre -0,7 y 0    0 y 0,7
        x_robot = distance * np.cos(np.radians(angle))
        y_robot = distance * np.sin(np.radians(angle))

        # if(angle < 0):
        #     y_robot *= -1

        return x_robot, -y_robot


    def object_to_map(self, x_obj, y_obj):
        """Convierte coordenadas del robot a coordenadas globales usando TF, asegurando sincronización correcta."""
        try:
            # vector robot objeto
            vector_local = Vector3Stamped()
            vector_local.header.frame_id = "base_link"
            vector_local.vector.x = x_obj
            vector_local.vector.y = y_obj
            vector_local.vector.z = 0.0

            # obtener posicion del robot respecto al origen
            now = self.get_clock().now()  # Usa el tiempo actual del sistema
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', tf2_ros.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Transformamos el vector al frame del mapa (respeta la rotación)
            vector_global = do_transform_vector3(vector_local, transform)

            # Obtener las coordenadas base del robot
            x_robot = transform.transform.translation.x
            y_robot = transform.transform.translation.y

            # para obtener las coordenadas del objeto respecto al origen
            # debemos sumar coords_robot_objeto + coords_origen_robot
            x_obj_map = x_robot + vector_global.vector.x
            y_obj_map = y_robot + vector_global.vector.y

            

            return x_obj_map, y_obj_map
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la transformación TF: {str(e)}")
            return None, None


    def get_coco_label(self, class_id):
        """Devuelve el nombre de la clase basada en el COCO Dataset."""
        COCO_CLASSES = [
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
            "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
            "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]
        return COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else "Unknown"

def main(args=None):
    rclpy.init(args=args)
    node = MapUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()