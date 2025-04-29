# -*- coding: utf-8 -*-
import torch
import cv2
import numpy as np
import socket
import json
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.datasets import letterbox
import time

# Cargar las clases del COCO Dataset
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

# Configuracion del socket UDP
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Cargar modelo YOLOv5
weights = 'yolov5s.pt'
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = attempt_load(weights, map_location=device)
model.eval()

# Configuracion de la camara
WIDTH = 1280
HEIGHT = 720

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cv2.namedWindow("Deteccion con COCO Dataset", cv2.WINDOW_NORMAL)

if not cap.isOpened():
    print("No se pudo abrir la camara")
    exit()

print("Camara detectada. Enviando datos por UDP. Presiona 'q' para salir.")

# obtener framerate de la camara para sincronizar con el envio de mensajes
fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0:  # fallback si no lo detecta
    fps = 30

frame_interval = 1.0 / fps

# Tamaños reales aproximados de algunos objetos (en cm)
REAL_WIDTHS = {
    "bird": 10.9,
    "cat": 20.0,
    "dog": 25,
    "umbrella": 90,
    "handbag": 30,
    "bottle": 7,
    "cup": 8,
    "fork": 2.5,
    "knife": 2.5,
    "spoon": 3,
    "bowl": 20,
    "banana": 15,
    "apple": 8,
    "sandwich": 12,
    "orange": 9,
    "carrot": 14,
    "laptop": 35,
    "mouse": 6,
    "keyboard": 40,
    "cell phone": 7,
    "book": 14,
    "scissors": 8,
    "teddy bear": 25,
    "toothbrush": 3
}

# Variables de calibración
calibrated = False
FOCAL_LENGTH = None

# Función para calcular la distancia focal
def calcular_focal(ancho_px, ancho_real_cm, distancia_cm):
    return (ancho_px * distancia_cm) / ancho_real_cm

# Función para calcular la distancia
def calcular_distancia(ancho_px, ancho_real_cm, focal):
    return (ancho_real_cm * focal) / ancho_px

while True:
    start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        print("No se pudo capturar el frame.")
        break

    # Preprocesamiento
    img = letterbox(frame, 640, stride=int(model.stride.max()))[0]
    img = img.transpose((2, 0, 1))[::-1]  # BGR a RGB
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device).float() / 255.0
    img = img.unsqueeze(0)

    # Inferencia
    pred = model(img)[0]
    pred = non_max_suppression(pred, 0.25, 0.45, classes=[14,15,16,25,26,39,41,42,43,44,45,46,47,48,49,51,63,64,66,67,73,76,77,79])

    detections = []
    for det in pred:
        if len(det):
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()
            for *xyxy, conf, cls in det:
                class_id = int(cls)
                label_text = COCO_CLASSES[class_id]
                bbox_width = int(xyxy[2]) - int(xyxy[0])

                distancia_cm = None
                if label_text in REAL_WIDTHS and bbox_width > 0:
                    if not calibrated:
                        # Asume distancia real durante calibración (por ejemplo, 40 cm)
                        FOCAL_LENGTH = calcular_focal(bbox_width, REAL_WIDTHS[label_text], 40)
                        print("Distancia focal calibrada con {}: {:.2f} px".format(label_text, FOCAL_LENGTH))
                        calibrated = True
                    elif FOCAL_LENGTH is not None:
                        distancia_cm = calcular_distancia(bbox_width, REAL_WIDTHS[label_text], FOCAL_LENGTH)
                if conf > 0.7:
                    detection = {
                        "xmin": int(xyxy[0]),
                        "ymin": int(xyxy[1]),
                        "xmax": int(xyxy[2]),
                        "ymax": int(xyxy[3]),
                        "confidence": float(conf),
                        "class": class_id,
                        "label": label_text,
                        "distance_m": round(distancia_cm / 100, 2) if distancia_cm else 0.4
                    }
                    detections.append(detection)

                # Dibujar en pantalla
                label = f'{label_text} {conf:.2f}'
                if distancia_cm:
                    label += f' {distancia_cm / 100:.2f}m'
                cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Enviar detecciones por UDP
    if detections:
        message = json.dumps(detections)
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

    cv2.imshow('Deteccion con COCO Dataset', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    elapsed_time = time.time() - start_time
    sleep_time = frame_interval - elapsed_time
    if sleep_time > 0:
        time.sleep(sleep_time)

cap.release()
cv2.destroyAllWindows()
sock.close()
print("Deteccion finalizada.")
