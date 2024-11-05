import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import pykinect_azure as pykinect
from pykinect_azure import K4A_CALIBRATION_TYPE_COLOR, k4a_float2_t
from ultralytics import YOLO

class KinectYoloNode(Node):
    def __init__(self):
        super().__init__('kinect_yolo_node')

        # Inicializar la librería Kinect Azure
        pykinect.initialize_libraries()

        # Configuración de la cámara
        device_config = pykinect.default_configuration
        device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

        # Iniciar el dispositivo
        self.device = pykinect.start_device(config=device_config)

        # Publicadores ROS para la imagen color y las detecciones
        self.color_image_publisher = self.create_publisher(Image, 'color_image', 10)
        self.detections_publisher = self.create_publisher(Image, 'yolo_detections_image', 10)
        self.detections_info_publisher = self.create_publisher(String, 'yolo_detections_info', 10)

        # Inicializar YOLOv8 con el modelo entrenado
        self.model = YOLO('/home/alejandro/Downloads/runs/detect/train2/weights/best.pt')  # Cambia esta ruta según tu archivo

        # Temporizador para capturar imágenes periódicamente
        self.timer = self.create_timer(0.1, self.process_images)

        # CV Bridge para convertir entre OpenCV y ROS
        self.bridge = CvBridge()

    def process_images(self):
        # Obtener la captura
        capture = self.device.update()

        # Obtener la imagen de color
        ret_color, color_image = capture.get_color_image()

        if not ret_color:
            self.get_logger().warn("Error al capturar la imagen")
            return

        # Convertir la imagen de color de Azure Kinect a formato OpenCV (BGR)
        color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)

        # Realizar la detección de objetos con YOLOv8
        results = self.model(color_image_bgr)

        # Dibujar las cajas de detección en la imagen
        detections = ""
        for result in results:
            for obj in result.boxes:
                # Dibujar las cajas en la imagen
                x1, y1, x2, y2 = map(int, obj.xyxy[0])  # Convertir coordenadas a enteros
                cv2.rectangle(color_image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{obj.cls} {obj.conf:.2f}"
                cv2.putText(color_image_bgr, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                
                detections += f"Objeto: {obj.cls}, Confianza: {obj.conf}, Coordenadas: {x1}, {y1}, {x2}, {y2}\n"

        # Publicar la imagen con las detecciones superpuestas
        detection_image_msg = self.bridge.cv2_to_imgmsg(color_image_bgr, encoding="bgr8")
        self.detections_publisher.publish(detection_image_msg)

        # Publicar las detecciones en formato de texto
        self.detections_info_publisher.publish(String(data=detections))
        self.get_logger().info('Detecciones publicadas')

        # Publicar la imagen original sin detecciones (opcional)
        color_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgra8")
        self.color_image_publisher.publish(color_image_msg)

def main(args=None):
    rclpy.init(args=args)
    kinect_yolo_node = KinectYoloNode()
    rclpy.spin(kinect_yolo_node)

    # Shutdown
    kinect_yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
