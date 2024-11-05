import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import pykinect_azure as pykinect
from sklearn.decomposition import PCA
import dlib

class FaceTrackingNode(Node):
    def __init__(self):
        super().__init__('face_tracking_node')
        self.get_logger().info("Inicializando nodo de detección de rostro y seguimiento")

        # Inicializar la API de pykinect_azure
        pykinect.initialize_libraries(track_body=True)
        self.device = pykinect.start_device()

        # Detector de rostros usando dlib
        self.face_detector = dlib.get_frontal_face_detector()

        # PCA para Eigenface (detector de rostro)
        self.pca = PCA(n_components=100)  # Reducimos a 100 componentes principales
        self.known_faces = []  # Rostros conocidos
        self.labels = []  # Etiquetas para cada rostro conocido
        self.pca_fitted = False  # Bandera para saber si el PCA ha sido ajustado

        # Temporizador para capturar imágenes y procesarlas cada 100 ms
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Capturar imagen de la cámara
        capture = self.device.update()
        if capture is None:
            self.get_logger().error("Error al actualizar la captura")
            return

        color_image_tuple = capture.get_color_image()
        if color_image_tuple is None or len(color_image_tuple) == 0:
            self.get_logger().error("No se pudo obtener la imagen de color")
            return

        # Verifica el tipo y longitud de color_image_tuple
        self.get_logger().info(f"Tipo de color_image: {type(color_image_tuple)}")
        
        color_image = color_image_tuple[0] 

        if isinstance(color_image, np.ndarray):
            self.get_logger().info(f"Tamaño de la imagen RGB (antes de conversión): {color_image.shape}")

            # Convertir BGRA a BGR
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
            self.get_logger().info(f"Tamaño de la imagen RGB (después de conversión): {color_image.shape}")

            # Convertir a escala de grises
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Detectar rostros en la imagen
            faces = self.face_detector(gray_image)

            if len(faces) > 0:
                self.get_logger().info(f"Se detectaron {len(faces)} rostros")
                for face in faces:
                    # Extraer la región de interés (ROI) del rostro
                    x, y, w, h = (face.left(), face.top(), face.width(), face.height())
                    face_roi = gray_image[y:y+h, x:x+w]
                    face_resized = cv2.resize(face_roi, (100, 100)).flatten()

                    # Entrenar el PCA en el primer rostro si no está entrenado
                    if not self.pca_fitted:
                        self.pca.fit([face_resized])  # Ajustar el PCA
                        self.pca_fitted = True
                        self.get_logger().info("PCA ajustado con el primer rostro")

                    # Aplicar Eigenface (PCA)
                    face_pca = self.pca.transform([face_resized])

                    # Comprobar si el rostro es conocido
                    if len(self.known_faces) > 0:
                        distances = np.linalg.norm(self.known_faces - face_pca, axis=1)
                        min_distance_index = np.argmin(distances)
                        if distances[min_distance_index] < 1.0:
                            self.get_logger().info(f"Rostro identificado como: {self.labels[min_distance_index]}")
                        else:
                            self.get_logger().info("Rostro no identificado. Añadiendo a la base de datos")
                            self.known_faces.append(face_pca)
                            self.labels.append(f"Persona_{len(self.labels)}")
                    else:
                        self.known_faces.append(face_pca)
                        self.labels.append(f"Persona_0")
                        self.get_logger().info("Primer rostro registrado")

                    # Obtener la distancia al rostro usando la imagen de profundidad
                    depth_image = capture.get_depth_image()
                    if depth_image is not None:
                        depth_value = depth_image[y + h // 2, x + w // 2]
                        if depth_value > 0:
                            self.get_logger().info(f"Distancia al rostro: {depth_value} mm")
                        else:
                            self.get_logger().info("No se pudo obtener la profundidad del rostro")
                    else:
                        self.get_logger().info("Imagen de profundidad no disponible")

                    # Dibujar rectángulo alrededor del rostro en la imagen RGB
                    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Mostrar la imagen RGB con las detecciones
            cv2.imshow("Azure Kinect RGB", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Cerrando nodo...")
                cv2.destroyAllWindows()
                rclpy.shutdown()
        else:
            self.get_logger().error("La imagen de color no es válida")

def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
