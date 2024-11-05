import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import struct
import pykinect_azure as pykinect

class KinectPointCloudNode(Node):
    def __init__(self):
        super().__init__('kinect_point_cloud_node_2')

        # Inicializar la biblioteca de Kinect Azure
        pykinect.initialize_libraries()

        # Configuración de la cámara
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

        # Iniciar el dispositivo
        self.device = pykinect.start_device(config=device_config)

        # Crear el publicador de la nube de puntos
        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'filtered_point_cloud', 10)

        # Crear un broadcaster de tf
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Configurar el temporizador para capturar y publicar la nube de puntos y la transformación
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        # Publicar la transformación de la cámara
        self.publish_camera_transform()

        # Capturar la imagen de profundidad y publicar la nube de puntos filtrada
        self.publish_filtered_point_cloud()

    def publish_camera_transform(self):
        # Crear un mensaje de transformación
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # El frame base
        t.child_frame_id = 'camera_link'  # El frame de la cámara

        # Definir la posición de la cámara (0, 0, 0) en el frame 'world'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Definir la orientación de la cámara (sin rotación)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publicar la transformación
        self.tf_broadcaster.sendTransform(t)

    def publish_filtered_point_cloud(self):
        # Capturar la imagen de profundidad
        capture = self.device.update()
        ret, depth_image = capture.get_depth_image()

        if not ret:
            self.get_logger().warn("No se pudo obtener la imagen de profundidad.")
            return

        # Obtener las dimensiones de la imagen de profundidad
        height, width = depth_image.shape

        # Crear una lista para almacenar los puntos XYZ
        points = []

        # Limitar la nube de puntos a 30 cm
        max_depth_mm = 300  # 300 mm es igual a 30 cm

        for y in range(height):
            for x in range(width):
                # Obtener la distancia en mm
                depth_value = depth_image[y, x]
                if depth_value == 0 or depth_value > max_depth_mm:
                    continue

                # Convertir coordenadas 2D a 3D
                point_2d = pykinect.k4a_float2_t((x, y))
                point_3d = self.device.calibration.convert_2d_to_3d(
                    point_2d, depth_value, pykinect.K4A_CALIBRATION_TYPE_DEPTH, pykinect.K4A_CALIBRATION_TYPE_DEPTH)

                # Convertir de mm a metros
                point_3d_m = [point_3d.xyz.x / 1000.0, point_3d.xyz.y / 1000.0, point_3d.xyz.z / 1000.0]

                # Agregar el punto a la lista (x, y, z)
                points.append(point_3d_m)

        # Si no hay puntos, no publicar
        if len(points) == 0:
            return

        # Crear el mensaje de ROS PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'  # Publicamos la nube de puntos en el frame de la cámara

        # Convertir la lista de puntos en un formato adecuado para PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Empaquetar los puntos en una lista de bytes
        point_cloud_data = []
        for point in points:
            point_cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))

        # Concatenar todos los datos de puntos
        point_cloud_data = b''.join(point_cloud_data)

        # Crear el mensaje de nube de puntos
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # Cada punto tiene 3 floats (4 bytes cada uno)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = point_cloud_data
        cloud_msg.is_dense = True

        # Publicar la nube de puntos
        self.point_cloud_publisher.publish(cloud_msg)
        self.get_logger().info(f"Nube de puntos filtrada publicada con {len(points)} puntos.")

def main(args=None):
    rclpy.init(args=args)
    point_cloud_node = KinectPointCloudNode()
    rclpy.spin(point_cloud_node)

    # Shutdown
    point_cloud_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
