import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import pykinect_azure as pykinect
import open3d as o3d
from geometry_msgs.msg import TransformStamped
import tf2_ros
import struct
import matplotlib.pyplot as plt

class AzureKinectPointCloudNode(Node):
    def __init__(self):
        super().__init__('azure_kinect_point_cloud_node')

        # Inicializa la librería PyKinectAzure
        pykinect.initialize_libraries()
        self.get_logger().info("PyKinectAzure libraries initialized.")

        # Configuración de la cámara
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

        # Iniciar el dispositivo
        self.device = pykinect.start_device(config=device_config)
        self.get_logger().info("Azure Kinect device started.")

        # Crear el publicador de PointCloud2
        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'scene_point_cloud', 10)
        self.get_logger().info("PointCloud2 publisher created.")

        # Crear un TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("TF broadcaster created.")

        # Timer para capturar y publicar datos de la nube de puntos
        self.timer = self.create_timer(0.5, self.publish_point_cloud)

        # Timer para publicar la transformación de la cámara
        self.tf_timer = self.create_timer(0.1, self.publish_camera_transform)

    def publish_camera_transform(self):
        # Crear una transformación desde el frame 'world' al frame 'camera_link'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # Frame fijo
        t.child_frame_id = 'camera_link'  # Frame de la cámara

        # Ajustar la posición y orientación de la cámara
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.5  # Colocamos la cámara 0.5 metros por encima del plano XY

        # Rotar la cámara para apuntar al plano XY correctamente (ajuste importante)
        t.transform.rotation.x = -0.707  # 45 grados hacia abajo en el eje X
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.707  # Ajuste con quaternion

        # Publicar la transformación
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Camera transform published.")

    def publish_point_cloud(self):
        # Capturar los datos de profundidad y color desde Kinect
        capture = self.device.update()
        ret_depth, depth_image = capture.get_depth_image()
        ret_color, color_image = capture.get_color_image()

        if not ret_depth:
            self.get_logger().warn("Failed to capture depth image.")
            return

        if not ret_color:
            self.get_logger().warn("Failed to capture color image.")
            return

        # Convertir la imagen de profundidad a nube de puntos
        self.get_logger().info("Converting depth image to point cloud.")
        point_cloud = self.create_point_cloud_from_depth_and_color(depth_image, color_image)

        # Publicar la nube de puntos
        self.publish_ros_point_cloud(point_cloud)
        self.get_logger().info("Point cloud published.")


    def create_point_cloud_from_depth_and_color(self, depth_image, color_image):
        height, width = depth_image.shape
        points = []
        colors = []

        min_z, max_z = None, None

        # Primero, calcular los límites de z (la altura)
        for y in range(height):
            for x in range(width):
                depth_value = depth_image[y, x]
                if depth_value == 0:
                    continue

                point_2d = pykinect.k4a_float2_t((x, y))
                point_3d = self.device.calibration.convert_2d_to_3d(
                    point_2d, depth_value, pykinect.K4A_CALIBRATION_TYPE_DEPTH, pykinect.K4A_CALIBRATION_TYPE_DEPTH)

                if min_z is None or point_3d.xyz.z < min_z:
                    min_z = point_3d.xyz.z
                if max_z is None or point_3d.xyz.z > max_z:
                    max_z = point_3d.xyz.z

        # Crear el punto 3D y el color correspondiente
        for y in range(height):
            for x in range(width):
                depth_value = depth_image[y, x]
                if depth_value == 0:
                    continue

                point_2d = pykinect.k4a_float2_t((x, y))
                point_3d = self.device.calibration.convert_2d_to_3d(
                    point_2d, depth_value, pykinect.K4A_CALIBRATION_TYPE_DEPTH, pykinect.K4A_CALIBRATION_TYPE_DEPTH)

                points.append([point_3d.xyz.x, point_3d.xyz.y, point_3d.xyz.z])

                # Escalar z al rango [0, 1] para mapear colores
                z_scaled = (point_3d.xyz.z - min_z) / (max_z - min_z)
            
                # Mapear a una paleta de colores (usamos la paleta jet)
                cmap = plt.get_cmap('jet')
                rgba_color = cmap(z_scaled)

                # Añadir el color RGB (ignoramos el canal alfa)
                colors.append([rgba_color[0], rgba_color[1], rgba_color[2]])

        # Crear la nube de puntos Open3D
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors)

        return cloud


    def publish_ros_point_cloud(self, point_cloud):
        # Usar la estructura PointCloud2 de ROS 2 y los campos del mensaje
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'  # Publicamos la nube de puntos en el frame de la cámara

        # Extraer los puntos y colores de Open3D PointCloud
        points = np.asarray(point_cloud.points)
        colors = np.asarray(point_cloud.colors)  # Extraer el color

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1)
        ]

        # Empaquetar los puntos y colores en formato bytes
        point_cloud_data = []
        for point, color in zip(points, colors):
            r, g, b = (color * 255).astype(np.uint8)
            point_cloud_data.append(struct.pack('fffBBB', point[0], point[1], point[2], r, g, b))

        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            fields=fields,
            is_bigendian=False,
            point_step=15,  # 12 para xyz, 3 para rgb
            row_step=15 * points.shape[0],
            data=b''.join(point_cloud_data),
            is_dense=True
        )

        # Publicar la nube de puntos
        self.point_cloud_publisher.publish(point_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AzureKinectPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
