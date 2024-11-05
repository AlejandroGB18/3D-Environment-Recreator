import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import struct
import pykinect_azure as pykinect
import open3d as o3d

class ObjectDigitizationNode(Node):
    def __init__(self):
        super().__init__('object_digitization_node')

        # Initialize Azure Kinect library
        pykinect.initialize_libraries()

        # Configure the Kinect camera
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

        # Start the Azure Kinect device
        self.device = pykinect.start_device(config=device_config)

        # Publisher for the digitized point cloud
        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'digitized_point_cloud', 10)

        # TF broadcaster for camera position
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for periodically digitizing the object
        self.timer = self.create_timer(0.5, self.publish_point_cloud)

        # Load canonical 3D model (could be a scanned template of the object)
        self.canonical_model = o3d.io.read_point_cloud("/path/to/canonical_model.pcd")
        self.get_logger().info("Canonical model loaded.")

        # Store the previous point cloud for ICP
        self.prev_cloud = None

    def publish_point_cloud(self):
        # Capture depth data from Azure Kinect
        capture = self.device.update()
        ret, depth_image = capture.get_depth_image()

        if not ret:
            self.get_logger().warn("Failed to get depth image.")
            return

        # Convert depth image to point cloud
        point_cloud = self.create_point_cloud_from_depth(depth_image)

        # If there's a previous cloud, perform ICP for alignment
        if self.prev_cloud is not None:
            aligned_cloud = self.align_to_canonical_model(point_cloud)
            self.publish_ros_point_cloud(aligned_cloud)
        else:
            self.prev_cloud = point_cloud
            self.publish_ros_point_cloud(point_cloud)

    def create_point_cloud_from_depth(self, depth_image):
        height, width = depth_image.shape
        points = []

        for y in range(height):
            for x in range(width):
                depth_value = depth_image[y, x]
                if depth_value == 0:
                    continue

                # Get 3D coordinates using Kinect calibration
                point_2d = pykinect.k4a_float2_t((x, y))
                point_3d = self.device.calibration.convert_2d_to_3d(
                    point_2d, depth_value, pykinect.K4A_CALIBRATION_TYPE_DEPTH, pykinect.K4A_CALIBRATION_TYPE_DEPTH)

                points.append([point_3d.xyz.x, point_3d.xyz.y, point_3d.xyz.z])

        # Create Open3D point cloud from points
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        return cloud

    def align_to_canonical_model(self, point_cloud):
        threshold = 0.02  # Threshold for ICP
        initial_transform = np.identity(4)

        # Perform ICP alignment to the canonical model
        result_icp = o3d.pipelines.registration.registration_icp(
            point_cloud, self.canonical_model, threshold, initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        self.get_logger().info(f"ICP fitness: {result_icp.fitness}, RMSE: {result_icp.inlier_rmse}")

        # Apply transformation to the point cloud
        aligned_cloud = point_cloud.transform(result_icp.transformation)
        return aligned_cloud

    def publish_ros_point_cloud(self, point_cloud):
        # Convert Open3D point cloud to ROS PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        points = np.asarray(point_cloud.points)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create PointCloud2 message
        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * points.shape[0],
            data=np.array(points, dtype=np.float32).tobytes()
        )

        # Publish the point cloud
        self.point_cloud_publisher.publish(point_cloud_msg)

    def publish_camera_transform(self):
        # Example: broadcast a static camera pose (adjust as needed)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDigitizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
