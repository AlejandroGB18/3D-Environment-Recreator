import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header  # Import Header
import pykinect_azure as pykinect

class PointCloudNode(Node):
    def __init__(self):
        super().__init__('point_cloud_node')
        
        # Initialize the Kinect library
        pykinect.initialize_libraries()

        # Configure the device
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        device_config.camera_fps = pykinect.K4A_FRAMES_PER_SECOND_30

        # Start the device
        self.device = pykinect.start_device(config=device_config)

        # Create a publisher for point clouds
        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)

        # Create a timer to call the capture method at a set frequency
        self.timer = self.create_timer(0.1, self.capture)

    def capture(self):
        # Get capture
        capture = self.device.update()

        # Get the depth image from the capture
        ret, depth_image = capture.get_depth_image()

        if ret:
            # Generate point cloud from the depth image
            point_cloud = self.create_point_cloud(depth_image)
            self.point_cloud_publisher.publish(point_cloud)
            self.get_logger().info('Publishing point cloud')

    def create_point_cloud(self, depth_image):
        # Get depth image size
        height, width = depth_image.shape

        # Create point cloud data
        points = []
        for v in range(height):
            for u in range(width):
                z = depth_image[v, u]  # Depth value
                if z > 0:  # Only consider points with valid depth
                    x = (u - width / 2) * z / 1000.0  # Convert to meters
                    y = (v - height / 2) * z / 1000.0  # Convert to meters
                    points.append((x, y, z / 1000.0))  # Store points in meters

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'kinect_frame'  # Set an appropriate frame ID

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1  # Set height to 1 for unorganized point cloud
        point_cloud_msg.width = len(points)  # Number of valid points
        point_cloud_msg.is_dense = True if len(points) == (width * height) else False
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud_msg.point_step = 12  # 4 bytes for x, 4 bytes for y, 4 bytes for z
        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width

        # Flatten the points list into a byte array
        if points:
            points_array = np.array(points, dtype=np.float32).tobytes()
            point_cloud_msg.data = points_array
        else:
            point_cloud_msg.data = np.array([], dtype=np.float32).tobytes()

        return point_cloud_msg

def main(args=None):
    rclpy.init(args=args)
    point_cloud_node = PointCloudNode()
    rclpy.spin(point_cloud_node)
    point_cloud_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
