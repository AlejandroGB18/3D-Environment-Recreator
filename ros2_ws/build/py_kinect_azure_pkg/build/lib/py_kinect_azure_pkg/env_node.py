import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import struct
from geometry_msgs.msg import TransformStamped
import tf2_ros

class KinectEnvNode(Node):
    def __init__(self):
        super().__init__('kinect_env_node')

        # ROS publishers
        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'scene_point_cloud', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ROS subscribers to receive shared color and depth images
        self.create_subscription(Image, '/shared/color_image', self.color_image_callback, 10)
        self.create_subscription(Image, '/shared/depth_image', self.depth_image_callback, 10)

        # Image and depth processing bridge
        self.bridge = CvBridge()

        # Initialize variables for storing images
        self.color_image = None
        self.depth_image = None

        # Timer for publishing point cloud and transformation data
        self.timer = self.create_timer(0.05, self.publish_point_cloud)

    def color_image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info("Color image received")

    def depth_image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image (depth)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'mono16')
        self.get_logger().info("Depth image received")

    def publish_camera_transform(self):
        # Create a transformation from the 'world' frame to the 'camera_link' frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'camera_link'
        
        # Set camera position and orientation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0  # Positioning camera 1 meter above the plane for a better field of view

        # Adjust quaternion for the camera to face directly along the Z-axis
        t.transform.rotation.x = 0.707  # 45 degrees rotation to face downward
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.707

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Camera transform published.")

    def publish_point_cloud(self):
        if self.color_image is None or self.depth_image is None:
            self.get_logger().warn("Waiting for color and depth images...")
            return

        # Publish the transformation
        self.publish_camera_transform()

        # Generate point cloud from depth and color images
        height, width = self.depth_image.shape
        points = []
        colors = []

        # Intrinsic camera parameters (adjust for Azure Kinect)
        fx = 600.0  # Focal length x
        fy = 600.0  # Focal length y
        cx = width / 2
        cy = height / 2
        scale_factor = 0.1  # Depth scale to meters

        # Define depth range for filtering points within expected object distances
        min_depth = 20.0  # Minimum distance for object points (in meters)
        max_depth = 200.0  # Maximum distance for object points (in meters)

        # Define Region of Interest (ROI) for object area on the table
        roi_x_start = int(width * 0.3)
        roi_x_end = int(width * 0.6)
        roi_y_start = int(height * 0.3)
        roi_y_end = int(height * 1.0)

        # Iterate through the depth image to create the point cloud
        for y in range(roi_y_start, roi_y_end):
            for x in range(roi_x_start, roi_x_end):
                depth_value = self.depth_image[y, x]
                if depth_value == 0:  # Skip invalid depth points
                    continue

                # Convert pixel (x, y) and depth to 3D point in camera space
                z3d = depth_value * scale_factor  # Convert depth to meters

                # Apply depth range filter
                if z3d < min_depth or z3d > max_depth:
                    continue  # Skip points outside the object depth range

                x3d = -(x - cx) * z3d / fx  # Negate x-axis to correct mirroring
                y3d = (y - cy) * z3d / fy

                # Append points/colors to lists
                points.append([x3d, -y3d, z3d])
                colors.append(self.color_image[y, x][:3] / 255.0)  # Normalize color values

        # Convert lists to arrays for PointCloud2 message
        points = np.array(points, dtype=np.float32)
        colors = np.array(colors, dtype=np.float32)

        # Prepare the point cloud data for ROS PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1)
        ]

        # Pack points and colors for PointCloud2 message
        point_cloud_data = []
        for point, color in zip(points, colors):
            r, g, b = (color * 255).astype(np.uint8)
            point_cloud_data.append(struct.pack('fffBBB', point[0], point[1], point[2], r, g, b))

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=15,
            row_step=15 * len(points),
            data=b''.join(point_cloud_data),
            is_dense=True
        )

        # Publish the point cloud
        self.point_cloud_publisher.publish(cloud_msg)
        self.get_logger().info("Filtered point cloud published")

def main(args=None):
    rclpy.init(args=args)
    kinect_env_node = KinectEnvNode()
    rclpy.spin(kinect_env_node)

    # Shutdown
    kinect_env_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
