import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pykinect_azure as pykinect
from pykinect_azure import K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH, k4a_float2_t

class KinectImageNode(Node):
    def __init__(self):
        super().__init__('kinect_image_node')

        # Initialize Kinect Azure library
        pykinect.initialize_libraries()

        # Modify camera configuration
        device_config = pykinect.default_configuration
        device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

        # Start the device
        self.device = pykinect.start_device(config=device_config)

        # ROS publisher for color image and depth data
        self.color_image_publisher = self.create_publisher(Image, 'color_image', 10)
        self.depth_info_publisher = self.create_publisher(Image, 'depth_image', 10)

        # Timer to periodically capture and publish data
        self.timer = self.create_timer(0.1, self.publish_images)

        # CV Bridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

    def publish_images(self):
        # Get capture
        capture = self.device.update()

        # Get the color image from the capture
        ret_color, color_image = capture.get_color_image()

        # Get the transformed depth image
        ret_depth, transformed_depth_image = capture.get_transformed_depth_image()

        if not ret_color or not ret_depth:
            self.get_logger().warn("Failed to capture images")
            return

        # Convert color and depth OpenCV images to ROS Image messages
        color_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgra8")
        depth_image_msg = self.bridge.cv2_to_imgmsg(transformed_depth_image, encoding="mono16")

        # Publish the images
        self.color_image_publisher.publish(color_image_msg)
        self.depth_info_publisher.publish(depth_image_msg)
        self.get_logger().info("Published color and depth images")

        # Extract 3D point information
        pix_x = color_image.shape[1] // 2
        pix_y = color_image.shape[0] // 2
        rgb_depth = transformed_depth_image[pix_y, pix_x]
        pixels = k4a_float2_t((pix_x, pix_y))

        pos3d_color = self.device.calibration.convert_2d_to_3d(
            pixels, rgb_depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR
        )
        pos3d_depth = self.device.calibration.convert_2d_to_3d(
            pixels, rgb_depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH
        )

        self.get_logger().info(f"RGB Depth: {rgb_depth}, RGB 3D Pos: {pos3d_color}, Depth 3D Pos: {pos3d_depth}")


def main(args=None):
    rclpy.init(args=args)
    kinect_image_node = KinectImageNode()
    rclpy.spin(kinect_image_node)

    # Shutdown
    kinect_image_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
