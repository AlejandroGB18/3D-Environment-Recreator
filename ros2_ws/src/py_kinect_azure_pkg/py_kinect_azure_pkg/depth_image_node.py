import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pykinect_azure as pykinect

class DepthImageNode(Node):
    def __init__(self):
        super().__init__('depth_image_node')

        # Initialize Kinect Azure library
        pykinect.initialize_libraries()

        # Modify camera configuration
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

        # Start the device
        self.device = pykinect.start_device(config=device_config)

        # ROS publisher
        self.image_publisher = self.create_publisher(Image, 'depth_image', 10)

        # Timer to periodically capture and publish depth images
        self.timer = self.create_timer(0.1, self.publish_depth_image)

        # CV Bridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

    def publish_depth_image(self):
        # Get capture
        capture = self.device.update()

        # Get the color depth image from the capture
        ret, depth_image = capture.get_colored_depth_image()

        if not ret:
            self.get_logger().warn("Failed to capture depth image")
            return

        # Convert OpenCV image (depth_image) to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="bgr8")

        # Publish the message
        self.image_publisher.publish(image_msg)
        self.get_logger().info("Depth image published")

def main(args=None):
    rclpy.init(args=args)
    depth_image_node = DepthImageNode()
    rclpy.spin(depth_image_node)

    # Shutdown
    depth_image_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
