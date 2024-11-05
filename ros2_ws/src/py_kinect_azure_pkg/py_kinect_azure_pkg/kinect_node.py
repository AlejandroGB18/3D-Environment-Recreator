import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pykinect_azure as pykinect

class KinectNode(Node):
    def __init__(self):
        super().__init__('kinect_node')
        
        # Initialize the Kinect library
        pykinect.initialize_libraries()

        # Configure the device
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        
        # Start the device
        self.device = pykinect.start_device(config=device_config)

        # Create a publisher for color images
        self.image_publisher = self.create_publisher(Image, 'color_image', 10)

        # Create a timer to call the capture method at a set frequency
        self.timer = self.create_timer(0.1, self.capture)

        # Create a CvBridge object to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

    def capture(self):
        # Get capture
        capture = self.device.update()

        # Get the color image from the capture
        ret, color_image = capture.get_color_image()

        if ret:
            # Convert the image to a ROS message
            ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            self.image_publisher.publish(ros_image)
            self.get_logger().info('Publishing color image')

def main(args=None):
    rclpy.init(args=args)
    kinect_node = KinectNode()
    rclpy.spin(kinect_node)
    kinect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
