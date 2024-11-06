import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pykinect_azure as pykinect
import cv2

class KinectManagerNode(Node):
    def __init__(self):
        super().__init__('kinect_manager')
        pykinect.initialize_libraries()

        # Kinect configuration
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

        self.device = pykinect.start_device(config=device_config)
        self.bridge = CvBridge()

        # Publishers for shared color and depth images
        self.color_image_publisher = self.create_publisher(Image, '/shared/color_image', 10)
        self.depth_image_publisher = self.create_publisher(Image, '/shared/depth_image', 10)

        # Timer to capture and publish images periodically
        self.timer = self.create_timer(0.1, self.publish_images)

    def publish_images(self):
        capture = self.device.update()

        # Capture color and depth images
        ret_color, color_image = capture.get_color_image()
        ret_depth, depth_image = capture.get_depth_image()

        if ret_color:
            color_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR), encoding='bgr8')
            self.color_image_publisher.publish(color_msg)
        
        if ret_depth:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='mono16')
            self.depth_image_publisher.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    kinect_manager_node = KinectManagerNode()
    rclpy.spin(kinect_manager_node)
    kinect_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
