import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import requests
import numpy as np

class KinectYoloNode(Node):
    def __init__(self):
        super().__init__('kinect_yolo_node')
        
        # Initialize publishers
        self.detections_publisher = self.create_publisher(Image, 'yolo_detections_image', 10)
        self.detections_info_publisher = self.create_publisher(String, 'yolo_detections_info', 10)
        
        # Initialize subscription to shared color image
        self.create_subscription(Image, '/shared/color_image', self.image_callback, 10)

        # Roboflow inference setup
        self.roboflow_url = "https://detect.roboflow.com/3d_identifier_v5/1?api_key=ObAq2LOm6KmsxcEXtSqm"
        self.bridge = CvBridge()

    def image_callback(self, msg):
        color_image_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Encode and send image for inference
        _, img_encoded = cv2.imencode('.jpg', color_image_bgr)
        img_bytes = img_encoded.tobytes()
        
        try:
            response = requests.post(self.roboflow_url, files={"file": img_bytes})
            response.raise_for_status()
            predictions = response.json()

            # Process predictions
            detections = ""
            for obj in predictions["predictions"]:
                x, y, w, h = obj["x"], obj["y"], obj["width"], obj["height"]
                x1, y1 = int(x - w / 2), int(y - h / 2)
                x2, y2 = int(x + w / 2), int(y + h / 2)
                label = obj["class"]
                confidence = obj["confidence"]
                
                cv2.rectangle(color_image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image_bgr, f"{label} {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                detections += f"Object: {label}, Confidence: {confidence:.2f}, Coordinates: {x1}, {y1}, {x2}, {y2}\n"

            detection_image_msg = self.bridge.cv2_to_imgmsg(color_image_bgr, encoding="bgr8")
            self.detections_publisher.publish(detection_image_msg)
            self.detections_info_publisher.publish(String(data=detections))

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Roboflow inference error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KinectYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
