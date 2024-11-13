import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # Import YOLO from ultralytics

class KinectYoloNode(Node):
    def __init__(self):
        super().__init__('kinect_yolo_node')
        
        # Initialize YOLOv8 model using best.pt
        self.model = YOLO('/home/alejandro/best.pt')  

        # Initialize publishers
        self.detections_publisher = self.create_publisher(Image, 'yolo_detections_image', 10)
        self.detections_info_publisher = self.create_publisher(String, 'yolo_detections_info', 10)
        self.detections_publisher_object = self.create_publisher(String, 'yolo_detections', 10)
        
        # Initialize subscription to shared color image
        self.create_subscription(Image, '/shared/color_image', self.image_callback, 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        color_image_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run inference directly on the image using best.pt
        results = self.model(color_image_bgr)

        # Process predictions
        detect = ""
        detections = ""
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                label = self.model.names[int(box.cls)]
                confidence = box.conf.item()
                
                cv2.rectangle(color_image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image_bgr, f"{label} {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                detections += f"Object: {label}, Confidence: {confidence:.2f}, Coordinates: {x1}, {y1}, {x2}, {y2}\n"
                detect = f"{label(0)},{label(1)}"

        detection_image_msg = self.bridge.cv2_to_imgmsg(color_image_bgr, encoding="bgr8")
        self.detections_publisher.publish(detection_image_msg)
        self.detections_info_publisher.publish(String(data=detections))
        self.detections_publisher_object.publish(String(data=detect))

def main(args=None):
    rclpy.init(args=args)
    node = KinectYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
