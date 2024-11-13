import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.srv import SpawnEntity
import open3d as o3d
import numpy as np
import ros2_numpy as r2n
from scipy.spatial.transform import Rotation as R

class SimEnvNode(Node):
    def __init__(self):
        super().__init__('sim_env_node')
        
        # Wait for the Gazebo spawn service to be ready
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for the Gazebo spawn service to be ready...")
        
        # Subscribe to YOLO detection and point cloud topics
        self.create_subscription(String, 'yolo_detections_info', self.process_detections, 10)
        self.create_subscription(PointCloud2, 'scene_point_cloud', self.update_point_cloud, 10)
        
        # Variables for storing object poses and point cloud data
        self.object_poses = {}
        self.current_point_cloud = None
        self.point_cloud_width = None
        self.point_cloud_height = None
        
        # Dimensions of the detection image
        self.image_width = 640
        self.image_height = 480
        
        # Load and preprocess canonical models
        self.canonical_models = {
            'can': self.load_and_preprocess_canonical_model("/home/alejandro/ros2_ws/src/py_kinect_azure_pkg/py_kinect_azure_pkg/canon_models/Can.ply"),
            'coffee': self.load_and_preprocess_canonical_model("/home/alejandro/ros2_ws/src/py_kinect_azure_pkg/py_kinect_azure_pkg/canon_models/Coffee.ply"),
            'apple': self.load_and_preprocess_canonical_model("/home/alejandro/ros2_ws/src/py_kinect_azure_pkg/py_kinect_azure_pkg/canon_models/Apple.ply")
        }

    def load_and_preprocess_canonical_model(self, model_path):
        try:
            model = o3d.io.read_triangle_mesh(model_path)
            if model.is_empty():
                raise ValueError("The file is empty or could not be loaded as a mesh.")
        except Exception as e:
            self.get_logger().warn(f"Failed to load file {model_path} as TriangleMesh: {e}")
            model = o3d.io.read_point_cloud(model_path)
            if model.is_empty():
                self.get_logger().error(f"The file {model_path} is empty or could not be loaded.")
                return None
        
        model = self.center_and_scale_model(model)
        return model

    def center_and_scale_model(self, model, scale_factor=0.001):
        points = model.vertices if hasattr(model, 'vertices') else model.points
        centroid = np.mean(np.asarray(points), axis=0)
        model.translate(-centroid)
        model.scale(scale_factor, center=(0, 0, 0))
        return model

    def process_detections(self, msg):
        detection_data = msg.data.strip().split('\n')
        for detection in detection_data:
            if detection:
                parsed_data = self.parse_detection(detection)
                if parsed_data is None:
                    continue
                label, confidence, x1, y1, x2, y2 = parsed_data
                centroid = self.get_centroid(x1, y1, x2, y2)
                pose = self.calculate_pose_from_centroid(centroid, label)
                
                if pose is None:
                    self.get_logger().warn(f"Could not calculate pose for object '{label}'.")
                    continue
                
                self.object_poses[label] = pose
                self.spawn_ball_in_gazebo(label, pose)

    def parse_detection(self, detection):
        elements = detection.split(',')

        if len(elements) < 3:
            self.get_logger().error(f"Unexpected message format: {detection}")
            return None

        try:
            label = elements[0].split(":")[1].strip()
            confidence = float(elements[1].split(":")[1].strip())
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error extracting label or confidence in message: {detection} - {e}")
            return None

        try:
            coords_text = detection.split("Coordinates:")[-1].strip()
            self.get_logger().info(f"Extracted coordinates (text): '{coords_text}'")
            coords = [int(value.strip()) for value in coords_text.split(',')]
            self.get_logger().info(f"Converted coordinates: {coords}")
            if len(coords) != 4:
                raise ValueError("Incorrect number of coordinates")
            x1, y1, x2, y2 = coords
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error extracting coordinates in message: {detection} - {e}")
            return None

        return label, confidence, x1, y1, x2, y2

    def get_centroid(self, x1, y1, x2, y2):
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        return int(cx), int(cy)

    def update_point_cloud(self, msg):
        self.current_point_cloud = r2n.point_cloud2.point_cloud2_to_array(msg)
        self.point_cloud_width = msg.width
        self.point_cloud_height = msg.height

    def calculate_pose_from_centroid(self, centroid, label):
        if self.current_point_cloud is None:
            self.get_logger().warn("No point cloud data available.")
            return None

        cx, cy = centroid

        # Scale the centroid coordinates to match the point cloud dimensions
        cx_scaled = int(cx * self.point_cloud_width / self.image_width)
        cy_scaled = int(cy * self.point_cloud_height / self.image_height)

        # Check if the coordinates are within bounds
        if not (0 <= cy_scaled < self.point_cloud_height and 0 <= cx_scaled < self.point_cloud_width):
            self.get_logger().error(f"Adjusted point cloud coordinates out of bounds: ({cx_scaled}, {cy_scaled})")
            return None

        # Calculate the linear index for accessing the point cloud depth
        index = cy_scaled * self.point_cloud_width + cx_scaled
        if index >= len(self.current_point_cloud):
            self.get_logger().error(f"Point cloud index out of bounds: {index}")
            return None

        # Access the depth information from the point cloud
        depth = self.current_point_cloud[index]['z']

        pose = Pose()
        pose.position.x = cx_scaled * 0.001  # Adjust as needed
        pose.position.y = cy_scaled * 0.001  # Adjust as needed
        pose.position.z = depth

        if label in self.canonical_models:
            transformed_model, transformation = self.align_model_to_point_cloud(label)
            rotation_matrix = transformation[:3, :3]
            quat = self.matrix_to_quaternion(rotation_matrix)
            pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        return pose

    def align_model_to_point_cloud(self, label):
        if self.current_point_cloud is None:
            self.get_logger().warn("No point cloud data available for alignment.")
            return None, None

        source = self.canonical_models[label]
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(self.current_point_cloud)

        threshold = 0.02
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        return reg_p2p.transformation, reg_p2p.transformation

    def matrix_to_quaternion(self, rotation_matrix):
        r = R.from_matrix(rotation_matrix)
        return r.as_quat()

    def spawn_ball_in_gazebo(self, label, pose):
        """
        Spawns a ball in Gazebo using the SpawnEntity service.
        """
        if pose is None:
            self.get_logger().warn(f"Could not calculate pose for object '{label}'.")
            return

        if not self.spawn_client.service_is_ready():
            self.get_logger().warn("The Gazebo spawn service is not ready.")
            return

        model_name = 'ball_model'
        model_path = f"model://{model_name}"

        request = SpawnEntity.Request()
        request.name = label
        request.xml = f"""
            <sdf version='1.6'>
              <model name='{label}'>
                <pose>{pose.position.x} {pose.position.y} {pose.position.z} 0 0 0</pose>
                <link name='link'>
                  <visual name='visual'>
                    <geometry>
                      <mesh>
                        <uri>{model_path}</uri>
                      </mesh>
                    </geometry>
                  </visual>
                </link>
              </model>
            </sdf>
        """

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Model '{label}' spawned in Gazebo.")
        else:
            self.get_logger().error(f"Failed to spawn '{label}' in Gazebo.")

def main(args=None):
    rclpy.init(args=args)
    node = SimEnvNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
