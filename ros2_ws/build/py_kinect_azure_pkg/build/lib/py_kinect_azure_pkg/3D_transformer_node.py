import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import String

class SimEnvNode(Node):
    def __init__(self):
        super().__init__('sim_env_node')

        # Client for spawning models in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for Gazebo spawn service...")

        # Client for deleting models in Gazebo
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for Gazebo delete service...")

        # Subscription to object detection topic
        self.create_subscription(String, 'yolo_detections', self.process_detections, 10)
        
        # Track the spawned object state and last detection state
        self.object_spawned = False
        self.last_detected = False  # Flag to remember the last detection state

    def process_detections(self, msg):
        # Parse detection state based on message content
        detected = 'Coffee' in msg.data
        self.get_logger().info(f"Processing detection message: detected={detected}, last_detected={self.last_detected}, object_spawned={self.object_spawned}")

        # If we detect "Coffee" and it hasn't been spawned, spawn it
        if detected and not self.object_spawned:
            self.get_logger().info("Detected 'Coffee' - Spawning in Gazebo.")
            self.spawn_coffee_in_gazebo()
            self.last_detected = True  # Mark detection as active
        # If no detection and object is spawned, then remove it
        elif not detected and self.object_spawned:
            self.get_logger().info("No 'Coffee' detected - Removing from Gazebo.")
            self.remove_coffee_from_gazebo()
            self.last_detected = False  # Mark detection as inactive

    def spawn_coffee_in_gazebo(self):
        request = SpawnEntity.Request()
        request.name = "coffee"
        request.xml = """
            <?xml version="1.0" ?>
            <sdf version="1.6">
              <model name="coffee">
                <static>false</static>
                <link name="link">
                  <visual name="visual">
                    <geometry>
                      <mesh>
                        <uri>model://coffee/meshes/Coffee.dae</uri>
                        <scale>1 1 1</scale> 
                      </mesh>
                    </geometry>
                  </visual>
                  <collision name="collision">
                    <geometry>
                      <box>
                        <size>0.1 0.1 0.1</size> 
                      </box>
                    </geometry>
                  </collision>
                </link>
              </model>
            </sdf>
        """
        request.robot_namespace = "sim_env_node"

        # Position for the Coffee model on the table
        request.initial_pose = Pose()
        request.initial_pose.position.x = 1.0
        request.initial_pose.position.y = 0.5
        request.initial_pose.position.z = 0.0

        self.get_logger().info("Sending spawn request to Gazebo...")
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_response_callback)

    def spawn_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Coffee model spawned in Gazebo.")
                self.object_spawned = True  # Mark object as spawned
            else:
                self.get_logger().error("Failed to spawn Coffee model in Gazebo.")
        except Exception as e:
            self.get_logger().error(f"Spawn request failed: {e}")

    def remove_coffee_from_gazebo(self):
        request = DeleteEntity.Request()
        request.name = "coffee"

        self.get_logger().info("Sending delete request to Gazebo...")
        future = self.delete_client.call_async(request)
        future.add_done_callback(self.delete_response_callback)

    def delete_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Coffee model removed from Gazebo.")
                self.object_spawned = False  # Mark object as removed
            else:
                self.get_logger().error("Failed to remove Coffee model from Gazebo.")
        except Exception as e:
            self.get_logger().error(f"Delete request failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimEnvNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
