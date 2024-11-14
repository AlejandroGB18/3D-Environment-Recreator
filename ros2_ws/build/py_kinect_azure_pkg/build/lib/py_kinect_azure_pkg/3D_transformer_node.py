import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import String

class SimEnvNode(Node):
    def __init__(self):
        super().__init__('sim_env_node')

        # Clients for spawning and deleting models in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for Gazebo spawn service...")

        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for Gazebo delete service...")

        # Subscription to object detection topic
        self.create_subscription(String, 'yolo_detections', self.process_detections, 10)
        
        # Tracking the detection state and spawned status for each object
        self.objects = {
            "coffee": {"spawned": False, "last_detected": False},
            "apple": {"spawned": False, "last_detected": False},
            "can": {"spawned": False, "last_detected": False}
        }

    def process_detections(self, msg):
        # Iterate over each object and check if it is detected
        for obj in self.objects:
            detected = obj.capitalize() in msg.data
            state = self.objects[obj]
            self.get_logger().info(f"Processing {obj}: detected={detected}, last_detected={state['last_detected']}, spawned={state['spawned']}")

            # Spawn the object if detected and not yet spawned
            if detected and not state["spawned"]:
                self.get_logger().info(f"Detected '{obj.capitalize()}' - Spawning in Gazebo.")
                self.spawn_object_in_gazebo(obj)
                state["last_detected"] = True
            # Remove the object if not detected and it was previously spawned
            elif not detected and state["spawned"]:
                self.get_logger().info(f"No '{obj.capitalize()}' detected - Removing from Gazebo.")
                self.remove_object_from_gazebo(obj)
                state["last_detected"] = False

    def spawn_object_in_gazebo(self, obj_name):
        request = SpawnEntity.Request()
        request.name = obj_name
        request.xml = self.get_sdf_for_object(obj_name)
        request.robot_namespace = "sim_env_node"
        request.initial_pose = self.get_pose_for_object(obj_name)

        self.get_logger().info(f"Sending spawn request for {obj_name} to Gazebo...")
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda f: self.spawn_response_callback(f, obj_name))

    def spawn_response_callback(self, future, obj_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{obj_name.capitalize()} model spawned in Gazebo.")
                self.objects[obj_name]["spawned"] = True
            else:
                self.get_logger().error(f"Failed to spawn {obj_name.capitalize()} model in Gazebo.")
        except Exception as e:
            self.get_logger().error(f"Spawn request failed: {e}")

    def remove_object_from_gazebo(self, obj_name):
        request = DeleteEntity.Request()
        request.name = obj_name

        self.get_logger().info(f"Sending delete request for {obj_name} to Gazebo...")
        future = self.delete_client.call_async(request)
        future.add_done_callback(lambda f: self.delete_response_callback(f, obj_name))

    def delete_response_callback(self, future, obj_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{obj_name.capitalize()} model removed from Gazebo.")
                self.objects[obj_name]["spawned"] = False
            else:
                self.get_logger().error(f"Failed to remove {obj_name.capitalize()} model from Gazebo.")
        except Exception as e:
            self.get_logger().error(f"Delete request failed: {e}")

    def get_sdf_for_object(self, obj_name):
        # Returns the SDF content for each object
        sdf_files = {
            "coffee": """
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
            """,
            "apple": """
                <?xml version="1.0" ?>
                <sdf version="1.6">
                  <model name="apple">
                    <static>false</static>
                    <link name="link">
                      <visual name="visual">
                        <geometry>
                          <mesh>
                            <uri>model://apple/meshes/Apple.dae</uri>
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
            """,
            "can": """
                <?xml version="1.0" ?>
                <sdf version="1.6">
                  <model name="can">
                    <static>false</static>
                    <link name="link">
                      <visual name="visual">
                        <geometry>
                          <mesh>
                            <uri>model://can/meshes/Can.dae</uri>
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
        }
        return sdf_files.get(obj_name, "")

    def get_pose_for_object(self, obj_name):
        # Returns the spawn position for each object
        positions = {
            "coffee": (-5.0, 2.0, 0.63),
            "apple": (-5.2, 2.0, 0.63),
            "can": (-5.4, 2.0, 0.63)
        }
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = positions.get(obj_name, (0, 0, 0))
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = SimEnvNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
