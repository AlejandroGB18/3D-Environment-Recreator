from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Launch manager_node
        Node(
            package='py_kinect_azure_pkg',
            executable='manager_node',
            name='manager_node',
            output='screen'
        ),
        # Launch env_node
        Node(
            package='py_kinect_azure_pkg',
            executable='env_node',
            name='env_node',
            output='screen'
        ),
        # Launch yolo_node
        Node(
            package='py_kinect_azure_pkg',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),
        # Launch transformer_node
        Node(
            package='py_kinect_azure_pkg',
            executable='transformer_node',
            name='transformer_node',
            output='screen'
        ),
        # Launch rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/alejandro/ros2_ws/src/py_kinect_azure_pkg/config/rviz_config.rviz'],
            output='screen'
        )

    ])
