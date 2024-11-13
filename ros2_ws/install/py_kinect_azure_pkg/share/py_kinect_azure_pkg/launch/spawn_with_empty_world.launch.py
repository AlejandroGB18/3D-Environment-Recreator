from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al paquete de Gazebo ROS
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    # Lanzar Gazebo con un entorno vacío
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items()
    )
    
    # Nodo de ROS2 transformer_node para spawnear los objetos
    transformer_node = Node(
        package='py_kinect_azure_pkg',
        executable='transformer_node',
        output='screen'
    )
    
    # Retornar la descripción de los lanzamientos
    return LaunchDescription([
        gazebo,
        transformer_node
    ])

