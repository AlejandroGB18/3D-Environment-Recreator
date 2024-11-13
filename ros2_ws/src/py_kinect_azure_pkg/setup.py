from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_kinect_azure_pkg'

setup(
    name=package_name,
    version='0.0.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de lanzamiento
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pykinect_azure==0.0.4'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alejandro@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'kinect_node = py_kinect_azure_pkg.kinect_node:main',
        'point_cloud_node = py_kinect_azure_pkg.point_cloud_node:main',
        'depth_image_node = py_kinect_azure_pkg.depth_image_node:main',
        'kinect_image_node = py_kinect_azure_pkg.kinect_image_node:main',
        'point_cloud_node_2 = py_kinect_azure_pkg.kinect_point_cloud_node:main',
        'point_cloud_node_2_1 = py_kinect_azure_pkg.kpc_node_2:main',
        '3D_node = py_kinect_azure_pkg.first_3D_node:main',
        'env_node = py_kinect_azure_pkg.env_node:main',
        'yolo_node = py_kinect_azure_pkg.yolo_node:main',
        'motion_detection = py_kinect_azure_pkg.test_people:main',
        'manager_node = py_kinect_azure_pkg.manager_node:main',
        'transformer_node = py_kinect_azure_pkg.3D_transformer_node:main'
        ],
    },
)
