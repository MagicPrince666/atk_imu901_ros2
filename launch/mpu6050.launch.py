import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo

import lifecycle_msgs.msg

def generate_launch_description():

    imu_cfg = os.path.join(get_package_share_directory('ros2_imu'), 'params', 'mpu6050.yaml')

    return LaunchDescription([
        Node(namespace='/', package='ros2_imu', executable='ros2_imu_node', name='ros2_imu_node', parameters=[imu_cfg], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','imu_link']),
    ])
