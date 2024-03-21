"""Laser Avoidance Launch File."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""
    laser_Avoidance_node = Node(
        package="yahboomcar_laser",
        executable="laser_Avoidance_a1_X3",
    )
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("sllidar_ros2"), "launch"), "/sllidar_s2_launch.py"]
        )
    )
    bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("yahboomcar_bringup"), "launch"),
                "/yahboomcar_bringup_X3_launch.py",
            ]
        )
    )
    return LaunchDescription([laser_Avoidance_node, lidar_node, bringup_node])
