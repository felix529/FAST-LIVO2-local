#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fast_livo")
    default_mapping_config = os.path.join(package_share, "config", "mid360.yaml")
    default_camera_config = os.path.join(package_share, "config", "camera_fisheye_ORLACO.yaml")
    rviz_config = os.path.join(package_share, "rviz_cfg", "fast_livo2.rviz")

    mapping_params_file = LaunchConfiguration("mapping_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    mapping_prefix = LaunchConfiguration("mapping_prefix")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
            description="Whether to launch RViz2",
        ),
        DeclareLaunchArgument(
            "mapping_params_file",
            default_value=default_mapping_config,
            description="Full path to the Mid-360 mapping parameter file",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=default_camera_config,
            description="Full path to the camera parameter file",
        ),
        DeclareLaunchArgument(
            "mapping_prefix",
            default_value="",
            description="Optional command prefix, for example valgrind",
        ),
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[mapping_params_file, camera_params_file],
            prefix=mapping_prefix,
            output="screen",
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
