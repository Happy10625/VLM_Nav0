"""Start mapping, empty-space Nav2, and the opt-in VLM easy-case controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    launch_dir = os.path.join(share, "launch")
    robot_params = os.path.join(share, "config", "robot.yaml")
    easy_nav2 = os.path.join(share, "config", "nav2_easy_case.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("target_description", default_value="chair"),
            DeclareLaunchArgument("enable_odom_adapter", default_value="true"),
            DeclareLaunchArgument("publish_camera_tf", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "mapping.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation.launch.py")
                ),
                launch_arguments={"overrides_file": easy_nav2}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "vlm_navigation.launch.py")
                ),
                launch_arguments={
                    "params_file": robot_params,
                    "enabled": LaunchConfiguration("enabled"),
                    "target_description": LaunchConfiguration(
                        "target_description"
                    ),
                    "easy_case_mode": "true",
                    "enable_odom_adapter": LaunchConfiguration(
                        "enable_odom_adapter"
                    ),
                    "publish_camera_tf": LaunchConfiguration(
                        "publish_camera_tf"
                    ),
                }.items(),
            ),
        ]
    )
