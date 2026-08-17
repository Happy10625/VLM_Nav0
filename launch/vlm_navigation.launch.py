import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    params = os.path.join(share, "config", "robot.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=params),
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("target_description", default_value="chair"),
            DeclareLaunchArgument("easy_case_mode", default_value="false"),
            DeclareLaunchArgument("enable_odom_adapter", default_value="true"),
            DeclareLaunchArgument("publish_camera_tf", default_value="true"),
            Node(
                package="vlm_nav",
                executable="fastlio_odom_adapter",
                name="fastlio_odom_adapter",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
                condition=IfCondition(LaunchConfiguration("enable_odom_adapter")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_fastlio_world",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
                condition=IfCondition(LaunchConfiguration("enable_odom_adapter")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera",
                arguments=[
                    "--x",
                    "-0.20",
                    "--y",
                    "0.0",
                    "--z",
                    "1.215",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "camera_link",
                ],
                condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
            ),
            Node(
                package="vlm_nav",
                executable="obstacle_cloud_filter",
                name="obstacle_cloud_filter",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="vlm_nav",
                executable="vlm_navigator",
                name="vlm_nav",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "enabled": LaunchConfiguration("enabled"),
                        "target_description": LaunchConfiguration("target_description"),
                        "easy_case_mode": LaunchConfiguration("easy_case_mode"),
                    },
                ],
            ),
        ]
    )
