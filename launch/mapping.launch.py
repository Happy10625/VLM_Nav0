import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    return LaunchDescription(
        [
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="fastlio_cloud_to_scan",
                output="screen",
                remappings=[
                    ("cloud_in", "/vlm_nav/obstacle_cloud"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "base_link",
                        "transform_tolerance": 0.10,
                        "min_height": 0.05,
                        "max_height": 1.50,
                        "angle_min": -3.141592653589793,
                        "angle_max": 3.141592653589793,
                        "angle_increment": 0.008726646259972,
                        "scan_time": 0.10,
                        "range_min": 0.50,
                        "range_max": 20.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[os.path.join(share, "config", "slam_toolbox.yaml")],
            ),
        ]
    )
