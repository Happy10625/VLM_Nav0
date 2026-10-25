import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    launch_dir = os.path.join(share, "launch")
    robot_params = os.path.join(share, "config", "robot.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("start_legacy_navigator", default_value="true"),
            DeclareLaunchArgument("nav2_overrides_file", default_value=""),
            DeclareLaunchArgument("target_description", default_value="chair"),
            DeclareLaunchArgument("enable_odom_adapter", default_value="true"),
            DeclareLaunchArgument("publish_camera_tf", default_value="true"),
            DeclareLaunchArgument("localization_mode", default_value="slam"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=""),
            DeclareLaunchArgument("keepout_mask_digest", default_value=""),
            DeclareLaunchArgument("amcl_scan_topic", default_value="/scan"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "mapping.launch.py")
                ),
                launch_arguments={
                    "localization_mode": LaunchConfiguration("localization_mode"),
                    "map": LaunchConfiguration("map"),
                    "amcl_scan_topic": LaunchConfiguration("amcl_scan_topic"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation.launch.py")
                ),
                launch_arguments={
                    "localization_mode": LaunchConfiguration("localization_mode"),
                    "overrides_file": LaunchConfiguration("nav2_overrides_file"),
                    "map": LaunchConfiguration("map"),
                    "keepout_mask_yaml": LaunchConfiguration("keepout_mask_yaml"),
                    "keepout_mask_digest": LaunchConfiguration("keepout_mask_digest"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "vlm_navigation.launch.py")
                ),
                launch_arguments={
                    "params_file": robot_params,
                    "enabled": LaunchConfiguration("enabled"),
                    "start_legacy_navigator": LaunchConfiguration("start_legacy_navigator"),
                    "target_description": LaunchConfiguration("target_description"),
                    "enable_odom_adapter": LaunchConfiguration("enable_odom_adapter"),
                    "publish_camera_tf": LaunchConfiguration("publish_camera_tf"),
                    "scan_first": "true",
                    "easy_case_mode": "false",
                    "allow_frontier_after_scan": PythonExpression(
                        ["'", LaunchConfiguration("localization_mode"), "' != 'amcl'"]
                    ),
                }.items(),
            ),
        ]
    )
