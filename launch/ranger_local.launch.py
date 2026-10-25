"""Existing Ranger map/TF/Nav2 only; no legacy loop, agent, bridge or task."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=get_package_share_directory('vlm_nav')
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('keepout_mask_yaml',default_value=''),
        DeclareLaunchArgument('keepout_mask_digest',default_value=''),
        DeclareLaunchArgument('enable_odom_adapter',default_value='true'),
        DeclareLaunchArgument('publish_camera_tf',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','system.launch.py')),
            launch_arguments=dict(map=LaunchConfiguration('map'),localization_mode='amcl',
                keepout_mask_yaml=LaunchConfiguration('keepout_mask_yaml'),
                keepout_mask_digest=LaunchConfiguration('keepout_mask_digest'),
                enabled='false',start_legacy_navigator='false',
                enable_odom_adapter=LaunchConfiguration('enable_odom_adapter'),
                publish_camera_tf=LaunchConfiguration('publish_camera_tf'),
                nav2_overrides_file=os.path.join(share,'config','nav2_ranger_local.yaml')).items())])
