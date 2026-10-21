"""Attach Ranger's task agent to its existing local perception and Nav2 stack."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    return [Node(package='multi_agent_nav',executable='multi_agent_agent_node',
        name='multi_agent_ranger_agent',output='screen',parameters=[dict(robot_id='ranger',
            simulation=False,topic_prefix='/multi_agent',
            config_dir=LaunchConfiguration('config_dir').perform(context))])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir',default_value=str(Path(get_package_share_directory('multi_agent_nav'))/'config')),
        SetEnvironmentVariable('ROS_DOMAIN_ID','0'),
        OpaqueFunction(function=build)])
