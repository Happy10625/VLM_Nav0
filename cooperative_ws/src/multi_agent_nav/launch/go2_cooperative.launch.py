from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def build(context):
    directory=LaunchConfiguration('config_dir').perform(context)
    params={'simulation':False,'topic_prefix':'/multi_agent','config_dir':directory}
    return [Node(package='multi_agent_nav',executable='multi_agent_coordinator_node',
                 name='multi_agent_coordinator',parameters=[params],output='screen'),
            Node(package='multi_agent_nav',executable='multi_agent_agent_node',
                 name='multi_agent_go2_agent',parameters=[params,{'robot_id':'go2'}],output='screen')]

def generate_launch_description():
    vlm=Path(get_package_share_directory('vlm_nav'))
    return LaunchDescription([
        DeclareLaunchArgument('config_dir',default_value=str(Path(get_package_share_directory('multi_agent_nav'))/'config')),
        SetEnvironmentVariable('ROS_DOMAIN_ID','0'),
        SetEnvironmentVariable('GO2_NET_IFACE','enp4s0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY','0'),
        SetEnvironmentVariable('CYCLONEDDS_URI','file://'+str(vlm/'config/cyclonedds_go2.xml')),
        OpaqueFunction(function=build)])
