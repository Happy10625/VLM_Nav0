import uuid
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    prefix='/multi_agent/mock/run_'+uuid.uuid4().hex[:12]
    params={'simulation':True,'topic_prefix':prefix}
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID','31'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY','1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI',''),
        Node(package='multi_agent_nav',executable='multi_agent_coordinator_node',
             name='multi_agent_coordinator',parameters=[params],output='screen'),
        Node(package='multi_agent_nav',executable='multi_agent_agent_node',
             name='multi_agent_go2_agent',parameters=[params,{'robot_id':'go2'}],output='screen'),
        Node(package='multi_agent_nav',executable='multi_agent_agent_node',
             name='multi_agent_ranger_agent',parameters=[params,{'robot_id':'ranger'}],output='screen')])
