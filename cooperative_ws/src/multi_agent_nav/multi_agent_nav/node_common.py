import os
from pathlib import Path
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from .config import load_config,live_gates
from .mock import mock_config

def qos(observation=False):
    return QoSProfile(depth=50 if observation else 10,reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)

def settings(node):
    node.declare_parameter('simulation',True)
    node.declare_parameter('topic_prefix','/multi_agent/mock')
    node.declare_parameter('config_dir','')
    simulation=node.get_parameter('simulation').value
    prefix=node.get_parameter('topic_prefix').value.rstrip('/')
    if not prefix.startswith('/') or not prefix:
        raise ValueError('absolute task topic prefix required')
    if simulation:
        if os.getenv('ROS_DOMAIN_ID')!='31' or os.getenv('ROS_LOCALHOST_ONLY')!='1' or not prefix.startswith('/multi_agent/mock/'):
            raise ValueError('mock requires Domain31, localhost restriction and unique /multi_agent/mock/<run_id> prefix')
        return mock_config(),simulation,prefix,[]
    if os.getenv('ROS_DOMAIN_ID','0')!='0' or prefix!='/multi_agent':
        raise ValueError('live requires Domain0 and /multi_agent task prefix')
    directory=node.get_parameter('config_dir').value
    if not directory:
        from ament_index_python.packages import get_package_share_directory
        directory=str(Path(get_package_share_directory('multi_agent_nav'))/'config')
    config=load_config(directory)
    local_robot=node.get_parameter('robot_id').value if node.has_parameter('robot_id') else 'go2'
    return config,False,prefix,live_gates(config,local_robot=local_robot)
