"""Ranger uses existing aligned RGB-D, Nav2 and chassis velocity feedback."""
from pathlib import Path
import yaml
from .nav2 import Nav2Adapter


def observer_parameters(config,read_only=False):
    path=config.get('observer_config_file')
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path=Path(get_package_share_directory('vlm_nav'))/'config/robot.yaml'
    base=yaml.safe_load(Path(path).read_text())['vlm_nav']['ros__parameters']
    base.update(enabled=False,raw_depth_mode=False,require_external_safety_gates=False,
        publish_stop_command=not read_only,camera_extrinsic_calibrated=config.get('camera_calibrated',False),
        behavior_costmap_topic=config['costmap_topic'],arrival_odom_topic=config['odom_topic'])
    return base


def make_observer(config,read_only=False,node_name=None):
    from rclpy.parameter import Parameter
    from vlm_nav.vlm_navigator import VLMNavigator
    return VLMNavigator(observation_only=True,node_name=node_name or 'multi_agent_ranger_rgbd',
        parameter_overrides=[Parameter(k,value=v) for k,v in observer_parameters(config,read_only).items()])


class RangerAdapter(Nav2Adapter):
    def readiness_checks(self,now):
        """Ranger needs no extra permission gate; shared Nav2 health still applies."""
        return {}
