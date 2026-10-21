"""Go2 profile and compatibility imports for the shared Nav2/vision adapter."""
import time
from .nav2 import Nav2Adapter
from .vision import ABCClient, ABC_RESPONSE_SCHEMA, AsyncVision, observations_from_results

def make_observer(config,read_only=False,node_name=None):
    from pathlib import Path
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from rclpy.parameter import Parameter
    from vlm_nav.vlm_navigator import VLMNavigator
    base=yaml.safe_load((Path(get_package_share_directory('vlm_nav'))/'config/robot_go2.yaml').read_text())['vlm_nav']['ros__parameters']
    base.update(enabled=False,camera_extrinsic_calibrated=config.get('camera_calibrated',False),
        publish_stop_command=not read_only,
        behavior_costmap_topic=config.get('costmap_topic','/local_costmap/costmap_raw'),
        arrival_odom_topic=config.get('odom_topic','/odometry'))
    return VLMNavigator(observation_only=True,node_name=node_name,
        parameter_overrides=[Parameter(k,value=v) for k,v in base.items()])

class Go2Adapter(Nav2Adapter):
    def readiness_checks(self,now):
        ages=[now-v for v in self.node.external_gate_receipts.values()]
        return dict(nav_ready_gate=bool(self.node.nav_ready),control_armed_gate=bool(self.node.control_armed),
            external_gates_fresh=bool(ages) and all(age<=self.node.p.external_gate_timeout for age in ages))

    def create_vision_client(self,config):
        # Preserve the historical injection/import boundary.
        return ABCClient(config)
