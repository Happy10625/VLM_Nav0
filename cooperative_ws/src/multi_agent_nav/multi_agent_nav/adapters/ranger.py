"""Ranger uses existing aligned RGB-D, Nav2 and chassis velocity feedback."""
from pathlib import Path
from concurrent.futures import Future
import yaml
from ..models import ApproachResult, Pose2D
from .nav2 import Nav2Adapter


def observer_parameters(config,read_only=False):
    path=config.get('observer_config_file')
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path=Path(get_package_share_directory('vlm_nav'))/'config/robot.yaml'
    base=yaml.safe_load(Path(path).read_text())['vlm_nav']['ros__parameters']
    base.update(enabled=False,raw_depth_mode=False,require_external_safety_gates=False,
        publish_stop_command=not read_only,camera_extrinsic_calibrated=config.get('camera_calibrated',False),
        behavior_costmap_topic=config['costmap_topic'],arrival_odom_topic=config['odom_topic'],
        stop_cmd_topic='/cmd_vel_nav')
    return base


def make_observer(config,read_only=False,node_name=None):
    from rclpy.parameter import Parameter
    from vlm_nav.vlm_navigator import VLMNavigator
    return VLMNavigator(observation_only=True,node_name=node_name or 'multi_agent_ranger_rgbd',
        parameter_overrides=[Parameter(k,value=v) for k,v in observer_parameters(config,read_only).items()])


class RangerAdapter(Nav2Adapter):
    def select_approach(self,local_object,excluded_candidates,*,standoff_distance=None,path_accept=None):
        """Plan to the object; Nav2's tolerance chooses the reachable endpoint."""
        output=Future()
        goal=Pose2D(local_object.x,local_object.y,self.get_health().pose.yaw)
        if not goal.valid or goal in excluded_candidates or self.costmap_transform() is None:
            output.set_result(ApproachResult(reason='target reference/map unavailable'))
            return output
        def planned(done):
            if output.done():return
            try:
                result=done.result()
                accepted=result.reachable and bool(result.path) and all(p.valid for p in result.path)
                if accepted:
                    endpoint=result.path[-1]
                    accepted=(self.costmap_candidate_free(endpoint,allow_outside=self.global_goal_checks) and
                        (not self.global_goal_checks or self.costmap_candidate_free(endpoint,global_goal=True)) and
                        (path_accept is None or path_accept(result)))
                output.set_result(ApproachResult(goal if accepted else None,
                    '' if accepted else result.reason or 'target reference path rejected'))
            except Exception as error:output.set_exception(error)
        self.compute_path(goal).add_done_callback(planned)
        return output

    def readiness_checks(self,now):
        """Ranger needs no extra permission gate; shared Nav2 health still applies."""
        return {}
