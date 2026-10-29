"""Ranger uses existing aligned RGB-D, Nav2 and chassis velocity feedback."""
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import Future
from dataclasses import replace
import json
import time
import yaml
from ..models import ApproachResult, Pose2D
from .nav2 import Nav2Adapter


def observer_parameters(config,read_only=False):
    path=config.get('observer_config_file')
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path=Path(get_package_share_directory('vlm_nav'))/'config/robot.yaml'
    base=yaml.safe_load(Path(path).read_text())['vlm_nav']['ros__parameters']
    for arrival_path in config.get('arrival_config_files',[]):
        arrival=yaml.safe_load(Path(arrival_path).read_text())['vlm_nav']['ros__parameters']
        base.update({k:v for k,v in arrival.items() if k.startswith('arrival_') or
            k in ('target_success_radius','approach_cancel_radius')})
    base.update(enabled=False,raw_depth_mode=False,require_external_safety_gates=False,
        publish_stop_command=not read_only,camera_extrinsic_calibrated=config.get('camera_calibrated',False),
        behavior_costmap_topic=config['costmap_topic'],
        stop_cmd_topic='/cmd_vel_nav')
    return base


def make_observer(config,read_only=False,node_name=None):
    from rclpy.parameter import Parameter
    from vlm_nav.ranger_single_robot_execution import RangerSingleRobotExecution
    return RangerSingleRobotExecution(observation_only=True,node_name=node_name or 'multi_agent_ranger_rgbd',
        parameter_overrides=[Parameter(k,value=v) for k,v in observer_parameters(config,read_only).items()])


class RangerAdapter(Nav2Adapter):
    def approach_steps(self,target,task_id,command_id,kind,record,clock=time.monotonic,check_motion=None):
        from ..single_robot_approach import SingleRobotApproach, load_parameters, SUCCEEDED, FAILED
        parameters=load_parameters(self.config.get('arrival_config_files'),robot_id='ranger')
        if parameters.arrival_odom_topic != self.node.p.arrival_odom_topic:
            raise ValueError('approach and observer arrival odometry topics differ')
        self.local_arrival_verified=False
        self.completed_approach=None
        approach=SingleRobotApproach(self,target,task_id,command_id,kind,parameters,record,clock)
        self.approach=approach
        self.approach_guard=check_motion
        try:
            while approach.state not in (SUCCEEDED,FAILED):
                approach.tick()
                yield
            if approach.state==FAILED:
                # Only a confirmed stationary distance miss restarts perception.
                if (approach.reason=='Approach stopped outside target distance contract after retry'
                        and self.node.execution_active):
                    approach.emit('APPROACH_REOBSERVE',reason=approach.reason,
                        measured_distance=approach.target_reference_distance(approach.robot_pose()))
                    self.node.reset_task('SCANNING')
                    return 'RESUME_SEARCH'
                return 'APPROACH_FAILED: '+approach.reason
            self.local_arrival_verified=True
            self.completed_approach=approach
            approach.emit('TARGET_APPROACHED',
                measured_distance=approach.target_reference_distance(approach.robot_pose()),
                arrival_retry=approach.arrival_retry_count)
            return 'TARGET_APPROACHED'
        finally:
            if approach.state!=SUCCEEDED:approach.close()
            self.approach=None
            self.approach_guard=None

    def start_approach_navigation(self,approach,local_goal,token):
        from nav2_msgs.action import NavigateToPose
        if self.approach_guard is not None:self.approach_guard()
        if self.motions or self.scan:raise RuntimeError('motion not serial')
        goal=NavigateToPose.Goal()
        goal.pose=self.node.make_pose_stamped(local_goal.x,local_goal.y,local_goal.yaw)
        self._clear_path('new approach navigation')
        self.path_after_stamp=self.node.get_clock().now().nanoseconds*1e-9
        def accepted(handle,goal_id):
            approach.goal_id=goal_id
            response=Future();response.set_result(handle)
            approach.on_goal_response(response,token,'approach')
        def completed(status,goal_id):
            result=Future();result.set_result(SimpleNamespace(status=status))
            approach.on_navigation_result(result,token,'approach')
        return self._motion(self.node.navigator,goal,approach.task_id,approach.command_id,
            approach.goal_kind_wire,[local_goal.x,local_goal.y,local_goal.yaw],
            on_accept=accepted,on_result=completed)[0]

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
                output.set_result(ApproachResult(goal if accepted else None,
                    '' if accepted else result.reason or 'target reference path rejected'))
            except Exception as error:output.set_exception(error)
        self.compute_path(goal).add_done_callback(planned)
        return output

    def readiness_checks(self,now):
        """Ranger needs no extra permission gate; shared Nav2 health still applies."""
        return {}

    def begin_local_execution(self,task_id,command_id,target_id,description):
        self.completed_approach=None
        self.node.begin_execution(self,task_id,command_id,target_id,description)
        if getattr(self,'visualization',None):self.visualization.begin(task_id)

    def start_scan(self,*args,**kwargs):
        reply=super().start_scan(*args,**kwargs)
        if getattr(self.node,'execution_active',False):
            # 251f9fb uses goal/confirmation/sensor timers, not an overall scan deadline.
            self.scan.update(deadline=None,request_timeout=self.node.vlm_request_timeout,vlm_attempt=1)
        return reply

    def stop_motion(self,reason):
        self.local_arrival_verified=False
        self.completed_approach=None
        approach=getattr(self,'approach',None)
        if approach is not None:approach.close()
        if getattr(self.node,'execution_active',False):self.node.end_execution()
        return super().stop_motion(reason)

    def hold_motion(self,reason):
        return super().stop_motion(reason)

    def get_health(self):
        health=super().get_health()
        if getattr(self.node,'execution_active',False):
            # RGB-D/image-time TF use the old local wait; every other gate stays fatal.
            checks=self.last_health_details['checks']
            health=replace(health,nav_ready=all(value for key,value in checks.items()
                if key not in ('rgbd_ready','localization_valid')),
                nav_ready_reason=self.nav_ready_reason(('rgbd_ready','localization_valid')))
        codes=tuple(self.last_health_details.get('nav_ready_failures',{}))
        previous=getattr(self,'_last_nav_failure_codes',())
        if not health.nav_ready and (getattr(self,'_last_nav_ready',False) or previous and previous!=codes):
            active=bool(getattr(self.node,'execution_active',False))
            ignored=('localization_valid','rgbd_ready') if active else ('localization_valid',)
            failed=[key for key,value in self.last_health_details['checks'].items()
                if not value and key not in ignored]
            self.node.get_logger().warning('Navigation health invalid: '+json.dumps(dict(
                failed_checks=failed,reason=health.nav_ready_reason,execution_active=active,health_details=self.last_health_details),
                ensure_ascii=False,default=str))
        self._last_nav_ready=health.nav_ready
        self._last_nav_failure_codes=codes
        return health

    def consume_local_observation(self,scan,results,observations,error):
        node=self.node
        if not getattr(node,'execution_active',False):return observations
        from vlm_nav.models import WorkerResult
        target=node.execution_target_id
        snapshot=scan['snapshot']
        stale=time.monotonic()-snapshot.captured_monotonic>scan['request_timeout']
        invalid=any(results.get(t,('API_ERROR',None))[0] not in ('DETECTED','NOT_DETECTED','UNCERTAIN')
            or results.get(t,('API_ERROR',None))[1] is None for t in scan['targets'])
        if error or stale or invalid:
            reason='TIMEOUT' if stale or error=='TimeoutError' else 'API_ERROR'
            attempt=scan.get('vlm_attempt',1)
            node.get_logger().warn(f'VLM view {scan["index"]+1} attempt {attempt}/2 failed: {reason}; '
                + ('retrying same view' if attempt<2 else 'skipping this view'))
            self._motion_event('VLM_VIEW_ATTEMPT_FAILED',view_index=scan['index'],attempt=attempt,reason=reason)
            if attempt<2:
                scan.update(vlm_attempt=attempt+1,phase='FRAME',after=time.monotonic())
                return None
            self._motion_event('SCAN_VIEW_SKIPPED',view_index=scan['index'],attempt=attempt,reason=reason)
            node.target_tracker.reset()
            node.clear_target_depth_recovery()
            node.scan_retry_count=0
            node.scan_waiting_for_vlm=False
            node.scan_request_sequence=-1
            node.set_state('SCANNING')
            scan['vlm_attempt']=1
            return [replace(o,status=reason,position_valid=False,reason='two VLM attempts failed') for o in observations]
        scan['vlm_attempt']=1
        if snapshot.sequence<=node.execution_last_sequence:
            scan.update(phase='FRAME',after=time.monotonic())
            return None
        node.execution_last_sequence=snapshot.sequence
        snapshot=replace(snapshot,task_epoch=scan['task_epoch'],target_description=scan['targets'][target])
        node.scan_waiting_for_vlm=True
        node.scan_request_sequence=snapshot.sequence
        before=node.state
        result=results.get(target,('API_ERROR',None))[1]
        node.handle_worker_result(WorkerResult(snapshot,result,0.,error))
        if node.state in ('FAILED','API_ERROR'):
            scan['reply'].set_exception(RuntimeError(node.last_failure_reason))
            self.scan=None
            return None
        if node.target_reference_position is not None:
            node.execution_recovering=False
            from ..models import Pose2D
            point=node.target_reference_position
            position=self.frame.to_common(Pose2D(float(point[0]),float(point[1])))
            return [replace(o,status='DETECTED',position_valid=True,common_position=position)
                if o.target_id==target else o for o in observations]
        if node.state in ('TARGET_CONFIRMING','TARGET_REOBSERVING','SENSOR_WAITING'):
            scan.update(phase='LOCAL_RECOVERY',after=snapshot.captured_monotonic if node.state=='TARGET_CONFIRMING' else time.monotonic())
            return None
        if before in ('TARGET_CONFIRMING','TARGET_REOBSERVING') or node.scan_retry_count:
            # begin_scan/retry_stationary_scan_view already chose the old next step.
            if node.scan_retry_count:scan.update(phase='FRAME',after=node.scan_settle_until)
            return None
        return [replace(o,status='NOT_DETECTED',position_valid=False) if o.target_id==target else o
            for o in observations]

    def tick(self):
        node=self.node
        if getattr(self,'execution_paused',False):
            super().tick(advance_scan=False)
            return
        if getattr(node,'execution_active',False):
            health=self.get_health()
            if health.localization_valid and not health.nav_ready:return
        if getattr(node,'execution_active',False):
            node.local_tick()
            if node.state=='SCANNING':node.advance_recovery_scan()
            if node.state=='SENSOR_WAITING':
                super().tick(advance_scan=False)
                return
            scan=self.scan
            if scan and node.state in ('FAILED','API_ERROR'):
                self.vision.cancel()
                scan['reply'].set_exception(RuntimeError(node.last_failure_reason))
                self.scan=None
                return
            if scan and scan['phase']=='INFER' and time.monotonic()-scan['request_started']>scan['request_timeout']:
                self.vision.discard()
                scan['pending']=Future()
                scan['pending'].set_exception(TimeoutError('VLM attempt exceeded request deadline'))
            if scan and scan['phase']=='LOCAL_RECOVERY':
                if self.motions or node.plan_pending:
                    super().tick(advance_scan=False)
                    return
                scan.update(phase='FRAME',after=max(scan['after'],node.scan_settle_until))
            if scan and (self.motions or node.plan_pending) and scan['phase'] not in ('SPIN_WAIT',):
                super().tick(advance_scan=False)
                return
        super().tick()
        recovery=getattr(node,'execution_recovery_scan',None)
        if recovery is not None and recovery.done():
            node.execution_recovery_scan=None
            if not recovery.cancelled():
                observations=recovery.result()
                if node.execution_on_observation:
                    for observation in observations:node.execution_on_observation(observation)
            node.execution_recovering=False
        if getattr(node,'execution_recovering',False) and self.scan is None and node.state=='SCANNING':
            node.execution_recovering=False
