"""Shared Nav2 action, path, stop and scan lifecycle; no hardware startup."""
from collections import deque
from concurrent.futures import Future
from dataclasses import asdict
import math
import time
import uuid
import numpy as np
from vlm_nav.geometry import normalize_angle
from ..models import *
from .vision import ABCClient, AsyncVision, observations_from_results

class Nav2Adapter:
    simulation=False
    def __init__(self,observer,config,frame,read_only=False,global_goal_checks=False):
        from nav_msgs.msg import Odometry,Path
        from nav2_msgs.msg import Costmap
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
        self.node,self.config,self.frame=observer,config,frame
        self.read_only=read_only
        self.global_goal_checks=global_goal_checks
        self.results=deque()
        self.motions={}
        self.last_action_uuid=''
        self.plans={}
        self.path=[]
        self.path_at=0.0
        self.path_stamp=None
        self.path_timeout_logged=False
        self.odom_at=0.0
        self.stationary_since=None
        self.costmap=None
        self.costmap_at=0.0
        self.global_costmap=None
        self.global_costmap_at=0.0
        self.motion_fault=False
        self.scan=None
        self.client=None
        # Ranger's worker also encodes images and saves evidence around the HTTP request.
        self.vision=AsyncVision(config['api_timeout']+(5. if config.get('robot_id')=='ranger' else 0.))
        self.node.create_subscription(Odometry,config.get('odom_topic','/odometry'),self.on_odometry,qos_profile_sensor_data)
        self.node.create_subscription(Costmap,config.get('costmap_topic','/local_costmap/costmap_raw'),
            self.on_costmap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if global_goal_checks:
            self.node.create_subscription(Costmap,config['goal_costmap_topic'],
                lambda message:self.on_costmap(message,global_goal=True),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if config.get('path_topic'):
            self.node.create_subscription(Path,config['path_topic'],self.on_path_update,10)

    def readiness_checks(self,now):
        """Platforms must provide a real, fresh local motion-permission source."""
        return {'platform_motion_permission':False}

    def create_vision_client(self,config):
        return ABCClient(config)

    def on_odometry(self,message):
        now=time.monotonic()
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        ros_now=self.node.get_clock().now().nanoseconds*1e-9
        if abs(ros_now-stamp)>self.config['odom_max_age']:
            self.stationary_since=None
            self.odom_at=0.0
            return
        if self.odom_at and now-self.odom_at>self.config['odom_max_age']:
            self.stationary_since=None
        self.odom_at=now
        v=message.twist.twist
        linear=math.sqrt(sum(getattr(v.linear,k,0.)**2 for k in ('x','y','z')))
        angular=math.sqrt(sum(getattr(v.angular,k,0.)**2 for k in ('x','y','z')))
        if linear<=self.config.get('stationary_linear_speed',self.config.get('stationary_speed',.02)) and angular<=self.config.get('stationary_angular_speed',self.config.get('stationary_speed',.02)):
            if self.stationary_since is None:self.stationary_since=now
        else:self.stationary_since=None

    def on_costmap(self,message,global_goal=False):
        if global_goal:self.global_costmap=None
        else:self.costmap=None
        meta=message.metadata
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        now=self.node.get_clock().now().nanoseconds*1e-9
        update=dict(accepted=False,reason='invalid_metadata',frame=message.header.frame_id,
                    header_age_s=abs(now-stamp),received_monotonic=time.monotonic())
        if not hasattr(self,'last_costmap_updates'):self.last_costmap_updates={}
        self.last_costmap_updates['global' if global_goal else 'local']=update
        if not message.header.frame_id or not math.isfinite(meta.resolution) or meta.resolution<=0 or meta.size_x*meta.size_y<=0 or len(message.data)!=meta.size_x*meta.size_y:
            return
        max_age=self.config.get('goal_costmap_max_age' if global_goal else 'costmap_max_age',1.)
        if abs(now-stamp)>max_age:
            update['reason']='header_stale';return
        q=meta.origin.orientation
        if not all(math.isfinite(v) for v in (meta.origin.position.x,meta.origin.position.y,q.x,q.y,q.z,q.w)) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1.)>.01:
            update['reason']='invalid_origin'
            return
        update.update(accepted=True,reason='ok')
        if global_goal:
            self.global_costmap=message
            self.global_costmap_at=time.monotonic()
        else:
            self.costmap=message
            self.costmap_at=time.monotonic()

    def costmap_transform(self,global_goal=False):
        """Map pose → costmap frame, at the payload stamp; no odom-as-map shortcut."""
        from rclpy.time import Time
        from vlm_nav.vlm_navigator import quaternion_yaw
        message=self.global_costmap if global_goal else self.costmap
        received=self.global_costmap_at if global_goal else self.costmap_at
        max_age=self.config.get('goal_costmap_max_age' if global_goal else 'costmap_max_age',1.)
        details=dict(available=False,reason='no_payload',max_age_s=max_age,
                     frame=message.header.frame_id if message is not None else None,
                     last_update=dict(getattr(self,'last_costmap_updates',{}).get('global' if global_goal else 'local',{})))
        if not hasattr(self,'last_costmap_checks'):self.last_costmap_checks={}
        self.last_costmap_checks['global' if global_goal else 'local']=details
        if message is None:return None
        details['receipt_age_s']=time.monotonic()-received
        if details['receipt_age_s']>max_age:
            details['reason']='receipt_stale';return None
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        details['header_age_s']=abs(self.node.get_clock().now().nanoseconds*1e-9-stamp)
        if details['header_age_s']>max_age:
            details['reason']='header_stale';return None
        try:
            if message.header.frame_id=='map':pose=Pose2D()
            else:
                tf=self.node.tf_buffer.lookup_transform(message.header.frame_id,'map',Time.from_msg(message.header.stamp)).transform
                pose=Pose2D(tf.translation.x,tf.translation.y,quaternion_yaw(tf.rotation))
            details.update(available=pose.valid,reason='ok' if pose.valid else 'invalid_tf')
            return pose if pose.valid else None
        except Exception as error:
            details.update(reason='tf_unavailable',error_type=type(error).__name__)
            return None

    def costmap_candidate_free(self,candidate,global_goal=False,*,allow_outside=False):
        from vlm_nav.vlm_navigator import quaternion_yaw
        transform=self.costmap_transform(global_goal=True) if global_goal else self.costmap_transform()
        if transform is None:return False
        c,s=math.cos(transform.yaw),math.sin(transform.yaw)
        x=c*candidate.x-s*candidate.y+transform.x
        y=s*candidate.x+c*candidate.y+transform.y
        message=self.global_costmap if global_goal else self.costmap
        meta=message.metadata
        yaw=quaternion_yaw(meta.origin.orientation)
        c,s=math.cos(yaw),math.sin(yaw)
        dx,dy=x-meta.origin.position.x,y-meta.origin.position.y
        x,y=c*dx+s*dy,-s*dx+c*dy
        local_yaw=normalize_angle(candidate.yaw+transform.yaw-yaw)
        footprint=self.config['footprint']
        # Conservative centered rectangle also covers asymmetric footprints.
        padding=self.config.get('footprint_padding',0.)
        length,width=2*(max(abs(p[0]) for p in footprint)+padding),2*(max(abs(p[1]) for p in footprint)+padding)
        c,s=math.cos(local_yaw),math.sin(local_yaw)
        if not allow_outside and any(not (0<=x+c*px-s*py<meta.size_x*meta.resolution and 0<=y+s*px+c*py<meta.size_y*meta.resolution)
            for px in (-length/2,length/2) for py in (-width/2,width/2)):return False
        raw=np.asarray(message.data,dtype=np.int16).reshape(meta.size_y,meta.size_x)
        # Nav2 253/254 are inscribed/lethal and 255 unknown. Unknown body cells veto too.
        # Test the actual pose, including any cell intersecting the body rectangle;
        # do not snap it to a cell center. Projection of a half-cell is conservative.
        gy,gx=np.indices(raw.shape)
        dx,dy=(gx+.5)*meta.resolution-x,(gy+.5)*meta.resolution-y
        half_cell=meta.resolution*(abs(c)+abs(s))/2
        body=(np.abs(c*dx+s*dy)<=length/2+half_cell) & (np.abs(-s*dx+c*dy)<=width/2+half_cell)
        return (allow_outside or bool(body.any())) and not np.any(body & (raw>=self.config.get('costmap_blocked_cost',253)))

    def observation_candidate_free(self,candidate):
        return self._rotation_free(candidate,True) and self._rotation_free(candidate,False,allow_outside=True)

    def approach_candidate_free(self,candidate):
        return (self.costmap_candidate_free(candidate,global_goal=True) and
            self.costmap_candidate_free(candidate,allow_outside=True) and self.observation_candidate_free(candidate))

    def _rotation_free(self,candidate,global_goal,allow_outside=False):
        from vlm_nav.vlm_navigator import quaternion_yaw
        from ..generated_candidates import circle_free, rotation_radius
        transform=self.costmap_transform(global_goal=global_goal)
        if transform is None:return False
        message=self.global_costmap if global_goal else self.costmap
        meta=message.metadata
        c,s=math.cos(transform.yaw),math.sin(transform.yaw)
        x=c*candidate.x-s*candidate.y+transform.x-meta.origin.position.x
        y=s*candidate.x+c*candidate.y+transform.y-meta.origin.position.y
        yaw=quaternion_yaw(meta.origin.orientation);c,s=math.cos(yaw),math.sin(yaw)
        raw=np.asarray(message.data,dtype=np.int16).reshape(meta.size_y,meta.size_x)
        x,y=c*x+s*y,-s*x+c*y
        radius=rotation_radius(self.config['footprint'],self.config.get('footprint_padding',0.))
        if allow_outside:
            gy,gx=np.indices(raw.shape)
            dx=np.maximum(np.abs((gx+.5)*meta.resolution-x)-meta.resolution/2,0)
            dy=np.maximum(np.abs((gy+.5)*meta.resolution-y)-meta.resolution/2,0)
            return not np.any((dx*dx+dy*dy<=radius*radius) & (raw>=self.config.get('costmap_blocked_cost',253)))
        return circle_free(raw,x,y,radius,
            meta.resolution,lambda cells:cells>=self.config.get('costmap_blocked_cost',253))

    def _motion_event(self,event,goal_id=None,motion=None,**fields):
        record=getattr(self,'record_event',None)
        if record is None:return
        if motion is None:goal_id,motion=getattr(self,'last_motion',('',{}))
        try:
            record(dict(event=event,task_id=motion.get('task',''),command_id=motion.get('command',''),
                goal_id=goal_id,native_action_uuid=motion.get('native_uuid',''),
                goal_kind=motion.get('kind',''),goal_pose=motion.get('target'),
                monotonic_time=time.monotonic(),**fields))
        except OSError:
            # Evidence failure must not prevent cancellation or sending zero.
            self.evidence_write_failed=True
            self.motion_fault=True

    def _clear_path(self,reason):
        self.path=[]
        self.path_at=0.0
        self.path_stamp=None
        self.path_timeout_logged=False
        for motion in self.motions.values():motion.pop('pending_path',None)
        self._motion_event('PATH_CLEARED',reason=reason)

    def on_path_update(self,message):
        from vlm_nav.vlm_navigator import quaternion_yaw
        # shortcut: Path has no goal UUID; exclusive source/replan association still needs field verification.
        motions=[(goal_id,m) for goal_id,m in self.motions.items()
                 if not m['cancel'] and m['kind']!='SCAN_SPIN']
        if len(motions)!=1:
            self._clear_path('no unique current navigation')
            return
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if message.header.frame_id!='map' or stamp<getattr(self,'path_after_stamp',0.):
            self._clear_path('wrong frame or previous navigation plan')
            return
        path=[Pose2D(p.pose.position.x,p.pose.position.y,quaternion_yaw(p.pose.orientation)) for p in message.poses]
        if not path or not all(p.valid for p in path) or not all(
            math.isfinite(p.pose.position.z) for p in message.poses):
            self._clear_path('empty or nonfinite execution path')
            return
        received_at=time.monotonic()
        stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        goal_id,motion=motions[0]
        if motion['handle'] is None:
            # DDS can deliver the first plan before the action acceptance callback.
            motion['pending_path']=(path,stamp,stamp_ns,received_at)
            self._motion_event('PATH_BUFFERED',goal_id,motion,path_header_stamp_ns=stamp_ns,
                poses=len(path),path_end=[path[-1].x,path[-1].y,path[-1].yaw])
            return
        self._set_execution_path(path,stamp,stamp_ns,received_at)

    def _set_execution_path(self,path,stamp,stamp_ns,received_at):
        if time.monotonic()-received_at>self.config.get('path_receive_timeout',5.0):
            self._clear_path('buffered execution path receive timeout')
            return
        interval=received_at-self.path_at if self.path_at else None
        changed=self.path_stamp is not None and self.path_stamp!=stamp
        self.path,self.path_at,self.path_stamp=path,received_at,stamp
        self.path_timeout_logged=False
        self._motion_event('PATH_RECEIVED',path_header_stamp_ns=stamp_ns,path_received_monotonic=received_at,
            receive_interval_s=interval,plan_stamp_changed=changed,poses=len(path),
            path_end=[path[-1].x,path[-1].y,path[-1].yaw])

    def get_health(self):
        from rclpy.time import Time
        from vlm_nav.vlm_navigator import quaternion_yaw
        now=time.monotonic()
        pose=Pose2D()
        valid=False
        pose_age=None;pose_error=None
        try:
            tf=self.node.tf_buffer.lookup_transform('map','base_link',Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            pose_age=abs(self.node.get_clock().now().nanoseconds*1e-9-stamp)
            valid=pose_age<self.config.get('pose_max_age',1.0)
            pose=Pose2D(tf.transform.translation.x,tf.transform.translation.y,quaternion_yaw(tf.transform.rotation))
        except Exception as error:
            valid=False
            pose_error=type(error).__name__
        stopped=now-self.odom_at<=self.config['odom_max_age'] and self.stationary_since is not None and now-self.stationary_since>=self.config['stationary_duration']
        gate_ages={name:now-v for name,v in getattr(self.node,'external_gate_receipts',{}).items()}
        checks=dict(self.readiness_checks(now),
            navigate_server=self.node.navigator.server_is_ready(),planner_server=self.node.path_planner.server_is_ready(),
            spin_server=self.node.spinner.server_is_ready(),motion_fault_clear=not self.motion_fault,
            depth_units_verified=bool(self.node.depth_units_verified),rgbd_ready=bool(self.node.vlm_input_ready(now)),
            odom_fresh=bool(self.odom_at and now-self.odom_at<=self.config['odom_max_age']),
            local_costmap=self.costmap_transform() is not None)
        if hasattr(self,'task_map_ready'):checks['active_task_map']=self.task_map_ready
        ready=all(checks.values())
        snapshot=getattr(self.node,'latest_snapshot',None)
        camera_tf_at=getattr(self.node,'last_camera_tf_success',None)
        self.last_health_details=dict(checks=dict(checks,localization_valid=valid),
            pose_age_s=pose_age,pose_max_age_s=self.config.get('pose_max_age',1.0),pose_tf_error=pose_error,
            external_gate_ages_s=gate_ages,external_gate_timeout_s=getattr(self.node.p,'external_gate_timeout',None),
            local_costmap=dict(getattr(self,'last_costmap_checks',{}).get('local',{})),
            rgbd_frame_age_s=now-snapshot.captured_monotonic if snapshot else None,
            camera_tf_age_s=now-camera_tf_at if camera_tf_at else None,
            rgbd_sync_healthy=getattr(self.node,'rgbd_sync_status',{}).get('healthy'),
            odom_age_s=now-self.odom_at,odom_max_age_s=self.config['odom_max_age'])
        path_valid=bool(self.path) and now-self.path_at<=self.config.get('path_receive_timeout',5.0)
        if self.path and not path_valid and not getattr(self,'path_timeout_logged',False):
            self.path_timeout_logged=True
            self._motion_event('PATH_RECEIVE_TIMEOUT',receive_age_s=now-self.path_at)
        return Health(pose,valid,ready,stopped,list(self.path),path_valid,self.config['footprint'],not self.motions,
                      'execution path missing or receive timeout' if not path_valid else '')

    def compute_path(self,local_goal):
        from nav2_msgs.action import ComputePathToPose
        future=Future()
        if not self.node.path_planner.server_is_ready():
            future.set_result(PathResult(False,reason='planner unavailable'))
            return future
        goal=ComputePathToPose.Goal()
        goal.goal=self.node.make_pose_stamped(local_goal.x,local_goal.y,local_goal.yaw)
        request=self.node.path_planner.send_goal_async(goal)
        self.plans[id(future)] = None
        def response(done):
            try:
                handle=done.result()
                if not handle.accepted: raise ValueError('path rejected')
                self.plans[id(future)]=handle
                if future.cancelled():handle.cancel_goal_async()
                def result(completed):
                    self.plans.pop(id(future),None)
                    if future.done():return
                    try:
                        value=completed.result()
                        if value.status!=4 or not value.result.path.poses:raise ValueError('no local path')
                        if value.result.path.header.frame_id!='map':raise ValueError('wrong path frame')
                        from vlm_nav.vlm_navigator import quaternion_yaw
                        path=[Pose2D(p.pose.position.x,p.pose.position.y,quaternion_yaw(p.pose.orientation)) for p in value.result.path.poses]
                        if not all(p.valid for p in path):raise ValueError('nonfinite path')
                        future.set_result(PathResult(True,path))
                    except Exception:future.set_result(PathResult(False,reason='local path failed'))
                handle.get_result_async().add_done_callback(result)
            except Exception:
                self.plans.pop(id(future),None)
                if not future.done():future.set_result(PathResult(False,reason='path request failed'))
        request.add_done_callback(response)
        return future

    def read_approach_parameters(self):
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType
        output=Future();values={}
        if not hasattr(self,'approach_parameter_clients'):
            self.approach_parameter_clients={name:self.node.create_client(GetParameters,'/'+name+'/get_parameters')
                for name in ('controller_server','planner_server')}
        def request(server,names,consume):
            client=self.approach_parameter_clients[server]
            if not client.service_is_ready():
                if not output.done():output.set_exception(RuntimeError(server+' parameter service unavailable'))
                return
            def completed(done):
                if output.done():return
                try:consume(done.result().values)
                except Exception as error:output.set_exception(error)
            client.call_async(GetParameters.Request(names=names)).add_done_callback(completed)
        def plugins(server,field,parameter,label):
            def selected(parameters):
                if len(parameters)!=1 or parameters[0].type!=ParameterType.PARAMETER_STRING_ARRAY or not parameters[0].string_array_value:
                    raise ValueError('missing active '+field)
                names=[name+'.'+parameter for name in parameters[0].string_array_value]
                def read(parameters):
                    if len(parameters)!=len(names) or any(p.type!=ParameterType.PARAMETER_DOUBLE or
                            not math.isfinite(p.double_value) or p.double_value<0 for p in parameters):
                        raise ValueError('invalid actual '+label)
                    values[label]=max(p.double_value for p in parameters)
                    if len(values)==2:output.set_result(values)
                request(server,names,read)
            request(server,[field],selected)
        try:
            plugins('controller_server','goal_checker_plugins','xy_goal_tolerance','xy_goal_tolerance')
            plugins('planner_server','planner_plugins','tolerance','planner_tolerance')
        except Exception as error:
            if not output.done():output.set_exception(error)
        return output

    def select_approach(self,local_object,excluded_candidates,*,standoff_distance=None,path_accept=None):
        output=Future()
        if self.costmap_transform() is None:
            output.set_result(ApproachResult(reason='local map/costmap unavailable'))
            return output
        candidates=[]
        free=self.approach_candidate_free if getattr(self,'global_goal_checks',False) else self.costmap_candidate_free
        distance=self.config['standoff_distance'] if standoff_distance is None else standoff_distance
        if isinstance(distance,bool) or not isinstance(distance,(int,float)) or not math.isfinite(distance) or distance<=0:
            output.set_result(ApproachResult(reason='invalid standoff distance'))
            return output
        for angle in np.linspace(-math.pi,math.pi,8,endpoint=False):
            candidate=Pose2D(local_object.x+distance*math.cos(angle),local_object.y+distance*math.sin(angle),normalize_angle(angle+math.pi))
            if candidate in excluded_candidates:continue
            if free(candidate):candidates.append(candidate)
        health=self.get_health()
        candidates.sort(key=lambda p:math.hypot(p.x-health.pose.x,p.y-health.pose.y))
        def attempt():
            if output.done():return
            if not candidates:
                output.set_result(ApproachResult(reason='no locally reachable candidate'))
                return
            candidate=candidates.pop(0)
            if not free(candidate):attempt();return
            future=self.compute_path(candidate)
            def done(f):
                if output.done():return
                try:
                    result=f.result()
                    if result.reachable and result.path and free(candidate) and (path_accept is None or path_accept(result)):
                        output.set_result(ApproachResult(candidate))
                    else:attempt()
                except Exception as error:output.set_exception(error)
            future.add_done_callback(done)
        attempt()
        return output

    def _motion(self,client,goal,task_id,command_id,kind,target=None):
        if getattr(self,'read_only',False):raise RuntimeError('read-only adapter cannot send motion goals')
        if getattr(self,'motion_guard',None):self.motion_guard()
        goal_id=uuid.uuid4().hex
        motion={'handle':None,'cancel':False,'reply':Future(),'task':task_id,'command':command_id,'kind':kind,'since':time.monotonic(),
                'native_uuid':'','target':target}
        self.motions[goal_id]=motion
        self.last_motion=(goal_id,motion)
        self._motion_event('MOTION_REQUESTED',goal_id,motion)
        def finish(status,native_status=None):
            self.motions.pop(goal_id,None)
            motion.pop('pending_path',None)
            if kind!='SCAN_SPIN':
                self._clear_path('motion terminal: '+status)
                self.results.append(GoalResult(task_id,command_id,goal_id,kind,status))
            self._motion_event('MOTION_TERMINAL',goal_id,motion,status=status,native_status=native_status)
            if not motion['reply'].done():motion['reply'].set_result(status)
        def accepted(done):
            try:
                handle=done.result()
                if not handle.accepted:finish('ABORTED');return
                motion['handle']=handle
                self.last_action_uuid=bytes(handle.goal_id.uuid).hex()
                motion['native_uuid']=self.last_action_uuid
                self._motion_event('MOTION_ACCEPTED',goal_id,motion)
                if motion['cancel']:self.cancel_navigation(goal_id)
                elif self.motions.get(goal_id) is motion:
                    pending=motion.pop('pending_path',None)
                    if pending:self._set_execution_path(*pending)
                def completed(done):
                    try:
                        status=done.result().status
                        finish({4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(status,'ABORTED'),status)
                    except Exception:self.motion_fault=True;finish('ABORTED')
                handle.get_result_async().add_done_callback(completed)
            except Exception:
                self.motion_fault=True
                finish('ABORTED')
        try:client.send_goal_async(goal).add_done_callback(accepted)
        except Exception:self.motion_fault=True;finish('ABORTED')
        return goal_id,motion['reply']

    def start_navigation(self,local_goal,task_id,command_id,goal_kind):
        from nav2_msgs.action import NavigateToPose
        if self.motions or self.scan:raise RuntimeError('motion not serial')
        goal=NavigateToPose.Goal()
        goal.pose=self.node.make_pose_stamped(local_goal.x,local_goal.y,local_goal.yaw)
        self._clear_path('new navigation')  # a preflight path is NOT an executing/replanned path
        self.path_after_stamp=self.node.get_clock().now().nanoseconds*1e-9
        return self._motion(self.node.navigator,goal,task_id,command_id,goal_kind,
                            [local_goal.x,local_goal.y,local_goal.yaw])[0]

    def cancel_navigation(self,goal_id):
        future=Future()
        motion=self.motions.get(goal_id)
        if not motion:future.set_result(True);return future
        motion['cancel']=True
        if motion['kind']!='SCAN_SPIN':self._clear_path('navigation canceled')
        motion['since']=time.monotonic()
        self._motion_event('CANCEL_REQUESTED',goal_id,motion)
        if motion['handle']:
            try:return motion['handle'].cancel_goal_async()
            except Exception:self.motion_fault=True
        future.set_result(False)  # acceptance pending; health still motion_terminal=false
        return future

    def stop_motion(self,reason):
        self.selection_epoch = getattr(self, 'selection_epoch', 0) + 1
        if getattr(self,'read_only',False):raise RuntimeError('read-only adapter cannot publish stopping commands')
        from geometry_msgs.msg import Twist
        self.vision.cancel()
        scan=self.scan
        if self.scan:
            self.scan['reply'].cancel()
            self.scan=None
        for goal_id in list(self.motions):self.cancel_navigation(goal_id)
        for handle in self.plans.values():
            if handle:handle.cancel_goal_async()
        self.node.stop_pub.publish(Twist())
        self._motion_event('STOP_COMMAND_SENT',reason=reason)
        if scan and scan.get('phase')=='INFER':
            self._record_scan_view(scan,[],'CANCELED','Canceled')
        future=Future();future.set_result(True)
        return future

    def start_scan(self,target_descriptions,scan_id,*,stop_on_target=None):
        if getattr(self,'read_only',False):raise RuntimeError('read-only adapter cannot start Spin/scan')
        if self.scan or self.motions:raise RuntimeError('motion not serial')
        if stop_on_target is not None and stop_on_target not in target_descriptions:
            raise ValueError('stop target must belong to requested scan')
        reply=Future()
        self.scan={'reply':reply,'id':scan_id,'targets':target_descriptions,'stop_on_target':stop_on_target,'index':0,'phase':'SPIN',
                   'navigation':getattr(self,'last_motion',('',{})),
                   'initial':self.get_health().pose.yaw,'pending':None,'observations':[],'after':0,'deadline':time.monotonic()+self.config.get('scan_timeout',120.0)}
        return reply

    def _record_scan_view(self,scan,observations,consumer_status,error_type=None):
        if scan.get('view_recorded'):return
        scan['view_recorded']=True
        evidence=scan.get('evidence')
        nav_id,nav=scan.get('navigation',('',{}))
        self._motion_event('VLM_VIEW_RESULT',nav_id,nav,scan_id=scan['id'],view_index=scan['index'],
            image_stamp_ns=scan['snapshot'].stamp.sec*10**9+scan['snapshot'].stamp.nanosec,
            consumer_status=consumer_status,error_type=error_type,
            evidence_dir=str(evidence.directory) if evidence else None,
            observations=[asdict(o) for o in observations])

    def tick(self):
        from nav2_msgs.action import Spin
        from rclpy.duration import Duration
        self.vision.poll()
        now=time.monotonic()
        for motion in self.motions.values():
            if motion['cancel'] and now-motion['since']>self.config.get('cancel_timeout',5.0):
                self.motion_fault=True  # keep handle occupied; no replacement movement
        scan=self.scan
        evidence=getattr(self,'_scan_evidence',None) or (scan.get('evidence') if scan else None)
        if evidence and evidence.write_failed:
            self._scan_evidence=None
            self.evidence_write_failed=True;self.motion_fault=True
            self.stop_motion('VLM evidence write failed');return
        if not scan:return
        if not self.get_health().nav_ready or not self.get_health().localization_valid:
            self.stop_motion('scan health lost')
            return
        if now>scan['deadline']:
            scan['reply'].set_exception(TimeoutError('scan timeout'))
            self.stop_motion('scan timeout')
            return
        phase=scan['phase']
        if phase=='SPIN':
            if not self.get_health().stopped or not self.vision.available:return
            headings=self.config['scan_angles']
            if scan['index']>=len(headings):
                scan['reply'].set_result(scan['observations']);self.scan=None;return
            yaw=self.get_health().pose.yaw
            relative=normalize_angle(scan['initial']+headings[scan['index']]-yaw)
            if abs(relative)<.01:
                scan['phase']='FRAME';scan['after']=now
            else:
                goal=Spin.Goal();goal.target_yaw=relative;goal.time_allowance=Duration(seconds=30.0).to_msg()
                _,scan['pending']=self._motion(self.node.spinner,goal,'',scan['id'],'SCAN_SPIN')
                scan['phase']='SPIN_WAIT'
        elif phase=='SPIN_WAIT' and scan['pending'].done() and self.get_health().stopped:
            if scan['pending'].result()!='SUCCEEDED':
                status=scan['pending'].result()
                reason=f'scan spin failed: view {scan["index"]+1}, action {status}'
                self._motion_event('SCAN_FAILED',reason=reason,view_index=scan['index'],status=status)
                scan['reply'].set_exception(RuntimeError(reason));self.scan=None;return
            scan['phase']='FRAME';scan['after']=now
        elif phase=='FRAME':
            snapshot=self.node.latest_snapshot
            if not self.node.vlm_input_ready(now) or snapshot.captured_monotonic<=scan['after'] or not self.vision.available:return
            evidence=None
            if getattr(self,'scan_evidence_dir',None):
                from ..scan_evidence import ScanEvidence
                nav_id,nav=scan['navigation']
                evidence=ScanEvidence(self.scan_evidence_dir,dict(scan_id=scan['id'],view_index=scan['index'],
                    viewpoint_id=getattr(self,'scan_viewpoint_id',''),
                    targets=scan['targets'],image_stamp_ns=snapshot.stamp.sec*10**9+snapshot.stamp.nanosec,
                    frame_id=snapshot.frame_id,image_sequence=snapshot.sequence,
                    image_shape=list(snapshot.rgb.shape),capture_pose=asdict(self.get_health().pose),
                    scan_initial_yaw=scan['initial'],scan_requested_yaw=normalize_angle(scan['initial']+self.config['scan_angles'][scan['index']]),
                    task_id=nav.get('task',''),command_id=nav.get('command',''),
                    navigation_goal_id=nav_id,native_action_uuid=nav.get('native_uuid',''),
                    navigation_goal_pose=nav.get('target'),api_timeout_s=self.config['api_timeout'],
                    adapter_timeout_s=self.vision.timeout))
            def infer():
                results={};error_type=None
                try:
                    if evidence:evidence.begin()
                    if self.client is None:self.client=self.create_vision_client(self.config)
                    self.client.evidence=evidence
                    results=self.client.infer_abc(snapshot.rgb,scan['targets'])
                    return results
                except Exception as error:
                    error_type=type(error).__name__;raise
                finally:
                    try:
                        if evidence and not evidence.write_failed:
                            try:evidence.finish(results,error_type)
                            except Exception:
                                evidence.write_failed=True
                                raise OSError('VLM evidence finalization failed') from None
                    finally:
                        if self.client is not None:self.client.evidence=None
                        if evidence and evidence.write_failed:
                            self.evidence_write_failed=True;self.motion_fault=True
            scan['snapshot']=snapshot
            scan['evidence']=evidence;scan['view_recorded']=False
            self._scan_evidence=evidence
            scan['pending']=self.vision.submit(infer)
            scan['phase']='INFER'
        elif phase=='INFER' and scan['pending'].done():
            error_type=None
            try:results=scan['pending'].result()
            except Exception as error:results={};error_type=type(error).__name__
            observations=observations_from_results(results,scan['snapshot'],self.node.ground_pixel_with_reason,
                self.frame,scan['id'],scan['index'],targets=scan['targets'],robot_id=self.config.get('robot_id','go2'))
            self._record_scan_view(scan,observations,'TIMEOUT' if error_type=='TimeoutError' else 'ERROR' if error_type else 'APPLIED',error_type)
            scan['observations']+=observations
            detected=next((o for o in observations if o.target_id==scan.get('stop_on_target') and o.status=='DETECTED'),None)
            if detected is not None:
                self._motion_event('SCAN_TARGET_DETECTED',target_id=detected.target_id,view_index=scan['index'],
                    position_valid=detected.position_valid,reason='requested target detected; remaining scan headings skipped')
                scan['reply'].set_result(scan['observations']);self.scan=None;return
            scan['index']+=1;scan['phase']='SPIN'

    def close(self,stop=True):
        if stop and not getattr(self,'read_only',False):self.stop_motion('shutdown')
        self.vision.close()
