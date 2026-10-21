"""Independent Go2 adapter acceptance. No task topics, peer, bridge, startup or ARM."""
import argparse
from dataclasses import asdict,replace
import json
import math
import os
from pathlib import Path
import time
import uuid
from contextlib import contextmanager
from .config import load_config,live_gates,go2_local_gates,robot_local_gates,map_file_info
from .models import Pose2D

MOTION=('navigate','scan','explore')
MOTION_LOCK='/tmp/go2-coop-acceptance-motion.lock'

class StopUnconfirmed(RuntimeError):
    pass

class MapParameterRead:
    """Poll alongside sensor callbacks; lost service replies must not block inspection."""
    def __init__(self,client,request):
        self.client,self.request=client,request
        self.path=None
        self.future=None
        self.attempts=0
        self.sent_at=self.retry_at=0.
        self.status='service_unavailable'

    def poll(self,now):
        if self.path:return
        if self.future is not None:
            if self.future.done():
                try:
                    values=self.future.result().values
                    value=values[0].string_value if values else ''
                    if isinstance(value,str) and value.strip():
                        self.path=str(Path(value).expanduser().resolve())
                        self.status='ok'
                        return
                    self.status='empty_parameter'
                except Exception as error:
                    self.status='response_error:'+type(error).__name__
            elif now-self.sent_at<2.:
                return
            else:
                self.future.cancel()
                self.client.remove_pending_request(self.future)
                self.status='response_timeout'
            self.future=None
            self.retry_at=now+.25
        if now<self.retry_at:return
        if not self.client.service_is_ready():
            self.status='service_unavailable'
            return
        self.attempts+=1
        self.future=self.client.call_async(self.request)
        self.sent_at=now
        self.status='response_pending'

@contextmanager
def motion_slot(robot_id='go2'):
    import fcntl
    lock=MOTION_LOCK if robot_id=='go2' else '/tmp/ranger-coop-acceptance-motion.lock'
    fd=os.open(lock,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    try:
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('another Go2 acceptance owns motion')
        if os.read(fd,64):raise RuntimeError('previous acceptance needs field stop reconciliation')
        os.write(fd,b'IN_PROGRESS')  # Survives abrupt process exit; no automatic continuation.
        try:yield
        except StopUnconfirmed:
            os.lseek(fd,0,os.SEEK_SET);os.ftruncate(fd,0);os.write(fd,b'STOP_UNCONFIRMED')
            raise
        except BaseException:
            # Preserve IN_PROGRESS when exceptional cleanup cannot prove stop.
            raise
        else:os.ftruncate(fd,0)
    finally:os.close(fd)

def other_owners(node,robot_id='go2'):
    names=node.get_node_names()
    owners=set(names) & {'vlm_nav','multi_agent_coordinator','multi_agent_go2_agent',
        'multi_agent_task_agent','multi_agent_go2_rgbd','multi_agent_ranger_agent','multi_agent_ranger_rgbd'}
    for name in ('multi_agent_go2_check_motion','multi_agent_ranger_check_motion'):
        if names.count(name)>(1 if name==f'multi_agent_{robot_id}_check_motion' else 0):owners.add(name)
    return sorted(owners)

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-dir')
    p.add_argument('--action',choices=('inspect','plan','approach',*MOTION),default='inspect')
    p.add_argument('--offline',action='store_true',help='configuration report only; never initialize ROS')
    p.add_argument('--map-yaml',help='print file digests for a candidate map; does not write configuration')
    p.add_argument('--require-ready',action='store_true',help='exit 1 if reported prerequisites are incomplete')
    p.add_argument('--allow-motion',action='store_true',help='required in addition to an explicit motion action')
    p.add_argument('--output-dir',help='new motion evidence directory; generated strategy defaults to the managed experiment root')
    p.add_argument('--target-id',choices=('A','B','C'))
    p.add_argument('--target',help='single exploration target description; defaults to the selected scene.targets ID')
    p.add_argument('--search-hint', help='location clue for furniture selection only; not visual detection evidence')
    p.add_argument('--approach-target', action='store_true', help='after grounded detection, approach using existing standoff planner')
    p.add_argument('--strategy', choices=('fixed_viewpoints', 'semantic_furniture', 'generated_candidates_vlm'),
                   help='Go2 internal exploration strategy; default fixed_viewpoints')
    p.add_argument('--goal',nargs=3,type=float,metavar=('X','Y','YAW'),help='measured local Go2 map pose; inspect only probes costmap footprint; approach treats X/Y as object position')
    p.add_argument('--timeout',type=float,default=15.,help='inspection/action deadline in seconds')
    p.add_argument('--cancel-after',type=float,help='explicit cancellation acceptance during navigate/scan')
    return p

def terminal_report(value):
    """Keep failure diagnostics on screen; coordinate arrays remain in result.json."""
    if isinstance(value,dict):
        summary={}
        for key,item in value.items():
            if key in ('footprint','path') and isinstance(item,list):
                summary[key+'_count']=len(item)
            else:summary[key]=terminal_report(item)
        return summary
    if isinstance(value,list):return [terminal_report(item) for item in value]
    return value

def emit(value):
    print(json.dumps(terminal_report(value),ensure_ascii=False,allow_nan=False,indent=2))

def config_report(config,map_yaml=None,*,robot_id='go2'):
    from .go2_exploration import exploration_gates
    path=map_yaml or config['robot_'+robot_id].get('map_path')
    try:info=map_file_info(path) if path else None
    except (OSError,KeyError,TypeError,ValueError):info={'error':'map YAML/image unreadable'}
    semantic_report = None
    if config['robot_'+robot_id].get('exploration_strategy') == 'semantic_furniture':
        from .semantic_search import load_catalog
        try:semantic_report = load_catalog(config,robot_id)
        except (OSError, KeyError, TypeError, ValueError) as error:
            semantic_report = {'error': str(error)}
    elif config['robot_'+robot_id].get('exploration_strategy') == 'generated_candidates_vlm':
        from .generated_candidates import GeneratedCandidates
        try:
            geometry=GeneratedCandidates(config,robot_id)
            semantic_report=dict(points=geometry.points, semantic_yaml=str(geometry.semantic_path) if geometry.semantic_path else None,
                                 rotation_radius=geometry.radius, max_candidates_per_request=12)
        except (OSError, KeyError, TypeError, ValueError) as error:
            semantic_report={'error':str(error)}
    arrival_report={}
    robot=config['robot_'+robot_id]
    if robot_id=='ranger':
        from vlm_nav.arrival import load_contract
        import yaml
        try:
            contract=load_contract(robot.get('arrival_config_files'))
            params=yaml.safe_load(Path(robot['nav2_profile']).read_text())
            arrival_report=dict(arrival_contract=contract,configured_nav2_tolerances=dict(
                planner_tolerance=params['planner_server']['ros__parameters']['GridBased']['tolerance'],
                goal_xy_tolerance=params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']))
        except (OSError,KeyError,TypeError,ValueError) as error:
            arrival_report={'arrival_configuration_error':str(error)}
    return {**arrival_report,'mode':'configuration_only','map_files':info, 'semantic_search': semantic_report,
        'exploration_strategy': config['robot_'+robot_id].get('exploration_strategy', 'fixed_viewpoints'),
        **({'go2_local_gates':go2_local_gates(config)} if robot_id=='go2' else {}),
        'robot_local_gates':go2_local_gates(config) if robot_id=='go2' else robot_local_gates(config,robot_id),
        'robot_id':robot_id,'common_frame_id':config['alignment'].get('common_frame_id','lab_ref'),
        'live_gates':live_gates(config) if robot_id=='go2' else ['cooperative gates are not evaluated in local_only mode'],
        'single_robot_experiment_gates':exploration_gates(config,robot_id),
        'path_monitoring':{'topic':config['robot_'+robot_id].get('path_topic'),
            'receive_timeout_s':config['robot_'+robot_id].get('path_receive_timeout',5.),
            'plan_age_checked':False,'source_field_verified':config['robot_'+robot_id].get('path_source_verified',False)},
        'motion_sent':False}

def inspection_ready(report,health,*,robot_id='go2',owners=()):
    ready=not report['robot_local_gates'] and report['active_map_matches'] and health.localization_valid and health.stopped and report['costmap_valid']
    if robot_id=='ranger':
        ready=ready and health.nav_ready and health.motion_terminal and not owners
    return bool(ready)

def stop_owned(adapter,spin,timeout,record=None):
    adapter.stop_motion('independent acceptance completion/cancel')
    deadline=time.monotonic()+timeout
    previous=None
    while time.monotonic()<deadline:
        spin()
        health=adapter.get_health()
        observed=(health.stopped,health.motion_terminal)
        if record and observed!=previous:
            try:
                record(dict(event='STOP_OBSERVATION',stopped=health.stopped,
                    motion_terminal=health.motion_terminal,pose=asdict(health.pose)))
            except OSError:
                adapter.evidence_write_failed=True
                adapter.motion_fault=True
            previous=observed
        if health.stopped and health.motion_terminal:return True
    return False

def require_ros_environment(robot_id='go2',config=None):
    if robot_id=='ranger':
        expected=str((config or {}).get('robot_ranger',{}).get('domain_id',0))
        if os.getenv('ROS_DOMAIN_ID','0')!=expected:
            raise ValueError('Ranger local ROS_DOMAIN_ID must equal '+expected)
        return
    if robot_id!='go2':raise ValueError('unknown robot_id')
    if os.getenv('ROS_DOMAIN_ID','0')!='0':raise ValueError('Go2 acceptance requires local Domain0')
    if os.getenv('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp' or os.getenv('GO2_NET_IFACE')!='enp4s0':
        raise ValueError('use runbook Domain0/CycloneDDS/enp4s0 environment')
    from urllib.parse import urlparse,unquote
    from ament_index_python.packages import get_package_share_directory
    uri=urlparse(os.getenv('CYCLONEDDS_URI',''))
    expected_xml=Path(get_package_share_directory('vlm_nav'))/'config/cyclonedds_go2.xml'
    if uri.scheme!='file' or uri.netloc or not uri.path or Path(unquote(uri.path)).read_bytes()!=expected_xml.read_bytes():
        raise ValueError('use the documented Go2-only CycloneDDS XML')

def run_ros(config,args,events=None,*,robot_id='go2'):
    if robot_id=='go2':require_ros_environment()
    else:require_ros_environment(robot_id,config)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    from .adapters.factory import create_runtime
    from .frame_adapter import FrameAdapter
    from .region_guard import RegionGuard
    motion=args.action in MOTION
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=adapter=executor=None
    started=False
    report=config_report(config,args.map_yaml,robot_id=robot_id)
    report['mode']='local_'+robot_id+'_'+args.action
    report['target_id']=getattr(args,'target_id',None) or ('B' if robot_id=='ranger' else 'A')
    latest={}; marker_pub=None; run_id=uuid.uuid4().hex; sequence=0
    generated_display={};generated_pubs=[]
    def record(event):
        nonlocal latest, sequence
        if events is None:return
        sequence+=1
        entry=config['alignment']['robots'][robot_id]
        if 'state' in event:
            latest=dict(event) if event['state']=='SELECTED' else dict(latest,**event)
        if event.get('state')=='CANDIDATE_GEOMETRY':
            generated_display.clear()
            generated_display.update(points=event['candidates'],records={},selectable={},selected=None)
        elif event.get('state')=='CANDIDATE_REQUEST':
            generated_display.update(selectable={p['id']:p['display_id'] for p in event['candidates']},
                                     records=event['records'],selected=None)
        elif event.get('state')=='CANDIDATE_SELECTED':generated_display['selected']=event['viewpoint_id']
        elif event.get('state')=='OBSERVATION_RECORD':generated_display['records']=event['records']
        if event.get('event')=='SESSION_END' or event.get('state') in ('STOPPING','STOP_UNCONFIRMED'):
            generated_display['selectable']={}
        row=dict(latest,**event)
        row.update(run_id=run_id,sequence=sequence,wall_time=time.time(),
            local_map_id=entry['local_map_id'],map_version=entry['map_version'],
            common_frame_id=config['alignment']['common_frame_id'],robot_id=robot_id,
            target_id=getattr(args,'target_id',None) or ('B' if robot_id=='ranger' else 'A'),target=args.target)
        try:
            events.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            events.flush();os.fsync(events.fileno())
        except OSError:
            report['evidence_write_failed']=True
            if adapter:
                adapter.evidence_write_failed=True
                adapter.motion_fault=True
        if marker_pub and 'state' in event:publish_marker()
    def publish_marker():
        from .go2_exploration import exploration_markers,generated_exploration_markers
        markers=exploration_markers(latest)
        for marker in markers.markers:marker.header.stamp=node.get_clock().now().to_msg()
        marker_pub.publish(markers)
        for publisher,markers in zip(generated_pubs,generated_exploration_markers(generated_display)):
            for marker in markers.markers:marker.header.stamp=node.get_clock().now().to_msg()
            publisher.publish(markers)
    try:
        node,adapter=create_runtime(robot_id,config,read_only=not motion,
            node_name=f'multi_agent_{robot_id}_check_motion' if motion else f'multi_agent_{robot_id}_check_readonly',
            global_goal_checks=args.action=='explore')
        entry=config['alignment']['robots'][robot_id]
        frame=FrameAdapter(entry['local_map_id'],entry['map_version'],config['alignment']['common_frame_id'],
            config['alignment']['alignment_id'],entry['transform'])
        adapter.record_event=record
        if args.action=='explore':
            adapter.scan_evidence_dir=Path(args.output_dir)/'vlm'
            adapter.selection_evidence_dir=Path(args.output_dir)/'selection'
            report['vlm_evidence_root']=str(adapter.scan_evidence_dir)
        executor=SingleThreadedExecutor();executor.add_node(node)
        if args.action=='explore':
            from visualization_msgs.msg import MarkerArray
            from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
            marker_pub=node.create_publisher(MarkerArray,'/vlm_nav/'+robot_id+'_exploration/current_point',
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
            for topic in ('generated_candidates','selected_candidate'):
                generated_pubs.append(node.create_publisher(MarkerArray,'/vlm_nav/'+robot_id+'_exploration/'+topic,
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
            publish_marker()
            node.create_timer(.5,publish_marker)
        parameters=node.create_client(GetParameters,'/map_server/get_parameters')
        active_map=MapParameterRead(parameters,GetParameters.Request(names=['yaml_filename']))
        def spin():
            executor.spin_once(timeout_sec=.05)
            active_map.poll(time.monotonic())
            if started:adapter.tick()
            elif getattr(adapter, 'vision', None) is not None:adapter.vision.poll()
        deadline=time.monotonic()+args.timeout
        # Initial data collection is passive, even for an explicit future motion action.
        until=min(deadline,time.monotonic()+(args.timeout if args.action=='inspect' else 2.))
        began=time.monotonic();next_sample=began;last_checks=None
        input_checks=('localization_valid','local_costmap','rgbd_ready','depth_units_verified','odom_fresh')
        window=dict(samples=0,first_input_ready_elapsed_s=None,post_input_ready_samples=0,
            post_input_ready_observed_s=0.,minimum_observation_s=2.,
            failure_counts={},post_input_ready_failure_counts={},transitions=[])
        while time.monotonic()<until:
            spin()
            now=time.monotonic()
            if args.action!='inspect' or now<next_sample:continue
            next_sample=now+.1
            sample=adapter.get_health();details=getattr(adapter,'last_health_details',{})
            current_owners=other_owners(node,robot_id)
            checks=dict(details.get('checks',{}),localization_valid=sample.localization_valid,
                nav_ready=sample.nav_ready,stopped=sample.stopped,motion_terminal=sample.motion_terminal,
                local_costmap=adapter.costmap_transform() is not None,
                rgbd_ready=node.vlm_input_ready(now),depth_units_verified=node.depth_units_verified,
                odom_fresh=details.get('odom_age_s',float('inf'))<=details.get('odom_max_age_s',0.),
                no_other_goal_owner=not current_owners)
            failed=[k for k,v in checks.items() if not v]
            window['samples']+=1
            if window['first_input_ready_elapsed_s'] is None and all(checks[k] for k in input_checks):
                window['first_input_ready_elapsed_s']=now-began
            if window['first_input_ready_elapsed_s'] is not None:
                window['post_input_ready_samples']+=1
                window['post_input_ready_observed_s']=now-began-window['first_input_ready_elapsed_s']
            for key in failed:
                window['failure_counts'][key]=window['failure_counts'].get(key,0)+1
                if window['first_input_ready_elapsed_s'] is not None and key in input_checks:
                    counts=window['post_input_ready_failure_counts'];counts[key]=counts.get(key,0)+1
            if checks!=last_checks:
                window['transitions'].append(dict(elapsed_s=now-began,failed_gates=failed,
                    health_details=details,other_goal_owners=current_owners))
                window['transitions']=window['transitions'][-50:]
                last_checks=checks
        if args.action=='inspect':
            window['non_motion_inputs_stable']=(window['first_input_ready_elapsed_s'] is not None
                and window['post_input_ready_samples']>1
                and window['post_input_ready_observed_s']>=window['minimum_observation_s']
                and not window['post_input_ready_failure_counts'])
            report['health_window']=window
        if motion and (not node.vlm_input_ready(time.monotonic()) or not active_map.path):
            record({'state':'WAITING_INPUTS','reason':'waiting for RGB-D/image-time TF and map_server parameter','motion_sent':False})
            # A new TF buffer may need longer than the initial two-second collection.
            while time.monotonic()<deadline and (not node.vlm_input_ready(time.monotonic()) or not active_map.path):spin()
        actual_path=active_map.path
        configured=config['robot_'+robot_id].get('map_path')
        map_matches=bool(actual_path and configured and actual_path==str(Path(configured).expanduser().resolve()))
        health=adapter.get_health()
        report.update(health=asdict(health),health_details=getattr(adapter,'last_health_details',{}),active_map_yaml=actual_path,active_map_matches=map_matches,
            map_parameter_read=dict(status=active_map.status,attempts=active_map.attempts),
            costmap_valid=adapter.costmap_transform() is not None,
            costmap_frame=adapter.costmap.header.frame_id if adapter.costmap else None,
            costmap_shape=[adapter.costmap.metadata.size_x,adapter.costmap.metadata.size_y] if adapter.costmap else None,
            depth_units_verified=node.depth_units_verified,rgbd_ready=node.vlm_input_ready(time.monotonic()),
            platform_readiness=adapter.readiness_checks(time.monotonic()) if hasattr(adapter,'readiness_checks') else {},
            own_motion_terminal=health.motion_terminal,
            note='own_motion_terminal tracks only this process; inspect does not cancel or certify other clients')
        readonly_ready=inspection_ready(report,health,robot_id=robot_id,owners=other_owners(node,robot_id))
        if args.action=='inspect':
            report['local_observation_ready']=readonly_ready
            if args.goal:
                probe=Pose2D(*args.goal)
                report['costmap_probe']={'local_map_pose':asdict(probe),
                    'footprint_free':adapter.costmap_candidate_free(probe)
                    if report['costmap_valid'] and health.footprint else None}
            return 0 if readonly_ready or not args.require_ready else 1
        gates=robot_local_gates(config,robot_id,motion=motion)
        if args.action=='explore':
            report.update(goal_costmap_topic=config['robot_'+robot_id].get('goal_costmap_topic'),
                goal_costmap_max_age=config['robot_'+robot_id].get('goal_costmap_max_age',1.),
                goal_costmap_valid=adapter.costmap_transform(global_goal=True) is not None)
            if not report['goal_costmap_valid']:gates.append('fresh global goal costmap/TF required')
        if motion and time.monotonic()>=deadline:gates.append('startup input wait timed out')
        if not map_matches:gates.append('active map_server YAML does not match configured map_path')
        if not health.localization_valid or not health.stopped or adapter.costmap_transform() is None:
            gates.append('fresh localization/stationary/realtime costmap required')
        if motion and not health.nav_ready:gates.append('existing NAV_READY/CONTROL_ARMED/RGB-D/action gates closed')
        if motion:
            owners=other_owners(node,robot_id)
            if owners:gates.append('other upper-level goal owner nodes present: '+', '.join(owners))
        if args.action in ('navigate','explore') and not config['robot_'+robot_id].get('path_topic'):
            gates.append('executing/replanned path topic required')
        if gates:
            report['rejected']=gates;record({'state':'REJECTED','reasons':gates,'motion_sent':False});return 1
        if args.action=='explore':
            from .adapters.go2 import ABCClient
            from .go2_exploration import explore,exploration_gates
            extra=exploration_gates(config,robot_id)
            if extra:
                report['rejected']=extra;record({'state':'REJECTED','reasons':extra,'motion_sent':False});return 1
            adapter.client=ABCClient(dict(config['robot_'+robot_id],vision_mode='abc' if
                config['robot_'+robot_id].get('exploration_strategy')=='generated_candidates_vlm' else 'single'))
            def before_motion():
                nonlocal started
                started=True;report['motion_sent']=True
            report['result']=explore(adapter,spin,config,frame,deadline,args.target,record,
                lambda:other_owners(node,robot_id),lambda:stop_owned(adapter,spin,config['robot_'+robot_id].get('cancel_timeout',5.),record),
                before_motion,clock=time.monotonic,robot_id=robot_id,
                target_id=getattr(args,'target_id',None) or ('B' if robot_id=='ranger' else 'A'))
            return 0 if not args.approach_target or report['result']=='TARGET_APPROACHED' else 1
        if args.action=='scan':
            from .adapters.go2 import ABCClient
            if any(not isinstance(config['scene']['targets'].get(t),str) or '现场填写' in config['scene']['targets'][t]
                   or not config['scene']['targets'][t].strip() for t in ('A','B','C')):
                report['rejected']=['fill actual A/B/C descriptions before scan'];return 1
            adapter.client=ABCClient(config['robot_'+robot_id])  # validate credentials/model before any Spin; no API call here
        goal=Pose2D(*args.goal) if args.goal else None
        future=None
        if args.action in ('plan','approach','navigate'):
            future=adapter.select_approach(goal,[]) if args.action=='approach' else adapter.compute_path(goal)
            while not future.done() and time.monotonic()<deadline:spin()
            if not future.done():future.cancel();report['result']='planning timeout';return 1
            result=future.result();report['preflight']=asdict(result)
            if args.action!='navigate':return 0 if (result.goal if args.action=='approach' else result.reachable) else 1
            if not result.reachable:return 1
        guard=RegionGuard(config['scene']['resources'],config['robot_'+robot_id]['stop_margin'])
        # This entry has no resource authority: every protected region remains unpermitted.
        health=adapter.get_health()
        if not health.nav_ready or not health.localization_valid or not health.stopped or not health.motion_terminal or adapter.costmap_transform() is None or other_owners(node,robot_id):
            report['rejected']=['motion prerequisites changed during preflight'];return 1
        check_path=result.path if args.action=='navigate' else [health.pose]
        decision=guard.check(health.pose,health.footprint,check_path,{},replace(health,path_valid=True))
        if decision.action!='ALLOW':report['rejected']=[decision.reason or 'protected resource requires real coordinator permit'];return 1
        started=True
        began=time.monotonic();acquired=False
        if args.action=='navigate':adapter.start_navigation(goal,robot_id+'_acceptance',uuid.uuid4().hex,'WAIT_NAV')
        else:future=adapter.start_scan(config['scene']['targets'],uuid.uuid4().hex)
        report['motion_sent']=True
        record({'state':'NAVIGATING' if args.action=='navigate' else 'SCANNING'})
        while time.monotonic()<deadline:
            spin();health=adapter.get_health()
            if not health.nav_ready or not health.localization_valid or other_owners(node,robot_id):
                report['result']='health lost';return 1
            if args.cancel_after is not None and time.monotonic()-began>=args.cancel_after:
                report['result']='explicit cancellation requested';return 0
            if args.action=='navigate':
                if adapter.results:
                    result=adapter.results.popleft();report['result']=asdict(result)
                    record(dict(state=result.status,event='NAVIGATION_RESULT',**asdict(result)))
                    return 0 if result.status=='SUCCEEDED' else 1
                if health.path_valid and health.path:acquired=True
                waiting=not acquired and health.stopped and time.monotonic()-began<config['robot_'+robot_id].get('path_start_timeout',1.)
                path=[health.pose] if waiting else health.path
                checked=replace(health,path_valid=True) if waiting else health
                if guard.check(health.pose,health.footprint,path,{},checked).action!='ALLOW':
                    report['result']='executing path/resource guard failed';return 1
            else:
                if guard.check(health.pose,health.footprint,[health.pose],{},replace(health,path_valid=True)).action!='ALLOW':
                    report['result']='spin stopping envelope failed';return 1
                if future.done():
                    observations=future.result();report['observations']=[asdict(o) for o in observations]
                    return 0 if {o.target_id for o in observations}==set('ABC') and all(o.status!='API_ERROR' for o in observations) else 1
        report['result']='action timeout';return 1
    except KeyboardInterrupt:
        report['result']='operator interrupt';record({'state':'STOPPING','reason':'operator interrupt'});return 1
    except Exception as error:
        from .go2_exploration import ExplorationStopped
        reason=str(error) if isinstance(error,ExplorationStopped) else type(error).__name__
        report['result']=reason;record({'state':'STOPPING','reason':reason});return 1
    finally:
        if started and adapter:
            report['last_native_action_uuid']=getattr(adapter,'last_action_uuid','')
            try:report['stop_confirmed']=stop_owned(adapter,spin,config['robot_'+robot_id].get('cancel_timeout',5.),record)
            except Exception as error:
                report['stop_confirmed']=False
                report['stop_error']=type(error).__name__
            report['evidence_write_failed']=getattr(adapter,'evidence_write_failed',False)
        if args.action=='explore' and adapter:
            job=getattr(getattr(adapter,'vision',None),'job',None)
            if hasattr(args,'_evidence_lease'):args._evidence_lease['pending']=job
            report['vlm_evidence_pending']=job is not None and not job.done()
            for handle in adapter.plans.values():
                if handle:handle.cancel_goal_async()
        if events is not None:
            state=latest.get('state','STOPPED')
            if state=='STOPPING' and report.get('stop_confirmed'):state='STOPPED'
            record({'state':'STOP_UNCONFIRMED' if started and not report['stop_confirmed'] else state,
                'event':'SESSION_END','result':report.get('result'),
                'motion_sent':report.get('motion_sent',False),
                'stop_confirmed':report.get('stop_confirmed') if started else None})
            report['evidence_write_failed']=report.get('evidence_write_failed',False) or getattr(adapter,'evidence_write_failed',False)
            try:
                (Path(args.output_dir)/'result.json').write_text(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
            except OSError:report['evidence_write_failed']=True
        if adapter:adapter.close(stop=False)
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        emit(report)
        if started and not report['stop_confirmed']:
            raise StopUnconfirmed('action terminal and physical stop were not both confirmed')

def main(argv=None,*,robot_id='go2'):
    p=parser();args=p.parse_args(argv)
    args.target_id=args.target_id or ('B' if robot_id=='ranger' else 'A')
    if not math.isfinite(args.timeout) or args.timeout<=0:p.error('positive finite timeout required')
    if args.action in MOTION and not args.allow_motion:p.error('motion requires --allow-motion and an explicit action')
    if args.offline and args.action!='inspect':p.error('--offline supports inspection only')
    if (args.approach_target or args.search_hint) and args.action!='explore':
        p.error('--approach-target/--search-hint require --action explore')
    if args.action=='explore' and (args.goal or args.cancel_after is not None):
        p.error('explore requires --output-dir, configured viewpoints, and no --goal/--cancel-after')
    if (args.goal and not all(math.isfinite(v) for v in args.goal)) or (args.action in ('plan','approach','navigate') and not args.goal):
        p.error('finite measured --goal X Y YAW required')
    if args.cancel_after is not None and (args.action not in MOTION or not math.isfinite(args.cancel_after) or not 0<args.cancel_after<args.timeout):
        p.error('--cancel-after requires motion and 0 < seconds < timeout')
    try:
        config=load_config(args.config_dir)
        if robot_id=='ranger':
            from .local_context import local_config
            config=local_config(config,robot_id)
        if args.target is not None:
            config['scene'].setdefault('targets',{})[args.target_id]=args.target
        args.target=args.target or config['scene'].get('targets',{}).get(args.target_id)
        if args.strategy:config['robot_'+robot_id]['exploration_strategy'] = args.strategy
        if args.approach_target:config['robot_'+robot_id]['approach_on_detection'] = True
        if args.search_hint:config['robot_'+robot_id]['search_hint'] = args.search_hint
        if args.action=='explore':
            from .go2_exploration import measured_viewpoints
            from .semantic_search import load_catalog
            semantic = config['robot_'+robot_id].get('exploration_strategy') in ('semantic_furniture','generated_candidates_vlm')
            if not args.output_dir:
                if config['robot_'+robot_id].get('exploration_strategy')!='generated_candidates_vlm':
                    p.error('fixed/semantic exploration requires --output-dir')
                root=Path(config['robot_'+robot_id]['experiment_records_root'])
                args.output_dir=str(root/('experiment.'+time.strftime('%Y%m%d_%H%M%S')+'.'+uuid.uuid4().hex[:8]))
            if not semantic:measured_viewpoints(config,robot_id)
            args.target=args.target or config['scene']['targets'][args.target_id]
            if not args.target.strip() or '现场填写' in args.target:
                raise ValueError('measured exploration target description required')
        if args.offline:
            report=config_report(config,args.map_yaml,robot_id=robot_id);report['target_id']=args.target_id;report['target']=args.target;emit(report)
            return 1 if args.require_ready and report['single_robot_experiment_gates'] else 0
        if args.action in MOTION:
            if args.output_dir:
                from .experiment_records import experiment_bundle
                root=config['robot_'+robot_id].get('experiment_records_root',str(Path(args.output_dir).parent / 'managed-experiments'))
                with experiment_bundle(args.output_dir,root) as lease:
                    args._evidence_lease=lease
                    with (Path(args.output_dir)/'events.jsonl').open('x') as events:
                        from .go2_exploration import exploration_gates
                        reasons=exploration_gates(config,robot_id) if args.action=='explore' else robot_local_gates(config,robot_id,motion=True)
                        if args.action=='navigate' and not config['robot_'+robot_id].get('path_topic'):
                            reasons.append('executing/replanned path topic required')
                        if not reasons:
                            try:
                                if robot_id=='go2':require_ros_environment()
                                else:require_ros_environment(robot_id,config)
                            except Exception as error:
                                reasons=[str(error) if isinstance(error,ValueError) else type(error).__name__]
                        if reasons:
                            report=config_report(config,robot_id=robot_id)
                            report.update(mode='local_'+robot_id+'_'+args.action,rejected=reasons,target=args.target,target_id=args.target_id)
                            if args.action=='explore':
                                report['viewpoints'] = (report.get('semantic_search') or {}).get('points', []) if semantic else measured_viewpoints(config,robot_id)
                            events.write(json.dumps({'state':'REJECTED','wall_time':time.time(),
                                'reasons':reasons,'motion_sent':False,'target':args.target,'target_id':args.target_id,'robot_id':robot_id,
                                'common_frame_id':config['alignment']['common_frame_id'],'viewpoints':report.get('viewpoints')},ensure_ascii=False)+'\n')
                            events.flush();os.fsync(events.fileno())
                            (Path(args.output_dir)/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
                            emit(report);return 1
                        with (motion_slot() if robot_id=='go2' else motion_slot(robot_id)):
                            return run_ros(config,args,events) if robot_id=='go2' else run_ros(config,args,events,robot_id=robot_id)
            with (motion_slot() if robot_id=='go2' else motion_slot(robot_id)):
                return run_ros(config,args) if robot_id=='go2' else run_ros(config,args,robot_id=robot_id)
        return run_ros(config,args) if robot_id=='go2' else run_ros(config,args,robot_id=robot_id)
    except Exception as error:
        import sys
        print(robot_id+' check failed: '+(str(error) if isinstance(error,(ValueError,FileNotFoundError)) else type(error).__name__),file=sys.stderr)
        return 1

if __name__=='__main__':raise SystemExit(main())
