import math
import time
from types import SimpleNamespace as S
from concurrent.futures import Future
import numpy as np
import pytest
from multi_agent_nav.config import load_config, live_gates,go2_local_gates,map_file_info
from multi_agent_nav.adapters.go2 import Go2Adapter
from multi_agent_nav.models import Pose2D,PathResult

def stamp(seconds):
    from builtin_interfaces.msg import Time
    return Time(sec=int(seconds),nanosec=int((seconds-int(seconds))*1e9))

def costmap(cost=0, yaw=0):
    return S(header=S(frame_id='odom',stamp=stamp(10)),
        metadata=S(size_x=80,size_y=80,resolution=.1,
            origin=S(position=S(x=-4,y=-4),orientation=S(x=0,y=0,z=math.sin(yaw/2),w=math.cos(yaw/2)))),
        data=[cost]*6400)

def adapter():
    a=Go2Adapter.__new__(Go2Adapter)
    a.config={'footprint':[[.38,.18],[.38,-.18],[-.38,-.18],[-.38,.18]],
              'standoff_distance':.9,'costmap_max_age':1.,'costmap_blocked_cost':253,
              'odom_max_age':.5,'stationary_speed':.02,
              'stationary_linear_speed':.02,'stationary_angular_speed':.03}
    origin=S(position=S(x=-4,y=-4),orientation=S(x=0,y=0,z=0,w=1))
    a.node=S(get_clock=lambda:S(now=lambda:S(nanoseconds=10_000_000_000)),
        grid=np.zeros((80,80)),map_message=S(info=S(origin=origin,resolution=.1)),
        last_behavior_costmap_received=time.monotonic(),
        tf_buffer=S(lookup_transform=lambda *args:S(transform=S(
            translation=S(x=0,y=0),rotation=S(x=0,y=0,z=0,w=1)))))
    a.costmap=None;a.costmap_at=0;a.global_costmap=None;a.global_costmap_at=0
    a.get_health=lambda:S(pose=Pose2D())
    return a

def test_explicit_missing_config_is_not_silently_replaced(tmp_path):
    with pytest.raises(FileNotFoundError):load_config(tmp_path/'typo')

def test_live_gates_reject_incomplete_scene_and_unverified_costmap():
    c=load_config();c['scene']['enabled']=True
    c['scene']['share_c_coordinates']=True
    c['robot_go2']['costmap_source_verified']=False
    c['scene']['resources']={'door':{'polygon':[[0,0],[1,0],[1,1]]}}
    c['scene']['exit_points']={'go2':[2,2,0]}
    errors=live_gates(c)
    assert any('basket' in e for e in errors)
    assert any('ranger' in e and ('exit' in e or 'viewpoint' in e) for e in errors)
    assert any('costmap_source_verified' in e for e in errors)

def test_empty_resources_allow_independent_exploration_but_not_cooperative_live():
    from multi_agent_nav.go2_exploration import exploration_gates
    c=load_config()
    c['scene']['resources']={}
    c['robot_go2'].update(stop_margin=.1,costmap_source_verified=True,
        camera_calibrated=True,path_topic='/fixture/path',path_source_verified=True)
    assert exploration_gates(c)==[]
    assert any('basket' in e for e in live_gates(c))
    c['robot_go2']['stop_margin']=None
    assert any('stop margin' in e for e in exploration_gates(c))

@pytest.mark.parametrize('resources',[None,[],{'door':{'polygon':[[0,0],[1,0]]}}])
def test_independent_motion_rejects_missing_or_malformed_resources(resources):
    c=load_config();c['scene']['resources']=resources;c['coordinate_mode']='local_only'
    assert any('resource' in e for e in go2_local_gates(c,motion=True))

def test_actual_costmap_vetoes_static_free_map_and_bad_updates():
    a=adapter()
    a.on_costmap(costmap(253))
    called=[]
    def path(goal):
        called.append(goal)
        f=Future();f.set_result(S(reachable=True));return f
    a.compute_path=path
    result=a.select_approach(Pose2D(),[]).result()
    assert result.goal is None and not called
    a.on_costmap(costmap())
    assert a.costmap_candidate_free(Pose2D())
    edge=costmap();edge.data[40*80+42]=255  # Unknown under the body, away from center.
    a.on_costmap(edge)
    assert not a.costmap_candidate_free(Pose2D())
    a.on_costmap(costmap(255))
    assert not a.costmap_candidate_free(Pose2D())
    bad=costmap();bad.data=[]
    a.on_costmap(bad)
    assert not a.costmap_candidate_free(Pose2D())
    old=costmap();old.header.stamp=stamp(1)
    a.on_costmap(old)
    assert not a.costmap_candidate_free(Pose2D())

def test_costmap_coordinates_include_tf_and_rotated_grid_origin():
    a=adapter();m=costmap(yaw=math.pi/2)
    m.metadata.origin.position.x=4
    a.on_costmap(m)
    assert a.costmap_candidate_free(Pose2D())
    a.node.tf_buffer.lookup_transform=lambda *args:S(transform=S(
        translation=S(x=20,y=0),rotation=S(x=0,y=0,z=0,w=1)))
    assert not a.costmap_candidate_free(Pose2D())

def test_actual_footprint_is_not_snapped_to_grid_cell_center():
    a=adapter();m=costmap();m.data[40*80+44]=254
    a.on_costmap(m)
    assert not a.costmap_candidate_free(Pose2D(.099,0,0))

def test_far_goal_uses_global_grid_and_does_not_replace_local_grid():
    a=adapter();local=costmap()
    local.metadata.size_x=local.metadata.size_y=30
    local.metadata.origin.position.x=local.metadata.origin.position.y=-1.5
    local.data=[0]*900
    a.on_costmap(local)
    global_grid=costmap();global_grid.header.frame_id='map'
    a.on_costmap(global_grid,global_goal=True)
    assert not a.costmap_candidate_free(Pose2D(3.,0.))
    assert a.costmap_candidate_free(Pose2D(3.,0.),global_goal=True)
    assert a.costmap is local and a.global_costmap is global_grid
    assert not a.costmap_candidate_free(Pose2D(4.,0.),global_goal=True)

@pytest.mark.parametrize('value',[253,254,255])
def test_global_goal_preserves_obstacle_and_unknown_veto(value):
    a=adapter();a.on_costmap(costmap())
    grid=costmap();grid.header.frame_id='map';grid.data[40*80+72]=value
    a.on_costmap(grid,global_goal=True)
    assert not a.costmap_candidate_free(Pose2D(3.,0.),global_goal=True)
    assert a.costmap_candidate_free(Pose2D())


def test_global_goal_header_age_uses_its_own_budget():
    a=adapter();a.config['goal_costmap_max_age']=3.
    a.node.get_clock=lambda:S(now=lambda:S(nanoseconds=12_000_000_000))
    grid=costmap();grid.header.frame_id='map'
    a.on_costmap(grid,global_goal=True)
    a.on_costmap(grid)
    assert a.costmap_candidate_free(Pose2D(),global_goal=True)
    assert a.costmap_transform() is None
    a.node.get_clock=lambda:S(now=lambda:S(nanoseconds=13_001_000_000))
    assert a.costmap_transform(global_goal=True) is None
    a.on_costmap(grid,global_goal=True)
    assert a.global_costmap is None


def test_global_goal_receipt_age_expires_without_cache_reads_renewing(monkeypatch):
    now=[10.]
    monkeypatch.setattr('multi_agent_nav.adapters.go2.time.monotonic',lambda:now[0])
    a=adapter();a.config['goal_costmap_max_age']=3.
    grid=costmap();grid.header.frame_id='map'
    a.on_costmap(grid,global_goal=True);a.on_costmap(grid)
    now[0]=12.
    assert a.costmap_candidate_free(Pose2D(),global_goal=True)
    assert a.global_costmap_at==10.
    assert a.costmap_transform() is None
    now[0]=13.001
    assert a.costmap_transform(global_goal=True) is None


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf'),True,None])
def test_global_goal_age_config_must_be_positive_finite(value):
    c=load_config();c['robot_go2']['goal_costmap_max_age']=value
    assert 'Go2 positive finite goal_costmap_max_age required' in go2_local_gates(c,motion=True)

@pytest.mark.parametrize('fault',['missing','payload','stamp','receipt','tf'])
def test_bad_global_goal_grid_blocks_without_invalidating_local_health(fault):
    a=adapter();a.on_costmap(costmap())
    if fault!='missing':
        grid=costmap();grid.header.frame_id='map'
        a.on_costmap(costmap(),global_goal=True)
        if fault=='payload':grid.data=[]
        if fault=='stamp':grid.header.stamp=stamp(1.)
        if fault=='tf':grid.header.frame_id='missing_frame'
        a.on_costmap(grid,global_goal=True)
        if fault=='receipt':a.global_costmap_at-=2.
        if fault=='tf':
            def missing_tf(*args):raise RuntimeError('unit missing TF')
            a.node.tf_buffer.lookup_transform=missing_tf
            a.costmap.header.frame_id='map'
    assert a.costmap_transform(global_goal=True) is None
    assert not a.costmap_candidate_free(Pose2D(),global_goal=True)
    assert a.costmap_transform() is not None

@pytest.mark.parametrize('enabled',[False,True])
def test_global_goal_subscription_only_enabled_for_exploration(enabled):
    subscriptions=[]
    node=S(create_subscription=lambda typ,topic,callback,qos:subscriptions.append((topic,qos)))
    a=Go2Adapter(node,dict(api_timeout=1.,goal_costmap_topic='/global_costmap/costmap_raw'),None,
        global_goal_checks=enabled)
    try:
        topics=[topic for topic,qos in subscriptions]
        assert ('/global_costmap/costmap_raw' in topics)==enabled
        if enabled:
            from rclpy.qos import DurabilityPolicy,ReliabilityPolicy
            qos=next(qos for topic,qos in subscriptions if topic=='/global_costmap/costmap_raw')
            assert qos.durability==DurabilityPolicy.TRANSIENT_LOCAL and qos.reliability==ReliabilityPolicy.RELIABLE
    finally:a.vision.close()

def test_approach_rechecks_costmap_when_planning_finishes():
    a=adapter();a.on_costmap(costmap())
    pending=Future();a.compute_path=lambda goal:pending
    result=a.select_approach(Pose2D(),[])
    a.on_costmap(costmap(254))
    pending.set_result(PathResult(True,[Pose2D(),Pose2D(1.,0.)]))
    assert result.result().goal is None

def test_stationary_uses_linear_norm_and_separate_angular_limit():
    a=adapter();a.stationary_since=None;a.odom_at=0
    v=S(linear=S(x=.019,y=.019,z=0),angular=S(x=0,y=0,z=0))
    m=S(header=S(stamp=stamp(10)),twist=S(twist=v))
    a.on_odometry(m)
    assert a.stationary_since is None
    v.linear.x=v.linear.y=0;v.angular.z=.025
    a.on_odometry(m)
    assert a.stationary_since is not None

def test_readonly_cleanup_sends_neither_cancel_nor_zero_velocity():
    a=adapter();calls=[]
    a.vision=S(close=lambda:calls.append('vision'))
    a.stop_motion=lambda reason:calls.append('motion')
    a.close(stop=False)
    assert calls==['vision']

def test_check_defaults_to_inspection_and_motion_requires_explicit_opt_in(capsys):
    from multi_agent_nav.go2_check import parser, main
    assert parser().parse_args([]).action=='inspect'
    assert main(['--offline'])==0
    assert 'live_gates' in capsys.readouterr().out
    with pytest.raises(SystemExit):main(['--action','navigate','--goal','1','2','0'])
    with pytest.raises(SystemExit):main(['--offline','--action','scan','--allow-motion'])

def test_map_file_digest_changes_when_image_changes(tmp_path):
    path=tmp_path/'map.yaml';image=tmp_path/'map.pgm'
    path.write_text('image: map.pgm\n');image.write_bytes(b'original')
    c=load_config();c['robot_go2'].update(map_path=str(path),map_digest=map_file_info(path)['map_digest'])
    assert not any('digest' in e for e in go2_local_gates(c))
    image.write_bytes(b'rebuilt')
    assert any('digest' in e for e in go2_local_gates(c))

def test_old_or_nonfinite_path_cannot_become_current_executing_path():
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    a=adapter();a.path_after_stamp=10.;a.path=[]
    a.motions={'nav':{'handle':object(),'cancel':False,'kind':'VIEWPOINT_NAV'}}
    m=Path();m.header.frame_id='map';m.header.stamp=stamp(9.9)
    m.poses=[PoseStamped()]
    a.on_path_update(m)
    assert not a.path
    m.header.stamp=stamp(10);m.poses[0].pose.position.x=float('nan')
    a.on_path_update(m)
    assert not a.path

def test_readonly_observer_has_no_velocity_publisher_and_adapter_rejects_motion(monkeypatch,tmp_path):
    import rclpy
    from multi_agent_nav.adapters.go2 import make_observer
    monkeypatch.setenv('ROS_DOMAIN_ID','31');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    rclpy.init(args=[])
    node=make_observer(load_config()['robot_go2'],read_only=True)
    try:
        assert node.stop_pub is None
        assert '/cmd_vel_bridge' not in [p.topic_name for p in node.publishers]
        a=adapter();a.read_only=True
        with pytest.raises(RuntimeError):a._motion(None,None,'t','c','WAIT_NAV')
        with pytest.raises(RuntimeError):a.start_scan({},'s')
        with pytest.raises(RuntimeError):a.stop_motion('test')
    finally:node.destroy_node();rclpy.shutdown()

def test_motion_config_cannot_disable_costmap_obstacles_or_stationary_check():
    c=load_config();c['robot_go2'].update(costmap_blocked_cost=256,stationary_duration=0)
    errors=go2_local_gates(c,motion=True)
    assert any('costmap_blocked_cost' in e for e in errors)
    assert any('stationary_duration' in e for e in errors)

def test_independent_cleanup_requires_both_physical_stop_and_action_terminal():
    from multi_agent_nav.go2_check import stop_owned
    calls=[]
    a=S(stop_motion=lambda reason:calls.append('cancel/zero'),
        get_health=lambda:S(stopped=True,motion_terminal=False))
    assert not stop_owned(a,lambda:time.sleep(.001),.003)
    a.get_health=lambda:S(stopped=False,motion_terminal=True)
    assert not stop_owned(a,lambda:time.sleep(.001),.003)
    a.get_health=lambda:S(stopped=True,motion_terminal=True)
    assert stop_owned(a,lambda:None,.003)
    assert len(calls)==3

def test_full_gates_only_read_the_current_pcs_map_files():
    c=load_config()
    for robot in ('go2','ranger'):
        c[f'robot_{robot}'].update(map_path='/absent/'+robot+'.yaml',map_digest='remote-digest')
    go2_errors=live_gates(c,local_robot='go2')
    ranger_errors=live_gates(c,local_robot='ranger')
    assert any('go2: readable' in e for e in go2_errors)
    assert not any('ranger: readable' in e for e in go2_errors)
    assert any('ranger: readable' in e for e in ranger_errors)
    assert not any('go2: readable' in e for e in ranger_errors)

@pytest.mark.parametrize('action,owners,terminal,change,expected',[
    ('inspect',[],True,'',0),('inspect',[],True,'rgbd_blink',0),
    ('inspect',[],True,'rgbd_late',0),('inspect',[],True,'arm_stale',0),
    ('inspect',[],True,'odom_stale',0),('navigate',[],True,'',0),
    ('navigate',['multi_agent_coordinator'],True,'',1),('navigate',['multi_agent_task_agent'],True,'',1),
    ('navigate',[],False,'',1),('navigate',[],True,'permit',1),
    ('navigate',[],True,'moving',1),('navigate',[],True,'terminal',1),
    ('navigate',[],True,'costmap',1),('navigate',[],True,'owner',1),
    ('navigate',[],True,'cleanup',1),('navigate',[],True,'stop_error',1),
    ('navigate',[],True,'path_finished',0),
    ('navigate',[],True,'path_timeout',1),
    ('navigate',[],True,'no_path_moving',1),
    ('navigate',[],True,'evidence_error',0),
    ('navigate',[],True,'rgbd_warmup',0),
    ('navigate',[],True,'rgbd_unavailable',1),
    ('navigate',[],True,'rgbd_deadline',1),
    ('navigate',[],True,'no_path_stopped',1)])
def test_independent_entry_runtime_with_one_stub_go2_no_ros_or_peer(monkeypatch,tmp_path,capsys,action,owners,terminal,change,expected):
    """Exercise entry dispatch/cleanup with ROS initialization stubbed, never a live domain participant."""
    import json
    import rclpy
    import rclpy.executors
    import multi_agent_nav.go2_check as check
    import multi_agent_nav.adapters.go2 as go2
    from multi_agent_nav.adapters.fake import FakeAdapter
    from dataclasses import replace
    now=[0.];calls=[]
    monkeypatch.setenv('ROS_DOMAIN_ID','0');monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
    monkeypatch.setenv('GO2_NET_IFACE','enp4s0')
    from ament_index_python.packages import get_package_share_directory
    monkeypatch.setenv('CYCLONEDDS_URI','file://'+get_package_share_directory('vlm_nav')+'/config/cyclonedds_go2.xml')
    monkeypatch.setattr(rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(rclpy,'ok',lambda:False)
    monkeypatch.setattr(check,'time',S(monotonic=lambda:now[0],time=lambda:100.))
    c=load_config();path=tmp_path/'map.yaml';path.write_text('image: map.pgm\n')
    (tmp_path/'map.pgm').write_bytes(b'unit-fixture')
    c['alignment']['robots']['go2'].update(local_map_id='fixture',map_version='1')
    c['robot_go2'].update(map_path=str(path),map_digest=map_file_info(path)['map_digest'],
        costmap_source_verified=True,camera_calibrated=True,stop_margin=.1,
        path_topic='/unit/path',path_source_verified=False)
    c['scene']['resources']={'basket':{'polygon':[[100,100],[102,100],[102,102],[100,102]]}}
    f=Future();f.set_result(S(values=[S(string_value=str(path))]))
    def rgbd_ready(t):
        if change=='rgbd_blink':return not (5.<=t<5.5)
        if change=='rgbd_late':return t>=11.5
        if change=='rgbd_warmup':return t>=5.
        if change=='rgbd_deadline':return t>=12.
        return change!='rgbd_unavailable'
    node=S(nav_ready=True,control_armed=True,depth_units_verified=True,
        create_client=lambda *args:S(call_async=lambda request:f,service_is_ready=lambda:True),
        vlm_input_ready=rgbd_ready,destroy_node=lambda:None,get_node_names=lambda:owners)
    class StubGo2(FakeAdapter):
        def __init__(self,*args,**kwargs):
            super().__init__('go2',delay=20. if change in ('path_timeout','no_path_moving','no_path_stopped') else .5)
            self.costmap=None;self.planned=False;self.nav_started=None
        def costmap_transform(self):return None if self.planned and change=='costmap' else Pose2D()
        def costmap_candidate_free(self,candidate):return True
        def tick(self):
            self.advance(now[0])
            if change=='path_finished' and not self.active:self.path=[]
        def get_health(self):
            h=replace(super().get_health(),motion_terminal=terminal)
            self.last_health_details=dict(checks=dict(depth_units_verified=True,
                control_armed_gate=change!='arm_stale',external_gates_fresh=change!='arm_stale'),
                odom_age_s=10. if change=='odom_stale' else .05,odom_max_age_s=.5)
            h.nav_ready=h.nav_ready and node.vlm_input_ready(now[0])
            if self.planned and change=='permit':h.nav_ready=False
            if self.planned and change=='moving':h.stopped=False
            if self.planned and change=='terminal':h.motion_terminal=False
            if change=='cleanup' and 'navigate' in calls:h.motion_terminal=False
            if self.nav_started is not None and self.active:
                if change=='path_timeout' and now[0]-self.nav_started>5.:h.path_valid=False
                if change in ('no_path_moving','no_path_stopped'):
                    h.path=[];h.path_valid=False;h.stopped=change=='no_path_stopped'
            return h
        def compute_path(self,*args):
            f=super().compute_path(*args);self.planned=True
            if change=='owner':owners.append('multi_agent_task_agent')
            return f
        def start_navigation(self,*args):
            calls.append('navigate')
            self.nav_started=now[0]
            if change=='path_finished':self.now=now[0]
            return super().start_navigation(*args)
        def cancel_navigation(self,goal_id):
            calls.append('cancel')
            return super().cancel_navigation(goal_id)
        def stop_motion(self,reason):
            calls.append('stop')
            if change=='stop_error':raise RuntimeError('unit transport failure')
            return super().stop_motion(reason)
        def close(self,stop=True):calls.append(('close',stop))
    monkeypatch.setattr(go2,'make_observer',lambda *args,**kw:node)
    monkeypatch.setattr(go2,'Go2Adapter',StubGo2)
    def spin(**kw):now[0]+=.05
    monkeypatch.setattr(rclpy.executors,'SingleThreadedExecutor',lambda:S(
        add_node=lambda n:None,spin_once=spin,shutdown=lambda:None))
    args=check.parser().parse_args(['--action',action,'--timeout','12','--goal','1','0','0','--allow-motion'])
    events=None
    if change=='evidence_error':
        class BrokenEvidence:
            def write(self,line):raise OSError('unit disk failure')
        events=BrokenEvidence();args.output_dir=str(tmp_path)
        write_text=check.Path.write_text
        def failed_result_write(path,*a,**kw):
            if path.name=='result.json':raise OSError('unit disk failure')
            return write_text(path,*a,**kw)
        monkeypatch.setattr(check.Path,'write_text',failed_result_write)
    if change in ('cleanup','stop_error'):
        with pytest.raises(check.StopUnconfirmed):check.run_ros(c,args)
    else:assert check.run_ros(c,args,events)==expected
    report=json.loads(capsys.readouterr().out)
    if action=='inspect':
        assert report['costmap_probe']['footprint_free'] is True
        assert report['health_window']['samples']>10
        assert report['health_window']['non_motion_inputs_stable']==(change not in ('rgbd_blink','rgbd_late','odom_stale'))
        if change=='rgbd_blink':assert report['health_window']['post_input_ready_failure_counts']['rgbd_ready']>0
        if change=='rgbd_late':assert report['health_window']['post_input_ready_observed_s']<2.
        if change=='arm_stale':assert report['health_window']['failure_counts']['external_gates_fresh']>0
        if change=='odom_stale':assert report['health_window']['failure_counts']['odom_fresh']>0
    assert report['live_gates']  # Ranger/alignment remain incomplete, even for a local unit goal.
    if change=='evidence_error':
        assert report['evidence_write_failed'] and report['stop_confirmed']
        assert 'stop' in calls and ('close',False) in calls
    elif action=='inspect' or owners or not terminal or change not in ('','cleanup','stop_error','path_finished','path_timeout','no_path_moving','no_path_stopped','rgbd_warmup'):
        assert 'navigate' not in calls and 'stop' not in calls and not report['motion_sent']
    else:
        assert 'navigate' in calls and 'stop' in calls
        assert report['stop_confirmed']==(change not in ('cleanup','stop_error'))
        if change in ('path_timeout','no_path_moving','no_path_stopped'):
            assert 'cancel' in calls
            assert report['result']=='executing path/resource guard failed'
            if change=='path_timeout':assert now[0]-2.>5.
            if change=='no_path_stopped':assert now[0]-2.>=1.
    if change=='rgbd_warmup':assert now[0]>=5.
    if change in ('rgbd_unavailable','rgbd_deadline'):
        assert now[0]>=12. and not report['motion_sent']


def test_offline_single_robot_config_does_not_require_cooperative_certification(capsys):
    from multi_agent_nav.go2_check import main
    import json
    assert main(['--offline','--require-ready'])==0
    report=json.loads(capsys.readouterr().out)
    assert report['single_robot_experiment_gates']==[]
    assert any('path_source_verified' in reason for reason in report['live_gates'])
    assert report['path_monitoring']['receive_timeout_s']==5.
    assert report['path_monitoring']['plan_age_checked'] is False

@pytest.mark.parametrize('action',['navigate','scan'])
def test_rejected_motion_keeps_evidence_without_exploration_viewpoints(action,monkeypatch,tmp_path,capsys):
    import json
    import multi_agent_nav.go2_check as check
    config=load_config();config['robot_go2']['stop_margin']=None
    monkeypatch.setattr(check,'load_config',lambda _:config)
    output=tmp_path/'evidence'
    args=['--action',action,'--allow-motion','--output-dir',str(output)]
    if action=='navigate':args+=['--goal','1','0','0']
    assert check.main(args)==1
    report=json.loads((output/'result.json').read_text())
    assert report['rejected'] and not report['motion_sent']
    assert json.loads((output/'events.jsonl').read_text())['state']=='REJECTED'
    assert 'KeyError' not in capsys.readouterr().err

def test_stop_observation_write_failure_does_not_interrupt_confirmation(monkeypatch):
    import multi_agent_nav.go2_check as check
    now=[0.];calls=[]
    adapter=S(stop_motion=lambda reason:calls.append('stop'),
        get_health=lambda:S(stopped=now[0]>=.2,motion_terminal=now[0]>=.2,pose=Pose2D()))
    monkeypatch.setattr(check.time,'monotonic',lambda:now[0])
    def spin():now[0]+=.1
    def record(event):raise OSError('unit disk failure')
    assert check.stop_owned(adapter,spin,1.,record)
    assert calls==['stop'] and now[0]>=.2 and adapter.evidence_write_failed

def test_independent_motion_lock_excludes_second_entry_and_releases_after_exit():
    from multi_agent_nav.go2_check import motion_slot
    with motion_slot():
        with pytest.raises(RuntimeError):
            with motion_slot():pass
    with motion_slot():pass

def test_unconfirmed_stop_persists_motion_block(monkeypatch,tmp_path):
    import multi_agent_nav.go2_check as check
    monkeypatch.setattr(check,'MOTION_LOCK',str(tmp_path/'motion.lock'))
    with pytest.raises(check.StopUnconfirmed):
        with check.motion_slot():raise check.StopUnconfirmed('unit fault')
    with pytest.raises(RuntimeError):
        with check.motion_slot():pass
