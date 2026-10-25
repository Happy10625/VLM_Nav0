"""Local exploration sequencing; fake hardware never connects to live ROS."""
import json
from pathlib import Path
import pytest
from multi_agent_nav.adapters.fake import FakeAdapter, completed
from multi_agent_nav.models import Pose2D, PathResult
from multi_agent_nav.frame_adapter import FrameAdapter


class LocalFixture(FakeAdapter):
    def __init__(self):
        super().__init__('go2', delay=.1)
        self.blocked = set()
        self.no_path = set()
        self.scans = 0
        self.lost_during_motion = False

    def costmap_transform(self,global_goal=False):
        return object()

    def costmap_candidate_free(self, goal,global_goal=False):
        return goal.x not in self.blocked

    def compute_path(self, goal):
        return completed(PathResult(goal.x not in self.no_path, [self.pose, goal], 'planner rejected'))

    def start_scan(self, descriptions, scan_id):
        self.scans += 1
        return super().start_scan(descriptions, scan_id)


def run_case(adapter, events, stop_ok=True, resources=None, approach=False):
    from multi_agent_nav.go2_exploration import explore
    now = [0.]
    def spin():
        now[0] += .05
        adapter.advance(now[0])
        if adapter.lost_during_motion and adapter.active:
            adapter.online = False
    def stop():
        adapter.stop_motion('fixture cleanup')
        adapter.advance(now[0] + 1)
        return stop_ok
    config = {'scene': {'viewpoints': {'go2': [
        {'id': 'p1', 'pose': [1., 0., 0.]},
        {'id': 'p2', 'pose': [2., 0., 0.]},
        {'id': 'p3', 'pose': [3., 0., 0.]}]},
        'resources': {'basket': {'polygon': [[100,100],[102,100],[102,102],[100,102]]}}},
        'robot_go2': {'stop_margin': .1, 'path_start_timeout': 1., 'scan_angles': [0.], 'approach_on_detection': approach, 'standoff_distance': .9,
            'arrival_config_files': [str(Path(__file__).resolve().parents[4]/'config/robot.yaml')]}}
    if resources is not None:
        config['scene']['resources']=resources
    frame = FrameAdapter('fixture', '1', 'lab_ref', '', [0,0,0])
    return explore(adapter, spin, config, frame, 10., '灭火器', events.append,
        lambda: [], stop, lambda: None, clock=lambda: now[0])


def test_explore_requires_explicit_motion_and_new_output_directory(tmp_path):
    from multi_agent_nav.go2_check import main, parser
    assert parser().parse_args(['--action','explore','--output-dir',str(tmp_path/'new')]).action == 'explore'
    with pytest.raises(SystemExit):
        main(['--action','explore','--output-dir',str(tmp_path/'new')])


def test_config_rejection_is_recorded_without_acquiring_motion_lock(monkeypatch,tmp_path):
    import multi_agent_nav.go2_check as check
    from multi_agent_nav.config import load_config
    invalid=load_config();invalid['robot_go2']['stop_margin']=None
    monkeypatch.setattr(check,'load_config',lambda *args:invalid)
    def forbidden():
        raise AssertionError('invalid configuration must not acquire motion')
    monkeypatch.setattr(check,'motion_slot',forbidden)
    output=tmp_path/'evidence'
    assert check.main(['--action','explore','--allow-motion','--output-dir',str(output)]) == 1
    row=json.loads((output/'events.jsonl').read_text().splitlines()[0])
    assert row['state']=='REJECTED' and row['motion_sent'] is False
    assert any('stop margin' in reason for reason in row['reasons'])
    assert json.loads((output/'result.json').read_text())['motion_sent'] is False
    assert check.main(['--action','explore','--allow-motion','--output-dir',str(output)]) == 1
    assert len((output/'events.jsonl').read_text().splitlines()) == 1

def test_exploration_requires_explicit_global_goal_costmap_topic():
    from multi_agent_nav.config import load_config
    from multi_agent_nav.go2_exploration import exploration_gates
    config=load_config();config['robot_go2']['goal_costmap_topic']=None
    assert 'global goal costmap topic required' in exploration_gates(config)


def test_single_target_adapter_does_not_fabricate_b_or_c_results():
    from types import SimpleNamespace
    import numpy as np
    from vlm_nav.models import VLMResult
    from multi_agent_nav.adapters.go2 import ABCClient,observations_from_results
    client=ABCClient.__new__(ABCClient);client.mode='single'
    result=VLMResult(False,0.,None,None,False,False,False)
    client.infer=lambda rgb,target:result
    output=client.infer_abc(np.zeros((5,5,3),dtype=np.uint8),{'A':'灭火器'})
    snapshot=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0))
    frame=FrameAdapter('fixture','1','lab_ref','',[0,0,0])
    observations=observations_from_results(output,snapshot,lambda *args:None,frame,'scan',0,targets={'A':'灭火器'})
    assert [(o.target_id,o.status) for o in observations] == [('A','NOT_DETECTED')]


def test_failed_planning_is_logged_then_next_point_is_scanned():
    adapter = LocalFixture(); adapter.no_path = {1.}; events = []
    assert run_case(adapter, events) == 'EXHAUSTED'
    selected = [e['viewpoint_id'] for e in events if e['state'] == 'SELECTED']
    assert selected == ['p1','p2','p3']
    failed = next(e for e in events if e['state'] == 'PLAN_FAILED')
    assert failed['viewpoint_id'] == 'p1' and failed['plan_reachable'] is False
    assert [c[1].x for c in adapter.calls] == [2., 3.]
    assert adapter.scans == 2

def test_exploration_without_resources_stops_on_invalid_executing_path():
    from dataclasses import replace
    from multi_agent_nav.go2_exploration import ExplorationStopped
    adapter=LocalFixture();events=[]
    original=adapter.get_health
    adapter.get_health=lambda:replace(original(),path_valid=False) if adapter.active else original()
    with pytest.raises(ExplorationStopped):
        run_case(adapter,events,resources={})
    assert len(adapter.calls)==1
    assert [e['viewpoint_id'] for e in events if e['state']=='SELECTED']==['p1']

def test_exploration_without_resources_scans_each_measured_point():
    adapter=LocalFixture();events=[]
    assert run_case(adapter,events,resources={})=='EXHAUSTED'
    assert [e['viewpoint_id'] for e in events if e['state']=='SELECTED']==['p1','p2','p3']
    assert adapter.scans==3


def test_terminal_path_clear_does_not_turn_arrival_into_path_fault():
    class TerminalPathFixture(LocalFixture):
        def advance(self, now):
            super().advance(now)
            if not self.active:self.path=[]
    adapter=TerminalPathFixture();events=[]
    assert run_case(adapter,events,resources={})=='EXHAUSTED'
    assert sum(e['state']=='ARRIVED' for e in events)==3
    assert adapter.scans==3


def test_blocked_parking_point_never_receives_navigation_goal():
    adapter = LocalFixture(); adapter.blocked = {1.}; events = []
    assert run_case(adapter, events) == 'EXHAUSTED'
    assert events[1]['state'] == 'FOOTPRINT_BLOCKED'
    assert [c[1].x for c in adapter.calls] == [2.,3.]

def test_exploration_checks_far_parking_on_global_grid_before_and_after_planning():
    adapter=LocalFixture();checks=[];events=[]
    def parking(goal,global_goal=False):
        checks.append(global_goal)
        return global_goal  # all targets are outside the local rolling window
    adapter.costmap_candidate_free=parking
    assert run_case(adapter,events,resources={})=='EXHAUSTED'
    assert adapter.scans==3 and checks==[True]*6

def test_global_grid_cannot_replace_missing_local_costmap_health():
    from multi_agent_nav.go2_exploration import ExplorationStopped
    adapter=LocalFixture();adapter.costmap_transform=lambda global_goal=False:object() if global_goal else None
    with pytest.raises(ExplorationStopped,match='costmap health lost'):
        run_case(adapter,[],resources={})
    assert not adapter.calls

def test_expiry_during_global_footprint_check_is_not_reported_as_obstacle():
    from multi_agent_nav.go2_exploration import ExplorationStopped
    adapter=LocalFixture();events=[];expired=[False]
    adapter.costmap_transform=lambda global_goal=False:None if global_goal and expired[0] else object()
    def parking(goal,global_goal=False):
        expired[0]=True
        return False
    adapter.costmap_candidate_free=parking
    with pytest.raises(ExplorationStopped,match='global goal costmap'):
        run_case(adapter,events,resources={})
    assert not any(event['state']=='FOOTPRINT_BLOCKED' for event in events)
    assert not adapter.calls

@pytest.mark.parametrize('fault',['missing','lost_after_plan','obstacle_after_plan'])
def test_global_goal_input_and_parking_are_rechecked_before_motion(fault):
    from multi_agent_nav.go2_exploration import ExplorationStopped
    adapter=LocalFixture();events=[];planned=[False]
    original=adapter.compute_path
    def plan(goal):
        result=original(goal);planned[0]=True;return result
    adapter.compute_path=plan
    def transform(global_goal=False):
        return None if global_goal and (fault=='missing' or fault=='lost_after_plan' and planned[0]) else object()
    adapter.costmap_transform=transform
    adapter.costmap_candidate_free=lambda goal,global_goal=False:not (fault=='obstacle_after_plan' and planned[0])
    if fault=='obstacle_after_plan':
        assert run_case(adapter,events,resources={})=='EXHAUSTED'
        assert any(e['state']=='FOOTPRINT_BLOCKED' for e in events)
    else:
        with pytest.raises(ExplorationStopped,match='global goal costmap'):
            run_case(adapter,events,resources={})
    assert not adapter.calls


def test_detection_stops_search_even_without_projected_position():
    from multi_agent_nav.models import Observation
    adapter = LocalFixture()
    adapter.start_scan = lambda descriptions, scan_id: completed([
        Observation(target_id='A',status='DETECTED',position_valid=False,view_index=0)])
    events = []
    assert run_case(adapter, events) == 'TARGET_DETECTED_UNLOCATED'
    assert len(adapter.calls) == 1


@pytest.mark.parametrize('later_located',[False,True])
def test_unreliable_depth_after_full_scan_continues_exploration(later_located):
    from types import SimpleNamespace as S
    from multi_agent_nav.adapters.vision import observations_from_results
    from vlm_nav.models import VLMResult, Pixel
    adapter=LocalFixture();events=[]
    frame=FrameAdapter('fixture','1','lab_ref','',[0,0,0])
    result=VLMResult(True,.95,Pixel(2,2),None,True,True,True)
    def scan(descriptions,scan_id):
        adapter.scans+=1
        project=lambda *args:((2.,3.,1.),'ok') if later_located and adapter.scans==2 else (
            None,'insufficient_valid_depth_samples:0/8')
        return completed(observations_from_results({'A':('DETECTED',result)},
            S(stamp=S(sec=adapter.scans,nanosec=0)),project,frame,scan_id,0,targets=descriptions))
    adapter.start_scan=scan
    assert run_case(adapter,events)==('TARGET_FOUND' if later_located else 'EXHAUSTED')
    assert adapter.scans==(2 if later_located else 3)
    assert not any(e['state']=='TARGET_DETECTED_UNLOCATED' for e in events)
    first_scan=next(e for e in events if e['state']=='SCAN_RESULT')
    assert first_scan['observations'][0]['status']=='UNCERTAIN'


@pytest.mark.parametrize('mode', ['lost', 'api', 'stop'])
def test_health_api_error_or_unconfirmed_stop_cannot_advance(mode):
    from multi_agent_nav.go2_exploration import ExplorationStopped
    adapter = LocalFixture(); events = []
    adapter.lost_during_motion = mode == 'lost'
    adapter.api_error = mode == 'api'
    with pytest.raises(ExplorationStopped):
        run_case(adapter, events, stop_ok=mode != 'stop')
    assert [e['viewpoint_id'] for e in events if e['state']=='SELECTED'] == ['p1']


def test_empty_duplicate_or_nonfinite_points_fail_before_selection():
    from multi_agent_nav.go2_exploration import measured_viewpoints
    for points in ([], [{'id':'x','pose':[float('nan'),0,0]}],
                   [{'id':'x','pose':[1,0,0]},{'id':'x','pose':[2,0,0]}]):
        with pytest.raises(ValueError):
            measured_viewpoints({'scene': {'viewpoints': {'go2': points}}})


def test_marker_coordinates_and_label_follow_latest_selection():
    from multi_agent_nav.go2_exploration import exploration_markers
    event = {'state':'PLANNED','viewpoint_id':'p2','pose':[2.,-1.,1.57079632679], 'plan_reachable': True}
    markers = exploration_markers(event)
    arrow, text = markers.markers
    assert arrow.header.frame_id == 'map'
    assert (arrow.pose.position.x,arrow.pose.position.y) == (2.,-1.)
    assert arrow.pose.orientation.z == pytest.approx(.70710678)
    assert 'p2' in text.text and 'PLANNED' in text.text and 'reachable=True' in text.text
    assert len({m.id for m in markers.markers}) == 2


@pytest.mark.parametrize('global_ready',[True,False])
def test_exploration_entry_publishes_and_logs_before_destroying_node(monkeypatch,tmp_path,global_ready):
    from types import SimpleNamespace as S
    import rclpy
    import rclpy.executors
    from builtin_interfaces.msg import Time
    from ament_index_python.packages import get_package_share_directory
    import multi_agent_nav.go2_check as check
    import multi_agent_nav.adapters.go2 as go2
    from multi_agent_nav.config import load_config,map_file_info
    config=load_config()
    map_path=tmp_path/'map.yaml';map_path.write_text('image: map.pgm\n')
    (tmp_path/'map.pgm').write_bytes(b'fixture')
    config['robot_go2'].update(map_path=str(map_path),map_digest=map_file_info(map_path)['map_digest'],
        costmap_source_verified=True,camera_calibrated=True,path_topic='/fixture/path',
        path_source_verified=False,stop_margin=.1,scan_angles=[0.],exploration_strategy='fixed_viewpoints')
    config['scene']['viewpoints']['go2']=[{'id':'point_one','pose':[1.,0.,0.]}]
    config['scene']['resources']={'basket':{'polygon':[[100,100],[102,100],[102,102],[100,102]]}}
    now=[0.];destroyed=[False];published=[];passive_polls=[]
    def publish(markers):
        assert not destroyed[0], 'publish after node destruction'
        published.append(markers)
    parameter_result=completed(S(values=[S(string_value=str(map_path))]))
    node=S(nav_ready=True,control_armed=True,depth_units_verified=True,
        create_client=lambda *args:S(call_async=lambda request:parameter_result,service_is_ready=lambda:True),
        vlm_input_ready=lambda t:True,get_node_names=lambda:[],
        create_publisher=lambda *args:S(publish=publish if args[1]=='/vlm_nav/go2_exploration/current_point' else lambda _:None),create_timer=lambda *args:None,
        get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time())),
        destroy_node=lambda:destroyed.__setitem__(0,True))
    class Adapter(LocalFixture):
        def __init__(self,*args,**kwargs):
            assert kwargs['global_goal_checks'] is True
            super().__init__();self.costmap=None;self.plans={}
            self.vision=S(poll=lambda:passive_polls.append(now[0]))
        def costmap_transform(self,global_goal=False):
            return None if global_goal and not global_ready else super().costmap_transform(global_goal)
        def tick(self):
            if self.active:
                self.record_event({'event':'PATH_RECEIVED','path_header_stamp_ns':100,
                    'receive_interval_s':.05,'plan_stamp_changed':False})
            self.advance(now[0])
        def close(self,stop=False):pass
    monkeypatch.setenv('ROS_DOMAIN_ID','0');monkeypatch.setenv('GO2_NET_IFACE','enp4s0')
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI','file://'+get_package_share_directory('vlm_nav')+'/config/cyclonedds_go2.xml')
    monkeypatch.setattr(rclpy,'init',lambda **kw:None);monkeypatch.setattr(rclpy,'ok',lambda:False)
    monkeypatch.setattr(check,'time',S(monotonic=lambda:now[0],time=lambda:now[0]))
    monkeypatch.setattr(go2,'make_observer',lambda *args,**kw:node)
    monkeypatch.setattr(go2,'Go2Adapter',Adapter)
    monkeypatch.setattr(go2,'ABCClient',lambda cfg:None)
    def spin(**kw):now[0]+=.05
    monkeypatch.setattr(rclpy.executors,'SingleThreadedExecutor',lambda:S(
        add_node=lambda node:None,spin_once=spin,shutdown=lambda:None))
    output=tmp_path/'evidence';output.mkdir()
    args=check.parser().parse_args(['--action','explore','--allow-motion','--target','灭火器',
        '--timeout','5','--output-dir',str(output)])
    with (output/'events.jsonl').open('x') as events:
        assert check.run_ros(config,args,events)==(0 if global_ready else 1)
    rows=[json.loads(line) for line in (output/'events.jsonl').read_text().splitlines()]
    report=json.loads((output/'result.json').read_text())
    assert report['goal_costmap_valid']==global_ready
    assert report['goal_costmap_topic']==config['robot_go2']['goal_costmap_topic']
    assert destroyed[0]
    assert passive_polls, 'first furniture request must be polled before motion starts'
    if not global_ready:
        assert not report['motion_sent'] and 'fresh global goal costmap/TF required' in report['rejected']
        assert not any(row['state']=='NAVIGATING' for row in rows)
        return
    assert rows[-1]['state']=='EXHAUSTED' and rows[-1]['stop_confirmed'] is True
    assert any(row['state']=='NAVIGATING' for row in rows)
    assert any(row['state']=='SCAN_RESULT' for row in rows)
    assert any(row.get('event')=='PATH_RECEIVED' and row['state']=='NAVIGATING' for row in rows)
    assert published[-1].markers[1].text.startswith('point_one | EXHAUSTED')
    assert destroyed[0]

@pytest.mark.parametrize('mode', ['success', 'unlocated', 'no_path', 'blocked', 'cancel'])
def test_grounded_detection_approaches_only_safe_standoff(mode):
    from multi_agent_nav.models import Observation, ApproachResult
    from multi_agent_nav.go2_exploration import ExplorationStopped
    adapter=LocalFixture();events=[];scans=[]
    def scan(descriptions, scan_id):
        scans.append(descriptions)
        return completed([Observation(target_id='A',status='DETECTED',position_valid=mode!='unlocated',
            common_position=Pose2D(5.,2.,0.),view_index=0)])
    adapter.start_scan=scan
    def select(obj, excluded, **kwargs):
        assert obj.x==5. and obj.y==2.
        if mode=='cancel':adapter.selection_epoch=1
        return completed(ApproachResult(Pose2D(4.3 if mode=='success' else 4.1,2.,0.)))
    adapter.select_approach=select
    if mode=='no_path':adapter.no_path={4.1}
    if mode=='blocked':adapter.blocked={4.1}
    if mode=='cancel':
        with pytest.raises(ExplorationStopped):run_case(adapter,events,resources={},approach=True)
    else:
        outcome=run_case(adapter,events,resources={},approach=True)
        assert outcome==({'success':'TARGET_APPROACHED','unlocated':'TARGET_DETECTED_UNLOCATED'}.get(mode,'APPROACH_FAILED'))
    assert len(adapter.calls)==(2 if mode=='success' else 1)
    assert scans==[{'A':'灭火器'}]
    if mode=='success':
        nav=[e for e in events if e['state']=='NAVIGATING']
        assert [e['goal_kind'] for e in nav]==['VIEWPOINT_NAV','PICKUP_NAV']
        assert events[-1]['stop_confirmed'] is True

@pytest.mark.parametrize('distance', [0., -1., float('nan'), True, None])
def test_approach_requires_positive_finite_standoff(distance):
    from multi_agent_nav.config import load_config
    from multi_agent_nav.go2_exploration import exploration_gates
    config=load_config()
    config['robot_go2'].update(approach_on_detection=True,standoff_distance=distance)
    assert 'positive finite target standoff_distance required' in exploration_gates(config)

@pytest.mark.parametrize('corrects', [True, False])
def test_arrival_verifies_stopped_distance_and_bounds_retry(corrects):
    from multi_agent_nav.models import Observation, ApproachResult
    adapter=LocalFixture();events=[];radii=[]
    adapter.start_scan=lambda *args:completed([Observation(target_id='A', status='DETECTED',
        position_valid=True, common_position=Pose2D(5.,2.,0.), view_index=0)])
    def select(obj, excluded, *, standoff_distance):
        radii.append(standoff_distance)
        # Simulate native success outside the distance contract on the first attempt.
        x=4.25 if corrects and len(radii)==2 else 3.87
        return completed(ApproachResult(Pose2D(x,2.,0.)))
    adapter.select_approach=select
    outcome=run_case(adapter,events,resources={},approach=True)
    assert outcome==('TARGET_APPROACHED' if corrects else 'APPROACH_DISTANCE_FAILED')
    assert len(radii)==2 and radii[1]<radii[0]
    verified=[e for e in events if e['state']=='ARRIVAL_VERIFIED']
    assert verified[0]['measured_distance']==pytest.approx(1.13)
    assert verified[0]['decision']=='RETRY'
    assert verified[-1]['decision']==('SUCCESS' if corrects else 'FAILED')


def test_distance_trigger_waits_for_confirmed_stop_before_success():
    from multi_agent_nav.models import Observation, ApproachResult
    adapter=LocalFixture();events=[]
    adapter.start_scan=lambda *args:completed([Observation(target_id='A',status='DETECTED',
        position_valid=True,common_position=Pose2D(5.,2.,0.),view_index=0)])
    adapter.select_approach=lambda *args,**kwargs:completed(ApproachResult(Pose2D(4.3,2.,0.)))
    original=adapter.advance
    def advance(now):
        if len(adapter.calls)>1 and adapter.active and all(v[2].status!='CANCELED' for v in adapter.active.values()):
            adapter.pose=Pose2D(4.3,2.,0.)
            return  # Goal still active; reaching the radius must cancel it.
        original(now)
    adapter.advance=advance
    assert run_case(adapter,events,resources={},approach=True)=='TARGET_APPROACHED'
    assert any(e['state']=='APPROACH_DISTANCE_STOP' for e in events)
    assert events[-1]['stop_confirmed'] is True
