from pathlib import Path
import copy
import json
import sys
import pytest
from multi_agent_nav.config import load_config
from multi_agent_nav.adapters.fake import FakeAdapter, completed
from multi_agent_nav.models import Pose2D, PathResult, Observation
from multi_agent_nav.frame_adapter import FrameAdapter


def test_ranger_offline_has_local_map_identity_and_never_initializes_ros(monkeypatch,capsys):
    from multi_agent_nav.ranger_check import main
    monkeypatch.setitem(sys.modules,'rclpy',None)
    assert main(['--offline'])==0
    report=json.loads(capsys.readouterr().out)
    assert report['robot_id']=='ranger' and report['target_id']=='B'
    assert report['arrival_contract']['target_success_radius']==.81
    assert report['arrival_contract']['approach_cancel_radius']==.89
    assert report['configured_nav2_tolerances']=={'planner_tolerance':.76,'goal_xy_tolerance':.05}
    assert report['common_frame_id']=='ranger_local_map' and report['motion_sent'] is False
    assert report['map_files']['yaml_path'].endswith('/shared_maps/lab_ranger_20261006/lab.yaml')
    assert report['exploration_strategy']=='generated_candidates_vlm'
    assert not any('viewpoint' in error.lower() for error in report['single_robot_experiment_gates'])
    assert report['semantic_search']['points']


def test_local_context_does_not_change_cooperative_alignment():
    from multi_agent_nav.local_context import local_config
    config=load_config();original=copy.deepcopy(config)
    local=local_config(config,'ranger')
    assert config==original and not config['alignment']['validated']
    assert local['alignment']['common_frame_id']=='ranger_local_map'
    assert local['alignment']['robots']['ranger']['transform']==[0.,0.,0.]
    assert local['scene']['resources']=={}


class RangerFixture(FakeAdapter):
    def __init__(self,detected):
        super().__init__('ranger',delay=.1);self.detected=detected;self.requested=[]
    def costmap_transform(self,global_goal=False):return object()
    def costmap_candidate_free(self,goal,global_goal=False):return True
    def start_scan(self,descriptions,scan_id,*,stop_on_target=None):
        self.requested.append(descriptions)
        return completed([Observation(target_id=t,view_index=i,status='DETECTED' if t==self.detected else 'NOT_DETECTED',
            common_position=Pose2D(1.5,0),position_valid=t==self.detected) for i in range(8) for t in 'ABC'])
@pytest.mark.parametrize('detected,expected',[('A','EXHAUSTED'),('C','EXHAUSTED'),('B','TARGET_APPROACHED')])
def test_ranger_only_approaches_b(detected,expected):
    from multi_agent_nav.go2_exploration import explore
    a=RangerFixture(detected);events=[];now=[0.]
    def spin():now[0]+=.05;a.advance(now[0])
    def stop():a.stop_motion('test');a.advance(now[0]+1);return True
    config={'robot_ranger':{'stop_margin':.1,'scan_angles':[i*.7853981633974483 for i in range(1,9)],
        'approach_on_detection':True,'standoff_distance':.81,'arrival_config_files':[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')]},
        'scene':{'resources':{},'viewpoints':{'ranger':[{'id':'r1','pose':[1.,0.,0.]}]}}}
    frame=FrameAdapter('r','v','ranger_local_map','local_only',[0,0,0])
    result=explore(a,spin,config,frame,10.,'杯',events.append,lambda:[],stop,lambda:None,
        clock=lambda:now[0],robot_id='ranger',target_id='B')
    assert result==expected and a.requested==[{'B':'杯'}]
    assert all(event['robot_id']=='ranger' and event['target_id']=='B' for event in events)


@pytest.mark.parametrize('located,prior_error,stop_ok,expected', [
    (True,False,True,'TARGET_APPROACHED'), (True,True,True,'TARGET_APPROACHED'),
    (False,True,True,'TARGET_DETECTED_UNLOCATED'), (True,False,False,None),
])
def test_ranger_partial_scan_detection_survives_prior_error_and_approaches_without_full_circle(located,prior_error,stop_ok,expected):
    from multi_agent_nav.go2_exploration import explore, ExplorationStopped
    a=RangerFixture('B');events=[];now=[0.];requested=[]
    def scan(descriptions,scan_id,*,stop_on_target=None):
        requested.append(stop_on_target)
        rows=[Observation(target_id='B',view_index=1,status='DETECTED',
            common_position=Pose2D(5.,2.),position_valid=located)]
        if prior_error:rows.insert(0,Observation(target_id='B',view_index=0,status='API_ERROR'))
        return completed(rows)
    a.start_scan=scan
    def spin():now[0]+=.05;a.advance(now[0])
    def stop():
        a.stop_motion('test');a.advance(now[0]+1)
        return stop_ok if requested else True
    config={'robot_ranger':{'stop_margin':.1,'scan_angles':[i*.7853981633974483 for i in range(1,9)],
        'approach_on_detection':True,'standoff_distance':.8,'arrival_config_files':[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')]},
        'scene':{'resources':{},'viewpoints':{'ranger':[{'id':'r1','pose':[1.,0.,0.]}]}}}
    def run():
        return explore(a,spin,config,FrameAdapter('r','v','ranger_local_map','local_only',[0,0,0]),
            10.,'纸巾',events.append,lambda:[],stop,lambda:None,clock=lambda:now[0],robot_id='ranger',target_id='B')
    if not stop_ok:
        with pytest.raises(ExplorationStopped,match='stop unconfirmed'):run()
        assert requested==['B'] and [c[0] for c in a.calls]==['VIEWPOINT_NAV']
        return
    outcome=run()
    assert outcome==expected and requested==['B']
    assert [c[0] for c in a.calls]==(['VIEWPOINT_NAV','PICKUP_NAV'] if located else ['VIEWPOINT_NAV'])
    assert not any(e['state']=='EXHAUSTED' for e in events)


def direct_reference_case(mode):
    """Nav2 may return a free path ending before the occupied object reference."""
    from multi_agent_nav.go2_exploration import explore
    frame=FrameAdapter('r','v','ranger_local_map','local_only',[10.,-2.,.4])
    target=Pose2D(1.5,0.) if mode=='already_close' else Pose2D(1.85,0.) if mode=='cancel_band' else Pose2D(5.,2.)
    adapter=RangerFixture('B');adapter.delay=.5
    planned=[];events=[];now=[0.]
    def scan(descriptions,scan_id,*,stop_on_target=None):
        return completed([Observation(target_id='B',status='DETECTED',position_valid=True,
            common_position=frame.to_common(target),view_index=0)])
    adapter.start_scan=scan
    adapter.select_approach=lambda *args,**kwargs:pytest.fail('Ranger must plan to the target reference directly')
    # The reference may sit on an object. Nav2 validates the actual path with planner tolerance.
    adapter.costmap_candidate_free=lambda goal,**kwargs:not (abs(goal.x-target.x)<1e-6 and abs(goal.y-target.y)<1e-6)
    def plan(goal):
        planned.append(goal)
        if abs(goal.x-target.x)>1e-6:return completed(PathResult(True,[adapter.pose,goal]))
        if mode=='epoch_changed':adapter.selection_epoch=1
        if mode=='no_path':return completed(PathResult(False,reason='planner rejected target reference'))
        if mode=='empty_path':return completed(PathResult(True,[]))
        return completed(PathResult(True,[adapter.pose,Pose2D(target.x-.7,target.y)]))
    adapter.compute_path=plan
    adapter.costmap_transform=lambda global_goal=False:None if (
        mode=='stale_global' and global_goal and len(planned)>1) else object()
    def spin():
        now[0]+=.05
        approaching=any(row[2].goal_kind=='PICKUP_NAV' for row in adapter.active.values())
        adapter.advance(now[0])
        if approaching and mode in ('retry','retry_exhausted'):
            attempts=sum(call[0]=='PICKUP_NAV' for call in adapter.calls)
            distance=.8 if mode=='retry' and attempts==2 else .85
            adapter.pose=Pose2D(target.x-distance,target.y,.37)
    def stop():
        if mode=='stop_unconfirmed' and len(adapter.calls)>1:return False
        adapter.stop_motion('fixture stop');adapter.advance(now[0]);return True
    resources={'zone':{'polygon':[[2.,.2],[3.,.2],[3.,2.],[2.,2.]]}} if mode=='protected_path' else {}
    viewpoint=frame.to_common(Pose2D(1.,0.,.37))
    config={'robot_ranger':{'stop_margin':.1,'scan_angles':[0.],
        'approach_on_detection':True,'arrival_config_files':[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')]},
        'scene':{'resources':resources,'viewpoints':{'ranger':[{'id':'r1','pose':[viewpoint.x,viewpoint.y,viewpoint.yaw]}]}}}
    outcome=explore(adapter,spin,config,frame,10.,'纸巾',events.append,lambda:[],stop,lambda:None,
        clock=lambda:now[0],robot_id='ranger',target_id='B')
    return outcome,adapter,planned,events


def test_ranger_plans_to_reference_with_planner_tolerance_and_cancels_on_distance():
    outcome,a,planned,events=direct_reference_case('success')
    assert outcome=='TARGET_APPROACHED'
    assert [planned[-1].x,planned[-1].y,planned[-1].yaw]==pytest.approx([5.,2.,.37])
    assert a.calls[-1][1]==planned[-1]
    assert any(e['state']=='APPROACH_DISTANCE_STOP' for e in events)
    assert not a.active and events[-1]['stop_confirmed'] is True
    assert events[-1]['measured_distance']<=.81


@pytest.mark.parametrize('mode',['no_path','empty_path','protected_path'])
def test_ranger_reference_planner_rejection_or_protected_path_cannot_send_approach(mode):
    outcome,a,planned,events=direct_reference_case(mode)
    assert outcome=='APPROACH_FAILED'
    assert [call[0] for call in a.calls]==['VIEWPOINT_NAV']
    if mode=='no_path':assert any(e.get('reason')=='planner rejected target reference' for e in events)


@pytest.mark.parametrize('mode,reason',[
    ('stale_global','global goal costmap'),('epoch_changed','cancelled'),('stop_unconfirmed','stop unconfirmed'),
])
def test_ranger_direct_approach_keeps_freshness_cancellation_and_stop_checks(mode,reason):
    from multi_agent_nav.go2_exploration import ExplorationStopped
    with pytest.raises(ExplorationStopped,match=reason):direct_reference_case(mode)


def test_ranger_already_close_succeeds_without_target_planning_or_approach():
    outcome,a,planned,events=direct_reference_case('already_close')
    assert outcome=='TARGET_APPROACHED' and len(planned)==1
    assert [call[0] for call in a.calls]==['VIEWPOINT_NAV']


def test_ranger_inside_cancel_radius_verifies_stop_before_any_target_plan():
    outcome,a,planned,events=direct_reference_case('cancel_band')
    assert outcome=='TARGET_APPROACHED' and len(planned)==2
    stopped_index=next(i for i,e in enumerate(events) if e['state']=='APPROACH_DISTANCE_STOP')
    planning_index=next(i for i,e in enumerate(events) if e['state']=='APPROACH_PLANNING')
    assert stopped_index<planning_index
    assert [e['decision'] for e in events if e['state']=='ARRIVAL_VERIFIED']==['RETRY','SUCCESS']


@pytest.mark.parametrize('mode,expected', [('retry','TARGET_APPROACHED'),('retry_exhausted','APPROACH_DISTANCE_FAILED')])
def test_ranger_retry_keeps_reference_goal_and_verifies_final_stopped_distance(mode,expected):
    outcome,a,planned,events=direct_reference_case(mode)
    assert outcome==expected
    assert len(planned)==3 and len(a.calls)==3 and not a.active
    assert all([p.x,p.y]==pytest.approx([5.,2.]) for p in planned[1:])
    verified=[e for e in events if e['state']=='ARRIVAL_VERIFIED']
    assert [e['decision'] for e in verified]==['RETRY','SUCCESS' if mode=='retry' else 'FAILED']
    radii=[e['cancel_radius'] for e in events if e['state']=='APPROACH_DISTANCE_STOP']
    assert radii==([.89,.81] if mode=='retry' else [.89])
    assert events[-1]['stop_confirmed'] is True


def test_ranger_exploration_does_not_require_unused_standoff_parameter():
    from multi_agent_nav.go2_exploration import exploration_gates
    config=load_config();config['robot_ranger'].update(approach_on_detection=True)
    config['robot_ranger'].pop('standoff_distance',None)
    assert 'positive finite target standoff_distance required' not in exploration_gates(config,'ranger')


# Exercise the same synthetic-map and serial-executor fixtures as the Go2 regression.
from test_generated_candidates import generated_config


@pytest.mark.parametrize('angle',[0.,.4])
def test_ranger_generated_partial_abc_keeps_b_and_unfinished_viewpoint(generated_config,tmp_path,angle):
    import yaml
    from test_generated_candidates import run_generated
    from multi_agent_nav.config import map_file_info
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    config=generated_config
    config['robot_ranger']=config.pop('robot_go2')
    config['alignment']['robots']['ranger']=config['alignment']['robots'].pop('go2')
    robot=config['robot_ranger'];robot['approach_on_detection']=False
    mp=Path(robot['map_path']);source=yaml.safe_load(mp.read_text());source['origin'][2]=angle
    mp.write_text(yaml.safe_dump(source));info=map_file_info(mp)
    robot['map_digest']=info['map_digest'];config['alignment']['robots']['ranger']['map_version']='sha256:'+info['map_digest']
    sem=Path(robot['generated_candidates']['semantic_yaml']);data=yaml.safe_load(sem.read_text())
    data['source_robot']='ranger';data['source_map']['yaml_sha256']=info['yaml_sha256']
    sem.write_text(yaml.safe_dump(data))
    _,events,requests,result=run_generated(config,tmp_path,mode='partial_detection',robot_id='ranger',target_id='B')
    assert result=='TARGET_FOUND'
    scanned=next(e for e in events if e['state']=='SCAN_RESULT')
    assert {o['target_id'] for o in scanned['observations']}=={'B'}
    last_record=next(e for e in reversed(events) if e['state']=='OBSERVATION_RECORD')
    assert 'observation_failed' in [v['status'] for v in last_record['records'].values()]
    assert all(v['status']!='observed' for v in last_record['records'].values())
    assert all(e['common_frame_id']=='ranger_local_map' and e['target_id']=='B' for e in events)
    assert len(requests[0]['images'])==2
    # Reject mislabeled or wrong-map semantic input without relabeling coordinates.
    data['source_robot']='go2';sem.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='frame/schema'):GeneratedCandidates(config,'ranger')


def test_ranger_require_ready_keeps_input_motion_terminal_and_owner_checks():
    from multi_agent_nav.go2_check import inspection_ready
    from multi_agent_nav.models import Health
    report=dict(robot_local_gates=[],active_map_matches=True,costmap_valid=True)
    health=Health(localization_valid=True,stopped=True,nav_ready=False,motion_terminal=True)
    assert not inspection_ready(report,health,robot_id='ranger')
    health.nav_ready=True
    assert inspection_ready(report,health,robot_id='ranger')
    assert not inspection_ready(report,health,robot_id='ranger',owners=['vlm_nav'])
    health.motion_terminal=False
    assert not inspection_ready(report,health,robot_id='ranger')


def test_ranger_offline_require_ready_does_not_require_permission_parameters(tmp_path,monkeypatch,capsys):
    import yaml
    from multi_agent_nav.ranger_check import main
    config=load_config()
    robot=config['robot_ranger']
    for key in ('motion_permission_topic','motion_permission_timeout','motion_permission_verified'):
        robot.pop(key,None)
    robot.update(stop_margin=.2,costmap_source_verified=True,camera_calibrated=True)
    config['scene']['viewpoints']['ranger']=[{'id':'test_view','pose':[1.,0.,0.]}]
    for name,value in config.items():(tmp_path/(name+'.yaml')).write_text(yaml.safe_dump(value))
    monkeypatch.setitem(sys.modules,'rclpy',None)
    assert main(['--config-dir',str(tmp_path),'--offline','--require-ready'])==0
    report=json.loads(capsys.readouterr().out)
    assert report['single_robot_experiment_gates']==[]
    assert report['motion_sent'] is False
