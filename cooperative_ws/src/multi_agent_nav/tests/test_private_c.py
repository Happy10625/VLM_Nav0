"""Private local maps, strict arrival and independent C completion, without ROS."""
from dataclasses import replace
from concurrent.futures import Future
import pytest
from multi_agent_nav.models import *
from multi_agent_nav.mock import mock_config, make_machine, MockMission
from multi_agent_nav.adapters.fake import FakeAdapter, completed
from multi_agent_nav.runner import AgentRunner


def private_config():
    config = mock_config()
    config['scene']['share_c_coordinates'] = False
    for robot in ROBOTS:
        config['alignment']['robots'][robot]['transform'] = None
        config['robot_'+robot].update(scan_angles=[0.], api_timeout=2., standoff_distance=.7,
            approach_on_detection=True, footprint=[[.38,.18],[.38,-.18],[-.38,-.18],[-.38,.18]])
    return config


def local_agent():
    config = private_config()
    machine, frame = make_machine(config, 'ranger', True)
    adapter = FakeAdapter('ranger', {'B':Pose2D(3,-3), 'C':Pose2D(9,1)})
    adapter.pose = Pose2D(3,-3)
    machine.tick(0., adapter.get_health())
    sync = Command(task_id='t', recipient_id='ranger', action='SYNC', kind='SYNC_LOCAL_C',
        common_frame_id='', alignment_id='')
    assert machine.accept(sync,0.).accepted
    machine.peer_received=0.
    command = replace(sync,action='EXECUTE',kind='APPROACH_PICKUP',target_id='B',command_id='b',sequence=1)
    return config,machine,adapter,command


def test_private_identity_has_no_transform_and_state_does_not_publish_local_pose():
    _,machine,adapter,_ = local_agent()
    state=machine.state()
    assert state.localization_valid and not state.pose_valid
    assert state.common_pose == Pose2D()
    assert state.common_frame_id == state.alignment_id == ''


def test_private_command_accepts_intent_and_rejects_coordinate_or_old_sync():
    _,machine,_,command=local_agent()
    assert not machine.accept(replace(command,position_valid=True,object_position=Pose2D(2,3)),0).accepted
    assert not machine.accept(replace(command,common_goal=Pose2D(2,3)),0).accepted
    assert not machine.accept(replace(command,action='SYNC',kind='WAIT'),0).accepted
    assert machine.accept(command,0).accepted
    assert [effect.kind for effect in machine.take_effects()] == ['LOCAL_EXECUTE']


def test_cache_is_local_network_is_zero_and_new_task_discards_cache():
    _,machine,_,command=local_agent()
    assert machine.accept(command,0).accepted
    machine.take_effects()
    observation=Observation(scan_id='b',observation_id='local-c',target_id='C',status='DETECTED',
        position_valid=True,common_position=Pose2D(9,1))
    machine.on_observation(observation,0)
    sent=machine.take_effects()[0].payload
    assert sent.status == 'CACHED' and sent.observation_id == 'local-c'
    assert not sent.position_valid and sent.common_position == Pose2D()
    assert machine.local_observations['local-c'].common_position == Pose2D(9,1)
    machine.stop('test');machine.tick(.1,machine.health);machine.take_effects()
    sync=replace(command,task_id='next',action='SYNC',kind='SYNC_LOCAL_C')
    assert machine.accept(sync,.2).accepted
    assert not machine.local_observations


def test_new_task_resets_published_sequence_and_old_command_identity():
    _,machine,_,command=local_agent()
    assert machine.accept(replace(command,sequence=20),0).accepted
    machine._state.requested_resource='door';machine._state.requested_command_id='b'
    machine.stop('test');machine.tick(.1,machine.health);machine.take_effects()
    assert machine.accept(replace(command,task_id='next',action='SYNC',kind='SYNC_LOCAL_C'),.2).accepted
    state=machine.state()
    assert state.sequence==0 and state.command_id==state.goal_id==''
    assert state.requested_resource==state.requested_command_id==''


def test_deposit_rejects_non_c_local_observation_reference():
    _,machine,_,command=local_agent()
    machine.local_observations['own-b']=Observation(target_id='B',position_valid=True,status='DETECTED',common_position=Pose2D(3,-3))
    machine._state.ready=True;machine.phase='DEPOSIT'
    assert not machine.accept(replace(command,phase='DEPOSIT',kind='APPROACH_DEPOSIT',target_id='C',observation_id='own-b'),0).accepted


def test_private_rejects_legacy_resource_grants():
    _,machine,_,command=local_agent()
    grant=replace(command,action='GRANT_RESOURCE',resource_id='basket',token='legacy',parent_command_id='b')
    assert not machine.accept(grant,0).accepted
    assert machine.state().token==machine.state().resource_id==''


def test_full_private_mission_keeps_ready_barrier_without_coordinate_messages():
    mission=MockMission(private_config())
    assert mission.start().accepted
    mission.run(800)
    assert mission.coordinator.phase=='DONE', (mission.coordinator.reason,mission.coordinator.states)
    deposits=[(r,c) for r,c in mission.executed if c.kind=='APPROACH_DEPOSIT']
    assert {r for r,c in deposits}==set(ROBOTS) and len(deposits)==2
    assert deposits[0][1].observation_id != deposits[1][1].observation_id
    for robot,deposit in deposits:
        assert deposit.observation_id.startswith(robot+'/')
    assert not mission.coordinator.resources.held
    assert not any(c.kind=='EXIT' for _,c in mission.executed)
    for robot,command in mission.executed:
        assert not command.position_valid and command.common_goal==command.object_position==Pose2D()
        assert command.common_frame_id==command.alignment_id==command.viewpoint_id==''
    for state in mission.coordinator.states.values():
        assert state.ready and state.deposited and state.stopped and not state.exited
        assert state.localization_valid and not state.pose_valid and state.common_pose==Pose2D()
    assert all(not o.position_valid and o.common_position==Pose2D() for o in mission.coordinator.observations.values())


@pytest.mark.parametrize('fault',['moving','far','aborted','cancel'])
def test_private_ready_requires_current_success_and_actual_stopped_distance(fault):
    config,machine,adapter,command=local_agent()
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config,success_radius=1.)
    assert machine.accept(command,0).accepted
    original_advance=adapter.advance
    if fault=='aborted':
        def advance(now):
            if any(row[2].goal_kind=='PICKUP_NAV' for row in adapter.active.values()):adapter.aborts=10
            original_advance(now)
        adapter.advance=advance
    injected=False
    for index in range(200):
        now=index*.05;machine.peer_received=machine.sync_received=now
        if not injected and any(kind=='PICKUP_NAV' for kind,_,_ in adapter.calls):
            injected=True
            if fault=='cancel':machine.stop('cancel')
            original=adapter.get_health
            if fault=='moving':adapter.get_health=lambda:replace(original(),stopped=False)
            elif fault=='far':adapter.get_health=lambda:replace(original(),pose=Pose2D(-100,-100))
        runner.tick(now)
    assert injected,'pickup navigation was not started'
    assert not machine.state().ready


@pytest.mark.parametrize('endpoint', [.7,.94,1.2])
def test_actual_path_endpoint_budget_controls_approach(endpoint):
    config,machine,adapter,command=local_agent()
    adapter.delay=.1
    original=adapter.compute_path
    def compute(goal):
        result=original(goal).result()
        if goal.x>0 and goal.y<-2 and goal!=Pose2D(3,-3):
            result.path[-1]=Pose2D(3-endpoint,-3)
        return completed(result)
    adapter.compute_path=compute
    # Keep fake candidate validation tied to the actual planner, as in Nav2Adapter.
    def select(obj,excluded,**kwargs):
        goal=Pose2D(3-endpoint,-3)
        path=compute(goal).result()
        return completed(ApproachResult(goal if kwargs['path_accept'](path) else None))
    adapter.select_approach=select
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config,success_radius=1.)
    assert machine.accept(command,0).accepted
    for index in range(100):
        now=index*.05;machine.peer_received=machine.sync_received=now;runner.tick(now)
    assert machine.state().ready is (endpoint==.7)
    if endpoint!=.7:assert not any(kind=='PICKUP_NAV' for kind,_,_ in adapter.calls)


def test_private_ignores_legacy_door_regions():
    config=private_config()
    config['scene']['resources']['door']={'polygon':[[-.5,-5],[.5,-5],[.5,5],[-.5,5]],
        'transit_exits':{r:[[2,-3,0],[-2,-3,3.14]] for r in ROBOTS}}
    mission=MockMission(config);assert mission.start().accepted
    mission.run(1000)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason
    calls=mission.adapters['go2'].calls
    assert 'DOOR_TRANSIT' not in [kind for kind,_,_ in calls]
    assert any(kind=='DEPOSIT_NAV' for kind,_,_ in calls)
    assert not mission.coordinator.resources.held
    assert not any(c.kind=='EXIT' for _,c in mission.executed)


def test_slow_local_vision_does_not_block_tick_or_pause():
    config,machine,adapter,command=local_agent()
    pending=Future()
    adapter.start_scan=lambda *a,**k:pending
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config,success_radius=1.)
    assert machine.accept(command,0).accepted
    for index in range(20):
        now=index*.05;machine.peer_received=machine.sync_received=now;runner.tick(now)
    assert not machine.state().ready and not pending.done()
    machine.stop('pause');runner.tick(1.)
    assert not machine.state().ready and not adapter.active


def test_adapter_guard_error_stops_and_cannot_escape_heartbeat_tick():
    config,machine,adapter,command=local_agent()
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config)
    assert machine.accept(command,0).accepted
    adapter.tick=lambda:adapter.motion_guard()
    runner.tick(4.)
    assert machine.paused and machine.state().run_state=='PAUSED'
    assert not adapter.active and not machine.state().ready


def test_native_terminal_path_clear_allows_local_arrival_consumption():
    config,machine,adapter,command=local_agent()
    original=adapter.advance
    def advance(now):
        original(now)
        if adapter.results:adapter.path=[]
    adapter.advance=advance
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config)
    assert machine.accept(command,0).accepted
    for index in range(100):
        now=index*.05;machine.sync_received=machine.peer_received=now;runner.tick(now)
    assert machine.state().ready,machine.state().reason


def test_private_heartbeat_loss_pauses_both_independent_agents():
    mission=MockMission(private_config());assert mission.start().accepted
    for _ in range(600):
        mission.step()
        if any(kind=='DEPOSIT_NAV' for kind,_,_ in mission.adapters['go2'].calls):break
    mission.online['go2']=False;mission.run(60)
    assert mission.coordinator.phase=='PAUSED'
    assert all(m.paused for m in mission.machines.values())
    assert all(not a.active for a in mission.adapters.values())
    before={r:len(a.calls) for r,a in mission.adapters.items()}
    mission.run(20)
    assert before=={r:len(a.calls) for r,a in mission.adapters.items()}


def test_peer_private_state_uses_local_receipt_and_pauses_on_map_change():
    config,machine,adapter,command=local_agent()
    peer=AgentState(task_id='t',robot_id='go2',local_map_id='synthetic_go2',map_version='1',
        common_frame_id='',alignment_id='',localization_valid=True,nav_ready=True,stamp=9999999999.)
    assert machine.receive_peer(peer,config['alignment']['robots']['go2'],1.)
    assert machine.peer_received==1.
    assert not machine.receive_peer(replace(peer,map_version='rebuilt'),config['alignment']['robots']['go2'],2.)
    assert machine.paused


def test_ranger_live_gates_require_local_inputs_and_do_not_require_alignment_or_go2_files(monkeypatch):
    from multi_agent_nav.config import load_config,live_gates
    import multi_agent_nav.config as module
    config=load_config();config['scene'].update(share_c_coordinates=False,enabled=True)
    config['scene']['resources']={'basket':{'polygon':[[7,0],[9,0],[9,2],[7,2]]}}
    config['scene']['exit_points']={'ranger':[1.,1.,0.]}
    config['scene']['viewpoints']['ranger']=[{'id':'r','pose':[1.,2.,0.]}]
    robot=config['robot_ranger'];robot.update(stop_margin=.1,costmap_source_verified=True,
        camera_calibrated=True,vision_validated=True,path_source_verified=True,peer_obstacle_verified=True,
        exploration_strategy='generated_candidates_vlm',scan_angles=[-(i+1)*.7853981633974483 for i in range(8)])
    monkeypatch.setattr(module,'map_file_gates',lambda config,robot,check_files=True:[])
    # Asset inspection has its own regression; do not require site-specific semantic files here.
    import multi_agent_nav.go2_exploration as core
    monkeypatch.setattr(core,'exploration_gates',lambda config,robot_id:[])
    assert live_gates(config,local_robot='ranger')==[]
    robot['path_source_verified']=False
    assert any('ranger' in reason and 'path_source_verified' in reason for reason in live_gates(config,local_robot='ranger'))


@pytest.mark.parametrize('resources',[['invalid'],'invalid',{'door':'invalid'}])
def test_private_live_config_does_not_inspect_removed_resource_geometry(resources):
    from multi_agent_nav.config import load_config,live_gates
    config=load_config();config['scene']['share_c_coordinates']=False
    config['scene']['resources']=resources
    assert not any('resource' in reason or 'polygon' in reason for reason in live_gates(config,local_robot='ranger'))


def test_deposit_never_requires_legacy_basket_permit():
    config=private_config()
    config['scene']['resources']['basket']['polygon']=[[50,50],[52,50],[52,52],[50,52]]
    mission=MockMission(config);assert mission.start().accepted
    for _ in range(800):
        mission.step()
        if any(kind=='DEPOSIT_NAV' for kind,_,_ in mission.adapters['go2'].calls):break
    else:pytest.fail('deposit did not start')
    assert not mission.machines['go2'].state().token
    assert not mission.coordinator.resources.held
    assert not any(c.kind=='EXIT' for _,c in mission.executed)


def test_motion_ownership_is_local_and_allows_only_one_agent_observer():
    from multi_agent_nav.runner import other_motion_owners
    assert other_motion_owners(['multi_agent_ranger_agent','multi_agent_ranger_rgbd'],'ranger')==[]
    assert other_motion_owners(['multi_agent_go2_agent','multi_agent_go2_rgbd','multi_agent_coordinator'],'go2')==[]
    for names in (['vlm_nav'],['multi_agent_coordinator'],['multi_agent_go2_rgbd'],
            ['multi_agent_ranger_agent']*2,['multi_agent_task_agent','multi_agent_ranger_agent']):
        assert other_motion_owners(names,'ranger')


from test_generated_candidates import generated_config


@pytest.mark.parametrize('robot_id',['go2','ranger'])
@pytest.mark.parametrize('annotated',[True,False])
def test_cooperative_runner_uses_same_local_generated_candidate_core(generated_config,tmp_path,robot_id,annotated):
    from types import SimpleNamespace as S
    import yaml
    config=private_config()
    config['robot_'+robot_id].update(generated_config['robot_go2'],standoff_distance=.7)
    config['alignment']['robots'][robot_id].update(generated_config['alignment']['robots']['go2'])
    semantic=tmp_path/'semantic.yaml';data=yaml.safe_load(semantic.read_text())
    data['source_robot']=robot_id;semantic.write_text(yaml.safe_dump(data))
    if not annotated:config['robot_'+robot_id]['generated_candidates']['semantic_yaml']=''
    machine,frame=make_machine(config,robot_id,True)
    target_id={'go2':'A','ranger':'B'}[robot_id]
    adapter=FakeAdapter(robot_id,{target_id:Pose2D(3,-3),'C':Pose2D(3,3)},delay=.1)
    snapshots=[]
    def choose(snapshot):snapshots.append(snapshot);return dict(candidate_id='P1',reason='first eligible side')
    adapter.client=S(choose_candidate=choose)
    adapter.vision=S(submit=lambda fn:completed(fn()))
    machine.tick(0,adapter.get_health())
    sync=Command(task_id='t',recipient_id=robot_id,action='SYNC',kind='SYNC_LOCAL_C',common_frame_id='',alignment_id='')
    assert machine.accept(sync,0).accepted
    command=replace(sync,action='EXECUTE',command_id='pickup',sequence=1,kind='APPROACH_PICKUP',target_id=target_id)
    assert machine.accept(command,0).accepted
    emitted=[]
    runner=AgentRunner(machine,adapter,config['scene']['targets'],emitted.append,config=config,success_radius=1.)
    for index in range(100):
        now=index*.05;machine.sync_received=machine.peer_received=now;runner.tick(now)
    assert machine.state().ready,machine.state().reason
    assert snapshots and snapshots[0]['candidates'][0]['display_id']=='P1'
    assert snapshots[0]['furniture_annotated'] is annotated
    assert [kind for kind,_,_ in adapter.calls]==['VIEWPOINT_NAV','PICKUP_NAV']
    assert all(command_id=='pickup' for _,_,command_id in adapter.calls)
    assert any(o.status=='CACHED' for o in emitted)
    assert all(o.common_position==Pose2D() and not o.position_valid for o in emitted)
