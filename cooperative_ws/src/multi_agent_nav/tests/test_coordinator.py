from dataclasses import replace
from multi_agent_nav.models import *
from multi_agent_nav.coordinator import Coordinator

def scenario():
    return {'scenario_id':'test', 'enabled':True, 'simulation':True,
            'viewpoints':{'go2':[{'id':'v1','pose':[0,0,0]}],
                          'ranger':[{'id':'v2','pose':[3,0,0]}]},
            'wait_points':{'go2':[-1,0,0], 'ranger':[5,0,0]},
            'exit_points':{'go2':[0,-3,0], 'ranger':[3,-3,0]},
            'resources':{'basket':{'polygon':[[0,1],[4,1],[4,4],[0,4]]}}}

def coord():
    c = Coordinator(scenario(), 'a', {'go2':('g','v'), 'ranger':('r','v')}, True)
    for r in ROBOTS:
        c.ingest_state(AgentState(robot_id=r, local_map_id='g' if r=='go2' else 'r',
                        map_version='v', alignment_id='a', pose_valid=True,
                        localization_valid=True, nav_ready=True), 0)
    assert c.start('test', 0).accepted
    for r in ROBOTS:
        c.ingest_state(replace(c.states[r], task_id=c.task_id), 0)
    return c

def test_ready_barrier_and_early_c_saved():
    c = coord()
    c.ingest_observation(Observation(task_id=c.task_id, robot_id='go2', observation_id='C1',
        alignment_id='a', local_map_id='g', map_version='v', target_id='C',
        status='DETECTED', position_valid=True, common_position=Pose2D(2,2)), 0)
    c.ingest_state(replace(c.states['go2'], ready=True), 0)
    commands = c.tick(0)
    assert c.phase == 'SEARCH_AB'
    assert not any(x.target_id=='C' for x in commands)
    c.commands.clear()  # no command running at barrier in this unit scenario
    c.ingest_state(replace(c.states['ranger'], ready=True), .1)
    commands = c.tick(.1)
    assert c.phase == 'DEPOSIT'
    execute = [x for x in commands if x.action=='EXECUTE']
    assert len(execute)==1 and execute[0].recipient_id=='go2' and execute[0].observation_id=='C1'

def test_deposit_exit_preserves_original_permit_and_order():
    c = coord()
    c.phase, c.c_observation = 'DEPOSIT', 'C1'
    c.scene['bound_c'] = 'C1'
    c.observations['C1'] = Observation(target_id='C', status='DETECTED', common_position=Pose2D(2,2), position_valid=True)
    for r in ROBOTS: c.states[r].ready=True
    deposit = next(x for x in c.tick(0) if x.action=='EXECUTE')
    c.ingest_state(replace(c.states['go2'], command_id=deposit.command_id, sequence=deposit.sequence,
        reply='SUCCEEDED', deposited=True, resource_id='basket', token='tok',
        permit_command_id=deposit.command_id), .1)
    commands = c.tick(.1)
    exit_cmd = next(x for x in commands if x.action=='EXECUTE')
    assert exit_cmd.kind=='EXIT' and exit_cmd.recipient_id=='go2'
    assert exit_cmd.token=='tok' and exit_cmd.parent_command_id==deposit.command_id
    assert not any(x.recipient_id=='ranger' and x.kind=='APPROACH_DEPOSIT' for x in commands)
    c.ingest_state(replace(c.states['go2'], command_id=exit_cmd.command_id, sequence=exit_cmd.sequence,
                           reply='SUCCEEDED', exited=True, token=''), .2)
    assert any(x.recipient_id=='ranger' and x.kind=='APPROACH_DEPOSIT' for x in c.tick(.2))

def test_old_ready_mixed_mode_and_receiver_clock():
    c = coord()
    c.ingest_state(replace(c.states['go2'], task_id='old', ready=True), 2)
    assert not c.states['go2'].ready
    c.ingest_state(replace(c.states['go2'], stamp=1e12), 0)
    c.tick(3.1)
    assert c.phase=='PAUSED'
    c = coord()
    c.ingest_state(replace(c.states['ranger'], simulation=False), .1)
    assert c.phase=='PAUSED'

def test_restart_and_unconfigured_c_do_not_resume():
    c = coord()
    c.scene['resources']={}
    for r in ROBOTS: c.states[r].ready=True
    c.tick(.1)
    assert c.phase=='PAUSED'
    restarted = Coordinator(scenario(), 'a', {'go2':('g','v'), 'ranger':('r','v')}, True)
    assert restarted.phase=='IDLE' and restarted.tick(0)==[]
