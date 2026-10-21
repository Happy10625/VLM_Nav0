from dataclasses import replace
from multi_agent_nav.mock import MockMission, mock_config

def test_two_maps_shared_a_ready_barrier_same_c_order_and_permit():
    mission=MockMission()
    assert mission.start().accepted
    mission.run(100)
    assert mission.coordinator.phase=='DONE', mission.coordinator.reason
    states=mission.coordinator.states
    assert all(s.ready and s.deposited and s.exited for s in states.values())
    calls=mission.executed
    deposits=[(robot,cmd) for robot,cmd in calls if cmd.kind=='APPROACH_DEPOSIT']
    assert [r for r,c in deposits]==['go2','ranger']
    assert deposits[0][1].observation_id==deposits[1][1].observation_id
    for robot,deposit in deposits:
        exit_cmd=next(cmd for r,cmd in calls if r==robot and cmd.kind=='EXIT')
        assert exit_cmd.parent_command_id==deposit.command_id and exit_cmd.token
    go2_pickup=next(c for r,c in calls if r=='go2' and c.kind=='APPROACH_PICKUP')
    assert '/ranger/' in go2_pickup.observation_id  # source clue from peer
    assert mission.frames['ranger'].yaw != 0
    assert not mission.coordinator.resources.held

def test_pause_late_success_cannot_cross_ready_barrier():
    m=MockMission()
    m.adapters['go2'].delay=5
    m.start();m.run(10)
    m.pause('test')
    m.run(100)
    assert m.coordinator.phase=='PAUSED'
    assert not m.machines['go2'].state().ready
    assert not any(c.kind=='APPROACH_DEPOSIT' for r,c in m.executed)

def test_vlm_timeout_retries_once_and_no_false_not_found():
    m=MockMission();m.adapters['ranger'].api_error=True
    m.start();m.run(100)
    assert m.coordinator.phase=='PAUSED'
    assert not m.machines['ranger'].state().ready
    assert any(o.status=='API_ERROR' for o in m.coordinator.observations.values())

def test_held_resource_peer_loss_freezes_permit():
    m=MockMission();m.start()
    for _ in range(100):
        m.step()
        if 'basket' in m.coordinator.resources.held:break
    assert 'basket' in m.coordinator.resources.held
    m.online['go2']=False
    m.run(60)
    assert m.coordinator.phase=='PAUSED'
    assert 'basket' in m.coordinator.resources.held

def test_local_unreachable_does_not_make_shared_clue_globally_unusable():
    m=MockMission();m.adapters['go2'].unreachable=True
    m.start();m.run(120)
    assert m.coordinator.phase=='PAUSED'
    assert not m.machines['go2'].state().ready
    assert m.machines['ranger'].state().ready

def test_duplicate_wire_commands_dont_duplicate_navigation():
    m=MockMission();m.duplicate_commands=True
    m.start();m.run(100)
    assert m.coordinator.phase=='DONE'
    for r in ('go2','ranger'):
        ids=[cmd for kind,p,cmd in m.adapters[r].calls]
        assert len(ids)==len(set(ids))

def test_door_permit_released_before_basket_and_original_main_command_resumes():
    config=mock_config()
    config['scene']['resources']['door']={'polygon':[[-.5,-5],[.5,-5],[.5,5],[-.5,5]],
        'transit_exits':{r:[[2,-3,0],[-2,-3,3.141592653589793]] for r in ('go2','ranger')}}
    m=MockMission(config)
    m.adapters['go2'].delay=1.0
    m.start();m.run(220)
    assert m.coordinator.phase=='DONE',m.coordinator.reason
    calls=m.adapters['go2'].calls
    kinds=[kind for kind,_,_ in calls]
    assert 'DOOR_TRANSIT' in kinds
    transit=next(cmd for kind,_,cmd in calls if kind=='DOOR_TRANSIT')
    deposit=next(cmd for kind,_,cmd in calls if kind=='DEPOSIT_NAV')
    assert transit==deposit
    assert not m.coordinator.resources.held
