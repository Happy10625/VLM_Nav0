from dataclasses import replace
from test_agent import machine,command,navigate
from test_coordinator import coord
from multi_agent_nav.models import Health,Command,Pose2D,GoalResult,ResourceRequest
from multi_agent_nav.mock import MockMission
from multi_agent_nav.region_guard import RegionGuard

def test_new_task_cannot_discard_uncertain_old_resource():
    agent=machine()
    agent._state.token='held';agent._state.resource_id='basket'
    agent._state.run_state='PAUSED';agent._state.stopped=True
    sync=replace(command(),action='SYNC',task_id='new')
    assert not agent.accept(sync,1).accepted
    c=coord();c.pause('restart')
    c.states['go2'].token='held'
    assert not c.start('test',1).accepted

def test_exit_outside_goal_cannot_claim_release_without_entry():
    m=MockMission();m.start()
    for _ in range(100):
        m.step()
        if m.machines['go2'].state().deposited:break
    state=m.machines['go2'].state()
    assert state.token and state.resource_status=='ENTERED'
    m.coordinator.resources.freeze('go2','loss')
    m.run(80)
    assert 'basket' in m.coordinator.resources.held
    assert not m.machines['go2'].state().exited

def test_cancel_ack_without_action_terminal_is_not_stopped():
    agent=machine();navigate(agent,command())
    agent.accept(replace(command(),action='CANCEL',cancel_command_id='c1'),1)
    agent.tick(1,Health(localization_valid=True,nav_ready=True,stopped=True,motion_terminal=False))
    assert agent.state().run_state=='STOPPING'
    assert not agent.accept(command(sequence=2),1).accepted

def test_missing_footprint_cannot_confirm_resource_exit():
    m=MockMission();m.start()
    for _ in range(100):
        m.step()
        if m.machines['go2'].state().deposited:break
    agent=m.machines['go2']
    agent._state.run_state='EXIT_STOP'
    health=m.adapters['go2'].get_health()
    health.footprint=[];health.pose=Pose2D(99,99)
    agent._monitor_region(health)
    assert agent.state().resource_status!='EXIT_CONFIRMED'
    assert agent.state().token

def test_retired_task_sync_cannot_revive_canceled_execution():
    agent=machine()
    old=command()
    assert agent.accept(old,0).accepted
    agent.accept(replace(old,action='CANCEL',cancel_command_id=old.command_id),0)
    agent.tick(0,Health(localization_valid=True,nav_ready=True,stopped=True))
    new=replace(old,action='SYNC',task_id='new')
    assert agent.accept(new,0).accepted
    agent.accept(replace(new,action='PAUSE'),0)
    agent.tick(0,Health(localization_valid=True,nav_ready=True,stopped=True))
    agent.take_effects()
    assert not agent.accept(replace(old,action='SYNC'),0).accepted
    assert not agent.accept(old,0).accepted
    assert not agent.take_effects()

def test_executing_path_startup_waits_only_while_stopped_and_is_bounded():
    def started():
        agent=machine()
        agent.guard=RegionGuard({'basket':{'polygon':[[7,-1],[11,-1],[11,3],[7,3]]}},.1)
        agent.tick(0,replace(health,motion_terminal=True))
        navigate(agent,command())
        assert not agent.paused
        return agent
    health=Health(localization_valid=True,nav_ready=True,stopped=True,
        footprint=[[-.38,-.18],[.38,-.18],[.38,.18],[-.38,.18]],motion_terminal=False)
    agent=started()
    agent.tick(.1,health)
    assert agent.state().run_state=='NAVIGATING'
    agent.tick(.2,replace(health,path=[Pose2D(),Pose2D(2,0)],path_valid=True,stopped=False))
    assert agent.state().run_state=='NAVIGATING'
    agent.tick(.3,health)  # No grace after the first executing path.
    assert agent.paused
    agent=started()
    agent.tick(.1,replace(health,stopped=False))
    assert agent.paused  # Movement before a valid executing path fails closed.
    agent=started()
    agent.tick(1.1,health)
    assert agent.paused  # A planner that never publishes cannot wait forever.
