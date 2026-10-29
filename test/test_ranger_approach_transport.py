"""Controlled Futures exercise real Ranger transport; no ROS nodes or motion."""
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from test_ranger_single_robot_parity import local_runtime, start_approach
from test_navigation_gate import stationary_odom


def test_confirmation_cannot_start_old_arrival_before_shared_executor(local_runtime):
    r=local_runtime
    r.node.set_confirmed_target_reference_position((.8,0.,0.))
    r.node.set_state('APPROACHING')
    r.node.local_tick()
    assert r.node.state=='APPROACHING' and not r.requests and not r.plans


def test_cancel_acknowledgement_without_terminal_never_succeeds(local_runtime):
    r=local_runtime;a=start_approach(r)
    r.node.robot_pose=lambda:(1.2,0.,0.)
    r.clock[0]+=.21;next(r.steps)
    assert a.arrival_cancel_acknowledged and not a.arrival_action_terminal
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=3.;a.advance_approach_stop()
    assert a.state=='APPROACH_STOPPING'
    r.clock[0]+=.001;a.advance_approach_stop()
    assert a.state=='FAILED' and r.adapter.motions
    assert not r.adapter.local_arrival_verified
    r.steps.close()


@pytest.mark.parametrize('pending',['plan','navigation'])
def test_external_cancel_cancels_late_acceptance(local_runtime,pending):
    r=local_runtime;response=Future();terminal=Future();cancels=[]
    client=r.node.path_planner if pending=='plan' else r.node.navigator
    client.send_goal_async=lambda _:response
    a=start_approach(r)
    r.adapter.stop_motion('external CANCEL')
    def cancel():
        cancels.append(True)
        f=Future();f.set_result(None);return f
    response.set_result(S(accepted=True,goal_id=S(uuid=bytes(16)),
        cancel_goal_async=cancel,get_result_async=lambda:terminal))
    terminal.set_result(S(status=4,result=S(path=S(poses=[]))))
    assert cancels and a.state=='FAILED'
    assert not r.adapter.results and not r.adapter.local_arrival_verified
    assert not r.adapter.motions and not r.requests
    r.steps.close()


def test_already_completed_native_result_cannot_restore_goal_handle(local_runtime):
    r=local_runtime
    def ready(value):
        f=Future();f.set_result(value);return f
    r.node.navigator.send_goal_async=lambda _:ready(S(accepted=True,goal_id=S(uuid=bytes(16)),
        get_result_async=lambda:ready(S(status=4)),cancel_goal_async=lambda:ready(None)))
    a=start_approach(r)
    assert a.arrival_action_terminal and a.goal_handle is None
    assert not r.adapter.motions and not r.adapter.results
    r.steps.close()


def test_native_result_has_one_consumer_even_if_callback_repeats(local_runtime):
    r=local_runtime;a=start_approach(r)
    motion=next(iter(r.adapter.motions.values()))
    results=[];original=a.on_navigation_result
    def consume(*args):
        results.append(args)
        original(*args)
    a.on_navigation_result=consume
    r.terminals[0].set_result(S(status=4))
    motion['finish']('SUCCEEDED',4)
    assert len(results)==1 and not r.adapter.results and not r.adapter.motions
    r.steps.close()


@pytest.mark.parametrize('status',[5,6])
def test_uncontrolled_cancel_or_abort_does_not_use_distance_retry(local_runtime,status):
    r=local_runtime;a=start_approach(r)
    r.terminals[0].set_result(S(status=status))
    assert a.state=='FAILED' and a.arrival_retry_count==0
    assert len(r.plans)==len(r.requests)==1 and not r.adapter.results
    r.steps.close()


def test_arrival_samples_forward_from_original_topic_only(local_runtime):
    r=local_runtime;a=start_approach(r,.8)
    r.adapter.config['odom_max_age']=.5
    r.adapter.odom_at=0.;r.adapter.stationary_since=None
    message=stationary_odom()
    message.header.stamp.sec=100
    r.adapter.on_odometry(message)
    assert a.arrival_stationary_count==0
    for _ in range(2):r.node.on_arrival_odom(message)
    assert a.arrival_stationary_count==2
    message.twist.twist.linear.y=.04
    r.node.on_arrival_odom(message)
    assert a.arrival_stationary_count==0
    r.steps.close()


@pytest.mark.parametrize('fault',['identity','context','paused','health','stop','terminal','distance','owner','no_evidence'])
def test_outer_barrier_rejects_invalid_completion(local_runtime,fault):
    from multi_agent_nav.runner import AgentRunner
    from multi_agent_nav.models import Pose2D
    r=local_runtime;a=start_approach(r,.8)
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(r.steps)
    with pytest.raises(StopIteration):next(r.steps)
    state=S(ready=False,deposited=False)
    failures=[]
    machine=S(command=S(task_id='task',command_id='command',kind='APPROACH_PICKUP'),
        context=S(task_id='task'),paused=False,_state=state,stop=failures.append,fail=failures.append)
    health=r.adapter.get_health()
    if fault=='identity':machine.command.command_id='new'
    if fault=='context':machine.context.task_id='new'
    if fault=='paused':machine.paused=True
    if fault=='health':health=replace(health,nav_ready=False)
    if fault=='stop':health=replace(health,stopped=False)
    if fault=='terminal':health=replace(health,motion_terminal=False)
    if fault=='distance':health=replace(health,pose=Pose2D(-1.,0.))
    if fault=='no_evidence':r.adapter.completed_approach=None
    r.adapter.get_health=lambda:health
    runner=AgentRunner.__new__(AgentRunner)
    runner.machine=machine;runner.adapter=r.adapter
    runner.local_completion=(('task','command'),'TARGET_APPROACHED')
    runner.owners=lambda:['other'] if fault=='owner' else []
    runner._complete_local()
    assert not state.ready and not state.deposited


@pytest.mark.parametrize('override',[False,True])
def test_single_entry_script_forwards_original_config_without_motion_gate_changes(tmp_path,override):
    from pathlib import Path
    import shutil,subprocess,json,os
    root=Path(__file__).resolve().parents[1]
    (tmp_path/'scripts').mkdir();(tmp_path/'config').mkdir();(tmp_path/'bin').mkdir()
    shutil.copy2(root/'scripts/start_ranger_single_robot.sh',tmp_path/'scripts')
    (tmp_path/'config/scene.yaml').write_text('task_timeout: 123.0\n')
    (tmp_path/'RangerVLM_env.bash').write_text(
        'case "${1:-local}" in local|--build|--legacy) ;; *) echo "invalid environment mode" >&2; return 1;; esac\n'
        'export RANGER_CONFIG="'+str(tmp_path/'config')+'"\n')
    fake=tmp_path/'bin/python3'
    fake.write_text('#!/usr/bin/python3\nimport sys,json\nfrom pathlib import Path\nprint(Path(sys.argv[2]).read_text().split()[-1] if sys.argv[1]=="-" else json.dumps(sys.argv[1:]))\n')
    fake.chmod(0o755)
    selected=tmp_path/'custom' if override else tmp_path/'config'
    if override:
        selected.mkdir();(selected/'scene.yaml').write_text('task_timeout: 456.0\n')
    options=['--config-dir',str(selected)] if override else []
    result=subprocess.run(['bash',str(tmp_path/'scripts/start_ranger_single_robot.sh'),*options,'--allow-motion','--target','cup'],
        env=dict(os.environ,PATH=str(tmp_path/'bin')+':'+os.environ['PATH']),capture_output=True,text=True,check=True)
    args=json.loads(result.stdout)
    assert args[:2]==['-m','multi_agent_nav.ranger_check']
    assert args[args.index('--config-dir')+1]==str(selected)
    assert args[args.index('--timeout')+1]==('456.0' if override else '123.0')
    assert args.count('--allow-motion')==1 and args[-2:]==['--target','cup']


@pytest.mark.parametrize('interruption',[None,'cancel','heartbeat','localization','owner','replacement'])
def test_real_cooperative_runner_holds_ready_until_outer_barrier(local_runtime,interruption,monkeypatch):
    from pathlib import Path
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'cooperative_ws/src/multi_agent_nav/tests'))
    from test_private_c import local_agent
    from multi_agent_nav.runner import AgentRunner
    from multi_agent_nav.models import Observation,Pose2D
    r=local_runtime;config,machine,_,command=local_agent()
    assert machine.accept(command,0.).accepted
    machine.take_effects()
    command=replace(command,kind='APPROACH_DEPOSIT',target_id='C',observation_id='c')
    machine.command=command
    machine.local_observations['c']=Observation(target_id='C',status='DETECTED',position_valid=True,
        observation_id='c',common_position=Pose2D(2.,0.))
    machine.frame=r.adapter.frame
    r.adapter.costmap_transform=lambda **_:Pose2D()
    runner=AgentRunner(machine,r.adapter,config['scene']['targets'],lambda _:None,config=config)
    machine.task_started_at=r.clock[0]
    health=r.adapter.get_health
    stopped=[False]
    r.adapter.get_health=lambda:replace(health(),stopped=stopped[0],footprint=[[.1,.1],[.1,-.1],[-.1,-.1],[-.1,.1]])
    runner.now=r.clock[0];runner._start_local()
    def tick(dt=.05):
        r.clock[0]+=dt
        machine.sync_received=machine.peer_received=r.clock[0]
        runner.tick(r.clock[0])
    tick();tick()
    assert len(r.requests)==len(r.plans)==1, machine.state()
    if interruption:
        if interruption=='cancel':machine.stop('external CANCEL')
        if interruption=='heartbeat':
            machine.sync_received=r.clock[0]-4.
        if interruption=='localization':
            r.adapter.get_health=lambda:replace(health(),localization_valid=False)
        if interruption=='owner':runner.owners=lambda:['other']
        if interruption=='replacement':machine.command=replace(command,command_id='new')
        runner.tick(r.clock[0])
        r.terminals[0].set_result(S(status=4))
        tick()
        assert not machine.state().ready and not machine.state().deposited
        assert not r.adapter.local_arrival_verified and not r.adapter.results
        return
    r.node.robot_pose=lambda:(1.2,0.,0.)
    tick(.21)
    assert r.adapter.approach.state=='APPROACH_STOPPING'
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.terminals[0].set_result(S(status=5))
    tick(.21);tick()
    assert runner.local_completion and not machine.state().deposited
    assert len(r.requests)==len(r.plans)==1 and not r.adapter.results
    stopped[0]=True;tick()
    assert machine.state().deposited and machine.state().reply=='SUCCEEDED'


def test_plan_callback_cannot_send_old_command_before_next_runner_tick(local_runtime):
    from multi_agent_nav.runner import AgentRunner
    r=local_runtime;response=Future();result=Future()
    r.node.path_planner.send_goal_async=lambda _:response
    runner=AgentRunner.__new__(AgentRunner)
    runner.adapter=r.adapter;runner.now=r.clock[0];runner.owners=lambda:[]
    runner.local_identity=('task','command');runner.local_steps=object()
    runner.machine=S(paused=False,command=S(task_id='task',command_id='command'),
        context=S(task_id='task'),sync_received=r.clock[0],peer_received=r.clock[0],lost_timeout=3.)
    r.adapter.motion_guard=runner.motion_guard
    a=start_approach(r)
    runner.machine.command=S(task_id='task',command_id='replacement')
    response.set_result(S(accepted=True,get_result_async=lambda:result,cancel_goal_async=lambda:Future()))
    result.set_result(S(status=4,result=S(path=S(poses=[S(pose=S(position=S(x=0.,y=0.)))]))))
    assert not r.requests and a.state=='FAILED'
    r.steps.close()


@pytest.mark.parametrize('fault',['health','owner'])
def test_independent_plan_callback_rechecks_existing_health(local_runtime,fault):
    from multi_agent_nav.go2_exploration import explore_steps
    from multi_agent_nav.models import Observation,Pose2D
    r=local_runtime;response=Future();result=Future()
    r.node.path_planner.send_goal_async=lambda _:response
    r.adapter.costmap_transform=lambda **_:Pose2D()
    original=r.adapter.get_health
    invalid=[False]
    r.adapter.get_health=lambda:replace(original(),nav_ready=not invalid[0],
        footprint=[[.1,.1],[.1,-.1],[-.1,-.1],[-.1,.1]])
    owners=[]
    config={'scene':{'resources':{},'targets':{'B':'cup'},'viewpoints':{'ranger':[dict(id='next',pose=[1.,0.,0.])]}},
        'robot_ranger':dict(stop_margin=.2,exploration_strategy='fixed_viewpoints',approach_on_detection=True)}
    steps=explore_steps(r.adapter,config,r.adapter.frame,200.,'cup',lambda _:None,lambda:owners,None,lambda:None,
        clock=lambda:r.clock[0],robot_id='ranger',target_id='B',
        initial_observation=Observation(target_id='B',status='DETECTED',position_valid=True,common_position=Pose2D(2.,0.)))
    next(steps);next(steps)
    a=r.adapter.approach
    if fault=='health':invalid[0]=True
    else:owners.append('other')
    response.set_result(S(accepted=True,get_result_async=lambda:result,cancel_goal_async=lambda:Future()))
    result.set_result(S(status=4,result=S(path=S(poses=[S(pose=S(position=S(x=0.,y=0.)))]))))
    assert not r.requests and a.state=='FAILED'
    steps.close()
