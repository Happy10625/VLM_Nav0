from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.mock import MockMission
from test_go2_health_diagnostics import healthy_adapter
from test_private_c import local_agent, private_config


@pytest.mark.parametrize('fault,code',[
    ('odom','odom_fresh.receipt_stale'),('planner','planner_server.unavailable'),
    ('costmap','local_costmap.receipt_stale'),('map','active_task_map.verification_stale'),
    ('keepout','active_task_keepout.mask_unverified'),('motion','motion_fault_clear.cancel_timeout'),
])
def test_health_exposes_failed_subitem_and_evidence(fault,code):
    a=healthy_adapter()
    if fault=='odom':a.odom_at-=2.
    elif fault=='planner':
        a.node.path_planner.server_is_ready=lambda:False
        a.node.p.compute_path_action='/compute_path_to_pose'
    elif fault=='costmap':a.costmap_at-=2.
    elif fault=='map':
        a.task_map_ready=False
        a.task_map_health=dict(path_matches=True,digest_verified=True,verification_age_s=4.,max_age_s=3.)
    elif fault=='keepout':
        a.task_keepout_ready=False
        a.task_keepout_health=dict(mask_matches=False,info_matches=True)
    else:
        a.motion_fault=True;a.motion_fault_reason='cancel_timeout'
    health=a.get_health()
    assert not health.nav_ready and code in health.nav_ready_reason
    if fault in ('odom','costmap','map'):assert 'max_age_s=' in health.nav_ready_reason
    if fault=='planner':assert '/compute_path_to_pose' in health.nav_ready_reason
    assert code in a.last_health_details['nav_ready_failures']


def test_detailed_nav_pause_remains_recoverable_and_manual_pause_does_not():
    _,machine,adapter,command=local_agent()
    machine.accept(command,0);machine.take_effects()
    health=adapter.get_health()
    failed=replace(health,nav_ready=False,nav_ready_reason='local_costmap.receipt_stale(receipt_age_s=1.2,max_age_s=1.0)')
    machine.tick(.1,failed)
    assert 'local_costmap.receipt_stale' in machine.state().reason
    machine.tick(.2,health)
    assert not machine.paused and machine.state().reason==''
    machine.tick(.3,failed)
    machine.accept(replace(command,action='PAUSE'),.4)
    machine.tick(.5,health)
    assert machine.paused and machine.state().reason=='coordinator pause/cancel'


def test_coordinator_and_peer_keep_root_subitem_and_resume_same_task():
    mission=MockMission(private_config());assert mission.start().accepted
    mission.run(4)
    adapter=mission.adapters['ranger'];get_health=adapter.get_health
    adapter.get_health=lambda:replace(get_health(),nav_ready=False,
        nav_ready_reason='odom_fresh.header_stale(header_age_s=2.0,max_age_s=1.0)')
    task=mission.coordinator.task_id
    mission.run(5)
    assert mission.coordinator.phase=='PAUSED'
    assert 'odom_fresh.header_stale' in mission.coordinator.reason
    assert 'odom_fresh.header_stale' in mission.machines['go2'].state().reason
    adapter.get_health=get_health;mission.run(200)
    assert mission.coordinator.task_id==task and mission.coordinator.phase=='DONE'


def test_localization_plus_nav_detail_never_becomes_auto_resumable():
    _,machine,adapter,command=local_agent()
    machine.accept(command,0);machine.take_effects()
    health=adapter.get_health()
    machine.tick(.1,replace(health,localization_valid=False,nav_ready=False,
        nav_ready_reason='planner_server.unavailable'))
    machine.tick(.2,health);machine.tick(.3,health)
    assert machine.paused and 'localization_valid' in machine.state().reason


def test_rejected_odom_header_is_distinct_from_missing_or_silent_publisher():
    from nav_msgs.msg import Odometry
    from test_field_preparation import stamp
    a=healthy_adapter()
    message=Odometry();message.header.stamp=stamp(7.)
    a.on_odometry(message)
    health=a.get_health()
    assert 'odom_fresh.header_stale' in health.nav_ready_reason
    assert 'header_age_s=3.000' in health.nav_ready_reason
    message.header.stamp=stamp(10.)
    a.on_odometry(message)
    assert a.get_health().nav_ready_reason==''


def test_cancel_timeout_produces_specific_latched_fault():
    import time
    a=healthy_adapter();a.scan=None;a.vision=S(poll=lambda:None)
    a.motions={'g':dict(cancel=True,since=time.monotonic()-6.)}
    a.tick()
    assert 'motion_fault_clear.cancel_timeout' in a.get_health().nav_ready_reason
    a._set_motion_fault('action_result_error')
    assert 'motion_fault_clear.cancel_timeout' in a.get_health().nav_ready_reason


@pytest.mark.parametrize('received,expected',[(False,'mask_missing'),(True,'mask_mismatch')])
def test_keepout_missing_message_is_distinct_from_mismatched_payload(received,expected):
    a=healthy_adapter();a.task_keepout_ready=False
    a.task_keepout_health=dict(mask_matches=False,info_matches=True,mask_received=received,info_received=True)
    assert 'active_task_keepout.'+expected in a.get_health().nav_ready_reason


@pytest.mark.parametrize('fault,expected',[
    ('missing','no_payload'),('receipt','receipt_stale'),('header','header_stale'),
    ('rejected','rejected_header_stale'),('tf','tf_unavailable')])
def test_costmap_failures_expose_the_existing_validation_stage(fault,expected):
    from test_field_preparation import costmap,stamp
    a=healthy_adapter()
    if fault=='missing':a.costmap=None
    elif fault=='receipt':a.costmap_at-=2.
    elif fault=='header':a.costmap.header.stamp=stamp(8.)
    elif fault=='rejected':
        payload=costmap();payload.header.stamp=stamp(8.);a.on_costmap(payload)
    else:
        a.costmap.header.frame_id='odom'
        original=a.node.tf_buffer.lookup_transform
        def lookup(target,source,*args):
            if source=='map':raise RuntimeError('missing costmap TF')
            return original(target,source,*args)
        a.node.tf_buffer.lookup_transform=lookup
    health=a.get_health()
    assert 'local_costmap.'+expected in health.nav_ready_reason
    assert health.localization_valid


def test_ranger_execution_excludes_rgbd_wait_from_nav_reason():
    from multi_agent_nav.adapters.ranger import RangerAdapter
    a=healthy_adapter();a.__class__=RangerAdapter
    a.node.execution_active=True;a.node.rgbd_sync_status['healthy']=False
    health=a.get_health()
    assert health.nav_ready and health.nav_ready_reason==''
    a.node.path_planner.server_is_ready=lambda:False
    health=a.get_health()
    assert 'planner_server.unavailable' in health.nav_ready_reason
    assert 'rgbd_ready' not in health.nav_ready_reason
    a.node.execution_active=False
    assert 'rgbd_ready' in a.get_health().nav_ready_reason


def test_startup_lists_all_failed_checks_without_creating_a_task():
    from multi_agent_nav.mock import make_machine
    machine,_=make_machine(private_config(),'ranger',True)
    a=healthy_adapter();a.node.path_planner.server_is_ready=lambda:False;a.odom_at=0.
    machine.tick(.1,a.get_health())
    state=machine.state()
    assert state.run_state=='PREPARING' and not state.task_id and not machine.paused
    assert 'planner_server.unavailable' in state.reason and 'odom_fresh.no_valid_message' in state.reason


def test_detailed_reason_keeps_runner_checkpoint_instead_of_closing_generator():
    from multi_agent_nav.runner import AgentRunner
    config,machine,adapter,command=local_agent()
    adapter.pause_local_execution=lambda _:None
    adapter.resume_local_execution=lambda:None
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config)
    machine.accept(command,0);machine.take_effects()
    progressed=[]
    def steps():
        progressed.append('original-view');yield
        progressed.append('continued');yield
    runner.local_steps=steps();runner.local_identity=(command.task_id,command.command_id)
    original=runner.local_steps;next(original)
    healthy=adapter.get_health
    adapter.get_health=lambda:replace(healthy(),nav_ready=False,nav_ready_reason='local_costmap.receipt_stale')
    runner.tick(.1)
    assert machine.local_suspended and runner.local_steps is original
    assert progressed==['original-view']
    adapter.get_health=healthy;runner.tick(.2)
    assert not machine.paused and runner.local_steps is original
    assert progressed==['original-view','continued']


def test_changed_nav_blocker_replaces_obsolete_diagnostics_without_another_stop():
    _,machine,adapter,command=local_agent()
    machine.accept(command,0);machine.take_effects()
    health=adapter.get_health()
    machine.tick(.1,replace(health,nav_ready=False,nav_ready_reason='odom_fresh.receipt_stale'))
    effects=machine.tick(.2,replace(health,nav_ready=False,nav_ready_reason='planner_server.unavailable'))
    assert machine.paused and not effects
    assert 'planner_server.unavailable' in machine.state().reason
    assert 'odom_fresh' not in machine.state().reason


def test_changed_nav_blocker_updates_peer_and_coordinator_diagnostics():
    mission=MockMission(private_config());assert mission.start().accepted
    mission.run(4)
    adapter=mission.adapters['ranger'];healthy=adapter.get_health;reason=['odom_fresh.receipt_stale']
    adapter.get_health=lambda:replace(healthy(),nav_ready=False,nav_ready_reason=reason[0])
    mission.run(5)
    reason[0]='planner_server.unavailable';mission.run(5)
    assert 'planner_server.unavailable' in mission.coordinator.reason
    assert 'odom_fresh' not in mission.coordinator.reason
    assert 'planner_server.unavailable' in mission.machines['go2'].state().reason
    assert mission.coordinator.phase=='PAUSED'


def test_ranger_logs_changed_subitem_but_not_every_age_update():
    import json,time
    from multi_agent_nav.adapters.ranger import RangerAdapter
    a=healthy_adapter();a.__class__=RangerAdapter;a.node.execution_active=True
    messages=[];a.node.get_logger=lambda:S(warning=messages.append)
    a.get_health();a.odom_at-=2.
    a.get_health();a.get_health()
    assert len(messages)==1
    a.odom_at=time.monotonic();a.node.path_planner.server_is_ready=lambda:False
    a.get_health();a.get_health()
    assert len(messages)==2
    latest=json.loads(messages[-1].split(': ',1)[1])
    assert 'planner_server.unavailable' in latest['reason']
    assert 'odom_fresh' not in latest['reason']
