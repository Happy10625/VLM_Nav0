"""Bounded in-place depth recovery through the actual Ranger scan adapter."""
import math
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.models import Observation
from test_ranger_single_robot_parity import local_runtime


@pytest.fixture
def probe(local_runtime):
    r=local_runtime
    r.reply=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.scan=r.adapter.scan
    r.scan.update(current_view_only=False,index=2,headings=[0.,.5,1.,1.5],initial=0.,
        observations=[Observation(target_id='B',view_index=i,status='NOT_DETECTED') for i in (0,1)])
    return r


def observe(r,depth=float('nan')):
    r.clock[0]+=.5
    r.frame(depth);r.adapter.tick();r.adapter.tick()


def finish_spin(r):
    assert hasattr(r.requests[-1],'target_yaw')
    r.terminals[-1].set_result(S(status=4))


@pytest.mark.parametrize('depth',[float('nan'),7.])
def test_invalid_depth_rotates_at_most_ten_degrees_without_navigation(probe,depth):
    r=probe;observe(r,depth)
    assert 0<abs(r.requests[-1].target_yaw)<=math.radians(10.)
    assert r.node.depth_reobserve_attempts==1
    assert not r.plans and not r.reply.done()


@pytest.mark.parametrize('invalid',['depth','not_detected','api_error','timeout','projection'])
def test_three_attempts_include_invalid_api_and_continue_original_scan(probe,invalid):
    r=probe;observe(r)
    if invalid=='not_detected':r.result[0]=replace(r.result[0],target_visible=False)
    if invalid=='api_error':r.adapter.client.infer_abc=lambda *_: {}
    if invalid=='timeout':
        def timeout(*_):raise TimeoutError('request timeout')
        r.adapter.client.infer_abc=timeout
        def submit(fn):
            f=Future()
            try:f.set_result(fn())
            except Exception as error:f.set_exception(error)
            return f
        r.adapter.vision.submit=submit
    if invalid=='projection':r.node.ground_pixel_with_reason=lambda *args,**kwargs:(None,'projection_error')
    for i in range(3):
        assert len(r.requests)==i+1
        finish_spin(r);observe(r)
    assert len(r.requests)==3 and not r.plans
    assert r.scan['index']==3 and r.scan['initial']==0. and r.scan['phase']=='SPIN'
    assert [o.view_index for o in r.scan['observations']]==[0,1,2]
    assert r.scan['observations'][-1].status=='UNCERTAIN'
    assert not r.scan['observations'][-1].position_valid
    assert r.node.state=='SCANNING' and not r.reply.done()


def test_valid_depth_keeps_confirmation_without_more_spins(probe):
    r=probe;observe(r);finish_spin(r)
    for i in range(3):
        observe(r,2.)
        assert r.reply.done()==(i==2)
    assert r.node.state=='APPROACHING' and r.reply.result()[-1].status=='DETECTED'
    assert len(r.requests)==1 and not r.plans


def test_initial_single_view_finishes_uncertain_after_three_attempts(probe):
    r=probe;r.scan.update(current_view_only=True,index=0,observations=[])
    observe(r)
    for _ in range(3):finish_spin(r);observe(r)
    assert r.reply.done() and r.adapter.scan is None
    assert r.reply.result()[0].status=='UNCERTAIN'
    assert len(r.requests)==3


def test_partial_confirmation_then_invalid_api_still_uses_same_budget(probe):
    r=probe;observe(r);finish_spin(r);observe(r,2.)
    assert r.node.state=='TARGET_CONFIRMING'
    r.adapter.client.infer_abc=lambda *_: {}
    observe(r)
    for _ in range(2):finish_spin(r);observe(r)
    assert len(r.requests)==3 and r.scan['index']==3


def test_exhausted_budget_is_not_reset_by_next_direction(probe):
    r=probe;observe(r)
    for _ in range(3):finish_spin(r);observe(r)
    r.scan.update(phase='FRAME',after=r.clock[0])
    observe(r)
    assert len(r.requests)==3 and r.scan['index']==4


def test_new_observation_point_gets_new_budget(probe):
    r=probe;r.scan.update(current_view_only=True,index=0,observations=[])
    observe(r)
    for _ in range(3):finish_spin(r);observe(r)
    r.adapter.start_scan({'B':'cup'},'new-point',stop_on_target='B',current_view_only=True)
    observe(r)
    assert len(r.requests)==4 and r.node.depth_reobserve_attempts==1


def test_sensor_wait_keeps_attempt_budget_and_scan_direction(probe):
    r=probe;observe(r);finish_spin(r);observe(r);finish_spin(r)
    r.node.enter_sensor_wait('RGB-D gap')
    assert r.node.recover_from_sensor_wait()
    assert r.scan['index']==2 and r.scan['initial']==0.
    observe(r);finish_spin(r);observe(r)
    assert len(r.requests)==3 and r.scan['index']==3


def test_third_rotation_can_confirm_target_without_fourth_rotation(probe):
    r=probe;observe(r)
    for _ in range(2):finish_spin(r);observe(r)
    finish_spin(r)
    for _ in range(3):observe(r,2.)
    assert r.reply.done() and r.reply.result()[-1].status=='DETECTED'
    assert len(r.requests)==3 and not r.plans


def test_configured_larger_limit_cannot_exceed_three(probe):
    r=probe;r.node.p.depth_reobserve_attempt_limit=10
    observe(r)
    for _ in range(3):finish_spin(r);observe(r)
    assert len(r.requests)==3 and r.scan['index']==3


def test_cooperative_pause_preserves_used_rotation_budget(probe):
    r=probe;observe(r);finish_spin(r)
    r.adapter.pause_local_execution('peer heartbeat timeout')
    r.clock[0]+=10
    r.adapter.resume_local_execution()
    observe(r)
    for _ in range(2):finish_spin(r);observe(r)
    assert len(r.requests)==3 and r.scan['index']==3


def test_invalid_intrinsics_during_reobservation_consume_attempt(probe):
    r=probe;observe(r);finish_spin(r)
    r.clock[0]+=.5;r.frame(2.)
    r.node.latest_snapshot=replace(r.node.latest_snapshot,intrinsics=(0.,0.,5.,5.))
    r.adapter.tick();r.adapter.tick()
    assert r.node.depth_reobserve_attempts==2
    assert len(r.requests)==2 and not r.plans


@pytest.mark.parametrize('active',[False,True])
def test_invalid_depth_with_invalid_intrinsics_still_has_bounded_rotation(probe,active):
    r=probe
    if active:observe(r);finish_spin(r)
    r.clock[0]+=.5;r.frame(float('nan'))
    r.node.latest_snapshot=replace(r.node.latest_snapshot,intrinsics=(0.,0.,5.,5.))
    r.adapter.tick();r.adapter.tick()
    assert r.node.depth_reobserve_attempts==(2 if active else 1)
    assert abs(r.requests[-1].target_yaw)<=math.radians(10.) and not r.plans
