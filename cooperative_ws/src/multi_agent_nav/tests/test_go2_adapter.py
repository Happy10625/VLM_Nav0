import time
from types import SimpleNamespace
import numpy as np
import pytest
from multi_agent_nav.adapters.go2 import ABCClient, observations_from_results, AsyncVision, Go2Adapter
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.models import Pose2D
from vlm_nav.models import VLMResult, Pixel

def grounding(visible):
    return dict(object_match=visible, qualifier_match=visible,
                relation_match=visible, confidence=.9,
                target_pixel={'u':2,'v':2} if visible else None,
                evidence_pixel={'u':2,'v':2} if visible else None)

def test_abc_structure_and_invalid_outputs():
    payload={t:dict(status='DETECTED' if t=='A' else 'NOT_DETECTED', grounding=grounding(t=='A')) for t in 'ABC'}
    results=ABCClient.parse(payload,5,5)
    assert results['A'][0]=='DETECTED' and results['B'][0]=='NOT_DETECTED'
    with pytest.raises(ValueError): ABCClient.parse({'A':payload['A']},5,5)
    payload['C']['grounding']['confidence']=float('nan')
    with pytest.raises(ValueError): ABCClient.parse(payload,5,5)

@pytest.mark.parametrize('flags',[(a,b,c) for a in (False,True) for b in (False,True) for c in (False,True)])
def test_abc_visibility_is_computed_locally(flags):
    visible=all(flags)
    payload={t:dict(status='DETECTED' if visible else 'UNCERTAIN',grounding=grounding(visible)) for t in 'ABC'}
    for item in payload.values():
        item['grounding'].update(zip(('object_match','qualifier_match','relation_match'),flags))
    result=ABCClient.parse(payload,5,5)
    assert all(value.target_visible==visible for status,value in result.values())
    assert all('target_visible' not in item['grounding'] for item in payload.values())

@pytest.mark.parametrize('fault',['model_visible','non_boolean','missing_flag','wrong_status','missing_pixel','bad_pixel'])
def test_abc_derived_visibility_keeps_strict_validation(fault):
    payload={t:dict(status='DETECTED',grounding=grounding(True)) for t in 'ABC'}
    g=payload['B']['grounding']
    if fault=='model_visible':g['target_visible']=True
    elif fault=='non_boolean':g['object_match']=1
    elif fault=='missing_flag':del g['relation_match']
    elif fault=='wrong_status':payload['B']['status']='UNCERTAIN'
    elif fault=='missing_pixel':g['evidence_pixel']=None
    else:g['target_pixel']={'u':1001,'v':5}
    with pytest.raises(ValueError):ABCClient.parse(payload,1280,720,'normalized_1000')

def test_detected_without_depth_is_not_navigation_position():
    frame=FrameAdapter('g','v','lab_ref','a',(0,0,0))
    result=VLMResult(True,.9,Pixel(2,2),None,True,True,True)
    snapshot=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0))
    outputs=observations_from_results({'A':('DETECTED',result),'B':('NOT_DETECTED',None),'C':('UNCERTAIN',None)},
        snapshot,lambda *args:(None,'invalid depth'),frame,'scan',0)
    assert len(outputs)==3 and outputs[0].status=='DETECTED' and not outputs[0].position_valid
    outputs=observations_from_results({'A':('DETECTED',result)},snapshot,lambda *args:((2,3,1),'ok'),frame,'scan',0)
    assert outputs[0].common_position==Pose2D(2,3) and outputs[0].position_valid

def test_async_vision_timeout_does_not_block_and_drops_canceled_result():
    worker=AsyncVision(.02)
    def slow():
        time.sleep(.08)
        return 'late'
    started=time.monotonic()
    future=worker.submit(slow)
    assert time.monotonic()-started<.02
    time.sleep(.03)
    worker.poll()
    assert future.done() and isinstance(future.exception(),TimeoutError)
    assert not worker.available  # one in-flight job; cannot accumulate timed-out work
    time.sleep(.08)
    worker.poll()
    assert worker.available
    future=worker.submit(slow)
    worker.cancel()
    assert future.cancelled()
    worker.close()

def test_motion_pending_cancel_then_late_handle_is_canceled_and_identity_retained():
    from concurrent.futures import Future
    from collections import deque
    adapter=Go2Adapter.__new__(Go2Adapter)
    adapter.motions={};adapter.results=deque();adapter.motion_fault=False
    sent=Future();terminal=Future()
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:terminal,
        goal_id=SimpleNamespace(uuid=[1]*16))
    canceled=[]
    handle.cancel_goal_async=lambda:canceled.append(True)
    client=SimpleNamespace(send_goal_async=lambda _:sent)
    goal_id,reply=adapter._motion(client,object(),'task','command','PICKUP_NAV')
    adapter.cancel_navigation(goal_id)
    assert adapter.motions and not reply.done()
    sent.set_result(handle)
    assert canceled
    assert adapter.last_action_uuid=='01'*16
    terminal.set_result(SimpleNamespace(status=5))
    assert not adapter.motions and reply.result()=='CANCELED'
    result=adapter.results.pop()
    assert (result.task_id,result.command_id,result.goal_id,result.goal_kind)==('task','command',goal_id,'PICKUP_NAV')

def test_observation_only_node_cannot_enable_old_autonomy(monkeypatch,tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    from vlm_nav.vlm_navigator import VLMNavigator
    monkeypatch.setenv('ROS_DOMAIN_ID','31');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    rclpy.init(args=[])
    node=VLMNavigator(observation_only=True)
    try:
        assert node.get_name()=='multi_agent_go2_rgbd'
        assert len(list(node.timers))==1 and node.worker is None
        assert not node.set_parameters([Parameter('enabled',value=True)])[0].successful
    finally:
        node.destroy_node();rclpy.shutdown()

def test_delayed_odometry_cannot_confirm_physical_stop():
    adapter=Go2Adapter.__new__(Go2Adapter)
    adapter.config={'odom_max_age':.5,'stationary_speed':.02}
    adapter.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    adapter.stationary_since=1;adapter.odom_at=1
    v=SimpleNamespace(linear=SimpleNamespace(x=0,y=0,z=0),angular=SimpleNamespace(z=0))
    message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)),twist=SimpleNamespace(twist=v))
    adapter.on_odometry(message)
    assert adapter.stationary_since is None and adapter.odom_at==0


def test_discarded_request_does_not_block_resend_or_overwrite_new_reply():
    from threading import Event
    started,release=Event(),Event()
    worker=AsyncVision(4.,max_workers=2)
    def blocked():
        started.set();release.wait(2.);return 'old'
    try:
        old=worker.submit(blocked)
        assert started.wait(1.)
        worker.discard()
        new=worker.submit(lambda:'new')
        deadline=time.monotonic()+1.
        while not new.done() and time.monotonic()<deadline:
            worker.poll();time.sleep(.001)
        assert new.result(timeout=.1)=='new'
        release.set()
        worker.poll()
        assert old.cancelled() and new.result()=='new'
    finally:
        release.set();worker.close()


def test_cancelled_http_requests_release_capacity_across_viewpoints(monkeypatch):
    import asyncio
    from threading import Event
    from types import SimpleNamespace as S
    from multi_agent_nav.adapters.vision import CancellableCompletion
    started=[Event() for _ in range(2)]
    stopped=[Event() for _ in range(2)]
    class Provider:
        def __init__(self,**options):
            self.http=options['http_client']
            self.chat=S(completions=S(create=self.create))
        async def __aenter__(self):return self
        async def __aexit__(self,*args):await self.http.aclose()
        async def create(self,**request):
            i=request['attempt'];started[i].set()
            try:await asyncio.sleep(100.)
            finally:stopped[i].set()
    monkeypatch.setattr('openai.AsyncOpenAI',Provider)
    worker=AsyncVision(4.,max_workers=2)
    clients=[]
    try:
        for i in range(2):
            client=CancellableCompletion(api_key='fake',base_url='http://unused',timeout=4.,max_retries=0)
            clients.append(client)
            reply=worker.submit(lambda i=i,client=client:client.create(attempt=i))
            reply.add_done_callback(lambda f,client=client:client.close() if f.cancelled() else None)
            assert started[i].wait(1.)
            worker.discard()
        replacement=worker.submit(lambda:'next view')
        deadline=time.monotonic()+1.
        while not replacement.done() and time.monotonic()<deadline:
            worker.poll();time.sleep(.001)
        assert replacement.result(timeout=.1)=='next view'
        assert all(e.wait(1.) for e in stopped)
        canceled=CancellableCompletion(api_key='fake',base_url='http://unused',timeout=4.,max_retries=0)
        canceled.close()
        import pytest
        with pytest.raises(TimeoutError):canceled.create(attempt=0)
    finally:
        for client in clients:client.close()
        worker.close()
