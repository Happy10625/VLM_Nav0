"""VLM recording with local SDK/ROS stubs; no provider requests or hardware."""
import base64
from concurrent.futures import Future
import json
import math
from types import SimpleNamespace as S

import cv2
import numpy as np
import pytest

from multi_agent_nav.adapters.go2 import ABCClient, Go2Adapter
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.models import Pose2D


def client_and_response(visible=False, content=None, failure=None):
    client=ABCClient.__new__(ABCClient)
    client.mode='single';client.model='stub-model';client._is_qwen=False
    client._structured_output_supported=True;client.image_detail='high';client.jpeg_quality=85
    client.last_raw_response=None
    payload=dict(target_visible=visible,object_match=visible,qualifier_match=visible,
        relation_match=visible,confidence=.9 if visible else 0.,
        target_pixel={'u':80,'v':50} if visible else None,
        evidence_pixel={'u':90,'v':50} if visible else None)
    text=json.dumps(payload) if content is None else content
    calls=[]
    def create(**kwargs):
        calls.append(kwargs)
        if failure:raise failure
        return S(id='completion-id',model='stub-model',_request_id='request-id',
            choices=[S(finish_reason='stop',message=S(content=text))])
    client.client=S(api_key='never-record-secret',chat=S(completions=S(create=create)))
    return client,calls,text


def evidence(tmp_path):
    from multi_agent_nav.scan_evidence import ScanEvidence
    e=ScanEvidence(tmp_path/'vlm',dict(scan_id='scan/unsafe/path',view_index=0,
        image_stamp_ns=123,frame_id='camera',capture_pose=dict(x=1.,y=2.,yaw=.3)))
    e.begin()
    return e


@pytest.mark.parametrize('visible',[False,True])
def test_each_view_keeps_exact_sent_jpeg_raw_text_and_annotated_result(tmp_path,visible):
    client,calls,text=client_and_response(visible);e=evidence(tmp_path);client.evidence=e
    rgb=np.full((100,160,3),(140,40,20),dtype=np.uint8);original=rgb.copy()
    results=client.infer_abc(rgb,{'A':'灭火器'});e.finish(results)
    sent=calls[0]['messages'][1]['content'][1]['image_url']['url'].split(',',1)[1]
    assert (e.directory/'input.jpg').read_bytes()==base64.b64decode(sent)
    assert (e.directory/'response.txt').read_text()==text
    request=json.loads((e.directory/'request.json').read_text())
    assert request['messages']==calls[0]['messages'] and request['model']=='stub-model'
    annotated=cv2.imread(str(e.directory/'result.png'))
    assert annotated.shape==rgb.shape and np.array_equal(rgb,original)
    if visible:
        assert annotated[38,80,2]>200 and annotated[38,80,1]<100
    meta=json.loads((e.directory/'metadata.json').read_text())
    assert meta['api_returned'] and meta['validation_success']
    assert meta['request_id']=='request-id' and meta['response_id']=='completion-id'
    assert meta['parsed_results']['A']['status']==('DETECTED' if visible else 'NOT_DETECTED')
    assert meta['parsed_results']['A']['grounding']['target_visible']==visible
    assert meta['image_stamp_ns']==123 and meta['capture_pose']['yaw']==.3
    assert e.directory.parent==tmp_path/'vlm'  # scan IDs are metadata, never paths
    assert 'never-record-secret' not in (e.directory/'metadata.json').read_text()


def test_invalid_model_json_is_retained_without_becoming_not_detected(tmp_path):
    client,calls,text=client_and_response(content='not JSON');e=evidence(tmp_path);client.evidence=e
    with pytest.raises(ValueError):client.infer_abc(np.zeros((100,160,3),dtype=np.uint8),{'A':'灭火器'})
    e.finish({},error_type='JSONDecodeError')
    meta=json.loads((e.directory/'metadata.json').read_text())
    assert meta['api_returned'] and not meta['validation_success']
    assert meta['parsed_results']=={} and meta['error_type']=='JSONDecodeError'
    assert (e.directory/'response.txt').read_text()==text
    assert (e.directory/'result.png').exists()


def test_provider_failure_keeps_input_and_safe_error_fields_only(tmp_path):
    class ProviderError(Exception):
        status_code=401;request_id='error-request-id'
    client,calls,text=client_and_response(failure=ProviderError('Authorization never-record-secret'))
    e=evidence(tmp_path);client.evidence=e
    with pytest.raises(ProviderError):client.infer_abc(np.zeros((100,160,3),dtype=np.uint8),{'A':'灭火器'})
    e.finish({},error_type='ProviderError')
    meta=json.loads((e.directory/'metadata.json').read_text())
    assert not meta['api_returned'] and meta['http_status']==401
    assert meta['request_id']=='error-request-id' and not meta['validation_success']
    assert (e.directory/'input.jpg').exists() and not (e.directory/'response.txt').exists()
    assert 'never-record-secret' not in (e.directory/'metadata.json').read_text()


def test_raw_model_text_redacts_configured_api_key(tmp_path):
    client,calls,text=client_and_response(content='never-record-secret')
    e=evidence(tmp_path);client.evidence=e
    with pytest.raises(ValueError):client.infer_abc(np.zeros((100,160,3),dtype=np.uint8),{'A':'灭火器'})
    assert (e.directory/'response.txt').read_text()=='[REDACTED]'


def test_input_record_failure_prevents_the_provider_call(tmp_path):
    client,calls,text=client_and_response();e=evidence(tmp_path);client.evidence=e
    (e.directory/'input.jpg').write_bytes(b'preserve existing data')
    with pytest.raises(OSError):client.infer_abc(np.zeros((100,160,3),dtype=np.uint8),{'A':'灭火器'})
    assert e.write_failed and calls==[]
    assert (e.directory/'input.jpg').read_bytes()==b'preserve existing data'


def scan_adapter(e,results=None,error=None):
    a=Go2Adapter.__new__(Go2Adapter);events=[];a.record_event=events.append
    a.config=dict(scan_angles=[0.,math.pi/2]);a.motions={};a.plans={};a.motion_fault=False
    a.vision=S(poll=lambda:None,cancel=lambda:None,available=True)
    a.frame=FrameAdapter('fixture','1','lab_ref','',[0,0,0])
    a.get_health=lambda:S(nav_ready=not a.motion_fault,localization_valid=True,stopped=True,pose=Pose2D())
    a.node=S(ground_pixel_with_reason=lambda *args:(None,''),stop_pub=S(publish=lambda m:None),spinner=S())
    pending=Future()
    if error:pending.set_exception(error)
    else:pending.set_result(results)
    scan=dict(reply=Future(),id='scan',targets={'A':'灭火器'},index=0,phase='INFER',
        initial=0.,pending=pending,observations=[],after=0.,deadline=float('inf'),
        snapshot=S(stamp=S(sec=1,nanosec=0)),evidence=e,
        navigation=('native-nav',dict(task='task',command='command',kind='VIEWPOINT_NAV',target=[1.,2.,0.])))
    a.scan=scan
    return a,scan,events


def test_completed_view_is_logged_before_later_spin_abort(tmp_path):
    client,calls,text=client_and_response();e=evidence(tmp_path);client.evidence=e
    results=client.infer_abc(np.zeros((100,160,3),dtype=np.uint8),{'A':'灭火器'});e.finish(results)
    a,scan,events=scan_adapter(e,results)
    a.tick()
    event=next(r for r in events if r.get('event')=='VLM_VIEW_RESULT')
    assert event['view_index']==0 and event['consumer_status']=='APPLIED'
    assert event['observations'][0]['status']=='NOT_DETECTED' and event['task_id']=='task'
    assert scan['phase']=='SPIN' and not scan['reply'].done()
    spin=Future();spin.set_result('ABORTED');a._motion=lambda *args:('spin',spin)
    a.tick();a.tick()
    assert isinstance(scan['reply'].exception(),RuntimeError)
    failure=next(r for r in events if r.get('event')=='SCAN_FAILED')
    assert failure['view_index']==1 and failure['status']=='ABORTED'
    assert 'view 2' in failure['reason'] and 'ABORTED' in failure['reason']
    assert a.scan is None and (e.directory/'input.jpg').exists()
    assert (e.directory/'metadata.json').exists()


def test_evidence_failure_requests_stop_and_preserves_failure_flag(tmp_path):
    e=evidence(tmp_path);e.write_failed=True
    a,scan,events=scan_adapter(e,error=RuntimeError('safe failure'))
    a.tick()
    assert a.evidence_write_failed and a.motion_fault and a.scan is None
    assert any(r.get('event')=='STOP_COMMAND_SENT' for r in events)


def test_timeout_has_a_per_view_failure_event(tmp_path):
    e=evidence(tmp_path)
    a,scan,events=scan_adapter(e,error=TimeoutError('vision timeout'))
    a.tick()
    event=next(r for r in events if r.get('event')=='VLM_VIEW_RESULT')
    assert event['consumer_status']=='TIMEOUT' and event['error_type']=='TimeoutError'
    assert event['observations'][0]['status']=='API_ERROR'


@pytest.mark.parametrize('stop_target,detected,located,ends', [
    ('B','B',True,True), ('B','B',False,True), ('B','A',True,False),
    (None,'B',True,False),
])
def test_scan_can_stop_after_requested_target_without_starting_another_spin(stop_target,detected,located,ends):
    from vlm_nav.models import VLMResult, Pixel
    from multi_agent_nav.models import Observation
    result=VLMResult(True,.95,Pixel(2,2),None,True,True,True)
    results={t:('DETECTED',result) if t==detected else ('NOT_DETECTED',None) for t in 'ABC'}
    a,_,events=scan_adapter(None,results)
    a.config['robot_id']='ranger';a.scan=None
    future=a.start_scan(dict(A='灭火器',B='纸巾',C='箱子'),'scan',stop_on_target=stop_target)
    scan=a.scan
    pending=Future();pending.set_result(results)
    scan.update(phase='INFER',pending=pending,index=1,
        snapshot=S(stamp=S(sec=1,nanosec=0)),
        observations=[Observation(target_id='B',view_index=0,status='API_ERROR')])
    a.node.ground_pixel_with_reason=lambda *args:((2.,3.,1.),'ok') if located else (None,'invalid depth')
    a.tick()
    assert future.done()==ends
    if ends:
        assert a.scan is None
        assert any(o.target_id=='B' and o.status=='DETECTED' and o.position_valid==located for o in future.result())
        event=next(e for e in events if e.get('event')=='SCAN_TARGET_DETECTED')
        assert event['target_id']=='B' and event['view_index']==1
        a._motion=lambda *args:pytest.fail('detected target must not trigger the next spin')
        a.tick()
    else:
        assert a.scan['phase']=='SPIN' and not future.done()


def test_stop_target_must_belong_to_requested_scan():
    a,_,_=scan_adapter(None,{})
    a.scan=None
    with pytest.raises(ValueError,match='stop target'):
        a.start_scan({'A':'灭火器'},'scan',stop_on_target='B')


def test_frame_worker_records_artifacts_without_blocking_ros_or_using_new_images(tmp_path):
    import threading,time
    from multi_agent_nav.adapters.go2 import AsyncVision
    client,calls,text=client_and_response();a,scan,events=scan_adapter(None,results={})
    a.scan=None;a.client=client;a.scan_evidence_dir=tmp_path/'vlm';a.scan_viewpoint_id='p1'
    a.config.update(api_timeout=2.);a.vision=AsyncVision(2.)
    a.last_motion=('native-nav',dict(task='task',command='command',kind='VIEWPOINT_NAV',target=[1.,2.,.7]))
    snapshot=S(stamp=S(sec=3,nanosec=123),frame_id='camera',sequence=7,
        rgb=np.full((100,160,3),60,dtype=np.uint8),captured_monotonic=time.monotonic()+1.)
    a.node.latest_snapshot=snapshot;a.node.vlm_input_ready=lambda now:True
    original=client.client.chat.completions.create;thread_names=[]
    def create(**kwargs):
        thread_names.append(threading.current_thread().name)
        assert (a.scan['evidence'].directory/'input.jpg').exists()
        return original(**kwargs)
    client.client.chat.completions.create=create
    try:
        reply=a.start_scan({'A':'灭火器'},'scan')
        a.tick();a.tick()
        end=time.monotonic()+3.
        while a.scan['phase']=='INFER' and time.monotonic()<end:a.tick();time.sleep(.001)
        result=next(r for r in events if r.get('event')=='VLM_VIEW_RESULT')
        directory=a.scan['evidence'].directory
        meta=json.loads((directory/'metadata.json').read_text())
        assert meta['viewpoint_id']=='p1' and meta['image_stamp_ns']==3_000_000_123
        assert meta['image_sequence']==7 and meta['navigation_goal_pose']==[1.,2.,.7]
        assert result['consumer_status']=='APPLIED' and client.evidence is None
        assert len(calls)==1 and all(name.startswith('cooperative-vision') for name in thread_names)
        a.stop_motion('fixture cleanup')
        assert reply.cancelled() and (directory/'response.txt').read_text()==text
    finally:a.vision.close()


def test_canceling_pending_view_records_cancellation_after_zero_command(tmp_path):
    e=evidence(tmp_path)
    a,scan,events=scan_adapter(e,results={})
    published=[];a.node.stop_pub.publish=lambda m:published.append(m)
    a.record_event=lambda row:events.append(dict(row,zero_already_sent=bool(published)))
    a.stop_motion('fixture cancellation')
    event=next(r for r in events if r.get('event')=='VLM_VIEW_RESULT')
    assert event['consumer_status']=='CANCELED' and event['observations']==[]
    assert event['zero_already_sent'] and scan['reply'].cancelled()


@pytest.mark.parametrize('ended',['timeout','cancel'])
def test_late_file_failure_after_scan_ends_still_requests_stop(tmp_path,ended):
    e=evidence(tmp_path);a,scan,events=scan_adapter(e,error=TimeoutError())
    a._scan_evidence=e
    if ended=='cancel':a.stop_motion('fixture cancellation')
    else:
        scan['index']=len(a.config['scan_angles'])-1
        a.tick();a.tick()
    assert a.scan is None
    events.clear();e.write_failed=True
    a.tick()
    assert a.evidence_write_failed and a.motion_fault
    assert any(r.get('event')=='STOP_COMMAND_SENT' for r in events)


def test_timeout_does_not_start_next_spin_until_worker_finishes(tmp_path):
    a,scan,events=scan_adapter(evidence(tmp_path),error=TimeoutError())
    a.vision.available=False
    a._motion=lambda *args:pytest.fail('started Spin while the old worker was running')
    a.tick();a.tick()
    assert scan['phase']=='SPIN' and not scan['reply'].done()


def test_canceled_worker_propagates_late_disk_error_without_ros_tick(tmp_path,monkeypatch):
    import threading,time
    from multi_agent_nav.adapters.go2 import AsyncVision
    from multi_agent_nav.scan_evidence import ScanEvidence
    client,calls,text=client_and_response();a,scan,events=scan_adapter(None,results={})
    a.scan=None;a.client=client;a.scan_evidence_dir=tmp_path/'vlm'
    a.config.update(api_timeout=2.);a.vision=AsyncVision(2.)
    a.node.latest_snapshot=S(stamp=S(sec=3,nanosec=0),frame_id='camera',sequence=1,
        rgb=np.zeros((100,160,3),dtype=np.uint8),captured_monotonic=time.monotonic()+1.)
    a.node.vlm_input_ready=lambda now:True
    entered=threading.Event();release=threading.Event()
    original=client.client.chat.completions.create
    def create(**kwargs):
        entered.set();assert release.wait(3.)
        return original(**kwargs)
    client.client.chat.completions.create=create
    original_write=ScanEvidence._write
    def write(e,name,content):
        if name=='response.txt':
            e.write_failed=True
            raise OSError('fixture disk failure')
        original_write(e,name,content)
    monkeypatch.setattr(ScanEvidence,'_write',write)
    try:
        a.start_scan({'A':'灭火器'},'scan');a.tick();a.tick()
        assert entered.wait(2.)
        job=a.vision.job
        a.stop_motion('fixture cancellation');assert a.scan is None
        release.set()
        with pytest.raises(OSError):job.result(timeout=3.)
        assert a.evidence_write_failed and a.motion_fault and client.evidence is None
    finally:
        release.set();a.vision.close()
