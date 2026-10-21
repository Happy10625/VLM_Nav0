"""Same safety gates with failure-time evidence; no hardware or API calls."""
import json
import time
from types import SimpleNamespace as S

import numpy as np
import pytest


def healthy_adapter():
    from test_field_preparation import adapter,costmap,stamp
    a=adapter();del a.get_health
    now=time.monotonic()
    a.config.update(stationary_duration=.5,pose_max_age=1.)
    a.node.nav_ready=a.node.control_armed=a.node.depth_units_verified=True
    a.node.external_gate_receipts={'nav_ready':now,'control_armed':now}
    a.node.p=S(external_gate_timeout=2.,rgbd_wait_timeout=2.,tf_failure_timeout=3.)
    a.node.latest_snapshot=S(captured_monotonic=now-.1)
    a.node.last_camera_tf_success=now-.1
    a.node.rgbd_sync_status={'healthy':True}
    a.node.vlm_input_ready=lambda now:a.node.rgbd_sync_status['healthy']
    a.node.tf_buffer.lookup_transform=lambda *args:S(header=S(stamp=stamp(10)),transform=S(
        translation=S(x=0.,y=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
    for name in ('navigator','path_planner','spinner'):setattr(a.node,name,S(server_is_ready=lambda:True))
    a.motion_fault=False;a.motions={};a.path=[];a.path_at=0.;a.odom_at=now;a.stationary_since=now-1.
    a.on_costmap(costmap())
    return a


@pytest.mark.parametrize('fault',[
    'nav_ready_gate','control_armed_gate','external_gates_fresh','navigate_server','planner_server',
    'spin_server','motion_fault_clear','depth_units_verified','rgbd_ready','local_costmap','localization_valid',
])
def test_each_failed_gate_is_identified_without_relaxing_readiness(fault):
    from test_field_preparation import stamp
    a=healthy_adapter()
    if fault=='nav_ready_gate':a.node.nav_ready=False
    elif fault=='control_armed_gate':a.node.control_armed=False
    elif fault=='external_gates_fresh':a.node.external_gate_receipts['nav_ready']-=3.
    elif fault in ('navigate_server','planner_server','spin_server'):
        name={'navigate_server':'navigator','planner_server':'path_planner','spin_server':'spinner'}[fault]
        getattr(a.node,name).server_is_ready=lambda:False
    elif fault=='motion_fault_clear':a.motion_fault=True
    elif fault=='depth_units_verified':a.node.depth_units_verified=False
    elif fault=='rgbd_ready':a.node.rgbd_sync_status['healthy']=False
    elif fault=='local_costmap':a.costmap.header.stamp=stamp(8.)
    else:
        lookup=a.node.tf_buffer.lookup_transform
        def failed_pose(target,source,*args):
            if source=='base_link':raise RuntimeError('fixture missing pose TF')
            return lookup(target,source,*args)
        a.node.tf_buffer.lookup_transform=failed_pose
    health=a.get_health()
    details=a.last_health_details
    assert details['checks'][fault] is False
    assert health.localization_valid if fault!='localization_valid' else not health.localization_valid
    if fault!='localization_valid':assert not health.nav_ready
    if fault=='local_costmap':
        assert details['local_costmap']['reason']=='header_stale'
        assert details['local_costmap']['header_age_s']==2.
    json.dumps(details,allow_nan=False)


def test_healthy_diagnostics_match_actual_ready_snapshot():
    a=healthy_adapter();health=a.get_health()
    assert health.nav_ready and health.localization_valid and health.stopped
    assert all(a.last_health_details['checks'].values())
    assert a.last_health_details['local_costmap']['reason']=='ok'


def test_rejected_costmap_update_retains_why_payload_is_missing():
    from test_field_preparation import costmap,stamp
    a=healthy_adapter();grid=costmap();grid.header.stamp=stamp(8.)
    a.on_costmap(grid);health=a.get_health()
    assert not health.nav_ready
    details=a.last_health_details['local_costmap']
    assert details['reason']=='no_payload'
    assert details['last_update']['reason']=='header_stale'
    assert details['last_update']['header_age_s']==2.


def test_failure_event_records_current_state_before_stopping():
    from test_go2_exploration import LocalFixture,run_case
    from multi_agent_nav.go2_exploration import ExplorationStopped
    a=LocalFixture();a.online=False
    a.last_health_details={'checks':{'rgbd_ready':False}}
    events=[]
    with pytest.raises(ExplorationStopped,match='rgbd_ready'):run_case(a,events,resources={})
    failed=next(e for e in events if e['state']=='HEALTH_FAILED')
    assert failed['health']['nav_ready'] is False
    assert failed['health_details']['checks']['rgbd_ready'] is False
    assert not a.calls


def test_abc_request_has_one_wrapper_contract_and_strict_schema(monkeypatch):
    from multi_agent_nav.adapters.go2 import ABCClient
    from vlm_nav.vlm_client import OpenAICompatibleVLMClient
    from test_go2_adapter import grounding
    client=ABCClient.__new__(ABCClient)
    client.client=S(api_key='never-log');client.mode='abc';client._is_qwen=True
    client.image_detail='high';client.jpeg_quality=85
    client._structured_output_supported=False;client.model='stub'
    calls=[]
    payload={t:dict(status='DETECTED' if t=='A' else 'NOT_DETECTED',grounding=grounding(t=='A')) for t in 'ABC'}
    def completion(self,messages,max_tokens,schema=None):
        calls.append((messages,schema));return S(choices=[S(message=S(content=json.dumps(payload)))])
    monkeypatch.setattr(OpenAICompatibleVLMClient,'_completion',completion)
    result=client.infer_abc(np.zeros((480,640,3),dtype=np.uint8),dict(A='灭火器',B='水杯',C='篮子'))
    assert result['A'][0]=='DETECTED' and set(result)==set('ABC')
    messages,schema=calls[0];prompt=messages[0]['content'][0]['text']
    assert 'Return exactly one valid JSON object matching the requested fields' not in prompt
    assert 'normalized 1000x1000' in prompt
    assert 'logical AND' in prompt and 'solid visible material' in prompt
    assert set(schema['schema']['properties'])==set('ABC')
    assert schema['schema']['additionalProperties'] is False
    assert set(schema['schema']['properties']['A']['properties'])=={'status','grounding'}
    for target in 'ABC':assert target in json.loads(prompt.split('TARGET DESCRIPTIONS: ',1)[1].split('\n',1)[0])
