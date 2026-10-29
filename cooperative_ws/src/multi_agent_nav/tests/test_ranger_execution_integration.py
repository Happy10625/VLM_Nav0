"""Ranger transport keeps one result consumer and the original arrival contract."""
from collections import deque
from concurrent.futures import Future
from types import SimpleNamespace as S

from multi_agent_nav.adapters.ranger import RangerAdapter
from multi_agent_nav.models import Health, PathResult, Pose2D


def test_arrival_odom_comes_from_original_contract_not_adapter_switch_guard(tmp_path):
    import yaml
    from multi_agent_nav.adapters.ranger import observer_parameters
    file=tmp_path/'robot.yaml'
    file.write_text(yaml.safe_dump({'vlm_nav':{'ros__parameters':{'arrival_odom_topic':'/fastlio/odom'}}}))
    params=observer_parameters(dict(observer_config_file=str(file),odom_topic='/odom',costmap_topic='/costmap'))
    assert params['arrival_odom_topic']=='/fastlio/odom'


def test_reference_path_ignores_endpoint_budget_and_body_veto():
    adapter = RangerAdapter.__new__(RangerAdapter)
    adapter.get_health = lambda: Health(pose=Pose2D(0, 0, .4))
    adapter.costmap_transform = lambda **_: Pose2D()
    adapter.costmap_candidate_free = lambda *a, **k: False
    adapter.global_goal_checks = True
    path = Future()
    path.set_result(PathResult(True, [Pose2D(), Pose2D(3, 0)]))
    adapter.compute_path = lambda goal: path
    result = adapter.select_approach(Pose2D(4, 0), [], path_accept=lambda _: False).result()
    assert result.goal == Pose2D(4, 0, .4)


def test_local_motion_terminal_has_only_one_consumer():
    adapter = RangerAdapter.__new__(RangerAdapter)
    adapter.motions = {}; adapter.results = deque(); adapter.path = []
    adapter._motion_event = lambda *a, **k: None
    adapter._clear_path = lambda reason: None
    response, terminal = Future(), Future()
    observed = []
    handle = S(accepted=True, goal_id=S(uuid=bytes(16)), get_result_async=lambda: terminal)
    goal_id, reply = adapter._motion(S(send_goal_async=lambda _: response), object(),
        'task', 'command', 'PICKUP_NAV',
        on_accept=lambda h, g: observed.append(('accepted', g)),
        on_result=lambda status, g: observed.append((status, g)))
    response.set_result(handle)
    terminal.set_result(S(status=4))
    assert observed == [('accepted', goal_id), (4, goal_id)]
    assert reply.result() == 'SUCCEEDED'
    assert not adapter.results and not adapter.motions


def test_nav_ready_failure_logs_effective_gate_and_age_once_per_loss(monkeypatch):
    import json
    from multi_agent_nav.adapters.nav2 import Nav2Adapter
    adapter=RangerAdapter.__new__(RangerAdapter);messages=[]
    adapter.node=S(execution_active=True,get_logger=lambda:S(warning=messages.append))
    checks=dict(localization_valid=True,rgbd_ready=False,local_costmap=True,odom_fresh=True)
    adapter.last_health_details=dict(checks=checks,local_costmap=dict(reason='ok'),odom_age_s=.01)
    monkeypatch.setattr(Nav2Adapter,'get_health',lambda self:Health(nav_ready=all(
        v for k,v in checks.items() if k!='localization_valid')))
    assert adapter.get_health().nav_ready  # active local execution handles RGB-D waits itself
    assert not messages
    checks['local_costmap']=False
    adapter.last_health_details['local_costmap']=dict(reason='receipt_stale',receipt_age_s=1.1,max_age_s=1.)
    assert not adapter.get_health().nav_ready
    assert not adapter.get_health().nav_ready
    assert len(messages)==1
    evidence=json.loads(messages[0].split(': ',1)[1])
    assert evidence['failed_checks']==['local_costmap']
    assert evidence['health_details']['local_costmap']['receipt_age_s']==1.1
    assert evidence['execution_active'] is True
    checks['local_costmap']=True;assert adapter.get_health().nav_ready
    adapter.node.execution_active=False
    assert not adapter.get_health().nav_ready
    assert json.loads(messages[-1].split(': ',1)[1])['failed_checks']==['rgbd_ready']
    assert len(messages)==2
