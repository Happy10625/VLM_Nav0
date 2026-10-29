"""Executing-path lifetime and replanning; action transport is controlled locally."""
from collections import deque
from concurrent.futures import Future
from types import SimpleNamespace as S
import logging

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from multi_agent_nav.adapters.go2 import Go2Adapter
from multi_agent_nav.adapters.ranger import RangerAdapter
from multi_agent_nav.models import Pose2D


@pytest.fixture(params=[Go2Adapter, RangerAdapter], ids=['go2', 'ranger'])
def navigation(monkeypatch, request):
    now = [10.]
    monkeypatch.setattr('multi_agent_nav.adapters.nav2.time.monotonic', lambda: now[0])
    requests = []
    def send(goal):
        response = Future()
        requests.append(response)
        return response
    a = request.param.__new__(request.param)
    a.config = dict(path_receive_timeout=5., path_source_verified=False, footprint=[[0., 0.]],
                   odom_max_age=.5, stationary_duration=.5)
    a.node = S(get_clock=lambda: S(now=lambda: S(nanoseconds=round(now[0]*1e9))),
               get_logger=lambda: logging.getLogger('executing_path_test'),
               navigator=S(send_goal_async=send,server_is_ready=lambda:False),
               path_planner=S(server_is_ready=lambda:False),spinner=S(server_is_ready=lambda:False),
               external_gate_receipts={},p=S(external_gate_timeout=2.),
               make_pose_stamped=lambda x, y, yaw: PoseStamped(),
               tf_buffer=S(lookup_transform=lambda *args: S(
                   header=S(stamp=S(sec=10, nanosec=0)),
                   transform=S(translation=S(x=0., y=0.), rotation=S(x=0., y=0., z=0., w=1.)))),
               nav_ready=False, control_armed=False, depth_units_verified=False,
               vlm_input_ready=lambda t: False)
    a.costmap_transform = lambda: None
    a.motions = {}; a.results = deque(); a.motion_fault = False; a.scan = None
    a.path = []; a.path_at = 0.; a.odom_at = 0.; a.stationary_since = None
    return a, now, requests


def path(seconds=10., end=(2., 0.), frame='map'):
    message = Path()
    ns = round(seconds*1e9)
    message.header.frame_id = frame
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)
    for x, y in [(0., 0.), end]:
        pose = PoseStamped()
        pose.pose.position.x = float(x); pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.
        message.poses.append(pose)
    return message


def accept(request, accepted=True):
    terminal = Future()
    request.set_result(S(accepted=accepted, goal_id=S(uuid=[1]*16),
                         get_result_async=lambda: terminal,
                         cancel_goal_async=lambda: Future()))
    return terminal


def begin(navigation):
    a, now, requests = navigation
    goal = a.start_navigation(Pose2D(2., 0.), 'task', 'command', 'VIEWPOINT_NAV')
    terminal = accept(requests[-1])
    now[0] = 10.1
    return goal, terminal


def test_path_requires_an_accepted_navigation(navigation):
    a, now, requests = navigation
    a.on_path_update(path())
    assert not a.path and not a.get_health().path_valid
    a.start_navigation(Pose2D(2., 0.), 'task', 'command', 'VIEWPOINT_NAV')
    now[0] = 10.1
    a.on_path_update(path(10.1))
    assert not a.path and not a.get_health().path_valid
    accept(requests[-1])
    assert a.path == [Pose2D(), Pose2D(2., 0.)] and a.get_health().path_valid


def test_first_plan_before_acceptance_survives_the_first_moving_guard_check(navigation):
    from multi_agent_nav.region_guard import RegionGuard
    a, now, requests = navigation
    rows = []; a.record_event = rows.append
    goal = a.start_navigation(Pose2D(2., 0.), 'task', 'command', 'VIEWPOINT_NAV')
    now[0] = 10.1
    a.on_path_update(path(10.1))
    assert not a.get_health().path_valid
    now[0] = 10.102191
    accept(requests[-1])
    health = a.get_health()
    assert not health.stopped
    assert RegionGuard({}, .2).check(health.pose, health.footprint, health.path, {}, health).action == 'ALLOW'
    buffered = next(r for r in rows if r.get('event') == 'PATH_BUFFERED')
    received = next(r for r in rows if r.get('event') == 'PATH_RECEIVED')
    assert buffered['goal_id'] == received['goal_id'] == goal
    assert received['native_action_uuid'] == '01' * 16
    assert received['path_received_monotonic'] == 10.1


@pytest.mark.parametrize('outcome', ['cancel', 'reject'])
def test_pending_path_is_discarded_on_cancel_or_rejection(navigation, outcome):
    a, now, requests = navigation
    goal = a.start_navigation(Pose2D(2., 0.), 'task', 'command', 'VIEWPOINT_NAV')
    now[0] = 10.1
    a.on_path_update(path(10.1))
    if outcome == 'cancel':
        a.cancel_navigation(goal)
        accept(requests[-1])
    else:
        accept(requests[-1], accepted=False)
        now[0] = 11.
        a.start_navigation(Pose2D(3., 0.), 'task', 'next', 'VIEWPOINT_NAV')
        accept(requests[-1])
    assert not a.path and not a.get_health().path_valid


@pytest.mark.parametrize('message', [path(9.9), path(10.1, frame='odom'),
                                    path(10.1, (float('nan'), 0.)), Path()])
def test_invalid_pending_plan_cannot_be_promoted_on_acceptance(navigation, message):
    a, now, requests = navigation
    a.start_navigation(Pose2D(2., 0.), 'task', 'command', 'VIEWPOINT_NAV')
    now[0] = 10.1
    a.on_path_update(path(10.1))
    a.on_path_update(message)
    accept(requests[-1])
    assert not a.path and not a.get_health().path_valid


@pytest.mark.parametrize('accepted_at,valid', [(12., True), (16., False)])
def test_pending_path_keeps_original_receipt_time(navigation, accepted_at, valid):
    a, now, requests = navigation
    a.start_navigation(Pose2D(2., 0.), 'task', 'command', 'VIEWPOINT_NAV')
    now[0] = 10.1
    a.on_path_update(path(10.1))
    now[0] = accepted_at
    accept(requests[-1])
    assert a.get_health().path_valid is valid
    if valid:
        assert a.path_at == 10.1
        now[0] = 15.101
        assert not a.get_health().path_valid


def test_execution_path_timeout_logs_receipt_age_once(navigation,caplog):
    a,now,requests=navigation
    a.config['path_topic']='/plan'
    begin(navigation)
    a.on_path_update(path(10.1))
    assert a.get_health().path_valid
    now[0]=16.1
    assert not a.get_health().path_valid
    assert not a.get_health().path_valid
    messages=[r.getMessage() for r in caplog.records if 'Execution path receive timeout' in r.getMessage()]
    assert len(messages)==1
    assert 'topic=/plan' in messages[0] and 'receive_age_s=6.000' in messages[0]
    assert 'timeout_s=5.0' in messages[0]


def test_newest_valid_pending_plan_replaces_previous_candidate(navigation):
    a, now, requests = navigation
    a.start_navigation(Pose2D(2., 0.), 'task', 'command', 'VIEWPOINT_NAV')
    now[0] = 10.1
    a.on_path_update(path(10.1))
    now[0] = 10.2
    a.on_path_update(path(10.2, (2., 1.)))
    accept(requests[-1])
    assert a.path[-1] == Pose2D(2., 1.) and a.path_at == 10.2


def test_replanning_replaces_the_current_path_and_drops_old_goal_path(navigation):
    a, now, requests = navigation
    goal, terminal = begin(navigation)
    a.on_path_update(path(10.1))
    now[0] = 10.2
    a.on_path_update(path(10.2, (2., 1.)))
    assert a.path[-1] == Pose2D(2., 1.) and a.get_health().path_valid
    terminal.set_result(S(status=4))
    now[0] = 10.3
    a.start_navigation(Pose2D(3., 0.), 'task', 'next', 'VIEWPOINT_NAV')
    accept(requests[-1])
    a.on_path_update(path(10.2))
    assert not a.path
    a.on_path_update(path(10.3, (3., 0.)))
    assert a.path[-1] == Pose2D(3., 0.)


@pytest.mark.parametrize('status', [4, 5, 6])
def test_terminal_navigation_clears_path_and_rejects_late_updates(navigation, status):
    a, now, requests = navigation
    goal, terminal = begin(navigation)
    a.on_path_update(path(10.1))
    assert a.get_health().path_valid
    terminal.set_result(S(status=status))
    assert not a.path and not a.get_health().path_valid
    a.on_path_update(path(10.1))
    assert not a.path


def test_canceling_navigation_immediately_invalidates_path(navigation):
    a, now, requests = navigation
    goal, terminal = begin(navigation)
    a.on_path_update(path(10.1))
    a.cancel_navigation(goal)
    assert not a.path and not a.get_health().path_valid
    a.on_path_update(path(10.1))
    assert not a.path


def test_spin_cannot_make_an_execution_path_valid(navigation):
    a, now, requests = navigation
    a._motion(a.node.navigator, S(target_yaw=.5), 'task', 'scan', 'SCAN_SPIN')
    accept(requests[-1])
    a.on_path_update(path())
    assert not a.path and not a.get_health().path_valid


@pytest.mark.parametrize('message', [path(9.9), path(10.1, frame='odom'),
                                    path(10.1, (float('nan'), 0.)), Path()])
def test_invalid_path_cannot_replace_a_valid_execution_path(navigation, message):
    a, now, requests = navigation
    begin(navigation)
    a.on_path_update(path(10.1))
    assert a.get_health().path_valid
    a.on_path_update(message)
    assert not a.path and not a.get_health().path_valid


def test_old_plan_is_valid_until_monotonic_receive_timeout(navigation):
    a, now, requests = navigation
    begin(navigation)
    now[0] = 100.
    a.on_path_update(path(10.1))  # this task's plan, old generation stamp
    assert a.path and a.get_health().path_valid
    now[0] = 105.
    assert a.get_health().path_valid
    now[0] = 105.001
    assert not a.get_health().path_valid


def test_receive_timeout_ignores_ros_clock_and_cache_reads_do_not_renew(navigation):
    a, now, requests = navigation
    begin(navigation)
    a.on_path_update(path(10.1))
    a.node.get_clock = lambda: S(now=lambda: S(nanoseconds=10**15))
    now[0] = 14.
    assert a.get_health().path_valid
    assert a.path_at == 10.1
    now[0] = 15.101
    assert not a.get_health().path_valid


def test_identical_geometry_and_stamp_are_new_valid_receipts(navigation):
    a, now, requests = navigation
    begin(navigation)
    message = path(10.1)
    a.on_path_update(message)
    first = list(a.path)
    now[0] = 14.
    a.on_path_update(message)
    assert a.path == first and a.path_at == 14.
    now[0] = 18.9
    assert a.get_health().path_valid
    now[0] = 19.01
    assert not a.get_health().path_valid


def test_invalid_message_does_not_extend_valid_receipt_time(navigation):
    a, now, requests = navigation
    begin(navigation)
    a.on_path_update(path(10.1))
    now[0] = 14.
    a.on_path_update(path(10.2, (float('inf'), 0.)))
    assert not a.path and not a.get_health().path_valid
    assert a.path_at <= 10.1


def test_path_events_retain_header_and_current_goal_without_geometry_replan_claim(navigation):
    a, now, requests = navigation
    rows = []; a.record_event = rows.append
    goal, terminal = begin(navigation)
    a.on_path_update(path(10.1))
    now[0] = 12.
    a.on_path_update(path(10.2))
    received = [r for r in rows if r.get('event') == 'PATH_RECEIVED']
    assert len(received) == 2
    assert received[-1]['task_id'] == 'task' and received[-1]['goal_id'] == goal
    assert received[-1]['path_header_stamp_ns'] == 10_200_000_000
    assert received[-1]['receive_interval_s'] == pytest.approx(1.9)
    assert received[-1]['plan_stamp_changed'] is True
    terminal.set_result(S(status=5))
    assert any(r.get('event') == 'MOTION_TERMINAL' and r['status'] == 'CANCELED' for r in rows)
    assert not a.path and a.path_at == 0.


def test_evidence_write_error_cannot_prevent_cancel_and_zero_command(navigation):
    a, now, requests = navigation
    goal, terminal = begin(navigation)
    calls=[]
    a.node.stop_pub=S(publish=lambda msg:calls.append('zero'))
    a.vision=S(cancel=lambda:None);a.plans={}
    a.motions[goal]['handle'].cancel_goal_async=lambda:calls.append('cancel') or Future()
    a.on_path_update(path(10.1))
    def failed_record(event):raise OSError('unit full disk')
    a.record_event=failed_record
    a.stop_motion('test stop')
    assert 'cancel' in calls and 'zero' in calls
    assert not a.path
    assert a.evidence_write_failed
