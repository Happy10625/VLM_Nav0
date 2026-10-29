"""Differential approach decisions against actual methods from the frozen Git object.

ROS transport is controlled here; no nodes, network, or hardware are started.
"""
import ast
import hashlib
from concurrent.futures import Future
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace as S

import pytest
import yaml

SHA = '32eab6244ec4b483807f8ed68e8667f5495874bb'
ROOT = Path(__file__).resolve().parents[4]
METHODS = ('navigation_tick', 'safety_tick', 'target_reference_distance',
    'approach_stop_radius', 'reset_approach_stop', 'request_approach_cancel',
    'on_approach_cancel_response', 'begin_approach_stop', 'advance_approach_stop',
    'finish_approach_success', 'on_arrival_odom', 'plan_then_navigate', 'goal_key',
    'request_next_plan', 'on_plan_response', 'on_plan_result', 'reject_current_plan',
    'send_navigation_goal', 'on_goal_response', 'on_navigation_result', 'elapsed_since')


def frozen(path):
    return (Path(__file__).parent / 'fixtures' / 'go2_frozen' / path).read_text()


@pytest.fixture(scope='module')
def reference_source():
    tree = ast.parse(frozen('vlm_nav/vlm_navigator.py'))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'VLMNavigator')
    node.bases = []; node.decorator_list = []
    node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    scope = dict(math=math, time=S(monotonic=lambda: 10.),
        GoalStatus=S(STATUS_SUCCEEDED=4, STATUS_CANCELED=5),
        ComputePathToPose=S(Goal=lambda: S()), NavigateToPose=S(Goal=lambda: S()),
        sample_polyline=lambda points, count: (points, None))
    for name in ('DISARMED API_ERROR SUCCEEDED FAILED SENSOR_WAITING TARGET_CONFIRMING '
                 'FRONTIER_SELECTING APPROACHING APPROACH_STOPPING TARGET_ALIGNING '
                 'TARGET_REOBSERVING SCANNING EXPLORING').split():scope[name] = name
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
                 SHA+':vlm_nav/vlm_navigator.py', 'exec'), scope)
    params = {}
    for path in ('config/robot.yaml', 'config/robot_go2.yaml'):
        params.update(yaml.safe_load(frozen(path))['vlm_nav']['ros__parameters'])
    return scope['VLMNavigator'], scope, params


@pytest.fixture
def reference(reference_source, monkeypatch):
    import multi_agent_nav.single_robot_approach as extracted
    monkeypatch.setattr(extracted, 'ComputePathToPose', S(Goal=lambda: S()))
    return reference_source


def done(value=None, error=None):
    f = Future()
    if error:f.set_exception(error)
    else:f.set_result(value)
    return f


class Harness:
    def __init__(self, cls, params, scope=None):
        self.now = 10.; self.pose = (0., 0., .7); self.events = []; self.plans = []; self.goals = []
        self.cancel = Future(); self.terminal = Future()
        a = self.a = cls.__new__(cls)
        a.p = S(**params); a.clock = lambda: self.now
        if scope is not None:scope['time'] = S(monotonic=lambda: self.now)
        a.state = 'APPROACHING'; a.target_reference_position = (2., 0.)
        a.goal_token = a.plan_token = 0
        a.goal_handle = a.plan_handle = None
        a.goal_pending = a.plan_pending = False
        a.goal_kind = a.goal_pose = a.active_motion_origin = None
        a.blocked_goals = {}; a.arrival_retry_count = 0; a.arrival_last_odom_received = 0.
        a.reset_approach_stop(); a.grid = a.map_message = object(); a.session_origin = (0., 0.)
        a.last_valid_rgbd_received = a.last_camera_tf_success = a.last_robot_tf_success = self.now
        a.get_parameter = lambda name: S(value=True)
        a.external_safety_gates_healthy = lambda: True
        a.robot_pose = lambda: self.pose
        a.easy_case_enabled = lambda: False
        a.publish_stop = lambda: self.events.append(('stop',))
        a.set_state = lambda state: (setattr(a, 'state', state), self.events.append(('state', state)))
        a.fail_safe = lambda reason: (setattr(a, 'state', 'FAILED'), self.events.append(('fail', reason)))
        a.get_logger = lambda: S(info=lambda *x: None, warn=lambda *x: None)
        a.make_pose_stamped = lambda x,y,yaw: (x,y,yaw)
        a.marker_pub = None
        a.publish_goal_marker = lambda *args: None
        a.emit = lambda *args, **kwargs: None
        a.path_planner = S(wait_for_server=lambda **kw: True, send_goal_async=self.plan)
        a.navigator = S(wait_for_server=lambda **kw: True, send_goal_async=self.navigate)
        a.adapter = S(start_approach_navigation=self.adapt_navigation)
        a.task_id = 'task'; a.command_id = 'command'; a.goal_id = ''; a.goal_kind_wire = 'PICKUP_NAV'

    def plan(self, goal):
        self.events.append(('plan', goal.goal, goal.use_start))
        f = Future(); self.plans.append(f); return f

    def planned(self, status=4, points=((0.,0.),(1.5,0.)), accepted=True):
        result = Future()
        self.plans[-1].set_result(S(accepted=accepted, get_result_async=lambda:result,
                                    cancel_goal_async=lambda:done()))
        path = [S(pose=S(position=S(x=x,y=y))) for x,y in points]
        result.set_result(S(status=status, result=S(path=S(poses=path))))

    def navigate(self, goal):
        self.events.append(('navigate', goal.pose)); f = Future(); self.goals.append(f); return f

    def adapt_navigation(self, owner, pose, token):
        f = self.navigate(S(pose=(pose.x,pose.y,pose.yaw)))
        def accepted(response):
            owner.on_goal_response(response,token,'approach')
            if response.result().accepted:
                response.result().get_result_async().add_done_callback(
                    lambda result:owner.on_navigation_result(result,token,'approach'))
        f.add_done_callback(accepted)
        return 'goal'

    def accept(self):
        self.goals[-1].set_result(S(accepted=True, get_result_async=lambda:self.terminal,
            cancel_goal_async=lambda:(self.events.append(('cancel',)) or self.cancel)))

    def odom(self, x=0., y=0., z=0., angular_z=0.):
        self.a.on_arrival_odom(S(twist=S(twist=S(linear=S(x=x,y=y,z=z),
            angular=S(x=10.,y=10.,z=angular_z)))))


def pair(reference):
    from multi_agent_nav.single_robot_approach import SingleRobotApproach
    cls, scope, params = reference
    return Harness(cls, params, scope), Harness(SingleRobotApproach, params)


def compare(reference, scenario):
    old, new = pair(reference)
    for h in (old,new):scenario(h)
    assert new.events == old.events
    assert new.a.state == old.a.state
    assert new.a.arrival_retry_count == old.a.arrival_retry_count
    return new


@pytest.mark.parametrize('status,points,accepted', [
    (4,((0.,0.),(1.5,0.)),True), (4,((1.5,0.),),True),
    (6,((0.,0.),),True), (4,(),True), (4,(),False)])
def test_plan_uses_original_request_or_original_failure(reference,status,points,accepted):
    def scenario(h):
        h.a.navigation_tick(); h.planned(status,points,accepted)
    h = compare(reference,scenario)
    if accepted and status == 4 and points:
        assert ('navigate',(2.,0.,.7)) in h.events
    else:assert h.a.state == 'FAILED'


@pytest.mark.parametrize('distance', [.889999,.89,.890001])
def test_initial_distance_boundary(reference,distance):
    def scenario(h):
        h.a.target_reference_position=(0.,0.); h.pose=(distance,0.,.7); h.a.navigation_tick()
    compare(reference,scenario)


@pytest.mark.parametrize('distance', [.809999,.81,.810001])
@pytest.mark.parametrize('status', [4,5,6])
def test_controlled_stop_terminal_and_distance(reference,distance,status):
    def scenario(h):
        h.a.navigation_tick(); h.planned(); h.accept()
        h.pose=(2.-.88,0.,.7); h.a.safety_tick()
        h.cancel.set_result(S(goals_canceling=[1]))
        for _ in range(3):h.odom()
        h.a.advance_approach_stop(); assert h.a.state == 'APPROACH_STOPPING'
        h.terminal.set_result(S(status=status)); h.pose=(2.-distance,0.,.7)
        h.a.advance_approach_stop()
    compare(reference,scenario)


@pytest.mark.parametrize('kind', ['moving_sample','stale','cancel_timeout','stop_timeout','double_retry','succeeded','aborted'])
def test_stop_sequences(reference,kind):
    def scenario(h):
        a=h.a; a.navigation_tick(); h.planned(); h.accept()
        if kind in ('succeeded','aborted'):
            h.terminal.set_result(S(status=4 if kind=='succeeded' else 6))
            if kind=='aborted':return
            assert a.state == 'APPROACH_STOPPING'
        else:
            h.pose=(1.12,0.,.7); a.safety_tick()
            h.cancel.set_result(None)
            if kind=='cancel_timeout':
                h.now+=3.; a.advance_approach_stop(); assert a.state=='APPROACH_STOPPING'
                h.now+=.001; a.advance_approach_stop(); return
            h.terminal.set_result(S(status=5))
        h.odom();h.odom()
        if kind=='moving_sample':h.odom(y=.04)
        h.odom(z=100.)
        if kind=='stale':h.now+=.501
        if kind=='stop_timeout':h.odom(x=.1);h.now+=5.001
        h.pose=(1.2,0.,.7); a.advance_approach_stop()
        if kind=='double_retry':
            # Re-enter original stopping with a terminal action twice outside 0.81.
            a.state='APPROACHING'; a.arrival_retry_count=0
            for _ in range(2):
                a.begin_approach_stop('nav2_action_succeeded',action_terminal=True)
                h.pose=(1.18,0.,.7)
                for _ in range(3):h.odom()
                a.advance_approach_stop()
    compare(reference,scenario)


@pytest.mark.parametrize('kind', ['timeout','distance_first','terminal_clear'])
def test_watchdogs_use_parameters_and_source_order(reference,kind):
    def scenario(h):
        a=h.a; a.p.max_travel_radius=7.; a.p.goal_timeout=20.
        a.navigation_tick();h.planned();h.accept()
        assert a.active_motion_origin == (0.,0.)
        if kind=='timeout':h.now=30.001
        if kind=='distance_first':h.pose=(1.2,0.,0.);h.now=30.001
        if kind=='terminal_clear':
            h.terminal.set_result(S(status=4));assert a.active_motion_origin is None
        a.last_valid_rgbd_received = a.last_camera_tf_success = a.last_robot_tf_success = h.now
        a.safety_tick()
    compare(reference,scenario)


@pytest.mark.parametrize('accepted', [False, True])
def test_long_approach_has_no_radius_stop_but_keeps_timeout(reference, accepted):
    _, h = pair(reference)
    a = h.a
    a.target_reference_position = (-3.671206870993619, -3.8647415258566866)
    origin = (1.8000003561377511, -4.999999713897704)
    h.pose = (*origin, 3.03)
    a.navigation_tick()
    h.planned(points=(origin, (-3.8812068627979803, -4.02474151691599)))
    if accepted:
        h.accept()
    h.pose = (-1.7003669332039335, -4.795960352276219, 3.12)
    assert math.dist(h.pose[:2], origin) > a.p.max_travel_radius + .5
    before = list(h.events)
    a.safety_tick()
    assert a.state == 'APPROACHING' and h.events == before
    assert a.goal_handle is not None if accepted else a.goal_pending

    h.now += a.p.goal_timeout + .001
    a.safety_tick()
    assert a.state == 'FAILED'
    assert h.events[-1] == ('fail', 'Navigation or path-validation goal timed out')



def test_ranger_parameters_use_original_file():
    from multi_agent_nav.single_robot_approach import load_parameters
    p=load_parameters([ROOT/'config/robot.yaml'], robot_id='ranger')
    original=yaml.safe_load((ROOT/'config/robot.yaml').read_text())['vlm_nav']['ros__parameters']
    assert vars(p)=={key:original[key] for key in vars(p)}
    assert p.arrival_odom_topic=='/fastlio/odom'


def test_executor_matches_fixed_source_except_authorized_safety_tick():
    import multi_agent_nav.single_robot_approach as module
    tree=ast.parse(Path(module.__file__).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SingleRobotApproach')
    # The radius-watchdog removal is covered by behavior tests; freeze the remaining source methods.
    cls.body=[n for n in cls.body if not isinstance(n,ast.FunctionDef) or n.name!='safety_tick']
    assert hashlib.sha256(ast.dump(cls).encode()).hexdigest()=='8a0fefbc8f8c89c969a9c081a02893bb98f245b27b0e8e5f949b427f9fb5f04c'

