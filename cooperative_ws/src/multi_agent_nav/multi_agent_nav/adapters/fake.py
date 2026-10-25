from concurrent.futures import Future
from collections import deque
import uuid
from ..models import *

def completed(value):
    future = Future()
    future.set_result(value)
    return future

class FakeAdapter:
    simulation = True
    def __init__(self, robot_id, detections=None, delay=.2):
        self.robot_id, self.detections, self.delay = robot_id, detections or {}, delay
        self.pose = Pose2D()
        self.now = 0.0
        self.active = {}
        self.results = deque()
        self.unreachable = False
        self.api_error = False
        self.online = True
        self.aborts = 0
        self.calls = []
        self.path = []

    def get_health(self):
        return Health(self.pose, self.online, self.online, not self.active, self.path,
                      bool(self.path), [[.38,.18],[.38,-.18],[-.38,-.18],[-.38,.18]], not self.active)

    def compute_path(self, local_goal):
        return completed(PathResult(not self.unreachable, [self.pose, local_goal],
                                    'unreachable' if self.unreachable else ''))

    def read_approach_parameters(self):
        return completed(dict(xy_goal_tolerance=.05,planner_tolerance=.76))

    def costmap_transform(self,global_goal=False):
        return Pose2D() if self.online else None

    def costmap_candidate_free(self,candidate,global_goal=False,*,allow_outside=False):
        return self.online and candidate.valid

    observation_candidate_free=approach_candidate_free=costmap_candidate_free

    def select_approach(self, local_object, excluded_candidates, *, standoff_distance=None,path_accept=None):
        offset = standoff_distance if standoff_distance is not None else (.8 if self.robot_id == 'go2' else 1.0)
        candidates = ([Pose2D(local_object.x,local_object.y,self.pose.yaw)] if self.robot_id=='ranger' else
                      [Pose2D(local_object.x-offset, local_object.y, 0),
                       Pose2D(local_object.x, local_object.y-offset, 1.5707963267948966)])
        goal = next((p for p in candidates if p not in excluded_candidates and
            (path_accept is None or path_accept(PathResult(True,[self.pose,p])))), None)
        return completed(ApproachResult(goal if not self.unreachable else None, 'no approach candidate'))

    def start_navigation(self, local_goal, task_id, command_id, goal_kind):
        goal_id = uuid.uuid4().hex
        self.active[goal_id] = [self.now+self.delay, local_goal,
                               GoalResult(task_id, command_id, goal_id, goal_kind, 'SUCCEEDED'),self.now,self.pose]
        self.path = [self.pose, local_goal]
        self.calls.append((goal_kind, local_goal, command_id))
        return goal_id

    def cancel_navigation(self, goal_id):
        if goal_id in self.active:
            self.active[goal_id][2].status = 'CANCELED'
            self.active[goal_id][0] = self.now
        return completed(True)

    def stop_motion(self, reason):
        for goal_id in list(self.active):
            self.cancel_navigation(goal_id)
        return completed(True)

    def start_scan(self, target_descriptions, scan_id, *, stop_on_target=None, task_id=None, command_id=None, current_view_only=False):
        values = []
        for target in TARGETS:
            pose = self.detections.get(target)
            values.append(Observation(target_id=target, scan_id=scan_id,
                observation_id=f'{self.robot_id}/{scan_id}/{target}',
                status='API_ERROR' if self.api_error else ('DETECTED' if pose else 'NOT_DETECTED'),
                common_position=pose or Pose2D(), position_valid=bool(pose) and not self.api_error))
        return completed(values)

    def advance(self, now):
        self.now = now
        for goal_id, (due, pose, result,started,origin) in list(self.active.items()):
            if due > now:
                t=max(0,min(1,(now-started)/(due-started)))
                self.pose=Pose2D(origin.x+t*(pose.x-origin.x),origin.y+t*(pose.y-origin.y),
                                 origin.yaw+t*(pose.yaw-origin.yaw))
                continue
            if result.status == 'SUCCEEDED' and self.aborts:
                result.status = 'ABORTED'
                self.aborts -= 1
            if result.status == 'SUCCEEDED':
                self.pose = pose
            self.results.append(result)
            del self.active[goal_id]
