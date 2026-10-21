from dataclasses import replace
from .models import *

NAV_KIND = {'SEARCH_VIEWPOINT':'VIEWPOINT_NAV', 'APPROACH_PICKUP':'PICKUP_NAV',
            'WAIT':'WAIT_NAV', 'APPROACH_DEPOSIT':'DEPOSIT_NAV', 'EXIT':'EXIT_NAV'}

class AgentMachine:
    def __init__(self, context, frame, lost_timeout=3.0, path_start_timeout=1.0):
        import math
        if not math.isfinite(path_start_timeout) or path_start_timeout <= 0:
            raise ValueError('path_start_timeout must be finite and positive')
        self.context, self.frame, self.lost_timeout = context, frame, lost_timeout
        self.path_start_timeout = path_start_timeout
        self.retired_tasks = set()
        self.navigation_started_at = 0.0
        self.path_acquired = False
        self._state = AgentState(robot_id=context.robot_id, task_id=context.task_id,
            local_map_id=context.local_map_id, map_version=context.map_version,
            common_frame_id=context.common_frame_id, alignment_id=context.alignment_id,
            simulation=context.simulation)
        self.command = None
        self.effects = []
        self.goal_kind = ''
        self.sync_received = None
        self.phase = 'IDLE'
        self.peer_received = None
        self.last_sequence = 0
        self.paused = False
        self.retry = 0
        self.excluded = []
        self.selected_goal = None
        self.health = Health()
        self.guard = None
        self.transit_goal = None
        self.private = context.common_frame_id == context.alignment_id == ''
        self.local_observations = {}
        self.task_started_at = 0.

    def state(self):
        return replace(self._state)

    def take_effects(self):
        effects, self.effects = self.effects, []
        return effects

    def receive_peer(self,state,entry,now):
        expected='ranger' if self.context.robot_id=='go2' else 'go2'
        if state.robot_id!=expected or state.task_id!=self.context.task_id or state.simulation!=self.context.simulation:
            return False
        identity=(state.local_map_id,state.map_version,state.common_frame_id,state.alignment_id)
        if state.schema_version!=SCHEMA_VERSION or identity!=(entry['local_map_id'],entry['map_version'],
                self.context.common_frame_id,self.context.alignment_id):
            self.stop('peer map/mode identity changed');return False
        if self.private and (state.pose_valid or state.common_pose!=Pose2D()):
            self.stop('private peer published coordinates');return False
        if not state.localization_valid or not state.nav_ready or not self.private and not state.pose_valid:
            self.stop('peer localization/navigation invalid');return False
        self.peer_received=now
        return True

    def _plan(self):
        if self.private:
            self.effects.append(Effect('LOCAL_EXECUTE'))
            return
        if self.command.kind.startswith('APPROACH'):
            self.effects.append(Effect('APPROACH', (self.frame.to_local(self.command.object_position), list(self.excluded))))
        else:
            self.selected_goal = self.frame.to_local(self.command.common_goal)
            self.effects.append(Effect('PLAN', self.selected_goal))

    def accept(self, command, now):
        command=normalize_command(command)
        # SYNC alone establishes a task; EXECUTE can never silently replace it.
        if command.action == 'SYNC' and command.task_id != self.context.task_id:
            if command.task_id in self.retired_tasks:
                return CommandReply(False, 'retired task')
            if self._state.token or self._state.run_state not in ('IDLE', 'PAUSED') or not self._state.stopped or not self.health.motion_terminal:
                return CommandReply(False, 'not stopped for new task')
            candidate = replace(self.context, task_id=command.task_id)
            reply = validate_command(command, candidate)
            if not reply.accepted:
                return reply
            if self.context.task_id:
                self.retired_tasks.add(self.context.task_id)
            self.context = candidate
            self.task_started_at=now
            self.peer_received=None
            self._state.task_id = command.task_id
            self._state.ready = self._state.deposited = self._state.exited = False
            self._state.sequence=0
            self._state.command_id=self._state.goal_id=''
            self._state.requested_resource=self._state.requested_command_id=''
            self._state.run_state,self._state.reason='IDLE',''
            self._state.reply = ''
            self.command = None
            self.local_observations.clear()
            self.effects.clear()
            self.selected_goal=self.transit_goal=None
            self.last_sequence = 0
            self.paused = False
        reply = validate_command(command, self.context)
        if not reply.accepted:
            return reply
        if command.action == 'SYNC':
            self.sync_received = now
            self.phase = command.phase
            return reply
        if command.action in ('PAUSE', 'CANCEL'):
            if command.action == 'CANCEL' and (not self.command or command.cancel_command_id != self.command.command_id):
                return CommandReply(False, 'cancel command mismatch')
            self.stop('coordinator pause/cancel')
            return reply
        if command.action == 'GRANT_RESOURCE':
            if command.kind == 'RELEASE':
                if not command.token or (command.resource_id,command.token,command.parent_command_id)!=(
                        self._state.resource_id,self._state.token,self._state.permit_command_id):
                    return CommandReply(False,'release permit identity mismatch')
                self._state.token = self._state.resource_id = self._state.permit_command_id = self._state.resource_status = ''
                return reply
            if self._state.token and (command.resource_id,command.token,command.parent_command_id)!=(
                    self._state.resource_id,self._state.token,self._state.permit_command_id):
                return CommandReply(False,'cannot replace held permit')
            owner = self.command.parent_command_id if self.command and self.command.kind == 'EXIT' else (self.command.command_id if self.command else '')
            if not command.token or command.parent_command_id != owner or command.resource_id != self._state.requested_resource:
                # Repeated grants for an already-held token are harmless.
                return CommandReply(bool(command.token) and (command.resource_id,command.token,command.parent_command_id)==(
                    self._state.resource_id,self._state.token,self._state.permit_command_id), 'permit mismatch')
            self._state.resource_id, self._state.token = command.resource_id, command.token
            self._state.permit_command_id = owner
            self._state.resource_status = 'RESERVED'
            self._state.requested_resource = ''
            if self.selected_goal and not (self.private and self.command.kind!='EXIT'):
                self.effects.append(Effect('PLAN', self.selected_goal))
            return reply
        if command.sequence <= self.last_sequence:
            return CommandReply(command.command_id == self._state.command_id, 'duplicate/stale command')
        if command.phase != self.phase:
            return CommandReply(False, 'phase must be synchronized before execution')
        if command.kind == 'APPROACH_PICKUP' and (self.phase != 'SEARCH_AB' or self._state.ready):
            return CommandReply(False, 'pickup phase/READY mismatch')
        if self.private and command.kind=='SEARCH_VIEWPOINT' and (self.phase not in ('SEARCH_C','DEPOSIT') or not self._state.ready):
            return CommandReply(False,'C search requires READY and C phase')
        if self.private and command.kind=='APPROACH_DEPOSIT':
            observation=self.local_observations.get(command.observation_id)
            if (not observation or observation.target_id!='C' or not observation.navigable or
                    (observation.task_id,observation.robot_id,observation.local_map_id,observation.map_version)!=
                    (self.context.task_id,self.context.robot_id,self.context.local_map_id,self.context.map_version)):
                return CommandReply(False,'local C observation missing/invalid')
        if command.kind in ('APPROACH_DEPOSIT','EXIT') and (self.phase != 'DEPOSIT' or not self._state.ready):
            return CommandReply(False, 'deposit requires READY and deposit phase')
        if self.paused or self._state.run_state == 'STOPPING':
            return CommandReply(False, 'paused: new task required')
        if self.command and self._state.reply not in ('SUCCEEDED', 'FAILED', 'CANCELED'):
            return CommandReply(False, 'one active command')
        if not self._state.stopped or not self.health.motion_terminal:
            return CommandReply(False, 'previous motion not stopped')
        if command.kind == 'EXIT':
            parent=self._state.permit_command_id
            if (not self._state.deposited or command.token != self._state.token or
                    command.parent_command_id != parent or command.resource_id != self._state.resource_id):
                return CommandReply(False, 'EXIT must preserve deposit identity')
        self.command, self.last_sequence = command, command.sequence
        self.retry, self.excluded = 0, []
        self._state.command_id, self._state.sequence = command.command_id, command.sequence
        self._state.reply, self._state.reason = 'ACCEPTED', ''
        self._state.run_state = 'PLANNING'
        self._plan()
        return reply

    def on_approach(self, approach, now):
        if approach.goal is None:
            self.fail(approach.reason or 'no reachable approach')
            return
        self.selected_goal = approach.goal
        self.effects.append(Effect('PLAN', approach.goal))

    def on_path(self, result, goal, now):
        if self.paused or not self.command:
            return
        if not result.reachable:
            self.fail(result.reason or 'unreachable')
            return
        self.selected_goal = goal
        self.health.path = result.path
        self.health.path_valid = True
        if self.guard:
            check_path = result.path
            if self._state.resource_id in self.guard.regions and self._state.resource_id != 'basket':
                self.transit_goal = self.guard.transit_exit(self._state.resource_id, self.context.robot_id, result.path)
                if self.transit_goal is None:
                    self.stop('missing door transit exit')
                    return
                # Precompute transit path through adapter, not a straight-line substitute.
                if goal != self.transit_goal:
                    self.effects.append(Effect('PLAN_TRANSIT', self.transit_goal))
                    return
            decision = self.guard.check(self.health.pose, self.health.footprint, result.path,
                                       {self._state.resource_id: self._state.token} if self._state.token else {}, self.health)
            if decision.action == 'STOP':
                self.stop(decision.reason)
                return
            if decision.resource_id:
                self._state.requested_resource = decision.resource_id
                self._state.requested_command_id = self.command.command_id
                self._state.run_state = 'WAITING_RESOURCE'
                return
        if not self.private and self.command.kind == 'APPROACH_DEPOSIT' and not self.transit_goal and self._state.resource_id != 'basket':
            self._state.requested_resource = 'basket'
            self._state.requested_command_id = self.command.command_id
            self._state.run_state = 'WAITING_RESOURCE'
            return
        nav_goal = self.transit_goal or goal
        kind = 'DOOR_TRANSIT' if self.transit_goal else NAV_KIND[self.command.kind]
        self.effects.append(Effect('NAVIGATE', (nav_goal, kind)))

    def navigation_started(self, goal_id, goal_kind, now=0.0):
        self.navigation_started_at, self.path_acquired = now, False
        self._state.goal_id, self.goal_kind = goal_id, goal_kind
        self._state.run_state, self._state.reply = 'NAVIGATING', 'EXECUTING'
        self._state.stopped = False

    def on_goal_result(self, result, now):
        if self.paused or not self.command or (result.task_id, result.command_id, result.goal_id, result.goal_kind) != (
            self.context.task_id, self.command.command_id, self._state.goal_id, self.goal_kind):
            return []
        self._state.goal_id = ''
        # Terminal is not physical stop; tick/adapter must confirm it before replacement.
        if result.status != 'SUCCEEDED':
            if self.retry < 1 and result.status == 'ABORTED':
                self.retry += 1
                self.excluded.append(self.selected_goal)
                self._state.run_state = 'RETRY_STOP'
            else:
                self.fail(result.reason or result.status)
            return []
        if self.goal_kind == 'DOOR_TRANSIT':
            self._state.run_state = 'TRANSIT_STOP'
        elif self.command.kind == 'SEARCH_VIEWPOINT':
            self._state.run_state = 'SCAN_STOP'
        else:
            if self.goal_kind == 'PICKUP_NAV' and self.command.kind == 'APPROACH_PICKUP':
                self._state.ready = True
            elif self.goal_kind == 'DEPOSIT_NAV' and self.command.kind == 'APPROACH_DEPOSIT':
                self._state.deposited = True
            elif self.goal_kind == 'EXIT_NAV' and self.command.kind == 'EXIT':
                self._state.run_state = 'EXIT_STOP'
                return []
            self._state.reply, self._state.run_state = 'SUCCEEDED', 'IDLE'
        return []

    def on_observation(self, observation, now):
        if not self.paused and self.command and observation.scan_id == self.command.command_id:
            observation=replace(observation,task_id=self.context.task_id,robot_id=self.context.robot_id,
                local_map_id=self.context.local_map_id,map_version=self.context.map_version,
                common_frame_id=self.context.common_frame_id,alignment_id=self.context.alignment_id,
                viewpoint_id=self.command.viewpoint_id,simulation=self.context.simulation)
            if self.private:
                if observation.navigable and observation.observation_id:
                    self.local_observations[observation.observation_id]=observation
                observation=replace(observation,status='CACHED' if observation.target_id=='C' and observation.navigable
                    else observation.status,common_position=Pose2D(),position_valid=False)
            self.effects.append(Effect('OBSERVATION',observation))
        return []

    def scan_completed(self, observations, now):
        if self.paused or not self.command:
            return
        for observation in observations:
            self.on_observation(observation, now)
        if {o.target_id for o in observations} != set(TARGETS) or any(o.status == 'API_ERROR' for o in observations):
            if self.retry < 1:
                self.retry += 1
                self.effects.append(Effect('SCAN', self.command.command_id))
            else:
                self.fail('scan failed after retry')
        else:
            self._state.reply, self._state.run_state = 'SUCCEEDED', 'IDLE'

    def fail(self, reason):
        self._state.reply, self._state.reason, self._state.run_state = 'FAILED', reason, 'IDLE'

    def stop(self, reason):
        if self._state.run_state == 'STOPPING' or self.paused:
            return
        self.paused = True
        self.effects.append(Effect('STOP', (self._state.goal_id, reason)))
        self._state.goal_id = ''
        self._state.run_state, self._state.reply, self._state.reason = 'STOPPING', 'CANCELED', reason

    def tick(self, now, health):
        self.health = health
        self._state.localization_valid = health.localization_valid and health.pose.valid
        self._state.pose_valid = self._state.localization_valid and not self.private
        self._state.nav_ready, self._state.stopped = health.nav_ready, health.stopped
        if self._state.pose_valid:
            self._state.common_pose = self.frame.to_common(health.pose)
        else:self._state.common_pose=Pose2D()
        if self.context.task_id and not self.paused and (
            self.sync_received is None or now-self.sync_received > self.lost_timeout or
            (self.peer_received is None and now-self.task_started_at>self.lost_timeout) or
            (self.peer_received is not None and now-self.peer_received > self.lost_timeout) or
            not health.localization_valid or not health.nav_ready):
            self.stop('SYNC/peer/localization/navigation health lost')
        if self._state.run_state == 'STOPPING':
            if health.stopped and health.motion_terminal:
                self._state.run_state = 'PAUSED'
            return self.take_effects()
        if self.guard and self.command:
            self._monitor_region(health, now)
        if health.stopped and health.motion_terminal:
            if self._state.run_state == 'RETRY_STOP':
                self._state.run_state = 'PLANNING'
                self._plan()
            elif self._state.run_state == 'SCAN_STOP':
                self._state.run_state = 'SCANNING'
                self.effects.append(Effect('SCAN', self.command.command_id))
            elif self._state.run_state == 'TRANSIT_STOP' and not self._state.token and not self.private:
                self.transit_goal = None
                self._plan()
            elif self._state.run_state == 'EXIT_STOP' and not self._state.token:
                self._state.exited = True
                self._state.run_state, self._state.reply = 'IDLE', 'SUCCEEDED'
        return self.take_effects()

    def _monitor_region(self, health, now=0.0):
        from .region_guard import footprint_intersects
        import math
        if len(health.footprint)<3 or not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in health.footprint):
            self.stop('invalid footprint monitor')
            return
        if self._state.token and health.localization_valid:
            polygon=self.guard.regions[self._state.resource_id]['polygon']
            intersects=footprint_intersects(health.pose,health.footprint,polygon)
            if intersects:
                self._state.resource_status='ENTERED'
            elif self._state.resource_status=='ENTERED' and health.stopped and health.motion_terminal and self._state.run_state in ('EXIT_STOP','TRANSIT_STOP'):
                self._state.resource_status='EXIT_CONFIRMED'
        if self._state.run_state=='NAVIGATING':
            if health.path_valid and health.path:
                self.path_acquired = True
            elif not self.path_acquired and health.stopped and now-self.navigation_started_at < self.path_start_timeout:
                # Only inspect the stationary stopping envelope while awaiting the
                # first executing path. Never substitute the preflight path.
                decision=self.guard.check(health.pose,health.footprint,[health.pose],
                    {self._state.resource_id:self._state.token} if self._state.token else {},
                    replace(health,path_valid=True))
                if decision.action!='ALLOW':
                    self.stop('unsafe stationary envelope while awaiting executing path')
                return
            decision=self.guard.check(health.pose,health.footprint,health.path,
                {self._state.resource_id:self._state.token} if self._state.token else {},health)
            if decision.action!='ALLOW':
                # A replanned path requires cancellation and physical stop before new permission.
                self.stop('replanned path needs permit or boundary monitor failed')
