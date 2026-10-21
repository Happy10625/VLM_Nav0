import uuid
from .models import *
from .planner import RulePlanner
from .resource_manager import ResourceManager

class Coordinator:
    def __init__(self, scene, alignment_id, map_ids, simulation, gates=None):
        self.scene, self.alignment_id, self.map_ids, self.simulation = scene, alignment_id, map_ids, simulation
        self.gates = list(gates or [])
        self.phase, self.task_id, self.reason = 'IDLE', '', ''
        self.states, self.received_at, self.observations = {}, {}, {}
        self.commands, self.reservations, self.rejected = {}, {}, set()
        self.sequence = {r:0 for r in ROBOTS}
        self.acked = {r:0 for r in ROBOTS}
        self.planner = RulePlanner()
        self.resources = ResourceManager()
        self.releases = {}
        self.started = 0.0
        self.c_observation = ''
        self.local_c = {}
        self.lost_timeout = scene.get('lost_timeout', 3.0)
        self.task_timeout = scene.get('task_timeout', 600.0)
        self.private = scene.get('share_c_coordinates',True) is False

    def _identity(self, data):
        return data.robot_id in ROBOTS and data.schema_version == SCHEMA_VERSION and (
            data.local_map_id, data.map_version) == self.map_ids[data.robot_id] and (
            data.common_frame_id, data.alignment_id) == (('','') if self.private else ('lab_ref', self.alignment_id))

    def ingest_state(self, state, received_at):
        if not self._identity(state):
            if self.task_id and state.robot_id in ROBOTS:self.pause('peer map/mode identity changed')
            return
        if self.private and (state.pose_valid or state.common_pose!=Pose2D()):return
        if state.simulation != self.simulation:
            if self.task_id:
                self.pause('simulation/live state mismatch')
            return
        if self.task_id and state.task_id not in ('', self.task_id):
            return
        previous = self.states.get(state.robot_id)
        if previous and previous.task_id == state.task_id and state.sequence < previous.sequence:
            return
        self.states[state.robot_id], self.received_at[state.robot_id] = state, received_at
        if self.resources:
            self._resource_feedback(state, received_at)

    def ingest_observation(self, observation, received_at):
        if not self._identity(observation) or observation.simulation != self.simulation or observation.task_id != self.task_id:
            return
        if observation.target_id not in TARGETS or observation.status not in ('DETECTED','NOT_DETECTED','UNCERTAIN','API_ERROR','CACHED'):
            return
        if self.private and (observation.position_valid or observation.common_position!=Pose2D() or
                observation.status=='CACHED' and observation.target_id!='C'):return
        if not self.private and observation.status=='CACHED':return
        previous=self.observations.get(observation.observation_id)
        recovered=self.private and observation.status=='CACHED' and previous and (
            previous.robot_id==observation.robot_id and previous.status!='CACHED')
        if observation.observation_id and (previous is None or recovered):
            self.observations[observation.observation_id] = observation
            if self.private and observation.status=='CACHED':
                self.local_c.setdefault(observation.robot_id,observation.observation_id)

    def start(self, scenario_id, now):
        if self.phase not in ('IDLE', 'PAUSED', 'DONE', 'FAILED'):
            return StartReply(False, reason='task already active')
        if scenario_id != self.scene['scenario_id'] or not self.scene['enabled'] or self.gates:
            return StartReply(False, reason='; '.join(self.gates) or 'scenario disabled/unknown')
        if self.resources and self.resources.held:
            return StartReply(False, reason='held resource requires field clearance; no automatic reset')
        for robot in ROBOTS:
            state = self.states.get(robot)
            if not state or now-self.received_at[robot] > self.lost_timeout or not (
                (self.private or state.pose_valid) and state.localization_valid and state.nav_ready and state.stopped) or state.token:
                return StartReply(False, reason=f'{robot}: fresh healthy stopped state required')
        self.task_id = uuid.uuid4().hex
        self.phase, self.started, self.reason = 'SEARCH_AB', now, ''
        self.observations, self.commands, self.reservations, self.rejected = {}, {}, {}, set()
        self.planner = RulePlanner()
        self.sequence = {r:0 for r in ROBOTS}
        self.acked = {r:0 for r in ROBOTS}
        self.c_observation = ''
        self.local_c = {}
        return StartReply(True, self.task_id)

    def control(self, robot, action, **kwargs):
        if self.private and action=='SYNC':kwargs['kind']='SYNC_LOCAL_C'
        return Command(task_id=self.task_id, recipient_id=robot, action=action, phase=self.phase,
                       common_frame_id='' if self.private else 'lab_ref',
                       alignment_id='' if self.private else self.alignment_id, simulation=self.simulation, **kwargs)

    def pause(self, reason):
        self.phase, self.reason = 'PAUSED', reason
        if self.resources:
            for robot in ROBOTS:
                self.resources.freeze(robot, reason)
                if robot in self.commands:self.resources.cancel_queued(robot,self.commands[robot].command_id)
        return [self.control(r, 'PAUSE') for r in ROBOTS] if self.task_id else []

    def _complete_commands(self):
        for robot, command in list(self.commands.items()):
            state = self.states[robot]
            if (state.task_id, state.command_id, state.sequence) != (self.task_id, command.command_id, command.sequence):
                continue
            if state.reply not in ('SUCCEEDED','FAILED','CANCELED') or not state.stopped:
                continue
            self.acked[robot] = command.sequence
            self.resources.cancel_queued(robot,command.command_id)
            if self.private and state.reply!='SUCCEEDED':
                self.pause(f'{robot}: local execution {state.reply.lower()}')
            if command.kind == 'SEARCH_VIEWPOINT':
                self.planner.visited.add((robot, command.phase, command.viewpoint_id))
                self.reservations.pop(command.viewpoint_id, None)
            elif state.reply == 'FAILED' and command.observation_id:
                self.rejected.add((robot, command.observation_id))
                if command.target_id == 'C':
                    self.pause(f'{robot}: bound C approach failed; explicit new task required')
            del self.commands[robot]

    def tick(self, now):
        if not self.task_id:
            return []
        if self.phase not in ('PAUSED','DONE','FAILED'):
            for robot in ROBOTS:
                state = self.states.get(robot)
                if not state or now-self.received_at[robot] > self.lost_timeout or not (
                    (self.private or state.pose_valid) and state.localization_valid and state.nav_ready) or state.run_state in ('STOPPING','PAUSED'):
                    self.pause(f'{robot}: health/heartbeat lost')
            if now-self.started > self.task_timeout:
                self.pause('task timeout')
        output = [self.control(r, 'SYNC', ack_sequence=self.acked[r]) for r in ROBOTS]
        if self.phase == 'PAUSED':
            return output + [self.control(r, 'PAUSE') for r in ROBOTS]
        if self.phase in ('DONE','FAILED'):
            return output
        if any(self.states[r].task_id != self.task_id for r in ROBOTS):
            if now-self.started > self.lost_timeout:
                return self.pause('task SYNC handshake timeout')
            return output
        self._complete_commands()
        if self.phase == 'PAUSED':
            return output + [self.control(r,'PAUSE') for r in ROBOTS]
        if self.phase == 'SEARCH_AB' and all(self.states[r].ready for r in ROBOTS) and not self.commands:
            self.phase = 'SEARCH_C'
        if self.phase == 'SEARCH_C':
            if self.private:
                if self.local_c:self.phase='DEPOSIT'
            else:
                if 'basket' not in self.scene['resources'] or any(r not in self.scene['exit_points'] for r in ROBOTS):
                    return output+self.pause('C region or exit geometry missing')
                clue=next((o for o in self.observations.values() if o.target_id=='C' and o.navigable),None)
                if clue and not self.commands:
                    self.c_observation=clue.observation_id
                    self.scene['bound_c']=clue.observation_id
                    self.phase='DEPOSIT'
        if self.phase=='DEPOSIT':
            if self.private:
                complete=not self.commands and all(self.states[r].deposited and self.states[r].stopped for r in ROBOTS)
            else:
                complete=all(self.states[r].deposited and self.states[r].exited and not self.states[r].token for r in ROBOTS) and not self.resources.held
            if complete:
                self.phase='DONE'
                return output
        if self.resources and not self.private:
            output += self._resource_commands(now)
        snapshot = Snapshot(self.phase, self.states, self.observations, self.reservations,
            self.resources, self.commands, self.scene, now, self.received_at, self.rejected, self.local_c)
        actions = self.planner.decide(snapshot)
        for action in actions:
            if action.robot_id in self.commands or not self.private and (action.goal is None or not action.goal.valid):
                continue
            robot = action.robot_id
            self.sequence[robot] += 1
            command = self.control(robot, 'EXECUTE', command_id=f'{self.task_id}/{robot}/{self.sequence[robot]}',
                sequence=self.sequence[robot], kind=action.kind, target_id=action.target_id,
                viewpoint_id=action.viewpoint_id, observation_id=action.observation_id,
                common_goal=Pose2D() if self.private else action.goal,
                object_position=Pose2D() if self.private else action.goal, position_valid=not self.private,
                resource_id=action.resource_id, token=action.token, parent_command_id=action.parent_command_id)
            self.commands[robot] = command
            if action.viewpoint_id:
                self.reservations[action.viewpoint_id] = robot
        if not self.commands and self.phase in ('SEARCH_AB','SEARCH_C') and not actions:
            return output+self.pause('this search round exhausted; target existence unknown')
        # Retry until a matching state acknowledges; duplicate execution is idempotent.
        output += [command for robot,command in self.commands.items() if
                   self.states[robot].command_id != command.command_id or self.states[robot].reply == '']
        return output

    def _resource_feedback(self, state, now):
        if not state.token or state.task_id!=self.task_id:
            return
        feedback=ResourceFeedback(state.task_id,state.robot_id,state.permit_command_id,
            state.resource_id,state.token,state.resource_status,state.localization_valid,
            state.resource_status in ('EXIT_CONFIRMED','CANCEL_OUTSIDE'),state.stopped)
        if self.resources.update(feedback,now) and feedback.status in ('EXIT_CONFIRMED','CANCEL_OUTSIDE'):
            self.releases[state.robot_id]=feedback

    def _resource_commands(self, now):
        output=[]
        for robot,state in self.states.items():
            release=self.releases.get(robot)
            if release:
                if state.token==release.token:
                    output.append(self.control(robot,'GRANT_RESOURCE',kind='RELEASE',
                        resource_id=release.resource_id,token=release.token,parent_command_id=release.parent_command_id))
                else:
                    del self.releases[robot]
            if not state.requested_resource or robot not in self.commands:
                continue
            cmd=self.commands[robot]
            if state.requested_command_id!=cmd.command_id or state.requested_resource not in self.scene['resources']:
                continue
            permit=self.resources.request(ResourceRequest(self.task_id,robot,cmd.command_id,state.requested_resource),now)
            if permit.granted:
                output.append(self.control(robot,'GRANT_RESOURCE',resource_id=permit.request.resource_id,
                    token=permit.token,parent_command_id=permit.request.parent_command_id))
        return output
