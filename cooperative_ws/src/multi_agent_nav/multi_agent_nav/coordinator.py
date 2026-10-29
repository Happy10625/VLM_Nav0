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
        self.transport_received_at, self.task_received_at = {}, {}
        self.state_errors = {}
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
        self.nav_pause_phase = None

    def _identity(self, data):
        return data.robot_id in ROBOTS and data.schema_version == SCHEMA_VERSION and (
            data.local_map_id, data.map_version) == self.map_ids[data.robot_id] and (
            data.common_frame_id, data.alignment_id) == (('','') if self.private else ('lab_ref', self.alignment_id))

    def ingest_state(self, state, received_at):
        if state.robot_id not in ROBOTS:return
        error=''
        if not self._identity(state):error='peer map/mode identity changed'
        elif self.private and (state.pose_valid or state.common_pose!=Pose2D()):error='private peer published coordinates'
        elif state.simulation!=self.simulation:error='simulation/live state mismatch'
        if error:
            self.state_errors[state.robot_id]=error
            if self.private and (not self.task_id or self.phase=='SYNCING'):
                self.states.pop(state.robot_id,None)
            elif self.task_id:self.pause(error)
            return
        self.state_errors.pop(state.robot_id,None)
        self.transport_received_at[state.robot_id]=received_at
        if self.task_id and state.task_id not in ('', self.task_id):
            return
        if self.private and self.task_id and self.phase not in ('SYNCING','PAUSED','DONE','FAILED') and not state.task_id:
            self.pause(state.robot_id+': peer restarted during task')
            return
        previous = self.states.get(state.robot_id)
        if previous and previous.task_id == state.task_id and state.sequence < previous.sequence:
            return
        if self.task_id and state.task_id==self.task_id:self.task_received_at[state.robot_id]=received_at
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
                return StartReply(False, reason=f'{robot}: '+self.state_errors.get(robot,'fresh healthy stopped state required'))
            if self.private and not (startup_state(state) or state.run_state=='PAUSED'):
                return StartReply(False,reason=f'{robot}: waiting for local/peer readiness')
        self.task_received_at.clear()
        self.task_id = uuid.uuid4().hex
        self.phase, self.started, self.reason = 'SYNCING' if self.private else 'SEARCH_AB', now, ''
        self.nav_pause_phase=None
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

    def pause(self, reason, *, nav_ready=False):
        if self.phase=='PAUSED':
            if self.nav_pause_phase and not nav_ready:
                self.nav_pause_phase=None
                self.reason=reason
            return self._pause_commands() if self.task_id else []
        self.nav_pause_phase=self.phase if self.private and nav_ready else None
        self.phase, self.reason = 'PAUSED', reason
        if self.resources:
            for robot in ROBOTS:
                self.resources.freeze(robot, reason)
                if robot in self.commands:self.resources.cancel_queued(robot,self.commands[robot].command_id)
        return self._pause_commands() if self.task_id else []

    def _pause_commands(self):
        return [self.control(r,'PAUSE',kind='NAV_READY_WAIT' if self.nav_pause_phase else 'WAIT') for r in ROBOTS]

    def _nav_recovery(self, now):
        waiting=[]
        for robot in ROBOTS:
            state=self.states.get(robot)
            if not state or now-self.received_at[robot]>self.lost_timeout:
                waiting.append(robot)
                continue
            if (state.task_id!=self.task_id or not state.localization_valid
                    or state.run_state in ('STOPPING','PAUSED') and not is_recoverable_pause(state.reason)):
                return self.pause(f'{robot}: fault during nav_ready recovery')
        if now-self.started>self.task_timeout:return self.pause('task timeout')
        if waiting:
            self.phase='PAUSED'
            self.reason='waiting heartbeat: '+', '.join(waiting)
        elif not all(s.nav_ready for s in self.states.values()):
            self.phase='PAUSED'
            self.reason='; '.join(robot+': health invalid: nav_ready'+
                (NAV_READY_DETAIL+state.reason.partition(NAV_READY_DETAIL)[2] if NAV_READY_DETAIL in state.reason else '')
                for robot,state in self.states.items() if not state.nav_ready)
        elif self.phase=='PAUSED' and all(s.stopped and s.run_state=='PAUSED' for s in self.states.values()):
            self.phase,self.reason=self.nav_pause_phase,''
        elif self.phase!='PAUSED' and all(s.run_state not in ('STOPPING','PAUSED') for s in self.states.values()):
            self.nav_pause_phase=None
        output=[self.control(r,'SYNC',ack_sequence=self.acked[r]) for r in ROBOTS]
        return output+self._pause_commands() if self.phase=='PAUSED' else output

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
        if self.nav_pause_phase:return self._nav_recovery(now)
        if self.phase not in ('PAUSED','DONE','FAILED') and now-self.started>self.task_timeout:
            return self.pause('task timeout')
        if self.phase=='SYNCING':
            pending=[robot for robot in ROBOTS if not (
                robot in self.states and now-self.received_at[robot]<=self.lost_timeout and
                self.states[robot].task_id==self.task_id and self.states[robot].reply=='SYNCED' and
                self.states[robot].run_state=='SYNCED' and startup_state(self.states[robot],syncing=True))]
            if pending:
                self.reason='waiting SYNCED: '+', '.join(robot+(': '+self.state_errors[robot] if robot in self.state_errors else '') for robot in pending)
                return [self.control(r,'SYNC') for r in ROBOTS]
            self.phase,self.reason='SEARCH_AB',''
        if self.phase not in ('PAUSED','DONE','FAILED'):
            for robot in ROBOTS:
                state = self.states.get(robot)
                if not state or now-self.received_at[robot]>self.lost_timeout:
                    self.pause(f'{robot}: heartbeat timeout',nav_ready=True)
                elif not ((self.private or state.pose_valid) and state.localization_valid and state.nav_ready):
                    detail=state.reason.partition(NAV_READY_DETAIL)[2] if not state.nav_ready else ''
                    self.pause(f'{robot}: health invalid: '+', '.join(name for name in ('localization_valid','nav_ready') if not getattr(state,name))
                        +(NAV_READY_DETAIL+detail if detail else ''),
                        nav_ready=state.localization_valid and not state.nav_ready)
                elif state.run_state in ('STOPPING','PAUSED'):
                    self.pause(f'{robot}: agent paused: '+state.reason,nav_ready=is_recoverable_pause(state.reason))
                if self.phase=='PAUSED':break
            if now-self.started > self.task_timeout:
                self.pause('task timeout')
        output = [self.control(r, 'SYNC', ack_sequence=self.acked[r]) for r in ROBOTS]
        if self.phase == 'PAUSED':
            return output + self._pause_commands()
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
