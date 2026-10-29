from dataclasses import replace
from collections import deque
from .models import *

NAV_KIND = {'SEARCH_VIEWPOINT':'VIEWPOINT_NAV', 'APPROACH_PICKUP':'PICKUP_NAV',
            'WAIT':'WAIT_NAV', 'APPROACH_DEPOSIT':'DEPOSIT_NAV', 'EXIT':'EXIT_NAV'}

class AgentMachine:
    def __init__(self, context, frame, lost_timeout=3.0, path_start_timeout=1.0, task_timeout=600.0):
        import math
        if not math.isfinite(path_start_timeout) or path_start_timeout <= 0:
            raise ValueError('path_start_timeout must be finite and positive')
        self.context, self.frame, self.lost_timeout = context, frame, lost_timeout
        self.path_start_timeout = path_start_timeout
        self.task_timeout = task_timeout
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
        self.peer_transport_received = None
        self.peer_identity_error = ''
        self.peer_history = deque(maxlen=32)
        self.peer_receive_count = self.peer_accept_count = 0
        self.heartbeat_diagnostic = None
        self.last_sequence = 0
        self.paused = False
        self.pause_reply = ''
        self.local_suspended = False
        self.awaiting_peer_resume = False
        self.resume_effect = None
        self.retry = 0
        self.excluded = []
        self.selected_goal = None
        self.health = Health()
        self.guard = None
        self.transit_goal = None
        self.private = context.common_frame_id == context.alignment_id == ''
        self.local_observations = {}
        self.task_started_at = 0.
        self.peer_state = None
        self.synced = False
        self.sync_acknowledged = False
        self.handshake_committed = False
        self.task_active = False
        if self.private:self._state.run_state='PREPARING'

    def state(self):
        state=replace(self._state)
        if not self.health.nav_ready and self.health.nav_ready_reason and NAV_READY_DETAIL not in state.reason:
            state.reason+=NAV_READY_DETAIL+self.health.nav_ready_reason
        return state

    def take_effects(self):
        effects, self.effects = self.effects, []
        return effects

    def receive_peer(self,state,entry,now):
        received=dict(received_monotonic=now,stamp=state.stamp,robot_id=state.robot_id[:160],
            task_id=state.task_id[:160],local_task_id=self.context.task_id[:160],
            run_state=state.run_state[:160],result='robot mismatch',peer_received=self.peer_received)
        self.peer_history.append(received)
        self.peer_receive_count+=1
        expected='ranger' if self.context.robot_id=='go2' else 'go2'
        if state.robot_id!=expected:
            return False
        identity=(state.local_map_id,state.map_version,state.common_frame_id,state.alignment_id)
        error=''
        if state.schema_version!=SCHEMA_VERSION or identity!=(entry['local_map_id'],entry['map_version'],
                self.context.common_frame_id,self.context.alignment_id):
            error='peer map/mode identity changed'
        elif state.simulation!=self.context.simulation:
            error='peer simulation/live mode changed'
        elif self.private and (state.pose_valid or state.common_pose!=Pose2D()):
            error='private peer published coordinates'
        if error:
            received['result']=error
            self.peer_identity_error=error
            if not self.private or self.task_active or self.handshake_committed:self.stop(error)
            self._update_waiting(now)
            return False
        self.peer_identity_error=''
        self.peer_transport_received=now
        if state.task_id!=self.context.task_id:
            received['result']='task mismatch'
            if self.private and (self.task_active or self.handshake_committed) and not state.task_id:
                self.stop('peer restarted during task')
            elif self.private and not state.task_id and not self.task_active and not self.handshake_committed:
                self.peer_state=None
                self.synced=self.sync_acknowledged=False
                self._update_waiting(now)
            return False
        # An online but unready peer still refreshes transport liveness.
        self.peer_received=now
        received.update(result='accepted',peer_received=now)
        self.peer_accept_count+=1
        self.peer_state=replace(state)
        if state.run_state not in ('STOPPING','PAUSED'):self.awaiting_peer_resume=False
        fault=self._peer_fault()
        if fault and (not self.private or self.task_active or self.handshake_committed):self.stop(fault)
        self._update_waiting(now)
        return True

    def _peer_fault(self):
        peer=self.peer_state
        if peer is None:return ''
        failed=[name for name in ('localization_valid','nav_ready') if not getattr(peer,name)]
        if not self.private and not peer.pose_valid:failed.append('pose_valid')
        if failed:
            detail=peer.reason.partition(NAV_READY_DETAIL)[2] if not peer.nav_ready else ''
            return 'peer health invalid: '+', '.join(failed)+(NAV_READY_DETAIL+detail if detail else '')
        if self.private and peer.run_state in ('STOPPING','PAUSED'):
            if is_recoverable_pause(peer.reason):
                return '' if self.awaiting_peer_resume and peer.run_state=='PAUSED' and peer.stopped else 'peer paused for recovery'
            return 'peer paused during task'
        return ''

    def _update_waiting(self,now):
        if not self.private or self.task_active or self.paused:return
        failed=[name for name in ('localization_valid','nav_ready','stopped','motion_terminal')
            if not getattr(self.health,name)]
        if not self.health.pose.valid:failed.append('pose')
        task=bool(self.context.task_id)
        state='SYNCING' if task else 'PREPARING' if failed else 'WAITING_PEER'
        peer=self.peer_state
        if failed:
            reason='local not ready: '+', '.join(failed)
        elif self.peer_identity_error:
            reason=self.peer_identity_error
        elif peer is None or self.peer_received is None:
            reason='waiting peer current-task state' if task else 'waiting first valid peer state'
        elif now-self.peer_received>self.lost_timeout:
            reason='peer current-task state stale' if task else 'valid peer state stale'
        elif peer.task_id!=self.context.task_id:
            reason='waiting peer task SYNC'
        elif not peer.localization_valid or not peer.nav_ready:
            reason='peer not ready: '+', '.join(name for name in ('localization_valid','nav_ready') if not getattr(peer,name))
        elif self.handshake_committed:
            reason=self._peer_fault()
        elif not clean_startup_state(self._state):
            reason='local startup state not clean'
        elif not startup_state(peer,syncing=task):
            reason='peer startup state not clean/ready: '+peer.run_state
        elif task and (self.sync_received is None or now-self.sync_received>self.lost_timeout):
            reason='waiting fresh coordinator SYNC'
        else:
            reason=''
        self.synced=not reason and task
        if self.synced:
            self.sync_acknowledged=True
            state='SYNCED'
        elif not task and not reason:
            state='STANDBY'
        self._state.reply='SYNCED' if self.synced else ''
        self._state.run_state,self._state.reason=state,reason

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
            if self.private and command.phase!='SYNCING':
                return CommandReply(False,'new private task requires SYNCING')
            if command.task_id in self.retired_tasks:
                return CommandReply(False, 'retired task')
            if self._state.token or self._state.run_state not in ('IDLE', 'PAUSED', 'PREPARING', 'WAITING_PEER', 'STANDBY', 'SYNCING', 'SYNCED') or not self._state.stopped or not self.health.motion_terminal:
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
            self.peer_state=None
            self.synced=self.task_active=False
            self.sync_acknowledged=self.handshake_committed=False
            self._state.task_id = command.task_id
            self._state.ready = self._state.deposited = self._state.exited = False
            self._state.sequence=0
            self._state.command_id=self._state.goal_id=''
            self._state.requested_resource=self._state.requested_command_id=''
            self._state.resource_id=self._state.token=self._state.permit_command_id=self._state.resource_status=''
            self._state.run_state,self._state.reason='IDLE',''
            self._state.reply = ''
            self.command = None
            self.local_observations.clear()
            self.effects.clear()
            if self.local_suspended:self.effects.append(Effect('STOP',('', 'new task replaces suspended execution')))
            self.selected_goal=self.transit_goal=None
            self.last_sequence = 0
            self.paused = False
            self.awaiting_peer_resume=False
            self.resume_effect=None
        reply = validate_command(command, self.context)
        if not reply.accepted:
            return reply
        if command.action == 'SYNC':
            if self.private and (self.task_active or self.handshake_committed) and command.phase=='SYNCING':
                return CommandReply(False,'cannot restart active task handshake')
            if self.private and command.phase=='SEARCH_AB' and self.sync_acknowledged:
                self.handshake_committed=True
            self.sync_received = now
            self.phase = command.phase
            self._update_waiting(now)
            return reply
        if command.action in ('PAUSE', 'CANCEL'):
            if command.action == 'CANCEL' and (not self.command or command.cancel_command_id != self.command.command_id):
                return CommandReply(False, 'cancel command mismatch')
            self.stop('coordinator recoverable pause' if self.private and command.action=='PAUSE'
                and command.kind=='NAV_READY_WAIT' else 'coordinator pause/cancel')
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
        self._update_waiting(now)
        if self.private and (not self.synced or self.phase=='SYNCING'):
            return CommandReply(False,'current task handshake not complete')
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
        if self.awaiting_peer_resume:
            return CommandReply(False, 'waiting for peer recovery acknowledgement')
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
        self.task_active=True
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
        # Private approaches complete only through the local executor's strict arrival checks.
        if self.private and result.goal_kind in ('PICKUP_NAV','DEPOSIT_NAV'):
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
        self.resume_effect=None
        if reason.startswith('local health invalid:') and not self.health.nav_ready and self.health.nav_ready_reason:
            reason+=NAV_READY_DETAIL+self.health.nav_ready_reason
        if self._state.run_state == 'STOPPING' or self.paused:
            # A manual stop or a nonrecoverable fault must revoke automatic recovery.
            if self.private and is_recoverable_pause(self._state.reason) and not is_recoverable_pause(reason):
                self._state.reason=reason
                if self.local_suspended:self.effects.append(Effect('STOP',('',reason)))
            elif self.private and is_recoverable_pause(self._state.reason) and (
                    reason.startswith(('local health invalid:','peer health invalid:'))
                    or reason in ('peer heartbeat timeout','coordinator heartbeat timeout')):
                self._state.reason=reason
            return
        self.pause_reply=self._state.reply
        self.pause_state,self.pause_goal_id=self._state.run_state,self._state.goal_id
        self.paused = True
        self.effects.append(Effect('STOP', (self._state.goal_id, reason)))
        self._state.goal_id = ''
        self._state.run_state, self._state.reply, self._state.reason = 'STOPPING', 'CANCELED', reason

    def _heartbeat_timeout(self, reason, now):
        if self.paused and self._state.reason==reason:return
        timers={name:getattr(self,name) for name in ('peer_received','peer_transport_received','sync_received')}
        # Capture the check before stop effects or later callbacks can change its evidence.
        self.heartbeat_diagnostic=dict(reason=reason,robot_id=self.context.robot_id,task_id=self.context.task_id,
            now=now,lost_timeout=self.lost_timeout,task_started_at=self.task_started_at,
            ages={name:None if value is None else now-value for name,value in timers.items()},
            peer_receive_count=self.peer_receive_count,peer_accept_count=self.peer_accept_count,
            peer_history=list(self.peer_history),**timers)
        self.stop(reason)

    def tick(self, now, health):
        self.health = health
        self._state.localization_valid = health.localization_valid and health.pose.valid
        self._state.pose_valid = self._state.localization_valid and not self.private
        self._state.nav_ready, self._state.stopped = health.nav_ready, health.stopped
        if self._state.pose_valid:
            self._state.common_pose = self.frame.to_common(health.pose)
        else:self._state.common_pose=Pose2D()
        self._update_waiting(now)
        recovering=self.private and self.paused and is_recoverable_pause(self._state.reason)
        if self.private and self.context.task_id and self.phase not in ('DONE','FAILED') and (not self.paused or recovering) and now-self.task_started_at>self.task_timeout:
            self.stop('task timeout')
        if self.context.task_id and (not self.private or self.task_active or self.handshake_committed) and (not self.paused or recovering):
            if not self._state.localization_valid:
                self.stop('local health invalid: localization_valid')
            elif self.sync_received is None or now-self.sync_received>self.lost_timeout:
                self._heartbeat_timeout('coordinator heartbeat timeout',now)
            elif (self.peer_received is None and now-self.task_started_at>self.lost_timeout or
                    self.peer_received is not None and now-self.peer_received>self.lost_timeout):
                self._heartbeat_timeout('peer heartbeat timeout',now)
            elif not health.nav_ready:
                self.stop('local health invalid: nav_ready')
            elif self._peer_fault():self.stop(self._peer_fault())
        if self._state.run_state == 'STOPPING':
            if health.stopped and health.motion_terminal:
                self._state.run_state = 'PAUSED'
            return self.take_effects()
        if (self.private and self.paused and is_recoverable_pause(self._state.reason)
                and self.phase in ('SEARCH_AB','SEARCH_C','DEPOSIT') and self._state.localization_valid
                and health.nav_ready and health.stopped and health.motion_terminal
                and self.peer_state and self.peer_state.task_id==self.context.task_id
                and self.peer_state.stopped and self.peer_state.run_state!='STOPPING'
                and not self.peer_identity_error
                and all(received is not None and now-received<=self.lost_timeout
                        for received in (self.sync_received,self.peer_received))
                and self._peer_fault() in ('','peer paused for recovery')):
            self.paused=False
            self.awaiting_peer_resume=self.peer_state.run_state=='PAUSED'
            self._state.reason=''
            self._state.reply=self.pause_reply
            self._state.run_state='IDLE'
            if self.local_suspended:
                self._state.run_state,self._state.goal_id=self.pause_state,self.pause_goal_id
                self.resume_effect=Effect('LOCAL_RESUME')
            elif self.command and self.pause_reply not in ('SUCCEEDED','FAILED','CANCELED'):
                self._state.run_state='PLANNING'
                self.resume_effect=Effect('LOCAL_EXECUTE')
        if self.resume_effect and not self.paused:
            if not self.awaiting_peer_resume:
                self.effects.append(self.resume_effect)
                self.resume_effect=None
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
                reason=health.reason if not health.path_valid and health.reason else decision.reason
                self.stop('execution path monitor: '+(reason or f'resource permit required: {decision.resource_id}'))
