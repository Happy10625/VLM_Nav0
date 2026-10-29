"""Execute machine effects without blocking ROS callbacks; late futures are fenced."""
from dataclasses import replace
from pathlib import Path
import math
import tempfile
from .models import Effect, Observation, TARGETS, Pose2D, is_recoverable_pause, RecoverablePause

def other_motion_owners(names,robot_id):
    allowed={'multi_agent_task_agent',f'multi_agent_{robot_id}_agent'}
    blocked={'vlm_nav','multi_agent_go2_check_motion','multi_agent_ranger_check_motion',
        'multi_agent_go2_rgbd','multi_agent_ranger_rgbd','multi_agent_go2_agent','multi_agent_ranger_agent',
        'multi_agent_task_agent','multi_agent_coordinator'}
    result=set(names)&(blocked-allowed-{f'multi_agent_{robot_id}_rgbd'})
    if robot_id=='go2' and names.count('multi_agent_coordinator')<=1:result.discard('multi_agent_coordinator')
    if sum(names.count(n) for n in allowed)>1:result.add('duplicate cooperative agent')
    if names.count(f'multi_agent_{robot_id}_rgbd')>1:result.add('duplicate RGB-D observer')
    return sorted(result)

class AgentRunner:
    def __init__(self,machine,adapter,descriptions,on_observation,*,config=None,success_radius=1.0,owners=lambda:[]):
        self.machine,self.adapter,self.descriptions=machine,adapter,descriptions
        self.on_observation=on_observation
        self.pending=[]
        self.config,self.success_radius=config,success_radius
        self.owners=owners
        if not math.isfinite(success_radius) or success_radius<=0:raise ValueError('positive success radius required')
        self.local_steps=None
        self.local_identity=None
        self.local_completion=None
        self.now=0.
        self.adapter.motion_guard=self.motion_guard
        self.adapter.local_pause_enabled=machine.private
        if machine.private:
            root=Path(config['robot_'+machine.context.robot_id].get('experiment_records_root',tempfile.gettempdir()))
            self.adapter.selection_evidence_dir=root/'cooperative'/machine.context.robot_id/'selections'
            self.adapter.scan_evidence_dir=root/'cooperative'/machine.context.robot_id/'scans'

    def motion_guard(self):
        machine=self.machine;health=self.adapter.get_health()
        if machine.paused or not machine.command or not health.localization_valid or not health.pose.valid:
            raise RuntimeError('motion identity/health invalid')
        if self.local_steps is not None and (self.local_identity!=(machine.command.task_id,machine.command.command_id)
                or self.local_identity[0]!=machine.context.task_id):
            raise RuntimeError('local motion identity changed')
        if self.owners():raise RuntimeError('another local motion owner is active')
        if not health.nav_ready:
            raise RecoverablePause('local health invalid: nav_ready'+
                (' | nav_ready: '+health.nav_ready_reason if health.nav_ready_reason else ''))
        for kind,received in (('coordinator',machine.sync_received),('peer',machine.peer_received)):
            if received is None or self.now-received>machine.lost_timeout:
                raise RecoverablePause(kind+' heartbeat timeout')

    def _record_local(self,event):
        machine=self.machine
        if event['state']=='APPROACH_PLANNING' and machine.context.robot_id=='ranger':
            machine._state.goal_id=''
            machine._state.run_state='LOCAL_EXECUTING'
        elif event['state']=='APPROACH_NAVIGATE':
            machine._state.goal_id=event['goal_id']
        elif event['state']=='NAVIGATING':
            machine.navigation_started(event['goal_id'],event['goal_kind'],self.now)
        elif event['state'] in ('ARRIVED','NAV_FAILED','TARGET_APPROACHED','SCANNING','APPROACH_STOPPING','SENSOR_WAITING'):
            machine._state.goal_id=''
            machine._state.run_state='LOCAL_EXECUTING'
        record=getattr(self.adapter,'record_event',None)
        if record:record(event)

    def _local_observation(self,observation):
        self.machine.on_observation(replace(observation,scan_id=self.machine.command.command_id),self.now)

    def _start_local(self):
        from .go2_exploration import explore_steps
        machine=self.machine;command=machine.command
        self.local_identity=(command.task_id,command.command_id)
        machine._state.run_state='LOCAL_EXECUTING'
        config=dict(self.config)
        robot=dict(config['robot_'+machine.context.robot_id])
        robot['approach_on_detection']=command.kind!='SEARCH_VIEWPOINT'
        robot['scan_angles']=[-(i+1)*math.pi/4 for i in range(8)]
        config['robot_'+machine.context.robot_id]=robot
        self.local_steps=explore_steps(self.adapter,config,machine.frame,
            machine.task_started_at+machine.task_timeout,self.descriptions[command.target_id],
            self._record_local,self.owners,None,self.motion_guard,clock=lambda:self.now,
            robot_id=machine.context.robot_id,target_id=command.target_id,
            success_radius=None if hasattr(self.adapter,'begin_local_execution') else self.success_radius,require_navigation_success=True,
            scan_at_start=machine.context.robot_id=='ranger' and command.sequence==1,
            task_id=command.task_id,command_id=command.command_id,on_observation=self._local_observation,
            initial_observation=machine.local_observations.get(command.observation_id) if command.kind=='APPROACH_DEPOSIT' else None)

    def _advance_local(self):
        machine=self.machine;command=machine.command
        if machine.local_suspended:return
        if machine.paused or not command or self.local_identity!=(command.task_id,command.command_id):
            self.local_steps.close();self.local_steps=None
            return
        if machine.peer_received is None:return
        try:
            next(self.local_steps)
        except StopIteration as completed:
            self.local_steps=None
            self.local_completion=(self.local_identity,completed.value)
        except Exception as error:
            self.local_steps.close();self.local_steps=None
            machine.stop(str(error) if machine.private and isinstance(error,RecoverablePause)
                         else 'local execution: '+str(error))

    def _complete_local(self):
        machine=self.machine;command=machine.command
        identity,outcome=self.local_completion
        if machine.paused or not command or identity!=(command.task_id,command.command_id) or identity[0]!=machine.context.task_id:
            self.local_completion=None
            return
        health=self.adapter.get_health()
        if self.owners():
            machine.stop('another local motion owner is active')
            return
        if outcome=='TARGET_APPROACHED' and hasattr(self.adapter,'approach_steps'):
            approach=getattr(self.adapter,'completed_approach',None)
            if (approach is None or approach.state!='SUCCEEDED' or
                    (approach.task_id,approach.command_id)!=identity):
                machine.fail('local approach success evidence missing')
                self.local_completion=None
                return
            distance=approach.target_reference_distance((health.pose.x,health.pose.y,health.pose.yaw))
            if not (health.nav_ready and health.localization_valid and health.stopped and health.motion_terminal
                    and distance<=approach.p.target_success_radius):return
        elif not (health.nav_ready and health.localization_valid and health.stopped and health.motion_terminal):return
        self.local_completion=None
        if outcome=='TARGET_APPROACHED' and command.kind in ('APPROACH_PICKUP','APPROACH_DEPOSIT'):
            if command.kind=='APPROACH_PICKUP':machine._state.ready=True
            else:machine._state.deposited=True
        elif outcome!='TARGET_FOUND' or command.kind!='SEARCH_VIEWPOINT':
            machine.fail('local execution: '+str(outcome));return
        machine._state.reply,machine._state.run_state='SUCCEEDED','IDLE'
        machine._state.goal_id=''

    def tick(self,now):
        self.now=now
        try:
            if hasattr(self.adapter,'advance'):self.adapter.advance(now)
            if hasattr(self.adapter,'tick'):self.adapter.tick()
        except Exception as error:
            self.machine.stop(str(error) if self.machine.private and isinstance(error,RecoverablePause)
                              else 'adapter tick failed: '+str(error))
            self.dispatch(self.machine.take_effects(),now)
        if self.local_steps and self.machine._state.run_state=='NAVIGATING' and any(
                (result.task_id,result.command_id,result.goal_id,result.goal_kind)==(
                    self.machine.context.task_id,self.machine.command.command_id,
                    self.machine._state.goal_id,self.machine.goal_kind) for result in self.adapter.results):
            # Native terminal clears its executing path before the core consumes the result.
            self.machine._state.run_state='LOCAL_STOP'
        while self.adapter.results and self.local_steps is None:
            self.machine.on_goal_result(self.adapter.results.popleft(),now)
        effects=self.machine.tick(now,self.adapter.get_health())
        for item in list(self.pending):
            future,identity,kind,goal=item
            if not future.done():continue
            self.pending.remove(item)
            command=self.machine.command
            if future.cancelled() or self.machine.paused or not command or identity!=(command.task_id,command.command_id):
                continue
            try:
                value=future.result()
                if kind=='APPROACH':self.machine.on_approach(value,now)
                elif kind in ('PLAN','PLAN_TRANSIT'):self.machine.on_path(value,goal,now)
                else:self.machine.scan_completed(value,now)
            except Exception:
                if kind=='SCAN':
                    self.machine.scan_completed([Observation(target_id=t,scan_id=command.command_id,
                        observation_id=f'{command.command_id}/error/{t}',status='API_ERROR',reason='scan operation failed') for t in TARGETS],now)
                else:self.machine.fail(f'{kind.lower()} operation failed')
        effects+=self.machine.take_effects()
        self.dispatch(effects,now)
        if self.local_steps:
            self._advance_local()
            self.dispatch(self.machine.take_effects(),now)
        if self.local_completion is not None:
            self._complete_local()
            self.dispatch(self.machine.take_effects(),now)

    def dispatch(self,effects,now):
        for effect in effects:
            command=self.machine.command
            if effect.kind=='STOP':
                self.local_completion=None
                goal_id,reason=effect.payload
                if (self.machine.private and is_recoverable_pause(reason) and self.local_steps
                        and getattr(self.adapter,'approach',None) is None
                        and hasattr(self.adapter,'pause_local_execution')):
                    if not self.machine.local_suspended:
                        self.machine.local_suspended=True
                        self.adapter.pause_local_execution(reason)
                    continue
                self.machine.local_suspended=False
                if self.local_steps:self.local_steps.close();self.local_steps=None
                for future,*_ in self.pending:future.cancel()
                self.pending.clear()
                if goal_id:self.adapter.cancel_navigation(goal_id)
                self.adapter.stop_motion(reason)
            elif effect.kind=='OBSERVATION':self.on_observation(effect.payload)
            elif command and not self.machine.paused:
                try:
                    if effect.kind=='LOCAL_RESUME':
                        self.machine.navigation_started_at=now
                        self.machine.path_acquired=False
                        self.adapter.resume_local_execution()
                        self.machine.local_suspended=False
                        continue
                    if effect.kind=='LOCAL_EXECUTE':self._start_local();continue
                    if effect.kind=='APPROACH':future=self.adapter.select_approach(*effect.payload);goal=None
                    elif effect.kind in ('PLAN','PLAN_TRANSIT'):
                        goal=effect.payload;future=self.adapter.compute_path(goal)
                    elif effect.kind=='SCAN':
                        future=self.adapter.start_scan(self.descriptions,effect.payload);goal=None
                    elif effect.kind=='NAVIGATE':
                        goal,kind=effect.payload
                        goal_id=self.adapter.start_navigation(goal,command.task_id,command.command_id,kind)
                        self.machine.navigation_started(goal_id,kind,now)
                        continue
                    else:continue
                    self.pending.append((future,(command.task_id,command.command_id),effect.kind,goal))
                except Exception as error:
                    self.machine.stop(str(error) if self.machine.private and isinstance(error,RecoverablePause)
                                      else f'{effect.kind.lower()} adapter failure')
                    self.dispatch(self.machine.take_effects(),now)
