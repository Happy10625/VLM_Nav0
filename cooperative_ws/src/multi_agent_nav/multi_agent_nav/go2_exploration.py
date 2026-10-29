"""Serial local exploration using the existing Go2 adapter and resource guard."""
from dataclasses import asdict, replace
import math
import time
import uuid
from .config import valid_numbers, go2_local_gates, robot_local_gates, file_signature
from .models import Pose2D, RecoverablePause
from .region_guard import RegionGuard
from vlm_nav.arrival import target_distance, stop_radius, arrival_decision, load_contract
from .semantic_search import FixedViewpoints, SemanticFurniture, SelectionState, load_catalog
from .generated_candidates import GeneratedCandidates, GeneratedCandidatesVLM, render_selection


class ExplorationStopped(RuntimeError):
    pass


def exploration_gates(config,robot_id='go2'):
    errors=robot_local_gates(config,robot_id,motion=True)
    robot=config['robot_'+robot_id]
    if robot.get('approach_on_detection'):
        if robot_id!='ranger':
            distance = robot.get('standoff_distance')
            if not valid_numbers([distance], 1) or distance <= 0:
                errors.append('positive finite target standoff_distance required')
        try:load_contract(robot.get('arrival_config_files'))
        except (OSError, KeyError, TypeError, ValueError) as error:
            errors.append('arrival contract invalid: ' + str(error))
    if not robot.get('path_topic'):
        errors.append('executing/replanned path topic required')
    if not robot.get('goal_costmap_topic'):
        errors.append('global goal costmap topic required')
    angles=robot.get('scan_angles')
    if not isinstance(angles,list) or not angles or not valid_numbers(angles,len(angles)):
        errors.append('finite scan angles required')
    if robot.get('exploration_strategy', 'fixed_viewpoints') == 'semantic_furniture':
        try:load_catalog(config,robot_id)
        except (OSError, KeyError, TypeError, ValueError) as error:
            errors.append('semantic catalog invalid: ' + str(error))
    elif robot.get('exploration_strategy') == 'generated_candidates_vlm':
        try:GeneratedCandidates(config,robot_id)
        except (OSError, KeyError, TypeError, ValueError) as error:
            errors.append('generated candidates invalid: ' + str(error))
        targets=config['scene'].get('targets',{})
        if any(not isinstance(targets.get(t),str) or not targets[t].strip() or '现场填写' in targets[t] for t in 'ABC'):
            errors.append('generated scans require measured A/B/C target descriptions')
    elif robot.get('exploration_strategy', 'fixed_viewpoints') != 'fixed_viewpoints':
        errors.append('unknown exploration strategy')
    if robot.get('exploration_strategy','fixed_viewpoints')=='fixed_viewpoints':
        try:measured_viewpoints(config,robot_id)
        except ValueError as error:errors.append(str(error))
    return errors


def measured_viewpoints(config,robot_id='go2'):
    points = config['scene'].get('viewpoints', {}).get(robot_id, [])
    if not points or any(not isinstance(p, dict) or not isinstance(p.get('id'), str)
                         or not p['id'].strip() or not valid_numbers(p.get('pose'), 3) for p in points):
        raise ValueError(f'measured {robot_id} viewpoints required')
    if len({p['id'] for p in points}) != len(points):
        raise ValueError(f'duplicate {robot_id} viewpoint IDs')
    return points


def explore(adapter, spin, config, frame, deadline, target, record, owners,
            stop, before_motion, clock=time.monotonic, *, robot_id='go2', target_id='A'):
    steps=explore_steps(adapter,config,frame,deadline,target,record,owners,stop,before_motion,
        clock,robot_id=robot_id,target_id=target_id)
    while True:
        try:next(steps)
        except StopIteration as completed:return completed.value
        spin()


def explore_steps(adapter, config, frame, deadline, target, record, owners,
            stop, before_motion, clock=time.monotonic, *, robot_id='go2', target_id='A',
            success_radius=None, require_navigation_success=False, task_id=None, command_id=None,
            on_observation=None, initial_observation=None, scan_at_start=False):
    local=robot_id=='ranger' and hasattr(adapter,'begin_local_execution')
    task_id=task_id or robot_id+'_explore/'+uuid.uuid4().hex
    if local:
        command_id=command_id or uuid.uuid4().hex
        adapter.begin_local_execution(task_id,command_id,target_id,target)
    try:
        return (yield from _explore_steps(adapter,config,frame,deadline,target,record,owners,
            stop,before_motion,clock,robot_id=robot_id,target_id=target_id,
            success_radius=success_radius,require_navigation_success=require_navigation_success,
            task_id=task_id,command_id=command_id,on_observation=on_observation,
            initial_observation=initial_observation,scan_at_start=scan_at_start or local and not require_navigation_success))
    finally:
        if local:adapter.node.end_execution()


def _explore_steps(adapter, config, frame, deadline, target, record, owners,
            stop, before_motion, clock=time.monotonic, *, robot_id='go2', target_id='A',
            success_radius=None, require_navigation_success=False, task_id=None, command_id=None,
            on_observation=None, initial_observation=None, scan_at_start=False):
    if robot_id not in ('go2','ranger') or target_id not in ('A','B','C'):
        raise ValueError('unknown robot or target ID')
    original_record=record
    def record(event):
        if getattr(adapter,'visualization',None):adapter.visualization.record(event)
        original_record(dict(event,robot_id=robot_id,target_id=target_id,
            common_frame_id=frame.ids[2],local_map_id=frame.ids[0],map_version=frame.ids[1]))
    guard = RegionGuard({} if require_navigation_success else config['scene']['resources'], config['robot_'+robot_id]['stop_margin'])
    task_id = task_id or robot_id+'_explore/' + uuid.uuid4().hex
    main_command_id = command_id
    cooperative = require_navigation_success
    background_preparation = cooperative or robot_id=='ranger'
    yield None
    current = {}
    strategy = config['robot_'+robot_id].get('exploration_strategy', 'fixed_viewpoints')
    generated = strategy == 'generated_candidates_vlm'
    semantic = strategy in ('semantic_furniture', 'generated_candidates_vlm')
    def background(function):
        while not getattr(adapter.vision,'available',True):
            if clock()>=deadline:raise ExplorationStopped('local preparation deadline reached')
            before_motion()
            yield None
        future=adapter.vision.submit(function)
        try:
            while not future.done():
                if clock()>=deadline:raise ExplorationStopped('local preparation deadline reached')
                before_motion()
                yield None
            before_motion()
            return future.result()
        finally:
            if not future.done():future.cancel()
    geometry=catalog=None
    if background_preparation and semantic:
        prepared=yield from background(lambda:GeneratedCandidates(config,robot_id) if generated else load_catalog(config,robot_id))
        if generated:geometry=prepared
        else:catalog=prepared
    else:
        geometry=GeneratedCandidates(config,robot_id) if generated else None
        catalog=load_catalog(config,robot_id) if semantic and not generated else None
    asset_paths=(geometry.digests if generated else catalog['asset_paths']) if cooperative and semantic else None
    assets_signature=file_signature(asset_paths) if asset_paths is not None else None
    points = geometry.points if generated else catalog['points'] if semantic else measured_viewpoints(config,robot_id)
    policy = GeneratedCandidatesVLM(adapter.client) if generated else SemanticFurniture(adapter.client) if semantic else FixedViewpoints(points)
    state = SelectionState(points)
    if generated:state.task_id = task_id
    completed_points=None
    if generated and robot_id=='ranger':
        saved=getattr(adapter,'generated_observations',None)
        if saved is None or saved[0]!=task_id:
            adapter.generated_observations=(task_id,{})
        completed_points=adapter.generated_observations[1]
        state.records.update({pid:dict(row) for pid,row in completed_points.items() if pid in state.records})
    selection_epoch = None
    history = []
    decision_id = None
    if generated:
        record(dict(state='CANDIDATE_GEOMETRY',task_id=state.task_id,candidates=points,
                    records=state.records.copy(),observed_radius=geometry.observed_radius,
                    rotation_radius=geometry.radius,semantic_yaml=str(geometry.semantic_path) if geometry.semantic_path else None))

    def catalog_unchanged():
        return geometry.unchanged() if generated else load_catalog(config,robot_id) == catalog

    def verify_catalog():
        if not semantic:return
        unchanged=(yield from background(catalog_unchanged)) if background_preparation else catalog_unchanged()
        if not unchanged:raise ExplorationStopped('local map/candidate assets changed during task')

    def emit(state, **fields):
        record(dict(current, state=state, robot_id=robot_id,target_id=target_id,**fields))

    execution=adapter.node if robot_id=='ranger' and hasattr(adapter,'begin_local_execution') else None
    if execution is not None:
        execution.execution_emit=emit
        execution.execution_on_observation=on_observation
        execution.execution_descriptions=dict(config['scene']['targets'],**{target_id:target}) if generated or cooperative else {target_id:target}

    def update(status, **details):
        state.update(current['viewpoint_id'], status, clock(), **details)
        if completed_points is not None and status=='observed' and current['viewpoint_id']!='task_start':
            completed_points[current['viewpoint_id']]=dict(state.records[current['viewpoint_id']])
        if generated and status in ('navigation_failed','observation_failed') and 'display_id' in current:
            history.append(dict(decision_id=decision_id,candidate_id=current['display_id'],
                furniture_id=current['furniture_id'],result=status))
        emit('OBSERVATION_RECORD', observation_status=status, records=state.records.copy())

    execution_path_token=None
    execution_path_acquired=False
    resume_sequence=getattr(adapter,'navigation_resume_sequence',0)
    resume_started=0.
    resume_path_acquired=False
    def health_check(path=None, navigating=False, began=0., acquired=False):
        nonlocal execution_path_token,execution_path_acquired
        nonlocal resume_sequence,resume_started,resume_path_acquired
        health = adapter.get_health()
        if assets_signature is not None and file_signature(asset_paths)!=assets_signature:
            raise ExplorationStopped('local map/candidate assets changed during task')
        if clock() >= deadline:
            raise ExplorationStopped('exploration deadline reached')
        other=owners();local_costmap_ok=adapter.costmap_transform() is not None
        if other or not health.nav_ready or not health.localization_valid or not local_costmap_ok:
            details=getattr(adapter,'last_health_details',{})
            failed=[key for key,value in details.get('checks',{}).items() if not value]
            for key,ok in (('other_goal_owner',not other),('nav_ready',health.nav_ready),
                           ('localization_valid',health.localization_valid),('local_costmap',local_costmap_ok)):
                if not ok and key not in failed:failed.append(key)
            emit('HEALTH_FAILED',failed_gates=failed,health=asdict(health),health_details=details,
                 costmap_checks=getattr(adapter,'last_costmap_checks',{}),other_goal_owners=other)
            if cooperative and not other and health.localization_valid and health.pose.valid:
                raise RecoverablePause('local health invalid: nav_ready'+
                    (' | nav_ready: '+health.nav_ready_reason if health.nav_ready_reason else ''))
            raise ExplorationStopped('motion owner/localization/Nav2/costmap health lost: '+', '.join(failed))
        if execution is not None and execution.goal_pose is not None and execution.state not in ('APPROACH_STOPPING','SENSOR_WAITING'):
            navigating=True
            began=execution.goal_started
            if execution.goal_token!=execution_path_token:
                execution_path_token=execution.goal_token
                execution_path_acquired=False
            acquired=execution_path_acquired
        if execution is not None and execution.state in ('APPROACH_STOPPING','SENSOR_WAITING'):
            navigating=False
        if getattr(adapter,'approach',None) is not None:
            checked = replace(health, path_valid=True)
            checked_path = [health.pose]
        elif navigating:
            if resume_sequence!=getattr(adapter,'navigation_resume_sequence',0):
                resume_sequence=adapter.navigation_resume_sequence
                resume_started=clock();resume_path_acquired=False
            if began<resume_started:
                began=resume_started;acquired=resume_path_acquired
            acquired = acquired or bool(health.path_valid and health.path)
            if began==resume_started:resume_path_acquired=acquired
            if execution is not None and execution.goal_pose is not None:execution_path_acquired=acquired
            waiting = not acquired and health.stopped and clock()-began < config['robot_'+robot_id].get('path_start_timeout', 1.)
            checked = replace(health, path_valid=True) if waiting else health
            checked_path = [health.pose] if waiting else health.path
        else:
            checked = replace(health, path_valid=True)
            checked_path = path if path is not None else [health.pose]
        decision = guard.check(health.pose, health.footprint, checked_path, {}, checked)
        if decision.action != 'ALLOW':
            raise ExplorationStopped(decision.reason or 'protected resource requires coordinator permit')
        return health, acquired

    def stopped():
        if execution is not None:
            while execution.execution_recovering:
                health_check()
                if execution.state in ('FAILED','API_ERROR'):raise ExplorationStopped(execution.last_failure_reason)
                yield None
            result=adapter.hold_motion('exploration stop')
        else:
            result = stop() if stop else adapter.stop_motion('exploration stop')
        expires=min(deadline,clock()+config['robot_'+robot_id].get('cancel_timeout',5.))
        while hasattr(result,'done') and not result.done():
            health_check()
            if clock()>=expires:raise ExplorationStopped('native action stop timeout')
            yield None
        if hasattr(result,'result'):result=result.result()
        if result is False:
            raise ExplorationStopped('native action terminal and physical stop unconfirmed')
        while True:
            health,_=health_check()
            if health.stopped and health.motion_terminal:return
            if (not cooperative and execution is None) or clock()>=expires:
                raise ExplorationStopped('stop prerequisites changed')
            yield None

    def path_decision(path,health):
        return guard.check(health.pose,health.footprint,path,{},replace(health,path_valid=True))

    def path_possible(path,health):
        return path_decision(path,health).action=='ALLOW'

    def parking_free(goal, observation=True):
        free=adapter.costmap_candidate_free(goal,global_goal=True)
        if generated and observation:free=free and adapter.observation_candidate_free(goal)
        if adapter.costmap_transform(global_goal=True) is None:
            raise ExplorationStopped('global goal costmap missing, stale or TF unavailable')
        return free

    def navigate(goal, command_id, kind, target=None, radius=None, planned=None, epoch=None):
        if cooperative:
            yield from verify_catalog()
            health,_=health_check(path=planned.path)
            if (not health.stopped or not health.motion_terminal or epoch != getattr(adapter,'selection_epoch',0)):
                raise ExplorationStopped('motion prerequisites changed after map validation')
            if ((kind=='VIEWPOINT_NAV' or robot_id!='ranger') and
                    not parking_free(goal,observation=kind=='VIEWPOINT_NAV') or
                    kind!='VIEWPOINT_NAV' and (not path_accept(planned) or
                        robot_id!='ranger' and not adapter.approach_candidate_free(goal))):
                raise ExplorationStopped('goal footprint/path changed after map validation')
        before_motion()
        goal_id = adapter.start_navigation(goal, task_id, main_command_id or command_id, kind)
        emit('NAVIGATING', task_id=task_id, command_id=main_command_id or command_id, goal_id=goal_id, goal_kind=kind)
        began = clock(); acquired = False; terminal = None
        while terminal is None:
            if execution is not None:
                while adapter.results:
                    reply=adapter.results.popleft()
                    if (reply.task_id,reply.command_id,reply.goal_id,reply.goal_kind)==(task_id,main_command_id or command_id,goal_id,kind):
                        terminal=reply.status
                if terminal is not None:break
            health, acquired = health_check(navigating=True, began=began, acquired=acquired)
            if not cooperative and robot_id!='ranger' and target is not None and target_distance((health.pose.x,health.pose.y), (target.x,target.y)) <= radius:
                emit('APPROACH_DISTANCE_STOP', cancel_radius=radius)
                (yield from stopped())
                return 'DISTANCE_STOPPED'
            yield None
            while adapter.results:
                reply = adapter.results.popleft()
                if (reply.task_id, reply.command_id, reply.goal_id, reply.goal_kind) == (task_id, main_command_id or command_id, goal_id, kind):
                    terminal = reply.status
        (yield from stopped())
        return terminal

    def approach_attempt(observation, distance, radius):
        epoch = getattr(adapter, 'selection_epoch', 0)
        object_pose = frame.to_local(observation.common_position)
        direct = robot_id=='ranger'
        method = 'target_reference' if direct else 'standoff'
        emit('APPROACH_PLANNING', object_pose=asdict(object_pose), observation_id=observation.observation_id,
             approach_method=method)
        expires = min(deadline, clock()+15.)
        if direct:
            health, _ = health_check()
            goal = Pose2D(object_pose.x,object_pose.y,health.pose.yaw)
        else:
            future = adapter.select_approach(object_pose, [], standoff_distance=distance,
                **({'path_accept':path_accept} if cooperative else {}))
            while not future.done():
                health_check()
                if clock() >= expires:
                    future.cancel()
                    raise ExplorationStopped('target approach planning timeout')
                yield None
            if clock() >= expires:
                raise ExplorationStopped('late target approach plan')
            goal = future.result().goal
            if goal is None:
                emit('APPROACH_UNREACHABLE', reason='no safe standoff candidate')
                return 'APPROACH_UNREACHABLE'
        future = adapter.compute_path(goal)
        while not future.done():
            health_check()
            if clock() >= expires:
                future.cancel()
                raise ExplorationStopped('target approach path timeout')
            yield None
        result = future.result()
        health, _ = health_check()
        if (clock() >= expires or epoch != getattr(adapter, 'selection_epoch', 0)
                or not health.stopped or not health.motion_terminal):
            raise ExplorationStopped('target approach cancelled or prerequisites changed')
        if direct and adapter.costmap_transform(global_goal=True) is None:
            raise ExplorationStopped('global goal costmap missing, stale or TF unavailable')
        if not result.reachable or not result.path:
            emit('APPROACH_UNREACHABLE', reason=(result.reason or 'target reference planner returned no path')
                 if direct else 'standoff footprint/path rejected')
            return 'APPROACH_UNREACHABLE'
        # A target reference can lie on an object; Nav2 validates its tolerated path.
        if (not direct and (not parking_free(goal, observation=False) or
                not (adapter.approach_candidate_free(goal) if cooperative else adapter.costmap_candidate_free(goal)))
                or not path_possible(result.path,health) or (cooperative or direct) and not path_accept(result)):
            emit('APPROACH_UNREACHABLE', reason='target reference path rejected' if direct else 'standoff footprint/path rejected')
            return 'APPROACH_UNREACHABLE'
        current['pose'] = [goal.x, goal.y, goal.yaw]
        emit('APPROACH_SELECTED', approach_method=method,
             **({'requested_goal':asdict(goal)} if direct else {'standoff_distance':distance}))
        return (yield from navigate(goal, uuid.uuid4().hex, 'DEPOSIT_NAV' if target_id=='C' else 'PICKUP_NAV', object_pose, radius, result, epoch))

    def approach_target(observation):
        nonlocal scan_at_start
        if execution is not None:
            object_pose=frame.to_local(observation.common_position)
            if execution.target_reference_position is None:
                execution.set_confirmed_target_reference_position((object_pose.x,object_pose.y,0.))
            target=execution.target_reference_position
            emit('APPROACH_PLANNING',object_pose=asdict(Pose2D(target[0],target[1])),
                observation_id=observation.observation_id)
            steps=adapter.approach_steps(Pose2D(target[0],target[1]),task_id,main_command_id,
                'DEPOSIT_NAV' if target_id=='C' else 'PICKUP_NAV',record,clock,check_motion=health_check)
            try:
                while True:
                    before_motion()
                    health_check()
                    try:next(steps)
                    except StopIteration as completed:
                        if completed.value=='RESUME_SEARCH':scan_at_start=True
                        return completed.value
                    yield None
            finally:
                steps.close()
        contract = (dict(target_success_radius=success_radius,approach_cancel_radius=success_radius,arrival_retry_limit=1)
            if cooperative else load_contract(config['robot_'+robot_id].get('arrival_config_files')))
        success = contract['target_success_radius'] if success_radius is None else success_radius
        retries = contract['arrival_retry_limit']
        object_pose = frame.to_local(observation.common_position)
        nonlocal path_accept
        distance = None if robot_id=='ranger' else min(config['robot_'+robot_id]['standoff_distance'], success)
        if cooperative or robot_id=='ranger':
            future=adapter.read_approach_parameters()
            expires=min(deadline,clock()+15.)
            while not future.done():
                health_check()
                if clock()>=expires:
                    future.cancel()
                    raise ExplorationStopped('approach parameter read timeout')
                yield None
            parameters=future.result()
            values=[parameters.get('xy_goal_tolerance'),parameters.get('planner_tolerance')]
            if not valid_numbers(values,2) or any(v<0 for v in values):
                raise ExplorationStopped('invalid actual Nav2 approach parameters')
            safety=config['robot_'+robot_id].get('approach_safety_margin',.02) if cooperative else 0.
            if not valid_numbers([safety],1) or safety<0:raise ExplorationStopped('invalid approach safety margin')
            budget=success-values[0]-safety
            if budget<=0:raise ExplorationStopped('no approach distance budget')
            if distance is not None:distance=min(distance,budget)
            if robot_id=='ranger' and sum(values)>success:
                raise ExplorationStopped('Nav2 planner and goal tolerances exceed target success radius')
            def path_accept(result):
                return (result.reachable and bool(result.path) and all(p.valid for p in result.path) and
                    target_distance((result.path[-1].x,result.path[-1].y),(object_pose.x,object_pose.y))+values[0]+safety<=success and
                    (adapter.costmap_candidate_free(result.path[-1],global_goal=True) and
                     adapter.costmap_candidate_free(result.path[-1],allow_outside=True) if robot_id=='ranger'
                     else adapter.approach_candidate_free(result.path[-1])))
            emit('APPROACH_BUDGET',success_radius=success,safety_margin=safety,**parameters)
        emit('ARRIVAL_CONTRACT', **contract)
        for retry in range(retries+1):
            health, _ = health_check()
            if not health.stopped or not health.motion_terminal:
                raise ExplorationStopped('arrival verification requires confirmed stop')
            measured = target_distance((health.pose.x,health.pose.y), (object_pose.x,object_pose.y))
            if measured <= success and not cooperative and robot_id!='ranger':
                emit('TARGET_APPROACHED', measured_distance=measured, success_radius=success,
                     arrival_retry=retry, stop_confirmed=True)
                return 'TARGET_APPROACHED'
            radius = stop_radius(success, contract['approach_cancel_radius'], retry)
            terminal = yield from approach_attempt(observation, distance, radius)
            if terminal not in ('SUCCEEDED', 'DISTANCE_STOPPED'):
                emit('APPROACH_FAILED', navigation_status=terminal)
                return 'APPROACH_FAILED'
            health, _ = health_check()
            measured = target_distance((health.pose.x,health.pose.y), (object_pose.x,object_pose.y))
            decision = arrival_decision(measured, success, retry, retries)
            emit('ARRIVAL_VERIFIED', measured_distance=measured, success_radius=success,
                 arrival_retry=retry, decision=decision, stop_confirmed=True)
            if decision == 'SUCCESS':
                emit('TARGET_APPROACHED', measured_distance=measured, success_radius=success, stop_confirmed=True)
                return 'TARGET_APPROACHED'
            if decision == 'FAILED':
                break
            if robot_id!='ranger':
                # Compensate measured terminal overshoot for the standoff planner.
                distance -= measured-success
                if not math.isfinite(distance) or distance <= 0:
                    break
            emit('APPROACH_RETRY', arrival_retry=retry+1,
                 **({'approach_method':'target_reference'} if robot_id=='ranger' else {'standoff_distance':distance}))
        emit('APPROACH_DISTANCE_FAILED', measured_distance=measured, success_radius=success, stop_confirmed=True)
        return 'APPROACH_DISTANCE_FAILED'

    def next_point():
        nonlocal selection_epoch, decision_id
        if not semantic:
            return policy.next_viewpoint(None)
        epoch = getattr(adapter, 'selection_epoch', 0)
        yield from verify_catalog()
        candidates = []
        health, _ = health_check()
        available=geometry.uncovered_points(state.records) if generated else points
        ordered=sorted(available,key=lambda p:(math.hypot(p['pose'][0]-health.pose.x,p['pose'][1]-health.pose.y),p['id'])) if generated else points
        for candidate in ordered:
            if candidate['id'] not in state.eligible(clock()):
                continue
            health, _ = health_check()
            if not health.stopped or not health.motion_terminal:
                raise ExplorationStopped('selection requires confirmed stop')
            goal = Pose2D(*candidate['pose'])
            if not parking_free(goal):
                state.update(candidate['id'], 'navigation_failed', clock(), reason='footprint blocked')
                continue
            future = adapter.compute_path(goal)
            plan_deadline = min(deadline, clock() + 15.)
            while not future.done():
                health_check()
                if clock() >= plan_deadline:
                    future.cancel()
                    raise ExplorationStopped('candidate planning timeout')
                yield None
            result = future.result()
            health, _ = health_check()
            if (not result.reachable or not result.path or generated and not parking_free(goal) or
                    not path_possible(result.path,health)):
                state.update(candidate['id'], 'navigation_failed', clock(), reason='no permitted path')
                continue
            length = sum(math.hypot(a.x-b.x, a.y-b.y) for a,b in zip(result.path, result.path[1:]))
            candidates.append(dict(candidate, path_length=length))
            if generated and len(candidates) == 12:break
        if generated:
            for candidate in candidates[:]:
                if not parking_free(Pose2D(*candidate['pose'])):
                    state.update(candidate['id'],'navigation_failed',clock(),reason='scan space changed during planning')
                    candidates.remove(candidate)
        if not candidates:
            return None
        if epoch != getattr(adapter, 'selection_epoch', 0):
            raise ExplorationStopped('selection cancelled during candidate planning')
        token = state.snapshot([p['id'] for p in candidates])
        selection_epoch = epoch
        decision_id = uuid.uuid4().hex
        snapshot = dict(task_id=state.task_id, decision_id=decision_id, revision=state.revision, main_target=target,
                        search_hint=config['robot_'+robot_id].get('search_hint'), candidates=candidates,
                        records={k: dict(v) for k,v in state.records.items()})
        if getattr(adapter,'selection_evidence_dir',None) is not None:
            from .scan_evidence import reserve_record_directory
            allocate=lambda:reserve_record_directory(adapter.selection_evidence_dir)
            directory=(yield from background(allocate)) if background_preparation else allocate()
            snapshot['evidence_directory']=str(directory)
        if generated:
            snapshot['observed_exclusion_radius']=geometry.observed_radius
            for index,candidate in enumerate(candidates,1):candidate['display_id']=f'P{index}'
            health, _ = health_check()
            def render():
                return render_selection(geometry,candidates,snapshot['records'],health.pose,snapshot['evidence_directory'])
            images=(yield from background(render)) if background_preparation else render()
            health_check()
            snapshot.update(furniture_annotated=bool(geometry.furniture),history=history[-12:],images=images)
        else:
            snapshot.update(targets=config['scene'].get('targets', {}),current_room=catalog['current_room'],
                            furniture=list(catalog['furniture'].values()),image=catalog['image'])
        request_type = 'candidate_selection' if generated else 'furniture_selection'
        record(dict(state='CANDIDATE_REQUEST' if generated else 'FURNITURE_REQUEST', request_type=request_type, **snapshot))
        future = adapter.vision.submit(lambda: policy.next_viewpoint(snapshot))
        expires = min(deadline, clock() + config['robot_'+robot_id].get('api_timeout', 20.))
        try:
            while not future.done():
                health_check()
                if clock() >= expires:
                    raise ExplorationStopped('furniture selection timeout')
                yield None
            if clock() >= expires:
                raise ExplorationStopped('late furniture selection discarded')
            point = future.result()
            yield from verify_catalog()
            health, _ = health_check()
            if (epoch != getattr(adapter, 'selection_epoch', 0) or
                    not state.accepts(token, point['id'], clock()) or
                    not health.stopped or not health.motion_terminal):
                raise ExplorationStopped('stale furniture selection discarded')
        except BaseException:
            future.cancel()
            state.cancel()
            if getattr(adapter.client,'selection_evidence_write_failed',False):
                adapter.evidence_write_failed=True
                adapter.motion_fault=True
            record(dict(state='CANDIDATE_SELECTION_FAILED' if generated else 'FURNITURE_SELECTION_FAILED', request_type=request_type,
                        task_id=state.task_id, decision_id=decision_id, motion_sent=False))
            raise
        record(dict(state='CANDIDATE_SELECTED' if generated else 'FURNITURE_SELECTED', request_type=request_type,
                    furniture_id=point['furniture_id'], viewpoint_id=point['id'],
                    candidate_id=point.get('display_id'), decision_id=decision_id,
                    reason=point['selection_reason'], task_id=state.task_id))
        return point

    path_accept=None
    if initial_observation is not None:
        if getattr(adapter,'visualization',None):adapter.visualization.observe([initial_observation])
        outcome=yield from approach_target(initial_observation)
        if outcome!='RESUME_SEARCH':return outcome

    while True:
        if execution is not None:
            while execution.execution_recovering:
                health_check()
                if execution.state in ('FAILED','API_ERROR'):raise ExplorationStopped(execution.last_failure_reason)
                yield None
            if execution.target_reference_position is not None:
                from .models import Observation
                p=execution.target_reference_position
                outcome=yield from approach_target(Observation(target_id=target_id,status='DETECTED',
                    position_valid=True,common_position=frame.to_common(Pose2D(float(p[0]),float(p[1])))))
                if outcome!='RESUME_SEARCH':return outcome
        initial_point=scan_at_start
        scan_at_start=False
        if initial_point:
            health,_=health_check()
            if not health.stopped or not health.motion_terminal:
                raise ExplorationStopped('initial observation requires confirmed stop')
            point=dict(id='task_start',pose=[health.pose.x,health.pose.y,health.pose.yaw],
                furniture_id='',display_id='START')
            if semantic:state.update(point['id'],'executing',clock())
        else:
            point = yield from next_point()
        if point is None:
            break
        goal = Pose2D(*point['pose']) if semantic or initial_point else frame.to_local(Pose2D(*point['pose']))
        current = {'viewpoint_id': point['id'], 'pose': [goal.x, goal.y, goal.yaw], 'plan_reachable': None,
            'goal_costmap_topic': config['robot_'+robot_id].get('goal_costmap_topic')}
        if semantic:current['furniture_id'] = point['furniture_id']
        if generated:current['display_id'] = point['display_id']
        emit('SELECTED')
        health, _ = health_check()
        if not health.stopped or not health.motion_terminal:
            raise ExplorationStopped('previous motion has not stopped')
        command_id = uuid.uuid4().hex
        if not initial_point:
            if not parking_free(goal):
                emit('FOOTPRINT_BLOCKED',reason='global costmap: footprint outside coverage or intersects obstacle/unknown')
                if semantic:update('navigation_failed')
                continue
            future = adapter.compute_path(goal)
            plan_deadline = min(deadline, clock()+15.)
            while not future.done():
                health_check()
                if clock() >= plan_deadline:
                    future.cancel()
                    raise ExplorationStopped('path planning timeout; reachability unknown')
                yield None
            result = future.result()
            current['plan_reachable'] = result.reachable
            emit('PLANNED' if result.reachable else 'PLAN_FAILED', reason=result.reason)
            if not result.reachable:
                if semantic:update('navigation_failed')
                continue
            if not result.path:
                raise ExplorationStopped('planner returned empty path')
            health, _ = health_check()
            decision = path_decision(result.path,health)
            if not path_possible(result.path,health):
                emit('RESOURCE_BLOCKED', reason=decision.reason or 'no protected resource permit')
                if semantic:update('navigation_failed')
                continue
            if not health.stopped or not health.motion_terminal:
                raise ExplorationStopped('motion prerequisites changed during planning')
            if not parking_free(goal):
                emit('FOOTPRINT_BLOCKED',reason='global costmap: footprint outside coverage or intersects obstacle/unknown')
                if semantic:update('navigation_failed')
                continue
            if semantic and (selection_epoch != getattr(adapter, 'selection_epoch', 0)
                             or point['id'] not in state.eligible(clock())):
                raise ExplorationStopped('selection invalidated before navigation')
            if semantic:update('executing')
            terminal = yield from navigate(goal, command_id, 'VIEWPOINT_NAV',planned=result,
                epoch=selection_epoch if semantic else getattr(adapter,'selection_epoch',0))
            emit('ARRIVED' if terminal == 'SUCCEEDED' else 'NAV_FAILED', navigation_status=terminal, stop_confirmed=True)
            if terminal != 'SUCCEEDED':
                if semantic:update('navigation_failed')
                continue
        health, _ = health_check()
        if generated and not initial_point and not parking_free(health.pose):
            emit('SCAN_SPACE_BLOCKED',actual_stopped_pose=asdict(health.pose),stop_confirmed=True)
            update('observation_failed',reason='actual stopped pose has no full rotation space')
            continue
        before_motion()
        emit('SCANNING')
        adapter.scan_viewpoint_id=point['id']
        descriptions=dict(config['scene']['targets'],**{target_id:target}) if generated or cooperative else {target_id:target}
        future = adapter.start_scan(descriptions, command_id,
            **({'stop_on_target':target_id} if robot_id=='ranger' or cooperative else {}),
            **(dict(task_id=task_id,command_id=main_command_id or command_id,current_view_only=True) if initial_point else {}))
        while not future.done():
            health_check()
            yield None
        if future.cancelled():
            raise ExplorationStopped('observation cancelled')
        try:
            scan_results = future.result()
            if on_observation:
                for observation in scan_results:on_observation(observation)
            observations = [o for o in scan_results if o.target_id == target_id]
        except Exception:
            if not semantic or execution is not None and execution.state in ('FAILED','API_ERROR'):raise
            (yield from stopped())
            update('observation_failed', reason='scan failed')
            continue
        if execution is None or execution.target_reference_position is None:
            yield from stopped()
        view_count=(len(execution.scan_headings) if execution is not None and execution.scan_headings else
            1 if initial_point else len(config['robot_'+robot_id]['scan_angles']))
        incomplete_abc = generated and (len(scan_results)!=len(descriptions)*view_count
                or {(o.target_id,o.view_index) for o in scan_results} !=
                   {(t,i) for t in descriptions for i in range(view_count)}
                or any(o.status not in ('DETECTED','NOT_DETECTED','UNCERTAIN') for o in scan_results))
        if incomplete_abc:
            update('observation_failed', reason='incomplete ABC scan or image/API/validation error')
            if not any(o.status=='DETECTED' for o in observations):continue
            # A valid detection survives errors in other views/targets; the point stays unfinished.
            observations=[o for o in observations if o.status=='DETECTED']
        detections = [o for o in observations if o.status == 'DETECTED']
        early_detection = (robot_id=='ranger' or cooperative) and bool(detections)
        complete_scan = {o.view_index for o in observations} == set(range(view_count)) and all(
            o.status in ('DETECTED','NOT_DETECTED','UNCERTAIN') for o in observations)
        emit('SCAN_RESULT', observations=[asdict(o) for o in observations], stop_confirmed=True,
             early_detection=early_detection and not complete_scan)
        if not early_detection and (not observations or any(o.status not in ('DETECTED','NOT_DETECTED','UNCERTAIN') for o in observations)):
            if semantic:
                update('observation_failed', reason='image/API/validation error')
                continue
            raise ExplorationStopped('target scan missing or API/validation error')
        if not incomplete_abc and not early_detection and not complete_scan:
            if semantic:
                update('observation_failed', reason='incomplete scan directions')
                continue
            raise ExplorationStopped('target scan directions incomplete')
        if semantic and not incomplete_abc and complete_scan:
            update('observed', results=[asdict(o) for o in (scan_results if generated else observations)])
        if generated:
            history.append(dict(decision_id=decision_id,candidate_id=point['display_id'],
                furniture_id=point['furniture_id'],observation_complete=not incomplete_abc,result='DETECTED' if detections else
                    'NOT_DETECTED' if all(o.status=='NOT_DETECTED' for o in observations) else 'UNCERTAIN'))
        if detections:
            outcome = 'TARGET_FOUND' if any(o.navigable for o in detections) else 'TARGET_DETECTED_UNLOCATED'
            emit(outcome)
            if outcome == 'TARGET_FOUND' and config['robot_'+robot_id].get('approach_on_detection', False):
                outcome=yield from approach_target(max((o for o in detections if o.navigable), key=lambda o:o.confidence))
                if outcome=='RESUME_SEARCH':continue
            return outcome
        emit('NOT_DETECTED' if all(o.status == 'NOT_DETECTED' for o in observations) else 'UNCERTAIN')
    if semantic and (any(row['status'] != 'observed' for row in state.records.values()) or generated and not points):
        reason=('本任务候选已覆盖、暂不可达或观察失败；不代表目标不存在' if generated and robot_id=='ranger'
                else '候选暂不可达或观察失败；不代表目标不存在')
        emit('WAITING_VIEWPOINTS', records=state.records, reason=reason)
        return 'WAITING_VIEWPOINTS'
    emit('EXHAUSTED', reason='当前视角已检查完；目标是否存在未知')
    return 'EXHAUSTED'


def exploration_markers(event):
    from visualization_msgs.msg import Marker, MarkerArray
    output = MarkerArray()
    if 'pose' not in event:
        return output
    x, y, yaw = event['pose']
    color = (.2, .2, .2) if event['state'] in ('TARGET_FOUND','TARGET_DETECTED_UNLOCATED','ARRIVED') else (.45, .45, .45)
    if event['state'] in ('PLAN_FAILED','FOOTPRINT_BLOCKED','RESOURCE_BLOCKED','NAV_FAILED','STOPPED','STOP_UNCONFIRMED','REJECTED'):
        color = (0., 0., 0.)
    for index, kind in enumerate((Marker.ARROW, Marker.TEXT_VIEW_FACING)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = event.get('robot_id','go2')+'_current_exploration_point'
        marker.id = index; marker.type = kind; marker.action = Marker.ADD
        marker.pose.position.x = x; marker.pose.position.y = y
        marker.pose.position.z = .1 if index == 0 else .6
        marker.pose.orientation.z = math.sin(yaw/2) if index == 0 else 0.
        marker.pose.orientation.w = math.cos(yaw/2) if index == 0 else 1.
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.
        if index == 0:
            marker.scale.x = .6; marker.scale.y = .12; marker.scale.z = .12
        else:
            marker.scale.z = .2
            marker.text = f"{event['viewpoint_id']} | {event['state']} | reachable={event.get('plan_reachable')}"
        output.markers.append(marker)
    return output


def generated_exploration_markers(display):
    from visualization_msgs.msg import Marker, MarkerArray
    candidates,selected=MarkerArray(),MarkerArray()
    for output in (candidates,selected):
        clear=Marker();clear.header.frame_id='map';clear.action=Marker.DELETEALL;output.markers.append(clear)
    records=display.get('records',{});selectable=display.get('selectable',{})
    for index,point in enumerate(display.get('points',[])):
        pid=point['id'];x,y,yaw=point['pose']
        status=records.get(pid,{}).get('status','unchecked')
        available=pid in selectable and status not in ('observed','executing')
        color=(.95,.5,.08) if available else (.4,.4,.4)
        for offset,kind in enumerate((Marker.SPHERE,Marker.TEXT_VIEW_FACING)):
            marker=Marker();marker.header.frame_id='map';marker.ns='generated_candidates'
            marker.id=index*2+offset;marker.type=kind;marker.action=Marker.ADD
            marker.pose.position.x=x;marker.pose.position.y=y;marker.pose.position.z=.1 if offset==0 else .35
            marker.pose.orientation.w=1.;marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=1.
            marker.scale.x=marker.scale.y=marker.scale.z=.12 if offset==0 else .16
            marker.text=f'{selectable.get(pid,pid)} | {point["furniture_id"]} | {status}'
            candidates.markers.append(marker)
        if display.get('selected')==pid:
            for offset,kind in enumerate((Marker.ARROW,Marker.TEXT_VIEW_FACING)):
                marker=Marker();marker.header.frame_id='map';marker.ns='vlm_selected_candidate'
                marker.id=offset;marker.type=kind;marker.action=Marker.ADD
                marker.pose.position.x=x;marker.pose.position.y=y;marker.pose.position.z=.2 if offset==0 else .6
                marker.pose.orientation.z=math.sin(yaw/2) if offset==0 else 0.
                marker.pose.orientation.w=math.cos(yaw/2) if offset==0 else 1.
                marker.color.g=.85;marker.color.b=.25;marker.color.a=1.
                marker.scale.x=.6 if offset==0 else 0.;marker.scale.y=.12 if offset==0 else 0.;marker.scale.z=.12 if offset==0 else .2
                marker.text=f'VLM selected: {selectable.get(pid,pid)} | {point["furniture_id"]}'
                selected.markers.append(marker)
    return candidates,selected
