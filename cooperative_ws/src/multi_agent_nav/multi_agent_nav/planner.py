from .models import PlannedAction, Pose2D, ROBOTS

class RulePlanner:
    def __init__(self):
        self.visited = set()
        self.waited = set()

    def decide(self, snapshot):
        if snapshot.scene.get('share_c_coordinates',True) is False:
            return self.decide_local(snapshot)
        actions = []
        for robot in ROBOTS:
            state = snapshot.states[robot]
            if robot in snapshot.commands or not state.stopped:
                continue
            if snapshot.phase == 'DEPOSIT':
                first = snapshot.states['go2']
                if robot == 'ranger' and not first.exited:
                    continue
                if state.exited:
                    continue
                if state.deposited:
                    actions.append(PlannedAction(robot, 'EXIT', goal=Pose2D(*snapshot.scene['exit_points'][robot]),
                        resource_id=state.resource_id, token=state.token, parent_command_id=state.permit_command_id))
                else:
                    obs_id = snapshot.scene['bound_c']
                    obs = snapshot.observations[obs_id]
                    if (robot, obs_id) not in snapshot.rejected:
                        actions.append(PlannedAction(robot, 'APPROACH_DEPOSIT', target_id='C',
                            observation_id=obs_id, goal=obs.common_position, resource_id='basket'))
                continue
            target = {'go2':'A','ranger':'B'}[robot] if snapshot.phase == 'SEARCH_AB' else 'C'
            if snapshot.phase == 'SEARCH_AB' and state.ready:
                if robot not in self.waited and robot in snapshot.scene['wait_points']:
                    self.waited.add(robot)
                    actions.append(PlannedAction(robot, 'WAIT', goal=Pose2D(*snapshot.scene['wait_points'][robot])))
                continue
            clue = next((o for o in snapshot.observations.values() if o.target_id == target and o.navigable
                         and (robot, o.observation_id) not in snapshot.rejected), None)
            if clue and target != 'C':
                actions.append(PlannedAction(robot, 'APPROACH_PICKUP', target_id=target,
                    observation_id=clue.observation_id, goal=clue.common_position))
                continue
            for point in snapshot.scene['viewpoints'][robot]:
                key = (robot, snapshot.phase, point['id'])
                if key in self.visited or point['id'] in snapshot.reservations:
                    continue
                actions.append(PlannedAction(robot, 'SEARCH_VIEWPOINT', viewpoint_id=point['id'],
                                             goal=Pose2D(*point['pose'])))
                break
        return actions

    def decide_local(self, snapshot):
        actions=[]
        for robot in ROBOTS:
            state=snapshot.states[robot]
            if robot in snapshot.commands or not state.stopped:continue
            if snapshot.phase=='SEARCH_AB' and not state.ready:
                actions.append(PlannedAction(robot,'APPROACH_PICKUP',target_id={'go2':'A','ranger':'B'}[robot]))
            elif snapshot.phase in ('SEARCH_C','DEPOSIT') and not state.deposited:
                observation_id=snapshot.local_c.get(robot)
                if not observation_id:
                    actions.append(PlannedAction(robot,'SEARCH_VIEWPOINT',target_id='C'))
                elif snapshot.phase=='DEPOSIT' and (robot,observation_id) not in snapshot.rejected:
                    actions.append(PlannedAction(robot,'APPROACH_DEPOSIT',target_id='C',observation_id=observation_id))
        return actions
