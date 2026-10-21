from collections import deque
import uuid
from .models import ResourceDecision

class ResourceManager:
    def __init__(self):
        self.held = {}
        self.queues = {}
        self.frozen = set()

    def request(self, request, now):
        current = self.held.get(request.resource_id)
        if current and current.request == request:
            return current
        queue = self.queues.setdefault(request.resource_id, deque())
        if request not in queue:
            queue.append(request)
        if current or request.robot_id in self.frozen or any(
            p.request.robot_id == request.robot_id for p in self.held.values()
        ) or queue[0] != request:
            return ResourceDecision(False, request)
        queue.popleft()
        permit = ResourceDecision(True, request, uuid.uuid4().hex, 'RESERVED')
        self.held[request.resource_id] = permit
        return permit

    def update(self, feedback, now):
        p = self.held.get(feedback.resource_id)
        if not p or feedback.robot_id in self.frozen or not feedback.localization_valid or (
            feedback.task_id, feedback.robot_id, feedback.parent_command_id, feedback.resource_id, feedback.token
        ) != (p.request.task_id, p.request.robot_id, p.request.parent_command_id, p.request.resource_id, p.token):
            return False
        if feedback.status == 'ENTERED':
            p.status = 'ENTERED'
            return True
        if feedback.status == 'EXIT_CONFIRMED' and p.status != 'ENTERED':
            return False
        if feedback.status not in ('EXIT_CONFIRMED', 'CANCEL_OUTSIDE') or not feedback.outside or not feedback.stopped:
            return False
        del self.held[feedback.resource_id]
        return True

    def cancel_queued(self, robot_id, command_id):
        for key, queue in self.queues.items():
            self.queues[key] = deque(r for r in queue if (r.robot_id,r.parent_command_id)!=(robot_id,command_id))

    def freeze(self, robot_id, reason):
        if any(p.request.robot_id == robot_id for p in self.held.values()):
            self.frozen.add(robot_id)
