from dataclasses import dataclass, field, replace
import math

SCHEMA_VERSION = 1
ROBOTS = ('go2', 'ranger')
TARGETS = ('A', 'B', 'C')
PHASES = ('IDLE', 'SYNCING', 'SEARCH_AB', 'SEARCH_C', 'DEPOSIT', 'DONE', 'PAUSED', 'FAILED')
KINDS = ('SEARCH_VIEWPOINT', 'APPROACH_PICKUP', 'WAIT', 'APPROACH_DEPOSIT', 'EXIT')

@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    @property
    def valid(self):
        return all(math.isfinite(v) for v in (self.x, self.y, self.yaw))

@dataclass
class Command:
    schema_version: int = SCHEMA_VERSION
    task_id: str = ''
    command_id: str = ''
    recipient_id: str = ''
    sequence: int = 0
    action: str = 'EXECUTE'
    phase: str = 'SEARCH_AB'
    kind: str = 'WAIT'
    target_id: str = ''
    viewpoint_id: str = ''
    observation_id: str = ''
    common_frame_id: str = 'lab_ref'
    alignment_id: str = ''
    common_goal: Pose2D = field(default_factory=Pose2D)
    object_position: Pose2D = field(default_factory=Pose2D)
    position_valid: bool = False
    parent_command_id: str = ''
    cancel_command_id: str = ''
    resource_id: str = ''
    token: str = ''
    ack_sequence: int = 0
    simulation: bool = True

@dataclass
class AgentState:
    schema_version: int = SCHEMA_VERSION
    task_id: str = ''
    robot_id: str = ''
    stamp: float = 0.0
    local_map_id: str = ''
    map_version: str = ''
    common_frame_id: str = 'lab_ref'
    alignment_id: str = ''
    simulation: bool = True
    common_pose: Pose2D = field(default_factory=Pose2D)
    pose_valid: bool = False
    localization_valid: bool = False
    nav_ready: bool = False
    stopped: bool = True
    run_state: str = 'IDLE'
    ready: bool = False
    deposited: bool = False
    exited: bool = False
    command_id: str = ''
    sequence: int = 0
    goal_id: str = ''
    reply: str = ''
    reason: str = ''
    resource_id: str = ''
    token: str = ''
    permit_command_id: str = ''
    resource_status: str = ''
    requested_resource: str = ''
    requested_command_id: str = ''

def clean_startup_state(state):
    return (state.sequence == 0 and not any(getattr(state, name) for name in (
        'command_id', 'goal_id', 'ready', 'deposited', 'exited', 'resource_id', 'token',
        'permit_command_id', 'resource_status', 'requested_resource', 'requested_command_id')))

def startup_state(state, syncing=False):
    states = ('SYNCING', 'SYNCED') if syncing else ('WAITING_PEER', 'STANDBY')
    return (clean_startup_state(state) and state.run_state in states and
        state.reply == ('SYNCED' if state.run_state == 'SYNCED' else '') and
        (bool(state.task_id) if syncing else not state.task_id) and
        state.localization_valid and state.nav_ready and state.stopped)

@dataclass
class Observation:
    schema_version: int = SCHEMA_VERSION
    task_id: str = ''
    robot_id: str = ''
    observation_id: str = ''
    viewpoint_id: str = ''
    scan_id: str = ''
    view_index: int = 0
    target_id: str = 'A'
    stamp: float = 0.0
    image_stamp: float = 0.0
    local_map_id: str = ''
    map_version: str = ''
    common_frame_id: str = 'lab_ref'
    alignment_id: str = ''
    simulation: bool = True
    status: str = 'UNCERTAIN'
    confidence: float = 0.0
    common_position: Pose2D = field(default_factory=Pose2D)
    position_valid: bool = False
    reason: str = ''

    @property
    def navigable(self):
        return self.status == 'DETECTED' and self.position_valid and self.common_position.valid

@dataclass
class AgentContext:
    robot_id: str
    task_id: str
    local_map_id: str
    map_version: str
    common_frame_id: str
    alignment_id: str
    simulation: bool

@dataclass
class CommandReply:
    accepted: bool
    reason: str = ''

@dataclass
class GoalResult:
    task_id: str
    command_id: str
    goal_id: str
    goal_kind: str
    status: str
    reason: str = ''

@dataclass
class Health:
    pose: Pose2D = field(default_factory=Pose2D)
    localization_valid: bool = False
    nav_ready: bool = False
    stopped: bool = False
    path: list = field(default_factory=list)
    path_valid: bool = False
    footprint: list = field(default_factory=list)
    motion_terminal: bool = True
    reason: str = ''

@dataclass
class PathResult:
    reachable: bool
    path: list = field(default_factory=list)
    reason: str = ''

@dataclass
class ApproachResult:
    goal: object = None
    reason: str = ''

@dataclass
class Effect:
    kind: str
    payload: object = None

@dataclass
class ResourceRequest:
    task_id: str
    robot_id: str
    parent_command_id: str
    resource_id: str

@dataclass
class ResourceFeedback(ResourceRequest):
    token: str = ''
    status: str = ''
    localization_valid: bool = False
    outside: bool = False
    stopped: bool = False

@dataclass
class ResourceDecision:
    granted: bool
    request: ResourceRequest
    token: str = ''
    status: str = 'QUEUED'

@dataclass
class StartReply:
    accepted: bool
    task_id: str = ''
    reason: str = ''

@dataclass
class Snapshot:
    phase: str
    states: dict
    observations: dict
    reservations: dict
    resources: object
    commands: dict
    scene: dict
    now: float
    received_at: dict = field(default_factory=dict)
    rejected: set = field(default_factory=set)
    local_c: dict = field(default_factory=dict)

@dataclass
class PlannedAction:
    robot_id: str
    kind: str
    target_id: str = ''
    viewpoint_id: str = ''
    observation_id: str = ''
    goal: object = None
    resource_id: str = ''
    parent_command_id: str = ''
    token: str = ''

def normalize_command(command):
    if command.action=='SYNC_LOCAL_C' and command.kind in ('','WAIT','SYNC_LOCAL_C'):
        return replace(command,action='SYNC',kind='SYNC_LOCAL_C')
    return command

def validate_command(command, context):
    command=normalize_command(command)
    if command.schema_version != SCHEMA_VERSION:
        return CommandReply(False, 'schema mismatch')
    if command.recipient_id != context.robot_id and not (
        command.recipient_id == '*' and command.action == 'SYNC'
    ):
        return CommandReply(False, 'recipient mismatch')
    for name in ('task_id', 'common_frame_id', 'alignment_id', 'simulation'):
        if getattr(command, name) != getattr(context, name):
            return CommandReply(False, f'{name} mismatch')
    private = context.common_frame_id == context.alignment_id == ''
    if private and command.action=='GRANT_RESOURCE':
        return CommandReply(False,'private local task does not use resource grants')
    if command.action == 'SYNC' and (command.kind == 'SYNC_LOCAL_C') != private:
        return CommandReply(False, 'coordinate mode mismatch')
    if private and command.action == 'SYNC' and (not command.task_id or command.phase not in PHASES or
            command.position_valid or command.common_goal != Pose2D() or command.object_position != Pose2D() or
            any(getattr(command, name) for name in ('command_id','sequence','viewpoint_id','resource_id',
                'token','parent_command_id','cancel_command_id','target_id','observation_id'))):
        return CommandReply(False, 'invalid private task sync')
    if command.action not in ('EXECUTE', 'SYNC', 'PAUSE', 'CANCEL', 'GRANT_RESOURCE'):
        return CommandReply(False, 'unknown action')
    if command.action == 'EXECUTE':
        if command.kind not in KINDS or command.phase not in PHASES:
            return CommandReply(False, 'unknown kind/phase')
        if private and (command.position_valid or command.common_goal != Pose2D() or
                command.object_position != Pose2D() or command.viewpoint_id or command.resource_id or command.token):
            return CommandReply(False, 'private command must contain intent only')
        if not private and not command.position_valid:
            return CommandReply(False, 'position invalid')
        pose = command.object_position if command.kind.startswith('APPROACH') else command.common_goal
        if not pose.valid:
            return CommandReply(False, 'nonfinite position')
        expected = {'go2': 'A', 'ranger': 'B'}[context.robot_id]
        if command.kind == 'APPROACH_PICKUP' and command.target_id != expected:
            return CommandReply(False, 'wrong pickup target')
        if command.kind == 'APPROACH_DEPOSIT' and command.target_id != 'C':
            return CommandReply(False, 'wrong deposit target')
        if private and command.kind == 'SEARCH_VIEWPOINT' and command.target_id != 'C':
            return CommandReply(False, 'private search target must be C')
        if private and command.kind == 'EXIT':
            return CommandReply(False, 'private C completes at the approach pose without EXIT')
        if private and command.kind == 'WAIT':
            return CommandReply(False, 'private READY waits locally')
    return CommandReply(True)
