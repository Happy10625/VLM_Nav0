"""Synthetic fixtures and deterministic dual-fake harness; never loads live interfaces."""
from dataclasses import replace
from .models import *
from .frame_adapter import configured_frame
from .agent import AgentMachine
from .coordinator import Coordinator
from .region_guard import RegionGuard
from .adapters.fake import FakeAdapter
from .runner import AgentRunner

def mock_config():
    return {'alignment':{'common_frame_id':'lab_ref','alignment_id':'SYNTHETIC_v1','validated':True,'simulation':True,
        'report':{'synthetic':True},'robots':{
        'go2':{'local_map_id':'synthetic_go2','map_version':'1','transform':[0,0,0]},
        'ranger':{'local_map_id':'synthetic_ranger','map_version':'2','transform':[10,-4,.7]}}},
        'scene':{'scenario_id':'synthetic','enabled':True,'simulation':True,'targets':{'A':'A','B':'B','C':'C'},
            'viewpoints':{'go2':[{'id':'vg','pose':[-2,-3,0]}], 'ranger':[{'id':'vr','pose':[3,-3,0]}]},
            'wait_points':{'go2':[-2,-4,0],'ranger':[4,-4,0]},
            'exit_points':{'go2':[5,-3,0],'ranger':[12,-3,0]},
            'resources':{'basket':{'polygon':[[7,-1],[11,-1],[11,3],[7,3]]}},
            'heartbeat_hz':2.0,'lost_timeout':3.0,'task_timeout':600.0},
        'robot_go2':{'robot_id':'go2','stop_margin':.1},'robot_ranger':{'robot_id':'ranger','stop_margin':.1}}

def make_machine(config,robot,simulation):
    alignment=config['alignment'];entry=alignment['robots'][robot]
    frame=configured_frame(config,robot)
    context=AgentContext(robot,'',*frame.ids,simulation)
    machine=AgentMachine(context,frame,config['scene'].get('lost_timeout',3.0),
        config[f'robot_{robot}'].get('path_start_timeout',1.0))
    regions={}
    for key,region in (config['scene'].get('resources',{}).items() if not machine.private else []):
        value=dict(region,polygon=frame.polygon_to_local(region['polygon']))
        value['transit_exits']={r:[[p.x,p.y,p.yaw] for p in (frame.to_local(Pose2D(*point)) for point in points)]
                                 for r,points in region.get('transit_exits',{}).items()}
        regions[key]=value
    machine.guard=RegionGuard(regions,config[f'robot_{robot}'].get('stop_margin'))
    return machine,frame

def make_coordinator(config,simulation,gates=None):
    alignment=config['alignment']
    ids={r:(alignment['robots'][r]['local_map_id'],alignment['robots'][r]['map_version']) for r in ROBOTS}
    return Coordinator(config['scene'],alignment['alignment_id'],ids,simulation,gates)

class MockMission:
    def __init__(self,config=None):
        self.config=config or mock_config()
        self.coordinator=make_coordinator(self.config,True)
        self.now=0.0
        self.machines={};self.frames={};self.adapters={};self.runners={}
        self.online={r:True for r in ROBOTS}
        self.executed=[];self.seen_commands=set();self.duplicate_commands=False
        for robot in ROBOTS:
            machine,frame=make_machine(self.config,robot,True)
            self.machines[robot],self.frames[robot]=machine,frame
            # Ranger detects A and shares its common physical location to Go2.
            detections={'C':Pose2D(9,1)}
            if robot=='ranger':detections.update(A=Pose2D(-3,-3),B=Pose2D(3,-3))
            if not machine.context.common_frame_id:
                detections.update({{'go2':'A','ranger':'B'}[robot]:Pose2D(-3,-3) if robot=='go2' else Pose2D(3,-3)})
            adapter=FakeAdapter(robot,detections)
            common_initial=Pose2D(-3,-3) if robot=='go2' else Pose2D(3,-3)
            adapter.pose=frame.to_local(common_initial)
            self.adapters[robot]=adapter
            self.runners[robot]=AgentRunner(machine,adapter,self.config['scene']['targets'],
                lambda obs:self.coordinator.ingest_observation(obs,self.now),config=self.config,success_radius=1.)
            machine.tick(0,adapter.get_health())
            self.coordinator.ingest_state(machine.state(),0)

    def start(self):
        return self.coordinator.start(self.config['scene']['scenario_id'],self.now)

    def pause(self,reason):
        for cmd in self.coordinator.pause(reason):
            self.machines[cmd.recipient_id].accept(cmd,self.now)

    def step(self,dt=.1):
        self.now+=dt
        for robot in ROBOTS:
            self.runners[robot].tick(self.now)
            if not self.online[robot]:continue
            state=self.machines[robot].state()
            self.coordinator.ingest_state(state,self.now)
            peer='ranger' if robot=='go2' else 'go2'
            if self.online[peer]:self.machines[peer].peer_received=self.now
        for command in self.coordinator.tick(self.now):
            if not self.online[command.recipient_id]:continue
            reply=self.machines[command.recipient_id].accept(command,self.now)
            if command.action=='EXECUTE' and reply.accepted and command.command_id not in self.seen_commands:
                self.executed.append((command.recipient_id,replace(command)))
                self.seen_commands.add(command.command_id)
            if self.duplicate_commands:self.machines[command.recipient_id].accept(command,self.now)

    def run(self,steps):
        for _ in range(steps):self.step()
