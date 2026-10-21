import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from multi_agent_task_msgs.msg import AgentState,Observation,Command
from .node_common import settings,qos
from .mock import make_machine
from .adapters.fake import FakeAdapter
from .runner import AgentRunner
from .ros_codec import to_ros,from_ros
from .models import Pose2D

class AgentNode(Node):
    def __init__(self):
        super().__init__('multi_agent_task_agent')
        self.declare_parameter('robot_id','go2')
        robot=self.get_parameter('robot_id').value
        if robot not in ('go2','ranger'):raise ValueError('unknown robot')
        config,simulation,prefix,gates=settings(self)
        self.gates=gates
        self.observer=None
        self.blocked=bool(gates)
        self.machine=self.adapter=self.runner=None
        self.state_pub=self.create_publisher(AgentState,f'{prefix}/{robot}/state',qos())
        self.observation_pub=self.create_publisher(Observation,f'{prefix}/{robot}/observation',qos(True))
        if simulation:
            machine,frame=make_machine(config,robot,True)
            detections={'C':Pose2D(9,1)}
            if robot=='ranger':detections.update(A=Pose2D(-3,-3),B=Pose2D(3,-3))
            self.adapter=FakeAdapter(robot,detections,delay=.5)
            self.adapter.pose=frame.to_local(Pose2D(-3,-3) if robot=='go2' else Pose2D(3,-3))
            self.machine=machine
        elif not gates:
            from .adapters.factory import create_runtime
            machine,frame=make_machine(config,robot,False)
            self.observer,self.adapter=create_runtime(robot,config,global_goal_checks=True)
            self.adapter.client=self.adapter.create_vision_client(dict(config['robot_'+robot],vision_mode='abc'))
            self.machine=machine
        else:
            self.get_logger().warning('Live adapter not activated: '+'; '.join(gates))
        if self.machine:
            self.runner=AgentRunner(self.machine,self.adapter,config['scene']['targets'],self.publish_observation,
                config=config,success_radius=1.,owners=self.other_owners)
        self.robot,self.config,self.simulation=robot,config,simulation
        self.map_probe=None;self.map_ready=False;self.map_verified_at=0.;self.next_map_probe=0.
        if not simulation and self.adapter:
            from rcl_interfaces.srv import GetParameters
            self.map_client=self.create_client(GetParameters,'/map_server/get_parameters')
            self.adapter.task_map_ready=False
        self.get_logger().info(f'Task prefix: {prefix}; robot={robot}; simulation={simulation}')
        self.create_subscription(Command,f'{prefix}/go2/command',self.command,qos())
        peer='ranger' if robot=='go2' else 'go2'
        self.create_subscription(AgentState,f'{prefix}/{peer}/state',lambda m:self.peer(m,peer),qos())
        self.create_subscription(Observation,f'{prefix}/{peer}/observation',lambda m:self.peer_observation(m,peer),qos(True))
        self.last_observations={}
        self.create_timer(.05,self.tick)
        self.create_timer(1/config['scene'].get('heartbeat_hz',2.0),self.publish_state)

    def other_owners(self):
        from .runner import other_motion_owners
        return other_motion_owners(self.get_node_names(),self.robot)

    def poll_map(self,now):
        if self.simulation or not self.adapter:return
        if self.map_probe is None and now>=self.next_map_probe:
            from rcl_interfaces.srv import GetParameters
            from .go2_check import MapParameterRead
            self.map_probe=MapParameterRead(self.map_client,GetParameters.Request(names=['yaml_filename']))
        if self.map_probe is not None:
            self.map_probe.poll(now)
            if self.map_probe.path:
                from .config import map_file_gates
                expected=Path(self.config['robot_'+self.robot]['map_path']).expanduser().resolve()
                self.map_ready=(Path(self.map_probe.path).resolve()==expected and not map_file_gates(self.config,self.robot))
                self.map_verified_at=now
                self.map_probe=None;self.next_map_probe=now+1.
        self.adapter.task_map_ready=self.map_ready and now-self.map_verified_at<=3.

    def command(self,message):
        if not self.machine:return
        reply=self.machine.accept(from_ros(message),time.monotonic())
        if message.recipient_id in (self.robot,'*') and not reply.accepted:
            self.get_logger().warning('command rejected: '+reply.reason,throttle_duration_sec=2)
        self.publish_state()

    def peer(self,message,source):
        if self.machine and message.robot_id==source:
            self.machine.receive_peer(from_ros(message),self.config['alignment']['robots'][source],time.monotonic())

    def peer_observation(self,message,source):
        if message.robot_id==source and self.machine and message.task_id==self.machine.context.task_id and message.simulation==self.simulation:
            self.last_observations[message.observation_id]=from_ros(message)
            if len(self.last_observations)>1000:self.last_observations.pop(next(iter(self.last_observations)))

    def publish_observation(self,observation):
        self.observation_pub.publish(to_ros(observation))

    def publish_state(self):
        if self.machine:
            state=self.machine.state()
        else:
            from .models import AgentState as State
            entry=self.config['alignment']['robots'][self.robot]
            private=self.config['scene'].get('share_c_coordinates',True) is False
            state=State(robot_id=self.robot,local_map_id=entry['local_map_id'],map_version=entry['map_version'],
                common_frame_id='' if private else self.config['alignment']['common_frame_id'],
                alignment_id='' if private else self.config['alignment']['alignment_id'],simulation=False,run_state='PAUSED',
                reason='; '.join(self.gates))
        state.stamp=self.get_clock().now().nanoseconds*1e-9
        self.state_pub.publish(to_ros(state))

    def tick(self):
        if self.runner:
            self.poll_map(time.monotonic())
            before=self.machine.state()
            self.runner.tick(time.monotonic())
            if before!=self.machine.state():self.publish_state()

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=AgentNode()
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    if node.observer:executor.add_node(node.observer)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if node.machine and rclpy.ok():
            node.machine.stop('agent shutdown')
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                node.runner.tick(time.monotonic())
                node.publish_state()
                if node.machine.state().run_state=='PAUSED':break
                executor.spin_once(timeout_sec=.05)
        if node.adapter and hasattr(node.adapter,'close'):node.adapter.close()
        executor.shutdown()
        if node.observer:node.observer.destroy_node()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
