import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from multi_agent_task_msgs.msg import AgentState,Observation,Command
from multi_agent_task_msgs.srv import StartTask
from .node_common import settings,qos
from .mock import make_coordinator
from .ros_codec import from_ros,to_ros

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('multi_agent_coordinator')
        config,simulation,prefix,gates=settings(self)
        self.coordinator=make_coordinator(config,simulation,gates)
        self.command_pub=self.create_publisher(Command,f'{prefix}/go2/command',qos())
        self.status_pub=self.create_publisher(String,f'{prefix}/coordinator/status',qos())
        for robot in ('go2','ranger'):
            self.create_subscription(AgentState,f'{prefix}/{robot}/state',
                lambda m,r=robot:self.state(m,r),qos())
            self.create_subscription(Observation,f'{prefix}/{robot}/observation',
                lambda m,r=robot:self.observation(m,r),qos(True))
        self.create_service(StartTask,f'{prefix}/coordinator/start',self.start)
        self.create_service(Trigger,f'{prefix}/coordinator/pause',self.pause)
        self.create_timer(1/config['scene'].get('heartbeat_hz',2.0),self.tick)
        self.get_logger().info(f'Task prefix: {prefix}; explicit start required')
        if gates:self.get_logger().warning('Live start blocked: '+'; '.join(gates))

    def state(self,message,source):
        if message.robot_id==source:self.coordinator.ingest_state(from_ros(message),time.monotonic())

    def observation(self,message,source):
        if message.robot_id==source:self.coordinator.ingest_observation(from_ros(message),time.monotonic())

    def start(self,request,response):
        result=self.coordinator.start(request.scenario_id,time.monotonic())
        response.accepted,response.task_id,response.reason=result.accepted,result.task_id,result.reason
        return response

    def pause(self,request,response):
        for command in self.coordinator.pause('explicit local pause'):self.command_pub.publish(to_ros(command))
        response.success,response.message=True,'paused; stop confirmation reported by agents'
        return response

    def tick(self):
        for command in self.coordinator.tick(time.monotonic()):self.command_pub.publish(to_ros(command))
        message=String()
        message.data=json.dumps({'task_id':self.coordinator.task_id,'phase':self.coordinator.phase,
            'reason':self.coordinator.reason,'held_resources':list(self.coordinator.resources.held),
            'online_robots':list(self.coordinator.states),
            'bound_c':self.coordinator.c_observation})
        self.status_pub.publish(message)

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=CoordinatorNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            for command in node.coordinator.pause('coordinator shutdown'):node.command_pub.publish(to_ros(command))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
