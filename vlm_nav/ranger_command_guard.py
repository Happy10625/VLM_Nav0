"""Ranger command passthrough and dead-man stop; deliberately no motion smoothing."""
import math
import time

from geometry_msgs.msg import Twist


class CommandGuard:
    def __init__(self,publish,timeout=1.,clock=time.monotonic,*,odom_timeout=.5):
        if not all(math.isfinite(v) and v>0 for v in (timeout,odom_timeout)):
            raise ValueError('watchdog timeouts must be finite and positive')
        self.publish,self.timeout,self.clock=publish,timeout,clock
        self.odom_timeout=odom_timeout
        self.received_at=None
        self.odometry_at=None
        self.active=True

    def stop(self):
        self.received_at=None
        self.publish(Twist())

    def set_active(self,active):
        self.active=active
        if not active:self.stop()

    def update_odometry(self,message):
        velocity=message.twist.twist
        linear=math.hypot(velocity.linear.x,velocity.linear.y)
        angular=abs(velocity.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.odometry_at=None;self.stop();return
        self.odometry_at=self.clock()

    def odometry_fresh(self):
        return self.odometry_at is not None and self.clock()-self.odometry_at<=self.odom_timeout

    def receive(self,message):
        values=[getattr(vector,axis) for vector in (message.linear,message.angular) for axis in 'xyz']
        if (not all(math.isfinite(v) for v in values) or
                message.linear.y!=0 or message.linear.z!=0 or
                message.angular.x!=0 or message.angular.y!=0 or
                message.linear.x!=0 and abs(message.angular.z)>=.3):
            self.stop()
            return
        if message.linear.x==0 and message.angular.z==0:
            self.stop();return
        if not self.active or not self.odometry_fresh():
            self.stop();return
        self.received_at=self.clock()
        self.publish(message)

    def tick(self):
        if self.received_at is not None and (self.clock()-self.received_at>self.timeout or not self.odometry_fresh()):
            self.stop()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.signals import SignalHandlerOptions
    from lifecycle_msgs.msg import State,TransitionEvent
    from nav_msgs.msg import Odometry
    import signal
    def stop_signal(_number,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop_signal)
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=Node('ranger_command_guard')
    node.declare_parameter('command_timeout',1.)
    node.declare_parameter('odom_timeout',.5)
    node.declare_parameter('odom_topic','/odom')
    publisher=node.create_publisher(Twist,'/cmd_vel',1)
    guard=CommandGuard(publisher.publish,node.get_parameter('command_timeout').value,
        odom_timeout=node.get_parameter('odom_timeout').value)
    node.create_subscription(Twist,'/cmd_vel_nav',guard.receive,1)
    node.create_subscription(Odometry,node.get_parameter('odom_topic').value,guard.update_odometry,1)
    node.create_timer(.05,guard.tick)
    node.create_subscription(TransitionEvent,'/controller_server/transition_event',
        lambda event:guard.set_active(event.goal_state.id==State.PRIMARY_STATE_ACTIVE),10)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            guard.stop()
            publisher.wait_for_all_acked(Duration(seconds=.2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
