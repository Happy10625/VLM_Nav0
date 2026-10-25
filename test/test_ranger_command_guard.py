"""Final Ranger command contract; no hardware or model requests."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from vlm_nav.ranger_command_guard import CommandGuard


def command(v=0., w=0.):
    msg=Twist();msg.linear.x=v;msg.angular.z=w
    return msg


def running_guard(publish,clock):
    guard=CommandGuard(publish,1.,clock)
    guard.update_odometry(Odometry())
    return guard


def test_passthrough_zero_and_timeout_never_replay_old_motion():
    now=[0.];sent=[]
    guard=running_guard(sent.append,lambda:now[0])
    guard.receive(command(.2));assert sent[-1].linear.x==.2
    now[0]=.99;guard.update_odometry(Odometry());guard.tick();assert len(sent)==1
    now[0]=1.01;guard.tick();assert sent[-1]==Twist()
    guard.tick();assert len(sent)==2
    guard.receive(command(w=.4));assert sent[-1].angular.z==.4
    guard.receive(Twist());assert sent[-1]==Twist()
    now[0]=10.;guard.tick();assert sent[-1]==Twist()


@pytest.mark.parametrize('msg',[command(.1,.3),command(.1,-.3),command(.1,.4),command(float('nan')),command(w=float('inf'))])
def test_overspeed_mixed_or_nonfinite_commands_stop_immediately(msg):
    sent=[];guard=running_guard(sent.append,lambda:0.)
    guard.receive(command(.1));guard.receive(msg)
    assert sent[-1]==Twist()


def test_transition_between_axes_is_not_interpolated_by_guard():
    sent=[];guard=running_guard(sent.append,lambda:0.)
    for msg in (command(.2),command(.1),Twist(),command(w=.1),command(w=.2)):
        guard.receive(msg)
    assert [(m.linear.x,m.angular.z) for m in sent]==[(.2,0.),(.1,0.),(0.,0.),(0.,.1),(0.,.2)]
    guard.stop();assert sent[-1]==Twist()


def test_ranger_launch_has_one_acceleration_owner_and_guarded_behavior_output():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('ranger_navigation',root/'launch/navigation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    context.launch_configurations.update(overrides_file=str(root/'config/nav2_ranger_local.yaml'))
    actions=module.configure_navigation(context)
    nodes=[a for a in actions if isinstance(a,Node)]
    assert not any(n.node_executable=='velocity_smoother' for n in nodes)
    guard=next(n for n in nodes if n.node_executable=='ranger_command_guard')
    assert guard.node_package=='vlm_nav'
    for name in ('controller_server','behavior_server'):
        node=next(n for n in nodes if n.node_executable==name)
        node._perform_substitutions(context)
        assert ('cmd_vel','cmd_vel_nav') in node.expanded_remapping_rules
    manager=next(n for n in nodes if n.node_executable=='lifecycle_manager')
    manager._perform_substitutions(context)
    parameters=''.join(Path(path).read_text() for path,is_file in manager._Node__expanded_parameter_arguments if is_file)
    assert 'controller_server' in parameters and 'velocity_smoother' not in parameters


def test_profile_keeps_arrival_tolerance_and_footprint_collision_checks():
    root=Path(__file__).resolve().parents[1]
    profile=yaml.safe_load((root/'config/nav2_ranger_local.yaml').read_text())
    controller=profile['controller_server']['ros__parameters']
    follow=controller['FollowPath']
    assert follow['plugin']=='vlm_nav::RangerDWBController'
    assert follow['trajectory_generator_name']=='vlm_nav::RangerTrajectoryGenerator'
    assert controller['general_goal_checker']['xy_goal_tolerance']==follow['xy_goal_tolerance']==.05
    assert follow['critics'][0]=='ObstacleFootprint'
    assert not {'Oscillation','RotateToGoal'} & set(follow['critics'])
    assert 0<follow['max_moving_angular_velocity']<.3
    assert follow['linear_granularity']<=.01 and follow['angular_granularity']<=.01


def test_inactive_latch_rejects_late_nonzero_until_active():
    sent=[];guard=CommandGuard(sent.append,1.,lambda:0.)
    guard.set_active(False);guard.receive(command(.1))
    assert sent[-1]==Twist()


def test_mixed_motion_and_axis_changes_pass_without_zero_boundary():
    sent=[];guard=running_guard(sent.append,lambda:0.)
    odometry=Odometry();odometry.twist.twist.linear.x=.1
    odometry.twist.twist.angular.z=.2;guard.update_odometry(odometry)
    messages=[command(.1),command(.1,.299),command(-.1,-.299),command(w=.4),command(.1,.2)]
    for message in messages:guard.receive(message)
    assert sent==messages


def test_missing_stale_or_invalid_odometry_stops_even_with_commands():
    now=[0.];sent=[];guard=CommandGuard(sent.append,1.,lambda:now[0])
    guard.receive(command(.1));assert sent[-1]==Twist()
    guard.update_odometry(Odometry());guard.receive(command(.1))
    assert sent[-1].linear.x==.1
    now[0]=.51;guard.tick();assert sent[-1]==Twist()
    odometry=Odometry();odometry.twist.twist.angular.z=float('nan')
    guard.update_odometry(odometry);guard.receive(command(w=.2));assert sent[-1]==Twist()


@pytest.mark.parametrize('shutdown_signal',['SIGINT','SIGTERM'])
def test_ros_final_command_topic_zero_timeout_lifecycle_and_shutdown(tmp_path,shutdown_signal):
    import os
    import signal
    import subprocess
    import sys
    import time
    import rclpy
    from lifecycle_msgs.msg import State,TransitionEvent
    if os.getenv('ROS_DOMAIN_ID')!='31' or os.getenv('ROS_LOCALHOST_ONLY')!='1':
        pytest.skip('requires the isolated Domain31 / localhost test entry')
    context=rclpy.context.Context();context.init(args=[])
    node=rclpy.create_node('ranger_guard_acceptance',context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context);executor.add_node(node)
    messages=[]
    node.create_subscription(Twist,'/cmd_vel',messages.append,10)
    publisher=node.create_publisher(Twist,'/cmd_vel_nav',1)
    transitions=node.create_publisher(TransitionEvent,'/controller_server/transition_event',10)
    odometry=node.create_publisher(Odometry,'/odom',1)
    node.create_timer(.05,lambda:odometry.publish(Odometry()))
    root=Path(__file__).resolve().parents[1]
    log=(tmp_path/'guard.log').open('w')
    process=subprocess.Popen([sys.executable,str(root/'scripts/ranger_command_guard'),
        '--ros-args','-p','command_timeout:=0.2'],stdout=log,stderr=log)
    def wait_for(predicate,timeout=5.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():return
        log.flush()
        pytest.fail('isolated command contract timed out: '+(tmp_path/'guard.log').read_text())
    def send(msg):
        before=len(messages);publisher.publish(msg)
        wait_for(lambda:len(messages)>before and messages[-1]==msg)
    try:
        wait_for(lambda:publisher.get_subscription_count()>0 and transitions.get_subscription_count()>0
                 and odometry.get_subscription_count()>0)
        for _ in range(5):executor.spin_once(timeout_sec=.05)
        send(command(.2,.29));send(command(.1,-.29));send(Twist());send(command(w=.4))
        wait_for(lambda:messages[-1]==Twist())
        send(command(.1));before=len(messages)
        transitions.publish(TransitionEvent(goal_state=State(id=State.PRIMARY_STATE_INACTIVE)))
        wait_for(lambda:len(messages)>before and messages[-1]==Twist())
        before=len(messages);publisher.publish(command(.1))
        wait_for(lambda:len(messages)>before and messages[-1]==Twist())
        transitions.publish(TransitionEvent(goal_state=State(id=State.PRIMARY_STATE_ACTIVE)))
        for _ in range(5):executor.spin_once(timeout_sec=.02)
        send(command(w=.2));before=len(messages);process.send_signal(getattr(signal,shutdown_signal))
        wait_for(lambda:len(messages)>before and messages[-1]==Twist())
        assert all(m.linear.x==0 or abs(m.angular.z)<.3 for m in messages)
    finally:
        if process.poll() is None:process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();executor.shutdown();node.destroy_node();context.shutdown()


def test_real_follow_path_arrival_cancel_deactivate_and_command_loss(tmp_path):
    import math
    import os
    import signal
    import subprocess
    import sys
    import time
    import rclpy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import FollowPath
    if os.getenv('ROS_DOMAIN_ID')!='31' or os.getenv('ROS_LOCALHOST_ONLY')!='1':
        pytest.skip('requires isolated Domain31 / localhost')
    root=Path(__file__).resolve().parents[1]
    controller=yaml.safe_load(Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml').read_text())['controller_server']['ros__parameters']
    overlay=yaml.safe_load((root/'config/nav2_ranger_local.yaml').read_text())['controller_server']['ros__parameters']
    for key,value in overlay.items():
        if isinstance(value,dict):controller.setdefault(key,{}).update(value)
        else:controller[key]=value
    controller.update(controller_frequency=20.,controller_plugins=['FollowPath'],
        progress_checker_plugin='progress_checker',odom_topic='/odom',
        progress_checker=dict(plugin='vlm_nav::RangerProgressChecker',
                              required_movement_radius=.02,movement_time_allowance=2.))
    params=tmp_path/'controller.yaml'
    params.write_text(yaml.safe_dump(dict(controller_server=dict(ros__parameters=controller),
        local_costmap=dict(local_costmap=dict(ros__parameters=dict(
            global_frame='odom',robot_base_frame='base_link',rolling_window=True,
            width=8,height=8,resolution=.025,update_frequency=20.,publish_frequency=5.,
            track_unknown_space=False,plugins=['obstacle_layer'],
            obstacle_layer=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=False),footprint_padding=.03,
            footprint='[[0.36,0.25],[0.36,-0.25],[-0.36,-0.25],[-0.36,0.25]]'))))))
    context=rclpy.context.Context();context.init(args=[])
    node=rclpy.create_node('ranger_follow_path_acceptance',context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context);executor.add_node(node)
    received=[];state=dict(x=0.,y=0.,yaw=0.,v=0.,w=0.,frozen=False,at=time.monotonic())
    def velocity(msg):
        received.append(msg)
        linear=msg.linear.x
        if abs(msg.angular.z)>=1e-6 and abs(linear/msg.angular.z)<.47644:linear=0.
        state.update(v=0. if state['frozen'] else linear,w=0. if state['frozen'] else msg.angular.z)
    node.create_subscription(Twist,'/cmd_vel',velocity,10)
    odom=node.create_publisher(Odometry,'/odom',1)
    tf=node.create_publisher(TFMessage,'/tf',10)
    raw=node.create_publisher(Twist,'/cmd_vel_nav',1)
    def simulate():
        now=time.monotonic();dt=now-state['at'];state['at']=now
        state['x']+=state['v']*math.cos(state['yaw'])*dt
        state['y']+=state['v']*math.sin(state['yaw'])*dt
        state['yaw']+=state['w']*dt
        transform=TransformStamped();transform.header.frame_id='odom';transform.child_frame_id='base_link'
        transform.header.stamp=node.get_clock().now().to_msg()
        transform.transform.translation.x=state['x'];transform.transform.translation.y=state['y']
        transform.transform.rotation.z=math.sin(state['yaw']/2);transform.transform.rotation.w=math.cos(state['yaw']/2)
        tf.publish(TFMessage(transforms=[transform]))
        message=Odometry();message.header=transform.header;message.child_frame_id='base_link'
        message.twist.twist.linear.x=state['v'];message.twist.twist.angular.z=state['w'];odom.publish(message)
    node.create_timer(.02,simulate)
    log=(tmp_path/'follow_path.log').open('w')
    processes=[subprocess.Popen([sys.executable,str(root/'scripts/ranger_command_guard')],stdout=log,stderr=log),
               subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server','--ros-args',
                   '--params-file',str(params),'-r','cmd_vel:=cmd_vel_nav'],stdout=log,stderr=log)]
    lifecycle=node.create_client(ChangeState,'/controller_server/change_state')
    client=ActionClient(node,FollowPath,'/follow_path')
    def wait_for(predicate,timeout=10.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():return
        log.flush();pytest.fail('FollowPath acceptance timed out: '+(tmp_path/'follow_path.log').read_text())
    def transition(value):
        future=lifecycle.call_async(ChangeState.Request(transition=Transition(id=value)))
        wait_for(future.done);assert future.result().success
    def goal(x,y):
        request=FollowPath.Goal();request.controller_id='FollowPath';request.goal_checker_id='general_goal_checker'
        request.path.header.frame_id='odom'
        for i in range(31):
            pose=PoseStamped();pose.header=request.path.header;pose.pose.orientation.w=1.
            pose.pose.position.x=state['x']+(x-state['x'])*i/30
            pose.pose.position.y=state['y']+(y-state['y'])*i/30
            request.path.poses.append(pose)
        future=client.send_goal_async(request);wait_for(future.done)
        handle=future.result();assert handle.accepted;return handle
    try:
        wait_for(lifecycle.service_is_ready)
        transition(Transition.TRANSITION_CONFIGURE);transition(Transition.TRANSITION_ACTIVATE)
        wait_for(client.server_is_ready)
        handle=goal(.5,.1);result=handle.get_result_async();wait_for(result.done)
        assert result.result().status==4,(tmp_path/'follow_path.log').read_text()
        wait_for(lambda:received and received[-1]==Twist())
        assert math.hypot(state['x']-.5,state['y']-.1)<=.05
        assert any(m.linear.x!=0 and m.angular.z!=0 for m in received)
        handle=goal(state['x'],state['y']+1.)
        wait_for(lambda:received[-1].angular.z>0)
        cancelled=handle.cancel_goal_async();wait_for(cancelled.done)
        result=handle.get_result_async();wait_for(result.done);assert result.result().status==5
        wait_for(lambda:received[-1]==Twist())
        state['frozen']=True
        handle=goal(state['x']+1.,state['y']);result=handle.get_result_async();wait_for(result.done)
        assert result.result().status==6
        wait_for(lambda:received[-1]==Twist())
        from rcl_interfaces.srv import GetParameters
        parameters=node.create_client(GetParameters,'/controller_server/get_parameters')
        request=parameters.call_async(GetParameters.Request(names=['ranger_recovery.reason']))
        wait_for(request.done);assert request.result().values[0].string_value=='no_progress'
        state['frozen']=False
        transition(Transition.TRANSITION_DEACTIVATE)
        before=len(received);raw.publish(command(.1))
        wait_for(lambda:len(received)>before and received[-1]==Twist())
        transition(Transition.TRANSITION_ACTIVATE)
        goal(state['x']+1.,state['y']);wait_for(lambda:received[-1].linear.x>0)
        before=len(received);processes[1].kill();processes[1].wait(timeout=5)
        wait_for(lambda:len(received)>before and received[-1]==Twist(),timeout=2.)
        assert all(m.linear.x==0 or abs(m.angular.z)<.3 for m in received)
    finally:
        for process in processes:
            if process.poll() is None:process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();executor.shutdown();node.destroy_node();context.shutdown()
