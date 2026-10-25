"""Native Nav2 recovery integration with synthetic ROS servers, TF and costmaps."""
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import yaml


@pytest.fixture
def ros():
    if os.getenv('ROS_DOMAIN_ID')!='31' or os.getenv('ROS_LOCALHOST_ONLY')!='1':
        pytest.skip('requires isolated Domain31 / localhost')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    context=rclpy.context.Context();context.init(args=[])
    node=rclpy.create_node('controller_server',context=context)
    executor=MultiThreadedExecutor(num_threads=4,context=context);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    yield node
    executor.shutdown();thread.join(timeout=5);node.destroy_node();context.shutdown()


def wait_for(predicate,log,timeout=8.):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.01)
    pytest.fail('synthetic recovery timeout:\n'+log.read_text())


def stop_process(process):
    if process.poll() is None:process.send_signal(signal.SIGINT)
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:process.kill();process.wait()


def test_navigation_launch_loads_recovery_plugins_and_tree(monkeypatch):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('recovery_launch',root/'launch/navigation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    context.launch_configurations['overrides_file']=str(root/'config/nav2_ranger_local.yaml')
    monkeypatch.setattr(module,'Node',lambda **kwargs:kwargs)
    nodes=module.configure_navigation(context)
    navigator=next(n for n in nodes if n['executable']=='bt_navigator')
    params=yaml.safe_load(Path(navigator['parameters'][0]).read_text())
    bt=params['bt_navigator']['ros__parameters']
    assert 'ranger_recovery_bt_node' in bt['plugin_lib_names']
    assert Path(bt['default_nav_to_pose_bt_xml']).is_file()
    assert params['controller_server']['ros__parameters']['progress_checker']['plugin']=='vlm_nav::RangerProgressChecker'
    assert params['controller_server']['ros__parameters']['failure_tolerance']==0.
    assert params['behavior_server']['ros__parameters']['spin']['plugin']=='vlm_nav::RangerSpin'


@pytest.mark.parametrize('case', ['no_progress','no_valid_trajectory','unrelated','spin_blocked','retry_failure','cancel','planner_failure'])
def test_native_navigator_recovers_once_only_for_classified_failures(ros,tmp_path,case):
    from rclpy.action import ActionClient,ActionServer,CancelResponse,GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import ComputePathToPose,FollowPath,NavigateToPose,Spin
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from tf2_msgs.msg import TFMessage
    root=Path(__file__).resolve().parents[1]
    tree=root/'behavior_trees/ranger_checked_turnaround.xml'
    assert tree.is_file()
    node=ros;node.declare_parameter('ranger_recovery.reason','')
    counts=dict(plan=0,follow=0,spin=0,spin_canceled=False);turns=[]
    group=ReentrantCallbackGroup()
    def plan(handle):
        counts['plan']+=1;result=ComputePathToPose.Result()
        result.path.header.frame_id='odom'
        for x in (0.,.5,1.):
            pose=PoseStamped();pose.header=result.path.header;pose.pose.position.x=x;pose.pose.orientation.w=1.
            result.path.poses.append(pose)
        if case=='planner_failure':handle.abort()
        else:handle.succeed()
        return result
    def follow(handle):
        counts['follow']+=1
        reason='' if case=='unrelated' else 'no_valid_trajectory' if case in ('no_valid_trajectory','spin_blocked') else 'no_progress'
        if counts['follow']==1 or case=='retry_failure':
            node.set_parameters([Parameter('ranger_recovery.reason',value=reason)]);handle.abort()
        else:
            node.set_parameters([Parameter('ranger_recovery.reason',value='')]);handle.succeed()
        return FollowPath.Result()
    def spin(handle):
        counts['spin']+=1;turns.append(handle.request)
        until=time.monotonic()+1.3
        while time.monotonic()<until or case=='cancel':
            if handle.is_cancel_requested:
                counts['spin_canceled']=True;handle.canceled();return Spin.Result()
            if not node.context.ok():return Spin.Result()
            time.sleep(.01)
        if case=='spin_blocked':handle.abort()
        else:handle.succeed()
        return Spin.Result()
    servers=[ActionServer(node,kind,name,callback,callback_group=group,
        goal_callback=lambda _:GoalResponse.ACCEPT,cancel_callback=lambda _:CancelResponse.ACCEPT)
        for kind,name,callback in [(ComputePathToPose,'compute_path_to_pose',plan),(FollowPath,'follow_path',follow),(Spin,'spin',spin)]]
    tf=node.create_publisher(TFMessage,'/tf',10);odom=node.create_publisher(Odometry,'/odom',10)
    def frames():
        pose=TransformStamped();pose.header.frame_id='odom';pose.child_frame_id='base_link'
        pose.header.stamp=node.get_clock().now().to_msg();pose.transform.rotation.w=1.
        tf.publish(TFMessage(transforms=[pose]));odom.publish(Odometry())
    timer=node.create_timer(.02,frames)
    params=yaml.safe_load(Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml').read_text())['bt_navigator']['ros__parameters']
    params.update(use_sim_time=False,global_frame='odom',default_nav_to_pose_bt_xml=str(tree),
        default_nav_through_poses_bt_xml=str(tree))
    params['plugin_lib_names'].append('ranger_recovery_bt_node')
    path=tmp_path/'navigator.yaml';path.write_text(yaml.safe_dump(dict(bt_navigator=dict(ros__parameters=params))))
    log=tmp_path/'navigator.log'
    with log.open('w') as stream:
        process=subprocess.Popen(['/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator','--ros-args','--params-file',str(path)],stdout=stream,stderr=stream)
        try:
            lifecycle=node.create_client(ChangeState,'/bt_navigator/change_state')
            wait_for(lifecycle.service_is_ready,log)
            for value in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
                future=lifecycle.call_async(ChangeState.Request(transition=Transition(id=value)))
                wait_for(future.done,log);assert future.result().success,log.read_text()
            client=ActionClient(node,NavigateToPose,'/navigate_to_pose');wait_for(client.server_is_ready,log)
            request=NavigateToPose.Goal();request.pose.header.frame_id='odom';request.pose.pose.position.x=1.;request.pose.pose.orientation.w=1.
            future=client.send_goal_async(request);wait_for(future.done,log);handle=future.result();assert handle.accepted
            result=handle.get_result_async()
            if case=='cancel':
                wait_for(lambda:counts['spin']==1,log)
                canceled=handle.cancel_goal_async();wait_for(canceled.done,log)
            wait_for(result.done,log)
            expected=4 if case in ('no_progress','no_valid_trajectory') else 5 if case=='cancel' else 6
            assert result.result().status==expected,log.read_text()
            assert counts['spin']==(0 if case in ('unrelated','planner_failure') else 1)
            assert counts['follow']==(0 if case=='planner_failure' else 2 if case in ('no_progress','no_valid_trajectory','retry_failure') else 1)
            if turns:
                assert turns[0].target_yaw==pytest.approx(math.pi)
                assert turns[0].time_allowance.sec==30
            if case in ('no_progress','no_valid_trajectory','retry_failure','spin_blocked'):
                assert counts['plan']>=2  # Replanning continues during a multi-second recovery.
            if case=='cancel':assert counts['spin_canceled']
        finally:
            stop_process(process);node.destroy_timer(timer)
            for server in servers:server.destroy()


@pytest.mark.parametrize('blocked',[True,False])
def test_native_spin_checks_full_rectangle_sweep_before_motion(ros,tmp_path,blocked):
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from geometry_msgs.msg import Twist,TransformStamped,PolygonStamped,Point32
    from nav2_msgs.action import Spin
    from nav2_msgs.msg import Costmap
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from tf2_msgs.msg import TFMessage
    root=Path(__file__).resolve().parents[1];node=ros
    params=yaml.safe_load((root/'config/nav2_ranger_local.yaml').read_text())['behavior_server']['ros__parameters']
    params.update(global_frame='odom',robot_base_frame='base_link',behavior_plugins=['spin'])
    # Speed up only the synthetic clocked plant; production behavior speed is unchanged.
    params.update(max_rotational_vel=2.,min_rotational_vel=.2,rotational_acc_lim=3.,cycle_frequency=50.)
    path=tmp_path/'behavior.yaml';path.write_text(yaml.safe_dump(dict(behavior_server=dict(ros__parameters=params))))
    state=dict(yaw=0.,w=0.,at=time.monotonic());messages=[]
    def velocity(msg):messages.append(msg);state['w']=msg.angular.z
    node.create_subscription(Twist,'/cmd_vel',velocity,10)
    tf=node.create_publisher(TFMessage,'/tf',10)
    grid_pub=node.create_publisher(Costmap,'/local_costmap/costmap_raw',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    footprint_pub=node.create_publisher(PolygonStamped,'/local_costmap/published_footprint',10)
    grid=Costmap();grid.header.frame_id='odom';grid.metadata.size_x=grid.metadata.size_y=200
    grid.metadata.resolution=.01;grid.metadata.origin.position.x=grid.metadata.origin.position.y=-1.
    grid.metadata.origin.orientation.w=1.;grid.data=[0]*40000
    if blocked:grid.data[100*200+145]=254  # Outside initial rectangle, hit by a rotating corner.
    def frames():
        now=time.monotonic();state['yaw']+=state['w']*(now-state['at']);state['at']=now
        transform=TransformStamped();transform.header.frame_id='odom';transform.child_frame_id='base_link'
        transform.header.stamp=node.get_clock().now().to_msg()
        transform.transform.rotation.z=math.sin(state['yaw']/2);transform.transform.rotation.w=math.cos(state['yaw']/2)
        tf.publish(TFMessage(transforms=[transform]));grid.header.stamp=transform.header.stamp
        grid.metadata.update_time=transform.header.stamp;grid_pub.publish(grid)
        footprint=PolygonStamped();footprint.header.stamp=transform.header.stamp;footprint.header.frame_id='base_link'
        footprint.polygon.points=[Point32(x=x,y=y) for x,y in ((.39,.28),(.39,-.28),(-.39,-.28),(-.39,.28))]
        footprint_pub.publish(footprint)
    timer=node.create_timer(.01,frames);log=tmp_path/'behavior.log'
    with log.open('w') as stream:
        process=subprocess.Popen(['/opt/ros/humble/lib/nav2_behaviors/behavior_server','--ros-args','--params-file',str(path)],stdout=stream,stderr=stream)
        try:
            lifecycle=node.create_client(ChangeState,'/behavior_server/change_state');wait_for(lifecycle.service_is_ready,log)
            for value in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
                future=lifecycle.call_async(ChangeState.Request(transition=Transition(id=value)))
                wait_for(future.done,log);assert future.result().success,log.read_text()
            client=ActionClient(node,Spin,'/spin');wait_for(client.server_is_ready,log)
            wait_for(lambda:grid_pub.get_subscription_count()>0 and footprint_pub.get_subscription_count()>0,log)
            time.sleep(.1)
            goal=Spin.Goal();goal.target_yaw=math.pi;goal.time_allowance.sec=10
            future=client.send_goal_async(goal);wait_for(future.done,log);handle=future.result();assert handle.accepted
            result=handle.get_result_async();wait_for(result.done,log)
            assert result.result().status==(6 if blocked else 4),log.read_text()
            wait_for(lambda:messages and messages[-1]==Twist(),log)
            if blocked:assert not any(m.angular.z!=0 for m in messages)
            else:assert state['yaw']>=math.pi-.02
        finally:stop_process(process);node.destroy_timer(timer)
