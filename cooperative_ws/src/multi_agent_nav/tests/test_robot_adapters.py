"""Platform boundaries exercised without hardware or provider calls."""
from concurrent.futures import Future
from types import SimpleNamespace as S
import math
import numpy as np
import pytest
from multi_agent_nav.adapters.go2 import ABCClient, observations_from_results
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.models import Pose2D
from vlm_nav.models import VLMResult


def test_single_scan_keeps_ranger_target_id_and_description():
    client=ABCClient.__new__(ABCClient);client.mode='single'
    descriptions=[]
    def infer(rgb,description):
        descriptions.append(description)
        return VLMResult(False,0.,None,None,False,False,False)
    client.infer=infer
    results=client.infer_abc(np.zeros((5,5,3),dtype=np.uint8),{'B':'水杯'})
    assert list(results)==['B'] and descriptions==['水杯']


def test_local_observation_carries_frame_identity():
    frame=FrameAdapter('ranger-map','sha256:123','ranger_local_map','local_only',[0,0,0])
    snapshot=S(stamp=S(sec=1,nanosec=0))
    observation=observations_from_results({'B':('NOT_DETECTED',None)},snapshot,None,frame,'scan',0,targets={'B':'杯'})[0]
    assert (observation.local_map_id,observation.map_version,observation.common_frame_id,observation.alignment_id)==frame.ids
    assert not observation.simulation


def test_ranger_readiness_needs_no_additional_permission_subscription(monkeypatch):
    from multi_agent_nav.adapters.nav2 import Nav2Adapter
    from multi_agent_nav.adapters.ranger import RangerAdapter
    def initialize(adapter,node,config,frame):
        adapter.node,adapter.config=node,config
    monkeypatch.setattr(Nav2Adapter,'__init__',initialize)
    def unexpected_subscription(*args):
        pytest.fail('Ranger must not subscribe to an extra motion-permission topic')
    a=RangerAdapter(S(create_subscription=unexpected_subscription),{},None)
    assert a.readiness_checks(10.)=={}


def test_factory_rejects_unknown_robot_before_ros_import():
    from multi_agent_nav.adapters.factory import create_runtime
    with pytest.raises(ValueError,match='robot'):
        create_runtime('unknown',{})


def test_ranger_accepts_recorded_three_second_reply_and_allows_evidence_finalization(monkeypatch):
    from multi_agent_nav.adapters.ranger import RangerAdapter
    from multi_agent_nav.config import load_config
    config=load_config()['robot_ranger']
    assert config['api_timeout']==15.
    now=[100.]
    monkeypatch.setattr('multi_agent_nav.adapters.vision.time.monotonic',lambda:now[0])
    adapter=RangerAdapter(S(create_subscription=lambda *args:None),config,None)
    try:
        assert adapter.vision.timeout==20.
        future=adapter.vision.submit(lambda:'DETECTED')
        adapter.vision.job.result(timeout=1.)
        now[0]=103.557
        adapter.vision.poll()
        assert future.result()=='DETECTED'
    finally:
        adapter.close(stop=False)


def test_ranger_eight_headings_are_relative_to_initial_and_require_fresh_frames():
    from multi_agent_nav.adapters.ranger import RangerAdapter
    from multi_agent_nav.models import Health
    from collections import deque
    a=RangerAdapter.__new__(RangerAdapter)
    a.config={'robot_id':'ranger','scan_angles':[i*math.pi/4 for i in range(1,9)],'api_timeout':3.}
    a.motions={};a.scan=None;a.motion_fault=False;a.results=deque();a.client=S()
    pose=[Pose2D(0,0,.37)];snap=[0]
    a.frame=FrameAdapter('r','v','ranger_local_map','local_only',[0,0,0])
    a.get_health=lambda:Health(pose=pose[0],localization_valid=True,nav_ready=True,stopped=True)
    a.node=S(spinner=S(),latest_snapshot=None,vlm_input_ready=lambda now:True)
    spins=[];frames=[]
    def motion(client,goal,*args):
        spins.append(goal.target_yaw)
        pose[0]=Pose2D(0,0,pose[0].yaw+goal.target_yaw)
        f=Future();f.set_result('SUCCEEDED');return 'spin',f
    a._motion=motion
    def infer(rgb,targets):
        frames.append(snap[0]);return {'B':('NOT_DETECTED',None)}
    a.client.infer_abc=infer
    class Vision:
        available=True
        def poll(self):pass
        def submit(self,fn):
            f=Future();f.set_result(fn());return f
    a.vision=Vision()
    a.node.ground_pixel_with_reason=lambda *args:(None,'')
    reply=a.start_scan({'B':'杯'},'scan')
    for _ in range(80):
        if reply.done():break
        if a.scan['phase']=='FRAME':
            # A pre-stop frame must not be consumed.
            a.node.latest_snapshot=S(captured_monotonic=a.scan['after']-1)
            a.tick();assert a.scan['phase']=='FRAME'
            snap[0]+=1
            a.node.latest_snapshot=S(captured_monotonic=a.scan['after']+1,
                stamp=S(sec=snap[0],nanosec=0),rgb=np.zeros((5,5,3),dtype=np.uint8))
        a.tick()
    assert reply.done() and not reply.cancelled()
    assert spins==pytest.approx([math.pi/4]*8)
    assert frames==list(range(1,9))
    assert [o.view_index for o in reply.result()]==list(range(8))
    assert all(o.target_id=='B' for o in reply.result())
    assert all(o.robot_id=='ranger' for o in reply.result())


def test_aligned_depth_observer_processes_disabled_input_without_autonomy(tmp_path,monkeypatch):
    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import TransformStamped
    from vlm_nav.vlm_navigator import VLMNavigator
    monkeypatch.setenv('ROS_DOMAIN_ID','31');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    rclpy.init(args=[])
    node=VLMNavigator(observation_only=True,node_name='ranger_test_observer',
        parameter_overrides=[Parameter('publish_stop_command',value=False)])
    try:
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='camera_color_optical_frame'
        tf.transform.rotation.w=1.
        node.tf_buffer.set_transform_static(tf,'test')
        info=CameraInfo();info.width=info.height=5;info.k=[10.,0.,2.,0.,10.,2.,0.,0.,1.]
        node.on_camera_info('color',info)
        rgb=Image();rgb.width=rgb.height=5;rgb.encoding='rgb8';rgb.step=15;rgb.data=bytes([50]*75)
        depth=Image();depth.width=depth.height=5;depth.encoding='16UC1';depth.step=10
        depth.data=np.full((5,5),1000,dtype=np.uint16).tobytes()
        for msg in (rgb,depth):
            msg.header.frame_id='camera_color_optical_frame';msg.header.stamp=node.get_clock().now().to_msg()
        node.on_rgbd(rgb,depth)
        assert node.latest_snapshot is not None
        assert node.vlm_input_ready() and node.depth_units_verified
        assert np.allclose(node.latest_snapshot.depth_m,1.)
        assert node.worker is None and node.stop_pub is None and len(list(node.timers))==1
    finally:
        node.destroy_node();rclpy.shutdown()


def test_ranger_original_scan_yaws_starts_at_45_degrees():
    from vlm_nav.geometry import scan_yaws
    assert list(scan_yaws(.2,8))==pytest.approx([.2+v for v in
        (.7853981633974483,1.5707963267948966,2.356194490192345,3.141592653589793,
         3.9269908169872414,4.71238898038469,5.497787143782138,6.283185307179586)])


def test_ranger_environment_uses_own_domain_without_go2_dds_requirements(monkeypatch):
    from multi_agent_nav.go2_check import require_ros_environment
    monkeypatch.setenv('ROS_DOMAIN_ID','0')
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    monkeypatch.delenv('GO2_NET_IFACE',raising=False);monkeypatch.delenv('CYCLONEDDS_URI',raising=False)
    require_ros_environment('ranger',{'robot_ranger':{'domain_id':0}})
    monkeypatch.setenv('ROS_DOMAIN_ID','40')
    with pytest.raises(ValueError,match='ROS_DOMAIN_ID'):require_ros_environment('ranger')


@pytest.mark.parametrize('fault',[None,'rgbd_ready','odom_fresh','local_costmap',
    'navigate_server','planner_server','spin_server','motion_fault_clear'])
def test_ranger_default_permission_preserves_shared_health_checks(fault):
    from test_go2_health_diagnostics import healthy_adapter
    from multi_agent_nav.adapters.ranger import RangerAdapter
    a=RangerAdapter.__new__(RangerAdapter);a.__dict__.update(healthy_adapter().__dict__)
    # Ranger must not inherit Go2 NAV_READY / CONTROL_ARMED requirements.
    a.node.nav_ready=a.node.control_armed=False
    a.node.external_gate_receipts={}
    if fault=='rgbd_ready':a.node.rgbd_sync_status['healthy']=False
    elif fault=='odom_fresh':a.odom_at=0.
    elif fault=='local_costmap':a.costmap=None
    elif fault=='motion_fault_clear':a.motion_fault=True
    elif fault:
        name={'navigate_server':'navigator','planner_server':'path_planner','spin_server':'spinner'}[fault]
        getattr(a.node,name).server_is_ready=lambda:False
    health=a.get_health()
    assert health.nav_ready is (fault is None)
    if fault:assert a.last_health_details['checks'][fault] is False
    else:assert health.localization_valid and health.stopped
