"""Effective Nav2 parameters and Domain31 synthetic costmaps; no hardware."""
from pathlib import Path
import importlib.util
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext

ROOT=Path(__file__).resolve().parents[4]
COOP=ROOT/'cooperative_ws/src/multi_agent_nav'

def load_launch(path):
    spec=importlib.util.spec_from_file_location('costmap_launch_check',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def test_effective_cooperative_costmaps_keep_dynamic_layers_and_other_parameters():
    from vlm_nav.go2_nav2_config import load_nav2_parameters
    default=Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml'
    args=(default,ROOT/'config/nav2_go2.yaml',ROOT/'config/nav2_go2_static_map.yaml')
    legacy=load_nav2_parameters(*args)
    effective=load_nav2_parameters(*args,COOP/'config/nav2_go2_cooperative_costmap.yaml')
    for name,layer in [('global_costmap','obstacle_layer'),('local_costmap','voxel_layer')]:
        assert legacy[name][name]['ros__parameters']['plugins']==['static_layer','inflation_layer']
        p=effective[name][name]['ros__parameters']
        assert p['plugins']==['static_layer',layer,'inflation_layer']
        assert p['always_send_full_costmap'] is True
        cloud=p[layer]['go2_cloud']
        assert p[layer]['enabled'] and p[layer]['observation_sources']=='go2_cloud'
        assert cloud['topic']=='/vlm_nav/obstacle_cloud' and cloud['marking'] and cloud['clearing']
        for field in ['global_frame','robot_base_frame','footprint','footprint_padding','rolling_window']:
            assert p.get(field)==legacy[name][name]['ros__parameters'].get(field)
    for data in (legacy,effective):
        data.pop('global_costmap');data.pop('local_costmap')
    assert legacy==effective  # AMCL, map_server, scan input and navigation untouched.

def test_actual_navigation_launch_applies_optional_overlay_last(monkeypatch):
    module=load_launch(ROOT/'launch/go2_navigation.launch.py')
    real=module.get_package_share_directory
    monkeypatch.setattr(module,'get_package_share_directory',lambda p:str(ROOT) if p=='vlm_nav' else real(p))
    context=LaunchContext();context.launch_configurations.update(localization_mode='amcl',
        overrides_file=str(ROOT/'config/nav2_go2.yaml'),amcl_scan_topic='/scan_amcl',
        costmap_overrides_file=str(COOP/'config/nav2_go2_cooperative_costmap.yaml'))
    actions=module.configure_go2_navigation(context)
    parameter_file=actions[0]._Node__parameters[0].evaluate(context)
    params=yaml.safe_load(Path(str(parameter_file)).read_text())
    assert params['local_costmap']['local_costmap']['ros__parameters']['plugins']==['static_layer','voxel_layer','inflation_layer']
    assert params['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] is False

def test_isolated_acceptance_launch_has_only_costmaps_and_lifecycle_manager(monkeypatch):
    module=load_launch(COOP/'launch/go2_costmap_acceptance.launch.py')
    real=module.get_package_share_directory
    monkeypatch.setattr(module,'get_package_share_directory',lambda p:str(COOP) if p=='multi_agent_nav' else real(p))
    actions=module.build(LaunchContext())
    host,manager=actions
    assert host.node_executable=='multi_agent_costmap_acceptance'
    p=yaml.safe_load(host._Node__parameters[0].evaluate(LaunchContext()).read_text())
    assert set(p)=={'/go2_costmap_acceptance/'+n+'/'+n for n in ('local_costmap','global_costmap')}
    assert manager._Node__node_name=='lifecycle_manager_costmaps'

def test_no_verification_flags_are_set_by_static_costmap_profile():
    config=yaml.safe_load((COOP/'config/robot_go2.yaml').read_text())
    assert config['costmap_source_verified'] is False and config['peer_obstacle_verified'] is False
    assert config['path_source_verified'] is False and config['stop_margin'] is None

def test_static_collection_dry_run_contains_only_read_commands():
    import subprocess
    script=ROOT/'scripts/collect_go2_cooperative_static.sh'
    result=subprocess.run(['bash',str(script),'--dry-run','--phase','mark'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert 'param dump /amcl' in result.stdout and 'bag record' in result.stdout
    assert 'send_goal' not in result.stdout and 'topic pub' not in result.stdout and '--allow-motion' not in result.stdout

@pytest.mark.parametrize('orderly_shutdown',[False,True],ids=['sigint','lifecycle_shutdown'])
def test_native_costmap_marking_and_clearing_with_synthetic_inputs_only(tmp_path,monkeypatch,orderly_shutdown):
    """Real Nav2, isolated Domain31; never field verification or a fake task peer."""
    import os,signal,subprocess,time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import Costmap
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import PointCloud2,PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from rcl_interfaces.srv import GetParameters
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from ament_index_python.packages import get_package_prefix
    for k,v in {'ROS_DOMAIN_ID':'31','ROS_LOCALHOST_ONLY':'1',
        'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_LOG_DIR':str(tmp_path)}.items():monkeypatch.setenv(k,v)
    monkeypatch.delenv('CYCLONEDDS_URI',raising=False)
    module=load_launch(COOP/'launch/go2_costmap_acceptance.launch.py')
    actions=module.build(LaunchContext())
    params=actions[0]._Node__parameters[0].evaluate(LaunchContext())
    native=Path(get_package_prefix('multi_agent_nav'))/'lib/multi_agent_nav/multi_agent_costmap_acceptance'
    manager=Path(get_package_prefix('nav2_lifecycle_manager'))/'lib/nav2_lifecycle_manager/lifecycle_manager'
    prefix='/go2_costmap_acceptance';names=[prefix+'/'+n+'/'+n for n in ('local_costmap','global_costmap')]
    processes=[];logs=[];latest={};forced=[]
    rclpy.init(args=[]);probe=Node('synthetic_costmap_probe')
    durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps=probe.create_publisher(OccupancyGrid,'/map',durable)
    transforms=probe.create_publisher(TFMessage,'/tf_static',durable)
    cloud=probe.create_publisher(PointCloud2,'/vlm_nav/obstacle_cloud',10)
    for n in ('local_costmap','global_costmap'):
        probe.create_subscription(Costmap,prefix+'/'+n+'/costmap_raw',lambda m,n=n:latest.update({n:m}),durable)
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
    grid.info.width=grid.info.height=120;grid.info.origin.position.x=grid.info.origin.position.y=-3.
    grid.info.origin.orientation.w=1.;grid.data=[0]*14400
    fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
    def publish(x):
        stamp=probe.get_clock().now().to_msg();grid.header.stamp=stamp;maps.publish(grid)
        ts=[]
        for parent,child in [('map','odom'),('odom','base_link')]:
            t=TransformStamped();t.header.frame_id=parent;t.header.stamp=stamp;t.child_frame_id=child
            t.transform.rotation.w=1.;ts.append(t)
        transforms.publish(TFMessage(transforms=ts))
        # After removal, a farther return on the SAME ray proves raytrace clearing.
        cloud.publish(point_cloud2.create_cloud(Header(stamp=stamp,frame_id='base_link'),fields,[(x,x*.125,0.)]))
    def spin_until(predicate,x,timeout=15):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            publish(x);rclpy.spin_once(probe,timeout_sec=.1)
            if predicate():return True
            if any(p.poll() is not None for p in processes):return False
        return bool(predicate())
    def old_obstacle_cost(m):
        md=m.metadata
        ix=int((.8-md.origin.position.x)/md.resolution);iy=int((.1-md.origin.position.y)/md.resolution)
        return max(m.data[y*md.size_x+x] for y in range(iy-1,iy+2) for x in range(ix-1,ix+2))
    try:
        commands=[[str(native),'--ros-args','--params-file',str(params)],
            [str(manager),'--ros-args','-r','__ns:='+prefix,'-r','__node:=lifecycle_manager_costmaps',
             '-p','autostart:=true','-p','bond_timeout:=0.0','-p','node_names:='+str(names)]]
        for i,command in enumerate(commands):
            log=(tmp_path/f'native_{i}.log').open('w');logs.append(log)
            processes.append(subprocess.Popen(command,env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT))
        assert spin_until(lambda:len(latest)==2 and all(old_obstacle_cost(m)>=253 for m in latest.values()),.8)
        for name in names:
            client=probe.create_client(GetParameters,name+'/get_parameters')
            assert client.wait_for_service(timeout_sec=2)
            future=client.call_async(GetParameters.Request(names=['plugins','always_send_full_costmap']))
            assert spin_until(future.done,.8,3)
            plugins=future.result().values[0].string_array_value
            assert ('voxel_layer' if 'local_costmap' in name else 'obstacle_layer') in plugins
            assert future.result().values[1].bool_value
        assert spin_until(lambda:len(latest)==2 and all(old_obstacle_cost(m)==0 for m in latest.values()),3.), {n:old_obstacle_cost(m) for n,m in latest.items()}
        for short in ('local_costmap','global_costmap'):
            publishers=probe.get_publisher_names_and_types_by_node(short,prefix+'/'+short)
            assert not any('cmd_vel' in topic or '/_action/' in topic for topic,_ in publishers)
        if orderly_shutdown:
            client=probe.create_client(ManageLifecycleNodes,prefix+'/lifecycle_manager_costmaps/manage_nodes')
            assert client.wait_for_service(timeout_sec=2)
            future=client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN))
            assert spin_until(future.done,3.,10) and future.result().success
            for name in names:
                client=probe.create_client(GetState,name+'/get_state')
                assert client.wait_for_service(timeout_sec=2)
                future=client.call_async(GetState.Request())
                assert spin_until(future.done,3.,3)
                assert future.result().current_state.id==4  # FINALIZED, not just manager success.
        print('Domain31 synthetic native costmaps: plugins/full payload/marking/clearing/no-motion publishers PASS')
    finally:
        for p in processes:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:forced.append(p.pid);p.kill();p.wait()
        for log in logs:log.close()
        for p in tmp_path.glob('native_*.log'):print(p.name+': '+p.read_text()[-2000:])
        probe.destroy_node();rclpy.shutdown()
    assert not forced, f'Forced shutdown: {forced}'
    assert [p.returncode for p in processes]==[0,0], 'Costmap host/manager must both exit normally'
