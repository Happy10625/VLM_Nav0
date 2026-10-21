"""Real generated interfaces, coordinator + two fake processes; localhost Domain31 only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import rclpy
from rclpy.node import Node
from multi_agent_task_msgs.srv import StartTask
from multi_agent_task_msgs.msg import AgentState,Observation
from std_msgs.msg import String
from ament_index_python.packages import get_package_prefix
from multi_agent_nav.node_common import qos

def test_dual_fake_ros_mission_and_topics(tmp_path,monkeypatch):
    prefix='/multi_agent/mock/run_'+uuid.uuid4().hex
    for name,value in {'ROS_DOMAIN_ID':'31','ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_LOG_DIR':str(tmp_path)}.items():
        monkeypatch.setenv(name,value)
    monkeypatch.delenv('CYCLONEDDS_URI',raising=False)
    root=Path(get_package_prefix('multi_agent_nav'))/'lib/multi_agent_nav'
    processes=[];logs=[]
    rclpy.init(args=[])
    probe=Node('cooperative_smoke_'+uuid.uuid4().hex[:8])
    phases=[];states={};observations=[]
    probe.create_subscription(String,prefix+'/coordinator/status',lambda m:phases.append(json.loads(m.data)),qos())
    for robot in ('go2','ranger'):
        probe.create_subscription(AgentState,prefix+f'/{robot}/state',lambda m:states.update({m.robot_id:m}),qos())
        probe.create_subscription(Observation,prefix+f'/{robot}/observation',observations.append,qos(True))
    start=probe.create_client(StartTask,prefix+'/coordinator/start')
    def spin_until(predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return True
            if any(p.poll() is not None for p in processes):return False
            rclpy.spin_once(probe,timeout_sec=.05)
        return bool(predicate())
    try:
        for role in ('coordinator','go2','ranger'):
            log=(tmp_path/f'{role}.log').open('w');logs.append(log)
            exe='multi_agent_coordinator_node' if role=='coordinator' else 'multi_agent_agent_node'
            args=[str(root/exe),'--ros-args','-r',f'__node:=smoke_{role}_{prefix.rsplit("/",1)[-1]}',
                  '-p','simulation:=true','-p',f'topic_prefix:={prefix}']
            if role!='coordinator':args+=['-p',f'robot_id:={role}']
            processes.append(subprocess.Popen(args,env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT))
        assert start.wait_for_service(timeout_sec=10)
        assert spin_until(lambda:len(states)==2 and all(s.nav_ready for s in states.values()) and
                         phases and len(phases[-1]['online_robots'])==2,10)
        assert all(not s.ready and not s.deposited and not s.command_id for s in states.values())
        request=StartTask.Request();request.scenario_id='synthetic'
        future=start.call_async(request)
        assert spin_until(future.done,5)
        assert future.result().accepted, future.result().reason
        assert spin_until(lambda:any(p['phase']=='DONE' for p in phases),35), phases[-5:]
        assert all(s.simulation and s.ready and s.deposited and s.exited for s in states.values())
        assert {o.target_id for o in observations}==set('ABC')
        assert all(o.task_id==future.result().task_id for o in observations)
        assert not phases[-1]['held_resources']
        # Query these fake nodes; a recently destroyed observer elsewhere in this
        # test process can leave temporary discovery entries in the domain graph.
        for role in ('go2','ranger'):
            node_name=f'smoke_{role}_{prefix.rsplit("/",1)[-1]}'
            endpoints=probe.get_publisher_names_and_types_by_node(node_name,'/')+probe.get_subscriber_names_and_types_by_node(node_name,'/')
            names=[name for name,_ in endpoints]
            assert '/cmd_vel' not in names and '/cmd_vel_bridge' not in names
            assert not any('/_action/' in name or name.startswith('/camera/') or name=='/tf' for name in names)
    finally:
        for process in processes:
            if process.poll() is None:process.send_signal(signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for log in logs:log.close()
        for role in ('coordinator','go2','ranger'):
            path=tmp_path/f'{role}.log'
            if path.exists():print(role+': '+path.read_text()[-2000:])
        probe.destroy_node();rclpy.shutdown()
