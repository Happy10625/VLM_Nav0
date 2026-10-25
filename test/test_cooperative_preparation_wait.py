"""Preparation sampler evidence tests without initializing ROS."""
import importlib.util
from dataclasses import replace
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('preparation_wait',ROOT/'scripts/wait_cooperative_ready.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
from multi_agent_nav.models import AgentState


def evidence():
    entry=dict(local_map_id='ranger-map',map_version='1')
    state=AgentState(robot_id='ranger',**entry,common_frame_id='',simulation=False,
        run_state='WAITING_PEER',localization_valid=True,nav_ready=True,stopped=True,stamp=1.)
    return module.PreparationEvidence('ranger',entry),state


def test_three_new_frames_and_standby_are_sufficient():
    sample,state=evidence()
    for i in range(3):
        sample.observe(replace(state,stamp=1.+i,run_state='STANDBY' if i else 'WAITING_PEER'),i*.5)
        assert bool(sample.issue(i*.5))==(i<2)
    assert sample.issue(4.01)


@pytest.mark.parametrize('change',[dict(run_state='IDLE'),dict(task_id='old'),dict(command_id='old'),
    dict(goal_id='old'),dict(ready=True),dict(reply='SYNCED'),dict(token='old'),
    dict(nav_ready=False),dict(simulation=True),dict(map_version='wrong'),dict(sequence=1)])
def test_dirty_or_unhealthy_state_resets_samples_and_recovers(change):
    sample,state=evidence()
    sample.observe(state,0)
    sample.observe(replace(state,stamp=2.,**change),.5)
    assert sample.count==0 and sample.issue(.5)
    for i in range(3):sample.observe(replace(state,stamp=3.+i),1.+i*.5)
    assert not sample.issue(2.)


def test_same_stamp_or_late_frames_cannot_accumulate_ready():
    sample,state=evidence()
    for i in range(3):sample.observe(state,i*.5)
    assert sample.issue(1.)
    for i in range(3):sample.observe(replace(state,stamp=3.+i),i*4.)
    assert sample.count==1


@pytest.mark.parametrize('fault',['late_peer','duplicate','source','reliability','durability','history','depth','interrupt'])
def test_wait_command_uses_live_samples_and_endpoint_gates_without_ros_init(tmp_path,monkeypatch,capsys,fault):
    from types import SimpleNamespace as S
    import rclpy
    import multi_agent_nav.config as config_module
    import multi_agent_nav.ros_codec as codec
    sample,state=evidence()
    entries={r:dict(sample.entry) for r in ('ranger','go2')}
    config=dict(scene={'share_c_coordinates':False},alignment={'robots':entries})
    monkeypatch.setattr(config_module,'load_config',lambda _:config)
    monkeypatch.setattr(codec,'from_ros',lambda message:message)
    (tmp_path/'bridge_ranger_pending.yaml').write_text('name: ranger_bridge\n')
    clock=[0.];turn=[0];subscriptions={};closed=[]
    def publishers(topic):
        peer='/go2/' in topic
        name='ranger_bridge_0' if peer else 'multi_agent_ranger_agent'
        bad=turn[0]<=3
        qos=S(reliability=S(name='BEST_EFFORT' if fault=='reliability' and bad else 'RELIABLE'),
            durability=S(name='TRANSIENT_LOCAL' if fault=='durability' and bad else 'VOLATILE'),
            history=S(name='KEEP_ALL' if fault=='history' and bad else 'KEEP_LAST'),
            depth=1 if fault=='depth' and bad else 10)
        endpoint=S(node_name='wrong_source' if fault=='source' and bad else name,
            topic_type='multi_agent_task_msgs/msg/AgentState',qos_profile=qos)
        return [endpoint,endpoint] if fault=='duplicate' and bad else [endpoint]
    node=S(create_subscription=lambda _,topic,callback,q:subscriptions.update({topic:callback}),
        get_publishers_info_by_topic=publishers,destroy_node=lambda:closed.append(True))
    monkeypatch.setattr(rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(rclpy,'create_node',lambda *a,**kw:node)
    monkeypatch.setattr(rclpy,'ok',lambda:True)
    monkeypatch.setattr(rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    def spin(*a,**kw):
        turn[0]+=1;clock[0]+=.5
        if fault=='interrupt':raise KeyboardInterrupt
        for robot in entries:
            if fault=='late_peer' and robot=='go2' and clock[0]<61.:continue
            subscriptions[f'/multi_agent/{robot}/state'](replace(state,robot_id=robot,stamp=clock[0]+1.))
    monkeypatch.setattr(rclpy,'spin_once',spin)
    assert module.main(['--side','ranger','--config-dir',str(tmp_path)])==(130 if fault=='interrupt' else 0)
    assert closed
    output=capsys.readouterr().out
    if fault=='late_peer':assert clock[0]>=62. and output.count('WAITING:')>=12
    elif fault!='interrupt':assert turn[0]>=6
    assert 'no start sent' in output
