from dataclasses import replace
from types import SimpleNamespace as S
from pathlib import Path

import yaml
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.models import Observation, Pose2D


def display():
    from multi_agent_nav.ranger_visualization import RangerVisualization
    topics={};policies={}
    def publisher(kind,topic,qos):
        topics[topic]=[];policies[topic]=qos
        return S(publish=topics[topic].append)
    node=S(create_publisher=publisher,get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time())))
    frame=FrameAdapter('r','v','common','align',[10.,20.,0.])
    return RangerVisualization(node,frame),topics,policies


def spheres(messages):
    return [m for m in messages[-1].markers if m.action==Marker.ADD and m.type==Marker.SPHERE]


def test_blue_points_are_only_the_candidates_in_the_actual_request():
    view,topics,policies=display();view.begin('task')
    topic='/vlm_nav/ranger_exploration/vlm_candidates'
    point=dict(id='p',display_id='P1',pose=[2.,3.,0.])
    view.record(dict(state='CANDIDATE_GEOMETRY',candidates=[point,dict(id='unused',pose=[8.,9.,0.])]))
    assert not spheres(topics[topic])
    view.record(dict(state='CANDIDATE_REQUEST',candidates=[point]))
    marks=spheres(topics[topic]);assert len(marks)==1
    assert (marks[0].pose.position.x,marks[0].pose.position.y)==(2.,3.)
    assert (marks[0].color.r,marks[0].color.g,marks[0].color.b)==(0.,0.,1.)
    view.record(dict(state='CANDIDATE_REQUEST',candidates=[]))
    assert not spheres(topics[topic])
    from rclpy.qos import DurabilityPolicy
    assert policies[topic].durability==DurabilityPolicy.TRANSIENT_LOCAL


def test_green_targets_require_detected_valid_positions_and_use_local_map():
    view,topics,_=display();view.begin('task')
    topic='/vlm_nav/ranger_exploration/targets'
    b=Observation(target_id='B',status='DETECTED',position_valid=True,common_position=Pose2D(12.,23.))
    view.observe([b,replace(b,target_id='A',position_valid=False),replace(b,target_id='C',status='TIMEOUT')])
    marks=spheres(topics[topic]);assert len(marks)==1
    assert marks[0].header.frame_id=='map'
    assert (marks[0].pose.position.x,marks[0].pose.position.y)==(2.,3.)
    assert (marks[0].color.r,marks[0].color.g,marks[0].color.b)==(0.,1.,0.)
    view.begin('task')
    assert len(spheres(topics[topic]))==1
    view.observe([replace(b,target_id='C',common_position=Pose2D(14.,25.))])
    assert len(spheres(topics[topic]))==2
    view.begin('next-task');assert not spheres(topics[topic])


def test_default_rviz_enables_ranger_marker_topics():
    root=Path(__file__).resolve().parents[4]
    doc=yaml.safe_load((root/'config/vlm_nav.rviz').read_text())
    displays={d.get('Topic',{}).get('Value'):d for d in doc['Visualization Manager']['Displays']}
    for name in ('vlm_candidates','targets','observed_points'):
        item=displays['/vlm_nav/ranger_exploration/'+name]
        assert item['Enabled'] and item['Topic']['Durability Policy']=='Transient Local'


def test_runtime_factory_attaches_same_visualization_for_both_live_entry_names(monkeypatch):
    from multi_agent_nav.adapters import factory,ranger
    from multi_agent_nav import ranger_visualization
    config={'scene':{'share_c_coordinates':False},'alignment':{'robots':{'ranger':{
        'local_map_id':'r','map_version':'v'}}},'robot_ranger':{}}
    created=[]
    monkeypatch.setattr(ranger,'make_observer',lambda config,**kw:S(name=kw['node_name']))
    monkeypatch.setattr(ranger,'RangerAdapter',lambda *a,**kw:S())
    monkeypatch.setattr(ranger_visualization,'RangerVisualization',lambda node,frame:created.append(node.name) or S())
    for name in ('multi_agent_ranger_rgbd','multi_agent_ranger_check_motion'):
        _,adapter=factory.create_runtime('ranger',config,node_name=name)
        assert adapter.visualization is not None
    assert created==['multi_agent_ranger_rgbd','multi_agent_ranger_check_motion']
    _,adapter=factory.create_runtime('ranger',config,read_only=True,node_name='readonly')
    assert not hasattr(adapter,'visualization')


def test_completed_points_show_gray_coverage_and_clear_for_new_task():
    view,topics,_=display();view.begin('task')
    topic='/vlm_nav/ranger_exploration/observed_points'
    view.record(dict(state='CANDIDATE_GEOMETRY',task_id='task',observed_radius=3.,
        candidates=[dict(id='done',pose=[2.,3.,0.]),dict(id='failed',pose=[8.,9.,0.])],records={}))
    view.record(dict(state='CANDIDATE_REQUEST',candidates=[dict(display_id='P1',pose=[2.,3.,0.])]))
    view.record(dict(state='OBSERVATION_RECORD',observation_status='observed',records={'done':dict(status='observed'),
        'failed':dict(status='observation_failed'),'task_start':dict(status='observed')}))
    marks=spheres(topics[topic]);assert len(marks)==1
    assert (marks[0].pose.position.x,marks[0].pose.position.y)==(2.,3.)
    assert (marks[0].color.r,marks[0].color.g,marks[0].color.b)==(.5,.5,.5)
    disks=[m for m in topics[topic][-1].markers if m.type==Marker.CYLINDER and m.action==Marker.ADD]
    assert len(disks)==1 and disks[0].scale.x==disks[0].scale.y==6.
    assert disks[0].header.frame_id=='map' and 0.<disks[0].color.a<.5
    assert not spheres(topics['/vlm_nav/ranger_exploration/vlm_candidates'])
    view.begin('task');assert len(spheres(topics[topic]))==1
    view.begin('next-task');assert not spheres(topics[topic])
