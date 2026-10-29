"""Task-local observation coverage filters proposals before planning, without motion."""
import math
from pathlib import Path
from types import SimpleNamespace as S

import numpy as np
from PIL import Image
import pytest

from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.generated_candidates import GeneratedCandidates, render_selection
from multi_agent_nav.models import Observation, Pose2D
from test_generated_candidates import generated_config
from test_go2_exploration import LocalFixture


@pytest.fixture
def ranger_geometry(generated_config):
    config=generated_config
    config['robot_ranger']=dict(config['robot_go2'],generated_candidates=dict(observation_distance=1.,semantic_yaml=''))
    config['alignment']['robots']['ranger']=config['alignment']['robots']['go2'].copy()
    return config,GeneratedCandidates(config,'ranger')


def test_disk_boundary_failed_scans_and_initial_view(ranger_geometry):
    _,geometry=ranger_geometry
    geometry.points=[dict(id=name,pose=[x,y,0.],furniture_id='') for name,x,y in
        [('done',0.,0.),('inside',2.99,0.),('boundary',0.,3.),('outside',3.00001,0.),('far',-4.,0.)]]
    for status in ('executing','navigation_failed','observation_failed'):
        assert geometry.uncovered_points({'done':dict(status=status)})==geometry.points
    assert geometry.uncovered_points({'task_start':dict(status='observed')})==geometry.points
    records={'done':dict(status='observed')}
    assert [p['id'] for p in geometry.observed_points(records)]==['done']
    assert [p['id'] for p in geometry.uncovered_points(records)]==['outside','far']
    assert len(geometry.uncovered_points({}))==5


def test_go2_candidate_policy_is_unchanged(generated_config):
    geometry=GeneratedCandidates(generated_config)
    assert geometry.observed_radius==0.
    assert geometry.uncovered_points({p['id']:dict(status='observed') for p in geometry.points})==geometry.points


def test_both_vlm_maps_draw_same_3m_disk_without_editing_map(ranger_geometry,tmp_path):
    _,geometry=ranger_geometry
    geometry.points=[dict(id='done',pose=[0.,0.,0.],furniture_id='')]
    original=Path(geometry.source['image']).read_bytes()
    blank=render_selection(geometry,[],{},Pose2D(-4.,-4.),tmp_path/'blank')
    marked=render_selection(geometry,[],{'done':dict(status='observed')},Pose2D(-4.,-4.),tmp_path/'marked')
    arrays=[np.array(Image.open(p)) for p in marked]
    assert np.array_equal(*arrays)
    before=np.array(Image.open(blank[0]))
    for x,y,changed in [(1.,0.,True),(0.,2.8,True),(3.3,0.,False),(0.,-3.3,False)]:
        col,row=map(round,geometry.pixel(x,y))
        assert bool(np.any(arrays[0][row,col]!=before[row,col])) is changed
    assert Path(geometry.source['image']).read_bytes()==original


@pytest.mark.parametrize('cooperative',[False,True])
def test_completed_disk_persists_in_task_and_resets_in_new_task(ranger_geometry,tmp_path,monkeypatch,cooperative):
    from multi_agent_nav.go2_exploration import explore_steps
    config,geometry=ranger_geometry
    geometry.points=[dict(id=name,pose=[x,0.,0.],furniture_id='') for name,x in [('first',0.),('near',1.),('far',3.1)]]
    monkeypatch.setattr('multi_agent_nav.go2_exploration.GeneratedCandidates',lambda *a:geometry)
    adapter=LocalFixture();adapter.selection_evidence_dir=tmp_path/'requests'
    requests=[];plans=[];events=[]
    adapter.client=S(choose_candidate=lambda snapshot:(requests.append(snapshot) or dict(candidate_id='P1',reason='next')))
    adapter.vision=S(submit=lambda work:completed(work()))
    adapter.observation_candidate_free=lambda goal:True
    compute=adapter.compute_path
    adapter.compute_path=lambda goal:(plans.append(goal) or compute(goal))
    adapter.start_scan=lambda descriptions,scan_id,**kw:completed([
        Observation(target_id=t,view_index=0,status='NOT_DETECTED') for t in descriptions])
    frame=FrameAdapter('fixture','1','ranger_local_map','',[0,0,0]);now=[0.]
    def run(task):
        steps=explore_steps(adapter,config,frame,100.,'target',events.append,lambda:[],lambda:True,lambda:None,
            clock=lambda:now[0],robot_id='ranger',task_id=task,require_navigation_success=cooperative)
        for _ in range(500):
            try:next(steps)
            except StopIteration as end:return end.value
            now[0]+=.05;adapter.advance(now[0])
        steps.close();pytest.fail('exploration did not finish')
    run('one')
    assert len(requests)==2
    assert [p['id'] for p in requests[1]['candidates']]==['far']
    assert requests[1]['observed_exclusion_radius']==3.
    assert [p.x for _,p,_ in adapter.calls]==[0.,3.1]
    completed_rows=[e for e in events if e.get('observation_status')=='observed']
    assert completed_rows[-1]['records']['near']['status']=='unobserved'
    count=len(requests);plans.clear()
    run('one')
    assert len(requests)==count and not plans
    run('two')
    assert len(requests)>count
    assert len(requests[count]['candidates'])==3


def test_candidates_must_be_outside_every_completed_observation_disk(ranger_geometry):
    _,geometry=ranger_geometry
    geometry.points=[dict(id=name,pose=[x,0.,0.],furniture_id='') for name,x in
        [('older',0.),('latest',10.),('near_older',2.8),('near_latest',7.2),('between',5.)]]
    records={name:dict(status='observed') for name in ('older','latest')}
    assert [p['id'] for p in geometry.uncovered_points(records)]==['between']
