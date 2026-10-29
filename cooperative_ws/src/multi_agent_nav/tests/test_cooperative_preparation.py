"""Heavy map preparation yields to heartbeat callbacks and respects cancellation."""
from concurrent.futures import Future
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.go2_exploration import explore_steps
from multi_agent_nav.adapters.fake import FakeAdapter
from multi_agent_nav.mock import make_machine
from multi_agent_nav.models import Pose2D
from test_generated_candidates import generated_config
from test_semantic_search import catalog_config
from test_private_c import private_config


@pytest.mark.parametrize('stage',['geometry','render','digest'])
@pytest.mark.parametrize('robot_id,cooperative',[('go2',True),('ranger',True),('ranger',False)])
def test_map_preparation_yields_and_cancels_pending_work(generated_config,tmp_path,monkeypatch,stage,robot_id,cooperative):
    import multi_agent_nav.go2_exploration as core
    config=private_config()
    config['robot_'+robot_id].update(generated_config['robot_go2'])
    config['alignment']['robots'][robot_id].update(generated_config['alignment']['robots']['go2'])
    if robot_id=='ranger':
        from pathlib import Path
        import yaml
        path=Path(config['robot_ranger']['generated_candidates']['semantic_yaml'])
        document=yaml.safe_load(path.read_text());document['source_robot']='ranger'
        path.write_text(yaml.safe_dump(document))
    machine,frame=make_machine(config,robot_id,True)
    adapter=FakeAdapter(robot_id)
    adapter.client=S(choose_candidate=lambda _:pytest.fail('not a model request'))
    adapter.selection_evidence_dir=tmp_path/'selections'
    pending=Future();jobs=[]
    def submit(fn):
        jobs.append(fn)
        if stage=='geometry' or (stage=='digest' and fn.__name__=='catalog_unchanged') or (stage=='render' and fn.__name__=='render'):return pending
        from multi_agent_nav.adapters.fake import completed
        return completed(fn())
    adapter.vision=S(submit=submit)
    if stage=='geometry':monkeypatch.setattr(core,'GeneratedCandidates',lambda *a:pytest.fail('geometry ran in ROS callback'))
    elif stage=='render':monkeypatch.setattr(core,'render_selection',lambda *a:pytest.fail('render ran in ROS callback'))
    else:monkeypatch.setattr(core.GeneratedCandidates,'unchanged',lambda *a:pytest.fail('digest ran in ROS callback'))
    steps=explore_steps(adapter,config,frame,100.,'target',lambda _:None,lambda:[],None,
        lambda:None,clock=lambda:0.,robot_id=robot_id,target_id='A',success_radius=1.,require_navigation_success=cooperative)
    try:
        for _ in range(5):next(steps)
        assert jobs and not adapter.calls
        assert any(fn.__name__==('render' if stage=='render' else 'catalog_unchanged') for fn in jobs) if stage!='geometry' else len(jobs)==1
    finally:steps.close()
    assert pending.cancelled()


def test_previous_worker_finishes_before_new_preparation_without_blocking(generated_config,monkeypatch):
    config=private_config();config['robot_go2'].update(generated_config['robot_go2'])
    config['alignment']['robots']['go2'].update(generated_config['alignment']['robots']['go2'])
    _,frame=make_machine(config,'go2',True)
    adapter=FakeAdapter('go2');adapter.client=S()
    pending=Future();submitted=[]
    def submit(fn):
        assert adapter.vision.available,'must wait for previous canceled work'
        submitted.append(fn);return pending
    adapter.vision=S(available=False,submit=submit)
    steps=explore_steps(adapter,config,frame,100.,'target',lambda _:None,lambda:[],None,
        lambda:None,clock=lambda:0.,require_navigation_success=True,success_radius=1.)
    try:
        for _ in range(4):next(steps)
        assert not submitted
        adapter.vision.available=True
        next(steps)
        assert len(submitted)==1 and not adapter.calls
    finally:steps.close()
    assert pending.cancelled()


@pytest.mark.parametrize('change',['stopped','terminal','health','epoch','footprint','digest_changed'])
def test_last_digest_wait_rechecks_motion_prerequisites_before_approach(change,monkeypatch):
    import multi_agent_nav.go2_exploration as core
    from dataclasses import replace
    from multi_agent_nav.models import Observation
    from multi_agent_nav.adapters.fake import completed
    config=private_config();config['robot_ranger']['exploration_strategy']='generated_candidates_vlm'
    _,frame=make_machine(config,'ranger',True)
    adapter=FakeAdapter('ranger');adapter.client=S();adapter.selection_epoch=0
    geometry=S(points=[],radius=.5,observed_radius=2.5,semantic_path=None,digests={},unchanged=lambda:True)
    monkeypatch.setattr(core,'GeneratedCandidates',lambda *a:geometry)
    pending=Future()
    adapter.vision=S(submit=lambda fn:pending if fn.__name__=='catalog_unchanged' else completed(fn()))
    observation=Observation(target_id='B',status='DETECTED',common_position=Pose2D(5,0),position_valid=True)
    steps=explore_steps(adapter,config,frame,100.,'B',lambda _:None,lambda:[],None,lambda:None,
        clock=lambda:0.,robot_id='ranger',target_id='B',require_navigation_success=True,
        success_radius=1.,initial_observation=observation)
    try:
        for _ in range(3):next(steps)
        assert not pending.done() and not adapter.calls
        health=adapter.get_health()
        if change=='stopped':adapter.get_health=lambda:replace(health,stopped=False)
        if change=='terminal':adapter.get_health=lambda:replace(health,motion_terminal=False)
        if change=='health':adapter.get_health=lambda:replace(health,nav_ready=False)
        if change=='epoch':adapter.selection_epoch=1
        if change=='footprint':adapter.costmap_candidate_free=lambda *a,**k:False
        pending.set_result(change!='digest_changed')
        expected=core.RecoverablePause if change=='health' else core.ExplorationStopped
        with pytest.raises(expected):next(steps)
        assert not adapter.calls
    finally:steps.close()


@pytest.mark.parametrize('asset',['points.yaml','semantic.yaml','annotated.png','map.yaml','map.pgm'])
def test_semantic_assets_changed_during_navigation_are_rejected(catalog_config,asset):
    from pathlib import Path
    from multi_agent_nav.models import Observation
    from multi_agent_nav.adapters.fake import completed
    from multi_agent_nav.go2_exploration import ExplorationStopped
    config=private_config();config['robot_go2'].update(catalog_config['robot_go2'])
    config['alignment']['robots']['go2'].update(catalog_config['alignment']['robots']['go2'])
    _,frame=make_machine(config,'go2',True)
    adapter=FakeAdapter('go2',delay=10.);adapter.client=S();adapter.vision=S(submit=lambda fn:completed(fn()))
    steps=explore_steps(adapter,config,frame,100.,'A',lambda _:None,lambda:[],None,lambda:None,
        clock=lambda:0.,require_navigation_success=True,success_radius=1.,
        initial_observation=Observation(target_id='A',status='DETECTED',position_valid=True,common_position=Pose2D(5,0)))
    try:
        for _ in range(10):
            next(steps)
            if adapter.calls:break
        assert adapter.active
        path=Path(config['robot_go2']['map_path']).parent/asset
        path.write_bytes(path.read_bytes()+b' ')
        with pytest.raises(ExplorationStopped,match='assets changed'):next(steps)
    finally:steps.close()
