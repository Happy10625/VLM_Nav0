"""First task observation runs before VLM selection or any waypoint transit."""
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as S

import pytest
import yaml

from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.models import Observation, Pose2D
from multi_agent_nav.runner import AgentRunner
from test_generated_candidates import generated_config
from test_private_c import local_agent


@pytest.fixture
def initial_run(generated_config,tmp_path):
    config,machine,adapter,command=local_agent()
    config['robot_ranger'].update(generated_config['robot_go2'])
    semantic=Path(config['robot_ranger']['generated_candidates']['semantic_yaml'])
    doc=yaml.safe_load(semantic.read_text());doc['source_robot']='ranger'
    semantic.write_text(yaml.safe_dump(doc))
    config['alignment']['robots']['ranger'].update(generated_config['alignment']['robots']['go2'])
    config['robot_ranger']['experiment_records_root']=str(tmp_path/'records')
    pending=Future();scans=[];requests=[];events=[]
    def scan(descriptions,scan_id,**options):
        scans.append((adapter.pose,descriptions,scan_id,options));return pending
    adapter.start_scan=scan
    adapter.client=S(choose_candidate=lambda snapshot:(requests.append(snapshot) or dict(candidate_id='P1',reason='next view')))
    adapter.vision=S(submit=lambda work:completed(work()))
    adapter.record_event=events.append
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config)
    return S(config=config,machine=machine,adapter=adapter,command=command,runner=runner,
        pending=pending,scans=scans,requests=requests,events=events)


def advance(run,until):
    for index in range(30):
        run.runner.tick(run.runner.now+.01)
        if until():return
    pytest.fail(f'condition not reached: {run.machine.state()}')


def observations(run,status='NOT_DETECTED'):
    return [Observation(target_id=t,view_index=i,scan_id=run.scans[0][2],
        status=status if t=='B' else 'NOT_DETECTED',common_position=Pose2D(3.,-3.),
        position_valid=status=='DETECTED' and t=='B') for i in range(1) for t in 'ABC']


@pytest.mark.parametrize('status',['NOT_DETECTED','UNCERTAIN'])
def test_initial_scan_completes_before_vlm_selects_and_navigates(initial_run,status):
    run=initial_run
    assert run.machine.accept(run.command,0.).accepted
    advance(run,lambda:bool(run.scans))
    assert run.scans[0][0]==Pose2D(3.,-3.)
    assert not run.requests and not run.adapter.calls
    assert run.scans[0][3]['stop_on_target']=='B'
    assert run.scans[0][3]['current_view_only'] is True
    run.pending.set_result(observations(run,status))
    advance(run,lambda:bool(run.adapter.calls))
    assert run.adapter.calls[0][0]=='VIEWPOINT_NAV'
    assert run.requests[0]['history'][0]['candidate_id']=='START'
    assert run.requests[0]['history'][0]['result']==status
    assert all(p['id']!='task_start' for p in run.requests[0]['candidates'])
    assert run.adapter.generated_observations[1]=={}
    assert next(e for e in run.events if e['state']=='SCANNING')['pose']==[3.,-3.,0.]


def test_located_target_at_start_skips_candidate_selection(initial_run):
    run=initial_run;assert run.machine.accept(run.command,0.).accepted
    advance(run,lambda:bool(run.scans));run.pending.set_result(observations(run,'DETECTED'))
    advance(run,lambda:bool(run.adapter.calls))
    assert not run.requests
    assert [call[0] for call in run.adapter.calls]==['PICKUP_NAV']


def test_current_view_does_not_require_rotation_space(initial_run):
    run=initial_run
    run.adapter.observation_candidate_free=lambda pose:pose!=Pose2D(3.,-3.)
    assert run.machine.accept(run.command,0.).accepted
    advance(run,lambda:bool(run.scans))
    assert not run.machine.paused and not run.requests and not run.adapter.calls


def test_cancel_during_initial_scan_never_selects_or_moves(initial_run):
    run=initial_run;assert run.machine.accept(run.command,0.).accepted
    advance(run,lambda:bool(run.scans))
    assert run.machine.accept(replace(run.command,action='CANCEL',cancel_command_id='b'),.1).accepted
    run.runner.tick(.1);run.pending.set_result(observations(run))
    run.runner.tick(.2)
    assert not run.requests and not run.adapter.calls


def test_later_command_does_not_repeat_task_start_scan(initial_run):
    run=initial_run;run.machine.last_sequence=1
    assert run.machine.accept(replace(run.command,sequence=2,command_id='next'),0.).accepted
    advance(run,lambda:bool(run.adapter.calls))
    assert not run.scans
    assert run.requests and run.adapter.calls[0][0]=='VIEWPOINT_NAV'


def test_new_task_scans_its_new_starting_pose(initial_run):
    run=initial_run;assert run.machine.accept(run.command,0.).accepted
    advance(run,lambda:bool(run.scans))
    peer=run.machine.peer_state
    run.machine.stop('end first task');run.runner.tick(.1)
    run.adapter.pose=Pose2D(-2.,-3.,.5)
    sync=replace(run.command,task_id='new-task',action='SYNC',kind='SYNC_LOCAL_C',
        phase='SYNCING',command_id='',sequence=0,target_id='')
    assert run.machine.accept(sync,.2).accepted
    run.machine.receive_peer(replace(peer,task_id='new-task',run_state='SYNCING'),
        run.config['alignment']['robots']['go2'],.2)
    assert run.machine.accept(replace(sync,phase='SEARCH_AB'),.2).accepted
    command=replace(run.command,task_id='new-task',command_id='first-again')
    assert run.machine.accept(command,.2).accepted
    advance(run,lambda:len(run.scans)==2)
    assert run.scans[-1][0]==Pose2D(-2.,-3.,.5)
    assert run.scans[-1][3]['task_id']=='new-task'
    assert not run.requests and not run.adapter.calls


def test_stationary_vlm_wait_does_not_pause_for_missing_or_unchanged_path(initial_run):
    run=initial_run
    assert run.machine.accept(run.command,0.).accepted
    advance(run,lambda:bool(run.scans))
    health=run.adapter.get_health
    run.adapter.get_health=lambda:replace(health(),path=[],path_valid=False,stopped=True,
        reason='execution path missing or receive timeout')
    for _ in range(170):
        now=run.runner.now+.1
        run.machine.peer_received=run.machine.sync_received=now
        run.runner.tick(now)
        assert not run.machine.paused
        assert run.machine.state().run_state=='LOCAL_EXECUTING'
    assert not run.adapter.calls and not run.requests


def test_shared_exploration_emits_the_same_candidate_set_to_visualization(initial_run):
    run=initial_run;shown=[]
    run.adapter.visualization=S(record=shown.append,observe=lambda _:None)
    assert run.machine.accept(run.command,0.).accepted
    advance(run,lambda:bool(run.scans));run.pending.set_result(observations(run))
    advance(run,lambda:bool(run.requests))
    request=next(e for e in shown if e['state']=='CANDIDATE_REQUEST')
    assert request['decision_id']==run.requests[0]['decision_id']
    assert request['candidates']==run.requests[0]['candidates']
