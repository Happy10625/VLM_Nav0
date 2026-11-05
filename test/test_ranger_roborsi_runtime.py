"""Real RoboRSI registration/dispatch with fake planners, never model requests."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from test_ranger_roborsi_session import entry, make_spec, Spawn, identity, evidence


def config():
    return dict(framework_root='/home/isee601/ws/SenseNova-RoboRSI',
                framework_commit='0208cd72ec7c885a7219040cd2be459d127e374e',
                cli_path='/home/isee601/.local/bin/codex', cli_version='codex-cli 0.166.0',
                cli_sha256='b8a11fedf24b2dfb59a70c9e52b030f9a508dcd49cd2dd0d41956f4c785156b8',
                model='gpt-6-astra', reasoning_effort='high', planner_timeout_s=1200,
                action_timeout_s=30, real_backend_enabled=False, cli_args=[], cli_overrides={},
                capability_verification=None)


@pytest.mark.parametrize('behavior', ['success', 'no_execution', 'early_finish', 'error', 'bad_target'])
def test_actual_framework_dispatch_and_host_truth(entry, monkeypatch, behavior):
    from roborsi.planner.base import PlannerResult
    import roborsi.embodied_agent as embodied
    from tools.ranger_agent import runtime
    from tools.ranger_agent.session import SearchSession
    spec = make_spec(entry)
    spec['framework_config'] = config()
    spawn = Spawn()
    sessions, construction = [], []
    def factory(spec):
        s = SearchSession(spec, spawn=spawn, identity=identity,
            send_signal=lambda *_: (evidence(s, result='operator interrupt'), setattr(spawn.process, 'code', 1)))
        sessions.append(s)
        return s
    class Planner:
        def preflight(self): pass
        def solve(self, *, toolkit, **kwargs):
            assert kwargs['user_message'] == spec['target']
            assert 'start_find_target' in kwargs['system_prompt']
            if behavior != 'no_execution':
                a = toolkit.execute_tool('start_find_target', {'target': spec['target']})
                if behavior == 'bad_target':
                    rejected = toolkit.execute_tool('start_find_target', {'target': 'changed'})
                    assert 'error' in rejected.result
                b = toolkit.execute_tool('start_find_target', {'target': spec['target']})
                assert a.result['task_id'] == b.result['task_id']
                if behavior in ('success', 'bad_target'):
                    evidence(sessions[0]); spawn.process.code = 0
                    row = toolkit.execute_tool('get_task_status', {'task_id': a.result['task_id']})
                    assert row.result['executor_success'] is True
            if behavior == 'error': raise RuntimeError('provider private error')
            return PlannerResult(finish_result={'status': 'success', 'summary': 'model claims success'})
    def build(*args, **kwargs):
        construction.append((args, kwargs)); return Planner()
    monkeypatch.setattr(embodied, 'build_planner', build)
    row = runtime.run_session(spec, session_factory=factory, preflight=lambda _: {'offline': True})
    assert row['executor_success'] is (behavior in ('success', 'bad_target'))
    assert len(spawn.calls) == (0 if behavior=='no_execution' else 1)
    assert construction[0][0][0] == 'codex_cli'
    assert construction[0][1]['model'] == 'gpt-6-astra'
    assert construction[0][1]['reasoning_effort'] == 'high'
    assert construction[0][1]['max_tokens'] is None
    assert construction[0][1]['codex_cli'].inherit_env is False
    assert 'DASHSCOPE_API_KEY' not in construction[0][1]['codex_cli'].env
    if behavior == 'no_execution': assert row['executor_result'] == 'AGENT_NO_EXECUTION'
    if behavior == 'error': assert row['framework_error']


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_signal_handler_only_flags_and_host_cancels(entry, monkeypatch, sig):
    from roborsi.planner.base import PlannerResult
    import roborsi.embodied_agent as embodied
    from tools.ranger_agent import runtime
    from tools.ranger_agent.session import SearchSession
    spec = make_spec(entry); spec['framework_config'] = config()
    spawn, sent = Spawn(), []
    def factory(spec):
        def send(*args):
            sent.append(args); evidence(s, result='operator interrupt'); spawn.process.code=1
        s=SearchSession(spec, spawn=spawn, identity=identity, send_signal=send)
        return s
    class Planner:
        def preflight(self): pass
        def solve(self, *, toolkit, cancel_event, **kwargs):
            toolkit.execute_tool('start_find_target', {'target': spec['target']})
            # Invoke the installed handler directly; never signal the test runner/real robot.
            handler=signal.getsignal(sig)
            handler(sig, None); handler(sig, None)
            assert cancel_event.wait(3)
            return PlannerResult(error_kind='cancelled', error='cancelled')
    monkeypatch.setattr(embodied, 'build_planner', lambda *a, **k: Planner())
    before=signal.getsignal(sig)
    row=runtime.run_session(spec, session_factory=factory, preflight=lambda _: {})
    assert signal.getsignal(sig) == before
    assert sent == [(spawn.process.pid, signal.SIGINT)]
    assert row['stop_status']=='CONFIRMED' and not row['executor_success']


def test_preflight_failure_never_starts_or_falls_back(entry):
    from tools.ranger_agent import runtime
    from tools.ranger_agent.session import SearchSession
    spec=make_spec(entry);spec['framework_config']=config();spawn=Spawn()
    def fail(_): raise ValueError('wrong CLI version')
    row=runtime.run_session(spec, session_factory=lambda spec: SearchSession(spec,spawn=spawn),preflight=fail)
    assert not spawn.calls and row['framework_error']
    assert row['stop_status']=='NOT_STARTED'


def test_enabled_backend_requires_bound_capability_proof(entry):
    from tools.ranger_agent import runtime
    spec=make_spec(entry);spec['framework_config']=dict(config(),real_backend_enabled=True)
    with pytest.raises(ValueError, match='capability'):
        runtime.verify_real_backend(spec)


def test_planner_budget_never_changes_search_timeout(entry):
    from tools.ranger_agent import runtime
    spec=make_spec(entry,timeout=1300);spec['framework_config']=config()
    with pytest.raises(ValueError, match='budget'):
        runtime.validate_budget(spec)
    assert spec['options']['timeout']==1300


def test_framework_cleanup_does_not_include_sibling_search(tmp_path):
    from roborsi.planner.codex_cli import _owned_descendants, _terminate_process
    # Two harmless interpreters have the same parent; only the CLI is cleaned by RoboRSI.
    cli=subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)'],start_new_session=True)
    stop=tmp_path/'finish'
    search=subprocess.Popen([sys.executable,'-c',
        'import pathlib,time,sys\np=pathlib.Path(sys.argv[1])\nwhile not p.exists():time.sleep(.02)',str(stop)],
        start_new_session=True)
    try:
        assert search.pid not in _owned_descendants(cli.pid)
        def parent(pid): return int(Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[1])
        assert parent(cli.pid)==parent(search.pid)==os.getpid()
        _terminate_process(cli,time.monotonic()+3)
        assert search.poll() is None
    finally:
        stop.touch();search.wait(timeout=5)
        if cli.poll() is None: _terminate_process(cli,time.monotonic()+3)


def test_tool_schema_rejects_extra_fields_without_start(entry):
    from tools.ranger_agent.runtime import local_tools
    from tools.ranger_agent.session import SearchSession
    spawn=Spawn();s=SearchSession(make_spec(entry),spawn=spawn,identity=identity)
    start=local_tools(s)[0]
    assert 'error' in start.handler({'target':s.spec['target'],'timeout':1}).result
    assert not spawn.calls


def test_bootstrap_private_fd_keeps_ros_environment_out_of_framework(entry, tmp_path):
    import yaml
    repo, _=entry
    cfg=config();cfg.update(real_backend_enabled=True)
    stub=repo/'fake_framework'
    capture=repo/'captured.json'
    stub.write_text('#!/usr/bin/python3\nimport json,os,sys\n'
        'fd=int(sys.argv[sys.argv.index("--spec-fd")+1])\n'
        'with os.fdopen(fd) as f: spec=json.load(f)\n'
        'from pathlib import Path\n'
        f'Path({str(capture)!r}).write_text(json.dumps(dict(target=spec["target"],'
        'ros_in_framework="PYTHONPATH" in os.environ,secret_in_framework="DASHSCOPE_API_KEY" in os.environ,'
        'ros_in_search=spec["search_env"].get("PYTHONPATH"),'
        'search_python=spec["search_python"])))\n')
    stub.chmod(0o755);cfg['framework_python']=str(stub)
    (repo/'tools/ranger_agent').mkdir(parents=True)
    (repo/'tools/ranger_agent/config.yaml').write_text(yaml.safe_dump(cfg))
    with (repo/'RangerVLM_env.bash').open('a') as f:
        f.write('export PATH=/usr/bin:/bin PYTHONPATH=/fixture/ros DASHSCOPE_API_KEY=test-private\n')
    driver=repo/'driver.py'
    driver.write_text('from pathlib import Path\nfrom tools.ranger_agent import runtime\n'
        'runtime.source_identity=lambda _: {"fixture":True}\n'
        'runtime.open_diagnostics=lambda _: None\n'
        f'raise SystemExit(runtime.bootstrap({{"allow_motion":True,"target_id":"B","target":"  杯子  "}},Path({str(repo)!r})))\n')
    run=subprocess.run(['/usr/bin/python3',str(driver)],env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[1])),
                       capture_output=True,text=True,timeout=10)
    assert run.returncode==0,run.stdout+run.stderr
    data=json.loads(capture.read_text())
    assert data==dict(target='  杯子  ',ros_in_framework=False,secret_in_framework=False,
                     ros_in_search='/fixture/ros',search_python='/usr/bin/python3')
    assert 'test-private' not in run.stdout+run.stderr
    assert '状态文件：' in run.stdout


def test_real_version_preflight_rejects_wrong_binary(entry, monkeypatch):
    from tools.ranger_agent import runtime
    from tools.ranger_agent.session import file_digest,SearchSession
    spec=make_spec(entry);cfg=config();spec['framework_config']=cfg
    fake=entry[0]/'wrong-codex'
    fake.write_text('#!/bin/sh\necho "codex-cli wrong-version"\n');fake.chmod(0o755)
    cfg.update(cli_path=str(fake),cli_sha256=file_digest(fake))
    SearchSession(spec)  # allocate the host directory only
    monkeypatch.setattr(runtime,'verify_real_backend',lambda _: {})
    with pytest.raises(ValueError, match='version mismatch'):
        runtime.preflight_framework(spec)


def test_capability_proof_is_invalidated_by_policy_change(entry,monkeypatch):
    from tools.ranger_agent import runtime
    from tools.ranger_agent.session import file_digest
    import hashlib
    spec=make_spec(entry);cfg=config();cfg['real_backend_enabled']=True
    spec['framework_config']=cfg
    proof=entry[0]/'proof.json';artifact=entry[0]/'evidence.txt';artifact.write_text('offline fixture only')
    policy={'cli_args':[],'config_overrides':{}}
    monkeypatch.setattr(runtime,'cli_policy',lambda _:policy)
    proof.write_text(json.dumps(dict(verified=True,cli_path=str(Path(cfg['cli_path']).resolve()),
        cli_sha256=file_digest(cfg['cli_path']),cli_version=cfg['cli_version'],framework_commit=cfg['framework_commit'],
        policy_sha256=hashlib.sha256(json.dumps(policy,sort_keys=True).encode()).hexdigest(),
        evidence_path=str(artifact),evidence_sha256=file_digest(artifact))))
    cfg['capability_verification']=str(proof)
    runtime.verify_real_backend(spec)
    policy['cli_args']=['--enable','shell_tool']
    with pytest.raises(ValueError,match='capability verification'):
        runtime.verify_real_backend(spec)


def test_signal_during_next_call_does_not_admit_start(entry,monkeypatch):
    from tools.ranger_agent import runtime
    from tools.ranger_agent.session import SearchSession
    import roborsi.embodied_agent as embodied
    from types import SimpleNamespace
    spec=make_spec(entry);spec['framework_config']=config();spawn=Spawn()
    class Episode:
        done=False
        def next_call(self,timeout):
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM,None)
            return SimpleNamespace(name='start_find_target',arguments={'target':spec['target']})
        def complete(self,*a): pass
        def close(self,timeout): self.done=True
    class Agent:
        def __init__(self,**kwargs): pass
        def start_episode(self,*a,**kwargs): return Episode()
    monkeypatch.setattr(embodied,'EmbodiedAgent',Agent)
    row=runtime.run_session(spec,session_factory=lambda s: SearchSession(s,spawn=spawn,identity=identity),
                            preflight=lambda _: {})
    assert not spawn.calls
    assert row['executor_result']=='AGENT_NO_EXECUTION'
    assert row['framework_error']=='HOST_SIGNAL'
