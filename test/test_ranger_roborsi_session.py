"""Offline contracts: all executor launches are injected or use a fixture script."""
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import subprocess
import signal
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def entry(tmp_path):
    (tmp_path / 'scripts').mkdir()
    shutil.copy2(ROOT / 'scripts/start_ranger_single_robot.sh', tmp_path / 'scripts')
    config = tmp_path / 'acceptance/config'
    config.mkdir(parents=True)
    (config / 'scene.yaml').write_text('task_timeout: 600.0\ntargets: {A: 纸巾, B: 浅棕色盒子, C: 白色桶}\n')
    for name in ('robot_ranger', 'robot_go2', 'alignment'):
        (config / (name + '.yaml')).write_text('{}\n')
    (tmp_path / 'RangerVLM_env.bash').write_text(f'export RANGER_CONFIG="{config}"\n')
    (tmp_path / 'bin').mkdir()
    python = tmp_path / 'bin/python3'
    python.write_text('#!/usr/bin/python3\nimport json,sys,yaml\nfrom pathlib import Path\n'
                      'print(yaml.safe_load(Path(sys.argv[2]).read_text())["task_timeout"] '
                      'if sys.argv[1]=="-" else json.dumps(sys.argv[1:],ensure_ascii=False))\n')
    python.chmod(0o755)
    return tmp_path, config


def captured(entry, options):
    repo, _ = entry
    result = subprocess.run(['/bin/bash', str(repo / 'scripts/start_ranger_single_robot.sh'), *options],
                            env=dict(os.environ, PATH=str(repo / 'bin') + ':/usr/bin:/bin'),
                            capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


def last_value(argv, flag):
    return argv[len(argv) - 1 - argv[::-1].index(flag) + 1]


def test_original_entry_defaults_and_overrides(entry):
    original = captured(entry, ['--allow-motion'])
    assert last_value(original, '--target-id') == 'B'
    assert last_value(original, '--timeout') == '600.0'
    custom = entry[0] / 'custom'
    custom.mkdir()
    (custom / 'scene.yaml').write_text('task_timeout: 432.0\n')
    overridden = captured(entry, ['--allow-motion', '--config-dir', str(custom),
                                  '--target-id', 'C', '--target', ' 原文 ', '--timeout', '99'])
    assert last_value(overridden, '--config-dir') == str(custom)
    assert last_value(overridden, '--timeout') == '99'
    assert last_value(overridden, '--target') == ' 原文 '


def make_spec(entry, **options):
    from tools.ranger_agent.session import prepare_spec
    repo, config = entry
    return prepare_spec(dict(allow_motion=True, target_id='B', **options), repo,
                        {'PATH': '/usr/bin:/bin', 'RANGER_CONFIG': str(config),
                         'PYTHONPATH': '/ros/path', 'DASHSCOPE_API_KEY': 'test-secret'},
                        session_dir=repo / 'runs/session-one')


def test_agent_reuses_original_script(entry):
    from tools.ranger_agent.session import build_search_argv
    spec = make_spec(entry)
    argv = build_search_argv(spec)
    assert argv[:2] == ['/bin/bash', str(entry[0] / 'scripts/start_ranger_single_robot.sh')]
    assert '--timeout' not in argv and '--strategy' not in argv
    assert last_value(argv, '--target') == '浅棕色盒子'
    actual = captured(entry, argv[2:])
    assert last_value(actual, '--timeout') == '600.0'
    assert last_value(actual, '--config-dir') == str(entry[1])
    assert not Path(spec['session_dir'], 'search').exists()


def test_environment_and_marker_resolution(entry):
    from tools.ranger_agent.session import prepare_spec, validate_inputs, framework_environment
    repo, config = entry
    custom = repo / 'custom'
    shutil.copytree(config, custom)
    spec = make_spec(entry, config_dir=str(custom), target='  白色杯子  ', timeout=120)
    assert spec['target'] == '  白色杯子  '
    assert spec['config_dir'] == str(custom)
    env = framework_environment(spec['search_env'])
    assert 'PYTHONPATH' not in env and 'DASHSCOPE_API_KEY' not in env
    spec['search_env']['RANGER_CONFIG'] = '/changed/marker'
    validate_inputs(spec)
    (custom / 'scene.yaml').write_text('targets: {B: changed}\n')
    with pytest.raises(ValueError, match='configuration changed'):
        validate_inputs(spec)
    with pytest.raises(ValueError, match='interpreter'):
        prepare_spec({'target_id': 'B'}, repo, {'PATH': str(repo / 'bin'), 'RANGER_CONFIG': str(config)})


class Process:
    pid = 123456789

    def __init__(self):
        self.code = None

    def poll(self):
        return self.code


class Spawn:
    def __init__(self):
        self.calls = []
        self.process = Process()

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        return self.process


def identity(pid):
    return {'pid': pid, 'pgid': pid, 'start_ticks': 100, 'boot_id': 'test'}


def session(entry, **kwargs):
    from tools.ranger_agent.session import SearchSession
    spec = make_spec(entry)
    spawn = Spawn()
    return SearchSession(spec, spawn=spawn, identity=identity, **kwargs), spawn


def evidence(s, *, result='TARGET_APPROACHED', stop=True, failed=False, pending=False,
             event_result=None, target_id='B', consumer_status=None):
    path = Path(s.spec['session_dir']) / 'search'
    path.mkdir(exist_ok=True)
    report = {'mode': 'local_ranger_explore', 'target_id': target_id, 'result': result,
              'stop_confirmed': stop, 'evidence_write_failed': failed, 'vlm_evidence_pending': pending}
    (path / 'result.json').write_text(json.dumps(report))
    rows = [dict(run_id='run-one', sequence=1, target_id=target_id, target=s.spec['target'],
                 state='TARGET_APPROACHED', stopped=False, stop_confirmed=True)]
    if consumer_status:
        rows.append(dict(run_id='run-one', sequence=2, target_id=target_id, target=s.spec['target'],
                         event='VLM_VIEW_RESULT', consumer_status=consumer_status))
    rows.append(dict(run_id='run-one', sequence=len(rows)+1, target_id=target_id, target=s.spec['target'],
                     event='SESSION_END', result=event_result or result, stop_confirmed=stop))
    (path / 'events.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))


def test_same_session_concurrent_start_consumes_once(entry):
    from tools.ranger_agent.session import SearchSession
    s, spawn = session(entry)
    other = SearchSession(s.spec, spawn=spawn, identity=identity)
    with ThreadPoolExecutor(8) as pool:
        rows = list(pool.map(lambda i: (s if i % 2 else other).start(s.spec['target']), range(20)))
    assert len(spawn.calls) == 1
    assert {r['task_id'] for r in rows} == {s.spec['task_id']}
    with pytest.raises(ValueError, match='fixed target'):
        s.start('different')
    restarted = SearchSession(s.spec, spawn=spawn, identity=identity)
    restarted.start(s.spec['target'])
    assert len(spawn.calls) == 1
    stored = (Path(s.spec['session_dir']) / 'manifest.json').read_text()
    assert 'test-secret' not in stored and 'PYTHONPATH' not in stored


def test_spawn_failure_is_consumed_and_not_retried(entry):
    from tools.ranger_agent.session import SearchSession
    calls = []
    def fail(*args, **kwargs):
        calls.append(1)
        raise OSError('private error must not enter tool output')
    s = SearchSession(make_spec(entry), spawn=fail)
    a = s.start(s.spec['target'])
    b = s.start(s.spec['target'])
    assert len(calls) == 1
    assert a['stop_status'] == 'NOT_STARTED'
    assert b['executor_success'] is False
    assert 'private error' not in json.dumps(b)


@pytest.mark.parametrize('target', [' 中文目标 ', '"杯子"; echo unsafe', '$(touch /tmp/do-not-create)'])
def test_target_is_exact_argv_not_shell(entry, target):
    from tools.ranger_agent.session import SearchSession
    spawn = Spawn()
    s = SearchSession(make_spec(entry, target=target), spawn=spawn, identity=identity)
    s.start(target)
    argv, kwargs = spawn.calls[0]
    assert last_value(argv, '--target') == target
    assert kwargs['shell'] is False and kwargs['start_new_session'] is True
    assert kwargs['env']['PYTHONPATH'] == '/ros/path'


@pytest.mark.parametrize('kind,success,quality', [
    ('ok', True, 'REPORTED_OK'), ('failed', False, 'REPORTED_OK'),
    ('no_stop', False, 'INCOMPLETE'), ('write_error', True, 'INCOMPLETE'),
    ('pending', True, 'INCOMPLETE'), ('partial_json', False, 'INCOMPLETE'),
    ('wrong_target', False, 'INCOMPLETE'), ('wrong_end', True, 'INCOMPLETE'),
    ('missing_end', True, 'INCOMPLETE'), ('late_work', True, 'UNKNOWN'),
])
def test_result_uses_current_final_evidence(entry, kind, success, quality):
    s, spawn = session(entry)
    s.start(s.spec['target'])
    evidence(s, result='FAILED' if kind=='failed' else 'TARGET_APPROACHED',
             stop=None if kind=='no_stop' else True, failed=kind=='write_error', pending=kind=='pending',
             target_id='C' if kind=='wrong_target' else 'B',
             event_result='FAILED' if kind=='wrong_end' else None,
             consumer_status='TIMEOUT' if kind=='late_work' else None)
    path = Path(s.spec['session_dir']) / 'search'
    if kind=='partial_json': (path / 'result.json').write_text('{')
    if kind=='missing_end': (path / 'events.jsonl').write_text('')
    assert not s.status(s.spec['task_id'])['executor_success']  # live data is not final
    spawn.process.code = 0
    row = s.status(s.spec['task_id'])
    assert row['executor_success'] is success
    assert row['evidence_status'] == quality
    assert row['active_process'] is False
    assert row['process_exit_code'] == 0
    assert row['executor_result'] != 'model says success'
    s.start(s.spec['target'])
    assert len(spawn.calls) == 1


def test_config_mutation_does_not_spawn(entry):
    s, spawn = session(entry)
    (entry[1] / 'scene.yaml').write_text('targets: {B: replaced}\n')
    with pytest.raises(ValueError, match='configuration changed'):
        s.start(s.spec['target'])
    assert not spawn.calls


def test_default_real_backend_disabled_even_with_motion_permission(entry):
    from tools.ranger_agent.session import SearchSession
    s = SearchSession(make_spec(entry))
    with pytest.raises(ValueError, match='real backend'):
        s.start(s.spec['target'])


@pytest.mark.parametrize('finish', [True, False])
def test_cancel_once_and_never_escalate(entry, finish):
    now, sent = [0.], []
    s, spawn = session(entry, clock=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt),
                       send_signal=lambda pgid, sig: sent.append((pgid, sig)))
    s.start(s.spec['target'])
    a = s.cancel(s.spec['task_id'])
    assert a['state'] == 'CANCEL_REQUESTED' and a['active_process'] is True
    s.cancel(s.spec['task_id'])
    if finish:
        evidence(s, result='operator interrupt')
        spawn.process.code = 1
    row = s.close()
    assert sent == [(spawn.process.pid, signal.SIGINT)]
    assert row['state'] == ('FINISHED' if finish else 'STOP_UNCONFIRMED')
    assert row['active_process'] is not finish
    assert not row['executor_success']
    s.start(s.spec['target'])
    assert len(spawn.calls) == 1
    if not finish:
        assert now[0] >= 30


def test_cancel_after_success_preserves_completion(entry):
    sent = []
    s, spawn = session(entry, send_signal=lambda *a: sent.append(a))
    s.start(s.spec['target'])
    evidence(s)
    spawn.process.code = 0
    row = s.cancel(s.spec['task_id'])
    assert row['executor_success'] is True and not sent
    assert row['cancel_requested_at'] is None


def test_identity_mismatch_never_signals_unknown_process(entry):
    sent = []
    s, _ = session(entry, send_signal=lambda *a: sent.append(a))
    s.start(s.spec['target'])
    s._identity = lambda pid: dict(identity(pid), start_ticks=101)
    row = s.cancel(s.spec['task_id'])
    assert not sent and row['state'] == 'STOP_UNCONFIRMED'
    assert row['active_process'] is True


def test_consumed_record_without_pid_never_restarts(entry):
    from tools.ranger_agent.session import SearchSession, atomic_json
    s, spawn = session(entry)
    p = Path(s.spec['session_dir']) / 'session.json'
    row = json.loads(p.read_text())
    row.update(consumed=True, state='STARTING', active_process=None)
    atomic_json(p, row)
    restarted = SearchSession(s.spec, spawn=spawn, identity=identity)
    row = restarted.start(s.spec['target'])
    assert not spawn.calls
    assert row['state'] == 'STOP_UNCONFIRMED' and row['active_process'] is None


def test_close_without_execution(entry):
    s, spawn = session(entry)
    row = s.close()
    assert row['executor_result'] == 'AGENT_NO_EXECUTION'
    assert row['stop_status'] == 'NOT_STARTED' and not spawn.calls


def test_separate_hosts_share_persistent_start_lock(entry):
    from tools.ranger_agent.session import SearchSession
    spec = make_spec(entry)
    count = entry[0] / 'spawn-count'
    def child():
        def spawn(*a, **k):
            with count.open('a') as stream: stream.write('spawn\n')
            return Process()
        SearchSession(spec, spawn=spawn, identity=identity).start(spec['target'])
    ctx = multiprocessing.get_context('fork')
    workers = [ctx.Process(target=child) for _ in range(4)]
    for p in workers: p.start()
    for p in workers:
        p.join(5); assert p.exitcode == 0
    assert count.read_text() == 'spawn\n'


def test_real_harmless_child_gets_one_sigint_and_writes_final_report(entry):
    from tools.ranger_agent.session import SearchSession
    spec = make_spec(entry)
    root = Path(spec['session_dir'])
    child = entry[0] / 'fake_executor.py'
    child.write_text('''import json,pathlib,signal,sys,time
p=pathlib.Path(sys.argv[1]);target=sys.argv[2];p.mkdir()
def finish(*_):
 r=dict(mode='local_ranger_explore',target_id='B',result='operator interrupt',stop_confirmed=True,evidence_write_failed=False,vlm_evidence_pending=False)
 (p/'result.json').write_text(json.dumps(r))
 e=dict(run_id='fake-run',sequence=1,target_id='B',target=target,event='SESSION_END',result='operator interrupt',stop_confirmed=True)
 (p/'events.jsonl').write_text(json.dumps(e)+'\\n')
 sys.exit(1)
signal.signal(signal.SIGINT,finish)
(p/'ready').touch()
while not (p/'test-finish').exists(): time.sleep(.01)
''')
    def spawn(argv, **kwargs):
        return subprocess.Popen([sys.executable, str(child), str(root / 'search'), spec['target']], **kwargs)
    s=SearchSession(spec, spawn=spawn)
    try:
        s.start(spec['target'])
        deadline=time.monotonic()+5
        while not (root/'search/ready').exists() and time.monotonic()<deadline: time.sleep(.01)
        assert (root/'search/ready').exists()
        row=s.close()
        assert row['process_exit_code']==1 and row['stop_status']=='CONFIRMED'
        assert row['active_process'] is False
    finally:
        if s.process and s.process.poll() is None:
            (root/'search/test-finish').touch();s.process.wait(timeout=5)


def test_old_output_directory_is_rejected_before_spawn(entry):
    s, spawn=session(entry)
    (Path(s.spec['session_dir'])/'search').mkdir()
    with pytest.raises(ValueError, match='output directory'):
        s.start(s.spec['target'])
    assert not spawn.calls


def test_foreign_observer_does_not_corrupt_owner_state(entry):
    from tools.ranger_agent.session import SearchSession
    s, spawn=session(entry);s.start(s.spec['target'])
    other=SearchSession(s.spec, spawn=spawn, identity=identity)
    assert other.status(s.spec['task_id'])['state']=='STOP_UNCONFIRMED'
    assert s.status(s.spec['task_id'])['state']=='RUNNING'


def test_referenced_evidence_missing_cannot_be_reported_ok(entry):
    s, spawn=session(entry);s.start(s.spec['target']);evidence(s)
    path=Path(s.spec['session_dir'])/'search/events.jsonl'
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    rows[0]['evidence_dir']=str(path.parent/'vlm/missing')
    path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    spawn.process.code=0
    row=s.status(s.spec['task_id'])
    assert row['executor_success'] is True
    assert row['evidence_status']=='INCOMPLETE'


def test_partial_event_waits_for_newline_before_finalization(entry):
    s, spawn=session(entry);s.start(s.spec['target'])
    path=Path(s.spec['session_dir'])/'search';path.mkdir()
    (path/'events.jsonl').write_text('{"run_id":')
    assert s.status(s.spec['task_id'])['state']=='RUNNING'
    with (path/'events.jsonl').open('a') as f:
        f.write(json.dumps('run-one')+',"sequence":1,"target_id":"B","target":'+json.dumps(s.spec['target'])+'}\n')
    report=dict(mode='local_ranger_explore',target_id='B',result='TARGET_APPROACHED',
                stop_confirmed=True,evidence_write_failed=False,vlm_evidence_pending=False)
    (path/'result.json').write_text(json.dumps(report))
    with (path/'events.jsonl').open('a') as f:
        f.write(json.dumps(dict(run_id='run-one',sequence=2,target_id='B',target=s.spec['target'],
            event='SESSION_END',result='TARGET_APPROACHED',stop_confirmed=True))+'\n')
    spawn.process.code=0
    assert s.status(s.spec['task_id'])['executor_success'] is True


def test_semantic_input_image_is_pinned_and_allowed(entry):
    import yaml
    from tools.ranger_agent.session import SearchSession
    catalog=entry[0]/'catalog';catalog.mkdir()
    image=catalog/'annotated.png';image.write_bytes(b'fixture image')
    semantic=catalog/'semantic.yaml';semantic.write_text('furniture: []\n')
    (catalog/'points.yaml').write_text('annotated_image: annotated.png\nsemantic_yaml: semantic.yaml\n')
    (entry[1]/'robot_ranger.yaml').write_text(yaml.safe_dump(dict(
        exploration_strategy='semantic_furniture',semantic_search={'observation_points':str(catalog/'points.yaml')})))
    spec=make_spec(entry);spawn=Spawn();s=SearchSession(spec,spawn=spawn,identity=identity)
    s.start(spec['target']);evidence(s)
    events=Path(spec['session_dir'])/'search/events.jsonl'
    rows=[json.loads(line) for line in events.read_text().splitlines()]
    rows[0].update(state='FURNITURE_REQUEST',image=str(image))
    events.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    spawn.process.code=0
    row=s.status(spec['task_id'])
    assert row['executor_success'] is True and row['evidence_status']=='REPORTED_OK'
    image.write_bytes(b'changed')
    with pytest.raises(ValueError,match='configuration changed'):
        from tools.ranger_agent.session import validate_inputs
        validate_inputs(spec)
    assert s.status(spec['task_id'])['evidence_status']=='INCOMPLETE'


def test_host_state_io_failure_still_cancels_owned_process_once(entry, monkeypatch):
    from tools.ranger_agent import session as module
    sent=[]
    s, spawn=session(entry,send_signal=lambda *args: sent.append(args))
    s.start(s.spec['target'])
    def disk_full(*args): raise OSError('disk full')
    monkeypatch.setattr(module,'atomic_json',disk_full)
    s.cancel(s.spec['task_id']);s.cancel(s.spec['task_id'])
    assert sent==[(spawn.process.pid,signal.SIGINT)]
    evidence(s,result='operator interrupt');spawn.process.code=1
    row=s.close()
    assert row['stop_status']=='CONFIRMED'
    assert row['evidence_status']=='INCOMPLETE' and row['framework_error']
    assert row['active_process'] is False
