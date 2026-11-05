"""Optional host for the pinned RoboRSI CLI planner; no ROS imports."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.ranger_agent.session import (
    SearchSession, atomic_json, file_digest, framework_environment, prepare_spec,
)

SKILL = Path(__file__).parent / 'skills/find_target/SKILL.md'


def validate_budget(spec):
    cfg = spec['framework_config']
    if type(cfg['planner_timeout_s']) is not int or cfg['planner_timeout_s'] <= 0:
        raise ValueError('positive integer planner budget required')
    if spec['effective_timeout'] > cfg['planner_timeout_s']:
        raise ValueError('search timeout exceeds planner budget; change outer configuration explicitly')
    if cfg['action_timeout_s'] != 30:
        raise ValueError('management action budget must be 30 seconds')


def cli_policy(cfg):
    """Disable inherited named MCP services without reading any authentication files."""
    import tomli
    overrides = dict(cfg.get('cli_overrides', {}))
    path = Path(os.environ.get('CODEX_HOME', str(Path.home() / '.codex'))) / 'config.toml'
    if path.exists():
        data = tomli.loads(path.read_text())
        for name in data.get('mcp_servers', {}):
            if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
                raise ValueError('unsupported inherited MCP server name')
            overrides[f'mcp_servers.{name}.enabled'] = False
    return dict(cli_args=cfg.get('cli_args', []), config_overrides=overrides)


def verify_real_backend(spec):
    cfg = spec.get('framework_config', {})
    if cfg.get('real_backend_enabled') is not True:
        raise ValueError('real backend disabled; stage B/C authorization required')
    proof_path = cfg.get('capability_verification')
    if not proof_path:
        raise ValueError('capability verification missing; real backend remains disabled')
    proof = json.loads(Path(proof_path).read_text())
    policy = cli_policy(cfg)
    expected = dict(cli_path=str(Path(cfg['cli_path']).resolve()), cli_sha256=file_digest(cfg['cli_path']),
        cli_version=cfg['cli_version'], framework_commit=cfg['framework_commit'],
        policy_sha256=hashlib.sha256(json.dumps(policy, sort_keys=True).encode()).hexdigest())
    if (proof.get('verified') is not True or any(proof.get(k) != v for k, v in expected.items())
            or expected['cli_sha256'] != cfg['cli_sha256']):
        raise ValueError('capability verification does not match current binary/configuration')
    if file_digest(proof['evidence_path']) != proof.get('evidence_sha256'):
        raise ValueError('capability verification evidence changed')
    return expected


def codex_config(spec):
    from roborsi.planner.codex_cli_config import CodexCliConfig
    cfg = spec['framework_config']
    return CodexCliConfig(expected_version=cfg['cli_version'], executable=str(Path(cfg['cli_path']).resolve()),
        expected_sha256=cfg['cli_sha256'], inherit_env=False, env=framework_environment(os.environ),
        tools='mcp', **cli_policy(cfg))


def preflight_framework(spec):
    validate_budget(spec)
    verify_real_backend(spec)
    cfg = spec['framework_config']
    import roborsi
    root = Path(cfg['framework_root']).resolve()
    if root not in Path(roborsi.__file__).resolve().parents:
        raise ValueError('framework import does not match configured checkout')
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], text=True).strip()
    if head != cfg['framework_commit'] or dirty:
        raise ValueError('framework checkout identity changed')
    from roborsi.planner.codex_cli import preflight_codex_cli
    return preflight_codex_cli(codex_config(spec), repo_root=spec['session_dir'],
                              env=framework_environment(os.environ))


def local_tools(session):
    from roborsi.embodied_agent import LocalToolSpec
    from roborsi.tools.toolkit import ToolResult
    tools = []
    for name, field, method, description in (
        ('start_find_target', 'target', session.start, 'Start the fixed target once, verbatim.'),
        ('get_task_status', 'task_id', session.status, 'Read this task state; never restart it.'),
        ('cancel_task', 'task_id', session.cancel, 'Request cancellation; confirmation is separate.'),
    ):
        def handle(arguments, name=name, field=field, method=method):
            try:
                if not isinstance(arguments, dict) or set(arguments) != {field} or not isinstance(arguments[field], str):
                    raise ValueError('tool accepts exactly one string field: ' + field)
                row = method(arguments[field])
                # Do not expose process argv, configuration or inherited environment to the model.
                result = {k: row.get(k) for k in ('task_id', 'state', 'reason', 'process_exit_code',
                    'executor_result', 'stop_status', 'executor_success', 'evidence_status',
                    'framework_error', 'active_process', 'evidence_path', 'stop_confirmed',
                    'evidence_write_failed', 'vlm_evidence_pending')}
            except ValueError as error:
                result = {'error': str(error)}
            except Exception as error:
                result = {'error': type(error).__name__}
            return ToolResult(name=name, result=result)
        tools.append(LocalToolSpec(name=name, description=description, handler=handle,
            input_schema=dict(type='object', properties={field: {'type': 'string'}},
                              required=[field], additionalProperties=False)))
    return tools


def run_session(spec, *, session_factory=SearchSession, preflight=preflight_framework):
    session = session_factory(spec)
    stop = threading.Event()
    previous = {}
    episode = None
    try:
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[number] = signal.getsignal(number)
            signal.signal(number, lambda *_: stop.set())
        identity = preflight(spec)
        atomic_json(Path(spec['session_dir']) / 'framework-identity.json', identity)
        from roborsi.embodied_agent import EmbodiedAgent
        cfg = spec['framework_config']
        tools = local_tools(session)
        handlers = {tool.name: tool.handler for tool in tools}
        agent = EmbodiedAgent(mcp_servers=[], local_tools=tools, planner='codex_cli',
            output_dir=Path(spec['session_dir']) / 'planner', working_dir=spec['session_dir'],
            model=cfg['model'], reasoning_effort=cfg['reasoning_effort'],
            max_turns=None, max_tokens=None, planner_timeout_s=cfg['planner_timeout_s'],
            action_timeout_s=cfg['action_timeout_s'], codex_cli=codex_config(spec))
        # start_episode shares static skills loading with run and exposes cancellation.
        episode = agent.start_episode(spec['target'], system_prompt='Use only the fixed Ranger task tools. '
            'The host, not the model, determines execution, stop and evidence outcomes.',
            skills=[SKILL], deferred_tools=list(handlers), include_finish=True)
        while not episode.done:
            if stop.is_set():
                session.cancel(spec['task_id'])
                try:
                    episode.close(timeout=0)
                except TimeoutError:
                    pass
                break
            try:
                call = episode.next_call(timeout=.1)
            except TimeoutError:
                continue
            if stop.is_set():
                continue
            if call is not None:
                episode.complete(call, handlers[call.name](call.arguments))
        if not stop.is_set():
            result = episode.wait()
            if result.error or result.error_kind:
                session.framework_failed('PLANNER_FAILED:' + (result.error_kind or 'unknown'))
        else:
            session.framework_failed('HOST_SIGNAL')
    except Exception as error:
        session.framework_failed('FRAMEWORK_FAILED:' + type(error).__name__)
    finally:
        try:
            if episode is not None:
                try:
                    episode.close(timeout=0)
                except TimeoutError:
                    pass
            row = session.close()
            if episode is not None:
                try:
                    episode.close(timeout=35)
                except TimeoutError:
                    session.framework_failed('CLI_CLEANUP_UNCONFIRMED')
                    row = session.close()
        finally:
            for number, handler in previous.items():
                signal.signal(number, handler)
    return row


def source_identity(repo):
    names = ['scripts/start_ranger_single_robot.sh', 'RangerVLM_env.bash', 'scripts/common.sh']
    names += ['cooperative_ws/src/multi_agent_nav/multi_agent_nav/' + n for n in (
        'ranger_check.py', 'go2_check.py', 'go2_exploration.py', 'adapters/ranger.py',
        'adapters/nav2.py', 'adapters/vision.py', 'single_robot_approach.py', 'generated_candidates.py')]
    names += ['vlm_nav/' + n for n in ('ranger_single_robot_execution.py', 'vlm_navigator.py', 'vlm_client.py')]
    return dict(head=subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
                files={n: dict(sha256=file_digest(repo / n), resolved=str((repo / n).resolve())) for n in names})


def monitor(directory):
    path = Path(directory) / 'session.json'
    previous = None
    try:
        while True:
            try:
                row = json.loads(path.read_text())
                value = {k: row.get(k) for k in ('state', 'reason', 'executor_result', 'stop_status',
                    'evidence_status', 'framework_error', 'active_process')}
                if value != previous:
                    print(json.dumps(value, ensure_ascii=False), flush=True)
                    previous = value
            except (OSError, ValueError):
                pass
            time.sleep(.5)
    except KeyboardInterrupt:
        return


def open_diagnostics(directory):
    terminal = shutil.which('x-terminal-emulator')
    if terminal and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        try:
            subprocess.Popen([terminal, '-e', '/usr/bin/python3', str(Path(__file__).resolve()),
                              '--monitor', str(directory)], stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        except OSError:
            pass  # Headless callers retain the same diagnostic files; this is not a motion gate.


def bootstrap(options, repo):
    import yaml
    directory = repo / 'build/ranger-roborsi-runs' / uuid.uuid4().hex
    directory.mkdir(parents=True, mode=0o700)
    log_path = directory / 'diagnostics.log'
    print('诊断日志：' + str(log_path), flush=True)
    print('状态文件：' + str(directory / 'session.json'), flush=True)
    open_diagnostics(directory)
    with log_path.open('a') as log:
        try:
            cfg = yaml.safe_load((repo / 'tools/ranger_agent/config.yaml').read_text())
            if not options['allow_motion']:
                raise ValueError('original --allow-motion is required')
            if cfg.get('real_backend_enabled') is not True:
                raise ValueError('real backend disabled; stage B/C remains unverified')
            # The environment travels only through memory/private FD, never a log or manifest.
            prepared = subprocess.run(['/bin/bash', '-c',
                'source "$1" local >&2 || exit; exec /usr/bin/python3 -c '\
                '\'import os,json; print(json.dumps(dict(os.environ)))\'',
                'ranger-environment', str(repo / 'RangerVLM_env.bash')],
                stdout=subprocess.PIPE, stderr=log, check=True)
            spec = prepare_spec(options, repo, json.loads(prepared.stdout), session_dir=directory)
            spec['framework_config'] = cfg
            spec['source_identity'] = source_identity(repo)
            validate_budget(spec)
            fd = os.memfd_create('ranger-private-environment', 0)
            with os.fdopen(os.dup(fd), 'w') as payload:
                json.dump(spec, payload, ensure_ascii=False)
            os.lseek(fd, 0, os.SEEK_SET)
            os.dup2(log.fileno(), 1); os.dup2(log.fileno(), 2)
            os.execve(cfg['framework_python'], [cfg['framework_python'], str(Path(__file__).resolve()),
                      '--spec-fd', str(fd)], framework_environment(spec['search_env']))
        except Exception as error:
            reason = str(error) if isinstance(error, ValueError) else type(error).__name__
            atomic_json(directory / 'session.json', dict(state='FINISHED', reason=reason,
                executor_result='AGENT_NO_EXECUTION', executor_success=False, active_process=False,
                stop_status='NOT_STARTED', evidence_status='UNKNOWN', framework_error=type(error).__name__))
            log.write(reason + '\n')
            return 1


def main():
    parser = argparse.ArgumentParser(description='可选 RoboRSI 入口；阶段 A 真实后端关闭，无自动重试。')
    parser.add_argument('--allow-motion', action='store_true')
    parser.add_argument('--target-id', choices=('A', 'B', 'C'), default='B')
    parser.add_argument('--target')
    parser.add_argument('--config-dir')
    parser.add_argument('--timeout', type=float)
    parser.add_argument('--strategy', choices=('fixed_viewpoints', 'semantic_furniture', 'generated_candidates_vlm'))
    parser.add_argument('--search-hint')
    parser.add_argument('--bootstrap', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--spec-fd', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--monitor', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.monitor:
        monitor(args.monitor)
        return 0
    if args.spec_fd is not None:
        with os.fdopen(args.spec_fd) as payload:
            spec = json.load(payload)
        row = run_session(spec)
        return 0 if row['executor_success'] and row['evidence_status'] == 'REPORTED_OK' and not row['framework_error'] else 1
    return bootstrap(vars(args), Path(__file__).resolve().parents[2])


if __name__ == '__main__':
    raise SystemExit(main())
