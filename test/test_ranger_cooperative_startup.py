"""Run the interactive Ranger workflow without ROS, hardware or desktop windows."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/start_ranger_cooperative.sh'
CONFIRMATIONS = '4\n'


@pytest.fixture
def workflow(tmp_path):
    assert SCRIPT.is_file(), '缺少 Ranger 协同启动入口'
    repo = tmp_path / 'repo with spaces'
    scripts = repo / 'scripts'
    scripts.mkdir(parents=True)
    shutil.copy2(SCRIPT, scripts / SCRIPT.name)
    cfg = repo / 'field config'
    cfg.mkdir()
    (cfg / 'bridge_ranger_pending.yaml').write_text('name: ranger_cooperative_task_bridge\n')
    (repo / 'Cooperative_env.bash').write_text(
        '[[ "$1" == ranger ]] || return 9\n'
        'export REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\n'
        'export COOP_CONFIG="${2:-$REPO/field config}"\n'
        'export RANGER_LOCAL_OUTPUT="$REPO/build"\n'
        'export ACCEPTANCE_RUN="$REPO/evidence"\n'
        'export COOP_BRIDGE_YAML="$COOP_CONFIG/bridge_ranger_pending.yaml"\n'
        'mkdir -p "$ACCEPTANCE_RUN"\n'
        'return "${ENV_EXIT:-0}"\n')
    (scripts / 'cooperative.sh').write_text('''
printf '%s\n' "$1 $2" >> "$TRACE"
[[ "$1" == ranger ]] || exit 9
case "$2" in
  preflight) echo '{"live_gates": [], "motion_sent": false}'; exit "${PREFLIGHT_EXIT:-0}";;
  bridge) echo /ranger_cooperative_task_bridge_0 >> "$NODES";;
  agent) echo /multi_agent_ranger_agent >> "$NODES";;
  wait) echo 'PREPARATION_OK'; exit "${WAIT_EXIT:-0}";;
  state|peer) echo "state display: $COOP_CONFIG";;
  *) exit 9;;
esac
''')
    (scripts / 'start_ranger_stack.sh').write_text(
        'echo hardware >> "$TRACE"\nexit "${HARDWARE_EXIT:-0}"\n')
    (scripts / 'check_ranger.sh').write_text(
        'printf "health %s\\n" "$*" >> "$TRACE"\nexit "${HEALTH_EXIT:-0}"\n')
    (scripts / 'terminal_runner.sh').write_text('exec bash "$@"\n')
    tools = tmp_path / 'bin'
    tools.mkdir()
    (tools / 'ros2').write_text('''#!/bin/bash
[[ "$*" == 'node list --no-daemon --spin-time 2' ]] || exit 9
exit_code="${DISCOVERY_EXIT:-0}"
cat "$NODES"
exit "$exit_code"
''')
    (tools / 'gnome-terminal').write_text('''#!/usr/bin/python3
import json, os, subprocess, sys
with open(os.environ['TRACE'], 'a') as f:
    f.write(json.dumps(sys.argv[1:])+'\\n')
args = sys.argv[1:]
if args[-1] == os.environ.get('FAIL_ACTION'): sys.exit(1)
sys.exit(subprocess.run(args[args.index('--')+1:], stdin=subprocess.DEVNULL).returncode)
''')
    for tool in tools.iterdir():
        tool.chmod(0o755)
    trace = tmp_path / 'trace'
    nodes = tmp_path / 'nodes'
    env = {**os.environ, 'PATH': str(tools)+':'+os.environ['PATH'],
           'DISPLAY': ':99', 'COOP_CONFIG': str(cfg), 'TRACE': str(trace), 'NODES': str(nodes)}

    def run(answers=CONFIRMATIONS, existing='', **extra):
        nodes.write_text(existing)
        result = subprocess.run(['bash', str(scripts / SCRIPT.name)], input=answers,
                                env={**env, **extra}, text=True, capture_output=True, timeout=10)
        calls = trace.read_text().splitlines() if trace.exists() else []
        return result, calls, repo
    return run


def test_numbered_workflow_preserves_config_and_never_starts_task(workflow):
    result, calls, repo = workflow()
    assert result.returncode == 0, result.stdout + result.stderr
    actions = [c for c in calls if not c.startswith('[')]
    assert actions == ['ranger preflight', 'hardware',
                       f'health --config-dir {repo}/field config',
                       'ranger bridge', 'ranger agent', 'ranger state', 'ranger peer']
    logs = list((repo / 'evidence').glob('cooperative-startup-*/startup.log'))
    assert len(logs) == 1
    log = logs[0].read_text()
    assert '[确认 4]' in log
    assert log.count('[确认 ')==1
    assert '不等待Go2' in log
    assert 'ranger wait' not in calls
    assert 'state display: ' + str(repo / 'field config') in (
        logs[0].parent / 'state.log').read_text()
    assert 'ranger start' not in calls and 'go2 start' not in calls


@pytest.mark.parametrize('extra', [{'PREFLIGHT_EXIT': '1'}, {'ENV_EXIT': '1'},
                                 {'DISCOVERY_EXIT': '1'}])
def test_early_failure_does_not_start_hardware_or_terminals(workflow, extra):
    result, calls, _ = workflow(**extra)
    assert result.returncode != 0
    assert 'hardware' not in calls and not any(c.startswith('[') for c in calls)


def test_wrong_step_and_eof_cannot_confirm_localization(workflow):
    result,calls,_=workflow(answers='3\nyes\n\n')
    assert result.returncode!=0
    assert calls==['ranger preflight','hardware']


def test_wrong_step_reprompts_until_exact_number(workflow):
    result, calls, _ = workflow(answers='yes\n'+CONFIRMATIONS)
    assert result.returncode == 0, result.stdout + result.stderr
    assert calls.count('hardware') == 1


@pytest.mark.parametrize('extra', [{'HARDWARE_EXIT': '1'}, {'HEALTH_EXIT': '1'}])
def test_unhealthy_local_stack_cannot_start_agent_or_bridge(workflow, extra):
    result, calls, _ = workflow(**extra)
    assert result.returncode != 0
    assert not any(c.startswith('[') for c in calls)


def test_late_peer_or_go2_auto_start_does_not_block_local_startup(workflow):
    result,calls,_=workflow(WAIT_EXIT='99')
    assert result.returncode==0,result.stdout+result.stderr
    assert 'ranger agent' in calls and 'ranger wait' not in calls
    assert '[完成]' in result.stdout
    assert 'Go2完整入口' in result.stdout and '不重复start' in result.stdout


@pytest.mark.parametrize('answers',['','3\n','\n','yes\n'])
def test_missing_localization_confirmation_cannot_start_task_components(workflow,answers):
    result,calls,_=workflow(answers=answers)
    assert result.returncode!=0 and '[完成]' not in result.stdout
    assert 'ranger bridge' not in calls and 'ranger agent' not in calls


def test_active_agent_requires_operator_to_end_old_process_first(workflow):
    result, calls, _ = workflow(existing='/multi_agent_ranger_agent\n')
    assert result.returncode != 0
    assert 'hardware' not in calls


def test_existing_unique_task_bridge_is_reused(workflow):
    result, calls, _ = workflow(existing='/ranger_cooperative_task_bridge_0\n')
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'ranger bridge' not in calls and 'ranger agent' in calls


def test_duplicate_task_bridges_do_not_launch_another_process(workflow):
    result, calls, _ = workflow(existing='/ranger_cooperative_task_bridge_0\n' * 2)
    assert result.returncode != 0
    assert 'ranger bridge' not in calls and 'ranger agent' not in calls


def test_help_does_not_source_environment():
    result = subprocess.run(['bash', str(SCRIPT), '--help'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert '步骤号' in result.stdout



@pytest.mark.parametrize('action',['bridge','agent','state','peer'])
def test_terminal_launch_failure_cannot_report_completion(workflow,action):
    result,calls,_=workflow(FAIL_ACTION=action)
    assert result.returncode!=0 and '[完成]' not in result.stdout
    assert 'ranger start' not in calls and 'go2 start' not in calls
    if action=='bridge':assert 'ranger agent' not in calls
