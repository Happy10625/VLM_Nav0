"""Exercise startup orchestration without starting ROS, hardware, or a GUI."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def startup(tmp_path):
    script = ROOT / 'scripts/start_ranger_stack.sh'
    assert script.is_file(), '缺少硬件/Nav2/RViz统一启动入口'
    repo = tmp_path / 'test repo'
    (repo / 'scripts').mkdir(parents=True)
    shutil.copy2(script, repo / 'scripts' / script.name)
    (repo / 'RangerVLM_env.bash').write_text(
        'export REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\n'
        'export RANGER_LOCAL_OUTPUT="$REPO/build"\n')
    (repo / 'scripts/start_hardware.sh').write_text(
        'echo hardware >> "$TRACE"\nexit "${HARDWARE_EXIT:-0}"\n')
    for name in ('terminal_runner.sh', 'start_ranger_local.sh', 'start_rviz.sh', 'monitor_ranger.sh'):
        (repo / 'scripts' / name).touch()
    config = repo / 'light rviz.rviz'
    config.write_text('test fixture')
    tools = tmp_path / 'bin'
    tools.mkdir()
    fake = '''#!/usr/bin/python3
import json, os
from pathlib import Path
import sys
name=Path(sys.argv[0]).name
args=sys.argv[1:]
with open(os.environ['TRACE'],'a') as f:f.write(json.dumps([name,*args])+'\\n')
if name=='ros2':
    if args[:2]==['node','list']:print(Path(os.environ['NODES']).read_text())
elif name=='gnome-terminal':
    if '--start-navigation' in args:
        Path(os.environ['NODES']).write_text(os.environ['FULL_NAV'])
'''
    for name in ('ros2', 'gnome-terminal'):
        path = tools / name
        path.write_text(fake)
        path.chmod(0o755)
    nodes = tmp_path / 'nodes'
    nodes.write_text('')
    trace = tmp_path / 'trace'
    env = {**os.environ, 'PATH': str(tools) + ':' + os.environ['PATH'],
           'DISPLAY': ':99', 'TRACE': str(trace), 'NODES': str(nodes),
           'FULL_NAV': '\n'.join('/' + n for n in (
               'map_server', 'amcl', 'controller_server', 'planner_server',
               'bt_navigator', 'behavior_server', 'smoother_server',
               'waypoint_follower', 'velocity_smoother', 'lifecycle_manager_navigation',
               'lifecycle_manager_localization', 'fastlio_odom_adapter',
               'odom_to_fastlio_world', 'base_to_camera', 'obstacle_cloud_filter'))}

    def run(initial='', **overrides):
        nodes.write_text(initial)
        result = subprocess.run(['bash', str(repo / 'scripts' / script.name), str(config)],
                                env={**env, **overrides}, capture_output=True, text=True, timeout=10)
        calls = [json.loads(line) for line in trace.read_text().splitlines() if line.startswith('[')] if trace.exists() else []
        return result, calls
    return run, env, trace


def test_cold_start_hardware_then_nav_then_rviz(startup):
    run, _, trace = startup
    result, calls = run()
    assert result.returncode == 0, result.stdout + result.stderr
    terminals = [call for call in calls if call[0] == 'gnome-terminal']
    assert len(terminals) == 3
    assert '--start-navigation' in terminals[0]
    assert any(item.endswith('start_rviz.sh') for item in terminals[1])
    assert terminals[1][-1].endswith('light rviz.rviz')
    assert terminals[2][-1].endswith('monitor_ranger.sh')
    assert trace.read_text().splitlines().index('hardware') < next(
        i for i, line in enumerate(trace.read_text().splitlines()) if '--start-navigation' in line)
    assert '2D Pose Estimate' in result.stdout
    assert 'check_ranger.sh' in result.stdout


def test_existing_complete_nav_and_rviz_are_reused(startup):
    run, env, _ = startup
    result, calls = run(env['FULL_NAV'] + '\n/rviz2\n/ranger_diagnostic\n')
    assert result.returncode == 0, result.stdout + result.stderr
    assert not any(call[0] == 'gnome-terminal' for call in calls)


def test_existing_stack_adds_only_missing_diagnostic_terminal(startup):
    run, env, _ = startup
    result, calls = run(env['FULL_NAV'] + '\n/rviz2\n')
    terminals = [call for call in calls if call[0] == 'gnome-terminal']
    assert result.returncode == 0
    assert len(terminals) == 1 and terminals[0][-1].endswith('monitor_ranger.sh')


@pytest.mark.parametrize('nodes', ['/amcl\n', '/vlm_nav\n',
                                 '/amcl\n/amcl\n', '/multi_agent_ranger_check_motion\n'])
def test_partial_duplicate_or_active_task_never_starts_another_stack(startup, nodes):
    run, _, trace = startup
    result, calls = run(nodes)
    assert result.returncode == 1
    assert not any(call[0] == 'gnome-terminal' for call in calls)
    assert 'hardware' not in trace.read_text().splitlines()


def test_hardware_start_failure_does_not_launch_nav_or_rviz(startup):
    run, _, _ = startup
    result, calls = run(HARDWARE_EXIT='1')
    assert result.returncode == 1
    assert not any(call[0] == 'gnome-terminal' for call in calls)


def test_help_does_not_need_environment_or_start_processes():
    script = ROOT / 'scripts/start_ranger_stack.sh'
    assert script.is_file(), '缺少硬件/Nav2/RViz统一启动入口'
    result = subprocess.run(['bash', str(script), '--help'], capture_output=True, text=True)
    assert result.returncode == 0
    assert 'RVIZ' in result.stdout
