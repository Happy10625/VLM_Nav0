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
    (repo / 'Cooperative_env.bash').write_text(
        '[[ "$1" == ranger ]] || return 9\n'
        'export REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\n'
        'export RANGER_LOCAL_OUTPUT="$REPO/build"\n'
        'export COOP_CONFIG="${2:-$REPO/config}"\n'
        'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n'
        'export CYCLONEDDS_URI="file://$COOP_CONFIG/local.xml"\n'
        'return "${ENV_EXIT:-0}"\n')
    cfg = repo / 'config'
    cfg.mkdir()
    map_dir = repo / 'shared_maps' / 'new map'
    map_dir.mkdir(parents=True)
    (map_dir / 'lab.yaml').write_text('image: lab.pgm\n')
    (map_dir / 'lab.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    (cfg / 'robot_ranger.yaml').write_text('map_path: ' + str(map_dir / 'lab.yaml') + '\n')
    for name in ('alignment', 'scene', 'robot_go2'):
        (cfg / (name + '.yaml')).write_text('{}\n')
    (repo / 'scripts/start_hardware.sh').write_text(
        'echo hardware >> "$TRACE"\n'
        'printf "ENV %s %s %s\\n" "${ROS_DOMAIN_ID:-UNSET}" "${RMW_IMPLEMENTATION:-UNSET}" "${CYCLONEDDS_URI:-UNSET}" >> "$TRACE"\n'
        'exit "${HARDWARE_EXIT:-0}"\n')
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
           'COOP_CONFIG': str(cfg),
           'PYTHONPATH': str(ROOT / 'cooperative_ws/src/multi_agent_nav') + os.pathsep + os.environ.get('PYTHONPATH', ''),
           'DISPLAY': ':99', 'TRACE': str(trace), 'NODES': str(nodes),
           'FULL_NAV': '\n'.join('/' + n for n in (
               'map_server', 'amcl', 'controller_server', 'planner_server',
               'bt_navigator', 'behavior_server', 'smoother_server',
               'waypoint_follower', 'ranger_command_guard', 'lifecycle_manager_navigation',
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
    expected_map = Path(terminals[0][-1].removeprefix('map:='))
    assert str(expected_map).endswith('/shared_maps/new map/lab.yaml')
    assert expected_map.is_file()
    assert any(item.endswith('start_rviz.sh') for item in terminals[1])
    assert terminals[1][-1].endswith('light rviz.rviz')
    assert terminals[2][-1].endswith('monitor_ranger.sh')
    assert trace.read_text().splitlines().index('hardware') < next(
        i for i, line in enumerate(trace.read_text().splitlines()) if '--start-navigation' in line)
    assert '2D Pose Estimate' in result.stdout
    assert 'check_ranger.sh' in result.stdout
    assert 'ENV 0 rmw_cyclonedds_cpp file://' in trace.read_text()
    assert all('Cooperative_env.bash' in call[call.index('-c') + 1] for call in terminals)


def test_unreadable_configured_map_never_starts_hardware(startup):
    run, env, trace = startup
    (Path(env['COOP_CONFIG']) / 'robot_ranger.yaml').write_text('map_path: /missing/map.yaml\n')
    result, calls = run()
    assert result.returncode != 0
    assert not any(call[0] == 'gnome-terminal' for call in calls)
    assert not trace.exists() or 'hardware' not in trace.read_text().splitlines()


def test_environment_failure_never_starts_components(startup):
    run, _, trace = startup
    result, calls = run(ENV_EXIT='7')
    assert result.returncode == 7
    assert not calls and not trace.exists()


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


def test_old_velocity_smoother_is_refused_before_starting_ranger(startup):
    run,_,trace=startup
    result,calls=run('/velocity_smoother\n')
    assert result.returncode==1
    assert not any(call[0]=='gnome-terminal' for call in calls)
    assert 'hardware' not in trace.read_text().splitlines()


def test_help_does_not_need_environment_or_start_processes():
    script = ROOT / 'scripts/start_ranger_stack.sh'
    assert script.is_file(), '缺少硬件/Nav2/RViz统一启动入口'
    result = subprocess.run(['bash', str(script), '--help'], capture_output=True, text=True)
    assert result.returncode == 0
    assert 'RVIZ' in result.stdout


def enable_keepout_fixture(env):
    import yaml
    from PIL import Image
    from vlm_nav.keepout import write_mask,KeepoutMask
    cfg=Path(env['COOP_CONFIG'])/'robot_ranger.yaml'
    robot=yaml.safe_load(cfg.read_text());source=Path(robot['map_path'])
    Image.new('L',(80,60),254).save(source.parent/'lab.pgm')
    source.write_text(yaml.safe_dump(dict(image='lab.pgm',resolution=.1,origin=[0.,0.,0.],
        mode='trinary',negate=0,free_thresh=.25,occupied_thresh=.65)))
    output=write_mask(source,[[1,1],[6,1],[6,5],[1,5]],source.parent/'keepout_mask.yaml',.5)
    robot.update(keepout_mask_yaml=str(output),keepout_mask_digest=KeepoutMask(source,output).digest)
    cfg.write_text(yaml.safe_dump(robot))
    return robot


def test_keepout_config_is_passed_to_nav2_and_old_stack_is_not_reused(startup):
    run,env,trace=startup
    robot=enable_keepout_fixture(env)
    extra='\n/keepout_mask_server\n/costmap_filter_info_server\n/lifecycle_manager_keepout\n'
    result,calls=run(FULL_NAV=env['FULL_NAV']+extra)
    assert result.returncode==0,result.stdout+result.stderr
    nav=next(call for call in calls if '--start-navigation' in call)
    assert 'keepout_mask_yaml:='+robot['keepout_mask_yaml'] in nav
    assert 'keepout_mask_digest:='+robot['keepout_mask_digest'] in nav


def test_existing_navigation_without_keepout_requires_explicit_restart(startup):
    run,env,trace=startup
    enable_keepout_fixture(env)
    result,calls=run(initial=env['FULL_NAV'])
    assert result.returncode!=0 and '部分定位/Nav2/TF节点' in result.stderr
    assert not any(call[0]=='gnome-terminal' for call in calls)
    assert 'hardware' not in trace.read_text().splitlines()
