"""Manual cooperative commands and incoming Go2 sync compatibility, without ROS nodes."""
from dataclasses import replace
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_environment_uses_local_mode_preserves_keys_and_nounset(tmp_path):
    repo=tmp_path/'env repo';repo.mkdir()
    shutil.copy2(ROOT/'Cooperative_env.bash',repo/'Cooperative_env.bash')
    cfg=repo/'config';cfg.mkdir()
    for name in ('cyclonedds_ranger_local.xml','cyclonedds_ranger_multi_agent_bridge.xml','bridge_ranger_pending.yaml'):
        (cfg/name).touch()
    (repo/'RangerVLM_env.bash').write_text('[[ "${1:-}" == local ]] || return 9\nexport RANGER_CONFIG="$FIXTURE_CONFIG"\n')
    result=subprocess.run(['bash','-c','set -u; source "$1/Cooperative_env.bash" ranger || exit; [[ "$-" == *u* && "$DASHSCOPE_API_KEY" == test_only && "$ROS_DOMAIN_ID" == 0 && "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]',
        'test',str(repo)],env={**os.environ,'FIXTURE_CONFIG':str(cfg),'DASHSCOPE_API_KEY':'test_only'},capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert 'test_only' not in result.stdout+result.stderr


def test_go2_environment_loads_independent_overlay_last(tmp_path):
    repo=tmp_path/'go2';repo.mkdir();(repo/'config').mkdir()
    shutil.copy2(ROOT/'Cooperative_env.bash',repo/'Cooperative_env.bash')
    cfg=repo/'task config';cfg.mkdir()
    (cfg/'bridge_go2_pending.yaml').touch()
    for name in ('cyclonedds_go2.xml','cyclonedds_go2_multi_agent_bridge.xml'):(repo/'config'/name).touch()
    overlay=repo/'build/go2-independent-c-20261020/install';overlay.mkdir(parents=True)
    (repo/'CoVLM_env.bash').write_text('export GO2_EXPLORE_CONFIG="$FIXTURE_CONFIG"\nexport OVERLAY_ORDER=old\n')
    (overlay/'local_setup.bash').write_text('export OVERLAY_ORDER=new\n')
    result=subprocess.run(['bash','-c','source "$1/Cooperative_env.bash" go2 || exit; [[ "$OVERLAY_ORDER" == new && "$GO2_NET_IFACE" == enp4s0 ]]',
        'test',str(repo)],env={**os.environ,'FIXTURE_CONFIG':str(cfg),'COOP_SETUP_FILE':''},capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_help_needs_no_environment():
    result = subprocess.run(['bash', str(ROOT/'scripts/cooperative.sh'), '--help'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'preflight' in result.stdout and 'pause' in result.stdout


@pytest.fixture
def command_fixture(tmp_path):
    repo = tmp_path/'manual setup'
    (repo/'scripts').mkdir(parents=True)
    shutil.copy2(ROOT/'scripts/cooperative.sh', repo/'scripts/cooperative.sh')
    (repo/'Cooperative_env.bash').write_text('export COOP_SIDE="$1"\nexport ROS_DOMAIN_ID=0\nexport COOP_CONFIG="$FIXTURE_CONFIG"\nexport COOP_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\nexport COOP_BRIDGE_DDS_XML="$FIXTURE_CONFIG/bridge.xml"\nexport COOP_BRIDGE_YAML="$FIXTURE_CONFIG/bridge.yaml"\n')
    cfg=tmp_path/'config';cfg.mkdir()
    (cfg/'scene.yaml').write_text('scenario_id: lab\n')
    tools=tmp_path/'bin';tools.mkdir()
    (tools/'ros2').write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$TRACE"\nif [[ "$1 $2" == "node list" ]]; then printf "%s\\n" "${NODES:-}"; else printf "ENV %s %s\\n" "${ROS_DOMAIN_ID-UNSET}" "${CYCLONEDDS_URI-}" >> "$TRACE"; fi\n')
    (tools/'ros2').chmod(0o755)
    # External Python/ROS validation is isolated here; real preflight is tested separately.
    (tools/'python3').write_text('#!/bin/bash\ncat >/dev/null\nif [[ "${PYTHON_MODE:-}" == scenario ]]; then echo lab; else exit "${PREFLIGHT_EXIT:-0}"; fi\n')
    (tools/'python3').chmod(0o755)
    trace=tmp_path/'trace'
    env={**os.environ,'PATH':str(tools)+':'+os.environ['PATH'],'TRACE':str(trace),'FIXTURE_CONFIG':str(cfg)}
    def run(side,action,**extra):
        result=subprocess.run(['bash',str(repo/'scripts/cooperative.sh'),side,action],env={**env,**extra},capture_output=True,text=True,timeout=5)
        calls=trace.read_text().splitlines() if trace.exists() else []
        return result,calls
    return run


def test_agent_uses_new_task_node_and_never_starts_task(command_fixture):
    result,calls=command_fixture('ranger','agent')
    assert result.returncode==0,result.stderr
    assert any('run multi_agent_nav multi_agent_agent_node' in c for c in calls)
    assert not any('multi_agent_comm' in c or 'multi_agent_start_task' in c for c in calls)


def test_duplicate_agent_is_rejected(command_fixture):
    result,calls=command_fixture('ranger','agent',NODES='/multi_agent_ranger_agent')
    assert result.returncode!=0
    assert not any('run multi_agent_nav' in c for c in calls)


def test_coordinator_can_attach_to_existing_go2_agent_and_its_observer(command_fixture):
    result,calls=command_fixture('go2','coordinator',NODES='/multi_agent_go2_agent\n/multi_agent_go2_rgbd')
    assert result.returncode==0,result.stderr
    assert any('run multi_agent_nav multi_agent_coordinator_node' in c for c in calls)


@pytest.mark.parametrize('action',['coordinator','start','pause','watch'])
def test_go2_only_actions_are_rejected_on_ranger(command_fixture,action):
    result,calls=command_fixture('ranger',action)
    assert result.returncode!=0 and not calls


def test_failed_live_preflight_cannot_start_task(command_fixture):
    result,calls=command_fixture('go2','start',PREFLIGHT_EXIT='1')
    assert result.returncode!=0 and not calls


def test_pause_does_not_depend_on_preflight(command_fixture):
    result,calls=command_fixture('go2','pause',PREFLIGHT_EXIT='1')
    assert result.returncode==0,result.stderr
    assert any('multi_agent_start_task pause' in c for c in calls)


def test_explicit_start_uses_configured_scenario(command_fixture):
    result,calls=command_fixture('go2','start',PYTHON_MODE='scenario')
    assert result.returncode==0,result.stderr
    assert any('multi_agent_start_task start --scenario lab' in c for c in calls)
    assert not any('multi_agent_agent_node' in c for c in calls)


def test_bridge_unsets_default_domain_and_uses_dual_domain_xml(command_fixture):
    result,calls=command_fixture('ranger','bridge',PYTHON_MODE='scenario')
    assert result.returncode==0,result.stderr
    assert any('run domain_bridge domain_bridge' in c for c in calls)
    assert any(c.startswith('ENV UNSET file://') and c.endswith('/bridge.xml') for c in calls)


@pytest.mark.parametrize('mode',['legacy','upstream','upstream_default'])
def test_private_agent_accepts_both_sync_encodings(mode):
    from multi_agent_nav.mock import mock_config,make_machine
    from multi_agent_nav.models import Command,validate_command
    cfg=mock_config();cfg['scene']['share_c_coordinates']=False
    machine,_=make_machine(cfg,'ranger',True)
    from multi_agent_nav.models import Health,Pose2D
    machine.tick(0,Health(Pose2D(),True,True,True,[],False,[[.3,.2],[.3,-.2],[-.3,-.2],[-.3,.2]],True))
    sync=Command(task_id='new-task',recipient_id='ranger',action='SYNC' if mode=='legacy' else 'SYNC_LOCAL_C',
        phase='SYNCING',common_frame_id='',alignment_id='')
    if mode!='upstream_default':sync=replace(sync,kind='SYNC_LOCAL_C' if mode=='legacy' else '')
    context=replace(machine.context,task_id='new-task')
    assert validate_command(sync,context).accepted
    assert machine.accept(sync,0).accepted
    assert machine.context.task_id=='new-task' and machine.sync_received==0


def test_upstream_private_sync_is_rejected_in_shared_mode():
    from multi_agent_nav.mock import mock_config,make_machine
    from multi_agent_nav.models import Command,validate_command
    machine,_=make_machine(mock_config(),'ranger',True)
    context=replace(machine.context,task_id='t')
    command=Command(task_id='t',recipient_id='ranger',action='SYNC_LOCAL_C',common_frame_id='lab_ref',alignment_id='SYNTHETIC_v1')
    assert not validate_command(command,context).accepted
