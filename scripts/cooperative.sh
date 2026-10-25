#!/usr/bin/env bash
# One command per terminal; no automatic task start or background process management.
set -eo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h || $# == 0 ]]; then
  cat <<'HELP'
用法：bash scripts/cooperative.sh ranger|go2 ACTION
  preflight   离线核对安装/协议/地图/候选/DDS/API环境与live gates；未就绪退出1
  bridge      启动本机5条任务话题的定向桥，保持本终端运行
  agent       启动本机新任务agent；有真实gates时只发布阻塞状态
  coordinator 仅Go2：启动唯一coordinator，不自动开始任务
  check       静止PAUSED传输检查12秒（两机agent与bridge须已运行）
  wait        只读无限等待：双方各3条干净新鲜 WAITING_PEER/STANDBY；每5秒说明原因
  state/peer  显示本机/对端任务状态，Ctrl+C只结束显示
  watch       仅Go2：显示coordinator状态，Ctrl+C只结束显示
  start       仅Go2：live预检通过后显式开始一次真实任务
  pause       仅Go2：请求软件暂停；不受live gates阻塞
每步独立执行；硬件/定位沿用原入口。配置用COOP_CONFIG覆盖，API继续用环境变量。
HELP
  exit 0
fi
side="$1" action="${2:-preflight}"
case "$side" in ranger|go2) ;; *) echo '机器人须为 ranger 或 go2。' >&2; exit 2;; esac
case "$action" in preflight|bridge|agent|coordinator|check|wait|state|peer|watch|start|pause) ;; *) echo '未知 ACTION；使用 --help。' >&2; exit 2;; esac
(( $# <= 2 )) || { echo '不接受额外参数；配置通过环境文件指定。' >&2; exit 2; }
case "$action" in coordinator|start|pause|watch)
  [[ "$side" == go2 ]] || { echo '该命令仅在Go2控制电脑执行。' >&2; exit 2; };;
esac
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo/Cooperative_env.bash" "$side" "${COOP_CONFIG:-}"

preflight() {
  python3 - "${1:-transport}" <<'PY'
from dataclasses import fields
from pathlib import Path
import hashlib, importlib, json, os, sys
sys.path.insert(0,str(Path(os.environ["COOP_REPO"])/"scripts"))
from check_multi_agent_comm import validate_bridge, validate_dds
import yaml
from multi_agent_nav.config import load_config, live_gates
from multi_agent_nav.models import AgentContext, AgentState, Command, Observation, Pose2D, Snapshot, validate_command
from multi_agent_nav.planner import RulePlanner
from multi_agent_nav.generated_candidates import GeneratedCandidates
from multi_agent_task_msgs import msg
side=os.environ['COOP_SIDE'];repo=Path(os.environ['COOP_REPO']);directory=Path(os.environ['COOP_CONFIG'])
interfaces=validate_dds(os.environ['COOP_LOCAL_DDS_XML'],os.environ['COOP_BRIDGE_DDS_XML'],side)
bridge=validate_bridge(yaml.safe_load(Path(os.environ['COOP_BRIDGE_YAML']).read_text()),side)
cfg=load_config(directory)
if cfg['scene'].get('share_c_coordinates') is not False:raise ValueError('本流程要求share_c_coordinates=false')
installed={}
for name in ('agent','agent_node','coordinator','models','planner','runner','go2_exploration','generated_candidates','config','coordinator_node','mock','semantic_search'):
    module=importlib.import_module('multi_agent_nav.'+name);path=Path(module.__file__)
    source=repo/'cooperative_ws/src/multi_agent_nav/multi_agent_nav'/f'{name}.py'
    installed[name]={'file':str(path),'matches_source':path.read_bytes()==source.read_bytes()}
if not all(row['matches_source'] for row in installed.values()):raise ValueError('安装与源码不一致；先更新本机overlay，不能混用旧任务实现')
states={r:AgentState(robot_id=r,ready=True) for r in ('go2','ranger')}
observations={r:Observation(robot_id=r,target_id='C',status='CACHED',observation_id=r) for r in states}
snapshot=Snapshot('DEPOSIT',states,observations,{},None,{},dict(share_c_coordinates=False),0.,local_c={r:r for r in states})
actions=RulePlanner().decide(snapshot)
if {a.robot_id for a in actions if a.kind=='APPROACH_DEPOSIT'}!=set(states):raise ValueError('安装未采用独立C并行接近规则')
for state in states.values():state.deposited=True
if RulePlanner().decide(snapshot):raise ValueError('安装仍在C完成后安排EXIT')
kwargs=dict(robot_id=side,task_id='offline',local_map_id='offline',map_version='1',common_frame_id='',alignment_id='',simulation=False)
if 'share_c_coordinates' in {f.name for f in fields(AgentContext)}:kwargs['share_c_coordinates']=False
context=AgentContext(**kwargs);sync={}
for label,action,kind in [('action','SYNC_LOCAL_C','WAIT'),('kind','SYNC','SYNC_LOCAL_C')]:
    sync[label]=validate_command(Command(task_id='offline',recipient_id=side,action=action,kind=kind,common_frame_id='',alignment_id='',simulation=False),context).accepted
if side=='ranger' and not sync['action']:raise ValueError('Ranger尚未支持远端action=SYNC_LOCAL_C；先同步接收兼容补丁')
schemas={name:hashlib.sha256(json.dumps(getattr(msg,name).get_fields_and_field_types(),sort_keys=True).encode()).hexdigest() for name in ('AgentState','Observation','Command')}
from vlm_nav.keepout import configured_mask
from ament_index_python.packages import get_package_share_directory
import vlm_nav.keepout as keepout_module
keepout_source=repo/'vlm_nav/keepout.py'
if Path(keepout_module.__file__).read_bytes()!=keepout_source.read_bytes():raise ValueError('Keepout安装与源码不一致')
if side=='ranger':
    for name in ('navigation.launch.py','system.launch.py','ranger_local.launch.py'):
        installed_launch=Path(get_package_share_directory('vlm_nav'))/'launch'/name
        if installed_launch.read_bytes()!=(repo/'launch'/name).read_bytes():raise ValueError('Nav2启动文件安装与源码不一致：'+name)
robot=cfg['robot_'+side];geometry=GeneratedCandidates(cfg,side)
mask=configured_mask(robot)
key=os.getenv(robot.get('api_key_env','DASHSCOPE_API_KEY')) or os.getenv('OPENAI_API_KEY')
endpoint=robot.get('endpoint') or os.getenv('DASHSCOPE_BASE_URL') or os.getenv('DASHSCOPE_WORKSPACE_ID') or os.getenv('OPENAI_BASE_URL')
model=robot.get('model') or os.getenv('DASHSCOPE_MODEL') or os.getenv('OPENAI_MODEL')
api_ready=bool(key and endpoint and model)
gates=live_gates(cfg,local_robot=side)
print(json.dumps(dict(side=side,config_dir=str(directory),bridge=bridge,interfaces=interfaces,installed=installed,
    schema_sha256=schemas,sync_encodings=sync,targets=cfg['scene']['targets'],scenario_id=cfg['scene']['scenario_id'],
    map_identity=cfg['alignment']['robots'],candidate_count=len(geometry.points),api_environment_complete=api_ready,
    keepout_mask_yaml=str(mask.path) if mask is not None else None,keepout_mask_digest=mask.digest if mask is not None else None,
    live_gates=gates,motion_sent=False),ensure_ascii=False,indent=2))
if sys.argv[1]=='live' and (gates or not api_ready):sys.exit(1)
PY
}

unique_node() {
  local name="$1" nodes
  nodes="$(timeout 8 ros2 node list --no-daemon --spin-time 2)" || return 1
  if awk -v name="/$name" '$0==name || index($0,name "_")==1 {found=1} END {exit !found}' <<< "$nodes"; then
    echo "已有节点 $name，拒绝重复启动；核对原终端。" >&2; return 1
  fi
  if [[ "$action" == agent ]] && grep -Eq '^/(vlm_nav|multi_agent_.*check_motion|multi_agent_.*rgbd)$' <<< "$nodes"; then
    echo '已有独立探索/感知任务拥有者，请先从原入口结束并确认停稳。' >&2; return 1
  fi
}

case "$action" in
  preflight) preflight live;;
  bridge)
    preflight transport
    bridge_name="$(python3 -c 'import sys,yaml;print(yaml.safe_load(open(sys.argv[1]))["name"])' "$COOP_BRIDGE_YAML")"
    unique_node "$bridge_name"
    exec env -u ROS_DOMAIN_ID CYCLONEDDS_URI="file://$COOP_BRIDGE_DDS_XML" ros2 run domain_bridge domain_bridge "$COOP_BRIDGE_YAML";;
  agent|coordinator)
    preflight transport
    node_name="multi_agent_${side}_agent";executable=multi_agent_agent_node
    params=(-p "robot_id:=$side")
    if [[ "$action" == coordinator ]]; then node_name=multi_agent_coordinator;executable=multi_agent_coordinator_node;params=();fi
    unique_node "$node_name"
    exec ros2 run multi_agent_nav "$executable" --ros-args -r "__node:=$node_name" \
      -p simulation:=false -p topic_prefix:=/multi_agent -p "config_dir:=$COOP_CONFIG" "${params[@]}";;
  check) exec python3 "$repo/scripts/check_multi_agent_comm.py" --side "$side" check --config-dir "$COOP_CONFIG";;
  wait) exec python3 "$repo/scripts/wait_cooperative_ready.py" --side "$side" --config-dir "$COOP_CONFIG";;
  state) exec ros2 topic echo "/multi_agent/$side/state" multi_agent_task_msgs/msg/AgentState;;
  peer)
    peer=go2;[[ "$side" == go2 ]] && peer=ranger
    exec ros2 topic echo "/multi_agent/$peer/state" multi_agent_task_msgs/msg/AgentState;;
  watch) exec ros2 topic echo /multi_agent/coordinator/status std_msgs/msg/String;;
  start)
    preflight live
    scenario="$(python3 -c 'import sys,yaml;print(yaml.safe_load(open(sys.argv[1]))["scenario_id"])' "$COOP_CONFIG/scene.yaml")"
    exec ros2 run multi_agent_nav multi_agent_start_task start --scenario "$scenario";;
  pause) exec ros2 run multi_agent_nav multi_agent_start_task pause;;
esac
