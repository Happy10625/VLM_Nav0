#!/usr/bin/env bash
# Source in each terminal; reuse each PC's existing hardware/API environment.
if [[ -z "${BASH_VERSION:-}" || "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '用法：source Cooperative_env.bash ranger|go2 [实际配置目录]' >&2
  return 1 2>/dev/null || exit 1
fi
_cooperative_environment() {
  local side="${1:-}" directory="${2:-}" repo base overlay status=0 had_nounset=false
  [[ "$-" == *u* ]] && had_nounset=true
  case "$side" in ranger|go2) ;; *) echo '须指定 ranger 或 go2。' >&2; return 2;; esac
  (( $# <= 2 )) || return 2
  repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1
  base="$repo/RangerVLM_env.bash"
  [[ "$side" == go2 ]] && base="$repo/CoVLM_env.bash"
  [[ -r "$base" ]] || { echo "缺少本机基础环境：$base" >&2; return 1; }
  overlay="${COOP_SETUP_FILE:-}"
  if [[ "$side" == go2 && -z "$overlay" ]]; then
    overlay="$repo/build/go2-independent-c-20261020/install/local_setup.bash"
  fi
  [[ -z "$overlay" || -r "$overlay" ]] || { echo "缺少协同 overlay：$overlay；请将 COOP_SETUP_FILE 指向本机已构建的新任务 overlay。" >&2; return 1; }
  set +u
  if [[ "$side" == ranger ]]; then source "$base" local || status=$?;
  else source "$base" || status=$?; fi
  set +u
  if (( status == 0 )) && [[ -n "$overlay" ]]; then source "$overlay" || status=$?; fi
  if "$had_nounset"; then set -u; else set +u; fi
  (( status == 0 )) || return "$status"
  if [[ -z "$directory" ]]; then
    if [[ "$side" == ranger ]]; then directory="${RANGER_CONFIG:-}";
    else directory="${GO2_CONFIG:-${GO2_EXPLORE_CONFIG:-}}"; fi
  fi
  [[ "$directory" == /* && -d "$directory" ]] || { echo '须指定本机实际配置目录的绝对路径。' >&2; return 1; }
  directory="$(cd -- "$directory" && pwd)" || return 1
  export COOP_SIDE="$side" COOP_REPO="$repo" COOP_CONFIG="$directory"
  export COOP_BRIDGE_YAML="$directory/bridge_${side}_pending.yaml"
  if [[ "$side" == ranger ]]; then
    export COOP_LOCAL_DDS_XML="$directory/cyclonedds_ranger_local.xml"
    export COOP_BRIDGE_DDS_XML="$directory/cyclonedds_ranger_multi_agent_bridge.xml"
    export RANGER_CONFIG="$directory" RANGER_BRIDGE_DDS_XML="$COOP_BRIDGE_DDS_XML"
  else
    export COOP_LOCAL_DDS_XML="$repo/config/cyclonedds_go2.xml"
    export COOP_BRIDGE_DDS_XML="$repo/config/cyclonedds_go2_multi_agent_bridge.xml"
    export GO2_CONFIG="$directory" GO2_NET_IFACE=enp4s0
  fi
  local asset
  for asset in "$COOP_LOCAL_DDS_XML" "$COOP_BRIDGE_DDS_XML" "$COOP_BRIDGE_YAML"; do
    [[ -r "$asset" ]] || { echo "缺少本机协同配置：$asset" >&2; return 1; }
  done
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$COOP_LOCAL_DDS_XML" COMM_ENV_FILE="$base"
  export PYTHONDONTWRITEBYTECODE=1
}
if _cooperative_environment "$@"; then unset -f _cooperative_environment;
else unset -f _cooperative_environment; return 1; fi
