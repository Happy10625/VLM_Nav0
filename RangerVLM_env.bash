#!/usr/bin/env bash
# Source-only environment for manual Ranger acceptance; never starts ROS nodes.
if [[ -z "${BASH_VERSION:-}" || "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '请使用 source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash' >&2
  return 1 2>/dev/null || exit 1
fi

_ranger_vlm_load_environment() {
  local mode="${1:-local}"
  local ranger_repo ranger_overlay ranger_marker ranger_session
  local ranger_status=0 ranger_had_nounset=false
  [[ "$-" == *u* ]] && ranger_had_nounset=true
  case "$mode" in
    local|--build|--legacy) ;;
    *) echo '用法：source RangerVLM_env.bash [--build|--legacy]' >&2; return 1 ;;
  esac
  ranger_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1
  ranger_overlay="$ranger_repo/build/ranger-local/install/setup.bash"
  if [[ ! -r "$ranger_repo/scripts/common.sh" ]]; then
    echo "缺少 Ranger 依赖环境：$ranger_repo/scripts/common.sh" >&2
    return 1
  fi
  if [[ "$mode" == local && ! -r "$ranger_overlay" ]]; then
    echo "缺少新 overlay：$ranger_overlay；构建终端先 source RangerVLM_env.bash --build。" >&2
    return 1
  fi

  # common.sh enables nounset; ROS setup files themselves require it disabled.
  set +u
  source "$ranger_repo/scripts/common.sh" || ranger_status=$?
  set +u
  if (( ranger_status == 0 )) && [[ "$mode" != --legacy && -r "$ranger_overlay" ]]; then
    source "$ranger_overlay" || ranger_status=$?
  fi
  if "$ranger_had_nounset"; then set -u; else set +u; fi
  (( ranger_status == 0 )) || return "$ranger_status"

  export REPO="$ranger_repo"
  export RANGER_LOCAL_OUTPUT="$REPO/build/ranger-local"
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
  export ROS_LOG_DIR="$RANGER_LOCAL_OUTPUT/field-logs"
  mkdir -p "$ROS_LOG_DIR" || return 1
  # Preserve this PC's RMW, network configuration and API environment.
  # A bridge terminal applies its reviewed dual-domain XML after this file.
  if [[ "$mode" != --legacy ]]; then
    ranger_marker="$RANGER_LOCAL_OUTPUT/current-acceptance-run.txt"
    export ACCEPTANCE_RUN='' RANGER_CONFIG='' RANGER_BRIDGE_DDS_XML=''
    if [[ -r "$ranger_marker" ]]; then
      IFS= read -r ranger_session < "$ranger_marker" || [[ -n "$ranger_session" ]] || return 1
      if [[ "$ranger_session" != /* || ! -d "$ranger_session/config" ]]; then
        echo "验收目录记录无效：$ranger_marker；请核对实际目录。" >&2
        return 1
      fi
      export ACCEPTANCE_RUN="$ranger_session"
      export RANGER_CONFIG="$ACCEPTANCE_RUN/config"
      export RANGER_BRIDGE_DDS_XML="$RANGER_CONFIG/cyclonedds_ranger_multi_agent_bridge.xml"
    fi
  fi
  cd "$REPO" || return 1
}

if _ranger_vlm_load_environment "$@"; then
  unset -f _ranger_vlm_load_environment
else
  unset -f _ranger_vlm_load_environment
  return 1
fi
