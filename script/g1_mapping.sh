#!/usr/bin/env bash
set -Eeuo pipefail

# G1 + MID360 + FAST-LIO 建图管理器
#
# 用法：
#   ./g1_mapping.sh start       启动雷达、FAST-LIO 和建图 RViz
#   ./g1_mapping.sh save        调用 /map_save 保存当前 PCD
#   ./g1_mapping.sh finish      保存地图后停止系统
#   ./g1_mapping.sh stop        停止系统（不主动调用 /map_save）
#   ./g1_mapping.sh restart
#   ./g1_mapping.sh status
#   ./g1_mapping.sh logs
#   ./g1_mapping.sh map-path    显示本次建图输出路径

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${G1_WS:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
PCL_SETUP="${PCL_SETUP:-${HOME}/ws_pcl_ros/install/setup.bash}"
WS_SETUP="${WS}/install/setup.bash"

SOURCE_CFG="${WS}/src/FAST_LIO_ROS2/config/mid360.yaml"
RVIZ_CFG="${WS}/src/FAST_LIO_ROS2/rviz/fastlio.rviz"

RUNTIME_DIR="${WS}/runtime/mapping"
LOG_DIR="${RUNTIME_DIR}/logs"
PID_FILE="${RUNTIME_DIR}/mapping.pids"
SESSION_FILE="${RUNTIME_DIR}/session.env"
RUNTIME_CFG="${RUNTIME_DIR}/mid360_mapping.yaml"

mkdir -p "${LOG_DIR}"

say()  { printf '\033[1;36m[G1-MAP]\033[0m %s\n' "$*"; }
ok()   { printf '\033[1;32m[OK]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[WARN]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[ERROR]\033[0m %s\n' "$*" >&2; exit 1; }

source_environment() {
  [[ -f "${ROS_SETUP}" ]] || die "找不到 ${ROS_SETUP}"

  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  if [[ -f "${PCL_SETUP}" ]]; then
    # shellcheck disable=SC1090
    source "${PCL_SETUP}"
  fi
  [[ -f "${WS_SETUP}" ]] || {
    set -u
    die "找不到 ${WS_SETUP}，请先编译工作空间"
  }
  # shellcheck disable=SC1090
  source "${WS_SETUP}"
  set -u

  if [[ -d "${WS}/install/livox_sdk2/lib" ]]; then
    export LD_LIBRARY_PATH="${WS}/install/livox_sdk2/lib:${LD_LIBRARY_PATH:-}"
  fi
}

check_package() {
  ros2 pkg prefix "$1" >/dev/null 2>&1 || die "ROS2 包不存在：$1"
}

check_prerequisites() {
  [[ -d "${WS}" ]] || die "工作空间不存在：${WS}"
  [[ -f "${SOURCE_CFG}" ]] || die "缺少 FAST-LIO 配置：${SOURCE_CFG}"
  [[ -f "${RVIZ_CFG}" ]] || die "缺少 RViz 配置：${RVIZ_CFG}"

  source_environment
  check_package livox_ros_driver2
  check_package fast_lio

  if timeout 4 ros2 node list 2>/dev/null | grep -Eq '/(controller_server|planner_server|g1_relocalization|map_server)$'; then
    die "检测到定位或导航系统仍在运行，请先执行 g1_navigation.sh stop"
  fi
}

pid_alive() { kill -0 "$1" 2>/dev/null; }

start_process() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"

  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  local pid=$!
  printf '%s %s\n' "${name}" "${pid}" >>"${PID_FILE}"
  say "已启动 ${name}，PID=${pid}，日志=${log_file}"
}

stop_from_pid_file() {
  [[ -f "${PID_FILE}" ]] || return 0

  tac "${PID_FILE}" | while read -r name pid; do
    [[ -n "${pid:-}" ]] || continue
    if pid_alive "${pid}"; then
      say "停止 ${name}，PID=${pid}"
      kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    fi
  done

  sleep 3

  tac "${PID_FILE}" | while read -r name pid; do
    [[ -n "${pid:-}" ]] || continue
    if pid_alive "${pid}"; then
      warn "${name} 未正常退出，发送 TERM"
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  rm -f "${PID_FILE}"
}

cleanup_stale_mapping() {
  local patterns=(
    "livox_ros_driver2_node"
    "fastlio_mapping"
  )

  for pattern in "${patterns[@]}"; do
    local pids
    pids="$(pgrep -u "${USER}" -f "${pattern}" || true)"
    if [[ -n "${pids}" ]]; then
      warn "清理旧进程 ${pattern}：${pids//$'\n'/ }"
      # shellcheck disable=SC2086
      kill -INT ${pids} 2>/dev/null || true
    fi
  done

  sleep 2
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  local elapsed=0

  while (( elapsed < timeout_sec )); do
    if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      ok "话题已出现：${topic}"
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done

  return 1
}

wait_for_service() {
  local service="$1"
  local timeout_sec="${2:-30}"
  local elapsed=0

  while (( elapsed < timeout_sec )); do
    if timeout 4 ros2 service list 2>/dev/null | grep -Fxq "${service}"; then
      ok "服务已出现：${service}"
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done

  return 1
}

generate_runtime_config() {
  local output_path
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  output_path="${G1_MAPPING_OUTPUT:-${WS}/maps/map_${stamp}.pcd}"

  mkdir -p "$(dirname "${output_path}")" "${RUNTIME_DIR}"

  python3 - "${SOURCE_CFG}" "${RUNTIME_CFG}" "${output_path}" <<'PY'
from pathlib import Path
import sys

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
output = Path(sys.argv[3]).resolve()

lines = src.read_text(encoding="utf-8").splitlines()
patched = []
found_path = False
found_save = False

for line in lines:
    stripped = line.strip()
    indent = line[:len(line) - len(line.lstrip())]

    if stripped.startswith("map_file_path:"):
        patched.append(f'{indent}map_file_path: "{output}"')
        found_path = True
    elif stripped.startswith("pcd_save_en:"):
        patched.append(f"{indent}pcd_save_en: true")
        found_save = True
    else:
        patched.append(line)

if not found_path:
    raise SystemExit("配置中找不到 map_file_path")
if not found_save:
    raise SystemExit("配置中找不到 pcd_save_en")

dst.parent.mkdir(parents=True, exist_ok=True)
dst.write_text("\n".join(patched) + "\n", encoding="utf-8")
PY

  cat >"${SESSION_FILE}" <<EOF_SESSION
MAP_OUTPUT=${output_path}
RUNTIME_CONFIG=${RUNTIME_CFG}
START_TIME=${stamp}
EOF_SESSION

  ok "本次地图输出：${output_path}"
  ok "运行配置：${RUNTIME_CFG}"
}

load_session() {
  [[ -f "${SESSION_FILE}" ]] || die "没有建图会话记录，请先执行 start"
  # shellcheck disable=SC1090
  source "${SESSION_FILE}"
}

start_all() {
  check_prerequisites
  stop_from_pid_file
  cleanup_stale_mapping

  : >"${PID_FILE}"
  rm -f "${LOG_DIR}"/*.log
  generate_runtime_config

  start_process \
    livox \
    ros2 launch livox_ros_driver2 msg_MID360_launch.py

  wait_for_topic /livox/lidar 35 || die "未收到 /livox/lidar，请查看 livox.log"
  wait_for_topic /livox/imu 20 || die "未收到 /livox/imu，请查看 livox.log"

  start_process \
    fast_lio \
    ros2 launch fast_lio mapping.launch.py \
      config_path:="${RUNTIME_DIR}" \
      config_file:="$(basename "${RUNTIME_CFG}")" \
      rviz:=false

  wait_for_topic /Odometry 40 || die "FAST-LIO 未发布 /Odometry"
  wait_for_topic /cloud_registered 30 || warn "未检测到 /cloud_registered"
  wait_for_service /map_save 30 || die "未检测到 /map_save"

  start_process rviz rviz2 -d "${RVIZ_CFG}"

  echo
  ok "建图系统已启动"
  say "使用遥控器缓慢走遍环境，尽量闭环并减少快速转动"
  say "保存地图：${SCRIPT_DIR}/g1_mapping.sh save"
  say "保存并停止：${SCRIPT_DIR}/g1_mapping.sh finish"
  show_map_path
}

save_map() {
  source_environment
  load_session

  wait_for_service /map_save 8 || die "/map_save 不存在，建图节点可能未运行"

  say "正在保存地图到 ${MAP_OUTPUT}"
  timeout 120 ros2 service call /map_save std_srvs/srv/Trigger "{}" \
    || die "调用 /map_save 失败"

  sleep 2
  [[ -s "${MAP_OUTPUT}" ]] || die "服务返回后仍未生成有效文件：${MAP_OUTPUT}"

  ok "地图保存成功"
  ls -lh "${MAP_OUTPUT}"
}

show_map_path() {
  if [[ -f "${SESSION_FILE}" ]]; then
    # shellcheck disable=SC1090
    source "${SESSION_FILE}"
    printf '当前地图输出：%s\n' "${MAP_OUTPUT}"
  else
    warn "尚未创建本次建图会话"
  fi
}

stop_all() {
  source_environment || true
  stop_from_pid_file
  cleanup_stale_mapping
  ok "建图系统已停止"
}

finish_all() {
  save_map
  stop_all
}

show_status() {
  source_environment
  echo "工作空间：${WS}"
  show_map_path
  echo

  if [[ -f "${PID_FILE}" ]]; then
    printf '%-14s %-10s %s\n' "进程" "PID" "状态"
    while read -r name pid; do
      if pid_alive "${pid}"; then
        printf '%-14s %-10s %s\n' "${name}" "${pid}" "运行中"
      else
        printf '%-14s %-10s %s\n' "${name}" "${pid}" "已退出"
      fi
    done <"${PID_FILE}"
  else
    warn "没有 PID 文件"
  fi

  echo
  for topic in /livox/lidar /livox/imu /Odometry /cloud_registered /cloud_registered_body; do
    if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      printf '[OK]   %s\n' "${topic}"
    else
      printf '[MISS] %s\n' "${topic}"
    fi
  done

  if timeout 4 ros2 service list 2>/dev/null | grep -Fxq /map_save; then
    printf '[OK]   /map_save\n'
  else
    printf '[MISS] /map_save\n'
  fi
}

show_logs() {
  for file in "${LOG_DIR}"/*.log; do
    echo
    printf '========== %s ==========\n' "$(basename "${file}")"
    [[ -f "${file}" ]] && tail -n 60 "${file}" || true
  done
}

usage() {
  cat <<EOF_USAGE
用法：
  $0 start
  $0 save
  $0 finish
  $0 stop
  $0 restart
  $0 status
  $0 logs
  $0 map-path

可选输出路径：
  G1_MAPPING_OUTPUT=/绝对路径/new_map.pcd $0 start
EOF_USAGE
}

main() {
  case "${1:-start}" in
    start) start_all ;;
    save) save_map ;;
    finish) finish_all ;;
    stop) stop_all ;;
    restart) stop_all; sleep 2; start_all ;;
    status) show_status ;;
    logs) show_logs ;;
    map-path) show_map_path ;;
    -h|--help|help) usage ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
