#!/usr/bin/env bash
set -Eeuo pipefail

# G1 + MID360 + FAST-LIO + NDT + Nav2 导航管理器
#
# 当前脚本只支持：
#   1. Nav2 规划模式（不接机器人）
#   2. Nav2 -> 安全桥 Dry Run（机器人不会运动）
#
# 不提供真实 Unitree 后端启动入口，避免误启动实机运动。
#
# 用法：
#   ./g1_navigation.sh start             手动初始化，启动定位 + Nav2
#   ./g1_navigation.sh start-dry         手动初始化，并启动 Dry Run 安全桥
#   ./g1_navigation.sh start-auto        使用 auto_start_pose.env 自动初始化
#   ./g1_navigation.sh start-auto-dry    自动初始化，并启动 Dry Run 安全桥
#   ./g1_navigation.sh continue          定位已成功后继续启动 Nav2
#   ./g1_navigation.sh continue-dry      定位已成功后继续启动 Nav2 + Dry Run
#   ./g1_navigation.sh enable-dry
#   ./g1_navigation.sh disable-dry
#   ./g1_navigation.sh stop|restart|status|logs

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${G1_WS:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
PCL_SETUP="${PCL_SETUP:-${HOME}/ws_pcl_ros/install/setup.bash}"
UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-}"
WS_SETUP="${WS}/install/setup.bash"

MAP_PCD="${G1_MAP_PCD:-${WS}/maps/map.pcd}"
MAP_PGM="${G1_MAP_PGM:-${WS}/maps/map_multilayer.pgm}"
MAP_YAML="${G1_MAP_YAML:-${WS}/maps/map_multilayer.yaml}"

FASTLIO_SOURCE_CFG="${WS}/src/FAST_LIO_ROS2/config/mid360.yaml"
RELOC_CFG="${WS}/src/g1_relocalization/config/relocalization.yaml"
NAV2_SOURCE_PARAMS="${WS}/src/g1_nav2_bringup/config/nav2_params.yaml"
NAV_RVIZ="${WS}/src/g1_nav2_bringup/rviz/navigation.rviz"
DRY_CFG="${WS}/src/g1_loco_bridge/config/g1_loco_bridge_nav2_dry_run.yaml"
AUTO_POSE_FILE="${SCRIPT_DIR}/auto_start_pose.env"

RUNTIME_DIR="${WS}/runtime/navigation"
LOG_DIR="${RUNTIME_DIR}/logs"
PID_FILE="${RUNTIME_DIR}/navigation.pids"
RUNTIME_MAP_YAML="${RUNTIME_DIR}/map_runtime.yaml"
RUNTIME_FASTLIO_CFG="${RUNTIME_DIR}/mid360_navigation.yaml"
RUNTIME_NAV2_PARAMS="${RUNTIME_DIR}/nav2_params_safe.yaml"

mkdir -p "${LOG_DIR}"

say()  { printf '\033[1;36m[G1-NAV]\033[0m %s\n' "$*"; }
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
  if [[ -f "${UNITREE_ROS2_SETUP}" ]]; then
    # 仅作为 unitree_api underlay；start/start-dry 都不会调用真实运动后端。
    # shellcheck disable=SC1090
    source "${UNITREE_ROS2_SETUP}"
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

check_file() { [[ -f "$1" ]] || die "缺少文件：$1"; }
check_package() { ros2 pkg prefix "$1" >/dev/null 2>&1 || die "ROS2 包不存在：$1"; }
pid_alive() { kill -0 "$1" 2>/dev/null; }

check_mapping_not_running() {
  local mapping_pid_file="${WS}/runtime/mapping/mapping.pids"
  if [[ -f "${mapping_pid_file}" ]]; then
    while read -r _name pid; do
      if [[ -n "${pid:-}" ]] && pid_alive "${pid}"; then
        die "建图系统仍在运行，请先执行 ${SCRIPT_DIR}/g1_mapping.sh stop"
      fi
    done <"${mapping_pid_file}"
  fi
}

check_prerequisites() {
  [[ -d "${WS}" ]] || die "工作空间不存在：${WS}"
  check_mapping_not_running

  check_file "${MAP_PCD}"
  check_file "${MAP_PGM}"
  check_file "${MAP_YAML}"
  check_file "${FASTLIO_SOURCE_CFG}"
  check_file "${RELOC_CFG}"
  check_file "${NAV2_SOURCE_PARAMS}"
  check_file "${NAV_RVIZ}"

  source_environment

  check_package livox_ros_driver2
  check_package fast_lio
  check_package g1_relocalization
  check_package nav2_map_server
  check_package g1_nav2_bringup

  ok "导航启动前检查通过"
}

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

cleanup_stale_navigation() {
  local patterns=(
    "livox_ros_driver2_node"
    "fastlio_mapping"
    "g1_relocalization_node"
    "nav2_map_server.*map_server"
    "controller_server"
    "planner_server"
    "recoveries_server"
    "bt_navigator"
    "lifecycle_manager_navigation"
    "g1_cmd_vel_dry_run"
    "body_to_nav_base"
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

  local rviz_pids
  rviz_pids="$(pgrep -u "${USER}" -x rviz2 || true)"
  if [[ -n "${rviz_pids}" ]]; then
    warn "清理旧 RViz：${rviz_pids//$'\n'/ }"
    # shellcheck disable=SC2086
    kill -INT ${rviz_pids} 2>/dev/null || true
  fi

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

wait_for_node() {
  local node="$1"
  local timeout_sec="${2:-30}"
  local elapsed=0

  while (( elapsed < timeout_sec )); do
    if timeout 4 ros2 node list 2>/dev/null | grep -Fxq "${node}"; then
      ok "节点已出现：${node}"
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done
  return 1
}

generate_runtime_files() {
  mkdir -p "${RUNTIME_DIR}"

  python3 - \
    "${MAP_YAML}" \
    "${MAP_PGM}" \
    "${RUNTIME_MAP_YAML}" \
    "${FASTLIO_SOURCE_CFG}" \
    "${RUNTIME_FASTLIO_CFG}" \
    "${NAV2_SOURCE_PARAMS}" \
    "${RUNTIME_NAV2_PARAMS}" <<'PY'
from pathlib import Path
import sys

map_yaml = Path(sys.argv[1])
map_pgm = Path(sys.argv[2]).resolve()
runtime_map = Path(sys.argv[3])
fastlio_src = Path(sys.argv[4])
fastlio_dst = Path(sys.argv[5])
nav_src = Path(sys.argv[6])
nav_dst = Path(sys.argv[7])

# 地图 YAML 使用绝对 PGM 路径。
map_lines = map_yaml.read_text(encoding="utf-8").splitlines()
map_out = []
replaced = False
for line in map_lines:
    if line.strip().startswith("image:"):
        map_out.append(f'image: "{map_pgm}"')
        replaced = True
    else:
        map_out.append(line)
if not replaced:
    map_out.insert(0, f'image: "{map_pgm}"')
runtime_map.write_text("\n".join(map_out) + "\n", encoding="utf-8")

# 导航阶段关闭 FAST-LIO PCD 累积，避免长时间导航占用大量内存。
fast_lines = fastlio_src.read_text(encoding="utf-8").splitlines()
fast_out = []
found_save = False
for line in fast_lines:
    stripped = line.strip()
    indent = line[:len(line) - len(line.lstrip())]
    if stripped.startswith("pcd_save_en:"):
        fast_out.append(f"{indent}pcd_save_en: false")
        found_save = True
    else:
        fast_out.append(line)
if not found_save:
    raise SystemExit("FAST-LIO 配置中找不到 pcd_save_en")
fastlio_dst.write_text("\n".join(fast_out) + "\n", encoding="utf-8")

# 生成适用于 G1 第一版导航的运行时 Nav2 参数。
# 按“参数名”修改，而不是依赖旧值，因此脚本可以重复执行，
# 也不会因为源码参数已经被手动修改而报错。
nav_lines = nav_src.read_text(encoding="utf-8").splitlines()
nav_values = {
    "min_vel_x": "0.0",
    "min_vel_y": "0.0",
    "max_vel_x": "0.50",
    "max_vel_y": "0.0",
    "max_vel_theta": "0.40",
    "min_speed_xy": "0.0",
    "max_speed_xy": "0.50",
    "min_speed_theta": "0.0",
    "acc_lim_x": "0.3",
    "acc_lim_y": "0.0",
    "acc_lim_theta": "0.5",
    "decel_lim_x": "-0.3",
    "decel_lim_y": "0.0",
    "decel_lim_theta": "-0.5",
    "vx_samples": "5",
    "vy_samples": "1",
    "vtheta_samples": "9",
}

found = {key: 0 for key in nav_values}
nav_out = []
for line in nav_lines:
    stripped = line.strip()
    replaced = False
    for key, value in nav_values.items():
        if stripped.startswith(key + ":"):
            indent = line[:len(line) - len(line.lstrip())]
            nav_out.append(f"{indent}{key}: {value}")
            found[key] += 1
            replaced = True
            break
    if not replaced:
        nav_out.append(line)

missing = [key for key, count in found.items() if count == 0]
duplicated = [key for key, count in found.items() if count > 1]
if missing:
    raise SystemExit(
        "Nav2 配置缺少必要参数：" + ", ".join(missing)
    )
if duplicated:
    raise SystemExit(
        "Nav2 配置存在重复参数，拒绝生成运行时文件：" + ", ".join(duplicated)
    )

nav_dst.write_text("\n".join(nav_out) + "\n", encoding="utf-8")
PY

  ok "运行时地图 YAML：${RUNTIME_MAP_YAML}"
  ok "导航 FAST-LIO 配置：${RUNTIME_FASTLIO_CFG}"
  ok "安全 Nav2 参数：${RUNTIME_NAV2_PARAMS}"
}

activate_map_server() {
  wait_for_node /map_server 30 || die "map_server 未启动"

  local state
  state="$(timeout 5 ros2 lifecycle get /map_server 2>/dev/null || true)"
  if grep -qi unconfigured <<<"${state}"; then
    timeout 20 ros2 lifecycle set /map_server configure >/dev/null \
      || die "map_server configure 失败"
  fi

  state="$(timeout 5 ros2 lifecycle get /map_server 2>/dev/null || true)"
  if grep -qi inactive <<<"${state}"; then
    timeout 20 ros2 lifecycle set /map_server activate >/dev/null \
      || die "map_server activate 失败"
  fi

  state="$(timeout 5 ros2 lifecycle get /map_server 2>/dev/null || true)"
  grep -qi active <<<"${state}" || die "map_server 未激活：${state}"
  ok "map_server 已激活"
}

publish_auto_pose() {
  [[ -f "${AUTO_POSE_FILE}" ]] || die "找不到自动起点：${AUTO_POSE_FILE}"
  # shellcheck disable=SC1090
  source "${AUTO_POSE_FILE}"

  : "${AUTO_X:?缺少 AUTO_X}"
  : "${AUTO_Y:?缺少 AUTO_Y}"
  : "${AUTO_Z:=0.0}"
  : "${AUTO_YAW:?缺少 AUTO_YAW}"

  local qz qw
  read -r qz qw < <(
    python3 - "${AUTO_YAW}" <<'PY'
import math
import sys
yaw = float(sys.argv[1])
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
  )

  say "等待 FAST-LIO 稳定 8 秒后发布固定初始位姿"
  sleep 8

  ros2 topic pub -1 \
    /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: ${AUTO_X}, y: ${AUTO_Y}, z: ${AUTO_Z}}, orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
}

wait_for_ndt_success() {
  local timeout_sec="${1:-300}"
  local elapsed=0
  local log_file="${LOG_DIR}/relocalization.log"

  say "等待 NDT 定位成功，最长 ${timeout_sec} 秒"

  while (( elapsed < timeout_sec )); do
    if [[ -f "${log_file}" ]] && grep -q "NDT success" "${log_file}"; then
      ok "检测到 NDT success"
      tail -n 5 "${log_file}" | grep -E "Received initial pose|NDT success" || true
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done

  return 1
}

start_base_stack() {
  local init_mode="$1"

  check_prerequisites
  stop_from_pid_file
  cleanup_stale_navigation

  : >"${PID_FILE}"
  rm -f "${LOG_DIR}"/*.log
  generate_runtime_files

  start_process livox ros2 launch livox_ros_driver2 msg_MID360_launch.py
  wait_for_topic /livox/lidar 35 || die "未收到 /livox/lidar"
  wait_for_topic /livox/imu 20 || die "未收到 /livox/imu"

  start_process \
    fast_lio \
    ros2 launch fast_lio mapping.launch.py \
      config_path:="${RUNTIME_DIR}" \
      config_file:="$(basename "${RUNTIME_FASTLIO_CFG}")" \
      rviz:=false

  wait_for_topic /Odometry 40 || die "FAST-LIO 未发布 /Odometry"
  wait_for_topic /cloud_registered_body 30 || die "FAST-LIO 未发布 /cloud_registered_body"

  start_process \
    relocalization \
    ros2 run g1_relocalization g1_relocalization_node \
      --ros-args \
      --params-file "${RELOC_CFG}" \
      -p map_path:="${MAP_PCD}"

  wait_for_topic /global_pcd_map 30 || die "未发布 /global_pcd_map"

  start_process \
    map_server \
    ros2 run nav2_map_server map_server \
      --ros-args \
      -p yaml_filename:="${RUNTIME_MAP_YAML}"

  # 先启动 RViz，再激活 map_server。
  # 防止 /map 在 RViz 完成订阅前已经发布，导致 Map 显示 No map received。
  wait_for_node /map_server 30 || die "map_server 未启动"

  start_process rviz rviz2 -d "${NAV_RVIZ}"
# 给 RViz 的 Map 显示留出建立 /map 订阅的时间
  sleep 2

  activate_map_server
  wait_for_topic /map 20 || die "未出现 /map"

  if [[ "${init_mode}" == "auto" ]]; then
    publish_auto_pose
  else
    echo
    say "请在 RViz 顶部点击 2D Pose Estimate"
    say "在二维地图上选择机器人实际位置并拖出正前方"
  fi
}

wait_nav2_active() {
  local nodes=(controller_server planner_server recoveries_server bt_navigator)
  local node state

  for node in "${nodes[@]}"; do
    local elapsed=0
    while (( elapsed < 60 )); do
      state="$(timeout 5 ros2 lifecycle get "/${node}" 2>/dev/null || true)"
      if grep -qi active <<<"${state}"; then
        ok "${node}: active"
        break
      fi
      sleep 1
      ((elapsed += 1))
    done
    [[ "${elapsed}" -lt 60 ]] || die "${node} 未进入 active，查看 nav2.log"
  done
}

check_tf_chain() {
  local tf_log="${RUNTIME_DIR}/tf_map_nav_base.txt"
  timeout 10 ros2 run tf2_ros tf2_echo map nav_base >"${tf_log}" 2>&1 || true

  if grep -q "Translation:" "${tf_log}"; then
    ok "TF 链 map -> nav_base 可用"
  else
    cat "${tf_log}" >&2
    die "TF 链 map -> nav_base 不可用"
  fi
}

start_nav2_stack() {
  local with_dry="$1"

  source_environment

  if ! grep -q "NDT success" "${LOG_DIR}/relocalization.log" 2>/dev/null; then
    die "尚未检测到 NDT success，请先完成 2D Pose Estimate"
  fi

  if timeout 4 ros2 node list 2>/dev/null | grep -Fxq /controller_server; then
    warn "Nav2 已经运行，跳过重复启动"
  else
    start_process \
      nav2 \
      ros2 launch g1_nav2_bringup planning_only.launch.py \
        params_file:="${RUNTIME_NAV2_PARAMS}"

    wait_for_node /controller_server 30 || die "controller_server 未启动"
    wait_nav2_active
  fi

  wait_for_topic /plan 20 || die "未出现 /plan"
  wait_for_topic /cmd_vel_nav 20 || die "未出现 /cmd_vel_nav"
  check_tf_chain

  if [[ "${with_dry}" == "yes" ]]; then
    check_file "${DRY_CFG}"
    check_package g1_loco_bridge

    if timeout 4 ros2 node list 2>/dev/null | grep -Fxq /g1_cmd_vel_dry_run; then
      warn "Dry Run 安全桥已经运行"
    else
      start_process \
        dry_bridge \
        ros2 run g1_loco_bridge g1_cmd_vel_dry_run \
          --ros-args \
          --params-file "${DRY_CFG}"

      wait_for_node /g1_cmd_vel_dry_run 20 || die "Dry Run 安全桥未启动"

      local dry_value
      dry_value="$(timeout 8 ros2 param get /g1_cmd_vel_dry_run dry_run 2>/dev/null || true)"
      grep -q "True" <<<"${dry_value}" || die "安全桥不是 dry_run=True，已拒绝继续"
      ok "Dry Run 安全桥已启动，安全门保持关闭"
    fi
  fi

  echo
  ok "导航系统已准备完成"
  say "发送目标前先确认 RViz Fixed Frame=map"
  if [[ "${with_dry}" == "yes" ]]; then
    say "启用模拟安全门：${SCRIPT_DIR}/g1_navigation.sh enable-dry"
    say "该模式 real_backend=false，机器人不会运动"
  else
    say "当前只启动 Nav2 规划，没有连接机器人控制桥"
  fi
}

start_sequence() {
  local init_mode="$1"
  local with_dry="$2"

  start_base_stack "${init_mode}"

  if ! wait_for_ndt_success 300; then
    warn "等待 NDT 成功超时，但定位基础节点保持运行"
    warn "完成 RViz 初始化后执行："
    if [[ "${with_dry}" == "yes" ]]; then
      warn "  ${SCRIPT_DIR}/g1_navigation.sh continue-dry"
    else
      warn "  ${SCRIPT_DIR}/g1_navigation.sh continue"
    fi
    return 1
  fi

  start_nav2_stack "${with_dry}"
}

enable_dry() {
  source_environment
  local dry_value
  dry_value="$(timeout 8 ros2 param get /g1_cmd_vel_dry_run dry_run 2>/dev/null || true)"
  grep -q "True" <<<"${dry_value}" || die "Dry Run 节点不存在或 dry_run 不是 True"

  ros2 service call \
    /g1_loco_bridge/enable \
    std_srvs/srv/SetBool \
    "{data: true}"
}

disable_dry() {
  source_environment
  if timeout 4 ros2 service list 2>/dev/null | grep -Fxq /g1_loco_bridge/enable; then
    ros2 service call \
      /g1_loco_bridge/enable \
      std_srvs/srv/SetBool \
      "{data: false}" || true
  else
    warn "Dry Run 安全门服务不存在"
  fi
}

stop_all() {
  source_environment || true
  disable_dry || true
  stop_from_pid_file
  cleanup_stale_navigation
  ok "定位与导航系统已停止"
}

show_status() {
  source_environment
  echo "工作空间：${WS}"
  echo "PCD 地图：${MAP_PCD}"
  echo "二维地图：${MAP_YAML}"
  echo

  if [[ -f "${PID_FILE}" ]]; then
    printf '%-18s %-10s %s\n' "进程" "PID" "状态"
    while read -r name pid; do
      if pid_alive "${pid}"; then
        printf '%-18s %-10s %s\n' "${name}" "${pid}" "运行中"
      else
        printf '%-18s %-10s %s\n' "${name}" "${pid}" "已退出"
      fi
    done <"${PID_FILE}"
  else
    warn "没有 PID 文件"
  fi

  echo
  say "关键话题"
  for topic in \
    /livox/lidar \
    /livox/imu \
    /Odometry \
    /cloud_registered_body \
    /global_pcd_map \
    /map \
    /localization_pose \
    /plan \
    /cmd_vel_nav
  do
    if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      printf '[OK]   %s\n' "${topic}"
    else
      printf '[MISS] %s\n' "${topic}"
    fi
  done

  echo
  say "生命周期节点"
  for node in controller_server planner_server recoveries_server bt_navigator; do
    printf '%-22s ' "${node}"
    timeout 5 ros2 lifecycle get "/${node}" 2>/dev/null || echo "未运行"
  done

  echo
  if timeout 4 ros2 node list 2>/dev/null | grep -Fxq /g1_cmd_vel_dry_run; then
    ros2 param get /g1_cmd_vel_dry_run dry_run 2>/dev/null || true
    ros2 param get /g1_cmd_vel_dry_run cmd_vel_topic 2>/dev/null || true
  else
    echo "Dry Run 安全桥：未运行"
  fi
}

show_logs() {
  for file in "${LOG_DIR}"/*.log; do
    echo
    printf '========== %s ==========\n' "$(basename "${file}")"
    [[ -f "${file}" ]] && tail -n 80 "${file}" || true
  done
}

usage() {
  cat <<EOF_USAGE
用法：
  $0 start
  $0 start-dry
  $0 start-auto
  $0 start-auto-dry
  $0 continue
  $0 continue-dry
  $0 enable-dry
  $0 disable-dry
  $0 stop
  $0 restart
  $0 status
  $0 logs

当前阶段推荐：
  $0 start-dry

说明：
  start-dry 只连接 DryRunBackend，不会向机器人发送真实运动命令。
EOF_USAGE
}

main() {
  case "${1:-start}" in
    start) start_sequence manual no ;;
    start-dry) start_sequence manual yes ;;
    start-auto) start_sequence auto no ;;
    start-auto-dry) start_sequence auto yes ;;
    continue) start_nav2_stack no ;;
    continue-dry) start_nav2_stack yes ;;
    enable-dry) enable_dry ;;
    disable-dry) disable_dry ;;
    stop) stop_all ;;
    restart) stop_all; sleep 2; start_sequence manual no ;;
    status) show_status ;;
    logs) show_logs ;;
    -h|--help|help) usage ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
