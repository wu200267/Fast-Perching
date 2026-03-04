#!/usr/bin/env bash

# 用法：
# 1) 直接运行：./sh_utils/bench_table1_our_proposed.sh
# 2) 可选参数：--samples N / --skip-build
# 3) 输出目录：/tmp/fast_perching_table1_our_时间戳
# 说明：该脚本只统计论文 Table I 中 Our Proposed 的 3 组坡度耗时。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

SAMPLES=30
SKIP_BUILD=0
INIT_TIMEOUT_SEC=40
# -110 deg 在严格固定终端状态下可能明显更慢，默认给更宽裕超时避免反复重触发
SAMPLE_TIMEOUT_SEC=180
# 终端切向速度固定为 0（strict 模式）
FIXED_VT_X="0.0"
FIXED_VT_Y="0.0"
# 开发期用于验收维度布局：每个 planning 进程仅打印一次布局日志。
PRINT_OPT_LAYOUT_ONCE="true"
# 兼容旧参数：--fixed-tail-f/--no-fixed-tail-f 仍可解析，但永远忽略。
DEPRECATED_TAIL_F_FLAG_USED=0
DEPRECATED_TAIL_F_VALUE=""

# 帮助信息
usage() {
  cat <<'EOF'
Usage:
  sh_utils/bench_table1_our_proposed.sh [--samples N] [--sample-timeout SEC] [--init-timeout SEC] [--fixed-tail-f V | --no-fixed-tail-f] [--skip-build]

Description:
  Benchmark "Our Proposed" computation time for three surface slopes:
  -70 deg, -90 deg, -110 deg.

  The script launches a minimal planning-only ROS graph, triggers planning
  repeatedly, parses "optmization costs: ... ms" from logs, and reports stats.

Options:
  --samples N            Number of successful samples per slope (default: 30)
  --sample-timeout SEC   Max wait per sample (default: 180)
  --init-timeout SEC     Max wait for node init (default: 40)
  --fixed-tail-f V       Deprecated and ignored (vt-only strict mode keeps tail_f optimizable)
  --no-fixed-tail-f      Deprecated and ignored
  --skip-build           Skip catkin_make Release build
  -h, --help             Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --samples)
      SAMPLES="${2:-}"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --sample-timeout)
      SAMPLE_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --init-timeout)
      INIT_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --fixed-tail-f)
      DEPRECATED_TAIL_F_FLAG_USED=1
      DEPRECATED_TAIL_F_VALUE="${2:-}"
      shift 2
      ;;
    --no-fixed-tail-f)
      DEPRECATED_TAIL_F_FLAG_USED=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

# 参数合法性检查
if ! [[ "${SAMPLES}" =~ ^[0-9]+$ ]] || [[ "${SAMPLES}" -le 0 ]]; then
  echo "--samples must be a positive integer, got: ${SAMPLES}" >&2
  exit 1
fi
if ! [[ "${SAMPLE_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${SAMPLE_TIMEOUT_SEC}" -le 0 ]]; then
  echo "--sample-timeout must be a positive integer (seconds), got: ${SAMPLE_TIMEOUT_SEC}" >&2
  exit 1
fi
if ! [[ "${INIT_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${INIT_TIMEOUT_SEC}" -le 0 ]]; then
  echo "--init-timeout must be a positive integer (seconds), got: ${INIT_TIMEOUT_SEC}" >&2
  exit 1
fi

cd "${REPO_ROOT}"

# 依赖命令检查，缺任何一个就直接退出
for cmd in catkin_make roslaunch rostopic awk sed grep; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Required command not found: ${cmd}" >&2
    exit 1
  fi
done

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  echo "[1/4] Building in Release mode..."
  catkin_make -DCMAKE_BUILD_TYPE=Release
fi

# 加载当前工作空间环境
# shellcheck disable=SC1091
source "${REPO_ROOT}/devel/setup.bash"

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="/tmp/fast_perching_table1_our_${RUN_TAG}"
mkdir -p "${OUT_DIR}"

if [[ "${DEPRECATED_TAIL_F_FLAG_USED}" -eq 1 ]]; then
  echo "[warn] --fixed-tail-f/--no-fixed-tail-f are deprecated and ignored in vt-only strict mode." >&2
  if [[ -n "${DEPRECATED_TAIL_F_VALUE}" ]]; then
    echo "[warn] Ignored --fixed-tail-f value: ${DEPRECATED_TAIL_F_VALUE}" >&2
  fi
fi
echo "[config] fix_terminal_state=true (vt-only), fixed_vt=(${FIXED_VT_X},${FIXED_VT_Y}), tail_f=optimizable"

LAUNCH_PID=""
CURRENT_LOG=""
CURRENT_LAUNCH=""
# 记录每个 case 的超时次数，便于识别“统计值看起来正常但采样链路不稳定”的情况。
declare -A CASE_TIMEOUTS

# 退出时清理 roslaunch 后台进程，避免残留
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 1
    kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

# 生成单个坡度 case 的临时 launch 文件（只启动计时需要的最小节点）
write_case_launch() {
  local theta="$1"
  local out_launch="$2"

  cat > "${out_launch}" <<EOF
<launch>
  <group ns="drone0">
    <node pkg="nodelet" type="nodelet" name="manager" args="manager" output="screen">
      <param name="num_worker_threads" value="16"/>
    </node>

    <node pkg="nodelet" type="nodelet" name="planning" args="load planning/Nodelet manager" output="screen">
      <remap from="~odom" to="odom"/>
      <remap from="~heartbeat" to="heartbeat"/>
      <remap from="~trajectory" to="trajectory"/>
      <remap from="~replanState" to="replanState"/>
      <remap from="~triger" to="/triger"/>
      <remap from="~land_triger" to="/land_triger"/>
      <remap from="~target" to="/target/odom"/>

      <param name="plan_hz" value="5"/>

      <param name="K" value="16"/>
      <param name="vmax" value="6.0"/>
      <param name="amax" value="6.0"/>

      <param name="thrust_max" value="17.0"/>
      <param name="thrust_min" value="5.0"/>
      <param name="omega_max" value="3.0"/>
      <param name="omega_yaw_max" value="0.5"/>

      <param name="robot_l" value="0.02"/>
      <param name="robot_r" value="0.13"/>
      <param name="platform_r" value="1.0"/>
      <!-- rest-to-rest: 终端法向相对速度设为 0 -->
      <param name="v_plus" value="0.0"/>

      <param name="rhoT" value="100000.0"/>
      <param name="rhoP" value="10000000.0"/>
      <param name="rhoV" value="1000.0"/>
      <param name="rhoA" value="1000.0"/>
      <!-- strict 模式下会跳过 rhoVt 成本，这里保留仅为兼容 -->
      <param name="rhoVt" value="1000000.0"/>
      <param name="rhoThrust" value="10000.0"/>
      <param name="rhoOmega" value="100000.0"/>
      <param name="rhoPerchingCollision" value="0.0"/>
      <!-- 仅本 bench 临时 launch 开启 strict 固定终端状态：
           1) fix_terminal_state=true 时，仅 vt 从 L-BFGS 优化向量移除；
           2) tail_f 始终为优化变量，不通过 fixed_tail_f 注入固定值；
           3) 这些参数不会改动系统默认行为（脚本外默认仍由常规 launch 决定）。 -->
      <param name="fix_terminal_state" value="true"/>
      <param name="fixed_vt_x" value="${FIXED_VT_X}"/>
      <param name="fixed_vt_y" value="${FIXED_VT_Y}"/>
      <param name="print_opt_layout_once" value="${PRINT_OPT_LAYOUT_ONCE}"/>

      <!-- 论文设置的初始位置 -->
      <param name="ini_px" value="0.0"/>
      <param name="ini_py" value="0.0"/>
      <param name="ini_pz" value="4.2"/>

      <param name="perching_px" value="4.0"/>
      <param name="perching_py" value="0.0"/>
      <param name="perching_pz" value="4.25"/>
      <param name="perching_vx" value="0.0"/>
      <param name="perching_vy" value="0.0"/>
      <param name="perching_vz" value="0.0"/>

      <param name="perching_axis_x" value="0.0"/>
      <param name="perching_axis_y" value="1.0"/>
      <param name="perching_axis_z" value="0.0"/>
      <param name="perching_theta" value="${theta}"/>

      <param name="replan" value="false"/>
      <param name="pause_debug" value="false"/>
      <!-- benchmark 快速模式：只做优化，不做逐点回放，避免 callback 长时间占用 -->
      <param name="bench_mode" value="true"/>
    </node>
  </group>
</launch>
EOF
}

# 在日志中等待指定关键字出现（带超时）
wait_for_log_pattern() {
  local pattern="$1"
  local timeout="$2"
  local log_file="$3"
  local elapsed=0

  while [[ "${elapsed}" -lt "${timeout}" ]]; do
    if grep -q "${pattern}" "${log_file}" 2>/dev/null; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

# 启动某个坡度 case，并等待 planning 节点就绪
launch_case() {
  local case_name="$1"
  local theta="$2"

  CURRENT_LAUNCH="${OUT_DIR}/bench_${case_name}.launch"
  CURRENT_LOG="${OUT_DIR}/bench_${case_name}.log"

  write_case_launch "${theta}" "${CURRENT_LAUNCH}"

  echo "[case ${case_name}] Launching..."
  stdbuf -oL -eL roslaunch "${CURRENT_LAUNCH}" > "${CURRENT_LOG}" 2>&1 &
  LAUNCH_PID=$!

  if ! wait_for_log_pattern "Planning node initialized!" "${INIT_TIMEOUT_SEC}" "${CURRENT_LOG}"; then
    echo "[case ${case_name}] planning node init timeout. Check log: ${CURRENT_LOG}" >&2
    return 1
  fi
  return 0
}

# 停止当前 case 的 roslaunch
stop_case() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  LAUNCH_PID=""
}

# 统计当前日志里已经出现了多少条 "optmization costs"
current_count() {
  local log_file="$1"
  grep -c "optmization costs:" "${log_file}" 2>/dev/null || true
}

# 发布一次触发消息，让 planner 进行一次轨迹优化
publish_trigger_once() {
  rostopic pub -1 /triger geometry_msgs/PoseStamped \
    "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
    >/dev/null 2>&1
}

# 采样循环：
# 每次触发后等待日志计数 +1，成功才记为一个样本
run_samples() {
  local case_name="$1"
  local log_file="$2"
  local needed="$3"
  local ok=0
  local timeout_count=0

  echo "[case ${case_name}] Collecting ${needed} samples..."
  while [[ "${ok}" -lt "${needed}" ]]; do
    local before after elapsed
    before="$(current_count "${log_file}")"

    publish_trigger_once

    elapsed=0
    while true; do
      after="$(current_count "${log_file}")"
      if [[ "${after}" -gt "${before}" ]]; then
        ok=$((ok + 1))
        echo "[case ${case_name}] sample ${ok}/${needed}"
        break
      fi
      sleep 1
      elapsed=$((elapsed + 1))
      if [[ "${elapsed}" -ge "${SAMPLE_TIMEOUT_SEC}" ]]; then
        echo "[case ${case_name}] timeout waiting for optimization result; retrying this sample..." >&2
        timeout_count=$((timeout_count + 1))
        break
      fi
    done
  done
  CASE_TIMEOUTS["${case_name}"]="${timeout_count}"
}

# 从日志抽取耗时并计算统计量，写入 CSV
summarize_case() {
  local case_name="$1"
  local theta="$2"
  local log_file="$3"
  local csv_file="$4"
  local timeout_count="$5"

  local vals_file="${OUT_DIR}/vals_${case_name}.txt"
  sed -n 's/.*optmization costs: \([0-9.]\+\)ms.*/\1/p' "${log_file}" > "${vals_file}"

  # 统计项：n, mean, std, min, max（单位 ms）
  local stats
  stats="$(awk '
    { x[NR]=$1; s+=$1; if(NR==1||$1<mn) mn=$1; if(NR==1||$1>mx) mx=$1 }
    END {
      if (NR==0) { print "0,NaN,NaN,NaN,NaN"; exit }
      m=s/NR
      for(i=1;i<=NR;i++) v+=(x[i]-m)*(x[i]-m)
      std=sqrt(v/NR)
      printf "%d,%.5f,%.5f,%.5f,%.5f", NR, m, std, mn, mx
    }
  ' "${vals_file}")"

  local n mean std minv maxv
  IFS=',' read -r n mean std minv maxv <<< "${stats}"
  echo "${case_name},${theta},${n},${mean},${std},${minv},${maxv},${timeout_count}" >> "${csv_file}"
}

CSV_FILE="${OUT_DIR}/summary.csv"
echo "case,theta_rad,n,mean_ms,std_ms,min_ms,max_ms,timeouts" > "${CSV_FILE}"

# 三个固定坡度：-70°, -90°, -110°
CASE_NAMES=("minus70" "minus90" "minus110")
CASE_THETA=("-1.2217304764" "-1.5707963268" "-1.9198621772")

echo "[2/4] Running 3 slope cases..."
for i in "${!CASE_NAMES[@]}"; do
  name="${CASE_NAMES[$i]}"
  theta="${CASE_THETA[$i]}"

  launch_case "${name}" "${theta}"
  run_samples "${name}" "${CURRENT_LOG}" "${SAMPLES}"
  stop_case
  summarize_case "${name}" "${theta}" "${OUT_DIR}/bench_${name}.log" "${CSV_FILE}" "${CASE_TIMEOUTS[$name]:-0}"
done

echo "[3/4] Summary (Our Proposed):"
column -s, -t "${CSV_FILE}" || cat "${CSV_FILE}"

# 若存在超时，说明“触发->求解->日志”链路不稳定，当前统计可参考但不建议用于论文对齐。
if awk -F',' 'NR>1 && $8+0>0 {exit 0} END{exit 1}' "${CSV_FILE}"; then
  echo "[warn] Some cases had timeouts. Consider increasing --sample-timeout or checking runtime load." >&2
fi

echo "[4/4] Output directory: ${OUT_DIR}"
echo "Logs and CSV are saved for traceability."
