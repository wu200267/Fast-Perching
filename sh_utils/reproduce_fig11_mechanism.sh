#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

SKIP_BUILD=0
FORCE=0
SAMPLE_DT="0.005"
INIT_TIMEOUT_SEC=40
EXPORT_TIMEOUT_SEC=60
OUT_DIR=""

# Fixed profile for unified mechanism validation v1.
PROFILE_K="32"
PROFILE_RHO_OMEGA="1400000.0"
PROFILE_RHO_THRUST="100000.0"
PROFILE_RHO_COLLISION="1000000.0"

THRUST_MIN="5.0"
THRUST_MAX="17.0"
OMEGA_MAX="3.0"
V_PLUS="0.3"
FIX_TERMINAL_STATE="false"

# Hard gate + warning thresholds (frozen in v1).
OMEGA_INT_THRESHOLD="3.0015"
OMEGA_EXPORT_WARN_THRESHOLD="3.0015"

# Unified scene (frozen in v1): ini_p=(0,0,2.0), perching_px=2.3, perching_vx=0.6
SCENE_INI_PX="0.0"
SCENE_INI_PY="0.0"
SCENE_INI_PZ="2.0"
SCENE_INI_VX="0.0"
SCENE_INI_VY="0.0"
SCENE_INI_VZ="0.0"

SCENE_PERCHING_PX="2.3"
SCENE_PERCHING_PY="0.0"
SCENE_PERCHING_VX="0.6"
SCENE_PERCHING_VY="0.0"
SCENE_PERCHING_VZ="0.0"
SCENE_PERCHING_AXIS_X="0.0"
SCENE_PERCHING_AXIS_Y="1.0"
SCENE_PERCHING_AXIS_Z="0.0"
SCENE_PERCHING_THETA="-1.5708"

CASE_NAMES=("zt_2_0" "zt_1_5" "zt_1_0")
CASE_ZTS=("2.0" "1.5" "1.0")

# Analysis thresholds for mechanism gates.
NORM_EPSILON="1e-6"
RESAMPLE_COUNT="201"
ADAPT_ABS_THRESHOLD_VT2="0.02"
ADAPT_REL_THRESHOLD_VT2="0.2"
ADAPT_ABS_THRESHOLD_VTNORM="0.02"
ADAPT_REL_THRESHOLD_VTNORM="0.2"
TRAJ_CHANGE_THRESHOLD="0.02"
TAU_CHANGE_THRESHOLD="0.03"
OMEGA_CHANGE_THRESHOLD="0.03"

# Runtime retry policy (v1 fixed): runtime-only retry once.
RUNTIME_RETRY_MAX=1
RETRY_COUNT=0
RUNTIME_RETRY_USED=0

# Runtime/audit states.
PREEXISTING_NONEMPTY=0
CLEANED_BEFORE_RUN=0
RUNTIME_FAIL_COUNT=0
FAIL_STAGE="none"
FAILED_GATE_NAME="none"

RED=$'\033[31m'
YELLOW=$'\033[33m'
NC=$'\033[0m'

usage() {
  cat <<'USAGE'
Usage:
  sh_utils/reproduce_fig11_mechanism.sh [--skip-build] [--force] [--sample-dt SEC] [--init-timeout SEC] [--export-timeout SEC] [--out-dir DIR]

Description:
  Unified-scene Fig.11 mechanism validation chain (v1):
  - Fixed profile: K=32, rhoOmega=1.4e6, rhoThrust=1e5
  - Fixed scene: ini_p=(0,0,2.0), perching_px=2.3, perching_vx=0.6
  - z_t sweep: 2.0, 1.5, 1.0
  - fix_terminal_state=false, v_plus=0.3
  - Runtime retry at most once for timeout/runtime failure only
  - Gate thresholds frozen (no post-hoc tuning in v1)
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --sample-dt)
      SAMPLE_DT="${2:-}"
      shift 2
      ;;
    --init-timeout)
      INIT_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --export-timeout)
      EXPORT_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="${2:-}"
      shift 2
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

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${REPO_ROOT}/fig11_outputs/fig11_mechanism_validation_v1"
fi

if [[ -d "${OUT_DIR}" ]]; then
  if [[ -n "$(find "${OUT_DIR}" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
    PREEXISTING_NONEMPTY=1
    if [[ "${FORCE}" -eq 1 ]]; then
      find "${OUT_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
      CLEANED_BEFORE_RUN=1
    else
      echo -e "${RED}[ERROR] Output directory exists and is non-empty: ${OUT_DIR}${NC}" >&2
      echo "Use --force to clear it before a formal v1 rerun." >&2
      exit 1
    fi
  fi
else
  mkdir -p "${OUT_DIR}"
fi

export ROS_HOME="${OUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"
MASTER_PORT=$((15000 + RANDOM % 10000))
export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

GATE_REVIEW_FILE="${OUT_DIR}/mechanism_gate_review.txt"

if ! [[ "${INIT_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${INIT_TIMEOUT_SEC}" -le 0 ]]; then
  echo "--init-timeout must be a positive integer (seconds)." >&2
  exit 1
fi
if ! [[ "${EXPORT_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${EXPORT_TIMEOUT_SEC}" -le 0 ]]; then
  echo "--export-timeout must be a positive integer (seconds)." >&2
  exit 1
fi

cd "${REPO_ROOT}"
for cmd in catkin_make roslaunch rostopic rosparam python3 git grep awk cp find; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Required command not found: ${cmd}" >&2
    exit 1
  fi
done

read_summary_value() {
  local key="$1"
  local file="$2"
  awk -F= -v key="${key}" '$1 == key { print $2; exit }' "${file}"
}

na_if_empty() {
  local value="$1"
  if [[ -z "${value}" ]]; then
    echo "NA"
  else
    echo "${value}"
  fi
}

is_number() {
  [[ "$1" =~ ^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$ ]]
}

max3_or_na() {
  local a="$1"
  local b="$2"
  local c="$3"
  if is_number "${a}" && is_number "${b}" && is_number "${c}"; then
    awk -v a="${a}" -v b="${b}" -v c="${c}" 'BEGIN{m=a; if(b>m)m=b; if(c>m)m=c; printf "%.9f", m}'
  else
    echo "NA"
  fi
}

margin_or_na() {
  local val="$1"
  local thr="$2"
  if is_number "${val}" && is_number "${thr}"; then
    awk -v v="${val}" -v t="${thr}" 'BEGIN{if(t==0){print "NA"}else{printf "%.9f", (v/t)-1.0}}'
  else
    echo "NA"
  fi
}

write_gate_review() {
  local summary_file="$1"

  local overall_result="FAIL"
  local feasibility_gate="NA"
  local adaptation_gate="NA"
  local curve_sync_gate="NA"
  local adaptation_driver="NA"
  local max_abs_omega2_int_overall="NA"

  local traj_20_15="NA"
  local traj_20_10="NA"
  local traj_15_10="NA"
  local tau_20_15="NA"
  local tau_20_10="NA"
  local tau_15_10="NA"
  local omega_20_15="NA"
  local omega_20_10="NA"
  local omega_15_10="NA"

  local traj_threshold="${TRAJ_CHANGE_THRESHOLD}"
  local tau_threshold="${TAU_CHANGE_THRESHOLD}"
  local omega_threshold="${OMEGA_CHANGE_THRESHOLD}"

  local summary_exists="0"
  if [[ -s "${summary_file}" ]]; then
    summary_exists="1"
    overall_result="$(na_if_empty "$(read_summary_value "overall_result" "${summary_file}")")"
    feasibility_gate="$(na_if_empty "$(read_summary_value "feasibility_gate" "${summary_file}")")"
    adaptation_gate="$(na_if_empty "$(read_summary_value "adaptation_gate" "${summary_file}")")"
    curve_sync_gate="$(na_if_empty "$(read_summary_value "curve_sync_gate" "${summary_file}")")"
    adaptation_driver="$(na_if_empty "$(read_summary_value "adaptation_driver" "${summary_file}")")"
    max_abs_omega2_int_overall="$(na_if_empty "$(read_summary_value "max_abs_omega2_int_overall" "${summary_file}")")"

    traj_20_15="$(na_if_empty "$(read_summary_value "traj_rmse_norm_2.0_1.5" "${summary_file}")")"
    traj_20_10="$(na_if_empty "$(read_summary_value "traj_rmse_norm_2.0_1.0" "${summary_file}")")"
    traj_15_10="$(na_if_empty "$(read_summary_value "traj_rmse_norm_1.5_1.0" "${summary_file}")")"
    tau_20_15="$(na_if_empty "$(read_summary_value "tau_rmse_norm_2.0_1.5" "${summary_file}")")"
    tau_20_10="$(na_if_empty "$(read_summary_value "tau_rmse_norm_2.0_1.0" "${summary_file}")")"
    tau_15_10="$(na_if_empty "$(read_summary_value "tau_rmse_norm_1.5_1.0" "${summary_file}")")"
    omega_20_15="$(na_if_empty "$(read_summary_value "omega_rmse_norm_2.0_1.5" "${summary_file}")")"
    omega_20_10="$(na_if_empty "$(read_summary_value "omega_rmse_norm_2.0_1.0" "${summary_file}")")"
    omega_15_10="$(na_if_empty "$(read_summary_value "omega_rmse_norm_1.5_1.0" "${summary_file}")")"

    local parsed_traj_thr
    local parsed_tau_thr
    local parsed_omega_thr
    parsed_traj_thr="$(read_summary_value "traj_change_threshold" "${summary_file}")"
    parsed_tau_thr="$(read_summary_value "tau_change_threshold" "${summary_file}")"
    parsed_omega_thr="$(read_summary_value "omega_change_threshold" "${summary_file}")"
    if is_number "${parsed_traj_thr}"; then traj_threshold="${parsed_traj_thr}"; fi
    if is_number "${parsed_tau_thr}"; then tau_threshold="${parsed_tau_thr}"; fi
    if is_number "${parsed_omega_thr}"; then omega_threshold="${parsed_omega_thr}"; fi
  fi

  local traj_max
  local tau_max
  local omega_max
  local control_max
  local traj_margin
  local tau_margin
  local omega_margin
  local control_margin
  local non_borderline_required="0.100000000"
  local curve_sync_non_borderline="FAIL"

  traj_max="$(max3_or_na "${traj_20_15}" "${traj_20_10}" "${traj_15_10}")"
  tau_max="$(max3_or_na "${tau_20_15}" "${tau_20_10}" "${tau_15_10}")"
  omega_max="$(max3_or_na "${omega_20_15}" "${omega_20_10}" "${omega_15_10}")"

  if is_number "${tau_max}" && is_number "${omega_max}"; then
    control_max="$(awk -v a="${tau_max}" -v b="${omega_max}" 'BEGIN{if(a>b)printf "%.9f",a; else printf "%.9f",b}')"
  else
    control_max="NA"
  fi

  traj_margin="$(margin_or_na "${traj_max}" "${traj_threshold}")"
  tau_margin="$(margin_or_na "${tau_max}" "${tau_threshold}")"
  omega_margin="$(margin_or_na "${omega_max}" "${omega_threshold}")"

  if is_number "${control_max}"; then
    local control_threshold
    control_threshold="$(awk -v a="${tau_threshold}" -v b="${omega_threshold}" 'BEGIN{if(a>b)printf "%.9f",a; else printf "%.9f",b}')"
    control_margin="$(margin_or_na "${control_max}" "${control_threshold}")"
  else
    control_margin="NA"
  fi

  if is_number "${traj_margin}" && is_number "${control_margin}"; then
    if awk -v a="${traj_margin}" -v b="${control_margin}" -v req="${non_borderline_required}" 'BEGIN{exit !((a>=req)&&(b>=req))}'; then
      curve_sync_non_borderline="PASS"
    fi
  fi

  cat > "${GATE_REVIEW_FILE}" <<EOF_REVIEW
generated_at=$(date -u +%Y-%m-%dT%H:%M:%SZ)
output_dir=${OUT_DIR}
summary_exists=${summary_exists}
overall_result=${overall_result}
force_used=${FORCE}
preexisting_nonempty=${PREEXISTING_NONEMPTY}
cleaned_before_run=${CLEANED_BEFORE_RUN}
runtime_retry_max=${RUNTIME_RETRY_MAX}
retry_count=${RETRY_COUNT}
runtime_retry_used=${RUNTIME_RETRY_USED}
runtime_fail_count=${RUNTIME_FAIL_COUNT}
fail_stage=${FAIL_STAGE}
failed_gate_name=${FAILED_GATE_NAME}
feasibility_gate=${feasibility_gate}
adaptation_gate=${adaptation_gate}
curve_sync_gate=${curve_sync_gate}
adaptation_driver=${adaptation_driver}
max_abs_omega2_int_overall=${max_abs_omega2_int_overall}
traj_change_threshold=${traj_threshold}
tau_change_threshold=${tau_threshold}
omega_change_threshold=${omega_threshold}
traj_rmse_norm_2.0_1.5=${traj_20_15}
traj_rmse_norm_2.0_1.0=${traj_20_10}
traj_rmse_norm_1.5_1.0=${traj_15_10}
tau_rmse_norm_2.0_1.5=${tau_20_15}
tau_rmse_norm_2.0_1.0=${tau_20_10}
tau_rmse_norm_1.5_1.0=${tau_15_10}
omega_rmse_norm_2.0_1.5=${omega_20_15}
omega_rmse_norm_2.0_1.0=${omega_20_10}
omega_rmse_norm_1.5_1.0=${omega_15_10}
traj_rmse_norm_max=${traj_max}
tau_rmse_norm_max=${tau_max}
omega_rmse_norm_max=${omega_max}
control_rmse_norm_max=${control_max}
traj_margin_vs_threshold=${traj_margin}
tau_margin_vs_threshold=${tau_margin}
omega_margin_vs_threshold=${omega_margin}
control_margin_vs_threshold=${control_margin}
non_borderline_margin_required=${non_borderline_required}
curve_sync_non_borderline=${curve_sync_non_borderline}
EOF_REVIEW
}

LAUNCH_PID=""
CURRENT_LOG=""
CURRENT_LAUNCH=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 1
    kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  LAUNCH_PID=""
}
trap cleanup EXIT

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

wait_for_nonempty_file() {
  local file="$1"
  local timeout="$2"
  local elapsed=0
  while [[ "${elapsed}" -lt "${timeout}" ]]; do
    if [[ -s "${file}" ]]; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

publish_trigger_once() {
  rostopic pub -1 /triger geometry_msgs/PoseStamped \
    "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
    >/dev/null 2>&1
}

write_case_launch() {
  local case_name="$1"
  local zt="$2"
  local out_launch="$3"
  local case_out_dir="$4"

  cat > "${out_launch}" <<EOF_LAUNCH
<launch>
  <group ns="drone0">
    <node pkg="nodelet" type="nodelet" name="manager" args="manager" output="screen">
      <param name="num_worker_threads" value="16"/>
    </node>

    <node pkg="nodelet" type="nodelet" name="planning" args="load planning/Nodelet manager" output="screen">
      <remap from="~triger" to="/triger"/>

      <param name="K" value="${PROFILE_K}"/>
      <param name="vmax" value="6.0"/>
      <param name="amax" value="6.0"/>

      <param name="thrust_max" value="${THRUST_MAX}"/>
      <param name="thrust_min" value="${THRUST_MIN}"/>
      <param name="omega_max" value="${OMEGA_MAX}"/>
      <param name="omega_yaw_max" value="0.5"/>

      <param name="robot_l" value="0.02"/>
      <param name="robot_r" value="0.13"/>
      <param name="platform_r" value="1.0"/>
      <param name="v_plus" value="${V_PLUS}"/>

      <param name="rhoT" value="100000.0"/>
      <param name="rhoP" value="10000000.0"/>
      <param name="rhoV" value="1000.0"/>
      <param name="rhoA" value="1000.0"/>
      <param name="rhoVt" value="100000.0"/>
      <param name="rhoThrust" value="${PROFILE_RHO_THRUST}"/>
      <param name="rhoOmega" value="${PROFILE_RHO_OMEGA}"/>
      <param name="rhoPerchingCollision" value="${PROFILE_RHO_COLLISION}"/>

      <param name="vt_multistart_enable" value="false"/>
      <param name="fix_terminal_state" value="${FIX_TERMINAL_STATE}"/>
      <param name="bench_mode" value="true"/>
      <param name="replan" value="false"/>
      <param name="pause_debug" value="false"/>

      <param name="fig11_export_enable" value="true"/>
      <param name="fig11_export_dir" value="${case_out_dir}"/>
      <param name="fig11_case_tag" value="${case_name}"/>
      <param name="fig11_sample_dt" value="${SAMPLE_DT}"/>

      <param name="ini_px" value="${SCENE_INI_PX}"/>
      <param name="ini_py" value="${SCENE_INI_PY}"/>
      <param name="ini_pz" value="${SCENE_INI_PZ}"/>
      <param name="ini_vx" value="${SCENE_INI_VX}"/>
      <param name="ini_vy" value="${SCENE_INI_VY}"/>
      <param name="ini_vz" value="${SCENE_INI_VZ}"/>

      <param name="perching_px" value="${SCENE_PERCHING_PX}"/>
      <param name="perching_py" value="${SCENE_PERCHING_PY}"/>
      <param name="perching_pz" value="${zt}"/>
      <param name="perching_vx" value="${SCENE_PERCHING_VX}"/>
      <param name="perching_vy" value="${SCENE_PERCHING_VY}"/>
      <param name="perching_vz" value="${SCENE_PERCHING_VZ}"/>

      <param name="perching_axis_x" value="${SCENE_PERCHING_AXIS_X}"/>
      <param name="perching_axis_y" value="${SCENE_PERCHING_AXIS_Y}"/>
      <param name="perching_axis_z" value="${SCENE_PERCHING_AXIS_Z}"/>
      <param name="perching_theta" value="${SCENE_PERCHING_THETA}"/>
    </node>
  </group>
</launch>
EOF_LAUNCH
}

launch_case() {
  local case_name="$1"
  local zt="$2"

  CURRENT_LAUNCH="${OUT_DIR}/${case_name}.launch"
  CURRENT_LOG="${OUT_DIR}/${case_name}.log"
  write_case_launch "${case_name}" "${zt}" "${CURRENT_LAUNCH}" "${OUT_DIR}"

  echo "[case ${case_name}] launching..."
  stdbuf -oL -eL roslaunch "${CURRENT_LAUNCH}" > "${CURRENT_LOG}" 2>&1 &
  LAUNCH_PID=$!

  if ! wait_for_log_pattern "Planning node initialized!" "${INIT_TIMEOUT_SEC}" "${CURRENT_LOG}"; then
    FAIL_STAGE="launch_timeout"
    return 1
  fi
  rosparam dump "${OUT_DIR}/${case_name}_rosparam.yaml" /drone0/planning >/dev/null
  return 0
}

stop_case() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  LAUNCH_PID=""
}

run_case_once() {
  local case_name="$1"
  local zt="$2"

  if ! launch_case "${case_name}" "${zt}"; then
    stop_case
    return 1
  fi

  echo "[case ${case_name}] triggering planner..."
  publish_trigger_once

  local csv_path="${OUT_DIR}/${case_name}.csv"
  local summary_path="${OUT_DIR}/${case_name}_summary.txt"

  if ! wait_for_nonempty_file "${csv_path}" "${EXPORT_TIMEOUT_SEC}"; then
    FAIL_STAGE="export_timeout_csv"
    stop_case
    return 1
  fi
  if ! wait_for_nonempty_file "${summary_path}" "${EXPORT_TIMEOUT_SEC}"; then
    FAIL_STAGE="export_timeout_summary"
    stop_case
    return 1
  fi

  stop_case
  return 0
}

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  echo "[1/8] Building in Release mode..."
  catkin_make -DCMAKE_BUILD_TYPE=Release
else
  echo "[1/8] Skipping build (--skip-build)."
fi

# shellcheck disable=SC1091
source "${REPO_ROOT}/devel/setup.bash"

GIT_COMMIT="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"

echo "[2/8] Writing run config..."
cat > "${OUT_DIR}/run_config.env" <<EOF_CONFIG
analysis_type=fig11_mechanism_validation_v1
sample_dt=${SAMPLE_DT}
init_timeout_sec=${INIT_TIMEOUT_SEC}
export_timeout_sec=${EXPORT_TIMEOUT_SEC}
force_used=${FORCE}
preexisting_nonempty=${PREEXISTING_NONEMPTY}
cleaned_before_run=${CLEANED_BEFORE_RUN}
runtime_retry_max=${RUNTIME_RETRY_MAX}
ROS_HOME=${ROS_HOME}
ROS_LOG_DIR=${ROS_LOG_DIR}
ROS_MASTER_URI=${ROS_MASTER_URI}
git_commit_hash=${GIT_COMMIT}
git_branch=${GIT_BRANCH}
profile_K=${PROFILE_K}
profile_rhoOmega=${PROFILE_RHO_OMEGA}
profile_rhoThrust=${PROFILE_RHO_THRUST}
profile_rhoPerchingCollision=${PROFILE_RHO_COLLISION}
thrust_min=${THRUST_MIN}
thrust_max=${THRUST_MAX}
omega_max=${OMEGA_MAX}
fix_terminal_state=${FIX_TERMINAL_STATE}
v_plus=${V_PLUS}
scene_ini_p=${SCENE_INI_PX},${SCENE_INI_PY},${SCENE_INI_PZ}
scene_ini_v=${SCENE_INI_VX},${SCENE_INI_VY},${SCENE_INI_VZ}
scene_perching_p_template=${SCENE_PERCHING_PX},${SCENE_PERCHING_PY},z_t
scene_perching_v=${SCENE_PERCHING_VX},${SCENE_PERCHING_VY},${SCENE_PERCHING_VZ}
scene_perching_axis=${SCENE_PERCHING_AXIS_X},${SCENE_PERCHING_AXIS_Y},${SCENE_PERCHING_AXIS_Z}
scene_perching_theta=${SCENE_PERCHING_THETA}
scene_zt_values=2.0,1.5,1.0
scene_role=unified_mechanism_validation_scene_not_claimed_as_exact_paper_scene
norm_epsilon=${NORM_EPSILON}
resample_count=${RESAMPLE_COUNT}
adapt_abs_threshold_vt2=${ADAPT_ABS_THRESHOLD_VT2}
adapt_rel_threshold_vt2=${ADAPT_REL_THRESHOLD_VT2}
adapt_abs_threshold_vtnorm=${ADAPT_ABS_THRESHOLD_VTNORM}
adapt_rel_threshold_vtnorm=${ADAPT_REL_THRESHOLD_VTNORM}
traj_change_threshold=${TRAJ_CHANGE_THRESHOLD}
tau_change_threshold=${TAU_CHANGE_THRESHOLD}
omega_change_threshold=${OMEGA_CHANGE_THRESHOLD}
omega_int_threshold=${OMEGA_INT_THRESHOLD}
omega_export_warn_threshold=${OMEGA_EXPORT_WARN_THRESHOLD}
EOF_CONFIG

echo "[3/8] Running unified z_t sweep..."
RUNTIME_FAILED=0
for i in "${!CASE_NAMES[@]}"; do
  case_name="${CASE_NAMES[$i]}"
  zt="${CASE_ZTS[$i]}"

  attempt=0
  while true; do
    rm -f "${OUT_DIR}/${case_name}.csv" "${OUT_DIR}/${case_name}_summary.txt" \
          "${OUT_DIR}/${case_name}_rosparam.yaml" "${OUT_DIR}/${case_name}.launch" \
          "${OUT_DIR}/${case_name}.log"

    if run_case_once "${case_name}" "${zt}"; then
      break
    fi

    if [[ "${attempt}" -lt "${RUNTIME_RETRY_MAX}" ]]; then
      attempt=$((attempt + 1))
      RETRY_COUNT=$((RETRY_COUNT + 1))
      RUNTIME_RETRY_USED=1
      echo -e "${YELLOW}[WARN][${case_name}] runtime failure (${FAIL_STAGE}), retry ${attempt}/${RUNTIME_RETRY_MAX}${NC}" >&2
      continue
    fi

    RUNTIME_FAIL_COUNT=$((RUNTIME_FAIL_COUNT + 1))
    RUNTIME_FAILED=1
    FAILED_GATE_NAME="runtime"
    break
  done

  if [[ "${RUNTIME_FAILED}" -eq 1 ]]; then
    break
  fi
done

SUMMARY_FILE="${OUT_DIR}/mechanism_summary.txt"
if [[ "${RUNTIME_FAILED}" -eq 1 ]]; then
  echo "[4/8] Runtime failure occurred; writing fail summary..."
  cat > "${SUMMARY_FILE}" <<EOF_FAIL
analysis_type=fig11_mechanism_validation_v1
overall_result=FAIL
result_note=PASS 表示 terminal-state-adjustment 机制在统一场景下成立，不表示论文 Fig.11 原始单标量结果已复现
scene_role=unified_mechanism_validation_scene_not_claimed_as_exact_paper_scene
profile_K=${PROFILE_K}
profile_rhoOmega=${PROFILE_RHO_OMEGA}
profile_rhoThrust=${PROFILE_RHO_THRUST}
scene_ini_p=${SCENE_INI_PX},${SCENE_INI_PY},${SCENE_INI_PZ}
scene_ini_v=${SCENE_INI_VX},${SCENE_INI_VY},${SCENE_INI_VZ}
scene_perching_p_template=${SCENE_PERCHING_PX},${SCENE_PERCHING_PY},z_t
scene_perching_v=${SCENE_PERCHING_VX},${SCENE_PERCHING_VY},${SCENE_PERCHING_VZ}
scene_perching_axis=${SCENE_PERCHING_AXIS_X},${SCENE_PERCHING_AXIS_Y},${SCENE_PERCHING_AXIS_Z}
scene_perching_theta=${SCENE_PERCHING_THETA}
scene_zt_values=2.0,1.5,1.0
runtime_fail_count=${RUNTIME_FAIL_COUNT}
feasibility_gate=FAIL
adaptation_gate=FAIL
curve_sync_gate=FAIL
EOF_FAIL

  write_gate_review "${SUMMARY_FILE}"
  {
    echo "retry_count=${RETRY_COUNT}"
    echo "runtime_retry_used=${RUNTIME_RETRY_USED}"
    echo "runtime_fail_count=${RUNTIME_FAIL_COUNT}"
    echo "fail_stage=${FAIL_STAGE}"
    echo "failed_gate_name=${FAILED_GATE_NAME}"
    echo "overall_result=FAIL"
  } >> "${OUT_DIR}/run_config.env"

  echo -e "${RED}[FAIL] Runtime failure/timeout. See ${OUT_DIR}${NC}" >&2
  exit 1
fi

echo "[4/8] Plotting fig11-style figure..."
python3 "${REPO_ROOT}/sh_utils/plot_fig11.py" \
  --input-dir "${OUT_DIR}" \
  --output "${OUT_DIR}/fig11_reproduced.png"
cp -f "${OUT_DIR}/fig11_reproduced.png" "${OUT_DIR}/fig11_mechanism.png"

echo "[5/8] Running mechanism analysis..."
python3 "${REPO_ROOT}/sh_utils/analyze_fig11_mechanism.py" \
  --input-dir "${OUT_DIR}" \
  --output-summary "${SUMMARY_FILE}" \
  --output-metrics "${OUT_DIR}/mechanism_metrics.tsv" \
  --output-pairwise "${OUT_DIR}/mechanism_pairwise.tsv" \
  --norm-epsilon "${NORM_EPSILON}" \
  --resample-count "${RESAMPLE_COUNT}" \
  --adapt-abs-threshold-vt2 "${ADAPT_ABS_THRESHOLD_VT2}" \
  --adapt-rel-threshold-vt2 "${ADAPT_REL_THRESHOLD_VT2}" \
  --adapt-abs-threshold-vtnorm "${ADAPT_ABS_THRESHOLD_VTNORM}" \
  --adapt-rel-threshold-vtnorm "${ADAPT_REL_THRESHOLD_VTNORM}" \
  --traj-change-threshold "${TRAJ_CHANGE_THRESHOLD}" \
  --tau-change-threshold "${TAU_CHANGE_THRESHOLD}" \
  --omega-change-threshold "${OMEGA_CHANGE_THRESHOLD}" \
  --omega-int-threshold "${OMEGA_INT_THRESHOLD}" \
  --omega-export-warn-threshold "${OMEGA_EXPORT_WARN_THRESHOLD}" \
  --runtime-fail-count "${RUNTIME_FAIL_COUNT}" \
  --profile-k "${PROFILE_K}" \
  --profile-rho-omega "${PROFILE_RHO_OMEGA}" \
  --profile-rho-thrust "${PROFILE_RHO_THRUST}" \
  --scene-ini-p "${SCENE_INI_PX},${SCENE_INI_PY},${SCENE_INI_PZ}" \
  --scene-ini-v "${SCENE_INI_VX},${SCENE_INI_VY},${SCENE_INI_VZ}" \
  --scene-perching-p-template "${SCENE_PERCHING_PX},${SCENE_PERCHING_PY},z_t" \
  --scene-perching-v "${SCENE_PERCHING_VX},${SCENE_PERCHING_VY},${SCENE_PERCHING_VZ}" \
  --scene-perching-axis "${SCENE_PERCHING_AXIS_X},${SCENE_PERCHING_AXIS_Y},${SCENE_PERCHING_AXIS_Z}" \
  --scene-perching-theta "${SCENE_PERCHING_THETA}" \
  --scene-zt-values "2.0,1.5,1.0"

OVERALL_RESULT="$(read_summary_value "overall_result" "${SUMMARY_FILE}")"
FEASIBILITY_GATE="$(read_summary_value "feasibility_gate" "${SUMMARY_FILE}")"
ADAPTATION_GATE="$(read_summary_value "adaptation_gate" "${SUMMARY_FILE}")"
CURVE_SYNC_GATE="$(read_summary_value "curve_sync_gate" "${SUMMARY_FILE}")"
ADAPTATION_DRIVER="$(read_summary_value "adaptation_driver" "${SUMMARY_FILE}")"
STRICT_COUNT="$(read_summary_value "strict_count" "${SUMMARY_FILE}")"
RELAXED_COUNT="$(read_summary_value "relaxed_count" "${SUMMARY_FILE}")"
FALLBACK_COUNT="$(read_summary_value "fallback_count" "${SUMMARY_FILE}")"

FAIL_STAGE="gate_eval"
FAILED_GATE_NAME="none"
if [[ "${OVERALL_RESULT}" != "PASS" ]]; then
  if [[ "${FEASIBILITY_GATE}" == "FAIL" ]]; then
    FAILED_GATE_NAME="feasibility_gate"
  elif [[ "${ADAPTATION_GATE}" == "FAIL" ]]; then
    FAILED_GATE_NAME="adaptation_gate"
  elif [[ "${CURVE_SYNC_GATE}" == "FAIL" ]]; then
    FAILED_GATE_NAME="curve_sync_gate"
  else
    FAILED_GATE_NAME="gate_eval_unknown"
  fi
else
  FAIL_STAGE="none"
fi

write_gate_review "${SUMMARY_FILE}"

echo "[6/8] Updating Fig.11 current entry pointer..."
mkdir -p "${REPO_ROOT}/fig11_outputs"
cat > "${REPO_ROOT}/fig11_outputs/CURRENT_FIG11_ENTRY.txt" <<EOF_ENTRY
generated_at=$(date -u +%Y-%m-%dT%H:%M:%SZ)
entry_type=mechanism_validation
entry_dir=${OUT_DIR}
entry_note=vt_mapping_dual_baseline remains historical variable-mapping audit entry
EOF_ENTRY

if [[ "${OVERALL_RESULT}" == "PASS" ]]; then
  cat > "${OUT_DIR}/mechanism_conclusion.txt" <<'EOF_CONCLUSION'
结论（机制验证版）：
在统一场景下（仅改变 z_t），二维切向终端变量 v_t 本体（vt1/vt2/vt_norm）与轨迹/控制曲线共同发生系统性变化，说明 terminal-state-adjustment 机制得到验证。
本结论用于验证 Fig.11 的机制目标，不等同于论文原图单标量结果的逐点精确复现。
EOF_CONCLUSION
fi

echo "[7/8] Finalizing metadata..."
{
  echo "retry_count=${RETRY_COUNT}"
  echo "runtime_retry_used=${RUNTIME_RETRY_USED}"
  echo "runtime_fail_count=${RUNTIME_FAIL_COUNT}"
  echo "fail_stage=${FAIL_STAGE}"
  echo "failed_gate_name=${FAILED_GATE_NAME}"
  echo "overall_result=${OVERALL_RESULT}"
  echo "feasibility_gate=${FEASIBILITY_GATE}"
  echo "adaptation_gate=${ADAPTATION_GATE}"
  echo "curve_sync_gate=${CURVE_SYNC_GATE}"
  echo "adaptation_driver=${ADAPTATION_DRIVER}"
  echo "strict_count=${STRICT_COUNT}"
  echo "relaxed_count=${RELAXED_COUNT}"
  echo "fallback_count=${FALLBACK_COUNT}"
  echo "current_fig11_entry=${OUT_DIR}"
} >> "${OUT_DIR}/run_config.env"

if [[ "${OVERALL_RESULT}" != "PASS" ]]; then
  echo -e "${RED}[FAIL] Mechanism gate not satisfied. See ${OUT_DIR}/mechanism_summary.txt and mechanism_gate_review.txt${NC}" >&2
  exit 1
fi

echo "[8/8] Done."
echo "Output directory: ${OUT_DIR}"
echo "  - fig11_reproduced.png"
echo "  - fig11_mechanism.png"
echo "  - zt_*.csv / zt_*_summary.txt"
echo "  - mechanism_metrics.tsv"
echo "  - mechanism_pairwise.tsv"
echo "  - mechanism_summary.txt"
echo "  - mechanism_gate_review.txt"
echo "  - mechanism_conclusion.txt"
echo "  - run_config.env"
