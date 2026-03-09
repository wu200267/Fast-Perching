#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

SKIP_BUILD=0
FORCE=0
OUT_DIR="${REPO_ROOT}/fig11_outputs/fig11_paperlike_best_effort"

SAMPLE_DT="0.005"
INIT_TIMEOUT_SEC=40
EXPORT_TIMEOUT_SEC=120

CASE_NAMES=("zt_2_0" "zt_1_5" "zt_1_0")
CASE_ZTS=("2.0" "1.5" "1.0")

# Fixed paper-like profile/scene.
PROFILE_K="32"
PROFILE_RHO_OMEGA="1000000.0"
PROFILE_RHO_THRUST="100000.0"
PROFILE_RHO_COLLISION="1000000.0"

SCENE_INI_PX="0.0"
SCENE_INI_PY="0.0"
SCENE_INI_PZ="2.0"
SCENE_INI_VX="0.0"
SCENE_INI_VY="0.0"
SCENE_INI_VZ="0.0"

SCENE_PERCHING_PX="4.0"
SCENE_PERCHING_PY="0.0"
SCENE_PERCHING_VX="0.0"
SCENE_PERCHING_VY="0.0"
SCENE_PERCHING_VZ="0.0"

SCENE_AXIS_X="0.0"
SCENE_AXIS_Y="1.0"
SCENE_AXIS_Z="0.0"
SCENE_THETA="-1.5708"

FIX_TERMINAL_STATE="false"
V_PLUS="0.3"
THRUST_MIN="5.0"
THRUST_MAX="17.0"
OMEGA_MAX="3.0"

VT_MULTISTART_ENABLE="true"
VT_MULTISTART_SEEDS="0.0,0.1,-0.1,0.4,-0.4"
VT_MULTISTART_EPS_STRICT="0.001"
VT_MULTISTART_EPS_RELAXED="0.01"

RED=$'\033[31m'
NC=$'\033[0m'

usage() {
  cat <<'USAGE'
Usage:
  sh_utils/reproduce_fig11_paperlike_visual.sh [--skip-build] [--force] [--out-dir DIR]

Description:
  Active paper-like visual reproduction workflow for Fig.11.
  This workflow targets visual/behavioral similarity to paper Fig.11,
  not strict source-code state matching and not mechanism-proof PASS/FAIL.
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
  echo "out-dir cannot be empty" >&2
  exit 1
fi

if [[ -d "${OUT_DIR}" ]]; then
  if [[ -n "$(find "${OUT_DIR}" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
    if [[ "${FORCE}" -eq 1 ]]; then
      find "${OUT_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
    else
      echo -e "${RED}[ERROR] output directory exists and is non-empty: ${OUT_DIR}${NC}" >&2
      echo "Use --force to clear it first." >&2
      exit 1
    fi
  fi
else
  mkdir -p "${OUT_DIR}"
fi

for cmd in catkin_make roslaunch rostopic rosparam python3 git grep awk; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Required command not found: ${cmd}" >&2
    exit 1
  fi
done

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

      <param name="vt_multistart_enable" value="${VT_MULTISTART_ENABLE}"/>
      <param name="vt_multistart_vt2_seeds" value="${VT_MULTISTART_SEEDS}"/>
      <param name="vt_multistart_eps_strict" value="${VT_MULTISTART_EPS_STRICT}"/>
      <param name="vt_multistart_eps_relaxed" value="${VT_MULTISTART_EPS_RELAXED}"/>

      <param name="fix_terminal_state" value="${FIX_TERMINAL_STATE}"/>
      <param name="bench_mode" value="true"/>
      <param name="replan" value="false"/>
      <param name="pause_debug" value="false"/>

      <param name="fig11_export_enable" value="true"/>
      <param name="fig11_export_dir" value="${OUT_DIR}"/>
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

      <param name="perching_axis_x" value="${SCENE_AXIS_X}"/>
      <param name="perching_axis_y" value="${SCENE_AXIS_Y}"/>
      <param name="perching_axis_z" value="${SCENE_AXIS_Z}"/>
      <param name="perching_theta" value="${SCENE_THETA}"/>
    </node>
  </group>
</launch>
EOF_LAUNCH
}

LAUNCH_PID=""
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

cd "${REPO_ROOT}"
if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  echo "[1/7] Building in Release mode..."
  catkin_make -DCMAKE_BUILD_TYPE=Release
fi

# shellcheck disable=SC1091
source "${REPO_ROOT}/devel/setup.bash"

export ROS_HOME="${OUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"
MASTER_PORT=$((15000 + RANDOM % 10000))
export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

echo "[2/7] Writing run config..."
GIT_COMMIT="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
git status --short > "${OUT_DIR}/git_status_short.txt" 2>/dev/null || true
echo "${GIT_COMMIT}" > "${OUT_DIR}/git_commit_hash.txt"
echo "${GIT_BRANCH}" > "${OUT_DIR}/git_branch.txt"

cat > "${OUT_DIR}/run_config.env" <<EOF_CFG
workflow=paperlike_visual_reproduction
entry_active=true
main_target=visual_similarity_to_paper_fig11
note=This workflow is visual best-effort, not mechanism proof and not exact numeric reproduction.

sample_dt=${SAMPLE_DT}
init_timeout_sec=${INIT_TIMEOUT_SEC}
export_timeout_sec=${EXPORT_TIMEOUT_SEC}

profile_K=${PROFILE_K}
profile_rhoOmega=${PROFILE_RHO_OMEGA}
profile_rhoThrust=${PROFILE_RHO_THRUST}
profile_rhoPerchingCollision=${PROFILE_RHO_COLLISION}

scene_ini_p=${SCENE_INI_PX},${SCENE_INI_PY},${SCENE_INI_PZ}
scene_ini_v=${SCENE_INI_VX},${SCENE_INI_VY},${SCENE_INI_VZ}
scene_perching_p0=${SCENE_PERCHING_PX},${SCENE_PERCHING_PY},z_t
scene_perching_v=${SCENE_PERCHING_VX},${SCENE_PERCHING_VY},${SCENE_PERCHING_VZ}
scene_surface_axis=${SCENE_AXIS_X},${SCENE_AXIS_Y},${SCENE_AXIS_Z}
scene_surface_theta=${SCENE_THETA}
scene_zt_values=2.0,1.5,1.0

fix_terminal_state=${FIX_TERMINAL_STATE}
v_plus=${V_PLUS}
thrust_min=${THRUST_MIN}
thrust_max=${THRUST_MAX}
omega_max=${OMEGA_MAX}
vt_multistart_enable=${VT_MULTISTART_ENABLE}
vt_multistart_vt2_seeds=${VT_MULTISTART_SEEDS}
vt_multistart_eps_strict=${VT_MULTISTART_EPS_STRICT}
vt_multistart_eps_relaxed=${VT_MULTISTART_EPS_RELAXED}

git_commit_hash=${GIT_COMMIT}
git_branch=${GIT_BRANCH}
EOF_CFG

printf "case\tstatus\tnotes\n" > "${OUT_DIR}/attempt_report.tsv"

echo "[3/7] Running 3 fixed z_t cases..."
for i in "${!CASE_NAMES[@]}"; do
  case_name="${CASE_NAMES[$i]}"
  zt="${CASE_ZTS[$i]}"
  launch_file="${OUT_DIR}/${case_name}.launch"
  log_file="${OUT_DIR}/${case_name}.log"

  write_case_launch "${case_name}" "${zt}" "${launch_file}"

  stdbuf -oL -eL roslaunch "${launch_file}" > "${log_file}" 2>&1 &
  LAUNCH_PID=$!

  if ! wait_for_log_pattern "Planning node initialized!" "${INIT_TIMEOUT_SEC}" "${log_file}"; then
    printf "%s\t%s\t%s\n" "${case_name}" "FAIL" "init_timeout" >> "${OUT_DIR}/attempt_report.tsv"
    cleanup
    echo -e "${RED}[FAIL] ${case_name} init timeout${NC}" >&2
    exit 1
  fi

  rosparam dump "${OUT_DIR}/${case_name}_rosparam.yaml" /drone0/planning >/dev/null
  publish_trigger_once

  if ! wait_for_nonempty_file "${OUT_DIR}/${case_name}.csv" "${EXPORT_TIMEOUT_SEC}"; then
    printf "%s\t%s\t%s\n" "${case_name}" "FAIL" "csv_timeout" >> "${OUT_DIR}/attempt_report.tsv"
    cleanup
    echo -e "${RED}[FAIL] ${case_name} csv timeout${NC}" >&2
    exit 1
  fi

  if ! wait_for_nonempty_file "${OUT_DIR}/${case_name}_summary.txt" "${EXPORT_TIMEOUT_SEC}"; then
    printf "%s\t%s\t%s\n" "${case_name}" "FAIL" "summary_timeout" >> "${OUT_DIR}/attempt_report.tsv"
    cleanup
    echo -e "${RED}[FAIL] ${case_name} summary timeout${NC}" >&2
    exit 1
  fi

  printf "%s\t%s\t%s\n" "${case_name}" "PASS" "runtime_ok" >> "${OUT_DIR}/attempt_report.tsv"
  cleanup
  echo "  - ${case_name}: done"
done

echo "[4/7] Plotting fig11_reproduced.png..."
python3 "${REPO_ROOT}/sh_utils/plot_fig11.py" \
  --input-dir "${OUT_DIR}" \
  --output "${OUT_DIR}/fig11_reproduced.png"

echo "[5/7] Running visual analysis..."
python3 "${REPO_ROOT}/sh_utils/analyze_fig11_paperlike_visual.py" \
  --input-dir "${OUT_DIR}" \
  --paper-crop "${REPO_ROOT}/fig11_outputs/vt_mapping_dual_baseline/paper_fig11_crop.png" \
  --review-out "${OUT_DIR}/best_effort_visual_review.txt" \
  --side-by-side-out "${OUT_DIR}/fig11_side_by_side_paperlike.png"

echo "[6/7] Writing paperlike entry pointer..."
cat > "${REPO_ROOT}/fig11_outputs/CURRENT_FIG11_PAPERLIKE_ENTRY.txt" <<EOF_ENTRY
generated_at=$(date -u +%Y-%m-%dT%H:%M:%SZ)
entry_type=paperlike_visual_reproduction
entry_dir=${OUT_DIR}
entry_note=visual similarity workflow; not mechanism proof; not exact numeric reproduction
EOF_ENTRY

echo "[7/7] Done"
echo "Output directory: ${OUT_DIR}"
echo "Generated files:"
echo "  - fig11_reproduced.png"
echo "  - fig11_side_by_side_paperlike.png"
echo "  - best_effort_visual_review.txt"
echo "  - zt_*.csv, zt_*_summary.txt"
echo "  - attempt_report.tsv, run_config.env"
