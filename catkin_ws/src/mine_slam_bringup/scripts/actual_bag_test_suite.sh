#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: actual_bag_test_suite.sh --bag PATH [options]

Options:
  --out DIR              Evidence suite output directory.
  --smoke-duration SEC   Short replay duration in seconds. Default: 60.
  --full-duration SEC    Full replay duration in seconds. Default: bag duration.
  --rate RATE            rosbag play rate for both replays. Default: 1.0.
  --local-odometry-config PATH
                         Override the local odometry YAML passed to replay.
  --center-topic TOPIC   Source bag topic remapped to /velodyne_points.
                         Default: /velodyne_points.
  --left-topic TOPIC     Source bag topic remapped to /left/lslidar_point_cloud.
                         Default: /left/lslidar_point_cloud.
  --right-topic TOPIC    Source bag topic remapped to /right/velodyne_points.
                         Default: /right/velodyne_points.
  --imu-topic TOPIC      Source bag IMU topic remapped to /imu/data.
                         Default: /imu/data.
  --time-reference-topic TOPIC
                         Source bag time reference remapped to /time_reference.
                         Default: /time_reference.
  --no-time-reference    Do not require or play a time reference topic. This is
                         only allowed for LiDAR+IMU-only initial bag tests and
                         does not satisfy final PPS/PTP time-sync evidence.
  --initial-velocity-topic TOPIC
                         Source bag velocity reference for start-only audit.
                         Default: /novatel_data/inspvax.
  --no-initial-velocity-reference
                         Declare that this initial LiDAR+IMU test bag has no
                         velocity reference topic. The suite records this as
                         not present and does not inspect a default topic.
  --execute              Run the generated suite command after planning.
  --skip-bag-inspect     Skip rosbag inspection. Intended for dry-run tests only;
                         requires --full-duration.
  -h, --help             Show this help.

Tunnel.bag policy:
  LiDAR+IMU-only bags are accepted as initial algorithm test data. PLC feedback
  is recorded as not present and the suite remains ineligible for final field
  acceptance. Any velocity reference is for initial audit only and is not played
  into SLAM.
EOF
}

die() {
  echo "actual_bag_test_suite: $*" >&2
  exit 1
}

is_positive_number() {
  python3 - "$1" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
except Exception:
    sys.exit(1)
sys.exit(0 if math.isfinite(value) and value > 0.0 else 1)
PY
}

absolute_path() {
  python3 - "$1" <<'PY'
import os
import sys
print(os.path.abspath(sys.argv[1]))
PY
}

shell_quote() {
  python3 - "$1" <<'PY'
import shlex
import sys
print(shlex.quote(sys.argv[1]))
PY
}

safe_token() {
  python3 - "$1" <<'PY'
import re
import sys
token = re.sub(r"[^A-Za-z0-9]+", "_", sys.argv[1]).strip("_")
print(token or "value")
PY
}

is_ros_topic_name() {
  python3 - "$1" <<'PY'
import re
import sys

topic = sys.argv[1]
ok = (
    re.fullmatch(r"/[A-Za-z0-9_][A-Za-z0-9_/]*", topic) is not None
    and "//" not in topic
    and "/." not in topic
    and topic != "/"
)
sys.exit(0 if ok else 1)
PY
}

bag_duration_seconds() {
  local bag="$1"
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck source=/opt/ros/noetic/setup.bash
    source /opt/ros/noetic/setup.bash
  fi
  python3 - "$bag" <<'PY'
import math
import sys

import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path)
try:
    duration = bag.get_end_time() - bag.get_start_time()
finally:
    bag.close()

if not math.isfinite(duration) or duration <= 0.0:
    raise SystemExit(2)
print(max(1, int(math.ceil(duration))))
PY
}

write_executable() {
  local path="$1"
  chmod +x "$path"
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
replay_script="$script_dir/actual_bag_replay.sh"

bag_path=""
out_dir=""
smoke_duration_s="60"
full_duration_s="auto"
full_duration_explicit=0
rate="1.0"
local_odometry_config=""
center_topic="/velodyne_points"
left_topic="/left/lslidar_point_cloud"
right_topic="/right/velodyne_points"
imu_topic="/imu/data"
time_reference_topic="/time_reference"
time_reference_topic_explicit=0
use_time_reference=1
initial_velocity_topic="/novatel_data/inspvax"
initial_velocity_topic_explicit=0
use_initial_velocity_reference=1
execute_suite=0
skip_bag_inspect=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || die "--bag requires a value"
      bag_path="$2"
      shift 2
      ;;
    --out)
      [[ $# -ge 2 ]] || die "--out requires a value"
      out_dir="$2"
      shift 2
      ;;
    --smoke-duration)
      [[ $# -ge 2 ]] || die "--smoke-duration requires a value"
      smoke_duration_s="$2"
      shift 2
      ;;
    --full-duration)
      [[ $# -ge 2 ]] || die "--full-duration requires a value"
      full_duration_s="$2"
      full_duration_explicit=1
      shift 2
      ;;
    --rate)
      [[ $# -ge 2 ]] || die "--rate requires a value"
      rate="$2"
      shift 2
      ;;
    --local-odometry-config)
      [[ $# -ge 2 ]] || die "--local-odometry-config requires a value"
      local_odometry_config="$2"
      shift 2
      ;;
    --center-topic)
      [[ $# -ge 2 ]] || die "--center-topic requires a value"
      center_topic="$2"
      shift 2
      ;;
    --left-topic)
      [[ $# -ge 2 ]] || die "--left-topic requires a value"
      left_topic="$2"
      shift 2
      ;;
    --right-topic)
      [[ $# -ge 2 ]] || die "--right-topic requires a value"
      right_topic="$2"
      shift 2
      ;;
    --imu-topic)
      [[ $# -ge 2 ]] || die "--imu-topic requires a value"
      imu_topic="$2"
      shift 2
      ;;
    --time-reference-topic)
      [[ $# -ge 2 ]] || die "--time-reference-topic requires a value"
      time_reference_topic="$2"
      time_reference_topic_explicit=1
      shift 2
      ;;
    --no-time-reference)
      use_time_reference=0
      shift
      ;;
    --initial-velocity-topic)
      [[ $# -ge 2 ]] || die "--initial-velocity-topic requires a value"
      initial_velocity_topic="$2"
      initial_velocity_topic_explicit=1
      shift 2
      ;;
    --no-initial-velocity-reference)
      use_initial_velocity_reference=0
      shift
      ;;
    --execute)
      execute_suite=1
      shift
      ;;
    --skip-bag-inspect)
      skip_bag_inspect=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

[[ -n "$bag_path" ]] || die "--bag is required"
[[ -f "$replay_script" ]] || die "actual_bag_replay.sh not found: $replay_script"
is_positive_number "$smoke_duration_s" || die "--smoke-duration must be a positive finite number"
is_positive_number "$rate" || die "--rate must be a positive finite number"
is_ros_topic_name "$center_topic" || die "--center-topic must be an absolute ROS topic"
is_ros_topic_name "$left_topic" || die "--left-topic must be an absolute ROS topic"
is_ros_topic_name "$right_topic" || die "--right-topic must be an absolute ROS topic"
is_ros_topic_name "$imu_topic" || die "--imu-topic must be an absolute ROS topic"
if [[ "$use_time_reference" -eq 1 ]]; then
  is_ros_topic_name "$time_reference_topic" || die "--time-reference-topic must be an absolute ROS topic"
elif [[ "$time_reference_topic_explicit" -eq 1 ]]; then
  die "--no-time-reference cannot be combined with --time-reference-topic"
fi
if [[ "$use_initial_velocity_reference" -eq 1 ]]; then
  is_ros_topic_name "$initial_velocity_topic" || die "--initial-velocity-topic must be an absolute ROS topic"
elif [[ "$initial_velocity_topic_explicit" -eq 1 ]]; then
  die "--no-initial-velocity-reference cannot be combined with --initial-velocity-topic"
fi

bag_abs="$(absolute_path "$bag_path")"
[[ -f "$bag_abs" ]] || die "bag does not exist: $bag_abs"

if [[ "$full_duration_s" == "auto" ]]; then
  if [[ "$skip_bag_inspect" -eq 1 || "$full_duration_explicit" -eq 1 ]]; then
    die "--full-duration must be a positive number when bag inspection is skipped"
  fi
  if ! full_duration_s="$(bag_duration_seconds "$bag_abs")"; then
    die "failed to inspect bag duration: $bag_abs"
  fi
else
  is_positive_number "$full_duration_s" || die "--full-duration must be a positive finite number"
fi

local_odometry_config_report="launch_default"
local_odometry_arg=""
if [[ -n "$local_odometry_config" ]]; then
  local_odometry_config_abs="$(absolute_path "$local_odometry_config")"
  [[ -f "$local_odometry_config_abs" ]] || die "local odometry config does not exist: $local_odometry_config_abs"
  local_odometry_config_report="$local_odometry_config_abs"
  local_odometry_arg=" --local-odometry-config $(shell_quote "$local_odometry_config_abs")"
fi

if [[ -z "$out_dir" ]]; then
  out_dir="actual_bag_test_suite_$(date -u +%Y%m%dT%H%M%SZ)"
fi

out_abs="$(absolute_path "$out_dir")"
reports_dir="$out_abs/reports"
commands_dir="$out_abs/commands"
logs_dir="$out_abs/logs"
mkdir -p "$reports_dir" "$commands_dir" "$logs_dir"

rate_token="$(safe_token "$rate")"
smoke_token="$(safe_token "$smoke_duration_s")"
full_token="$(safe_token "$full_duration_s")"
smoke_dir_name="smoke_${smoke_token}s_rate${rate_token}"
full_dir_name="full_${full_token}s_rate${rate_token}"
smoke_out="$out_abs/$smoke_dir_name"
full_out="$out_abs/$full_dir_name"
skip_bag_inspect_arg=""
if [[ "$skip_bag_inspect" -eq 1 ]]; then
  skip_bag_inspect_arg=" --skip-bag-inspect"
fi
time_reference_arg=" --time-reference-topic $(shell_quote "$time_reference_topic")"
reported_time_reference_topic="$time_reference_topic"
time_reference_status="PRESENT_REQUIRED"
time_sync_evidence_status="INITIAL_TIME_STATUS_CAPTURE_REQUIRED"
if [[ "$use_time_reference" -eq 0 ]]; then
  time_reference_arg=" --no-time-reference"
  reported_time_reference_topic="NONE"
  time_reference_status="NOT_PRESENT_INITIAL_TEST"
  time_sync_evidence_status="NOT_PRESENT_INITIAL_TEST"
fi

actual_bag_test_scope="INITIAL_LIDAR_IMU_ONLY"
bag_sensor_set="LIDAR_IMU_ONLY"
plc_feedback_status="NOT_PRESENT_NA"
plc_feedback_gate_status="NA_INITIAL_TEST"
machine_motion_assumption="CONTINUOUS_MOTION"
vibration_profile="NORMAL"
initial_velocity_reference_policy="START_ONLY_AUDIT"
initial_velocity_reference_required="YES"
reported_initial_velocity_topic="$initial_velocity_topic"
initial_velocity_reference_status="PENDING_CAPTURE"
initial_velocity_arg=" --initial-velocity-topic $(shell_quote "$initial_velocity_topic")"
if [[ "$use_initial_velocity_reference" -eq 0 ]]; then
  initial_velocity_reference_policy="NOT_AVAILABLE_INITIAL_TEST"
  initial_velocity_reference_required="NO"
  reported_initial_velocity_topic="NONE"
  initial_velocity_reference_status="NOT_PRESENT_INITIAL_TEST"
  initial_velocity_arg=" --no-initial-velocity-reference"
fi
field_acceptance_requires_plc_feedback="YES"
field_acceptance_eligible="NO"
field_acceptance_status="NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
plc_feedback_collection_command='record_session.sh --topics "/points_raw /tf /tf_static /plc/left_track_speed /plc/right_track_speed /plc/cutting_on /machine/state" then commands/validate_evidence.sh'
section_export_collection_command="record_session.sh generated commands/capture_section_export.sh"
pps_ptp_wiring_collection_command="record_session.sh generated commands/capture_time_sync.sh && commands/capture_pps_ptp_wiring.sh"
power_loss_resume_collection_command="record_session.sh generated commands/capture_power_loss_resume.sh"
runtime_deployment_collection_command="runtime_ops.sh then record_session.sh generated commands/capture_runtime_health.sh && commands/capture_runtime_deployment.sh"
runtime_stability_24h_collection_command="record_session.sh generated commands/run_runtime_stability.sh"
field_acceptance_collection_command="record_session.sh generated commands/capture_field_acceptance.sh && commands/validate_evidence.sh"
topic_arg=" --center-topic $(shell_quote "$center_topic") --left-topic $(shell_quote "$left_topic") --right-topic $(shell_quote "$right_topic") --imu-topic $(shell_quote "$imu_topic")$time_reference_arg$initial_velocity_arg"

cat > "$commands_dir/run_smoke_replay.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
$(shell_quote "$replay_script") --bag $(shell_quote "$bag_abs") --out $(shell_quote "$smoke_out") --duration $smoke_duration_s --rate $rate$local_odometry_arg$topic_arg$skip_bag_inspect_arg --execute
EOF
write_executable "$commands_dir/run_smoke_replay.sh"

cat > "$commands_dir/run_full_replay.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
$(shell_quote "$replay_script") --bag $(shell_quote "$bag_abs") --out $(shell_quote "$full_out") --duration $full_duration_s --rate $rate$local_odometry_arg$topic_arg$skip_bag_inspect_arg --execute
EOF
write_executable "$commands_dir/run_full_replay.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -uo pipefail'
  echo "suite_root=$(shell_quote "$out_abs")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  echo "commands_dir=$(shell_quote "$commands_dir")"
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "logs_dir=$(shell_quote "$logs_dir")"
  echo "smoke_dir=$(shell_quote "$smoke_out")"
  echo "full_dir=$(shell_quote "$full_out")"
  echo "smoke_duration_s=$(shell_quote "$smoke_duration_s")"
  echo "full_duration_s=$(shell_quote "$full_duration_s")"
  echo "rate=$(shell_quote "$rate")"
  echo "time_reference_status=$(shell_quote "$time_reference_status")"
  echo "time_sync_evidence_status=$(shell_quote "$time_sync_evidence_status")"
  echo "initial_velocity_reference_status=$(shell_quote "$initial_velocity_reference_status")"
  echo "initial_velocity_reference_required=$(shell_quote "$initial_velocity_reference_required")"
  echo "reported_initial_velocity_topic=$(shell_quote "$reported_initial_velocity_topic")"
  echo "initial_velocity_reference_policy=$(shell_quote "$initial_velocity_reference_policy")"
  cat <<'EOF'
mkdir -p "$reports_dir" "$logs_dir"

summary_value() {
  local key="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    sed -n "s/^${key}=//p" "$path" | tail -n 1
  fi
}

validation_overall() {
  local path="$1"
  if [[ -f "$path" ]]; then
    sed -n 's/^overall=\([^;]*\).*/\1/p' "$path" | tail -n 1
  fi
}

run_event_validation() {
  local label="$1"
  local replay_dir="$2"
  local log_file="$logs_dir/${label}_event_validation.log"
  local report_file="$replay_dir/reports/actual_bag_replay_hil_validation_report.txt"
  local status="FAIL"
  local validation_status=""
  if [[ -x "$replay_dir/commands/validate_actual_bag_events.sh" ]]; then
    if "$replay_dir/commands/validate_actual_bag_events.sh" > "$log_file" 2>&1; then
      validation_status="$(validation_overall "$report_file")"
      if [[ "$validation_status" == "PASS" ]]; then
        status="PASS"
      fi
    fi
  fi
  echo "$status"
}

overall="PASS"
smoke_replay_status="FAIL"
full_replay_status="FAIL"
smoke_event_validation_status="FAIL"
full_event_validation_status="FAIL"
smoke_replay_exit=0
full_replay_exit=0

if "$commands_dir/run_smoke_replay.sh" > "$logs_dir/smoke_replay_suite.log" 2>&1; then
  smoke_replay_status="$(summary_value actual_bag_replay_status "$smoke_dir/reports/actual_bag_replay_summary.txt")"
else
  smoke_replay_exit=$?
fi
if [[ "$smoke_replay_status" != "PASS" ]]; then
  overall="FAIL"
fi
smoke_event_validation_status="$(run_event_validation smoke "$smoke_dir")"
if [[ "$smoke_event_validation_status" != "PASS" ]]; then
  overall="FAIL"
fi

if "$commands_dir/run_full_replay.sh" > "$logs_dir/full_replay_suite.log" 2>&1; then
  full_replay_status="$(summary_value actual_bag_replay_status "$full_dir/reports/actual_bag_replay_summary.txt")"
else
  full_replay_exit=$?
fi
if [[ "$full_replay_status" != "PASS" ]]; then
  overall="FAIL"
fi
full_event_validation_status="$(run_event_validation full "$full_dir")"
if [[ "$full_event_validation_status" != "PASS" ]]; then
  overall="FAIL"
fi

ros_residual_status="PASS"
ros_residual_report="$reports_dir/ros_residual_processes.txt"
if pgrep -af '(^|/)(roslaunch|rosbag|rostopic|roscore|rosmaster)( |$)' > "$ros_residual_report" 2>/dev/null; then
  ros_residual_status="FAIL"
else
  : > "$ros_residual_report"
fi
if [[ "$ros_residual_status" != "PASS" ]]; then
  overall="FAIL"
fi

failed_records=1
if [[ "$overall" == "PASS" ]]; then
  failed_records=0
fi

{
  echo "actual_bag_test_suite_status=$overall"
  echo "bag_path=$bag_path"
  echo "suite_root=$suite_root"
  echo "smoke_duration_s=$smoke_duration_s"
  echo "full_duration_s=$full_duration_s"
  echo "rate=$rate"
  echo "smoke_replay_dir=$smoke_dir"
  echo "full_replay_dir=$full_dir"
  echo "smoke_replay_status=${smoke_replay_status:-FAIL}"
  echo "full_replay_status=${full_replay_status:-FAIL}"
  echo "smoke_replay_exit=$smoke_replay_exit"
  echo "full_replay_exit=$full_replay_exit"
  echo "smoke_event_validation_status=$smoke_event_validation_status"
  echo "full_event_validation_status=$full_event_validation_status"
  echo "ros_residual_status=$ros_residual_status"
  echo "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"
  echo "bag_sensor_set=LIDAR_IMU_ONLY"
  echo "plc_feedback_status=NOT_PRESENT_NA"
  echo "plc_feedback_gate_status=NA_INITIAL_TEST"
  echo "machine_motion_assumption=CONTINUOUS_MOTION"
  echo "vibration_profile=NORMAL"
  echo "time_reference_status=$time_reference_status"
  echo "time_sync_evidence_status=$time_sync_evidence_status"
  echo "initial_velocity_reference_status=$initial_velocity_reference_status"
  echo "initial_velocity_reference_required=$initial_velocity_reference_required"
  echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
  echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
  echo "velocity_reference_played_to_slam=NO"
  echo "continuous_velocity_reference_used=NO"
  echo "generated_event_validation=YES"
  echo "field_acceptance_requires_plc_feedback=YES"
  echo "field_acceptance_eligible=NO"
  echo "field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
} > "$reports_dir/actual_bag_test_suite_summary.txt"

{
  echo "overall=$overall;total_records=1;failed_records=$failed_records"
  echo "session=$(basename "$suite_root");scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;status=$overall;failed_checks=$failed_records;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"
  echo "---"
  echo "detail=smoke_replay_status;status=${smoke_replay_status:-FAIL};value=${smoke_replay_status:-FAIL};threshold=PASS"
  echo "detail=full_replay_status;status=${full_replay_status:-FAIL};value=${full_replay_status:-FAIL};threshold=PASS"
  echo "detail=smoke_event_validation_status;status=$smoke_event_validation_status;value=$smoke_event_validation_status;threshold=PASS"
  echo "detail=full_event_validation_status;status=$full_event_validation_status;value=$full_event_validation_status;threshold=PASS"
  echo "detail=ros_residual_status;status=$ros_residual_status;value=$ros_residual_status;threshold=PASS"
  echo "detail=field_acceptance_eligible;status=PASS;value=NO;threshold=NO"
} > "$reports_dir/actual_bag_test_suite_metrics_report.txt"

{
  echo "event=session_start;scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;session_id=$(basename "$suite_root");t=0.000;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"
  echo "event=actual_bag_test_suite;scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;session_id=$(basename "$suite_root");t=$full_duration_s;queue_backlog=0;actual_bag_test_suite_status=$overall;smoke_replay_status=${smoke_replay_status:-FAIL};full_replay_status=${full_replay_status:-FAIL};smoke_event_validation_status=$smoke_event_validation_status;full_event_validation_status=$full_event_validation_status;plc_feedback_status=NOT_PRESENT_NA;field_acceptance_eligible=NO;velocity_reference_played_to_slam=NO;continuous_velocity_reference_used=NO"
} > "$reports_dir/actual_bag_test_suite_events.txt"

[[ "$overall" == "PASS" ]]
EOF
} > "$commands_dir/run_suite.sh"
write_executable "$commands_dir/run_suite.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -uo pipefail'
  echo "suite_root=$(shell_quote "$out_abs")"
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "smoke_dir_name=$(shell_quote "$smoke_dir_name")"
  echo "full_dir_name=$(shell_quote "$full_dir_name")"
  echo "manifest_file=$(shell_quote "$reports_dir/actual_bag_test_suite_manifest.txt")"
  echo "validation_report=$(shell_quote "$reports_dir/actual_bag_test_suite_manifest_validation.txt")"
  cat <<'EOF'

value_for_key() {
  local key="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    awk -v target="$key" '
      {
        pos = index($0, "=")
        if (pos > 0 && substr($0, 1, pos - 1) == target) {
          value = substr($0, pos + 1)
          found = 1
        }
      }
      END {
        if (found) {
          print value
        }
      }
    ' "$path"
  fi
}

manifest_value() {
  value_for_key "$1" "$manifest_file"
}

manifest_path() {
  local key="$1"
  local rel
  rel="$(manifest_value "$key")"
  if [[ -z "$rel" ]]; then
    echo ""
  elif [[ "$rel" == /* ]]; then
    echo "$rel"
  else
    echo "$suite_root/$rel"
  fi
}

check_manifest_path() {
  local key="$1"
  local expected="$2"
  local actual
  actual="$(manifest_value "$key")"
  if [[ "$actual" == "$expected" ]]; then
    echo "${key}_path_status=PASS"
    return 0
  fi
  echo "${key}_path_status=FAIL"
  echo "${key}_path_value=${actual:-missing}"
  echo "${key}_path_expected=$expected"
  return 1
}

record_overall() {
  local path="$1"
  if [[ -f "$path" ]]; then
    sed -n 's/^overall=\([^;]*\).*/\1/p' "$path" | tail -n 1
  fi
}

check_key_equals() {
  local label="$1"
  local path="$2"
  local key="$3"
  local expected="$4"
  local actual
  actual="$(value_for_key "$key" "$path")"
  if [[ "$actual" == "$expected" ]]; then
    echo "${label}_${key}_status=PASS"
    return 0
  fi
  echo "${label}_${key}_status=FAIL"
  echo "${label}_${key}_value=${actual:-missing}"
  echo "${label}_${key}_expected=$expected"
  return 1
}

check_key_equals_if_present() {
  local label="$1"
  local path="$2"
  local key="$3"
  local expected="$4"
  local actual
  actual="$(value_for_key "$key" "$path")"
  if [[ -z "$actual" ]]; then
    echo "${label}_${key}_status=NOT_PRESENT_NA"
    return 0
  fi
  if [[ "$actual" == "$expected" ]]; then
    echo "${label}_${key}_status=PASS"
    return 0
  fi
  echo "${label}_${key}_status=FAIL"
  echo "${label}_${key}_value=${actual:-missing}"
  echo "${label}_${key}_expected=$expected"
  return 1
}

check_key_path_equals() {
  local label="$1"
  local path="$2"
  local key="$3"
  local expected="$4"
  local actual
  actual="$(value_for_key "$key" "$path")"
  if [[ "$actual" == "$expected" ]]; then
    echo "${label}_${key}_path_status=PASS"
    return 0
  fi
  echo "${label}_${key}_path_status=FAIL"
  echo "${label}_${key}_path_value=${actual:-missing}"
  echo "${label}_${key}_path_expected=$expected"
  return 1
}

check_file_exists() {
  local label="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    echo "${label}_exists_status=PASS"
    return 0
  fi
  echo "${label}_exists_status=FAIL"
  echo "${label}_path=${path:-missing}"
  return 1
}

check_grep() {
  local label="$1"
  local path="$2"
  local pattern="$3"
  if [[ -f "$path" ]] && grep -F "$pattern" "$path" >/dev/null 2>&1; then
    echo "${label}_status=PASS"
    return 0
  fi
  echo "${label}_status=FAIL"
  echo "${label}_pattern=$pattern"
  return 1
}

check_overall_pass() {
  local label="$1"
  local path="$2"
  local actual
  actual="$(record_overall "$path")"
  if [[ "$actual" == "PASS" ]]; then
    echo "${label}_overall_status=PASS"
    return 0
  fi
  echo "${label}_overall_status=FAIL"
  echo "${label}_overall_value=${actual:-missing}"
  return 1
}

check_time_reference_pair() {
  local label="$1"
  local path="$2"
  local status
  local evidence
  status="$(value_for_key time_reference_status "$path")"
  evidence="$(value_for_key time_sync_evidence_status "$path")"
  if [[ "$status" == "PRESENT_REQUIRED" && "$evidence" == "INITIAL_TIME_STATUS_CAPTURE_REQUIRED" ]]; then
    echo "${label}_time_reference_pair_status=PASS"
    return 0
  fi
  if [[ "$status" == "NOT_PRESENT_INITIAL_TEST" && "$evidence" == "NOT_PRESENT_INITIAL_TEST" ]]; then
    echo "${label}_time_reference_pair_status=PASS"
    return 0
  fi
  echo "${label}_time_reference_pair_status=FAIL"
  echo "${label}_time_reference_status_value=${status:-missing}"
  echo "${label}_time_sync_evidence_status_value=${evidence:-missing}"
  return 1
}

check_initial_velocity_required() {
  local label="$1"
  local path="$2"
  local required
  required="$(value_for_key initial_velocity_reference_required "$path")"
  if [[ -z "$required" || "$required" == "YES" || "$required" == "NO" ]]; then
    echo "${label}_initial_velocity_reference_required_status=PASS"
    return 0
  fi
  echo "${label}_initial_velocity_reference_required_status=FAIL"
  echo "${label}_initial_velocity_reference_required_value=$required"
  return 1
}

overall="PASS"
details="$(mktemp)"
trap 'rm -f "$details"' EXIT

record_check() {
  if "$@" >> "$details"; then
    return 0
  fi
  overall="FAIL"
  return 1
}

record_check check_file_exists manifest "$manifest_file" || true
record_check check_key_equals manifest "$manifest_file" actual_bag_test_suite_manifest_status READY || true
record_check check_key_equals manifest "$manifest_file" actual_bag_test_scope INITIAL_LIDAR_IMU_ONLY || true
record_check check_key_equals manifest "$manifest_file" field_acceptance_eligible NO || true
record_check check_key_equals manifest "$manifest_file" validation_scope ACTUAL_LIDAR_IMU_FRONTEND_ONLY || true
record_check check_time_reference_pair manifest "$manifest_file" || true
record_check check_initial_velocity_required manifest "$manifest_file" || true
record_check check_manifest_path summary reports/actual_bag_test_suite_summary.txt || true
record_check check_manifest_path metrics_report reports/actual_bag_test_suite_metrics_report.txt || true
record_check check_manifest_path event_file reports/actual_bag_test_suite_events.txt || true
record_check check_manifest_path ros_residual_report reports/ros_residual_processes.txt || true
record_check check_manifest_path smoke_summary "$smoke_dir_name/reports/actual_bag_replay_summary.txt" || true
record_check check_manifest_path full_summary "$full_dir_name/reports/actual_bag_replay_summary.txt" || true
record_check check_manifest_path smoke_event_validation "$smoke_dir_name/reports/actual_bag_replay_hil_validation_report.txt" || true
record_check check_manifest_path full_event_validation "$full_dir_name/reports/actual_bag_replay_hil_validation_report.txt" || true
record_check check_manifest_path smoke_initial_velocity "$smoke_dir_name/reports/initial_velocity_reference.txt" || true
record_check check_manifest_path full_initial_velocity "$full_dir_name/reports/initial_velocity_reference.txt" || true
record_check check_manifest_path smoke_inspection "$smoke_dir_name/reports/actual_bag_inspection.txt" || true
record_check check_manifest_path full_inspection "$full_dir_name/reports/actual_bag_inspection.txt" || true
record_check check_manifest_path field_acceptance_gap_report reports/field_acceptance_gap_report.txt || true
record_check check_manifest_path actual_bag_initial_test_readiness reports/actual_bag_initial_test_readiness.txt || true
record_check check_manifest_path field_acceptance_handoff reports/field_acceptance_handoff.txt || true
record_check check_manifest_path field_acceptance_handoff_manifest reports/field_acceptance_handoff_manifest.txt || true
record_check check_manifest_path field_acceptance_collection_plan reports/field_acceptance_collection_plan.txt || true
record_check check_manifest_path field_acceptance_collection_plan_validation reports/field_acceptance_collection_plan_validation.txt || true
expected_time_reference_status="$(value_for_key time_reference_status "$manifest_file")"
expected_time_sync_evidence_status="$(value_for_key time_sync_evidence_status "$manifest_file")"
expected_initial_velocity_required="$(value_for_key initial_velocity_reference_required "$manifest_file")"
if [[ -z "$expected_initial_velocity_required" ]]; then
  expected_initial_velocity_required="YES"
fi
expected_initial_velocity_topic="$(value_for_key initial_velocity_reference_topic "$manifest_file")"
if [[ -z "$expected_initial_velocity_topic" ]]; then
  expected_initial_velocity_topic="missing"
fi
expected_initial_velocity_policy="$(value_for_key initial_velocity_reference_policy "$manifest_file")"
if [[ -z "$expected_initial_velocity_policy" ]]; then
  expected_initial_velocity_policy="START_ONLY_AUDIT"
fi

summary_file="$(manifest_path summary)"
metrics_file="$(manifest_path metrics_report)"
event_file="$(manifest_path event_file)"
smoke_summary_file="$(manifest_path smoke_summary)"
full_summary_file="$(manifest_path full_summary)"
smoke_event_validation_file="$(manifest_path smoke_event_validation)"
full_event_validation_file="$(manifest_path full_event_validation)"
smoke_initial_velocity_file="$(manifest_path smoke_initial_velocity)"
full_initial_velocity_file="$(manifest_path full_initial_velocity)"
smoke_inspection_file="$(manifest_path smoke_inspection)"
full_inspection_file="$(manifest_path full_inspection)"
ros_residual_file="$(manifest_path ros_residual_report)"
field_acceptance_collection_plan_file="$(manifest_path field_acceptance_collection_plan)"
field_acceptance_collection_plan_validation_file="$(manifest_path field_acceptance_collection_plan_validation)"

for item in \
  summary_file metrics_file event_file smoke_summary_file full_summary_file \
  smoke_event_validation_file full_event_validation_file \
  smoke_initial_velocity_file full_initial_velocity_file \
  smoke_inspection_file full_inspection_file ros_residual_file; do
  value="${!item}"
  record_check check_file_exists "$item" "$value" || true
done

record_check check_key_equals suite_summary "$summary_file" actual_bag_test_suite_status PASS || true
record_check check_key_equals suite_summary "$summary_file" smoke_replay_status PASS || true
record_check check_key_equals suite_summary "$summary_file" full_replay_status PASS || true
record_check check_key_equals suite_summary "$summary_file" smoke_event_validation_status PASS || true
record_check check_key_equals suite_summary "$summary_file" full_event_validation_status PASS || true
record_check check_key_equals suite_summary "$summary_file" ros_residual_status PASS || true
record_check check_key_equals suite_summary "$summary_file" actual_bag_test_scope INITIAL_LIDAR_IMU_ONLY || true
record_check check_key_equals suite_summary "$summary_file" bag_sensor_set LIDAR_IMU_ONLY || true
record_check check_key_equals suite_summary "$summary_file" plc_feedback_status NOT_PRESENT_NA || true
record_check check_key_equals suite_summary "$summary_file" time_reference_status "$expected_time_reference_status" || true
record_check check_key_equals suite_summary "$summary_file" time_sync_evidence_status "$expected_time_sync_evidence_status" || true
record_check check_key_equals_if_present suite_summary "$summary_file" initial_velocity_reference_required "$expected_initial_velocity_required" || true
record_check check_key_equals_if_present suite_summary "$summary_file" initial_velocity_reference_topic "$expected_initial_velocity_topic" || true
record_check check_key_equals_if_present suite_summary "$summary_file" initial_velocity_reference_policy "$expected_initial_velocity_policy" || true
record_check check_key_equals suite_summary "$summary_file" velocity_reference_played_to_slam NO || true
record_check check_key_equals suite_summary "$summary_file" continuous_velocity_reference_used NO || true
record_check check_key_equals suite_summary "$summary_file" field_acceptance_eligible NO || true
record_check check_key_equals_if_present suite_summary "$summary_file" verified_suite_status PASS || true
record_check check_key_equals_if_present suite_summary "$summary_file" suite_manifest_validation_after_execute PASS || true
record_check check_key_equals_if_present suite_summary "$summary_file" field_acceptance_gap_audit_exit 1 || true
record_check check_key_equals_if_present suite_summary "$summary_file" field_acceptance_gap_validation_after_execute PASS || true
record_check check_key_equals_if_present suite_summary "$summary_file" actual_bag_initial_test_readiness_after_execute PASS || true
record_check check_key_equals_if_present suite_summary "$summary_file" field_acceptance_handoff_after_execute PASS || true
record_check check_key_equals_if_present suite_summary "$summary_file" field_acceptance_handoff_manifest_after_execute PASS || true
record_check check_key_equals_if_present suite_summary "$summary_file" field_acceptance_collection_plan_after_execute PASS || true
collection_plan_after_execute="$(value_for_key field_acceptance_collection_plan_after_execute "$summary_file")"
if [[ "$collection_plan_after_execute" == "PASS" || -f "$field_acceptance_collection_plan_validation_file" ]]; then
  record_check check_file_exists field_acceptance_collection_plan "$field_acceptance_collection_plan_file" || true
  record_check check_file_exists field_acceptance_collection_plan_validation "$field_acceptance_collection_plan_validation_file" || true
  record_check check_key_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" field_acceptance_collection_plan_status PASS || true
  record_check check_key_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" collection_plan_ready YES || true
  record_check check_key_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" field_acceptance_eligible NO || true
  record_check check_key_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" field_acceptance_status NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY || true
  record_check check_key_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" field_acceptance_handoff_manifest_validation_status PASS || true
  record_check check_key_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" final_success_gate "record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS" || true
  record_check check_key_path_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" source_field_acceptance_handoff "$reports_dir/field_acceptance_handoff.txt" || true
  record_check check_key_path_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" source_field_acceptance_handoff_validation "$reports_dir/field_acceptance_handoff_validation.txt" || true
  record_check check_key_path_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" source_field_acceptance_handoff_manifest "$reports_dir/field_acceptance_handoff_manifest.txt" || true
  record_check check_key_path_equals field_acceptance_collection_plan "$field_acceptance_collection_plan_file" source_field_acceptance_handoff_manifest_validation "$reports_dir/field_acceptance_handoff_manifest_validation.txt" || true
  record_check check_key_equals field_acceptance_collection_plan_validation "$field_acceptance_collection_plan_validation_file" field_acceptance_collection_plan_validation_status PASS || true
  record_check check_key_equals field_acceptance_collection_plan_validation "$field_acceptance_collection_plan_validation_file" final_success_gate_status PASS || true
  record_check check_key_equals field_acceptance_collection_plan_validation "$field_acceptance_collection_plan_validation_file" source_field_acceptance_handoff_path_status PASS || true
  record_check check_key_equals field_acceptance_collection_plan_validation "$field_acceptance_collection_plan_validation_file" source_field_acceptance_handoff_validation_path_status PASS || true
  record_check check_key_equals field_acceptance_collection_plan_validation "$field_acceptance_collection_plan_validation_file" source_field_acceptance_handoff_manifest_path_status PASS || true
  record_check check_key_equals field_acceptance_collection_plan_validation "$field_acceptance_collection_plan_validation_file" source_field_acceptance_handoff_manifest_validation_path_status PASS || true
fi
record_check check_overall_pass suite_metrics "$metrics_file" || true
record_check check_grep suite_event_status "$event_file" "actual_bag_test_suite_status=PASS" || true
record_check check_grep suite_event_field_acceptance "$event_file" "field_acceptance_eligible=NO" || true
record_check check_grep suite_event_velocity "$event_file" "velocity_reference_played_to_slam=NO" || true
record_check check_grep suite_event_continuous_velocity "$event_file" "continuous_velocity_reference_used=NO" || true

for label in smoke full; do
  summary_var="${label}_summary_file"
  validation_var="${label}_event_validation_file"
  velocity_var="${label}_initial_velocity_file"
  inspection_var="${label}_inspection_file"
  replay_summary="${!summary_var}"
  validation="${!validation_var}"
  velocity="${!velocity_var}"
  inspection="${!inspection_var}"

  record_check check_key_equals "${label}_replay" "$replay_summary" actual_bag_replay_status PASS || true
  record_check check_key_equals "${label}_replay" "$replay_summary" actual_bag_test_scope INITIAL_LIDAR_IMU_ONLY || true
  record_check check_key_equals "${label}_replay" "$replay_summary" bag_sensor_set LIDAR_IMU_ONLY || true
  record_check check_key_equals "${label}_replay" "$replay_summary" plc_feedback_status NOT_PRESENT_NA || true
  record_check check_key_equals "${label}_replay" "$replay_summary" time_reference_status "$expected_time_reference_status" || true
  record_check check_key_equals "${label}_replay" "$replay_summary" time_sync_evidence_status "$expected_time_sync_evidence_status" || true
  record_check check_key_equals "${label}_replay" "$replay_summary" velocity_reference_played_to_slam NO || true
  record_check check_key_equals "${label}_replay" "$replay_summary" continuous_velocity_reference_used NO || true
  record_check check_overall_pass "${label}_event_validation" "$validation" || true
  if [[ "$expected_initial_velocity_required" == "NO" ]]; then
    record_check check_key_equals "${label}_replay" "$replay_summary" initial_velocity_reference_required NO || true
    record_check check_key_equals "${label}_replay" "$replay_summary" initial_velocity_reference_topic NONE || true
    record_check check_key_equals "${label}_replay" "$replay_summary" initial_velocity_reference_policy NOT_AVAILABLE_INITIAL_TEST || true
    record_check check_key_equals "${label}_initial_velocity" "$velocity" initial_velocity_reference_status NOT_PRESENT_INITIAL_TEST || true
    record_check check_key_equals "${label}_initial_velocity" "$velocity" initial_velocity_reference_required NO || true
    record_check check_key_equals "${label}_initial_velocity" "$velocity" initial_velocity_reference_topic NONE || true
    record_check check_key_equals "${label}_initial_velocity" "$velocity" initial_velocity_reference_policy NOT_AVAILABLE_INITIAL_TEST || true
  else
    record_check check_key_equals "${label}_initial_velocity" "$velocity" initial_velocity_reference_status CAPTURED || true
    record_check check_key_equals "${label}_initial_velocity" "$velocity" initial_velocity_reference_policy START_ONLY_AUDIT || true
  fi
  record_check check_key_equals "${label}_initial_velocity" "$velocity" velocity_reference_played_to_slam NO || true
  record_check check_key_equals "${label}_initial_velocity" "$velocity" continuous_velocity_reference_used NO || true
  record_check check_key_equals "${label}_inspection" "$inspection" actual_bag_status PASS || true
  record_check check_key_equals "${label}_inspection" "$inspection" actual_bag_test_scope INITIAL_LIDAR_IMU_ONLY || true
  record_check check_key_equals "${label}_inspection" "$inspection" bag_sensor_set LIDAR_IMU_ONLY || true
  record_check check_key_equals "${label}_inspection" "$inspection" plc_feedback_status NOT_PRESENT_NA || true
  record_check check_key_equals "${label}_inspection" "$inspection" time_reference_status "$expected_time_reference_status" || true
  record_check check_key_equals "${label}_inspection" "$inspection" time_sync_evidence_status "$expected_time_sync_evidence_status" || true
  if [[ "$expected_initial_velocity_required" == "NO" ]]; then
    record_check check_key_equals "${label}_inspection" "$inspection" initial_velocity_reference_required NO || true
    record_check check_key_equals "${label}_inspection" "$inspection" initial_velocity_reference_topic NONE || true
    record_check check_key_equals "${label}_inspection" "$inspection" initial_velocity_reference_policy NOT_AVAILABLE_INITIAL_TEST || true
  fi
  if [[ "$expected_time_reference_status" == "NOT_PRESENT_INITIAL_TEST" ]]; then
    record_check check_key_equals "${label}_replay" "$replay_summary" time_reference_topic NONE || true
    record_check check_key_equals "${label}_inspection" "$inspection" time_reference_topic NONE || true
  fi
  record_check check_key_equals "${label}_inspection" "$inspection" "topic_count[/plc/left_track_speed]" 0 || true
  record_check check_key_equals "${label}_inspection" "$inspection" "topic_count[/plc/right_track_speed]" 0 || true
  record_check check_key_equals "${label}_inspection" "$inspection" "topic_count[/plc/cutting_on]" 0 || true
  record_check check_key_equals "${label}_inspection" "$inspection" "topic_count[/machine/state]" 0 || true
done

{
  echo "actual_bag_test_suite_manifest_validation_status=$overall"
  echo "manifest_file=$manifest_file"
  echo "suite_root=$suite_root"
  echo "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"
  echo "validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
  echo "field_acceptance_eligible=NO"
  echo "time_reference_status=${expected_time_reference_status:-missing}"
  echo "time_sync_evidence_status=${expected_time_sync_evidence_status:-missing}"
  echo "velocity_reference_played_to_slam=NO"
  echo "continuous_velocity_reference_used=NO"
  echo "---"
  cat "$details"
} > "$validation_report"

[[ "$overall" == "PASS" ]]
EOF
} > "$commands_dir/validate_actual_bag_test_suite.sh"
write_executable "$commands_dir/validate_actual_bag_test_suite.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -euo pipefail'
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "gap_report=$(shell_quote "$reports_dir/field_acceptance_gap_report.txt")"
  echo "validation_report=$(shell_quote "$reports_dir/field_acceptance_gap_validation.txt")"
  echo "plc_feedback_collection_command=$(shell_quote "$plc_feedback_collection_command")"
  echo "section_export_collection_command=$(shell_quote "$section_export_collection_command")"
  echo "pps_ptp_wiring_collection_command=$(shell_quote "$pps_ptp_wiring_collection_command")"
  echo "power_loss_resume_collection_command=$(shell_quote "$power_loss_resume_collection_command")"
  echo "runtime_deployment_collection_command=$(shell_quote "$runtime_deployment_collection_command")"
  echo "runtime_stability_24h_collection_command=$(shell_quote "$runtime_stability_24h_collection_command")"
  echo "field_acceptance_collection_command=$(shell_quote "$field_acceptance_collection_command")"
  cat <<'EOF'

declare -A values
declare -A statuses
overall="PASS"
duplicate_keys=0
malformed_lines=0

mark_fail() {
  local key="$1"
  statuses["$key"]="FAIL"
  overall="FAIL"
}

mark_pass() {
  local key="$1"
  if [[ "${statuses[$key]:-}" != "FAIL" ]]; then
    statuses["$key"]="PASS"
  fi
}

check_equals() {
  local key="$1"
  local expected="$2"
  if [[ "${values[$key]+set}" != "set" || "${values[$key]}" != "$expected" ]]; then
    mark_fail "$key"
  else
    mark_pass "$key"
  fi
}

if [[ ! -f "$gap_report" ]]; then
  mkdir -p "$reports_dir"
  {
    echo "field_acceptance_gap_validation_status=FAIL"
    echo "field_acceptance_gap_report=$gap_report"
    echo "field_acceptance_gap_report_status=MISSING"
  } > "$validation_report"
  exit 1
fi

while IFS= read -r line || [[ -n "$line" ]]; do
  [[ -z "$line" ]] && continue
  if [[ "$line" != *=* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  key="${line%%=*}"
  value="${line#*=}"
  if [[ -z "$key" || "$key" == *[[:space:]]* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  if [[ "${values[$key]+set}" == "set" ]]; then
    duplicate_keys=$((duplicate_keys + 1))
    overall="FAIL"
    continue
  fi
  values["$key"]="$value"
done < "$gap_report"

check_equals "field_acceptance_gap_audit_status" "FAIL"
check_equals "actual_bag_initial_evidence_status" "PASS"
check_equals "actual_bag_test_scope" "INITIAL_LIDAR_IMU_ONLY"
check_equals "validation_scope" "ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
check_equals "field_acceptance_ready" "NO"
check_equals "field_acceptance_eligible" "NO"
check_equals "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_equals "plc_feedback_evidence_status" "MISSING"
check_equals "section_export_evidence_status" "MISSING"
check_equals "pps_ptp_wiring_evidence_status" "MISSING"
check_equals "power_loss_resume_evidence_status" "MISSING"
check_equals "runtime_deployment_evidence_status" "MISSING"
check_equals "runtime_stability_24h_evidence_status" "MISSING"
check_equals "field_acceptance_report_status" "MISSING"
check_equals "required_next_evidence" "PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
check_equals "plc_feedback_collection_command" "$plc_feedback_collection_command"
check_equals "section_export_collection_command" "$section_export_collection_command"
check_equals "pps_ptp_wiring_collection_command" "$pps_ptp_wiring_collection_command"
check_equals "power_loss_resume_collection_command" "$power_loss_resume_collection_command"
check_equals "runtime_deployment_collection_command" "$runtime_deployment_collection_command"
check_equals "runtime_stability_24h_collection_command" "$runtime_stability_24h_collection_command"
check_equals "field_acceptance_collection_command" "$field_acceptance_collection_command"

{
  echo "field_acceptance_gap_validation_status=$overall"
  echo "field_acceptance_gap_report=$gap_report"
  echo "duplicate_key_count=$duplicate_keys"
  echo "malformed_line_count=$malformed_lines"
  for key in "${!statuses[@]}"; do
    echo "${key}_status=${statuses[$key]}"
  done | sort
  echo "field_acceptance_ready=${values[field_acceptance_ready]:-missing}"
  echo "field_acceptance_eligible=${values[field_acceptance_eligible]:-missing}"
  echo "field_acceptance_status=${values[field_acceptance_status]:-missing}"
  echo "required_next_evidence=${values[required_next_evidence]:-missing}"
  echo "plc_feedback_collection_command=${values[plc_feedback_collection_command]:-missing}"
  echo "section_export_collection_command=${values[section_export_collection_command]:-missing}"
  echo "pps_ptp_wiring_collection_command=${values[pps_ptp_wiring_collection_command]:-missing}"
  echo "power_loss_resume_collection_command=${values[power_loss_resume_collection_command]:-missing}"
  echo "runtime_deployment_collection_command=${values[runtime_deployment_collection_command]:-missing}"
  echo "runtime_stability_24h_collection_command=${values[runtime_stability_24h_collection_command]:-missing}"
  echo "field_acceptance_collection_command=${values[field_acceptance_collection_command]:-missing}"
} > "$validation_report"

[[ "$overall" == "PASS" ]]
EOF
} > "$commands_dir/validate_field_acceptance_gap.sh"
write_executable "$commands_dir/validate_field_acceptance_gap.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -uo pipefail'
  echo "suite_root=$(shell_quote "$out_abs")"
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "logs_dir=$(shell_quote "$logs_dir")"
  echo "validation_command=$(shell_quote "$commands_dir/validate_actual_bag_test_suite.sh")"
  echo "manifest_validation_report=$(shell_quote "$reports_dir/actual_bag_test_suite_manifest_validation.txt")"
  echo "gap_report=$(shell_quote "$reports_dir/field_acceptance_gap_report.txt")"
  echo "plc_feedback_collection_command=$(shell_quote "$plc_feedback_collection_command")"
  echo "section_export_collection_command=$(shell_quote "$section_export_collection_command")"
  echo "pps_ptp_wiring_collection_command=$(shell_quote "$pps_ptp_wiring_collection_command")"
  echo "power_loss_resume_collection_command=$(shell_quote "$power_loss_resume_collection_command")"
  echo "runtime_deployment_collection_command=$(shell_quote "$runtime_deployment_collection_command")"
  echo "runtime_stability_24h_collection_command=$(shell_quote "$runtime_stability_24h_collection_command")"
  echo "field_acceptance_collection_command=$(shell_quote "$field_acceptance_collection_command")"
  cat <<'EOF'

value_for_key() {
  local key="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    awk -v target="$key" '
      {
        pos = index($0, "=")
        if (pos > 0 && substr($0, 1, pos - 1) == target) {
          value = substr($0, pos + 1)
          found = 1
        }
      }
      END {
        if (found) {
          print value
        }
      }
    ' "$path"
  fi
}

mkdir -p "$reports_dir" "$logs_dir"

validation_exit=0
if "$validation_command" > "$logs_dir/field_acceptance_gap_manifest_validation.log" 2>&1; then
  validation_exit=0
else
  validation_exit=$?
fi

manifest_validation_status="$(value_for_key actual_bag_test_suite_manifest_validation_status "$manifest_validation_report")"
actual_bag_initial_evidence_status="FAIL"
if [[ "$validation_exit" -eq 0 && "$manifest_validation_status" == "PASS" ]]; then
  actual_bag_initial_evidence_status="PASS"
fi

{
  echo "field_acceptance_gap_audit_status=FAIL"
  echo "actual_bag_initial_evidence_status=$actual_bag_initial_evidence_status"
  echo "actual_bag_test_suite_manifest_validation_status=${manifest_validation_status:-missing}"
  echo "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"
  echo "validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
  echo "field_acceptance_ready=NO"
  echo "field_acceptance_eligible=NO"
  echo "field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
  echo "plc_feedback_evidence_status=MISSING"
  echo "plc_feedback_required_status=MISSING"
  echo "section_export_evidence_status=MISSING"
  echo "pps_ptp_wiring_evidence_status=MISSING"
  echo "power_loss_resume_evidence_status=MISSING"
  echo "runtime_deployment_evidence_status=MISSING"
  echo "runtime_stability_24h_evidence_status=MISSING"
  echo "field_acceptance_report_status=MISSING"
  echo "required_next_evidence=PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
  echo "plc_feedback_collection_command=$plc_feedback_collection_command"
  echo "section_export_collection_command=$section_export_collection_command"
  echo "pps_ptp_wiring_collection_command=$pps_ptp_wiring_collection_command"
  echo "power_loss_resume_collection_command=$power_loss_resume_collection_command"
  echo "runtime_deployment_collection_command=$runtime_deployment_collection_command"
  echo "runtime_stability_24h_collection_command=$runtime_stability_24h_collection_command"
  echo "field_acceptance_collection_command=$field_acceptance_collection_command"
  echo "final_gate_command=record_session.sh generated commands/validate_evidence.sh"
} > "$gap_report"

exit 1
EOF
} > "$commands_dir/audit_field_acceptance_gap.sh"
write_executable "$commands_dir/audit_field_acceptance_gap.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -uo pipefail'
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "logs_dir=$(shell_quote "$logs_dir")"
  echo "summary_report=$(shell_quote "$reports_dir/actual_bag_test_suite_summary.txt")"
  echo "manifest_validation_report=$(shell_quote "$reports_dir/actual_bag_test_suite_manifest_validation.txt")"
  echo "gap_report=$(shell_quote "$reports_dir/field_acceptance_gap_report.txt")"
  echo "gap_validation_report=$(shell_quote "$reports_dir/field_acceptance_gap_validation.txt")"
  echo "readiness_report=$(shell_quote "$reports_dir/actual_bag_initial_test_readiness.txt")"
  echo "manifest_validation_command=$(shell_quote "$commands_dir/validate_actual_bag_test_suite.sh")"
  echo "gap_validation_command=$(shell_quote "$commands_dir/validate_field_acceptance_gap.sh")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  cat <<'EOF'

value_for_key() {
  local key="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    awk -v target="$key" '
      {
        pos = index($0, "=")
        if (pos > 0 && substr($0, 1, pos - 1) == target) {
          value = substr($0, pos + 1)
          found = 1
        }
      }
      END {
        if (found) {
          print value
        }
      }
    ' "$path"
  fi
}

mkdir -p "$reports_dir" "$logs_dir"

manifest_validation_exit=0
if "$manifest_validation_command" > "$logs_dir/initial_test_readiness_manifest_validation.log" 2>&1; then
  manifest_validation_exit=0
else
  manifest_validation_exit=$?
fi

gap_validation_exit=0
if "$gap_validation_command" > "$logs_dir/initial_test_readiness_gap_validation.log" 2>&1; then
  gap_validation_exit=0
else
  gap_validation_exit=$?
fi

suite_status="$(value_for_key actual_bag_test_suite_status "$summary_report")"
scope="$(value_for_key actual_bag_test_scope "$summary_report")"
sensor_set="$(value_for_key bag_sensor_set "$summary_report")"
plc_feedback_status="$(value_for_key plc_feedback_status "$summary_report")"
plc_feedback_gate_status="$(value_for_key plc_feedback_gate_status "$summary_report")"
machine_motion_assumption="$(value_for_key machine_motion_assumption "$summary_report")"
vibration_profile="$(value_for_key vibration_profile "$summary_report")"
time_reference_status="$(value_for_key time_reference_status "$summary_report")"
time_sync_evidence_status="$(value_for_key time_sync_evidence_status "$summary_report")"
initial_velocity_reference_required="$(value_for_key initial_velocity_reference_required "$summary_report")"
initial_velocity_reference_topic="$(value_for_key initial_velocity_reference_topic "$summary_report")"
initial_velocity_reference_policy="$(value_for_key initial_velocity_reference_policy "$summary_report")"
velocity_reference_played_to_slam="$(value_for_key velocity_reference_played_to_slam "$summary_report")"
continuous_velocity_reference_used="$(value_for_key continuous_velocity_reference_used "$summary_report")"
field_acceptance_eligible="$(value_for_key field_acceptance_eligible "$summary_report")"
field_acceptance_status="$(value_for_key field_acceptance_status "$summary_report")"

manifest_validation_status="$(value_for_key actual_bag_test_suite_manifest_validation_status "$manifest_validation_report")"
smoke_initial_velocity_capture_status="$(value_for_key smoke_initial_velocity_initial_velocity_reference_status_status "$manifest_validation_report")"
full_initial_velocity_capture_status="$(value_for_key full_initial_velocity_initial_velocity_reference_status_status "$manifest_validation_report")"
gap_validation_status="$(value_for_key field_acceptance_gap_validation_status "$gap_validation_report")"
actual_bag_initial_evidence_status="$(value_for_key actual_bag_initial_evidence_status "$gap_report")"
required_next_evidence="$(value_for_key required_next_evidence "$gap_report")"

readiness_status="FAIL"
user_bag_test_ready="NO"
if [[ "$suite_status" == "PASS" \
      && "$manifest_validation_exit" -eq 0 \
      && "$manifest_validation_status" == "PASS" \
      && "$gap_validation_exit" -eq 0 \
      && "$gap_validation_status" == "PASS" \
      && "$actual_bag_initial_evidence_status" == "PASS" \
      && "$scope" == "INITIAL_LIDAR_IMU_ONLY" \
      && "$sensor_set" == "LIDAR_IMU_ONLY" \
      && "$plc_feedback_status" == "NOT_PRESENT_NA" \
      && "$plc_feedback_gate_status" == "NA_INITIAL_TEST" \
      && "$machine_motion_assumption" == "CONTINUOUS_MOTION" \
      && "$vibration_profile" == "NORMAL" \
      && "$smoke_initial_velocity_capture_status" == "PASS" \
      && "$full_initial_velocity_capture_status" == "PASS" \
      && "$velocity_reference_played_to_slam" == "NO" \
      && "$continuous_velocity_reference_used" == "NO" \
      && "$field_acceptance_eligible" == "NO" \
      && "$field_acceptance_status" == "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY" ]]; then
  readiness_status="PASS"
  user_bag_test_ready="YES"
fi

{
  echo "actual_bag_initial_test_readiness_status=$readiness_status"
  echo "actual_bag_user_bag_test_ready=$user_bag_test_ready"
  echo "actual_bag_initial_evidence_status=${actual_bag_initial_evidence_status:-missing}"
  echo "actual_bag_test_suite_status=${suite_status:-missing}"
  echo "actual_bag_test_suite_manifest_validation_status=${manifest_validation_status:-missing}"
  echo "field_acceptance_gap_validation_status=${gap_validation_status:-missing}"
  echo "actual_bag_test_scope=${scope:-missing}"
  echo "validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
  echo "bag_sensor_set=${sensor_set:-missing}"
  echo "plc_feedback_status=${plc_feedback_status:-missing}"
  echo "plc_feedback_gate_status=${plc_feedback_gate_status:-missing}"
  echo "machine_motion_assumption=${machine_motion_assumption:-missing}"
  echo "vibration_profile=${vibration_profile:-missing}"
  echo "time_reference_status=${time_reference_status:-missing}"
  echo "time_sync_evidence_status=${time_sync_evidence_status:-missing}"
  echo "initial_velocity_reference_required=${initial_velocity_reference_required:-missing}"
  echo "initial_velocity_reference_topic=${initial_velocity_reference_topic:-missing}"
  echo "initial_velocity_reference_policy=${initial_velocity_reference_policy:-missing}"
  echo "smoke_initial_velocity_capture_status=${smoke_initial_velocity_capture_status:-missing}"
  echo "full_initial_velocity_capture_status=${full_initial_velocity_capture_status:-missing}"
  echo "velocity_reference_played_to_slam=${velocity_reference_played_to_slam:-missing}"
  echo "continuous_velocity_reference_used=${continuous_velocity_reference_used:-missing}"
  echo "field_acceptance_eligible=${field_acceptance_eligible:-missing}"
  echo "field_acceptance_status=${field_acceptance_status:-missing}"
  echo "required_next_evidence=${required_next_evidence:-missing}"
  echo "bag_path=$bag_path"
  echo "actual_bag_test_suite_summary=$summary_report"
  echo "actual_bag_test_suite_manifest_validation_report=$manifest_validation_report"
  echo "field_acceptance_gap_report=$gap_report"
  echo "field_acceptance_gap_validation_report=$gap_validation_report"
  echo "recommended_user_bag_entry=actual_bag_profile.sh --bag $bag_path"
} > "$readiness_report"

[[ "$readiness_status" == "PASS" ]]
EOF
} > "$commands_dir/audit_actual_bag_initial_test_readiness.sh"
write_executable "$commands_dir/audit_actual_bag_initial_test_readiness.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -euo pipefail'
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "readiness_report=$(shell_quote "$reports_dir/actual_bag_initial_test_readiness.txt")"
  echo "validation_report=$(shell_quote "$reports_dir/actual_bag_initial_test_readiness_validation.txt")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  echo "expected_time_reference_status=$(shell_quote "$time_reference_status")"
  echo "expected_time_sync_evidence_status=$(shell_quote "$time_sync_evidence_status")"
  echo "expected_initial_velocity_reference_required=$(shell_quote "$initial_velocity_reference_required")"
  echo "expected_initial_velocity_reference_topic=$(shell_quote "$reported_initial_velocity_topic")"
  echo "expected_initial_velocity_reference_policy=$(shell_quote "$initial_velocity_reference_policy")"
  cat <<'EOF'

declare -A values
declare -A statuses
overall="PASS"
duplicate_keys=0
malformed_lines=0

mark_fail() {
  local key="$1"
  statuses["$key"]="FAIL"
  overall="FAIL"
}

mark_pass() {
  local key="$1"
  if [[ "${statuses[$key]:-}" != "FAIL" ]]; then
    statuses["$key"]="PASS"
  fi
}

check_equals() {
  local key="$1"
  local expected="$2"
  if [[ "${values[$key]+set}" != "set" || "${values[$key]}" != "$expected" ]]; then
    mark_fail "$key"
  else
    mark_pass "$key"
  fi
}

if [[ ! -f "$readiness_report" ]]; then
  mkdir -p "$reports_dir"
  {
    echo "actual_bag_initial_test_readiness_validation_status=FAIL"
    echo "actual_bag_initial_test_readiness_report=$readiness_report"
    echo "actual_bag_initial_test_readiness_report_status=MISSING"
  } > "$validation_report"
  exit 1
fi

while IFS= read -r line || [[ -n "$line" ]]; do
  [[ -z "$line" ]] && continue
  if [[ "$line" != *=* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  key="${line%%=*}"
  value="${line#*=}"
  if [[ -z "$key" || "$key" == *[[:space:]]* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  if [[ "${values[$key]+set}" == "set" ]]; then
    duplicate_keys=$((duplicate_keys + 1))
    overall="FAIL"
    continue
  fi
  values["$key"]="$value"
done < "$readiness_report"

check_equals "actual_bag_initial_test_readiness_status" "PASS"
check_equals "actual_bag_user_bag_test_ready" "YES"
check_equals "actual_bag_initial_evidence_status" "PASS"
check_equals "actual_bag_test_suite_status" "PASS"
check_equals "actual_bag_test_suite_manifest_validation_status" "PASS"
check_equals "field_acceptance_gap_validation_status" "PASS"
check_equals "actual_bag_test_scope" "INITIAL_LIDAR_IMU_ONLY"
check_equals "validation_scope" "ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
check_equals "bag_sensor_set" "LIDAR_IMU_ONLY"
check_equals "plc_feedback_status" "NOT_PRESENT_NA"
check_equals "plc_feedback_gate_status" "NA_INITIAL_TEST"
check_equals "machine_motion_assumption" "CONTINUOUS_MOTION"
check_equals "vibration_profile" "NORMAL"
check_equals "time_reference_status" "$expected_time_reference_status"
check_equals "time_sync_evidence_status" "$expected_time_sync_evidence_status"
check_equals "initial_velocity_reference_required" "$expected_initial_velocity_reference_required"
check_equals "initial_velocity_reference_topic" "$expected_initial_velocity_reference_topic"
check_equals "initial_velocity_reference_policy" "$expected_initial_velocity_reference_policy"
check_equals "smoke_initial_velocity_capture_status" "PASS"
check_equals "full_initial_velocity_capture_status" "PASS"
check_equals "velocity_reference_played_to_slam" "NO"
check_equals "continuous_velocity_reference_used" "NO"
check_equals "field_acceptance_eligible" "NO"
check_equals "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_equals "required_next_evidence" "PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
check_equals "bag_path" "$bag_path"

{
  echo "actual_bag_initial_test_readiness_validation_status=$overall"
  echo "actual_bag_initial_test_readiness_report=$readiness_report"
  echo "duplicate_key_count=$duplicate_keys"
  echo "malformed_line_count=$malformed_lines"
  for key in "${!statuses[@]}"; do
    echo "${key}_status=${statuses[$key]}"
  done | sort
  echo "actual_bag_user_bag_test_ready=${values[actual_bag_user_bag_test_ready]:-missing}"
  echo "actual_bag_initial_test_readiness_status=${values[actual_bag_initial_test_readiness_status]:-missing}"
  echo "field_acceptance_eligible=${values[field_acceptance_eligible]:-missing}"
  echo "field_acceptance_status=${values[field_acceptance_status]:-missing}"
  echo "required_next_evidence=${values[required_next_evidence]:-missing}"
} > "$validation_report"

[[ "$overall" == "PASS" ]]
EOF
} > "$commands_dir/validate_actual_bag_initial_test_readiness.sh"
write_executable "$commands_dir/validate_actual_bag_initial_test_readiness.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -uo pipefail'
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "logs_dir=$(shell_quote "$logs_dir")"
  echo "readiness_report=$(shell_quote "$reports_dir/actual_bag_initial_test_readiness.txt")"
  echo "readiness_validation_report=$(shell_quote "$reports_dir/actual_bag_initial_test_readiness_validation.txt")"
  echo "gap_report=$(shell_quote "$reports_dir/field_acceptance_gap_report.txt")"
  echo "gap_validation_report=$(shell_quote "$reports_dir/field_acceptance_gap_validation.txt")"
  echo "handoff_report=$(shell_quote "$reports_dir/field_acceptance_handoff.txt")"
  echo "readiness_validation_command=$(shell_quote "$commands_dir/validate_actual_bag_initial_test_readiness.sh")"
  echo "gap_validation_command=$(shell_quote "$commands_dir/validate_field_acceptance_gap.sh")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  cat <<'EOF'

value_for_key() {
  local key="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    awk -v target="$key" '
      {
        pos = index($0, "=")
        if (pos > 0 && substr($0, 1, pos - 1) == target) {
          value = substr($0, pos + 1)
          found = 1
        }
      }
      END {
        if (found) {
          print value
        }
      }
    ' "$path"
  fi
}

mkdir -p "$reports_dir" "$logs_dir"

readiness_validation_exit=0
if "$readiness_validation_command" > "$logs_dir/field_acceptance_handoff_readiness_validation.log" 2>&1; then
  readiness_validation_exit=0
else
  readiness_validation_exit=$?
fi

gap_validation_exit=0
if "$gap_validation_command" > "$logs_dir/field_acceptance_handoff_gap_validation.log" 2>&1; then
  gap_validation_exit=0
else
  gap_validation_exit=$?
fi

readiness_validation_status="$(value_for_key actual_bag_initial_test_readiness_validation_status "$readiness_validation_report")"
readiness_status="$(value_for_key actual_bag_initial_test_readiness_status "$readiness_report")"
user_bag_test_ready="$(value_for_key actual_bag_user_bag_test_ready "$readiness_report")"
gap_validation_status="$(value_for_key field_acceptance_gap_validation_status "$gap_validation_report")"
field_acceptance_ready="$(value_for_key field_acceptance_ready "$gap_report")"
field_acceptance_eligible="$(value_for_key field_acceptance_eligible "$gap_report")"
field_acceptance_status="$(value_for_key field_acceptance_status "$gap_report")"
required_next_evidence="$(value_for_key required_next_evidence "$gap_report")"

plc_feedback_collection_command="$(value_for_key plc_feedback_collection_command "$gap_report")"
section_export_collection_command="$(value_for_key section_export_collection_command "$gap_report")"
pps_ptp_wiring_collection_command="$(value_for_key pps_ptp_wiring_collection_command "$gap_report")"
power_loss_resume_collection_command="$(value_for_key power_loss_resume_collection_command "$gap_report")"
runtime_deployment_collection_command="$(value_for_key runtime_deployment_collection_command "$gap_report")"
runtime_stability_24h_collection_command="$(value_for_key runtime_stability_24h_collection_command "$gap_report")"
field_acceptance_collection_command="$(value_for_key field_acceptance_collection_command "$gap_report")"
final_gate_command="$(value_for_key final_gate_command "$gap_report")"

handoff_status="FAIL"
handoff_ready="NO"
if [[ "$readiness_validation_exit" -eq 0 \
      && "$readiness_validation_status" == "PASS" \
      && "$readiness_status" == "PASS" \
      && "$user_bag_test_ready" == "YES" \
      && "$gap_validation_exit" -eq 0 \
      && "$gap_validation_status" == "PASS" \
      && "$field_acceptance_ready" == "NO" \
      && "$field_acceptance_eligible" == "NO" \
      && "$field_acceptance_status" == "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY" ]]; then
  handoff_status="PASS"
  handoff_ready="YES"
fi

{
  echo "field_acceptance_handoff_status=$handoff_status"
  echo "field_acceptance_handoff_ready=$handoff_ready"
  echo "field_acceptance_handoff_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
  echo "actual_bag_initial_test_readiness_validation_status=${readiness_validation_status:-missing}"
  echo "actual_bag_initial_test_readiness_status=${readiness_status:-missing}"
  echo "actual_bag_user_bag_test_ready=${user_bag_test_ready:-missing}"
  echo "field_acceptance_gap_validation_status=${gap_validation_status:-missing}"
  echo "field_acceptance_ready=${field_acceptance_ready:-missing}"
  echo "field_acceptance_eligible=${field_acceptance_eligible:-missing}"
  echo "field_acceptance_status=${field_acceptance_status:-missing}"
  echo "required_next_evidence=${required_next_evidence:-missing}"
  echo "plc_feedback_collection_command=${plc_feedback_collection_command:-missing}"
  echo "section_export_collection_command=${section_export_collection_command:-missing}"
  echo "pps_ptp_wiring_collection_command=${pps_ptp_wiring_collection_command:-missing}"
  echo "power_loss_resume_collection_command=${power_loss_resume_collection_command:-missing}"
  echo "runtime_deployment_collection_command=${runtime_deployment_collection_command:-missing}"
  echo "runtime_stability_24h_collection_command=${runtime_stability_24h_collection_command:-missing}"
  echo "field_acceptance_collection_command=${field_acceptance_collection_command:-missing}"
  echo "final_gate_command=${final_gate_command:-missing}"
  echo "bag_path=$bag_path"
  echo "actual_bag_initial_test_readiness_report=$readiness_report"
  echo "actual_bag_initial_test_readiness_validation_report=$readiness_validation_report"
  echo "field_acceptance_gap_report=$gap_report"
  echo "field_acceptance_gap_validation_report=$gap_validation_report"
} > "$handoff_report"

[[ "$handoff_status" == "PASS" ]]
EOF
} > "$commands_dir/generate_field_acceptance_handoff.sh"
write_executable "$commands_dir/generate_field_acceptance_handoff.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -euo pipefail'
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "handoff_report=$(shell_quote "$reports_dir/field_acceptance_handoff.txt")"
  echo "validation_report=$(shell_quote "$reports_dir/field_acceptance_handoff_validation.txt")"
  echo "plc_feedback_collection_command=$(shell_quote "$plc_feedback_collection_command")"
  echo "section_export_collection_command=$(shell_quote "$section_export_collection_command")"
  echo "pps_ptp_wiring_collection_command=$(shell_quote "$pps_ptp_wiring_collection_command")"
  echo "power_loss_resume_collection_command=$(shell_quote "$power_loss_resume_collection_command")"
  echo "runtime_deployment_collection_command=$(shell_quote "$runtime_deployment_collection_command")"
  echo "runtime_stability_24h_collection_command=$(shell_quote "$runtime_stability_24h_collection_command")"
  echo "field_acceptance_collection_command=$(shell_quote "$field_acceptance_collection_command")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  cat <<'EOF'

declare -A values
declare -A statuses
overall="PASS"
duplicate_keys=0
malformed_lines=0

mark_fail() {
  local key="$1"
  statuses["$key"]="FAIL"
  overall="FAIL"
}

mark_pass() {
  local key="$1"
  if [[ "${statuses[$key]:-}" != "FAIL" ]]; then
    statuses["$key"]="PASS"
  fi
}

check_equals() {
  local key="$1"
  local expected="$2"
  if [[ "${values[$key]+set}" != "set" || "${values[$key]}" != "$expected" ]]; then
    mark_fail "$key"
  else
    mark_pass "$key"
  fi
}

if [[ ! -f "$handoff_report" ]]; then
  mkdir -p "$reports_dir"
  {
    echo "field_acceptance_handoff_validation_status=FAIL"
    echo "field_acceptance_handoff_report=$handoff_report"
    echo "field_acceptance_handoff_report_status=MISSING"
  } > "$validation_report"
  exit 1
fi

while IFS= read -r line || [[ -n "$line" ]]; do
  [[ -z "$line" ]] && continue
  if [[ "$line" != *=* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  key="${line%%=*}"
  value="${line#*=}"
  if [[ -z "$key" || "$key" == *[[:space:]]* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  if [[ "${values[$key]+set}" == "set" ]]; then
    duplicate_keys=$((duplicate_keys + 1))
    overall="FAIL"
    continue
  fi
  values["$key"]="$value"
done < "$handoff_report"

check_equals "field_acceptance_handoff_status" "PASS"
check_equals "field_acceptance_handoff_ready" "YES"
check_equals "field_acceptance_handoff_scope" "FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
check_equals "actual_bag_initial_test_readiness_validation_status" "PASS"
check_equals "actual_bag_initial_test_readiness_status" "PASS"
check_equals "actual_bag_user_bag_test_ready" "YES"
check_equals "field_acceptance_gap_validation_status" "PASS"
check_equals "field_acceptance_ready" "NO"
check_equals "field_acceptance_eligible" "NO"
check_equals "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_equals "required_next_evidence" "PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
check_equals "plc_feedback_collection_command" "$plc_feedback_collection_command"
check_equals "section_export_collection_command" "$section_export_collection_command"
check_equals "pps_ptp_wiring_collection_command" "$pps_ptp_wiring_collection_command"
check_equals "power_loss_resume_collection_command" "$power_loss_resume_collection_command"
check_equals "runtime_deployment_collection_command" "$runtime_deployment_collection_command"
check_equals "runtime_stability_24h_collection_command" "$runtime_stability_24h_collection_command"
check_equals "field_acceptance_collection_command" "$field_acceptance_collection_command"
check_equals "final_gate_command" "record_session.sh generated commands/validate_evidence.sh"
check_equals "bag_path" "$bag_path"

{
  echo "field_acceptance_handoff_validation_status=$overall"
  echo "field_acceptance_handoff_report=$handoff_report"
  echo "duplicate_key_count=$duplicate_keys"
  echo "malformed_line_count=$malformed_lines"
  for key in "${!statuses[@]}"; do
    echo "${key}_status=${statuses[$key]}"
  done | sort
  echo "field_acceptance_handoff_status=${values[field_acceptance_handoff_status]:-missing}"
  echo "field_acceptance_handoff_ready=${values[field_acceptance_handoff_ready]:-missing}"
  echo "field_acceptance_status=${values[field_acceptance_status]:-missing}"
  echo "required_next_evidence=${values[required_next_evidence]:-missing}"
  echo "field_acceptance_collection_command=${values[field_acceptance_collection_command]:-missing}"
} > "$validation_report"

[[ "$overall" == "PASS" ]]
EOF
} > "$commands_dir/validate_field_acceptance_handoff.sh"
write_executable "$commands_dir/validate_field_acceptance_handoff.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -euo pipefail'
  echo "suite_root=$(shell_quote "$out_abs")"
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "handoff_manifest=$(shell_quote "$reports_dir/field_acceptance_handoff_manifest.txt")"
  echo "validation_report=$(shell_quote "$reports_dir/field_acceptance_handoff_manifest_validation.txt")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  cat <<'EOF'

declare -A values
declare -A statuses
overall="PASS"
duplicate_keys=0
malformed_lines=0

mark_fail() {
  local key="$1"
  statuses["$key"]="FAIL"
  overall="FAIL"
}

mark_pass() {
  local key="$1"
  if [[ "${statuses[$key]:-}" != "FAIL" ]]; then
    statuses["$key"]="PASS"
  fi
}

value_for_key() {
  local key="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    awk -v target="$key" '
      {
        pos = index($0, "=")
        if (pos > 0 && substr($0, 1, pos - 1) == target) {
          value = substr($0, pos + 1)
          found = 1
        }
      }
      END {
        if (found) {
          print value
        }
      }
    ' "$path"
  fi
}

manifest_path() {
  local key="$1"
  local rel="${values[$key]:-}"
  if [[ -z "$rel" || "$rel" == "missing" || "$rel" == "__DUPLICATE_KEY__" ]]; then
    echo ""
  elif [[ "$rel" == /* ]]; then
    echo "$rel"
  else
    echo "$suite_root/$rel"
  fi
}

check_manifest_path() {
  local key="$1"
  local expected="$2"
  local actual="${values[$key]:-}"
  if [[ "$actual" == "$expected" ]]; then
    mark_pass "${key}_path"
  else
    mark_fail "${key}_path"
  fi
}

check_equals() {
  local key="$1"
  local expected="$2"
  if [[ "${values[$key]+set}" != "set" || "${values[$key]}" != "$expected" ]]; then
    mark_fail "$key"
  else
    mark_pass "$key"
  fi
}

check_file_key() {
  local key="$1"
  local path
  path="$(manifest_path "$key")"
  if [[ -n "$path" && -f "$path" ]]; then
    mark_pass "$key"
  else
    mark_fail "$key"
  fi
}

check_executable_key() {
  local key="$1"
  local path="${values[$key]:-}"
  if [[ -n "$path" && -x "$path" ]]; then
    mark_pass "$key"
  else
    mark_fail "$key"
  fi
}

check_report_equals() {
  local label="$1"
  local path="$2"
  local key="$3"
  local expected="$4"
  local actual
  actual="$(value_for_key "$key" "$path")"
  if [[ "$actual" == "$expected" ]]; then
    mark_pass "${label}_${key}"
  else
    mark_fail "${label}_${key}"
  fi
}

if [[ ! -f "$handoff_manifest" ]]; then
  mkdir -p "$reports_dir"
  {
    echo "field_acceptance_handoff_manifest_validation_status=FAIL"
    echo "field_acceptance_handoff_manifest=$handoff_manifest"
    echo "field_acceptance_handoff_manifest_status=MISSING"
  } > "$validation_report"
  exit 1
fi

while IFS= read -r line || [[ -n "$line" ]]; do
  [[ -z "$line" ]] && continue
  if [[ "$line" != *=* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  key="${line%%=*}"
  value="${line#*=}"
  if [[ -z "$key" || "$key" == *[[:space:]]* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  if [[ "${values[$key]+set}" == "set" ]]; then
    duplicate_keys=$((duplicate_keys + 1))
    overall="FAIL"
    continue
  fi
  values["$key"]="$value"
done < "$handoff_manifest"

check_equals "field_acceptance_handoff_manifest_status" "READY"
check_equals "handoff_bundle_scope" "FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
check_equals "actual_bag_test_scope" "INITIAL_LIDAR_IMU_ONLY"
check_equals "field_acceptance_eligible" "NO"
check_equals "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_equals "required_next_evidence" "PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
check_equals "bag_path" "$bag_path"
check_manifest_path "suite_summary" "reports/actual_bag_test_suite_summary.txt"
check_manifest_path "suite_manifest" "reports/actual_bag_test_suite_manifest.txt"
check_manifest_path "suite_manifest_validation" "reports/actual_bag_test_suite_manifest_validation.txt"
check_manifest_path "field_acceptance_gap_report" "reports/field_acceptance_gap_report.txt"
check_manifest_path "field_acceptance_gap_validation" "reports/field_acceptance_gap_validation.txt"
check_manifest_path "actual_bag_initial_test_readiness" "reports/actual_bag_initial_test_readiness.txt"
check_manifest_path "actual_bag_initial_test_readiness_validation" "reports/actual_bag_initial_test_readiness_validation.txt"
check_manifest_path "field_acceptance_handoff" "reports/field_acceptance_handoff.txt"
check_manifest_path "field_acceptance_handoff_validation" "reports/field_acceptance_handoff_validation.txt"

for key in \
  suite_summary suite_manifest suite_manifest_validation field_acceptance_gap_report \
  field_acceptance_gap_validation actual_bag_initial_test_readiness \
  actual_bag_initial_test_readiness_validation field_acceptance_handoff \
  field_acceptance_handoff_validation; do
  check_file_key "$key"
done

for key in \
  validate_command field_acceptance_gap_validation_command \
  actual_bag_initial_test_readiness_validation_command \
  field_acceptance_handoff_validation_command \
  field_acceptance_handoff_manifest_validation_command; do
  check_executable_key "$key"
done

suite_summary_path="$(manifest_path suite_summary)"
suite_manifest_validation_path="$(manifest_path suite_manifest_validation)"
gap_report_path="$(manifest_path field_acceptance_gap_report)"
gap_validation_path="$(manifest_path field_acceptance_gap_validation)"
readiness_path="$(manifest_path actual_bag_initial_test_readiness)"
readiness_validation_path="$(manifest_path actual_bag_initial_test_readiness_validation)"
handoff_path="$(manifest_path field_acceptance_handoff)"
handoff_validation_path="$(manifest_path field_acceptance_handoff_validation)"

check_report_equals "suite_summary" "$suite_summary_path" "actual_bag_test_suite_status" "PASS"
check_report_equals "suite_summary" "$suite_summary_path" "field_acceptance_eligible" "NO"
check_report_equals "suite_summary" "$suite_summary_path" "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_report_equals "suite_manifest_validation" "$suite_manifest_validation_path" "actual_bag_test_suite_manifest_validation_status" "PASS"
check_report_equals "field_acceptance_gap_report" "$gap_report_path" "field_acceptance_ready" "NO"
check_report_equals "field_acceptance_gap_report" "$gap_report_path" "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_report_equals "field_acceptance_gap_validation" "$gap_validation_path" "field_acceptance_gap_validation_status" "PASS"
check_report_equals "actual_bag_initial_test_readiness" "$readiness_path" "actual_bag_initial_test_readiness_status" "PASS"
check_report_equals "actual_bag_initial_test_readiness" "$readiness_path" "actual_bag_user_bag_test_ready" "YES"
check_report_equals "actual_bag_initial_test_readiness" "$readiness_path" "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_report_equals "actual_bag_initial_test_readiness_validation" "$readiness_validation_path" "actual_bag_initial_test_readiness_validation_status" "PASS"
check_report_equals "field_acceptance_handoff" "$handoff_path" "field_acceptance_handoff_status" "PASS"
check_report_equals "field_acceptance_handoff" "$handoff_path" "field_acceptance_handoff_ready" "YES"
check_report_equals "field_acceptance_handoff" "$handoff_path" "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_report_equals "field_acceptance_handoff_validation" "$handoff_validation_path" "field_acceptance_handoff_validation_status" "PASS"

{
  echo "field_acceptance_handoff_manifest_validation_status=$overall"
  echo "field_acceptance_handoff_manifest=$handoff_manifest"
  echo "duplicate_key_count=$duplicate_keys"
  echo "malformed_line_count=$malformed_lines"
  for key in "${!statuses[@]}"; do
    echo "${key}_status=${statuses[$key]}"
  done | sort
  echo "field_acceptance_eligible=${values[field_acceptance_eligible]:-missing}"
  echo "field_acceptance_status=${values[field_acceptance_status]:-missing}"
  echo "required_next_evidence=${values[required_next_evidence]:-missing}"
  echo "field_acceptance_handoff=${values[field_acceptance_handoff]:-missing}"
  echo "field_acceptance_handoff_validation=${values[field_acceptance_handoff_validation]:-missing}"
} > "$validation_report"

[[ "$overall" == "PASS" ]]
EOF
} > "$commands_dir/validate_field_acceptance_handoff_manifest.sh"
write_executable "$commands_dir/validate_field_acceptance_handoff_manifest.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -uo pipefail'
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "logs_dir=$(shell_quote "$logs_dir")"
  echo "handoff_report=$(shell_quote "$reports_dir/field_acceptance_handoff.txt")"
  echo "handoff_validation_report=$(shell_quote "$reports_dir/field_acceptance_handoff_validation.txt")"
  echo "handoff_manifest=$(shell_quote "$reports_dir/field_acceptance_handoff_manifest.txt")"
  echo "handoff_manifest_validation_report=$(shell_quote "$reports_dir/field_acceptance_handoff_manifest_validation.txt")"
  echo "collection_plan=$(shell_quote "$reports_dir/field_acceptance_collection_plan.txt")"
  echo "handoff_validation_command=$(shell_quote "$commands_dir/validate_field_acceptance_handoff.sh")"
  echo "handoff_manifest_validation_command=$(shell_quote "$commands_dir/validate_field_acceptance_handoff_manifest.sh")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  cat <<'EOF'

value_for_key() {
  local key="$1"
  local path="$2"
  if [[ -f "$path" ]]; then
    awk -v target="$key" '
      {
        pos = index($0, "=")
        if (pos > 0 && substr($0, 1, pos - 1) == target) {
          value = substr($0, pos + 1)
          found = 1
        }
      }
      END {
        if (found) {
          print value
        }
      }
    ' "$path"
  fi
}

mkdir -p "$reports_dir" "$logs_dir"

handoff_validation_exit=0
if "$handoff_validation_command" > "$logs_dir/field_acceptance_collection_plan_handoff_validation.log" 2>&1; then
  handoff_validation_exit=0
else
  handoff_validation_exit=$?
fi

handoff_manifest_validation_exit=0
if "$handoff_manifest_validation_command" > "$logs_dir/field_acceptance_collection_plan_handoff_manifest_validation.log" 2>&1; then
  handoff_manifest_validation_exit=0
else
  handoff_manifest_validation_exit=$?
fi

handoff_status="$(value_for_key field_acceptance_handoff_status "$handoff_report")"
handoff_ready="$(value_for_key field_acceptance_handoff_ready "$handoff_report")"
handoff_validation_status="$(value_for_key field_acceptance_handoff_validation_status "$handoff_validation_report")"
handoff_manifest_validation_status="$(value_for_key field_acceptance_handoff_manifest_validation_status "$handoff_manifest_validation_report")"
field_acceptance_eligible="$(value_for_key field_acceptance_eligible "$handoff_report")"
field_acceptance_status="$(value_for_key field_acceptance_status "$handoff_report")"
required_next_evidence="$(value_for_key required_next_evidence "$handoff_report")"

plc_feedback_collection_command="$(value_for_key plc_feedback_collection_command "$handoff_report")"
section_export_collection_command="$(value_for_key section_export_collection_command "$handoff_report")"
pps_ptp_wiring_collection_command="$(value_for_key pps_ptp_wiring_collection_command "$handoff_report")"
power_loss_resume_collection_command="$(value_for_key power_loss_resume_collection_command "$handoff_report")"
runtime_deployment_collection_command="$(value_for_key runtime_deployment_collection_command "$handoff_report")"
runtime_stability_24h_collection_command="$(value_for_key runtime_stability_24h_collection_command "$handoff_report")"
field_acceptance_collection_command="$(value_for_key field_acceptance_collection_command "$handoff_report")"
final_gate_command="$(value_for_key final_gate_command "$handoff_report")"

plan_status="FAIL"
plan_ready="NO"
if [[ "$handoff_validation_exit" -eq 0 \
      && "$handoff_manifest_validation_exit" -eq 0 \
      && "$handoff_status" == "PASS" \
      && "$handoff_ready" == "YES" \
      && "$handoff_validation_status" == "PASS" \
      && "$handoff_manifest_validation_status" == "PASS" \
      && "$field_acceptance_eligible" == "NO" \
      && "$field_acceptance_status" == "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY" \
      && "$required_next_evidence" == "PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE" ]]; then
  plan_status="PASS"
  plan_ready="YES"
fi

{
  echo "field_acceptance_collection_plan_status=$plan_status"
  echo "collection_plan_ready=$plan_ready"
  echo "collection_plan_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
  echo "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"
  echo "field_acceptance_eligible=${field_acceptance_eligible:-missing}"
  echo "field_acceptance_status=${field_acceptance_status:-missing}"
  echo "field_acceptance_handoff_status=${handoff_status:-missing}"
  echo "field_acceptance_handoff_ready=${handoff_ready:-missing}"
  echo "field_acceptance_handoff_validation_status=${handoff_validation_status:-missing}"
  echo "field_acceptance_handoff_manifest_validation_status=${handoff_manifest_validation_status:-missing}"
  echo "required_next_evidence=${required_next_evidence:-missing}"
  echo "plc_feedback_collection_step=1"
  echo "plc_feedback_collection_command=${plc_feedback_collection_command:-missing}"
  echo "section_export_collection_step=2"
  echo "section_export_collection_command=${section_export_collection_command:-missing}"
  echo "pps_ptp_wiring_collection_step=3"
  echo "pps_ptp_wiring_collection_command=${pps_ptp_wiring_collection_command:-missing}"
  echo "power_loss_resume_collection_step=4"
  echo "power_loss_resume_collection_command=${power_loss_resume_collection_command:-missing}"
  echo "runtime_deployment_collection_step=5"
  echo "runtime_deployment_collection_command=${runtime_deployment_collection_command:-missing}"
  echo "runtime_stability_24h_collection_step=6"
  echo "runtime_stability_24h_collection_command=${runtime_stability_24h_collection_command:-missing}"
  echo "field_acceptance_collection_step=7"
  echo "field_acceptance_collection_command=${field_acceptance_collection_command:-missing}"
  echo "final_gate_command=${final_gate_command:-missing}"
  echo "final_success_gate=record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS"
  echo "bag_path=$bag_path"
  echo "source_field_acceptance_handoff=$handoff_report"
  echo "source_field_acceptance_handoff_validation=$handoff_validation_report"
  echo "source_field_acceptance_handoff_manifest=$handoff_manifest"
  echo "source_field_acceptance_handoff_manifest_validation=$handoff_manifest_validation_report"
} > "$collection_plan"

[[ "$plan_status" == "PASS" ]]
EOF
} > "$commands_dir/generate_field_acceptance_collection_plan.sh"
write_executable "$commands_dir/generate_field_acceptance_collection_plan.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -euo pipefail'
  echo "reports_dir=$(shell_quote "$reports_dir")"
  echo "collection_plan=$(shell_quote "$reports_dir/field_acceptance_collection_plan.txt")"
  echo "validation_report=$(shell_quote "$reports_dir/field_acceptance_collection_plan_validation.txt")"
  echo "expected_source_field_acceptance_handoff=$(shell_quote "$reports_dir/field_acceptance_handoff.txt")"
  echo "expected_source_field_acceptance_handoff_validation=$(shell_quote "$reports_dir/field_acceptance_handoff_validation.txt")"
  echo "expected_source_field_acceptance_handoff_manifest=$(shell_quote "$reports_dir/field_acceptance_handoff_manifest.txt")"
  echo "expected_source_field_acceptance_handoff_manifest_validation=$(shell_quote "$reports_dir/field_acceptance_handoff_manifest_validation.txt")"
  echo "plc_feedback_collection_command=$(shell_quote "$plc_feedback_collection_command")"
  echo "section_export_collection_command=$(shell_quote "$section_export_collection_command")"
  echo "pps_ptp_wiring_collection_command=$(shell_quote "$pps_ptp_wiring_collection_command")"
  echo "power_loss_resume_collection_command=$(shell_quote "$power_loss_resume_collection_command")"
  echo "runtime_deployment_collection_command=$(shell_quote "$runtime_deployment_collection_command")"
  echo "runtime_stability_24h_collection_command=$(shell_quote "$runtime_stability_24h_collection_command")"
  echo "field_acceptance_collection_command=$(shell_quote "$field_acceptance_collection_command")"
  echo "bag_path=$(shell_quote "$bag_abs")"
  cat <<'EOF'

declare -A values
declare -A statuses
overall="PASS"
duplicate_keys=0
malformed_lines=0

mark_fail() {
  local key="$1"
  statuses["$key"]="FAIL"
  overall="FAIL"
}

mark_pass() {
  local key="$1"
  if [[ "${statuses[$key]:-}" != "FAIL" ]]; then
    statuses["$key"]="PASS"
  fi
}

check_equals() {
  local key="$1"
  local expected="$2"
  if [[ "${values[$key]+set}" != "set" || "${values[$key]}" != "$expected" ]]; then
    mark_fail "$key"
  else
    mark_pass "$key"
  fi
}

check_file_key() {
  local key="$1"
  local path="${values[$key]:-}"
  if [[ -n "$path" && -f "$path" ]]; then
    mark_pass "$key"
  else
    mark_fail "$key"
  fi
}

check_path_equals() {
  local key="$1"
  local expected="$2"
  local path="${values[$key]:-}"
  if [[ "$path" == "$expected" ]]; then
    mark_pass "${key}_path"
  else
    mark_fail "${key}_path"
  fi
}

if [[ ! -f "$collection_plan" ]]; then
  mkdir -p "$reports_dir"
  {
    echo "field_acceptance_collection_plan_validation_status=FAIL"
    echo "field_acceptance_collection_plan=$collection_plan"
    echo "field_acceptance_collection_plan_status=MISSING"
  } > "$validation_report"
  exit 1
fi

while IFS= read -r line || [[ -n "$line" ]]; do
  [[ -z "$line" ]] && continue
  if [[ "$line" != *=* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  key="${line%%=*}"
  value="${line#*=}"
  if [[ -z "$key" || "$key" == *[[:space:]]* ]]; then
    malformed_lines=$((malformed_lines + 1))
    overall="FAIL"
    continue
  fi
  if [[ "${values[$key]+set}" == "set" ]]; then
    duplicate_keys=$((duplicate_keys + 1))
    overall="FAIL"
    continue
  fi
  values["$key"]="$value"
done < "$collection_plan"

check_equals "field_acceptance_collection_plan_status" "PASS"
check_equals "collection_plan_ready" "YES"
check_equals "collection_plan_scope" "FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
check_equals "actual_bag_test_scope" "INITIAL_LIDAR_IMU_ONLY"
check_equals "field_acceptance_eligible" "NO"
check_equals "field_acceptance_status" "NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"
check_equals "field_acceptance_handoff_status" "PASS"
check_equals "field_acceptance_handoff_ready" "YES"
check_equals "field_acceptance_handoff_validation_status" "PASS"
check_equals "field_acceptance_handoff_manifest_validation_status" "PASS"
check_equals "required_next_evidence" "PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
check_equals "plc_feedback_collection_step" "1"
check_equals "plc_feedback_collection_command" "$plc_feedback_collection_command"
check_equals "section_export_collection_step" "2"
check_equals "section_export_collection_command" "$section_export_collection_command"
check_equals "pps_ptp_wiring_collection_step" "3"
check_equals "pps_ptp_wiring_collection_command" "$pps_ptp_wiring_collection_command"
check_equals "power_loss_resume_collection_step" "4"
check_equals "power_loss_resume_collection_command" "$power_loss_resume_collection_command"
check_equals "runtime_deployment_collection_step" "5"
check_equals "runtime_deployment_collection_command" "$runtime_deployment_collection_command"
check_equals "runtime_stability_24h_collection_step" "6"
check_equals "runtime_stability_24h_collection_command" "$runtime_stability_24h_collection_command"
check_equals "field_acceptance_collection_step" "7"
check_equals "field_acceptance_collection_command" "$field_acceptance_collection_command"
check_equals "final_gate_command" "record_session.sh generated commands/validate_evidence.sh"
check_equals "final_success_gate" "record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS"
check_equals "bag_path" "$bag_path"
check_file_key "source_field_acceptance_handoff"
check_file_key "source_field_acceptance_handoff_validation"
check_file_key "source_field_acceptance_handoff_manifest"
check_file_key "source_field_acceptance_handoff_manifest_validation"
check_path_equals "source_field_acceptance_handoff" "$expected_source_field_acceptance_handoff"
check_path_equals "source_field_acceptance_handoff_validation" "$expected_source_field_acceptance_handoff_validation"
check_path_equals "source_field_acceptance_handoff_manifest" "$expected_source_field_acceptance_handoff_manifest"
check_path_equals "source_field_acceptance_handoff_manifest_validation" "$expected_source_field_acceptance_handoff_manifest_validation"

{
  echo "field_acceptance_collection_plan_validation_status=$overall"
  echo "field_acceptance_collection_plan=$collection_plan"
  echo "duplicate_key_count=$duplicate_keys"
  echo "malformed_line_count=$malformed_lines"
  for key in "${!statuses[@]}"; do
    echo "${key}_status=${statuses[$key]}"
  done | sort
  echo "field_acceptance_eligible=${values[field_acceptance_eligible]:-missing}"
  echo "field_acceptance_status=${values[field_acceptance_status]:-missing}"
  echo "required_next_evidence=${values[required_next_evidence]:-missing}"
  echo "final_success_gate=${values[final_success_gate]:-missing}"
} > "$validation_report"

[[ "$overall" == "PASS" ]]
EOF
} > "$commands_dir/validate_field_acceptance_collection_plan.sh"
write_executable "$commands_dir/validate_field_acceptance_collection_plan.sh"

{
  echo '#!/usr/bin/env bash'
  echo 'set -euo pipefail'
  echo "commands_dir=$(shell_quote "$commands_dir")"
  echo "reports_dir=$(shell_quote "$reports_dir")"
  cat <<'EOF'

"$commands_dir/run_suite.sh"
"$commands_dir/validate_actual_bag_test_suite.sh"

field_acceptance_gap_audit_exit=0
if "$commands_dir/audit_field_acceptance_gap.sh"; then
  field_acceptance_gap_audit_exit=0
else
  field_acceptance_gap_audit_exit=$?
fi

if [[ "$field_acceptance_gap_audit_exit" -ne 1 ]]; then
  {
    echo "verified_suite_status=FAIL"
    echo "field_acceptance_gap_audit_exit=$field_acceptance_gap_audit_exit"
    echo "field_acceptance_gap_audit_expected_exit=1"
  } >> "$reports_dir/actual_bag_test_suite_summary.txt"
  exit 1
fi

"$commands_dir/validate_field_acceptance_gap.sh"
"$commands_dir/audit_actual_bag_initial_test_readiness.sh"
"$commands_dir/validate_actual_bag_initial_test_readiness.sh"
"$commands_dir/generate_field_acceptance_handoff.sh"
"$commands_dir/validate_field_acceptance_handoff.sh"
"$commands_dir/validate_field_acceptance_handoff_manifest.sh"
"$commands_dir/generate_field_acceptance_collection_plan.sh"
"$commands_dir/validate_field_acceptance_collection_plan.sh"

{
  echo "verified_suite_status=PASS"
  echo "suite_manifest_validation_after_execute=PASS"
  echo "field_acceptance_gap_audit_exit=1"
  echo "field_acceptance_gap_validation_after_execute=PASS"
  echo "actual_bag_initial_test_readiness_after_execute=PASS"
  echo "field_acceptance_handoff_after_execute=PASS"
  echo "field_acceptance_handoff_manifest_after_execute=PASS"
  echo "field_acceptance_collection_plan_after_execute=PASS"
} >> "$reports_dir/actual_bag_test_suite_summary.txt"

"$commands_dir/validate_actual_bag_test_suite.sh"
EOF
} > "$commands_dir/run_verified_suite.sh"
write_executable "$commands_dir/run_verified_suite.sh"

{
  echo "actual_bag_test_suite_manifest_status=READY"
  echo "suite_root=$out_abs"
  echo "bag_path=$bag_abs"
  echo "actual_bag_test_scope=$actual_bag_test_scope"
  echo "bag_sensor_set=$bag_sensor_set"
  echo "validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
  echo "plc_feedback_status=$plc_feedback_status"
  echo "time_reference_status=$time_reference_status"
  echo "time_sync_evidence_status=$time_sync_evidence_status"
  echo "initial_velocity_reference_status=$initial_velocity_reference_status"
  echo "initial_velocity_reference_required=$initial_velocity_reference_required"
  echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
  echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
  echo "field_acceptance_eligible=$field_acceptance_eligible"
  echo "summary=reports/actual_bag_test_suite_summary.txt"
  echo "metrics_report=reports/actual_bag_test_suite_metrics_report.txt"
  echo "event_file=reports/actual_bag_test_suite_events.txt"
  echo "ros_residual_report=reports/ros_residual_processes.txt"
  echo "smoke_summary=$smoke_dir_name/reports/actual_bag_replay_summary.txt"
  echo "full_summary=$full_dir_name/reports/actual_bag_replay_summary.txt"
  echo "smoke_event_validation=$smoke_dir_name/reports/actual_bag_replay_hil_validation_report.txt"
  echo "full_event_validation=$full_dir_name/reports/actual_bag_replay_hil_validation_report.txt"
  echo "smoke_initial_velocity=$smoke_dir_name/reports/initial_velocity_reference.txt"
  echo "full_initial_velocity=$full_dir_name/reports/initial_velocity_reference.txt"
  echo "smoke_inspection=$smoke_dir_name/reports/actual_bag_inspection.txt"
  echo "full_inspection=$full_dir_name/reports/actual_bag_inspection.txt"
  echo "field_acceptance_gap_report=reports/field_acceptance_gap_report.txt"
  echo "actual_bag_initial_test_readiness=reports/actual_bag_initial_test_readiness.txt"
  echo "field_acceptance_handoff=reports/field_acceptance_handoff.txt"
  echo "field_acceptance_handoff_manifest=reports/field_acceptance_handoff_manifest.txt"
  echo "field_acceptance_collection_plan=reports/field_acceptance_collection_plan.txt"
  echo "field_acceptance_collection_plan_validation=reports/field_acceptance_collection_plan_validation.txt"
  echo "run_verified_command=$commands_dir/run_verified_suite.sh"
  echo "validate_command=$commands_dir/validate_actual_bag_test_suite.sh"
  echo "field_acceptance_gap_audit_command=$commands_dir/audit_field_acceptance_gap.sh"
  echo "field_acceptance_gap_validation_command=$commands_dir/validate_field_acceptance_gap.sh"
  echo "actual_bag_initial_test_readiness_audit_command=$commands_dir/audit_actual_bag_initial_test_readiness.sh"
  echo "actual_bag_initial_test_readiness_validation_command=$commands_dir/validate_actual_bag_initial_test_readiness.sh"
  echo "field_acceptance_handoff_command=$commands_dir/generate_field_acceptance_handoff.sh"
  echo "field_acceptance_handoff_validation_command=$commands_dir/validate_field_acceptance_handoff.sh"
  echo "field_acceptance_handoff_manifest_validation_command=$commands_dir/validate_field_acceptance_handoff_manifest.sh"
  echo "field_acceptance_collection_plan_command=$commands_dir/generate_field_acceptance_collection_plan.sh"
  echo "field_acceptance_collection_plan_validation_command=$commands_dir/validate_field_acceptance_collection_plan.sh"
} > "$reports_dir/actual_bag_test_suite_manifest.txt"

{
  echo "actual_bag_test_suite_plan_status=READY"
  echo "bag_path=$bag_abs"
  echo "out_dir=$out_abs"
  echo "smoke_duration_s=$smoke_duration_s"
  echo "full_duration_s=$full_duration_s"
  echo "rate=$rate"
  echo "center_lidar_topic=$center_topic"
  echo "left_lidar_topic=$left_topic"
  echo "right_lidar_topic=$right_topic"
  echo "imu_topic=$imu_topic"
  echo "time_reference_topic=$reported_time_reference_topic"
  echo "time_reference_status=$time_reference_status"
  echo "time_sync_evidence_status=$time_sync_evidence_status"
  echo "initial_velocity_reference_status=$initial_velocity_reference_status"
  echo "initial_velocity_reference_required=$initial_velocity_reference_required"
  echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
  echo "actual_bag_test_scope=$actual_bag_test_scope"
  echo "bag_sensor_set=$bag_sensor_set"
  echo "plc_feedback_status=$plc_feedback_status"
  echo "plc_feedback_gate_status=$plc_feedback_gate_status"
  echo "machine_motion_assumption=$machine_motion_assumption"
  echo "vibration_profile=$vibration_profile"
  echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
  echo "local_odometry_config=$local_odometry_config_report"
  echo "replay_script=$replay_script"
  echo "smoke_replay_dir=$smoke_out"
  echo "full_replay_dir=$full_out"
  echo "smoke_replay_command=$commands_dir/run_smoke_replay.sh"
  echo "full_replay_command=$commands_dir/run_full_replay.sh"
  echo "run_command=$commands_dir/run_suite.sh"
  echo "run_verified_command=$commands_dir/run_verified_suite.sh"
  echo "validate_command=$commands_dir/validate_actual_bag_test_suite.sh"
  echo "field_acceptance_gap_audit_command=$commands_dir/audit_field_acceptance_gap.sh"
  echo "field_acceptance_gap_validation_command=$commands_dir/validate_field_acceptance_gap.sh"
  echo "actual_bag_initial_test_readiness_audit_command=$commands_dir/audit_actual_bag_initial_test_readiness.sh"
  echo "actual_bag_initial_test_readiness_validation_command=$commands_dir/validate_actual_bag_initial_test_readiness.sh"
  echo "field_acceptance_handoff_command=$commands_dir/generate_field_acceptance_handoff.sh"
  echo "field_acceptance_handoff_validation_command=$commands_dir/validate_field_acceptance_handoff.sh"
  echo "field_acceptance_handoff_manifest_validation_command=$commands_dir/validate_field_acceptance_handoff_manifest.sh"
  echo "field_acceptance_collection_plan_command=$commands_dir/generate_field_acceptance_collection_plan.sh"
  echo "field_acceptance_collection_plan_validation_command=$commands_dir/validate_field_acceptance_collection_plan.sh"
  echo "actual_bag_test_suite_manifest=reports/actual_bag_test_suite_manifest.txt"
  echo "field_acceptance_gap_report=reports/field_acceptance_gap_report.txt"
  echo "actual_bag_initial_test_readiness=reports/actual_bag_initial_test_readiness.txt"
  echo "field_acceptance_handoff=reports/field_acceptance_handoff.txt"
  echo "field_acceptance_handoff_manifest=reports/field_acceptance_handoff_manifest.txt"
  echo "field_acceptance_collection_plan=reports/field_acceptance_collection_plan.txt"
  echo "generated_event_validation=YES"
  echo "field_acceptance_requires_plc_feedback=$field_acceptance_requires_plc_feedback"
  echo "field_acceptance_eligible=$field_acceptance_eligible"
  echo "field_acceptance_status=$field_acceptance_status"
  echo "velocity_reference_played_to_slam=NO"
  echo "continuous_velocity_reference_used=NO"
} > "$reports_dir/actual_bag_test_suite_plan.txt"

{
  echo "field_acceptance_handoff_manifest_status=READY"
  echo "handoff_bundle_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
  echo "actual_bag_test_scope=$actual_bag_test_scope"
  echo "field_acceptance_eligible=$field_acceptance_eligible"
  echo "field_acceptance_status=$field_acceptance_status"
  echo "required_next_evidence=PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
  echo "bag_path=$bag_abs"
  echo "suite_summary=reports/actual_bag_test_suite_summary.txt"
  echo "suite_manifest=reports/actual_bag_test_suite_manifest.txt"
  echo "suite_manifest_validation=reports/actual_bag_test_suite_manifest_validation.txt"
  echo "field_acceptance_gap_report=reports/field_acceptance_gap_report.txt"
  echo "field_acceptance_gap_validation=reports/field_acceptance_gap_validation.txt"
  echo "actual_bag_initial_test_readiness=reports/actual_bag_initial_test_readiness.txt"
  echo "actual_bag_initial_test_readiness_validation=reports/actual_bag_initial_test_readiness_validation.txt"
  echo "field_acceptance_handoff=reports/field_acceptance_handoff.txt"
  echo "field_acceptance_handoff_validation=reports/field_acceptance_handoff_validation.txt"
  echo "validate_command=$commands_dir/validate_actual_bag_test_suite.sh"
  echo "field_acceptance_gap_validation_command=$commands_dir/validate_field_acceptance_gap.sh"
  echo "actual_bag_initial_test_readiness_validation_command=$commands_dir/validate_actual_bag_initial_test_readiness.sh"
  echo "field_acceptance_handoff_validation_command=$commands_dir/validate_field_acceptance_handoff.sh"
  echo "field_acceptance_handoff_manifest_validation_command=$commands_dir/validate_field_acceptance_handoff_manifest.sh"
} > "$reports_dir/field_acceptance_handoff_manifest.txt"

if [[ "$execute_suite" -eq 1 ]]; then
  "$commands_dir/run_verified_suite.sh"
else
  {
    echo "actual_bag_test_suite_status=DRY_RUN"
    echo "bag_path=$bag_abs"
    echo "suite_root=$out_abs"
    echo "smoke_duration_s=$smoke_duration_s"
    echo "full_duration_s=$full_duration_s"
    echo "rate=$rate"
    echo "run_command=$commands_dir/run_suite.sh"
    echo "run_verified_command=$commands_dir/run_verified_suite.sh"
    echo "validate_command=$commands_dir/validate_actual_bag_test_suite.sh"
    echo "field_acceptance_gap_audit_command=$commands_dir/audit_field_acceptance_gap.sh"
    echo "field_acceptance_gap_validation_command=$commands_dir/validate_field_acceptance_gap.sh"
    echo "actual_bag_initial_test_readiness_audit_command=$commands_dir/audit_actual_bag_initial_test_readiness.sh"
    echo "actual_bag_initial_test_readiness_validation_command=$commands_dir/validate_actual_bag_initial_test_readiness.sh"
  echo "field_acceptance_handoff_command=$commands_dir/generate_field_acceptance_handoff.sh"
  echo "field_acceptance_handoff_validation_command=$commands_dir/validate_field_acceptance_handoff.sh"
  echo "field_acceptance_handoff_manifest_validation_command=$commands_dir/validate_field_acceptance_handoff_manifest.sh"
  echo "field_acceptance_collection_plan_command=$commands_dir/generate_field_acceptance_collection_plan.sh"
  echo "field_acceptance_collection_plan_validation_command=$commands_dir/validate_field_acceptance_collection_plan.sh"
  echo "actual_bag_test_suite_manifest=reports/actual_bag_test_suite_manifest.txt"
  echo "field_acceptance_gap_report=reports/field_acceptance_gap_report.txt"
  echo "actual_bag_initial_test_readiness=reports/actual_bag_initial_test_readiness.txt"
  echo "field_acceptance_handoff=reports/field_acceptance_handoff.txt"
  echo "field_acceptance_handoff_manifest=reports/field_acceptance_handoff_manifest.txt"
  echo "field_acceptance_collection_plan=reports/field_acceptance_collection_plan.txt"
    echo "actual_bag_test_scope=$actual_bag_test_scope"
    echo "bag_sensor_set=$bag_sensor_set"
    echo "plc_feedback_status=$plc_feedback_status"
    echo "plc_feedback_gate_status=$plc_feedback_gate_status"
    echo "machine_motion_assumption=$machine_motion_assumption"
    echo "vibration_profile=$vibration_profile"
    echo "initial_velocity_reference_status=$initial_velocity_reference_status"
    echo "initial_velocity_reference_required=$initial_velocity_reference_required"
    echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
    echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
    echo "velocity_reference_played_to_slam=NO"
    echo "continuous_velocity_reference_used=NO"
    echo "generated_event_validation=YES"
    echo "field_acceptance_requires_plc_feedback=$field_acceptance_requires_plc_feedback"
    echo "field_acceptance_eligible=$field_acceptance_eligible"
    echo "field_acceptance_status=$field_acceptance_status"
  } > "$reports_dir/actual_bag_test_suite_summary.txt"

  {
    echo "overall=FAIL;total_records=1;failed_records=1"
    echo "session=$(basename "$out_abs");scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;status=DRY_RUN;failed_checks=1;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=$actual_bag_test_scope"
    echo "---"
    echo "detail=actual_bag_test_suite_status;status=DRY_RUN;value=DRY_RUN;threshold=PASS"
    echo "detail=field_acceptance_eligible;status=PASS;value=NO;threshold=NO"
  } > "$reports_dir/actual_bag_test_suite_metrics_report.txt"

  {
    echo "event=session_start;scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;session_id=$(basename "$out_abs");t=0.000;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=$actual_bag_test_scope"
    echo "event=actual_bag_test_suite;scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;session_id=$(basename "$out_abs");t=$full_duration_s;queue_backlog=-1;actual_bag_test_suite_status=DRY_RUN;plc_feedback_status=$plc_feedback_status;field_acceptance_eligible=NO;velocity_reference_played_to_slam=NO;continuous_velocity_reference_used=NO"
  } > "$reports_dir/actual_bag_test_suite_events.txt"

  {
    echo "field_acceptance_gap_audit_status=DRY_RUN"
    echo "actual_bag_initial_evidence_status=DRY_RUN"
    echo "actual_bag_test_scope=$actual_bag_test_scope"
    echo "validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
    echo "field_acceptance_ready=NO"
    echo "field_acceptance_eligible=NO"
    echo "field_acceptance_status=$field_acceptance_status"
    echo "plc_feedback_evidence_status=MISSING"
    echo "section_export_evidence_status=MISSING"
    echo "pps_ptp_wiring_evidence_status=MISSING"
    echo "power_loss_resume_evidence_status=MISSING"
    echo "runtime_deployment_evidence_status=MISSING"
    echo "runtime_stability_24h_evidence_status=MISSING"
    echo "field_acceptance_report_status=MISSING"
    echo "required_next_evidence=PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
    echo "plc_feedback_collection_command=$plc_feedback_collection_command"
    echo "section_export_collection_command=$section_export_collection_command"
    echo "pps_ptp_wiring_collection_command=$pps_ptp_wiring_collection_command"
    echo "power_loss_resume_collection_command=$power_loss_resume_collection_command"
    echo "runtime_deployment_collection_command=$runtime_deployment_collection_command"
    echo "runtime_stability_24h_collection_command=$runtime_stability_24h_collection_command"
    echo "field_acceptance_collection_command=$field_acceptance_collection_command"
    echo "final_gate_command=record_session.sh generated commands/validate_evidence.sh"
	  } > "$reports_dir/field_acceptance_gap_report.txt"

	  {
	    echo "actual_bag_initial_test_readiness_status=DRY_RUN"
	    echo "actual_bag_user_bag_test_ready=NO"
	    echo "actual_bag_initial_evidence_status=DRY_RUN"
	    echo "actual_bag_test_suite_status=DRY_RUN"
	    echo "actual_bag_test_suite_manifest_validation_status=NOT_RUN"
	    echo "field_acceptance_gap_validation_status=NOT_RUN"
	    echo "actual_bag_test_scope=$actual_bag_test_scope"
	    echo "validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"
	    echo "bag_sensor_set=$bag_sensor_set"
	    echo "plc_feedback_status=$plc_feedback_status"
	    echo "plc_feedback_gate_status=$plc_feedback_gate_status"
	    echo "machine_motion_assumption=$machine_motion_assumption"
	    echo "vibration_profile=$vibration_profile"
	    echo "time_reference_status=$time_reference_status"
	    echo "time_sync_evidence_status=$time_sync_evidence_status"
	    echo "initial_velocity_reference_required=$initial_velocity_reference_required"
	    echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
	    echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
	    echo "velocity_reference_played_to_slam=NO"
	    echo "continuous_velocity_reference_used=NO"
	    echo "field_acceptance_eligible=$field_acceptance_eligible"
	    echo "field_acceptance_status=$field_acceptance_status"
	    echo "required_next_evidence=PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
	    echo "bag_path=$bag_abs"
	    echo "recommended_user_bag_entry=actual_bag_profile.sh --bag $bag_abs"
	  } > "$reports_dir/actual_bag_initial_test_readiness.txt"

	  {
	    echo "field_acceptance_handoff_status=DRY_RUN"
	    echo "field_acceptance_handoff_ready=NO"
	    echo "field_acceptance_handoff_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
	    echo "actual_bag_initial_test_readiness_validation_status=NOT_RUN"
	    echo "actual_bag_initial_test_readiness_status=DRY_RUN"
	    echo "actual_bag_user_bag_test_ready=NO"
	    echo "field_acceptance_gap_validation_status=NOT_RUN"
	    echo "field_acceptance_ready=NO"
	    echo "field_acceptance_eligible=$field_acceptance_eligible"
	    echo "field_acceptance_status=$field_acceptance_status"
	    echo "required_next_evidence=PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
	    echo "plc_feedback_collection_command=$plc_feedback_collection_command"
	    echo "section_export_collection_command=$section_export_collection_command"
	    echo "pps_ptp_wiring_collection_command=$pps_ptp_wiring_collection_command"
	    echo "power_loss_resume_collection_command=$power_loss_resume_collection_command"
	    echo "runtime_deployment_collection_command=$runtime_deployment_collection_command"
	    echo "runtime_stability_24h_collection_command=$runtime_stability_24h_collection_command"
	    echo "field_acceptance_collection_command=$field_acceptance_collection_command"
	    echo "final_gate_command=record_session.sh generated commands/validate_evidence.sh"
	    echo "bag_path=$bag_abs"
	  } > "$reports_dir/field_acceptance_handoff.txt"

	  {
	    echo "field_acceptance_collection_plan_status=DRY_RUN"
	    echo "collection_plan_ready=NO"
	    echo "collection_plan_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"
	    echo "actual_bag_test_scope=$actual_bag_test_scope"
	    echo "field_acceptance_eligible=$field_acceptance_eligible"
	    echo "field_acceptance_status=$field_acceptance_status"
	    echo "field_acceptance_handoff_status=DRY_RUN"
	    echo "field_acceptance_handoff_ready=NO"
	    echo "field_acceptance_handoff_validation_status=NOT_RUN"
	    echo "field_acceptance_handoff_manifest_validation_status=NOT_RUN"
	    echo "required_next_evidence=PLC_FEEDBACK_BAG,SECTION_EXPORT,PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"
	    echo "plc_feedback_collection_step=1"
	    echo "plc_feedback_collection_command=$plc_feedback_collection_command"
	    echo "section_export_collection_step=2"
	    echo "section_export_collection_command=$section_export_collection_command"
	    echo "pps_ptp_wiring_collection_step=3"
	    echo "pps_ptp_wiring_collection_command=$pps_ptp_wiring_collection_command"
	    echo "power_loss_resume_collection_step=4"
	    echo "power_loss_resume_collection_command=$power_loss_resume_collection_command"
	    echo "runtime_deployment_collection_step=5"
	    echo "runtime_deployment_collection_command=$runtime_deployment_collection_command"
	    echo "runtime_stability_24h_collection_step=6"
	    echo "runtime_stability_24h_collection_command=$runtime_stability_24h_collection_command"
	    echo "field_acceptance_collection_step=7"
	    echo "field_acceptance_collection_command=$field_acceptance_collection_command"
	    echo "final_gate_command=record_session.sh generated commands/validate_evidence.sh"
	    echo "final_success_gate=record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS"
	    echo "bag_path=$bag_abs"
	  } > "$reports_dir/field_acceptance_collection_plan.txt"
	fi

echo "$out_abs"
