#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: actual_bag_replay.sh --bag PATH [options]

Options:
  --out DIR              Evidence output directory.
  --duration SEC         Replay duration in seconds. Default: 30.
  --start SEC            rosbag play start offset in seconds. Default: 0.
  --rate RATE            rosbag play rate. Default: 1.0.
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
                         velocity reference topic. No default velocity topic is
                         inspected or played.
  --execute              Run the generated replay command after inspection.
  --skip-bag-inspect     Skip rosbag inspection. Intended for dry-run tests only.
  -h, --help             Show this help.

Tunnel.bag policy:
  /novatel_data/inspvax is used only for initial velocity audit. It is not
  played into the SLAM graph and is not used as continuous trajectory truth.
EOF
}

die() {
  echo "actual_bag_replay: $*" >&2
  exit 1
}

is_nonnegative_number() {
  python3 - "$1" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
except Exception:
    sys.exit(1)
sys.exit(0 if math.isfinite(value) and value >= 0.0 else 1)
PY
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

ceil_replay_seconds_plus() {
  python3 - "$1" "$2" "$3" <<'PY'
import math
import sys
duration = float(sys.argv[1])
rate = float(sys.argv[2])
extra = int(sys.argv[3])
print(int(math.ceil(duration / rate)) + extra)
PY
}

write_executable() {
  local path="$1"
  chmod +x "$path"
}

bag_path=""
out_dir=""
duration_s="30"
start_s="0"
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
execute_replay=0
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
    --duration)
      [[ $# -ge 2 ]] || die "--duration requires a value"
      duration_s="$2"
      shift 2
      ;;
    --start)
      [[ $# -ge 2 ]] || die "--start requires a value"
      start_s="$2"
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
      execute_replay=1
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
is_positive_number "$duration_s" || die "--duration must be a positive finite number"
is_nonnegative_number "$start_s" || die "--start must be a non-negative finite number"
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

local_odometry_config_report="launch_default"
local_odometry_config_arg=""
if [[ -n "$local_odometry_config" ]]; then
  local_odometry_config_abs="$(absolute_path "$local_odometry_config")"
  [[ -f "$local_odometry_config_abs" ]] || die "local odometry config does not exist: $local_odometry_config_abs"
  local_odometry_config_report="$local_odometry_config_abs"
  local_odometry_config_arg=" local_odometry_config:=$(shell_quote "$local_odometry_config_abs")"
fi

if [[ -z "$out_dir" ]]; then
  out_dir="actual_bag_replay_$(date -u +%Y%m%dT%H%M%SZ)"
fi
out_abs="$(absolute_path "$out_dir")"
reports_dir="$out_abs/reports"
commands_dir="$out_abs/commands"
logs_dir="$out_abs/logs"
mkdir -p "$reports_dir" "$commands_dir" "$logs_dir"

session_id="$(basename "$out_abs" | tr -c 'A-Za-z0-9_-' '_')"
capture_timeout_s="$(ceil_replay_seconds_plus "$duration_s" "$rate" 30)"
minimum_fusion_published="$(python3 - "$duration_s" <<'PY'
import math
import sys

duration = float(sys.argv[1])
print(max(1, int(math.floor(duration * 5.0))))
PY
)"
minimum_local_odometry_published="$(python3 - "$duration_s" <<'PY'
import math
import sys

duration = float(sys.argv[1])
print(max(1, int(math.floor(duration * 1.0))))
PY
)"
actual_bag_test_scope="INITIAL_LIDAR_IMU_ONLY"
actual_bag_scenario="ACTUAL_TUNNEL_LIDAR_IMU_INITIAL"
bag_sensor_set="LIDAR_IMU_ONLY"
plc_feedback_status="NOT_PRESENT_NA"
plc_feedback_gate_status="NA_INITIAL_TEST"
machine_motion_assumption="CONTINUOUS_MOTION"
vibration_profile="NORMAL"
field_acceptance_requires_plc_feedback="YES"
actual_bag_metrics_report_rel="reports/actual_bag_replay_metrics_report.txt"
actual_bag_event_file_rel="reports/actual_bag_replay_events.txt"
plc_absent_launch_args=" start_machine_state:=false start_mapping_control:=false start_section_manager:=false"
bag_quoted="$(shell_quote "$bag_abs")"
out_quoted="$(shell_quote "$out_abs")"
session_root_abs="$out_abs/session_state"
session_root_quoted="$(shell_quote "$session_root_abs")"
canonical_center_topic="/velodyne_points"
canonical_left_topic="/left/lslidar_point_cloud"
canonical_right_topic="/right/velodyne_points"
canonical_imu_topic="/imu/data"
canonical_time_reference_topic="/time_reference"
reported_time_reference_topic="$time_reference_topic"
reported_canonical_time_reference_topic="$canonical_time_reference_topic"
time_reference_status="PRESENT_REQUIRED"
time_sync_evidence_status="INITIAL_TIME_STATUS_CAPTURE_REQUIRED"
reported_initial_velocity_topic="$initial_velocity_topic"
initial_velocity_reference_required="YES"
initial_velocity_reference_policy="START_ONLY_AUDIT"
initial_velocity_reference_status="PENDING_CAPTURE"
play_topics_csv="$center_topic,$left_topic,$right_topic,$imu_topic"
play_topic_args="$(shell_quote "$center_topic") $(shell_quote "$left_topic") $(shell_quote "$right_topic") $(shell_quote "$imu_topic")"
topic_remap_args=""

append_topic_remap() {
  local source_topic="$1"
  local target_topic="$2"
  if [[ "$source_topic" != "$target_topic" ]]; then
    topic_remap_args+=" $(shell_quote "${source_topic}:=${target_topic}")"
  fi
}

append_topic_remap "$center_topic" "$canonical_center_topic"
append_topic_remap "$left_topic" "$canonical_left_topic"
append_topic_remap "$right_topic" "$canonical_right_topic"
append_topic_remap "$imu_topic" "$canonical_imu_topic"
if [[ "$use_time_reference" -eq 1 ]]; then
  play_topics_csv+=",$time_reference_topic"
  play_topic_args+=" $(shell_quote "$time_reference_topic")"
  append_topic_remap "$time_reference_topic" "$canonical_time_reference_topic"
else
  reported_time_reference_topic="NONE"
  reported_canonical_time_reference_topic="NONE"
  time_reference_status="NOT_PRESENT_INITIAL_TEST"
  time_sync_evidence_status="NOT_PRESENT_INITIAL_TEST"
fi
if [[ "$use_initial_velocity_reference" -eq 0 ]]; then
  reported_initial_velocity_topic="NONE"
  initial_velocity_reference_required="NO"
  initial_velocity_reference_policy="NOT_AVAILABLE_INITIAL_TEST"
  initial_velocity_reference_status="NOT_PRESENT_INITIAL_TEST"
fi

inspection_report="$reports_dir/actual_bag_inspection.txt"
initial_velocity_report="$reports_dir/initial_velocity_reference.txt"

if [[ "$skip_bag_inspect" -eq 1 ]]; then
  {
    echo "actual_bag_status=SKIPPED"
    echo "bag_path=$bag_abs"
    echo "skip_reason=skip_bag_inspect"
    echo "actual_bag_test_scope=$actual_bag_test_scope"
    echo "bag_sensor_set=$bag_sensor_set"
    echo "plc_feedback_status=$plc_feedback_status"
    echo "plc_feedback_gate_status=$plc_feedback_gate_status"
    echo "machine_motion_assumption=$machine_motion_assumption"
    echo "vibration_profile=$vibration_profile"
    echo "field_acceptance_requires_plc_feedback=$field_acceptance_requires_plc_feedback"
    echo "center_lidar_topic=$center_topic"
    echo "left_lidar_topic=$left_topic"
    echo "right_lidar_topic=$right_topic"
    echo "imu_topic=$imu_topic"
    echo "time_reference_topic=$reported_time_reference_topic"
    echo "time_reference_status=$time_reference_status"
    echo "time_sync_evidence_status=$time_sync_evidence_status"
    echo "canonical_center_lidar_topic=$canonical_center_topic"
    echo "canonical_left_lidar_topic=$canonical_left_topic"
    echo "canonical_right_lidar_topic=$canonical_right_topic"
    echo "canonical_imu_topic=$canonical_imu_topic"
    echo "canonical_time_reference_topic=$reported_canonical_time_reference_topic"
    echo "initial_velocity_reference_status=$initial_velocity_reference_status"
    echo "initial_velocity_reference_required=$initial_velocity_reference_required"
    echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
    echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
    echo "continuous_velocity_reference_used=NO"
  } > "$inspection_report"
  {
    echo "initial_velocity_reference_status=$initial_velocity_reference_status"
    echo "initial_velocity_reference_required=$initial_velocity_reference_required"
    echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
    echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
    echo "continuous_velocity_reference_used=NO"
    echo "velocity_reference_played_to_slam=NO"
  } > "$initial_velocity_report"
else
  python3 - "$bag_abs" "$inspection_report" "$initial_velocity_report" \
    "$center_topic" "$left_topic" "$right_topic" "$imu_topic" \
    "$time_reference_topic" "$initial_velocity_topic" "$use_time_reference" \
    "$use_initial_velocity_reference" <<'PY'
import math
import sys

import rosbag

(
    bag_path,
    inspection_path,
    initial_path,
    center_topic,
    left_topic,
    right_topic,
    imu_topic,
    time_reference_topic,
    initial_velocity_topic,
    use_time_reference,
    use_initial_velocity_reference,
) = sys.argv[1:12]
time_reference_enabled = use_time_reference == "1"
initial_velocity_enabled = use_initial_velocity_reference == "1"
cloud_topics = [center_topic, left_topic, right_topic]
required_topics = cloud_topics + [imu_topic]
if time_reference_enabled:
    required_topics.append(time_reference_topic)
plc_feedback_topics = [
    "/plc/left_track_speed",
    "/plc/right_track_speed",
    "/plc/cutting_on",
    "/machine/state",
]
full_fields = {"x", "y", "z", "intensity", "ring", "time"}
xyzi_fields = {"x", "y", "z", "intensity"}
status = "PASS"
issues = []
topic_counts = {}
field_names = {}
frame_ids = {}
initial_velocity = None

try:
    bag = rosbag.Bag(bag_path)
except Exception as exc:
    with open(inspection_path, "w", encoding="utf-8") as output:
        output.write("actual_bag_status=FAIL\n")
        output.write("bag_open_error=%s\n" % str(exc).replace("\n", " "))
    with open(initial_path, "w", encoding="utf-8") as output:
        output.write("initial_velocity_reference_status=FAIL\n")
        output.write("initial_velocity_reference_policy=START_ONLY_AUDIT\n")
        output.write("continuous_velocity_reference_used=NO\n")
    sys.exit(1)

try:
    topic_info = bag.get_type_and_topic_info().topics
    for topic, info in topic_info.items():
        topic_counts[topic] = info.message_count

    for topic in required_topics:
        if topic_counts.get(topic, 0) <= 0:
            status = "FAIL"
            issues.append("missing_or_empty_topic:%s" % topic)

    sample_topics = list(required_topics)
    if initial_velocity_enabled:
        sample_topics.append(initial_velocity_topic)
    for topic, msg, stamp in bag.read_messages(topics=sample_topics):
        if topic in cloud_topics and topic not in field_names:
            field_names[topic] = [field.name for field in msg.fields]
            frame_ids[topic] = getattr(msg.header, "frame_id", "")
        elif topic == imu_topic and topic not in frame_ids:
            frame_ids[topic] = getattr(msg.header, "frame_id", "")
        elif time_reference_enabled and topic == time_reference_topic and topic not in frame_ids:
            frame_ids[topic] = getattr(msg.header, "frame_id", "")
        elif initial_velocity_enabled and topic == initial_velocity_topic and initial_velocity is None:
            north = float(getattr(msg, "north_velocity", 0.0))
            east = float(getattr(msg, "east_velocity", 0.0))
            up = float(getattr(msg, "up_velocity", 0.0))
            initial_velocity = {
                "stamp": stamp.to_sec(),
                "north": north,
                "east": east,
                "up": up,
                "speed": math.sqrt(north * north + east * east + up * up),
            }
        time_reference_done = (not time_reference_enabled) or time_reference_topic in frame_ids
        initial_velocity_done = (not initial_velocity_enabled) or initial_velocity is not None
        if all(topic in field_names for topic in cloud_topics) and imu_topic in frame_ids and time_reference_done and initial_velocity_done:
            break

    for topic in (center_topic, right_topic):
        fields = set(field_names.get(topic, []))
        if not full_fields.issubset(fields):
            status = "FAIL"
            issues.append("missing_full_fields:%s" % topic)

    left_fields = set(field_names.get(left_topic, []))
    left_legacy = xyzi_fields.issubset(left_fields) and not full_fields.issubset(left_fields)
    if not xyzi_fields.issubset(left_fields):
        status = "FAIL"
        issues.append("missing_xyzi_fields:%s" % left_topic)

    with open(inspection_path, "w", encoding="utf-8") as output:
        output.write("actual_bag_status=%s\n" % status)
        output.write("bag_path=%s\n" % bag_path)
        output.write("bag_start_s=%.6f\n" % bag.get_start_time())
        output.write("bag_end_s=%.6f\n" % bag.get_end_time())
        output.write("bag_duration_s=%.6f\n" % (bag.get_end_time() - bag.get_start_time()))
        output.write("actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n")
        output.write("bag_sensor_set=LIDAR_IMU_ONLY\n")
        output.write("plc_feedback_status=NOT_PRESENT_NA\n")
        output.write("plc_feedback_gate_status=NA_INITIAL_TEST\n")
        output.write("machine_motion_assumption=CONTINUOUS_MOTION\n")
        output.write("vibration_profile=NORMAL\n")
        output.write("field_acceptance_requires_plc_feedback=YES\n")
        output.write("center_lidar_topic=%s\n" % center_topic)
        output.write("left_lidar_topic=%s\n" % left_topic)
        output.write("right_lidar_topic=%s\n" % right_topic)
        output.write("imu_topic=%s\n" % imu_topic)
        output.write("time_reference_topic=%s\n" % (time_reference_topic if time_reference_enabled else "NONE"))
        output.write("time_reference_status=%s\n" % ("PRESENT_REQUIRED" if time_reference_enabled else "NOT_PRESENT_INITIAL_TEST"))
        output.write("time_sync_evidence_status=%s\n" % ("INITIAL_TIME_STATUS_CAPTURE_REQUIRED" if time_reference_enabled else "NOT_PRESENT_INITIAL_TEST"))
        output.write("canonical_center_lidar_topic=/velodyne_points\n")
        output.write("canonical_left_lidar_topic=/left/lslidar_point_cloud\n")
        output.write("canonical_right_lidar_topic=/right/velodyne_points\n")
        output.write("canonical_imu_topic=/imu/data\n")
        output.write("canonical_time_reference_topic=%s\n" % ("/time_reference" if time_reference_enabled else "NONE"))
        for topic in required_topics:
            output.write("topic_count[%s]=%s\n" % (topic, topic_counts.get(topic, 0)))
        for topic in plc_feedback_topics:
            output.write("topic_count[%s]=%s\n" % (topic, topic_counts.get(topic, 0)))
        for topic in cloud_topics:
            output.write("fields[%s]=%s\n" % (topic, ",".join(field_names.get(topic, []))))
            output.write("frame_id[%s]=%s\n" % (topic, frame_ids.get(topic, "")))
        output.write("frame_id[%s]=%s\n" % (imu_topic, frame_ids.get(imu_topic, "")))
        if time_reference_enabled:
            output.write("frame_id[%s]=%s\n" % (time_reference_topic, frame_ids.get(time_reference_topic, "")))
        output.write("left_lidar_legacy_xyzi_defaults_required=%s\n" % ("YES" if left_legacy else "NO"))
        output.write("replay_profile=bringup_replay_tunnel_bag.launch\n")
        output.write("initial_velocity_reference_status=%s\n" % ("PENDING_CAPTURE" if initial_velocity_enabled else "NOT_PRESENT_INITIAL_TEST"))
        output.write("initial_velocity_reference_required=%s\n" % ("YES" if initial_velocity_enabled else "NO"))
        output.write("initial_velocity_reference_policy=%s\n" % ("START_ONLY_AUDIT" if initial_velocity_enabled else "NOT_AVAILABLE_INITIAL_TEST"))
        output.write("initial_velocity_reference_topic=%s\n" % (initial_velocity_topic if initial_velocity_enabled else "NONE"))
        output.write("continuous_velocity_reference_used=NO\n")
        output.write("velocity_reference_played_to_slam=NO\n")
        if issues:
            output.write("issues=%s\n" % ",".join(issues))

    with open(initial_path, "w", encoding="utf-8") as output:
        if not initial_velocity_enabled:
            output.write("initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST\n")
        elif initial_velocity is None:
            output.write("initial_velocity_reference_status=MISSING\n")
        else:
            output.write("initial_velocity_reference_status=CAPTURED\n")
            output.write("initial_velocity_first_sample_stamp_s=%.6f\n" % initial_velocity["stamp"])
            output.write("initial_velocity_north_mps=%.6f\n" % initial_velocity["north"])
            output.write("initial_velocity_east_mps=%.6f\n" % initial_velocity["east"])
            output.write("initial_velocity_up_mps=%.6f\n" % initial_velocity["up"])
            output.write("initial_velocity_speed_mps=%.6f\n" % initial_velocity["speed"])
        output.write("initial_velocity_reference_required=%s\n" % ("YES" if initial_velocity_enabled else "NO"))
        output.write("initial_velocity_reference_policy=%s\n" % ("START_ONLY_AUDIT" if initial_velocity_enabled else "NOT_AVAILABLE_INITIAL_TEST"))
        output.write("initial_velocity_reference_topic=%s\n" % (initial_velocity_topic if initial_velocity_enabled else "NONE"))
        output.write("continuous_velocity_reference_used=NO\n")
        output.write("velocity_reference_played_to_slam=NO\n")
finally:
    bag.close()

sys.exit(0 if status == "PASS" else 2)
PY
fi

cat > "$commands_dir/launch_pipeline.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
if [[ -f /home/bai/Desktop/Tunnel-LIO/catkin_ws/devel/setup.bash ]]; then
  source /home/bai/Desktop/Tunnel-LIO/catkin_ws/devel/setup.bash
fi
mkdir -p $session_root_quoted
roslaunch mine_slam_bringup bringup_replay_tunnel_bag.launch section_session_id:=$session_id session_root:=$session_root_quoted$plc_absent_launch_args$local_odometry_config_arg
EOF
write_executable "$commands_dir/launch_pipeline.sh"

cat > "$commands_dir/play_selected_topics.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
rosbag play --clock --start $start_s --duration $duration_s --rate $rate $bag_quoted$topic_remap_args --topics $play_topic_args
EOF
write_executable "$commands_dir/play_selected_topics.sh"

actual_bag_event_file_quoted="$(shell_quote "$reports_dir/actual_bag_replay_events.txt")"
actual_bag_hil_report_quoted="$(shell_quote "$reports_dir/actual_bag_replay_hil_validation_report.txt")"
cat > "$commands_dir/validate_actual_bag_events.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
if [[ -f /home/bai/Desktop/Tunnel-LIO/catkin_ws/devel/setup.bash ]]; then
  source /home/bai/Desktop/Tunnel-LIO/catkin_ws/devel/setup.bash
fi
roslaunch lio_eval_tools validation_report.launch \\
  event_file:=$actual_bag_event_file_quoted \\
  report_file:=$actual_bag_hil_report_quoted
EOF
write_executable "$commands_dir/validate_actual_bag_events.sh"

cat > "$commands_dir/run_replay.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
if [[ -f /home/bai/Desktop/Tunnel-LIO/catkin_ws/devel/setup.bash ]]; then
  source /home/bai/Desktop/Tunnel-LIO/catkin_ws/devel/setup.bash
fi

out_dir=$out_quoted
reports_dir="\$out_dir/reports"
logs_dir="\$out_dir/logs"
commands_dir="\$out_dir/commands"
mkdir -p "\$reports_dir" "\$logs_dir"
capture_timeout_s=$capture_timeout_s
actual_bag_session_id=$session_id
actual_bag_scenario=$actual_bag_scenario

choose_free_port() {
  python3 - <<'PY'
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(("127.0.0.1", 0))
print(sock.getsockname()[1])
sock.close()
PY
}

roscore_pid=""
roscore_pgid=""
pipeline_pid=""
pipeline_pgid=""
cleanup() {
  if [[ -n "\$pipeline_pgid" ]]; then
    kill -- "-\$pipeline_pgid" 2>/dev/null || true
    sleep 1
    kill -9 -- "-\$pipeline_pgid" 2>/dev/null || true
  elif [[ -n "\$pipeline_pid" ]] && kill -0 "\$pipeline_pid" 2>/dev/null; then
    kill "\$pipeline_pid" 2>/dev/null || true
  fi
  if [[ -n "\$pipeline_pid" ]]; then
    wait "\$pipeline_pid" 2>/dev/null || true
  fi
  if [[ -n "\$roscore_pgid" ]]; then
    kill -- "-\$roscore_pgid" 2>/dev/null || true
    sleep 1
    kill -9 -- "-\$roscore_pgid" 2>/dev/null || true
  fi
  if [[ -n "\$roscore_pid" ]]; then
    wait "\$roscore_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros_master_port="\$(choose_free_port)"
export ROS_MASTER_URI="http://127.0.0.1:\$ros_master_port"
export ROS_HOSTNAME="127.0.0.1"
setsid roscore -p "\$ros_master_port" > "\$logs_dir/roscore.log" 2>&1 &
roscore_pid=\$!
roscore_pgid="\$roscore_pid"
for _ in {1..20}; do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
rosparam list >/dev/null 2>&1

setsid "\$commands_dir/launch_pipeline.sh" > "\$logs_dir/pipeline.log" 2>&1 &
pipeline_pid=\$!
pipeline_pgid="\$pipeline_pid"
sleep 6

timeout \$capture_timeout_s rostopic echo /diagnostics/lidar_fusion > "\$reports_dir/lidar_fusion_diag.txt" 2> "\$logs_dir/lidar_fusion_diag.err" &
fusion_capture_pid=\$!
timeout \$capture_timeout_s rostopic echo /diagnostics/lio_preprocess > "\$reports_dir/lio_preprocess_diag.txt" 2> "\$logs_dir/lio_preprocess_diag.err" &
preprocess_capture_pid=\$!
timeout \$capture_timeout_s rostopic echo /diagnostics/lio_local_odometry > "\$reports_dir/lio_local_odometry_diag.txt" 2> "\$logs_dir/lio_local_odometry_diag.err" &
local_odom_diag_capture_pid=\$!
timeout \$capture_timeout_s rostopic echo --noarr -n 1 /points_raw > "\$reports_dir/points_raw.txt" 2> "\$logs_dir/points_raw.err" &
points_raw_capture_pid=\$!
timeout \$capture_timeout_s rostopic echo --noarr -n 1 /lio/points_deskewed > "\$reports_dir/points_deskewed.txt" 2> "\$logs_dir/points_deskewed.err" &
points_deskewed_capture_pid=\$!
timeout \$capture_timeout_s rostopic echo -n 1 /lio/odom_local > "\$reports_dir/odom_local.txt" 2> "\$logs_dir/odom_local.err" &
odom_capture_pid=\$!
timeout \$capture_timeout_s rostopic echo /time/status > "\$reports_dir/time_status.txt" 2> "\$logs_dir/time_status.err" &
time_capture_pid=\$!

"\$commands_dir/play_selected_topics.sh" > "\$logs_dir/rosbag_play.log" 2>&1
sleep 2

wait "\$fusion_capture_pid" 2>/dev/null || true
wait "\$preprocess_capture_pid" 2>/dev/null || true
wait "\$local_odom_diag_capture_pid" 2>/dev/null || true
wait "\$points_raw_capture_pid" 2>/dev/null || true
wait "\$points_deskewed_capture_pid" 2>/dev/null || true
wait "\$odom_capture_pid" 2>/dev/null || true
wait "\$time_capture_pid" 2>/dev/null || true

cleanup
trap - EXIT

ros_master_cleanup_status="PASS"
if pgrep -af "ros(master|core).*(-p \$ros_master_port|--core -p \$ros_master_port)" >/dev/null 2>&1; then
  ros_master_cleanup_status="FAIL"
fi

check_file() {
  local path="\$1"
  if [[ -s "\$path" ]]; then
    echo "PASS"
  else
    echo "FAIL"
  fi
}

max_key_value() {
  local key="\$1"
  local path="\$2"
  sed -n "s/.*\${key}=\([0-9][0-9]*\).*/\1/p" "\$path" | awk 'BEGIN { max = 0 } { if (\$1 > max) max = \$1 } END { print max }'
}

diagnostic_key_value_max() {
  local key="\$1"
  local path="\$2"
  python3 - "\$key" "\$path" <<'PY'
import re
import sys

target, path = sys.argv[1:3]
same_line = re.compile(
    r"(?:^|[^A-Za-z0-9_])" + re.escape(target) + r"=([0-9]+)\b"
)
key_line = re.compile(r'^\s*key:\s*"?([^"]+)"?\s*$')
value_line = re.compile(r'^\s*value:\s*"?([^"]+)"?\s*$')
maximum = 0
pending_key = None

try:
    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            match = same_line.search(line)
            if match:
                maximum = max(maximum, int(match.group(1)))

            key_match = key_line.match(line)
            if key_match:
                pending_key = key_match.group(1)
                continue

            value_match = value_line.match(line)
            if value_match and pending_key == target:
                value = value_match.group(1).strip()
                if re.fullmatch(r"[0-9]+", value):
                    maximum = max(maximum, int(value))
                pending_key = None
except OSError:
    pass

print(maximum)
PY
}

pointcloud_width() {
  local path="\$1"
  sed -n 's/^[[:space:]]*width:[[:space:]]*\([0-9][0-9]*\).*/\1/p' "\$path" | head -n 1
}

positive_status() {
  local value="\$1"
  if [[ "\$value" =~ ^[0-9]+$ ]] && [[ "\$value" -gt 0 ]]; then
    echo "PASS"
  else
    echo "FAIL"
  fi
}

at_least_status() {
  local value="\$1"
  local minimum="\$2"
  if [[ "\$value" =~ ^[0-9]+$ ]] && [[ "\$minimum" =~ ^[0-9]+$ ]] && [[ "\$value" -ge "\$minimum" ]]; then
    echo "PASS"
  else
    echo "FAIL"
  fi
}

pipeline_error_status="PASS"
if rg -n 'invalid session manifest|terminate called|\\[FATAL\\]|\\[ERROR\\]' "\$logs_dir/pipeline.log" >/dev/null 2>&1; then
  pipeline_error_status="FAIL"
fi

fusion_status="\$(check_file "\$reports_dir/lidar_fusion_diag.txt")"
preprocess_status="\$(check_file "\$reports_dir/lio_preprocess_diag.txt")"
local_odom_diag_status="\$(check_file "\$reports_dir/lio_local_odometry_diag.txt")"
points_raw_status="\$(check_file "\$reports_dir/points_raw.txt")"
deskewed_status="\$(check_file "\$reports_dir/points_deskewed.txt")"
odom_status="\$(check_file "\$reports_dir/odom_local.txt")"
time_status="\$(check_file "\$reports_dir/time_status.txt")"
fusion_callbacks="\$(diagnostic_key_value_max callbacks "\$reports_dir/lidar_fusion_diag.txt")"
fusion_published="\$(diagnostic_key_value_max published "\$reports_dir/lidar_fusion_diag.txt")"
legacy_clouds="\$(diagnostic_key_value_max legacy_xyzi_clouds "\$reports_dir/lidar_fusion_diag.txt")"
legacy_points="\$(diagnostic_key_value_max legacy_xyzi_points "\$reports_dir/lidar_fusion_diag.txt")"
local_odometry_clouds="\$(diagnostic_key_value_max cloud_count "\$reports_dir/lio_local_odometry_diag.txt")"
local_odometry_published="\$(diagnostic_key_value_max published_odometry "\$reports_dir/lio_local_odometry_diag.txt")"
local_odometry_rejected="\$(diagnostic_key_value_max rejected_registrations "\$reports_dir/lio_local_odometry_diag.txt")"
local_odometry_keyframe_reseeds="\$(diagnostic_key_value_max keyframe_reseeds "\$reports_dir/lio_local_odometry_diag.txt")"
points_raw_width="\$(pointcloud_width "\$reports_dir/points_raw.txt")"
deskewed_width="\$(pointcloud_width "\$reports_dir/points_deskewed.txt")"
fusion_published_status="\$(positive_status "\$fusion_published")"
local_odometry_published_status="\$(positive_status "\$local_odometry_published")"
minimum_fusion_published=$minimum_fusion_published
minimum_local_odometry_published=$minimum_local_odometry_published
fusion_duration_coverage_status="\$(at_least_status "\$fusion_published" "\$minimum_fusion_published")"
local_odometry_duration_coverage_status="\$(at_least_status "\$local_odometry_published" "\$minimum_local_odometry_published")"
legacy_clouds_status="\$(positive_status "\$legacy_clouds")"
legacy_points_status="\$(positive_status "\$legacy_points")"
points_raw_width_status="\$(positive_status "\$points_raw_width")"
deskewed_width_status="\$(positive_status "\$deskewed_width")"
fusion_callbacks="\${fusion_callbacks:-0}"
fusion_published="\${fusion_published:-0}"
legacy_clouds="\${legacy_clouds:-0}"
legacy_points="\${legacy_points:-0}"
local_odometry_clouds="\${local_odometry_clouds:-0}"
local_odometry_published="\${local_odometry_published:-0}"
local_odometry_rejected="\${local_odometry_rejected:-0}"
local_odometry_keyframe_reseeds="\${local_odometry_keyframe_reseeds:-0}"
points_raw_width="\${points_raw_width:-0}"
deskewed_width="\${deskewed_width:-0}"

overall="FAIL"
if [[ "\$fusion_status" == "PASS" && "\$preprocess_status" == "PASS" && "\$local_odom_diag_status" == "PASS" && "\$points_raw_status" == "PASS" && "\$deskewed_status" == "PASS" && "\$odom_status" == "PASS" && "\$time_status" == "PASS" && "\$fusion_published_status" == "PASS" && "\$local_odometry_published_status" == "PASS" && "\$fusion_duration_coverage_status" == "PASS" && "\$local_odometry_duration_coverage_status" == "PASS" && "\$legacy_clouds_status" == "PASS" && "\$legacy_points_status" == "PASS" && "\$points_raw_width_status" == "PASS" && "\$deskewed_width_status" == "PASS" && "\$pipeline_error_status" == "PASS" && "\$ros_master_cleanup_status" == "PASS" ]]; then
  overall="PASS"
fi

{
  echo "actual_bag_replay_status=\$overall"
  echo "bag_path=$bag_abs"
  echo "duration_s=$duration_s"
  echo "start_s=$start_s"
  echo "rate=$rate"
  echo "actual_bag_test_scope=$actual_bag_test_scope"
  echo "bag_sensor_set=$bag_sensor_set"
  echo "plc_feedback_status=$plc_feedback_status"
  echo "plc_feedback_gate_status=$plc_feedback_gate_status"
  echo "machine_motion_assumption=$machine_motion_assumption"
  echo "vibration_profile=$vibration_profile"
  echo "field_acceptance_requires_plc_feedback=$field_acceptance_requires_plc_feedback"
  echo "local_odometry_config=$local_odometry_config_report"
  echo "center_lidar_topic=$center_topic"
  echo "left_lidar_topic=$left_topic"
  echo "right_lidar_topic=$right_topic"
  echo "imu_topic=$imu_topic"
  echo "time_reference_topic=$reported_time_reference_topic"
  echo "time_reference_status=$time_reference_status"
  echo "time_sync_evidence_status=$time_sync_evidence_status"
  echo "canonical_center_lidar_topic=$canonical_center_topic"
  echo "canonical_left_lidar_topic=$canonical_left_topic"
  echo "canonical_right_lidar_topic=$canonical_right_topic"
  echo "canonical_imu_topic=$canonical_imu_topic"
  echo "canonical_time_reference_topic=$reported_canonical_time_reference_topic"
  echo "play_topics=$play_topics_csv"
  echo "excluded_velocity_reference_topic=$reported_initial_velocity_topic"
  echo "initial_velocity_reference_status=$initial_velocity_reference_status"
  echo "initial_velocity_reference_required=$initial_velocity_reference_required"
  echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
  echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
  echo "ros_master_uri=\$ROS_MASTER_URI"
  echo "ros_master_cleanup_status=\$ros_master_cleanup_status"
  echo "lidar_fusion_diag_captured=\$fusion_status"
  echo "lio_preprocess_diag_captured=\$preprocess_status"
  echo "lio_local_odometry_diag_captured=\$local_odom_diag_status"
  echo "points_raw_captured=\$points_raw_status"
  echo "deskewed_cloud_captured=\$deskewed_status"
  echo "odom_local_captured=\$odom_status"
  echo "time_status_captured=\$time_status"
  echo "fusion_callbacks=\$fusion_callbacks"
  echo "fusion_published=\$fusion_published"
  echo "fusion_published_status=\$fusion_published_status"
  echo "local_odometry_clouds=\$local_odometry_clouds"
  echo "local_odometry_published=\$local_odometry_published"
  echo "local_odometry_published_status=\$local_odometry_published_status"
  echo "local_odometry_rejected_registrations=\$local_odometry_rejected"
  echo "local_odometry_keyframe_reseeds=\$local_odometry_keyframe_reseeds"
  echo "minimum_fusion_published=\$minimum_fusion_published"
  echo "minimum_local_odometry_published=\$minimum_local_odometry_published"
  echo "fusion_duration_coverage_status=\$fusion_duration_coverage_status"
  echo "local_odometry_duration_coverage_status=\$local_odometry_duration_coverage_status"
  echo "legacy_xyzi_clouds=\$legacy_clouds"
  echo "legacy_xyzi_points=\$legacy_points"
  echo "legacy_xyzi_clouds_status=\$legacy_clouds_status"
  echo "legacy_xyzi_points_status=\$legacy_points_status"
  echo "points_raw_width=\$points_raw_width"
  echo "deskewed_cloud_width=\$deskewed_width"
  echo "points_raw_width_status=\$points_raw_width_status"
  echo "deskewed_cloud_width_status=\$deskewed_width_status"
  echo "pipeline_error_status=\$pipeline_error_status"
  echo "velocity_reference_played_to_slam=NO"
  echo "continuous_velocity_reference_used=NO"
} > "\$reports_dir/actual_bag_replay_summary.txt"

actual_bag_failed_records=1
actual_bag_failed_checks=1
actual_bag_event_queue_backlog=-1
if [[ "\$overall" == "PASS" ]]; then
  actual_bag_failed_records=0
  actual_bag_failed_checks=0
  actual_bag_event_queue_backlog=0
fi

{
  echo "overall=\$overall;total_records=1;failed_records=\$actual_bag_failed_records"
  echo "session=\$actual_bag_session_id;scenario=\$actual_bag_scenario;status=\$overall;failed_checks=\$actual_bag_failed_checks;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=$actual_bag_test_scope"
  echo "---"
  echo "detail=actual_bag_replay_status;status=\$overall;value=\$overall;threshold=PASS"
  echo "detail=fusion_duration_coverage_status;status=\$fusion_duration_coverage_status;value=\$fusion_published;threshold=\$minimum_fusion_published"
  echo "detail=local_odometry_duration_coverage_status;status=\$local_odometry_duration_coverage_status;value=\$local_odometry_published;threshold=\$minimum_local_odometry_published"
  echo "detail=local_odometry_keyframe_reseeds;status=PASS;value=\$local_odometry_keyframe_reseeds;threshold=diagnostic_only"
  echo "detail=velocity_reference_played_to_slam;status=PASS;value=NO;threshold=NO"
  echo "detail=continuous_velocity_reference_used;status=PASS;value=NO;threshold=NO"
  echo "detail=plc_feedback_status;status=PASS;value=$plc_feedback_status;threshold=$plc_feedback_status"
  echo "detail=field_acceptance_eligible;status=PASS;value=NO;threshold=NO"
} > "\$reports_dir/actual_bag_replay_metrics_report.txt"

{
  echo "event=session_start;scenario=\$actual_bag_scenario;session_id=\$actual_bag_session_id;t=0.000;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=$actual_bag_test_scope"
  echo "event=actual_bag_replay;scenario=\$actual_bag_scenario;session_id=\$actual_bag_session_id;t=$duration_s;queue_backlog=\$actual_bag_event_queue_backlog;actual_bag_replay_status=\$overall;fusion_published=\$fusion_published;minimum_fusion_published=\$minimum_fusion_published;local_odometry_published=\$local_odometry_published;minimum_local_odometry_published=\$minimum_local_odometry_published;local_odometry_rejected_registrations=\$local_odometry_rejected;local_odometry_keyframe_reseeds=\$local_odometry_keyframe_reseeds;plc_feedback_status=$plc_feedback_status;field_acceptance_eligible=NO;velocity_reference_played_to_slam=NO;continuous_velocity_reference_used=NO"
} > "\$reports_dir/actual_bag_replay_events.txt"

[[ "\$overall" == "PASS" ]]
EOF
write_executable "$commands_dir/run_replay.sh"

{
  echo "actual_bag_replay_plan_status=READY"
  echo "bag_path=$bag_abs"
  echo "out_dir=$out_abs"
  echo "duration_s=$duration_s"
  echo "start_s=$start_s"
  echo "rate=$rate"
  echo "actual_bag_test_scope=$actual_bag_test_scope"
  echo "bag_sensor_set=$bag_sensor_set"
  echo "plc_feedback_status=$plc_feedback_status"
  echo "plc_feedback_gate_status=$plc_feedback_gate_status"
  echo "machine_motion_assumption=$machine_motion_assumption"
  echo "vibration_profile=$vibration_profile"
  echo "field_acceptance_requires_plc_feedback=$field_acceptance_requires_plc_feedback"
  echo "local_odometry_config=$local_odometry_config_report"
  echo "replay_profile=bringup_replay_tunnel_bag.launch"
  echo "center_lidar_topic=$center_topic"
  echo "left_lidar_topic=$left_topic"
  echo "right_lidar_topic=$right_topic"
  echo "imu_topic=$imu_topic"
  echo "time_reference_topic=$reported_time_reference_topic"
  echo "time_reference_status=$time_reference_status"
  echo "time_sync_evidence_status=$time_sync_evidence_status"
  echo "canonical_center_lidar_topic=$canonical_center_topic"
  echo "canonical_left_lidar_topic=$canonical_left_topic"
  echo "canonical_right_lidar_topic=$canonical_right_topic"
  echo "canonical_imu_topic=$canonical_imu_topic"
  echo "canonical_time_reference_topic=$reported_canonical_time_reference_topic"
  echo "play_topics=$play_topics_csv"
  echo "excluded_velocity_reference_topic=$reported_initial_velocity_topic"
  echo "initial_velocity_reference_status=$initial_velocity_reference_status"
  echo "initial_velocity_reference_required=$initial_velocity_reference_required"
  echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
  echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
  echo "minimum_fusion_published=$minimum_fusion_published"
  echo "minimum_local_odometry_published=$minimum_local_odometry_published"
  echo "continuous_velocity_reference_used=NO"
  echo "velocity_reference_played_to_slam=NO"
  echo "actual_bag_metrics_report=$actual_bag_metrics_report_rel"
  echo "actual_bag_event_file=$actual_bag_event_file_rel"
  echo "actual_bag_event_validation_command=$commands_dir/validate_actual_bag_events.sh"
  echo "run_command=$commands_dir/run_replay.sh"
} > "$reports_dir/actual_bag_replay_plan.txt"

if [[ "$execute_replay" -eq 1 ]]; then
  "$commands_dir/run_replay.sh"
else
  {
    echo "actual_bag_replay_status=DRY_RUN"
    echo "bag_path=$bag_abs"
    echo "run_command=$commands_dir/run_replay.sh"
    echo "actual_bag_test_scope=$actual_bag_test_scope"
    echo "bag_sensor_set=$bag_sensor_set"
    echo "plc_feedback_status=$plc_feedback_status"
    echo "plc_feedback_gate_status=$plc_feedback_gate_status"
    echo "machine_motion_assumption=$machine_motion_assumption"
    echo "vibration_profile=$vibration_profile"
    echo "field_acceptance_requires_plc_feedback=$field_acceptance_requires_plc_feedback"
    echo "center_lidar_topic=$center_topic"
    echo "left_lidar_topic=$left_topic"
    echo "right_lidar_topic=$right_topic"
    echo "imu_topic=$imu_topic"
    echo "time_reference_topic=$reported_time_reference_topic"
    echo "time_reference_status=$time_reference_status"
    echo "time_sync_evidence_status=$time_sync_evidence_status"
    echo "canonical_center_lidar_topic=$canonical_center_topic"
    echo "canonical_left_lidar_topic=$canonical_left_topic"
    echo "canonical_right_lidar_topic=$canonical_right_topic"
    echo "canonical_imu_topic=$canonical_imu_topic"
    echo "canonical_time_reference_topic=$reported_canonical_time_reference_topic"
    echo "play_topics=$play_topics_csv"
    echo "excluded_velocity_reference_topic=$reported_initial_velocity_topic"
    echo "initial_velocity_reference_status=$initial_velocity_reference_status"
    echo "initial_velocity_reference_required=$initial_velocity_reference_required"
    echo "initial_velocity_reference_policy=$initial_velocity_reference_policy"
    echo "initial_velocity_reference_topic=$reported_initial_velocity_topic"
    echo "velocity_reference_played_to_slam=NO"
    echo "continuous_velocity_reference_used=NO"
    echo "local_odometry_keyframe_reseeds=0"
    echo "actual_bag_metrics_report=$actual_bag_metrics_report_rel"
    echo "actual_bag_event_file=$actual_bag_event_file_rel"
  } > "$reports_dir/actual_bag_replay_summary.txt"

  {
    echo "overall=FAIL;total_records=1;failed_records=1"
    echo "session=$session_id;scenario=$actual_bag_scenario;status=DRY_RUN;failed_checks=1;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=$actual_bag_test_scope"
    echo "---"
    echo "detail=actual_bag_replay_status;status=DRY_RUN;value=DRY_RUN;threshold=PASS"
    echo "detail=field_acceptance_eligible;status=PASS;value=NO;threshold=NO"
  } > "$reports_dir/actual_bag_replay_metrics_report.txt"

  {
    echo "event=session_start;scenario=$actual_bag_scenario;session_id=$session_id;t=0.000;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;field_acceptance_eligible=NO;actual_bag_test_scope=$actual_bag_test_scope"
    echo "event=actual_bag_replay;scenario=$actual_bag_scenario;session_id=$session_id;t=$duration_s;queue_backlog=-1;actual_bag_replay_status=DRY_RUN;local_odometry_keyframe_reseeds=0;plc_feedback_status=$plc_feedback_status;field_acceptance_eligible=NO;velocity_reference_played_to_slam=NO;continuous_velocity_reference_used=NO"
  } > "$reports_dir/actual_bag_replay_events.txt"
fi

echo "$out_abs"
