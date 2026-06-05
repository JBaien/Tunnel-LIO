#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: actual_bag_profile.sh --bag PATH [options]

Options:
  --out DIR    Profile output directory.
  -h, --help   Show this help.

This script inspects a user-collected LiDAR+IMU bag, writes a topic profile,
and generates a recommended actual_bag_test_suite.sh command. It does not run
SLAM and does not create final field-acceptance evidence.
EOF
}

die() {
  echo "actual_bag_profile: $*" >&2
  exit 1
}

absolute_path() {
  python3 - "$1" <<'PY'
import os
import sys
print(os.path.abspath(sys.argv[1]))
PY
}

bag_path=""
out_dir=""

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
bag_abs="$(absolute_path "$bag_path")"
[[ -f "$bag_abs" ]] || die "bag does not exist: $bag_abs"

if [[ -z "$out_dir" ]]; then
  out_dir="actual_bag_profile_$(date -u +%Y%m%dT%H%M%SZ)"
fi
out_abs="$(absolute_path "$out_dir")"
reports_dir="$out_abs/reports"
commands_dir="$out_abs/commands"
logs_dir="$out_abs/logs"
mkdir -p "$reports_dir" "$commands_dir" "$logs_dir"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
suite_script="$script_dir/actual_bag_test_suite.sh"
[[ -f "$suite_script" ]] || die "actual_bag_test_suite.sh not found: $suite_script"

python3 - "$bag_abs" "$reports_dir/actual_bag_profile.txt" \
  "$commands_dir/run_recommended_actual_bag_test_suite.sh" \
  "$suite_script" "$out_abs/recommended_suite" <<'PY'
import math
import os
import shlex
import sys

import rosbag

bag_path, profile_path, command_path, suite_script, suite_out = sys.argv[1:6]
full_fields = {"x", "y", "z", "intensity", "ring", "time"}
xyzi_fields = {"x", "y", "z", "intensity"}
plc_topics = [
    "/plc/left_track_speed",
    "/plc/right_track_speed",
    "/plc/cutting_on",
    "/machine/state",
]


def topic_sort_key(item):
    topic, count = item
    return (-count, topic)


def name_has(topic, needle):
    return needle in topic.lower().replace("-", "_")


def first_by_score(candidates, scorer):
    if not candidates:
        return ""
    return sorted(candidates, key=lambda item: (-scorer(item), item[0]))[0][0]


def shell_join(args):
    return " ".join(shlex.quote(arg) for arg in args)


def fields_csv(fields):
    return ",".join(fields)


def nested_attr(obj, path):
    value = obj
    for part in path:
        value = getattr(value, part)
    return value


def vector_velocity(msg):
    candidates = [
        ("north_velocity", "east_velocity", "up_velocity"),
        ("north_vel", "east_vel", "up_vel"),
    ]
    for names in candidates:
        if all(hasattr(msg, name) for name in names):
            return tuple(float(getattr(msg, name)) for name in names)

    nested_candidates = [
        (("twist", "linear", "x"), ("twist", "linear", "y"), ("twist", "linear", "z")),
        (
            ("twist", "twist", "linear", "x"),
            ("twist", "twist", "linear", "y"),
            ("twist", "twist", "linear", "z"),
        ),
        (("vector", "x"), ("vector", "y"), ("vector", "z")),
    ]
    for paths in nested_candidates:
        try:
            return tuple(float(nested_attr(msg, path)) for path in paths)
        except AttributeError:
            continue

    if hasattr(msg, "data"):
        return float(getattr(msg, "data")), 0.0, 0.0

    return None


def first_velocity_sample(bag, topic):
    if not topic:
        return None
    for _, msg, stamp in bag.read_messages(topics=[topic]):
        velocity = vector_velocity(msg)
        if velocity is None:
            return None
        north, east, up = velocity
        speed = math.sqrt(north * north + east * east + up * up)
        values = (north, east, up, speed)
        if not all(math.isfinite(value) for value in values):
            return None
        return {
            "stamp": stamp.to_sec(),
            "north": north,
            "east": east,
            "up": up,
            "speed": speed,
        }
    return None


status = "PASS"
issues = []
topic_counts = {}
topic_types = {}
cloud_fields = {}
cloud_frames = {}
imu_topics = []
time_reference_topics = []
velocity_reference_topics = []

try:
    bag = rosbag.Bag(bag_path)
except Exception as exc:
    with open(profile_path, "w", encoding="utf-8") as output:
        output.write("actual_bag_profile_status=FAIL\n")
        output.write("bag_path=%s\n" % bag_path)
        output.write("bag_open_error=%s\n" % str(exc).replace("\n", " "))
    raise SystemExit(1)

try:
    info = bag.get_type_and_topic_info().topics
    for topic, topic_info in info.items():
        topic_counts[topic] = topic_info.message_count
        datatype = getattr(topic_info, "datatype", getattr(topic_info, "msg_type", ""))
        topic_types[topic] = datatype
        if datatype == "sensor_msgs/Imu":
            imu_topics.append(topic)
        if datatype == "sensor_msgs/TimeReference" or name_has(topic, "time_reference"):
            time_reference_topics.append(topic)
        if name_has(topic, "inspvax") or name_has(topic, "velocity"):
            velocity_reference_topics.append(topic)

    pointcloud_topics = [
        topic
        for topic, datatype in topic_types.items()
        if datatype == "sensor_msgs/PointCloud2"
    ]
    sample_topics = pointcloud_topics + imu_topics + time_reference_topics
    seen_clouds = set()
    for topic, msg, _ in bag.read_messages(topics=sample_topics):
        if topic in pointcloud_topics and topic not in seen_clouds:
            cloud_fields[topic] = [field.name for field in msg.fields]
            cloud_frames[topic] = getattr(msg.header, "frame_id", "")
            seen_clouds.add(topic)
        if len(seen_clouds) == len(pointcloud_topics):
            break

    full_clouds = [
        topic
        for topic in pointcloud_topics
        if full_fields.issubset(set(cloud_fields.get(topic, [])))
    ]
    xyzi_clouds = [
        topic
        for topic in pointcloud_topics
        if xyzi_fields.issubset(set(cloud_fields.get(topic, [])))
    ]

    center_topic = first_by_score(
        [(topic, topic_counts.get(topic, 0)) for topic in full_clouds],
        lambda item: (
            (80 if name_has(item[0], "center") else 0)
            + (60 if item[0] == "/velodyne_points" else 0)
            + (30 if name_has(item[0], "velodyne") else 0)
            - (100 if name_has(item[0], "left") or name_has(item[0], "right") else 0)
            + min(item[1], 1000) / 1000.0
        ),
    )
    left_topic = first_by_score(
        [(topic, topic_counts.get(topic, 0)) for topic in xyzi_clouds],
        lambda item: (
            (100 if name_has(item[0], "left") else 0)
            - (50 if item[0] == center_topic else 0)
            + min(item[1], 1000) / 1000.0
        ),
    )
    right_topic = first_by_score(
        [(topic, topic_counts.get(topic, 0)) for topic in full_clouds],
        lambda item: (
            (100 if name_has(item[0], "right") else 0)
            - (50 if item[0] == center_topic else 0)
            - (50 if item[0] == left_topic else 0)
            + min(item[1], 1000) / 1000.0
        ),
    )
    imu_topic = first_by_score(
        [(topic, topic_counts.get(topic, 0)) for topic in imu_topics],
        lambda item: (
            (100 if item[0] == "/imu/data" else 0)
            + (30 if name_has(item[0], "imu") else 0)
            + min(item[1], 1000) / 1000.0
        ),
    )

    chosen = {
        "center_lidar_topic": center_topic,
        "left_lidar_topic": left_topic,
        "right_lidar_topic": right_topic,
        "imu_topic": imu_topic,
    }
    for key, value in chosen.items():
        if not value:
            status = "FAIL"
            issues.append("missing_candidate:%s" % key)

    time_reference_topic = first_by_score(
        [(topic, topic_counts.get(topic, 0)) for topic in time_reference_topics],
        lambda item: (
            (100 if item[0] == "/time_reference" else 0)
            + (60 if name_has(item[0], "time_reference") else 0)
            + min(item[1], 1000) / 1000.0
        ),
    )
    if time_reference_topic:
        time_reference_status = "PRESENT_REQUIRED"
        time_sync_evidence_status = "INITIAL_TIME_STATUS_CAPTURE_REQUIRED"
        time_reference_arg = "--time-reference-topic %s" % shlex.quote(time_reference_topic)
    else:
        time_reference_status = "NOT_PRESENT_INITIAL_TEST"
        time_sync_evidence_status = "NOT_PRESENT_INITIAL_TEST"
        time_reference_arg = "--no-time-reference"

    initial_velocity_topic = first_by_score(
        [(topic, topic_counts.get(topic, 0)) for topic in velocity_reference_topics],
        lambda item: (
            (100 if name_has(item[0], "inspvax") else 0)
            + (50 if name_has(item[0], "velocity") else 0)
            + min(item[1], 1000) / 1000.0
        ),
    )
    initial_velocity = first_velocity_sample(bag, initial_velocity_topic)
    if not initial_velocity_topic:
        initial_velocity_status = "NOT_PRESENT_INITIAL_TEST"
        initial_velocity_required = "NO"
        initial_velocity_policy = "NOT_AVAILABLE_INITIAL_TEST"
    elif initial_velocity is None:
        initial_velocity_status = "UNPARSEABLE"
        initial_velocity_required = "YES"
        initial_velocity_policy = "START_ONLY_AUDIT"
    else:
        initial_velocity_status = "CAPTURED"
        initial_velocity_required = "YES"
        initial_velocity_policy = "START_ONLY_AUDIT"
    plc_feedback_topic_count = sum(
        1 for topic in plc_topics if topic_counts.get(topic, 0) > 0
    )
    plc_feedback_status = (
        "NOT_PRESENT_NA" if plc_feedback_topic_count == 0 else "PRESENT_PROFILE_ONLY"
    )
    plc_feedback_gate_status = (
        "NA_INITIAL_TEST"
        if plc_feedback_topic_count == 0
        else "PRESENT_PROFILE_ONLY_FIELD_VALIDATION_REQUIRED"
    )

    command_args = [
        suite_script,
        "--bag",
        bag_path,
        "--out",
        suite_out,
    ]
    if center_topic:
        command_args += ["--center-topic", center_topic]
    if left_topic:
        command_args += ["--left-topic", left_topic]
    if right_topic:
        command_args += ["--right-topic", right_topic]
    if imu_topic:
        command_args += ["--imu-topic", imu_topic]
    if time_reference_topic:
        command_args += ["--time-reference-topic", time_reference_topic]
    else:
        command_args += ["--no-time-reference"]
    if initial_velocity_topic:
        command_args += ["--initial-velocity-topic", initial_velocity_topic]
    else:
        command_args += ["--no-initial-velocity-reference"]
    command_args += ["--rate", "1.0", "--execute"]
    recommended_command = shell_join(command_args)

    duration = bag.get_end_time() - bag.get_start_time()
    with open(profile_path, "w", encoding="utf-8") as output:
        output.write("actual_bag_profile_status=%s\n" % status)
        output.write("bag_path=%s\n" % bag_path)
        output.write("bag_start_s=%.6f\n" % bag.get_start_time())
        output.write("bag_end_s=%.6f\n" % bag.get_end_time())
        output.write("bag_duration_s=%.6f\n" % duration)
        output.write("actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n")
        output.write("bag_sensor_set=LIDAR_IMU_ONLY\n")
        output.write("field_acceptance_eligible=NO\n")
        output.write("plc_feedback_topic_count=%d\n" % plc_feedback_topic_count)
        output.write("plc_feedback_status=%s\n" % plc_feedback_status)
        output.write("plc_feedback_gate_status=%s\n" % plc_feedback_gate_status)
        output.write("machine_motion_assumption=CONTINUOUS_MOTION\n")
        output.write("vibration_profile=NORMAL\n")
        output.write("field_acceptance_requires_plc_feedback=YES\n")
        output.write("recommended_suite_verified_execute_required=YES\n")
        output.write("recommended_suite_initial_readiness_required=YES\n")
        output.write("recommended_suite_field_acceptance_handoff_required=YES\n")
        output.write(
            "recommended_suite_field_acceptance_handoff_manifest_required=YES\n"
        )
        output.write(
            "recommended_suite_field_acceptance_collection_plan_required=YES\n"
        )
        output.write(
            "recommended_suite_collection_plan_manifest_revalidation_required=YES\n"
        )
        output.write("pointcloud_topic_count=%d\n" % len(pointcloud_topics))
        output.write("imu_topic_count=%d\n" % len(imu_topics))
        output.write("time_reference_topic_count=%d\n" % len(time_reference_topics))
        output.write("center_lidar_topic=%s\n" % (center_topic or "missing"))
        output.write("left_lidar_topic=%s\n" % (left_topic or "missing"))
        output.write("right_lidar_topic=%s\n" % (right_topic or "missing"))
        output.write("imu_topic=%s\n" % (imu_topic or "missing"))
        output.write("time_reference_topic=%s\n" % (time_reference_topic or "NONE"))
        output.write("time_reference_status=%s\n" % time_reference_status)
        output.write("time_sync_evidence_status=%s\n" % time_sync_evidence_status)
        output.write("recommended_time_reference_arg=%s\n" % time_reference_arg)
        output.write("initial_velocity_reference_status=%s\n" % initial_velocity_status)
        output.write("initial_velocity_reference_required=%s\n" % initial_velocity_required)
        output.write("initial_velocity_reference_policy=%s\n" % initial_velocity_policy)
        output.write("initial_velocity_reference_topic=%s\n" % (initial_velocity_topic or "NONE"))
        if initial_velocity is not None:
            output.write(
                "initial_velocity_first_sample_stamp_s=%.6f\n"
                % initial_velocity["stamp"]
            )
            output.write(
                "initial_velocity_north_mps=%.6f\n" % initial_velocity["north"]
            )
            output.write(
                "initial_velocity_east_mps=%.6f\n" % initial_velocity["east"]
            )
            output.write("initial_velocity_up_mps=%.6f\n" % initial_velocity["up"])
            output.write(
                "initial_velocity_speed_mps=%.6f\n" % initial_velocity["speed"]
            )
        output.write("continuous_velocity_reference_used=NO\n")
        output.write("velocity_reference_played_to_slam=NO\n")
        for topic, count in sorted(topic_counts.items()):
            output.write("topic_count[%s]=%s\n" % (topic, count))
            output.write("topic_type[%s]=%s\n" % (topic, topic_types.get(topic, "")))
        for topic in sorted(pointcloud_topics):
            output.write("fields[%s]=%s\n" % (topic, fields_csv(cloud_fields.get(topic, []))))
            output.write("frame_id[%s]=%s\n" % (topic, cloud_frames.get(topic, "")))
        output.write("recommended_suite_out=%s\n" % suite_out)
        output.write("recommended_suite_command=%s\n" % recommended_command)
        if issues:
            output.write("issues=%s\n" % ",".join(issues))

    with open(command_path, "w", encoding="utf-8") as output:
        output.write("#!/usr/bin/env bash\n")
        output.write("set -euo pipefail\n")
        output.write('script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\n')
        output.write("suite_out=%s\n" % shlex.quote(suite_out))
        output.write('"$script_dir/validate_actual_bag_profile.sh"\n')
        output.write("%s\n" % recommended_command)
        output.write('"$suite_out/commands/validate_actual_bag_initial_test_readiness.sh"\n')
        output.write('"$suite_out/commands/validate_field_acceptance_handoff.sh"\n')
        output.write('"$suite_out/commands/validate_field_acceptance_handoff_manifest.sh"\n')
        output.write('"$suite_out/commands/validate_field_acceptance_collection_plan.sh"\n')
        output.write('"$suite_out/commands/validate_actual_bag_test_suite.sh"\n')
    os.chmod(command_path, 0o755)
finally:
    bag.close()

if status != "PASS":
    raise SystemExit(2)
PY

cat > "$commands_dir/validate_actual_bag_profile.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
suite_root="$(cd "$script_dir/.." && pwd)"
profile_file="$suite_root/reports/actual_bag_profile.txt"
validation_report="$suite_root/reports/actual_bag_profile_validation.txt"
recommended_entry_file="$script_dir/run_recommended_actual_bag_test_suite.sh"

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

safe_value() {
  local value="$1"
  [[ -n "$value" ]] || return 1
  [[ "$value" != "missing" ]] || return 1
  [[ "$value" != "__DUPLICATE_KEY__" ]] || return 1
  [[ "$value" != *";"* ]] || return 1
  [[ "$value" != *$'\r'* ]] || return 1
  [[ "$value" != *$'\n'* ]] || return 1
  [[ "$value" != [[:space:]]* ]] || return 1
  [[ "$value" != *[[:space:]] ]] || return 1
}

is_number() {
  python3 - "$1" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
except Exception:
    sys.exit(1)
sys.exit(0 if math.isfinite(value) else 1)
PY
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

check_safe_present() {
  local key="$1"
  if [[ "${values[$key]+set}" != "set" ]] || ! safe_value "${values[$key]}"; then
    mark_fail "$key"
  else
    mark_pass "$key"
  fi
}

check_number_present() {
  local key="$1"
  if [[ "${values[$key]+set}" != "set" ]] || ! is_number "${values[$key]}"; then
    mark_fail "$key"
  else
    mark_pass "$key"
  fi
}

check_plc_feedback_profile_gate() {
  local count="${values[plc_feedback_topic_count]:-missing}"
  if [[ "$count" =~ ^[0-9]+$ ]]; then
    mark_pass "plc_feedback_topic_count"
  else
    mark_fail "plc_feedback_topic_count"
    return
  fi

  if [[ "$count" == "0" ]]; then
    check_equals "plc_feedback_status" "NOT_PRESENT_NA"
    check_equals "plc_feedback_gate_status" "NA_INITIAL_TEST"
  else
    check_equals "plc_feedback_status" "PRESENT_PROFILE_ONLY"
    check_equals "plc_feedback_gate_status" "PRESENT_PROFILE_ONLY_FIELD_VALIDATION_REQUIRED"
  fi
}

check_recommended_suite_out_anchor() {
  local expected="$suite_root/recommended_suite"
  if [[ "${values[recommended_suite_out]+set}" != "set" ]] ||
     ! safe_value "${values[recommended_suite_out]}" ||
     [[ "${values[recommended_suite_out]}" != "$expected" ]]; then
    mark_fail "recommended_suite_out"
  else
    mark_pass "recommended_suite_out"
  fi
}

check_recommended_suite_entry_gate() {
  if [[ ! -f "$recommended_entry_file" ]]; then
    mark_fail "recommended_suite_entry_gate"
    return
  fi
  if python3 - "$recommended_entry_file" "${values[recommended_suite_command]:-}" "${values[recommended_suite_out]:-}" <<'PY'
import pathlib
import re
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
expected_command = sys.argv[2]
expected_suite_out = sys.argv[3]
lines = [
    line.strip()
    for line in text.splitlines()
    if line.strip() and not line.lstrip().startswith("#")
]

def line_position(predicate, start=0):
    for index in range(start, len(lines)):
        line = lines[index]
        if predicate(line):
            return index
    return -1

def is_early_terminator(line):
    return re.search(r"(^|[\s;&|(){}])(exit|return|exec)(?=$|[\s;&|(){}])", line) is not None

def relaxes_strict_mode(line):
    words = [word.strip("'\"") for word in re.findall(r"[^\s;&|(){}]+", line)]
    for set_index, word in enumerate(words):
        if word != "set":
            continue
        options = words[set_index + 1:]
        if any(option.startswith("+") and ("e" in option or "u" in option)
               for option in options):
            return True
        for option_index in range(len(options) - 1):
            if options[option_index] == "+o" and options[option_index + 1] == "pipefail":
                return True
    return False

validator_position = line_position(
    lambda line: line == '"$script_dir/validate_actual_bag_profile.sh"'
)
suite_position = line_position(lambda line: line == expected_command)
manifest_revalidation_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_actual_bag_test_suite.sh"',
    suite_position + 1 if suite_position >= 0 else 0,
)
suite_out_positions = [
    index for index, line in enumerate(lines) if line.startswith("suite_out=")
]
has_strict_mode = "set -euo pipefail" in text
has_expected_command = bool(expected_command) and suite_position >= 0
expected_suite_out_assignment = "suite_out=%s" % expected_suite_out
has_expected_suite_out = (
    bool(expected_suite_out)
    and len(suite_out_positions) == 1
    and lines[suite_out_positions[0]] == expected_suite_out_assignment
    and suite_out_positions[0] < suite_position
)
has_closed_loop_revalidation = manifest_revalidation_position > suite_position
has_no_early_termination = (
    has_closed_loop_revalidation
    and not any(is_early_terminator(line) for line in lines[:manifest_revalidation_position])
)
keeps_strict_mode = (
    has_closed_loop_revalidation
    and not any(relaxes_strict_mode(line) for line in lines[:manifest_revalidation_position])
)
valid = (
    has_strict_mode
    and has_expected_command
    and has_expected_suite_out
    and has_no_early_termination
    and keeps_strict_mode
    and validator_position >= 0
    and suite_position >= 0
    and validator_position < suite_position
)
sys.exit(0 if valid else 1)
PY
  then
    mark_pass "recommended_suite_entry_gate"
  else
    mark_fail "recommended_suite_entry_gate"
  fi
}

check_recommended_suite_initial_readiness_entry_gate() {
  if [[ ! -f "$recommended_entry_file" ]]; then
    mark_fail "recommended_suite_initial_readiness_entry"
    return
  fi
  if python3 - "$recommended_entry_file" "${values[recommended_suite_command]:-}" <<'PY'
import pathlib
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
expected_command = sys.argv[2]
lines = [
    line.strip()
    for line in text.splitlines()
    if line.strip() and not line.lstrip().startswith("#")
]

def line_position(predicate):
    for index, line in enumerate(lines):
        if predicate(line):
            return index
    return -1

suite_position = line_position(lambda line: line == expected_command)
readiness_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_actual_bag_initial_test_readiness.sh"'
)
valid = bool(expected_command) and suite_position >= 0 and readiness_position > suite_position
sys.exit(0 if valid else 1)
PY
  then
    mark_pass "recommended_suite_initial_readiness_entry"
  else
    mark_fail "recommended_suite_initial_readiness_entry"
  fi
}

check_recommended_suite_field_acceptance_handoff_entry_gate() {
  if [[ ! -f "$recommended_entry_file" ]]; then
    mark_fail "recommended_suite_field_acceptance_handoff_entry"
    return
  fi
  if python3 - "$recommended_entry_file" "${values[recommended_suite_command]:-}" <<'PY'
import pathlib
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
expected_command = sys.argv[2]
lines = [
    line.strip()
    for line in text.splitlines()
    if line.strip() and not line.lstrip().startswith("#")
]

def line_position(predicate):
    for index, line in enumerate(lines):
        if predicate(line):
            return index
    return -1

suite_position = line_position(lambda line: line == expected_command)
readiness_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_actual_bag_initial_test_readiness.sh"'
)
handoff_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_field_acceptance_handoff.sh"'
)
valid = (
    bool(expected_command)
    and suite_position >= 0
    and readiness_position > suite_position
    and handoff_position > readiness_position
)
sys.exit(0 if valid else 1)
PY
  then
    mark_pass "recommended_suite_field_acceptance_handoff_entry"
  else
    mark_fail "recommended_suite_field_acceptance_handoff_entry"
  fi
}

check_recommended_suite_field_acceptance_handoff_manifest_entry_gate() {
  if [[ ! -f "$recommended_entry_file" ]]; then
    mark_fail "recommended_suite_field_acceptance_handoff_manifest_entry"
    return
  fi
  if python3 - "$recommended_entry_file" "${values[recommended_suite_command]:-}" <<'PY'
import pathlib
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
expected_command = sys.argv[2]
lines = [
    line.strip()
    for line in text.splitlines()
    if line.strip() and not line.lstrip().startswith("#")
]

def line_position(predicate):
    for index, line in enumerate(lines):
        if predicate(line):
            return index
    return -1

suite_position = line_position(lambda line: line == expected_command)
readiness_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_actual_bag_initial_test_readiness.sh"'
)
handoff_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_field_acceptance_handoff.sh"'
)
handoff_manifest_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_field_acceptance_handoff_manifest.sh"'
)
valid = (
    bool(expected_command)
    and suite_position >= 0
    and readiness_position > suite_position
    and handoff_position > readiness_position
    and handoff_manifest_position > handoff_position
)
sys.exit(0 if valid else 1)
PY
  then
    mark_pass "recommended_suite_field_acceptance_handoff_manifest_entry"
  else
    mark_fail "recommended_suite_field_acceptance_handoff_manifest_entry"
  fi
}

check_recommended_suite_field_acceptance_collection_plan_entry_gate() {
  if [[ ! -f "$recommended_entry_file" ]]; then
    mark_fail "recommended_suite_field_acceptance_collection_plan_entry"
    return
  fi
  if python3 - "$recommended_entry_file" "${values[recommended_suite_command]:-}" <<'PY'
import pathlib
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
expected_command = sys.argv[2]
lines = [
    line.strip()
    for line in text.splitlines()
    if line.strip() and not line.lstrip().startswith("#")
]

def line_position(predicate):
    for index, line in enumerate(lines):
        if predicate(line):
            return index
    return -1

suite_position = line_position(lambda line: line == expected_command)
readiness_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_actual_bag_initial_test_readiness.sh"'
)
handoff_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_field_acceptance_handoff.sh"'
)
handoff_manifest_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_field_acceptance_handoff_manifest.sh"'
)
collection_plan_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_field_acceptance_collection_plan.sh"'
)
valid = (
    bool(expected_command)
    and suite_position >= 0
    and readiness_position > suite_position
    and handoff_position > readiness_position
    and handoff_manifest_position > handoff_position
    and collection_plan_position > handoff_manifest_position
)
sys.exit(0 if valid else 1)
PY
  then
    mark_pass "recommended_suite_field_acceptance_collection_plan_entry"
  else
    mark_fail "recommended_suite_field_acceptance_collection_plan_entry"
  fi
}

check_recommended_suite_collection_plan_manifest_revalidation_entry_gate() {
  if [[ ! -f "$recommended_entry_file" ]]; then
    mark_fail "recommended_suite_collection_plan_manifest_revalidation_entry"
    return
  fi
  if python3 - "$recommended_entry_file" "${values[recommended_suite_command]:-}" <<'PY'
import pathlib
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
expected_command = sys.argv[2]
lines = [
    line.strip()
    for line in text.splitlines()
    if line.strip() and not line.lstrip().startswith("#")
]

def line_position(predicate, start=0):
    for index, line in enumerate(lines):
        if index < start:
            continue
        if predicate(line):
            return index
    return -1

suite_position = line_position(lambda line: line == expected_command)
collection_plan_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_field_acceptance_collection_plan.sh"'
)
manifest_revalidation_position = line_position(
    lambda line: line == '"$suite_out/commands/validate_actual_bag_test_suite.sh"',
    collection_plan_position + 1,
)
valid = (
    bool(expected_command)
    and suite_position >= 0
    and collection_plan_position > suite_position
    and manifest_revalidation_position > collection_plan_position
)
sys.exit(0 if valid else 1)
PY
  then
    mark_pass "recommended_suite_collection_plan_manifest_revalidation_entry"
  else
    mark_fail "recommended_suite_collection_plan_manifest_revalidation_entry"
  fi
}

if [[ ! -f "$profile_file" ]]; then
  {
    echo "actual_bag_profile_validation_status=FAIL"
    echo "profile_file=$profile_file"
    echo "profile_file_status=MISSING"
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
done < "$profile_file"

check_equals "actual_bag_profile_status" "PASS"
check_equals "actual_bag_test_scope" "INITIAL_LIDAR_IMU_ONLY"
check_equals "bag_sensor_set" "LIDAR_IMU_ONLY"
check_equals "field_acceptance_eligible" "NO"
check_plc_feedback_profile_gate
check_equals "machine_motion_assumption" "CONTINUOUS_MOTION"
check_equals "vibration_profile" "NORMAL"
check_equals "field_acceptance_requires_plc_feedback" "YES"
check_equals "recommended_suite_verified_execute_required" "YES"
check_equals "recommended_suite_initial_readiness_required" "YES"
check_equals "recommended_suite_field_acceptance_handoff_required" "YES"
check_equals "recommended_suite_field_acceptance_handoff_manifest_required" "YES"
check_equals "recommended_suite_field_acceptance_collection_plan_required" "YES"
check_equals "recommended_suite_collection_plan_manifest_revalidation_required" "YES"
check_equals "velocity_reference_played_to_slam" "NO"
check_equals "continuous_velocity_reference_used" "NO"

for key in center_lidar_topic left_lidar_topic right_lidar_topic imu_topic recommended_suite_command; do
  check_safe_present "$key"
done
check_recommended_suite_out_anchor

if [[ "${values[recommended_suite_command]+set}" != "set" ]] ||
   [[ "${values[recommended_suite_command]}" != *"actual_bag_test_suite.sh"* ]] ||
   [[ "${values[recommended_suite_command]}" != *"--execute"* ]]; then
  mark_fail "recommended_suite_command"
fi
if [[ "${values[initial_velocity_reference_topic]+set}" == "set" ]] &&
   safe_value "${values[initial_velocity_reference_topic]}" &&
   [[ "${values[initial_velocity_reference_topic]}" != "missing" ]] &&
   [[ "${values[initial_velocity_reference_topic]}" != "NONE" ]] &&
   [[ "${values[recommended_suite_command]:-}" != *"--initial-velocity-topic ${values[initial_velocity_reference_topic]}"* ]]; then
  mark_fail "recommended_suite_command"
fi
check_recommended_suite_entry_gate
check_recommended_suite_initial_readiness_entry_gate
check_recommended_suite_field_acceptance_handoff_entry_gate
check_recommended_suite_field_acceptance_handoff_manifest_entry_gate
check_recommended_suite_field_acceptance_collection_plan_entry_gate
check_recommended_suite_collection_plan_manifest_revalidation_entry_gate

time_status="${values[time_reference_status]:-missing}"
case "$time_status" in
  PRESENT_REQUIRED)
    check_safe_present "time_reference_topic"
    check_equals "time_sync_evidence_status" "INITIAL_TIME_STATUS_CAPTURE_REQUIRED"
    ;;
  NOT_PRESENT_INITIAL_TEST)
    check_equals "time_reference_topic" "NONE"
    check_equals "time_sync_evidence_status" "NOT_PRESENT_INITIAL_TEST"
    ;;
  *)
    mark_fail "time_reference_status"
    ;;
esac

velocity_status="${values[initial_velocity_reference_status]:-missing}"
case "$velocity_status" in
  CAPTURED)
    check_equals "initial_velocity_reference_required" "YES"
    check_equals "initial_velocity_reference_policy" "START_ONLY_AUDIT"
    check_safe_present "initial_velocity_reference_topic"
    check_number_present "initial_velocity_first_sample_stamp_s"
    check_number_present "initial_velocity_north_mps"
    check_number_present "initial_velocity_east_mps"
    check_number_present "initial_velocity_up_mps"
    check_number_present "initial_velocity_speed_mps"
    ;;
  UNPARSEABLE)
    check_equals "initial_velocity_reference_required" "YES"
    check_equals "initial_velocity_reference_policy" "START_ONLY_AUDIT"
    check_safe_present "initial_velocity_reference_topic"
    mark_pass "initial_velocity_reference_status"
    ;;
  NOT_PRESENT_INITIAL_TEST)
    check_equals "initial_velocity_reference_required" "NO"
    check_equals "initial_velocity_reference_policy" "NOT_AVAILABLE_INITIAL_TEST"
    check_equals "initial_velocity_reference_topic" "NONE"
    if [[ "${values[recommended_suite_command]:-}" != *"--no-initial-velocity-reference"* ]]; then
      mark_fail "recommended_suite_command"
    fi
    mark_pass "initial_velocity_reference_status"
    ;;
  *)
    mark_fail "initial_velocity_reference_status"
    ;;
esac

{
  echo "actual_bag_profile_validation_status=$overall"
  echo "profile_file=$profile_file"
  echo "duplicate_key_count=$duplicate_keys"
  echo "malformed_line_count=$malformed_lines"
  for key in "${!statuses[@]}"; do
    echo "${key}_status=${statuses[$key]}"
  done | sort
  echo "field_acceptance_eligible=${values[field_acceptance_eligible]:-missing}"
  echo "velocity_reference_played_to_slam=${values[velocity_reference_played_to_slam]:-missing}"
  echo "continuous_velocity_reference_used=${values[continuous_velocity_reference_used]:-missing}"
} > "$validation_report"

[[ "$overall" == "PASS" ]]
EOF
chmod +x "$commands_dir/validate_actual_bag_profile.sh"

echo "$out_abs"
