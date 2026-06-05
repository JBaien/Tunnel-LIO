#!/usr/bin/env bash
set -euo pipefail

ROOT="/tmp/tunnel_lio_sessions"
NAME=""
PREFIX="tunnel_lio"
SCENARIO="UNSPECIFIED"
TOPICS="/points_raw /sensors/imu/raw /diagnostics/imu_modbus /tf /tf_static /rosout"
PCAP_INTERFACE="any"
START_PCAP=0
RUNTIME_DIR=""
RUNTIME_STABILITY_SAMPLES=1440
RUNTIME_STABILITY_INTERVAL=60
TIME_STATUS_TOPIC="/time/status"
PPS_TOPIC="/time/pps_event"
DRY_RUN=0

usage() {
  cat <<'USAGE'
Usage: record_session.sh [--root DIR] [--name NAME] [--prefix PREFIX] [--scenario NAME] [--topics "TOPICS"] [--pcap-interface IFACE] [--start-pcap [true|false]] [--runtime-dir DIR] [--runtime-stability-samples N] [--runtime-stability-interval SEC] [--time-status-topic TOPIC] [--pps-topic TOPIC] [--dry-run]

Creates a session directory with metadata, reproducible command files, ROS
parameter/TF snapshot helpers, and starts rosbag recording for the configured
Tunnel-LIO runtime topics.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --root)
      ROOT="$2"
      shift 2
      ;;
    --name)
      NAME="$2"
      shift 2
      ;;
    --prefix)
      PREFIX="$2"
      shift 2
      ;;
    --scenario)
      SCENARIO="$2"
      shift 2
      ;;
    --topics)
      TOPICS="$2"
      shift 2
      ;;
    --pcap-interface)
      PCAP_INTERFACE="$2"
      shift 2
      ;;
    --runtime-dir)
      RUNTIME_DIR="$2"
      shift 2
      ;;
    --runtime-stability-samples)
      RUNTIME_STABILITY_SAMPLES="$2"
      shift 2
      ;;
    --runtime-stability-interval)
      RUNTIME_STABILITY_INTERVAL="$2"
      shift 2
      ;;
    --time-status-topic)
      TIME_STATUS_TOPIC="$2"
      shift 2
      ;;
    --pps-topic)
      PPS_TOPIC="$2"
      shift 2
      ;;
    --start-pcap)
      if [[ $# -gt 1 && "$2" != --* ]]; then
        if [[ "$2" == "true" || "$2" == "1" ]]; then
          START_PCAP=1
        elif [[ "$2" == "false" || "$2" == "0" ]]; then
          START_PCAP=0
        else
          echo "start_pcap must be true, false, 1, or 0" >&2
          exit 2
        fi
        shift 2
      else
        START_PCAP=1
        shift
      fi
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$NAME" ]]; then
  NAME="$(date +%Y%m%d_%H%M%S)"
fi

validate_manifest_value() {
  local field_name="$1"
  local value="$2"
  local allow_empty="$3"
  if [[ -z "$value" ]]; then
    if [[ "$allow_empty" == "true" ]]; then
      return 0
    fi
    echo "${field_name} must not be empty" >&2
    return 2
  fi
  if [[ "$value" =~ ^[[:space:]] || "$value" =~ [[:space:]]$ ]]; then
    echo "${field_name} must not have leading or trailing whitespace" >&2
    return 2
  fi
  if [[ "$value" == "missing" || "$value" == "__DUPLICATE_KEY__" ]]; then
    echo "${field_name} must not be ${value}" >&2
    return 2
  fi
  if [[ "$value" == *";"* || "$value" == *$'\n'* || "$value" == *$'\r'* ]]; then
    echo "${field_name} must not contain manifest separators" >&2
    return 2
  fi
}

validate_csv_safe_value() {
  local field_name="$1"
  local value="$2"
  local allow_empty="$3"
  if [[ -z "$value" ]]; then
    if [[ "$allow_empty" == "true" ]]; then
      return 0
    fi
    echo "${field_name} must not be empty" >&2
    return 2
  fi
  if [[ "$value" == *","* ]]; then
    echo "${field_name} must not contain CSV separators" >&2
    return 2
  fi
}

validate_path_segment() {
  local field_name="$1"
  local value="$2"
  if [[ "$value" == "." || "$value" == ".." ||
        "$value" == *"/"* || "$value" == *"\\"* ]]; then
    echo "${field_name} must be a single path segment" >&2
    return 2
  fi
}

validate_generated_script_token() {
  local field_name="$1"
  local value="$2"
  if [[ ! "$value" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "${field_name} must be a safe generated script token" >&2
    return 2
  fi
}

validate_scenario_token() {
  local value="$1"
  if [[ ! "$value" =~ ^[A-Z0-9_]+$ ]]; then
    echo "scenario must be an uppercase scenario token" >&2
    return 2
  fi
}

validate_generated_script_literal() {
  local field_name="$1"
  local value="$2"
  local allow_empty="$3"
  if [[ -z "$value" ]]; then
    if [[ "$allow_empty" == "true" ]]; then
      return 0
    fi
    echo "${field_name} must not be empty" >&2
    return 2
  fi
  if [[ "$value" == *'$'* || "$value" == *'`'* ||
        "$value" == *'"'* || "$value" == *"'"* ||
        "$value" == *'('* || "$value" == *')'* ||
        "$value" == *'|'* || "$value" == *'&'* ||
        "$value" == *'<'* || "$value" == *'>'* ||
        "$value" == *'*'* || "$value" == *'?'* ||
        "$value" == *'['* || "$value" == *']'* ]]; then
    echo "${field_name} must not contain shell metacharacters" >&2
    return 2
  fi
}

validate_absolute_path() {
  local field_name="$1"
  local value="$2"
  local allow_empty="$3"
  if [[ -z "$value" ]]; then
    if [[ "$allow_empty" == "true" ]]; then
      return 0
    fi
    echo "${field_name} must not be empty" >&2
    return 2
  fi
  if [[ "$value" =~ ^[[:space:]] || "$value" =~ [[:space:]]$ ]]; then
    echo "${field_name} must not have leading or trailing whitespace" >&2
    return 2
  fi
  if [[ "$value" != /* ]]; then
    echo "${field_name} must be an absolute path" >&2
    return 2
  fi
  if [[ "$value" == "/" ]]; then
    echo "${field_name} must not be filesystem root" >&2
    return 2
  fi
  local path_segments
  IFS='/' read -r -a path_segments <<< "$value"
  local segment
  for segment in "${path_segments[@]}"; do
    if [[ "$segment" == "." || "$segment" == ".." ]]; then
      echo "${field_name} must not contain dot path segments" >&2
      return 2
    fi
  done
}

validate_ros_topic_name() {
  local field_name="$1"
  local value="$2"
  if [[ ! "$value" =~ ^(/|~)?[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$ ]]; then
    echo "${field_name} must be a valid ROS topic name" >&2
    return 2
  fi
}

validate_ros_topic_list() {
  local field_name="$1"
  local value="$2"
  local topic=""
  local topic_count=0
  IFS=$' \t\n' read -ra topic_tokens <<< "$value"
  for topic in "${topic_tokens[@]}"; do
    validate_ros_topic_name "$field_name" "$topic"
    topic_count=$((topic_count + 1))
  done
  if [[ "$topic_count" -eq 0 ]]; then
    echo "${field_name} must contain at least one ROS topic name" >&2
    return 2
  fi
}

validate_pcap_interface() {
  local value="$1"
  if [[ ! "$value" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
    echo "pcap_interface must be a tcpdump-compatible interface name" >&2
    return 2
  fi
}

validate_positive_integer() {
  local field_name="$1"
  local value="$2"
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${field_name} must be a positive integer" >&2
    return 2
  fi
}

validate_nonnegative_integer() {
  local field_name="$1"
  local value="$2"
  if [[ ! "$value" =~ ^[0-9]+$ ]]; then
    echo "${field_name} must be a nonnegative integer" >&2
    return 2
  fi
}

validate_manifest_value "session_id" "$NAME" false
validate_manifest_value "session_root" "$ROOT" false
validate_manifest_value "artifact_prefix" "$PREFIX" false
validate_path_segment "session_id" "$NAME"
validate_path_segment "artifact_prefix" "$PREFIX"
validate_generated_script_token "session_id" "$NAME"
validate_generated_script_token "artifact_prefix" "$PREFIX"
validate_absolute_path "session_root" "$ROOT" false
validate_manifest_value "scenario" "$SCENARIO" false
validate_scenario_token "$SCENARIO"
validate_manifest_value "runtime_dir" "$RUNTIME_DIR" true
validate_manifest_value "topics" "$TOPICS" false
validate_manifest_value "pcap_interface" "$PCAP_INTERFACE" false
validate_manifest_value "time_status_topic" "$TIME_STATUS_TOPIC" false
validate_manifest_value "pps_topic" "$PPS_TOPIC" false
validate_generated_script_literal "runtime_dir" "$RUNTIME_DIR" true
validate_csv_safe_value "runtime_dir" "$RUNTIME_DIR" true
validate_absolute_path "runtime_dir" "$RUNTIME_DIR" true
validate_generated_script_literal "topics" "$TOPICS" false
validate_generated_script_literal "pcap_interface" "$PCAP_INTERFACE" false
validate_generated_script_literal "time_status_topic" "$TIME_STATUS_TOPIC" false
validate_generated_script_literal "pps_topic" "$PPS_TOPIC" false
validate_ros_topic_list "topics" "$TOPICS"
validate_pcap_interface "$PCAP_INTERFACE"
validate_ros_topic_name "time_status_topic" "$TIME_STATUS_TOPIC"
validate_ros_topic_name "pps_topic" "$PPS_TOPIC"
validate_positive_integer "runtime_stability_samples" "$RUNTIME_STABILITY_SAMPLES"
validate_nonnegative_integer "runtime_stability_interval" "$RUNTIME_STABILITY_INTERVAL"

SESSION_DIR="${ROOT}/${NAME}"
BAG_DIR="${SESSION_DIR}/bags"
PCAP_DIR="${SESSION_DIR}/pcap"
SNAPSHOT_DIR="${SESSION_DIR}/snapshots"
LOG_DIR="${SESSION_DIR}/logs"
COMMAND_DIR="${SESSION_DIR}/commands"
REPORT_DIR="${SESSION_DIR}/reports"
BAG_PATH="${BAG_DIR}/${PREFIX}.bag"
PCAP_PATH="${PCAP_DIR}/${PREFIX}.pcap"
EVIDENCE_MANIFEST_PATH="${SESSION_DIR}/evidence_manifest.txt"

mkdir -p "$BAG_DIR" "$PCAP_DIR" "$SNAPSHOT_DIR" "$LOG_DIR" "$COMMAND_DIR" "$REPORT_DIR"

GIT_COMMIT="unknown"
if git -C /home/bai/Desktop/Tunnel-LIO rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  GIT_COMMIT="$(git -C /home/bai/Desktop/Tunnel-LIO rev-parse HEAD)"
fi

cat > "${SESSION_DIR}/metadata.env" <<EOF
session_name=${NAME}
session_dir=${SESSION_DIR}
created_at=$(date --iso-8601=seconds)
git_commit=${GIT_COMMIT}
scenario=${SCENARIO}
bag_prefix=${PREFIX}
bag_path=${BAG_PATH}
pcap_dir=${PCAP_DIR}
pcap_path=${PCAP_PATH}
pcap_interface=${PCAP_INTERFACE}
start_pcap=${START_PCAP}
snapshot_dir=${SNAPSHOT_DIR}
log_dir=${LOG_DIR}
report_dir=${REPORT_DIR}
evidence_manifest_path=${EVIDENCE_MANIFEST_PATH}
runtime_dir=${RUNTIME_DIR}
runtime_stability_samples=${RUNTIME_STABILITY_SAMPLES}
runtime_stability_interval=${RUNTIME_STABILITY_INTERVAL}
time_status_topic=${TIME_STATUS_TOPIC}
pps_topic=${PPS_TOPIC}
topics=${TOPICS}
EOF

cat > "${EVIDENCE_MANIFEST_PATH}" <<EOF
session_id=${NAME};scenario=${SCENARIO};runtime_dir=${RUNTIME_DIR};metrics_report=reports/validation_metrics_report.txt;event_file=reports/replay_events.txt;bag_file=bags/${PREFIX}.bag;pcap_file=pcap/${PREFIX}.pcap;tf_snapshot=snapshots/tf_monitor.txt;params_snapshot=snapshots/rosparams.yaml;runtime_log=logs/record_session.log;time_sync=logs/time_sync_status.txt;pps_ptp_wiring=reports/pps_ptp_wiring_verified.txt;runtime_health=logs/runtime_health_latest.txt;runtime_deployment=logs/runtime_deployment_check.txt;runtime_stability_csv=logs/runtime_stability.csv;runtime_stability_summary=logs/runtime_stability_summary.txt;runtime_stability_run_log=logs/runtime_stability_run.log;section_export=reports/section_export.csv;power_loss_resume=reports/power_loss_resume_verified.txt;field_acceptance=reports/field_acceptance_report.txt
EOF

cat > "${REPORT_DIR}/README.md" <<EOF
# Tunnel-LIO validation reports

Place raw validation metrics in validation_metrics.txt or normalized replay/HIL
events in replay_events.txt. Run commands/validate_evidence.sh to generate the
metrics report and the evidence completeness report for this session.
EOF

cat > "${LOG_DIR}/record_session.log" <<EOF
record_session created_at=$(date --iso-8601=seconds)
session_dir=${SESSION_DIR}
dry_run=${DRY_RUN}
EOF

cat > "${SNAPSHOT_DIR}/README.md" <<EOF
# Tunnel-LIO session snapshots

This directory stores ROS parameter and TF snapshots captured before bag
recording. PCAP files from LiDAR drivers or tcpdump should be placed in:

${PCAP_DIR}
EOF

cat > "${COMMAND_DIR}/record_rosbag.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd "${SESSION_DIR}"
exec rosbag record -O "bags/${PREFIX}.bag" ${TOPICS}
EOF
chmod +x "${COMMAND_DIR}/record_rosbag.sh"

cat > "${COMMAND_DIR}/record_pcap.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd "${SESSION_DIR}"
TCPDUMP_FILTER="\${TCPDUMP_FILTER:-udp}"
exec tcpdump -i "${PCAP_INTERFACE}" -w "pcap/${PREFIX}.pcap" \${TCPDUMP_FILTER}
EOF
chmod +x "${COMMAND_DIR}/record_pcap.sh"

cat > "${COMMAND_DIR}/bringup_fusion_timoo_session.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
exec roslaunch mine_slam_bringup bringup_fusion_timoo.launch section_session_id:=${NAME} "\$@"
EOF
chmod +x "${COMMAND_DIR}/bringup_fusion_timoo_session.sh"

cat > "${COMMAND_DIR}/bringup_fusion_tmlidar_session.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
exec roslaunch mine_slam_bringup bringup_fusion_tmlidar.launch section_session_id:=${NAME} "\$@"
EOF
chmod +x "${COMMAND_DIR}/bringup_fusion_tmlidar_session.sh"

cat > "${COMMAND_DIR}/snapshot_ros_state.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
mkdir -p "${SNAPSHOT_DIR}" "${LOG_DIR}"
rosparam dump "${SNAPSHOT_DIR}/rosparams.yaml" || true
rosrun tf2_tools view_frames.py "${SNAPSHOT_DIR}/tf_frames.pdf" > "${LOG_DIR}/tf2_tools.log" 2>&1 || true
rosrun tf tf_monitor > "${SNAPSHOT_DIR}/tf_monitor.txt" 2>&1 || true
rosnode list > "${SNAPSHOT_DIR}/rosnodes.txt" 2>&1 || true
rostopic list > "${SNAPSHOT_DIR}/rostopics.txt" 2>&1 || true
EOF
chmod +x "${COMMAND_DIR}/snapshot_ros_state.sh"

cat > "${COMMAND_DIR}/capture_time_sync.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
time_status_topic="\${TIME_STATUS_TOPIC:-${TIME_STATUS_TOPIC}}"
pps_topic="\${PPS_TOPIC:-${PPS_TOPIC}}"
target="${LOG_DIR}/time_sync_status.txt"
raw="${LOG_DIR}/time_status_raw.yaml"
mkdir -p "${LOG_DIR}"

is_valid_text_value() {
  local value="\$1"
  [[ -n "\${value}" ]] && \\
    [[ ! "\${value}" =~ ^[[:space:]] ]] && \\
    [[ ! "\${value}" =~ [[:space:]]\$ ]] && \\
    [[ "\${value}" != "missing" ]] && \\
    [[ "\${value}" != "__DUPLICATE_KEY__" ]] && \\
    [[ "\${value}" != *";"* ]] && \\
    [[ "\${value}" != *\$'\\r'* ]] && \\
    [[ "\${value}" != *\$'\\n'* ]]
}

is_valid_ros_topic_name() {
  local value="\$1"
  is_valid_text_value "\${value}" && \\
    [[ "\${value}" =~ ^(/|~)?[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*\$ ]]
}

time_sync_metadata_status="PASS"
if ! is_valid_ros_topic_name "\${time_status_topic}" || \\
   ! is_valid_ros_topic_name "\${pps_topic}"; then
  time_sync_metadata_status="FAIL"
fi
capture_status="UNAVAILABLE"
if [[ "\${time_sync_metadata_status}" == "FAIL" ]]; then
  capture_status="INVALID_METADATA"
elif [[ -n "\${TIME_SYNC_SOURCE:-}" && -s "\${TIME_SYNC_SOURCE}" ]]; then
  cp -f "\${TIME_SYNC_SOURCE}" "\${raw}"
  capture_status="CAPTURED"
elif command -v rostopic >/dev/null 2>&1; then
  if timeout 5 rostopic echo -n1 "\${time_status_topic}" > "\${raw}" 2>/dev/null; then
    capture_status="CAPTURED"
  fi
fi

pps_status="FAIL"
clock_offset_status="FAIL"
pps_jitter_ms="missing"
mean_offset_ms="missing"
time_sync_numeric_status="PASS"
is_number() {
  local value="\$1"
  [[ "\${value}" =~ ^[+-]?(([0-9]+(\\.[0-9]*)?)|(\\.[0-9]+))([eE][+-]?[0-9]+)?\$ ]]
}
is_nonnegative_number() {
  local value="\$1"
  is_number "\${value}" && awk -v value="\${value}" 'BEGIN { exit(value >= 0 ? 0 : 1) }'
}
if [[ "\${capture_status}" == "CAPTURED" && -s "\${raw}" ]]; then
  extract_status_field() {
    local status_name="\$1"
    local field_name="\$2"
    awk -v status_name="\${status_name}" -v field_name="\${field_name}" '
      function clean(value) {
        sub(/^[[:space:]]*[^:]+:[[:space:]]*/, "", value)
        gsub(/"/, "", value)
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
        return value
      }
      function maybe_emit() {
        if (!emitted && matched && field_name == "level" && level != "") {
          print level
          emitted = 1
          exit
        }
      }
      {
        line = \$0
        first = match(line, /[^[:space:]]/)
        indent = first == 0 ? 0 : first - 1
        trimmed = line
        sub(/^[[:space:]]*/, "", trimmed)
        if (trimmed == "-" && indent <= 2) {
          maybe_emit()
          matched = 0
          level = ""
          pending_key = ""
          next
        }
        if (trimmed ~ /^level:/) {
          level = clean(trimmed)
          next
        }
        if (trimmed ~ /^name:/) {
          matched = clean(trimmed) == status_name
          next
        }
        if (!matched) {
          next
        }
        if (trimmed ~ /^key:/) {
          pending_key = clean(trimmed)
          next
        }
        if (trimmed ~ /^value:/ && pending_key == field_name) {
          print clean(trimmed)
          exit
        }
      }
      END {
        maybe_emit()
      }' "\${raw}"
  }

  pps_level=\$(extract_status_field "lio_time_manager pps: \${pps_topic}" "level")
  clock_level=\$(extract_status_field "lio_time_manager clock: \${pps_topic}" "level")
  if [[ "\${pps_level}" == "0" ]]; then
    pps_status="PASS"
  fi
  if [[ "\${clock_level}" == "0" ]]; then
    clock_offset_status="PASS"
  fi
  pps_jitter_ms=\$(extract_status_field "lio_time_manager pps: \${pps_topic}" "interval_jitter_ms")
  mean_offset_ms=\$(extract_status_field "lio_time_manager clock: \${pps_topic}" "mean_offset_ms")
  [[ -n "\${pps_jitter_ms}" ]] || pps_jitter_ms="missing"
  [[ -n "\${mean_offset_ms}" ]] || mean_offset_ms="missing"
  if [[ "\${pps_status}" == "PASS" ]] && ! is_nonnegative_number "\${pps_jitter_ms}"; then
    pps_status="FAIL"
    time_sync_numeric_status="FAIL"
  fi
  if [[ "\${clock_offset_status}" == "PASS" ]] && ! is_number "\${mean_offset_ms}"; then
    clock_offset_status="FAIL"
    time_sync_numeric_status="FAIL"
  fi
fi

time_sync_status="FAIL"
if [[ "\${time_sync_metadata_status}" == "PASS" ]] && \\
   [[ "\${capture_status}" == "CAPTURED" ]] && \\
   [[ "\${pps_status}" == "PASS" ]] && \\
   [[ "\${clock_offset_status}" == "PASS" ]] && \\
   is_nonnegative_number "\${pps_jitter_ms}" && \\
   is_number "\${mean_offset_ms}"; then
  time_sync_status="PASS"
fi

{
  echo "timestamp=\$(date --iso-8601=seconds)"
  echo "time_sync_status=\${time_sync_status}"
  echo "time_status_topic=\${time_status_topic}"
  echo "pps_topic=\${pps_topic}"
  echo "capture_status=\${capture_status}"
  echo "pps_status=\${pps_status}"
  echo "clock_offset_status=\${clock_offset_status}"
  echo "pps_jitter_ms=\${pps_jitter_ms}"
  echo "mean_offset_ms=\${mean_offset_ms}"
  echo "raw=\${raw}"
} > "\${target}"
echo "\${target}"
if [[ "\${time_sync_metadata_status}" != "PASS" ]] || [[ "\${time_sync_numeric_status}" != "PASS" ]]; then
  exit 1
fi
EOF
chmod +x "${COMMAND_DIR}/capture_time_sync.sh"

cat > "${COMMAND_DIR}/capture_pps_ptp_wiring.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
target="${REPORT_DIR}/pps_ptp_wiring_verified.txt"
time_sync_report="${LOG_DIR}/time_sync_status.txt"
manual_confirmation_file="${REPORT_DIR}/pps_ptp_wiring_confirmation.txt"
session_dir="${SESSION_DIR}"
mkdir -p "${REPORT_DIR}" "${LOG_DIR}"

line_value() {
  local file="\$1"
  local key="\$2"
  if [[ -s "\${file}" ]]; then
    awk -v key="\${key}" '
      function trim_value(value) {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
        return value
      }
      {
        split_pos = index(\$0, "=")
        if (split_pos == 0) {
          next
        }
        parsed_key = trim_value(substr(\$0, 1, split_pos - 1))
        if (parsed_key == key) {
          ++count
          value = substr(\$0, split_pos + 1)
        }
      }
      END { if (count == 1) print value; else if (count > 1) print "__DUPLICATE_KEY__" }
    ' "\${file}"
  fi
}

line_keys_unique() {
  local file="\$1"
  [[ -s "\${file}" ]] && awk '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    {
      split_pos = index(\$0, "=")
      if (\$0 ~ /^[[:space:]]*$/) {
        next
      }
      if (split_pos == 0) {
        malformed = 1
        next
      }
      key = trim_value(substr(\$0, 1, split_pos - 1))
      if (key == "") {
        malformed = 1
        next
      }
      if (seen[key]) {
        duplicate = 1
      }
      seen[key] = 1
    }
    END { exit((malformed || duplicate) ? 1 : 0) }
  ' "\${file}"
}

is_valid_text_value() {
  local value="\$1"
  [[ -n "\${value}" ]] && \\
    [[ ! "\${value}" =~ ^[[:space:]] ]] && \\
    [[ ! "\${value}" =~ [[:space:]]\$ ]] && \\
    [[ "\${value}" != "missing" ]] && \\
    [[ "\${value}" != "__DUPLICATE_KEY__" ]] && \\
    [[ "\${value}" != *";"* ]] && \\
    [[ "\${value}" != *\$'\\r'* ]] && \\
    [[ "\${value}" != *\$'\\n'* ]]
}

is_valid_ros_topic_name() {
  local value="\$1"
  is_valid_text_value "\${value}" && \\
    [[ "\${value}" =~ ^(/|~)?[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*\$ ]]
}

is_leap_year() {
  local year="\$1"
  (( (10#\${year} % 4 == 0 && 10#\${year} % 100 != 0) || 10#\${year} % 400 == 0 ))
}

days_in_month() {
  local year="\$1"
  local month="\$2"
  case "\${month}" in
    01|03|05|07|08|10|12) echo 31 ;;
    04|06|09|11) echo 30 ;;
    02)
      if is_leap_year "\${year}"; then
        echo 29
      else
        echo 28
      fi
      ;;
    *) echo 0 ;;
  esac
}

is_iso8601_seconds_timestamp() {
  local value="\$1"
  is_valid_text_value "\${value}" || return 1
  [[ "\${value}" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}(Z|[+-][0-9]{2}:[0-9]{2})\$ ]] || return 1
  local year="\${value:0:4}"
  local month="\${value:5:2}"
  local day="\${value:8:2}"
  local hour="\${value:11:2}"
  local minute="\${value:14:2}"
  local second="\${value:17:2}"
  local max_day
  max_day=\$(days_in_month "\${year}" "\${month}")
  local offset_hour="0"
  local offset_minute="0"
  if [[ "\${value:19:1}" != "Z" ]]; then
    offset_hour="\${value:20:2}"
    offset_minute="\${value:23:2}"
  fi
  ((10#\${year} > 0 &&
    10#\${month} >= 1 && 10#\${month} <= 12 &&
    \${max_day} > 0 &&
    10#\${day} >= 1 && 10#\${day} <= \${max_day} &&
    10#\${hour} <= 23 &&
    10#\${minute} <= 59 &&
    10#\${second} <= 59 &&
    10#\${offset_hour} <= 23 &&
    10#\${offset_minute} <= 59))
}

is_number() {
  local value="\$1"
  [[ "\${value}" =~ ^[+-]?(([0-9]+(\\.[0-9]*)?)|(\\.[0-9]+))([eE][+-]?[0-9]+)?\$ ]]
}

is_nonnegative_number() {
  local value="\$1"
  is_number "\${value}" && awk -v value="\${value}" 'BEGIN { exit(value >= 0 ? 0 : 1) }'
}

has_dot_path_segment() {
  local value="\$1"
  local segment
  IFS='/' read -r -a path_segments <<< "\${value}"
  for segment in "\${path_segments[@]}"; do
    if [[ "\${segment}" == "." || "\${segment}" == ".." ]]; then
      return 0
    fi
  done
  return 1
}

resolve_time_sync_raw_path() {
  local value="\$1"
  if ! is_valid_text_value "\${value}" || \\
     [[ "\${value}" == *\\\\* ]] || \\
     [[ "\${value}" == *"//"* ]] || \\
     has_dot_path_segment "\${value}"; then
    return 1
  fi
  if [[ "\${value}" == /* ]]; then
    printf '%s\\n' "\${value}"
  else
    printf '%s/%s\\n' "\${session_dir}" "\${value}"
  fi
}

path_within_session() {
  local value="\$1"
  [[ "\${value}" == "\${session_dir}/"* ]]
}

time_sync_reported_status=\$(line_value "\${time_sync_report}" "time_sync_status")
time_sync_timestamp=\$(line_value "\${time_sync_report}" "timestamp")
capture_status=\$(line_value "\${time_sync_report}" "capture_status")
time_status_topic=\$(line_value "\${time_sync_report}" "time_status_topic")
pps_topic=\$(line_value "\${time_sync_report}" "pps_topic")
pps_status=\$(line_value "\${time_sync_report}" "pps_status")
clock_offset_status=\$(line_value "\${time_sync_report}" "clock_offset_status")
pps_jitter_ms=\$(line_value "\${time_sync_report}" "pps_jitter_ms")
mean_offset_ms=\$(line_value "\${time_sync_report}" "mean_offset_ms")
time_sync_raw=\$(line_value "\${time_sync_report}" "raw")
[[ -n "\${time_sync_reported_status}" ]] || time_sync_reported_status="missing"
[[ -n "\${time_sync_timestamp}" ]] || time_sync_timestamp="missing"
[[ -n "\${capture_status}" ]] || capture_status="missing"
[[ -n "\${time_status_topic}" ]] || time_status_topic="missing"
[[ -n "\${pps_topic}" ]] || pps_topic="missing"
[[ -n "\${pps_status}" ]] || pps_status="missing"
[[ -n "\${clock_offset_status}" ]] || clock_offset_status="missing"
[[ -n "\${pps_jitter_ms}" ]] || pps_jitter_ms="missing"
[[ -n "\${mean_offset_ms}" ]] || mean_offset_ms="missing"
[[ -n "\${time_sync_raw}" ]] || time_sync_raw="missing"
time_sync_keys_status="FAIL"
if line_keys_unique "\${time_sync_report}"; then
  time_sync_keys_status="PASS"
fi
time_sync_raw_status="FAIL"
time_sync_raw_path=""
if time_sync_raw_path=\$(resolve_time_sync_raw_path "\${time_sync_raw}") && \\
   path_within_session "\${time_sync_raw_path}" && \\
   [[ -s "\${time_sync_raw_path}" ]]; then
  time_sync_raw_status="PASS"
fi

time_sync_status="FAIL"
if [[ "\${time_sync_keys_status}" == "PASS" ]] && \\
   [[ "\${time_sync_raw_status}" == "PASS" ]] && \\
   [[ "\${time_sync_reported_status}" == "PASS" ]] && \\
   is_iso8601_seconds_timestamp "\${time_sync_timestamp}" && \\
   [[ "\${capture_status}" == "CAPTURED" ]] && \\
   is_valid_ros_topic_name "\${time_status_topic}" && \\
   is_valid_ros_topic_name "\${pps_topic}" && \\
   [[ "\${pps_status}" == "PASS" ]] && \\
   [[ "\${clock_offset_status}" == "PASS" ]] && \\
   is_nonnegative_number "\${pps_jitter_ms}" && \\
   is_number "\${mean_offset_ms}"; then
  time_sync_status="PASS"
fi

wiring_confirmation="\${PPS_PTP_WIRING_VERIFIED:-FAIL}"
wiring_confirmation_source="manual_env"
wiring_confirmation_overall=\$(line_value "\${manual_confirmation_file}" "pps_ptp_wiring_verified")
[[ -n "\${wiring_confirmation_overall}" ]] || wiring_confirmation_overall="missing"
wiring_confirmation_keys_status="missing"
pps_wiring_verified="missing"
ptp_wiring_verified="missing"
wiring_verified_by="missing"
wiring_verified_at="missing"
if [[ -s "\${manual_confirmation_file}" ]]; then
  wiring_confirmation_source="manual_file"
  wiring_confirmation_keys_status="FAIL"
  if line_keys_unique "\${manual_confirmation_file}"; then
    wiring_confirmation_keys_status="PASS"
  fi
  pps_wiring_verified=\$(line_value "\${manual_confirmation_file}" "pps_wiring_verified")
  ptp_wiring_verified=\$(line_value "\${manual_confirmation_file}" "ptp_wiring_verified")
  wiring_verified_by=\$(line_value "\${manual_confirmation_file}" "wiring_verified_by")
  wiring_verified_at=\$(line_value "\${manual_confirmation_file}" "wiring_verified_at")
  [[ -n "\${pps_wiring_verified}" ]] || pps_wiring_verified="missing"
  [[ -n "\${ptp_wiring_verified}" ]] || ptp_wiring_verified="missing"
  [[ -n "\${wiring_verified_by}" ]] || wiring_verified_by="missing"
  [[ -n "\${wiring_verified_at}" ]] || wiring_verified_at="missing"
  if [[ "\${wiring_confirmation_keys_status}" == "PASS" ]] && \\
     [[ "\${wiring_confirmation_overall}" == "PASS" ]] && \\
     [[ "\${pps_wiring_verified}" == "PASS" ]] && \\
     [[ "\${ptp_wiring_verified}" == "PASS" ]] && \\
     is_valid_text_value "\${wiring_verified_by}" && \\
     is_iso8601_seconds_timestamp "\${wiring_verified_at}"; then
    wiring_confirmation="PASS"
  else
    wiring_confirmation="FAIL"
  fi
fi

timedatectl_status="unavailable"
if command -v timedatectl >/dev/null 2>&1; then
  timedatectl_status=\$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unavailable)
  [[ -n "\${timedatectl_status}" ]] || timedatectl_status="unavailable"
fi

chronyc_status="unavailable"
chronyc_tracking="${LOG_DIR}/chronyc_tracking.txt"
if command -v chronyc >/dev/null 2>&1; then
  if chronyc tracking > "\${chronyc_tracking}" 2>&1; then
    chronyc_status="available"
  else
    chronyc_status="unavailable"
  fi
fi
ptp_status="\${chronyc_status}"

pps_ptp_wiring_verified="FAIL"
if [[ "\${time_sync_status}" == "PASS" && \\
      "\${wiring_confirmation}" == "PASS" && \\
      "\${wiring_confirmation_source}" == "manual_file" ]]; then
  pps_ptp_wiring_verified="PASS"
fi

{
  echo "timestamp=\$(date --iso-8601=seconds)"
  echo "pps_ptp_wiring_verified=\${pps_ptp_wiring_verified}"
  echo "time_sync_status=\${time_sync_status}"
  echo "time_sync_timestamp=\${time_sync_timestamp}"
  echo "capture_status=\${capture_status}"
  echo "time_status_topic=\${time_status_topic}"
  echo "pps_topic=\${pps_topic}"
  echo "pps_status=\${pps_status}"
  echo "clock_offset_status=\${clock_offset_status}"
  echo "pps_jitter_ms=\${pps_jitter_ms}"
  echo "mean_offset_ms=\${mean_offset_ms}"
  echo "time_sync_raw=\${time_sync_raw}"
  echo "time_sync_raw_status=\${time_sync_raw_status}"
  echo "wiring_confirmation=\${wiring_confirmation}"
  echo "wiring_confirmation_overall=\${wiring_confirmation_overall}"
  echo "wiring_confirmation_keys_status=\${wiring_confirmation_keys_status}"
  echo "wiring_confirmation_source=\${wiring_confirmation_source}"
  echo "pps_wiring_verified=\${pps_wiring_verified}"
  echo "ptp_wiring_verified=\${ptp_wiring_verified}"
  echo "wiring_verified_by=\${wiring_verified_by}"
  echo "wiring_verified_at=\${wiring_verified_at}"
  echo "timedatectl_status=\${timedatectl_status}"
  echo "chronyc_status=\${chronyc_status}"
  echo "ptp_status=\${ptp_status}"
  echo "time_sync_report=\${time_sync_report}"
} > "\${target}"
echo "\${target}"
if [[ "\${pps_ptp_wiring_verified}" != "PASS" ]]; then
  exit 5
fi
EOF
chmod +x "${COMMAND_DIR}/capture_pps_ptp_wiring.sh"

cat > "${COMMAND_DIR}/capture_runtime_health.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="\${RUNTIME_DIR:-${RUNTIME_DIR}}"
target="${LOG_DIR}/runtime_health_latest.txt"
mkdir -p "${LOG_DIR}"
if [[ -n "\${runtime_dir}" && -x "\${runtime_dir}/commands/runtime_health.sh" ]]; then
  report_path=\$("\${runtime_dir}/commands/runtime_health.sh")
  if [[ -s "\${report_path}" ]]; then
    cp -f "\${report_path}" "\${target}"
    echo "\${target}"
    exit 0
  fi
  if [[ -s "\${runtime_dir}/logs/runtime_health_latest.txt" ]]; then
    cp -f "\${runtime_dir}/logs/runtime_health_latest.txt" "\${target}"
    echo "\${target}"
    exit 0
  fi
fi
cat > "\${target}" <<'HEALTH'
capture_status=UNAVAILABLE
HEALTH
echo "\${target}"
EOF
chmod +x "${COMMAND_DIR}/capture_runtime_health.sh"

cat > "${COMMAND_DIR}/capture_runtime_deployment.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="\${RUNTIME_DIR:-${RUNTIME_DIR}}"
target="${LOG_DIR}/runtime_deployment_check.txt"
mkdir -p "${LOG_DIR}"
if [[ -n "\${runtime_dir}" && -x "\${runtime_dir}/commands/runtime_deployment_check.sh" ]]; then
  report_path=\$("\${runtime_dir}/commands/runtime_deployment_check.sh")
  if [[ -s "\${report_path}" ]]; then
    cp -f "\${report_path}" "\${target}"
    echo "\${target}"
    exit 0
  fi
  if [[ -s "\${runtime_dir}/logs/runtime_deployment_check.txt" ]]; then
    cp -f "\${runtime_dir}/logs/runtime_deployment_check.txt" "\${target}"
    echo "\${target}"
    exit 0
  fi
fi
cat > "\${target}" <<'DEPLOYMENT'
deployment_status=FAIL
capture_status=UNAVAILABLE
DEPLOYMENT
echo "\${target}"
EOF
chmod +x "${COMMAND_DIR}/capture_runtime_deployment.sh"

cat > "${COMMAND_DIR}/run_runtime_stability.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="\${RUNTIME_DIR:-${RUNTIME_DIR}}"
samples="\${RUNTIME_STABILITY_SAMPLES:-${RUNTIME_STABILITY_SAMPLES}}"
interval="\${RUNTIME_STABILITY_INTERVAL:-${RUNTIME_STABILITY_INTERVAL}}"
run_log="${LOG_DIR}/runtime_stability_run.log"
command_output_log="${LOG_DIR}/runtime_stability_command_output.log"
capture_marker="${LOG_DIR}/runtime_stability_capture_in_progress"
mkdir -p "${LOG_DIR}"
: > "\${command_output_log}"
{
  echo "started_at=\$(date --iso-8601=seconds)"
  echo "runtime_dir=\${runtime_dir}"
  echo "samples=\${samples}"
  echo "interval=\${interval}"
  echo "command_output_log=\${command_output_log}"
} > "\${run_log}"
if [[ -z "\${runtime_dir}" ]]; then
  echo "runtime_dir is not configured" | tee -a "\${command_output_log}" >&2
  echo "exit_status=2" >> "\${run_log}"
  exit 2
fi
if [[ ! -x "\${runtime_dir}/commands/runtime_stability_check.sh" ]]; then
  echo "runtime_stability_check.sh is not executable under \${runtime_dir}/commands" | tee -a "\${command_output_log}" >&2
  echo "exit_status=2" >> "\${run_log}"
  exit 2
fi
set +e
{
  echo "[runtime_stability_check]"
  "\${runtime_dir}/commands/runtime_stability_check.sh" --samples "\${samples}" --interval "\${interval}"
} >> "\${command_output_log}" 2>&1
status=\$?
set -e
echo "exit_status=\${status}" >> "\${run_log}"
if [[ "\${status}" -ne 0 ]]; then
  exit "\${status}"
fi
capture_token="\$(date +%s%N)-\$\$"
echo "capture_token=\${capture_token}" >> "\${run_log}"
printf 'run_log=%s\ncapture_token=%s\n' "\${run_log}" "\${capture_token}" > "\${capture_marker}"
set +e
{
  echo "[capture_runtime_stability]"
  RUN_RUNTIME_STABILITY_CAPTURE_MARKER="\${capture_marker}" "${COMMAND_DIR}/capture_runtime_stability.sh"
} >> "\${command_output_log}" 2>&1
capture_status=\$?
set -e
rm -f "\${capture_marker}"
echo "capture_exit_status=\${capture_status}" >> "\${run_log}"
if [[ "\${capture_status}" -ne 0 ]]; then
  exit "\${capture_status}"
fi
echo "finished_at=\$(date --iso-8601=seconds)" >> "\${run_log}"
EOF
chmod +x "${COMMAND_DIR}/run_runtime_stability.sh"

cat > "${COMMAND_DIR}/capture_runtime_stability.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="\${RUNTIME_DIR:-${RUNTIME_DIR}}"
expected_samples="\${RUNTIME_STABILITY_SAMPLES:-${RUNTIME_STABILITY_SAMPLES}}"
expected_interval="\${RUNTIME_STABILITY_INTERVAL:-${RUNTIME_STABILITY_INTERVAL}}"
target_csv="${LOG_DIR}/runtime_stability.csv"
target_summary="${LOG_DIR}/runtime_stability_summary.txt"
run_log="${LOG_DIR}/runtime_stability_run.log"
capture_marker="\${RUN_RUNTIME_STABILITY_CAPTURE_MARKER:-}"
mkdir -p "${LOG_DIR}"

line_value() {
  local file="\$1"
  local key="\$2"
  if [[ -s "\${file}" ]]; then
    awk -v key="\${key}" '
      function trim_value(value) {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
        return value
      }
      {
        split_pos = index(\$0, "=")
        if (split_pos == 0) {
          next
        }
        parsed_key = trim_value(substr(\$0, 1, split_pos - 1))
        if (parsed_key == key) {
          ++count
          value = substr(\$0, split_pos + 1)
        }
      }
      END { if (count == 1) print value; else if (count > 1) print "__DUPLICATE_KEY__" }
    ' "\${file}"
  fi
}

line_keys_unique() {
  local file="\$1"
  [[ -s "\${file}" ]] && awk '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    {
      split_pos = index(\$0, "=")
      if (\$0 ~ /^[[:space:]]*$/) {
        next
      }
      if (split_pos == 0) {
        malformed = 1
        next
      }
      key = trim_value(substr(\$0, 1, split_pos - 1))
      if (key == "") {
        malformed = 1
        next
      }
      if (seen[key]) {
        duplicate = 1
      }
      seen[key] = 1
    }
    END { exit((malformed || duplicate) ? 1 : 0) }
  ' "\${file}"
}

write_failed_capture() {
  local capture_status="\$1"
  cat > "\${target_csv}" <<'STABILITY_CSV'
sample,timestamp,disk_guard_status,watchdog_status,health_report
STABILITY_CSV
  cat > "\${target_summary}" <<STABILITY_SUMMARY
timestamp=\$(date --iso-8601=seconds)
overall=FAIL
capture_status=\${capture_status}
samples=0
interval_s=0
disk_failures=0
watchdog_failures=0
health_failures=0
STABILITY_SUMMARY
  echo "\${target_summary}"
}

if [[ ! -s "\${run_log}" ]]; then
  write_failed_capture "RUN_LOG_MISSING"
  exit 5
fi

if ! line_keys_unique "\${run_log}"; then
  write_failed_capture "RUN_LOG_FAILED"
  exit 5
fi

run_log_runtime_dir=\$(line_value "\${run_log}" "runtime_dir")
run_log_samples=\$(line_value "\${run_log}" "samples")
run_log_interval=\$(line_value "\${run_log}" "interval")
run_log_exit_status=\$(line_value "\${run_log}" "exit_status")
run_log_capture_exit_status=\$(line_value "\${run_log}" "capture_exit_status")
run_log_finished_at=\$(line_value "\${run_log}" "finished_at")
run_log_capture_token=\$(line_value "\${run_log}" "capture_token")
capture_marker_status="FAIL"
if [[ -n "\${capture_marker}" && -s "\${capture_marker}" ]]; then
  if line_keys_unique "\${capture_marker}"; then
    capture_marker_run_log=\$(line_value "\${capture_marker}" "run_log")
    capture_marker_token=\$(line_value "\${capture_marker}" "capture_token")
    if [[ "\${capture_marker_run_log}" == "\${run_log}" && \\
          "\${run_log_capture_token}" =~ ^[0-9]+-[0-9]+\$ && \\
          "\${capture_marker_token}" == "\${run_log_capture_token}" ]]; then
      capture_marker_status="PASS"
    fi
  fi
fi
if [[ "\${run_log_runtime_dir}" != "\${runtime_dir}" || \\
      "\${run_log_samples}" != "\${expected_samples}" || \\
      "\${run_log_interval}" != "\${expected_interval}" ]]; then
  write_failed_capture "RUN_LOG_MISMATCH"
  exit 5
fi
if [[ "\${run_log_exit_status}" != "0" || \\
      ( -n "\${run_log_capture_exit_status}" && \\
        "\${run_log_capture_exit_status}" != "0" ) || \\
      ( -z "\${run_log_capture_exit_status}" && \\
        ( -n "\${run_log_finished_at}" || \\
          "\${capture_marker_status}" != "PASS" ) ) ]]; then
  write_failed_capture "RUN_LOG_FAILED"
  exit 5
fi

is_safe_health_report_filename() {
  local value="\$1"
  [[ -n "\${value}" ]] && \\
    [[ "\${value}" != "." ]] && \\
    [[ "\${value}" != ".." ]] && \\
    [[ "\${value}" != *"/"* ]] && \\
    [[ "\${value}" != *\\\\* ]] && \\
    [[ "\${value}" != *","* ]] && \\
    [[ "\${value}" != *";"* ]] && \\
    [[ "\${value}" != *\$'\\r'* ]] && \\
    [[ "\${value}" != *\$'\\n'* ]]
}

archive_runtime_stability_health_reports() {
  local source_csv="\$1"
  local target_csv="\$2"
  local source_csv_dir
  local target_csv_dir
  source_csv_dir=\$(cd "\$(dirname "\${source_csv}")" && pwd)
  target_csv_dir=\$(cd "\$(dirname "\${target_csv}")" && pwd)
  local temp_csv="\${target_csv}.tmp"
  local archive_failed=0
  local header_seen=0
  : > "\${temp_csv}"
  local line
  while IFS= read -r line || [[ -n "\${line}" ]]; do
    if ((header_seen == 0)); then
      echo "\${line}" >> "\${temp_csv}"
      header_seen=1
      continue
    fi
    if [[ "\${line}" =~ ^[[:space:]]*\$ ]]; then
      echo "\${line}" >> "\${temp_csv}"
      continue
    fi
    local sample
    local timestamp
    local disk_status
    local watchdog_status
    local health_report
    local extra
    IFS=, read -r sample timestamp disk_status watchdog_status health_report extra <<< "\${line}"
    if [[ -n "\${extra}" || -z "\${health_report}" ]]; then
      echo "\${line}" >> "\${temp_csv}"
      archive_failed=1
      continue
    fi
    local source_health_report
    if [[ "\${health_report}" == /* ]]; then
      source_health_report="\${health_report}"
    else
      source_health_report="\${source_csv_dir}/\${health_report}"
    fi
    local health_basename
    health_basename=\$(basename "\${health_report}")
    if is_safe_health_report_filename "\${health_basename}" && \\
       [[ -s "\${source_health_report}" && -f "\${source_health_report}" ]]; then
      cp -f "\${source_health_report}" "\${target_csv_dir}/\${health_basename}"
      health_report="\${health_basename}"
    else
      archive_failed=1
    fi
    echo "\${sample},\${timestamp},\${disk_status},\${watchdog_status},\${health_report}" >> "\${temp_csv}"
  done < "\${source_csv}"
  mv -f "\${temp_csv}" "\${target_csv}"
  return "\${archive_failed}"
}

if [[ -n "\${runtime_dir}" && -s "\${runtime_dir}/logs/runtime_stability.csv" ]]; then
  archive_runtime_stability_health_reports \\
    "\${runtime_dir}/logs/runtime_stability.csv" "\${target_csv}" || true
else
  cat > "\${target_csv}" <<'STABILITY_CSV'
sample,timestamp,disk_guard_status,watchdog_status,health_report
STABILITY_CSV
fi
if [[ -n "\${runtime_dir}" && -s "\${runtime_dir}/logs/runtime_stability_summary.txt" ]]; then
  cp -f "\${runtime_dir}/logs/runtime_stability_summary.txt" "\${target_summary}"
else
  cat > "\${target_summary}" <<'STABILITY_SUMMARY'
timestamp=missing
overall=FAIL
capture_status=UNAVAILABLE
samples=0
STABILITY_SUMMARY
fi
echo "\${target_summary}"
EOF
chmod +x "${COMMAND_DIR}/capture_runtime_stability.sh"

cat > "${COMMAND_DIR}/capture_section_export.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
target="${REPORT_DIR}/section_export.csv"
service_name="\${SECTION_EXPORT_SERVICE:-/section/export}"
min_chainage="\${SECTION_EXPORT_MIN_CHAINAGE:--1000000000}"
max_chainage="\${SECTION_EXPORT_MAX_CHAINAGE:-1000000000}"
min_quality="\${SECTION_EXPORT_MIN_QUALITY:-C}"
expected_session_id="${NAME}"
section_export_source_valid() {
  local source="\$1"
  awk -F',' -v expected_session_id="\${expected_session_id}" '
    BEGIN {
      header = "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points"
      rows = 0
      invalid = 0
    }
    /^[[:space:]]*$/ { next }
    NR == 1 {
      if (\$0 != header) {
        invalid = 1
        exit
      }
      next
    }
    {
      if (NF != 7) {
        invalid = 1
        exit
      }
      for (i = 1; i <= NF; ++i) {
        if (\$i == "" || \$i ~ /^[[:space:]]/ || \$i ~ /[[:space:]]\$/) {
          invalid = 1
          exit
        }
      }
      if (\$1 != expected_session_id) {
        invalid = 1
        exit
      }
      if (\$2 !~ /^-?[0-9]+([.][0-9]+)?\$/ ||
          \$3 !~ /^(IDLE_STATIC|CUTTING_STATIC|FWD_MOVE|REV_MOVE|TURNING|CMD_MOVE_NO_DISP|CONFLICT|RELOCALIZING)\$/ ||
          \$4 !~ /^[ABC]\$/ ||
          \$5 !~ /^[0-9]+([.][0-9]+)?\$/ ||
          \$6 !~ /^[0-9]+([.][0-9]+)?\$/ ||
          \$7 !~ /^[1-9][0-9]*\$/) {
        invalid = 1
        exit
      }
      if (\$5 + 0 < 0 || \$5 + 0 > 1 || \$6 + 0 < 0) {
        invalid = 1
        exit
      }
      ++rows
    }
    END {
      exit invalid == 0 && rows > 0 ? 0 : 1
    }
  ' "\${source}"
}
mkdir -p "${REPORT_DIR}" "${LOG_DIR}"
if [[ -n "\${SECTION_EXPORT_SOURCE:-}" && -s "\${SECTION_EXPORT_SOURCE}" ]]; then
  if ! section_export_source_valid "\${SECTION_EXPORT_SOURCE}"; then
    cat > "\${target}" <<'SECTION_EXPORT'
session_id,chainage_m,state_source,quality,completeness,rmse_mm,points
SECTION_EXPORT
    echo "\${target}"
    exit 5
  fi
  cp -f "\${SECTION_EXPORT_SOURCE}" "\${target}"
  echo "\${target}"
  exit 0
fi
if command -v rosservice >/dev/null 2>&1; then
  if timeout 10 rosservice call "\${service_name}" \\
      "min_chainage_m: \${min_chainage}
max_chainage_m: \${max_chainage}
min_quality: '\${min_quality}'
output_path: '\${target}'" > "${LOG_DIR}/section_export_service.log" 2>&1; then
    if [[ -s "\${target}" ]] && section_export_source_valid "\${target}"; then
      echo "\${target}"
      exit 0
    fi
  fi
fi
cat > "\${target}" <<'SECTION_EXPORT'
session_id,chainage_m,state_source,quality,completeness,rmse_mm,points
SECTION_EXPORT
echo "\${target}"
exit 5
EOF
chmod +x "${COMMAND_DIR}/capture_section_export.sh"

cat > "${COMMAND_DIR}/capture_power_loss_resume.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
target="${REPORT_DIR}/power_loss_resume_verified.txt"
confirmation_file="${REPORT_DIR}/power_loss_resume_confirmation.txt"
metrics_report="${REPORT_DIR}/validation_metrics_report.txt"
max_recovery_time_s="\${FIELD_ACCEPTANCE_MAX_RECOVERY_TIME_S:-45}"
mkdir -p "${REPORT_DIR}"

line_value() {
  local file="\$1"
  local key="\$2"
  if [[ -s "\${file}" ]]; then
    awk -v key="\${key}" '
      function trim_value(value) {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
        return value
      }
      {
        split_pos = index(\$0, "=")
        if (split_pos == 0) {
          next
        }
        parsed_key = trim_value(substr(\$0, 1, split_pos - 1))
        if (parsed_key == key) {
          ++count
          value = substr(\$0, split_pos + 1)
        }
      }
      END { if (count == 1) print value; else if (count > 1) print "__DUPLICATE_KEY__" }
    ' "\${file}"
  fi
}

line_keys_unique() {
  local file="\$1"
  [[ -s "\${file}" ]] && awk '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    {
      split_pos = index(\$0, "=")
      if (\$0 ~ /^[[:space:]]*$/) {
        next
      }
      if (split_pos == 0) {
        malformed = 1
        next
      }
      key = trim_value(substr(\$0, 1, split_pos - 1))
      if (key == "") {
        malformed = 1
        next
      }
      if (seen[key]) {
        duplicate = 1
      }
      seen[key] = 1
    }
    END { exit((malformed || duplicate) ? 1 : 0) }
  ' "\${file}"
}

is_valid_text_value() {
  local value="\$1"
  [[ -n "\${value}" ]] && \\
    [[ ! "\${value}" =~ ^[[:space:]] ]] && \\
    [[ ! "\${value}" =~ [[:space:]]\$ ]] && \\
    [[ "\${value}" != "missing" ]] && \\
    [[ "\${value}" != "__DUPLICATE_KEY__" ]] && \\
    [[ "\${value}" != *";"* ]] && \\
    [[ "\${value}" != *\$'\\r'* ]] && \\
    [[ "\${value}" != *\$'\\n'* ]]
}

is_leap_year() {
  local year="\$1"
  (( (10#\${year} % 4 == 0 && 10#\${year} % 100 != 0) || 10#\${year} % 400 == 0 ))
}

days_in_month() {
  local year="\$1"
  local month="\$2"
  case "\${month}" in
    01|03|05|07|08|10|12) echo 31 ;;
    04|06|09|11) echo 30 ;;
    02)
      if is_leap_year "\${year}"; then
        echo 29
      else
        echo 28
      fi
      ;;
    *) echo 0 ;;
  esac
}

is_iso8601_seconds_timestamp() {
  local value="\$1"
  is_valid_text_value "\${value}" || return 1
  [[ "\${value}" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}(Z|[+-][0-9]{2}:[0-9]{2})\$ ]] || return 1
  local year="\${value:0:4}"
  local month="\${value:5:2}"
  local day="\${value:8:2}"
  local hour="\${value:11:2}"
  local minute="\${value:14:2}"
  local second="\${value:17:2}"
  local max_day
  max_day=\$(days_in_month "\${year}" "\${month}")
  local offset_hour="0"
  local offset_minute="0"
  if [[ "\${value:19:1}" != "Z" ]]; then
    offset_hour="\${value:20:2}"
    offset_minute="\${value:23:2}"
  fi
  ((10#\${year} > 0 &&
    10#\${month} >= 1 && 10#\${month} <= 12 &&
    \${max_day} > 0 &&
    10#\${day} >= 1 && 10#\${day} <= \${max_day} &&
    10#\${hour} <= 23 &&
    10#\${minute} <= 59 &&
    10#\${second} <= 59 &&
    10#\${offset_hour} <= 23 &&
    10#\${offset_minute} <= 59))
}

is_nonnegative_number() {
  local value="\$1"
  [[ "\${value}" =~ ^[+-]?(([0-9]+(\\.[0-9]*)?)|(\\.[0-9]+))([eE][+-]?[0-9]+)?\$ ]] && \\
    awk -v value="\${value}" 'BEGIN { exit(value >= 0 ? 0 : 1) }'
}

is_positive_number() {
  local value="\$1"
  is_nonnegative_number "\${value}" && awk -v value="\${value}" 'BEGIN { exit(value > 0 ? 0 : 1) }'
}

recovery_time_within_limit() {
  local recovery="\$1"
  local max_recovery="\$2"
  is_nonnegative_number "\${recovery}" && \\
    is_positive_number "\${max_recovery}" && \\
    awk -v recovery="\${recovery}" -v max_recovery="\${max_recovery}" 'BEGIN { exit(recovery <= max_recovery ? 0 : 1) }'
}

numbers_equal() {
  local left="\$1"
  local right="\$2"
  is_number "\${left}" && is_number "\${right}" && \\
    awk -v left="\${left}" -v right="\${right}" 'BEGIN { exit(left == right ? 0 : 1) }'
}

metrics_report_passed() {
  local file="\$1"
  local expected_session="${NAME}"
  local expected_scenario="${SCENARIO}"
  [[ -s "\${file}" ]] && awk \\
    -v expected_session="\${expected_session}" \\
    -v expected_scenario="\${expected_scenario}" '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    function clear_values(key) {
      for (key in values) {
        delete values[key]
      }
      for (key in seen_key) {
        delete seen_key[key]
      }
    }
    function parse_record(line, count, i, split_pos, key, value) {
      clear_values()
      count = split(line, tokens, ";")
      for (i = 1; i <= count; ++i) {
        split_pos = index(tokens[i], "=")
        if (split_pos == 0) {
          return 0
        }
        key = trim_value(substr(tokens[i], 1, split_pos - 1))
        value = substr(tokens[i], split_pos + 1)
        if (key == "" || seen_key[key]) {
          return 0
        }
        seen_key[key] = 1
        values[key] = value
      }
      return 1
    }
    \$0 ~ /^[[:space:]]*$/ { next }
    !seen_summary {
      seen_summary = 1
      if (!parse_record(\$0)) {
        bad = 1
        next
      }
      if (values["overall"] == "PASS" &&
          values["total_records"] ~ /^[1-9][0-9]*$/ &&
          values["failed_records"] ~ /^0$/) {
        summary_ok = 1
      }
      next
    }
    \$0 == "---" { next }
    {
      if (index(\$0, ";") == 0) {
        next
      }
      if (!parse_record(\$0)) {
        bad = 1
        next
      }
      if (values["session"] == expected_session &&
          values["scenario"] == expected_scenario &&
          values["status"] == "PASS" &&
          values["failed_checks"] ~ /^0$/) {
        matching_record_ok = 1
      }
    }
    END { exit(!bad && summary_ok && matching_record_ok ? 0 : 1) }
  ' "\${file}"
}

metrics_report_recovery_time() {
  local file="\$1"
  local expected_session="${NAME}"
  local expected_scenario="${SCENARIO}"
  [[ -s "\${file}" ]] && awk \\
    -v expected_session="\${expected_session}" \\
    -v expected_scenario="\${expected_scenario}" '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    function clear_values(key) {
      for (key in values) {
        delete values[key]
      }
      for (key in seen_key) {
        delete seen_key[key]
      }
    }
    function parse_record(line, count, i, split_pos, key, value) {
      clear_values()
      count = split(line, tokens, ";")
      for (i = 1; i <= count; ++i) {
        split_pos = index(tokens[i], "=")
        if (split_pos == 0) {
          return 0
        }
        key = trim_value(substr(tokens[i], 1, split_pos - 1))
        value = substr(tokens[i], split_pos + 1)
        if (key == "" || seen_key[key]) {
          return 0
        }
        seen_key[key] = 1
        values[key] = value
      }
      return 1
    }
    function reset_block() {
      matching_record = 0
      recovery_time = ""
      recovery_count = 0
      block_bad = 0
    }
    function emit_matching_recovery() {
      if (!found && matching_record && !block_bad) {
        if (recovery_count == 1) {
          print recovery_time
          found = 1
        } else if (recovery_count > 1) {
          print "__DUPLICATE_KEY__"
          found = 1
        }
      }
    }
    BEGIN { reset_block() }
    \$0 ~ /^[[:space:]]*$/ { next }
    !seen_summary {
      seen_summary = 1
      next
    }
    \$0 == "---" {
      emit_matching_recovery()
      reset_block()
      next
    }
    {
      if (index(\$0, ";") > 0) {
        if (!parse_record(\$0)) {
          if (matching_record) {
            block_bad = 1
          }
          next
        }
        emit_matching_recovery()
        reset_block()
        if (values["session"] == expected_session &&
            values["scenario"] == expected_scenario &&
            values["status"] == "PASS" &&
            values["failed_checks"] ~ /^0$/) {
          matching_record = 1
        }
        next
      }
      if (matching_record && !block_bad) {
        split_pos = index(\$0, "=")
        if (split_pos > 0) {
          key = trim_value(substr(\$0, 1, split_pos - 1))
          value = substr(\$0, split_pos + 1)
          if (key == "recovery_time_s") {
            recovery_time = value
            ++recovery_count
          }
        }
      }
    }
    END {
      emit_matching_recovery()
      exit(found ? 0 : 1)
    }
  ' "\${file}"
}

power_loss_resume_status="FAIL"
power_loss_resume_source="missing"
recovery_time_s="missing"
resume_verified_by="missing"
resume_verified_at="missing"
manual_confirmation_keys_status="missing"
manual_confirmation_overall=\$(line_value "\${confirmation_file}" "power_loss_resume_status")
[[ -n "\${manual_confirmation_overall}" ]] || manual_confirmation_overall="missing"
if [[ "\${POWER_LOSS_RESUME_VERIFIED:-FAIL}" == "PASS" ]]; then
  power_loss_resume_source="manual_env"
fi
if [[ -s "\${confirmation_file}" ]]; then
  power_loss_resume_source="manual_file"
  manual_confirmation_keys_status="FAIL"
  if line_keys_unique "\${confirmation_file}"; then
    manual_confirmation_keys_status="PASS"
  fi
  recovery_time_s=\$(line_value "\${confirmation_file}" "recovery_time_s")
  resume_verified_by=\$(line_value "\${confirmation_file}" "resume_verified_by")
  resume_verified_at=\$(line_value "\${confirmation_file}" "resume_verified_at")
  [[ -n "\${recovery_time_s}" ]] || recovery_time_s="missing"
  [[ -n "\${resume_verified_by}" ]] || resume_verified_by="missing"
  [[ -n "\${resume_verified_at}" ]] || resume_verified_at="missing"
  if [[ "\${manual_confirmation_keys_status}" == "PASS" ]] && \\
     [[ "\${manual_confirmation_overall}" == "PASS" ]] && \\
     [[ "\${recovery_time_s}" != "missing" ]] && \\
     is_valid_text_value "\${resume_verified_by}" && \\
     is_iso8601_seconds_timestamp "\${resume_verified_at}" && \\
     recovery_time_within_limit "\${recovery_time_s}" "\${max_recovery_time_s}"; then
    power_loss_resume_status="PASS"
  else
    power_loss_resume_status="FAIL"
  fi
elif [[ -s "\${metrics_report}" ]]; then
  recovery_time_s=\$(metrics_report_recovery_time "\${metrics_report}" || true)
  if metrics_report_passed "\${metrics_report}" && \\
     [[ -n "\${recovery_time_s}" ]] && \\
     recovery_time_within_limit "\${recovery_time_s}" "\${max_recovery_time_s}"; then
    power_loss_resume_status="PASS"
    power_loss_resume_source=metrics_report
  else
    power_loss_resume_status="FAIL"
    power_loss_resume_source=metrics_report
    [[ -n "\${recovery_time_s}" ]] || recovery_time_s="missing"
  fi
fi

{
  echo "timestamp=\$(date --iso-8601=seconds)"
  echo "power_loss_resume_status=\${power_loss_resume_status}"
  echo "power_loss_resume_confirmation_overall=\${manual_confirmation_overall}"
  echo "power_loss_resume_confirmation_keys_status=\${manual_confirmation_keys_status}"
  echo "power_loss_resume_source=\${power_loss_resume_source}"
  echo "recovery_time_s=\${recovery_time_s}"
  echo "max_recovery_time_s=\${max_recovery_time_s}"
  echo "resume_verified_by=\${resume_verified_by}"
  echo "resume_verified_at=\${resume_verified_at}"
  echo "metrics_report=\${metrics_report}"
} > "\${target}"
echo "\${target}"
if [[ "\${power_loss_resume_status}" != "PASS" ]]; then
  exit 5
fi
EOF
chmod +x "${COMMAND_DIR}/capture_power_loss_resume.sh"

cat > "${COMMAND_DIR}/capture_field_acceptance.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
# Required PASS contract: event_file_status=PASS time_sync_status=PASS metrics_status=PASS runtime_health_status=PASS runtime_deployment_status=PASS runtime_stability_status=PASS power_loss_resume_status=PASS pps_ptp_wiring_verified=PASS section_export_status=PASS field_acceptance_status=PASS
target="${REPORT_DIR}/field_acceptance_report.txt"
time_sync_report="${LOG_DIR}/time_sync_status.txt"
deployment_report="${LOG_DIR}/runtime_deployment_check.txt"
runtime_health_report="${LOG_DIR}/runtime_health_latest.txt"
stability_summary="${LOG_DIR}/runtime_stability_summary.txt"
stability_csv="${LOG_DIR}/runtime_stability.csv"
stability_run_log="${LOG_DIR}/runtime_stability_run.log"
metrics_report="${REPORT_DIR}/validation_metrics_report.txt"
event_file="${REPORT_DIR}/replay_events.txt"
power_loss_report="${REPORT_DIR}/power_loss_resume_verified.txt"
section_export_report="${REPORT_DIR}/section_export.csv"
session_dir="${SESSION_DIR}"
min_duration_h="\${FIELD_ACCEPTANCE_MIN_STABILITY_HOURS:-24}"
max_recovery_time_s="\${FIELD_ACCEPTANCE_MAX_RECOVERY_TIME_S:-45}"
mkdir -p "${REPORT_DIR}"

line_value() {
  local file="\$1"
  local key="\$2"
  if [[ -s "\${file}" ]]; then
    awk -v key="\${key}" '
      function trim_value(value) {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
        return value
      }
      {
        split_pos = index(\$0, "=")
        if (split_pos == 0) {
          next
        }
        parsed_key = trim_value(substr(\$0, 1, split_pos - 1))
        if (parsed_key == key) {
          ++count
          value = substr(\$0, split_pos + 1)
        }
      }
      END { if (count == 1) print value; else if (count > 1) print "__DUPLICATE_KEY__" }
    ' "\${file}"
  fi
}

line_keys_unique() {
  local file="\$1"
  [[ -s "\${file}" ]] && awk '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    {
      split_pos = index(\$0, "=")
      if (\$0 ~ /^[[:space:]]*$/) {
        next
      }
      if (split_pos == 0) {
        malformed = 1
        next
      }
      key = trim_value(substr(\$0, 1, split_pos - 1))
      if (key == "") {
        malformed = 1
        next
      }
      if (seen[key]) {
        duplicate = 1
      }
      seen[key] = 1
    }
    END { exit((malformed || duplicate) ? 1 : 0) }
  ' "\${file}"
}

is_positive_integer() {
  local value="\$1"
  [[ "\${value}" =~ ^[1-9][0-9]*\$ ]]
}

is_zero_integer() {
  local value="\$1"
  [[ "\${value}" =~ ^0\$ ]]
}

is_nonnegative_integer() {
  local value="\$1"
  is_zero_integer "\${value}" || is_positive_integer "\${value}"
}

is_number() {
  local value="\$1"
  [[ "\${value}" =~ ^[+-]?(([0-9]+(\\.[0-9]*)?)|(\\.[0-9]+))([eE][+-]?[0-9]+)?\$ ]]
}

is_nonnegative_number() {
  local value="\$1"
  is_number "\${value}" && awk -v value="\${value}" 'BEGIN { exit(value >= 0 ? 0 : 1) }'
}

is_valid_text_value() {
  local value="\$1"
  [[ -n "\${value}" ]] && \\
    [[ ! "\${value}" =~ ^[[:space:]] ]] && \\
    [[ ! "\${value}" =~ [[:space:]]\$ ]] && \\
    [[ "\${value}" != "missing" ]] && \\
    [[ "\${value}" != "__DUPLICATE_KEY__" ]] && \\
    [[ "\${value}" != *";"* ]] && \\
    [[ "\${value}" != *\$'\\r'* ]] && \\
    [[ "\${value}" != *\$'\\n'* ]]
}

is_valid_ros_topic_name() {
  local value="\$1"
  is_valid_text_value "\${value}" && \\
    [[ "\${value}" =~ ^(/|~)?[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*\$ ]]
}

has_dot_path_segment() {
  local value="\$1"
  local segment
  IFS='/' read -r -a path_segments <<< "\${value}"
  for segment in "\${path_segments[@]}"; do
    if [[ "\${segment}" == "." || "\${segment}" == ".." ]]; then
      return 0
    fi
  done
  return 1
}

resolve_time_sync_raw_path() {
  local value="\$1"
  if ! is_valid_text_value "\${value}" || \\
     [[ "\${value}" == *\\\\* ]] || \\
     [[ "\${value}" == *"//"* ]] || \\
     has_dot_path_segment "\${value}"; then
    return 1
  fi
  if [[ "\${value}" == /* ]]; then
    printf '%s\\n' "\${value}"
  else
    printf '%s/%s\\n' "\${session_dir}" "\${value}"
  fi
}

path_within_session() {
  local value="\$1"
  [[ "\${value}" == "\${session_dir}/"* ]]
}

is_valid_relative_artifact_path() {
  local value="\$1"
  local segment
  is_valid_text_value "\${value}" || return 1
  [[ "\${value}" != /* ]] || return 1
  [[ "\${value}" != *\\\\* ]] || return 1
  IFS='/' read -r -a path_segments <<< "\${value}"
  for segment in "\${path_segments[@]}"; do
    if [[ -z "\${segment}" || "\${segment}" == "." || "\${segment}" == ".." ]]; then
      return 1
    fi
  done
  return 0
}

runtime_health_report_content_passes() {
  local report_path="\$1"
  local report_timestamp
  local report_runtime_dir
  local report_disk_available_gb
  local report_runtime_pid
  local report_systemd_active
  local report_systemd_active_source
  local report_docker_status
  local report_docker_status_source
  report_timestamp=\$(line_value "\${report_path}" "timestamp")
  report_runtime_dir=\$(line_value "\${report_path}" "runtime_dir")
  report_disk_available_gb=\$(line_value "\${report_path}" "disk_available_gb")
  report_runtime_pid=\$(line_value "\${report_path}" "runtime_pid")
  report_systemd_active=\$(line_value "\${report_path}" "systemd_active")
  report_systemd_active_source=\$(line_value "\${report_path}" "systemd_active_source")
  report_docker_status=\$(line_value "\${report_path}" "docker_container_status")
  report_docker_status_source=\$(line_value "\${report_path}" "docker_container_status_source")
  line_keys_unique "\${report_path}" && \\
    is_iso8601_seconds_timestamp "\${report_timestamp}" && \\
    is_valid_text_value "\${report_runtime_dir}" && \\
    [[ "\${report_runtime_dir}" == "${RUNTIME_DIR}" ]] && \\
    is_nonnegative_number "\${report_disk_available_gb}" && \\
    is_positive_integer "\${report_runtime_pid}" && \\
    [[ "\${report_systemd_active}" == "active" ]] && \\
    [[ "\${report_systemd_active_source}" == "systemctl" ]] && \\
    [[ "\${report_docker_status}" == "running" ]] && \\
    [[ "\${report_docker_status_source}" == "docker_inspect" ]]
}

runtime_stability_csv_health_report_passes() {
  local value="\$1"
  local resolved_path
  if is_valid_relative_artifact_path "\${value}"; then
    resolved_path="\$(dirname "\${stability_csv}")/\${value}"
  elif is_valid_text_value "\${value}" && \\
       [[ "\${value}" == /* ]] && \\
       [[ "\${value}" != *\\\\* ]] && \\
       ! has_dot_path_segment "\${value}" && \\
       path_within_session "\${value}"; then
    resolved_path="\${value}"
  else
    return 1
  fi
  [[ -s "\${resolved_path}" && -f "\${resolved_path}" ]] && \\
    runtime_health_report_content_passes "\${resolved_path}"
}

is_leap_year() {
  local year="\$1"
  (( (10#\${year} % 4 == 0 && 10#\${year} % 100 != 0) || 10#\${year} % 400 == 0 ))
}

days_in_month() {
  local year="\$1"
  local month="\$2"
  case "\${month}" in
    01|03|05|07|08|10|12) echo 31 ;;
    04|06|09|11) echo 30 ;;
    02)
      if is_leap_year "\${year}"; then
        echo 29
      else
        echo 28
      fi
      ;;
    *) echo 0 ;;
  esac
}

is_iso8601_seconds_timestamp() {
  local value="\$1"
  is_valid_text_value "\${value}" || return 1
  [[ "\${value}" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}(Z|[+-][0-9]{2}:[0-9]{2})\$ ]] || return 1
  local year="\${value:0:4}"
  local month="\${value:5:2}"
  local day="\${value:8:2}"
  local hour="\${value:11:2}"
  local minute="\${value:14:2}"
  local second="\${value:17:2}"
  local max_day
  max_day=\$(days_in_month "\${year}" "\${month}")
  local offset_hour="0"
  local offset_minute="0"
  if [[ "\${value:19:1}" != "Z" ]]; then
    offset_hour="\${value:20:2}"
    offset_minute="\${value:23:2}"
  fi
  ((10#\${year} > 0 &&
    10#\${month} >= 1 && 10#\${month} <= 12 &&
    \${max_day} > 0 &&
    10#\${day} >= 1 && 10#\${day} <= \${max_day} &&
    10#\${hour} <= 23 &&
    10#\${minute} <= 59 &&
    10#\${second} <= 59 &&
    10#\${offset_hour} <= 23 &&
    10#\${offset_minute} <= 59))
}

iso8601_seconds_epoch() {
  local value="\$1"
  date -d "\${value}" +%s 2>/dev/null
}

iso8601_seconds_elapsed() {
  local started_at="\$1"
  local finished_at="\$2"
  local started_epoch
  local finished_epoch
  started_epoch=\$(iso8601_seconds_epoch "\${started_at}") || return 1
  finished_epoch=\$(iso8601_seconds_epoch "\${finished_at}") || return 1
  [[ "\${started_epoch}" =~ ^-?[0-9]+\$ ]] || return 1
  [[ "\${finished_epoch}" =~ ^-?[0-9]+\$ ]] || return 1
  echo \$((finished_epoch - started_epoch))
}

iso8601_seconds_finished_not_before_started() {
  local elapsed_s
  elapsed_s=\$(iso8601_seconds_elapsed "\$1" "\$2") || return 1
  [[ "\${elapsed_s}" =~ ^-?[0-9]+\$ ]] || return 1
  ((elapsed_s >= 0))
}

runtime_stability_required_elapsed_s() {
  local samples="\$1"
  local interval_s="\$2"
  is_positive_integer "\${samples}" || return 1
  is_positive_integer "\${interval_s}" || return 1
  local configured_s
  configured_s=\$((10#\${samples} * 10#\${interval_s}))
  local tolerance_s=60
  if ((10#\${interval_s} < tolerance_s)); then
    tolerance_s=\$((10#\${interval_s}))
  fi
  local required_s=\$((configured_s - tolerance_s))
  if ((required_s < 0)); then
    required_s=0
  fi
  echo "\${required_s}"
}

is_positive_number() {
  local value="\$1"
  is_nonnegative_number "\${value}" && awk -v value="\${value}" 'BEGIN { exit(value > 0 ? 0 : 1) }'
}

recovery_time_within_limit() {
  local recovery="\$1"
  local max_recovery="\$2"
  is_nonnegative_number "\${recovery}" && \\
    is_positive_number "\${max_recovery}" && \\
    awk -v recovery="\${recovery}" -v max_recovery="\${max_recovery}" 'BEGIN { exit(recovery <= max_recovery ? 0 : 1) }'
}

numbers_equal() {
  local left="\$1"
  local right="\$2"
  is_number "\${left}" && is_number "\${right}" && \\
    awk -v left="\${left}" -v right="\${right}" 'BEGIN { exit(left == right ? 0 : 1) }'
}

min_duration_status="PASS"
if ! is_number "\${min_duration_h}" || \\
   ! awk -v min_duration="\${min_duration_h}" 'BEGIN { exit(min_duration >= 24.0 ? 0 : 1) }'; then
  min_duration_status="FAIL"
fi

metrics_report_passed() {
  local file="\$1"
  local expected_session="${NAME}"
  local expected_scenario="${SCENARIO}"
  [[ -s "\${file}" ]] && awk \\
    -v expected_session="\${expected_session}" \\
    -v expected_scenario="\${expected_scenario}" '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    function clear_values(key) {
      for (key in values) {
        delete values[key]
      }
      for (key in seen_key) {
        delete seen_key[key]
      }
    }
    function parse_record(line, count, i, split_pos, key, value) {
      clear_values()
      count = split(line, tokens, ";")
      for (i = 1; i <= count; ++i) {
        split_pos = index(tokens[i], "=")
        if (split_pos == 0) {
          return 0
        }
        key = trim_value(substr(tokens[i], 1, split_pos - 1))
        value = substr(tokens[i], split_pos + 1)
        if (key == "" || seen_key[key]) {
          return 0
        }
        seen_key[key] = 1
        values[key] = value
      }
      return 1
    }
    \$0 ~ /^[[:space:]]*$/ { next }
    !seen_summary {
      seen_summary = 1
      if (!parse_record(\$0)) {
        bad = 1
        next
      }
      if (values["overall"] == "PASS" &&
          values["total_records"] ~ /^[1-9][0-9]*$/ &&
          values["failed_records"] ~ /^0$/) {
        summary_ok = 1
      }
      next
    }
    \$0 == "---" { next }
    {
      if (index(\$0, ";") == 0) {
        next
      }
      if (!parse_record(\$0)) {
        bad = 1
        next
      }
      if (values["session"] == expected_session &&
          values["scenario"] == expected_scenario &&
          values["status"] == "PASS" &&
          values["failed_checks"] ~ /^0$/) {
        matching_record_ok = 1
      }
    }
    END { exit(!bad && summary_ok && matching_record_ok ? 0 : 1) }
  ' "\${file}"
}

metrics_report_recovery_time() {
  local file="\$1"
  local expected_session="${NAME}"
  local expected_scenario="${SCENARIO}"
  [[ -s "\${file}" ]] && awk \\
    -v expected_session="\${expected_session}" \\
    -v expected_scenario="\${expected_scenario}" '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    function clear_values(key) {
      for (key in values) {
        delete values[key]
      }
      for (key in seen_key) {
        delete seen_key[key]
      }
    }
    function parse_record(line, count, i, split_pos, key, value) {
      clear_values()
      count = split(line, tokens, ";")
      for (i = 1; i <= count; ++i) {
        split_pos = index(tokens[i], "=")
        if (split_pos == 0) {
          return 0
        }
        key = trim_value(substr(tokens[i], 1, split_pos - 1))
        value = substr(tokens[i], split_pos + 1)
        if (key == "" || seen_key[key]) {
          return 0
        }
        seen_key[key] = 1
        values[key] = value
      }
      return 1
    }
    function reset_block() {
      matching_record = 0
      recovery_time = ""
      recovery_count = 0
      block_bad = 0
    }
    function emit_matching_recovery() {
      if (!found && matching_record && !block_bad) {
        if (recovery_count == 1) {
          print recovery_time
          found = 1
        } else if (recovery_count > 1) {
          print "__DUPLICATE_KEY__"
          found = 1
        }
      }
    }
    BEGIN { reset_block() }
    \$0 ~ /^[[:space:]]*$/ { next }
    !seen_summary {
      seen_summary = 1
      next
    }
    \$0 == "---" {
      emit_matching_recovery()
      reset_block()
      next
    }
    {
      if (index(\$0, ";") > 0) {
        if (!parse_record(\$0)) {
          if (matching_record) {
            block_bad = 1
          }
          next
        }
        emit_matching_recovery()
        reset_block()
        if (values["session"] == expected_session &&
            values["scenario"] == expected_scenario &&
            values["status"] == "PASS" &&
            values["failed_checks"] ~ /^0$/) {
          matching_record = 1
        }
        next
      }
      if (matching_record && !block_bad) {
        split_pos = index(\$0, "=")
        if (split_pos > 0) {
          key = trim_value(substr(\$0, 1, split_pos - 1))
          value = substr(\$0, split_pos + 1)
          if (key == "recovery_time_s") {
            recovery_time = value
            ++recovery_count
          }
        }
      }
    }
    END {
      emit_matching_recovery()
      exit(found ? 0 : 1)
    }
  ' "\${file}"
}

replay_event_file_passed() {
  local file="\$1"
  local expected_session="${NAME}"
  local expected_scenario="${SCENARIO}"
  [[ -s "\${file}" ]] && awk \\
    -v expected_session="\${expected_session}" \\
    -v expected_scenario="\${expected_scenario}" '
    function trim_value(value) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      return value
    }
    function skippable_line(line) {
      line = trim_value(line)
      return line == "" || substr(line, 1, 1) == "#"
    }
    function clear_values(key) {
      for (key in values) {
        delete values[key]
      }
      for (key in seen_key) {
        delete seen_key[key]
      }
    }
    function parse_record(line, count, i, split_pos, key, value) {
      clear_values()
      count = split(line, tokens, ";")
      for (i = 1; i <= count; ++i) {
        split_pos = index(tokens[i], "=")
        if (split_pos == 0) {
          return 0
        }
        key = trim_value(substr(tokens[i], 1, split_pos - 1))
        value = substr(tokens[i], split_pos + 1)
        if (key == "" || seen_key[key]) {
          return 0
        }
        seen_key[key] = 1
        values[key] = value
      }
      return 1
    }
    function is_number(value) {
      return value ~ /^[+-]?(([0-9]+(\\.[0-9]*)?)|(\\.[0-9]+))([eE][+-]?[0-9]+)?$/
    }
    function is_integer(value) {
      return value ~ /^[+-]?[0-9]+$/
    }
    function is_nonnegative_number(value) {
      return is_number(value) && value + 0 >= 0
    }
    function is_nonnegative_integer(value) {
      return is_integer(value) && value + 0 >= 0
    }
    function abs_value(value) {
      return value < 0 ? -value : value
    }
    function max_value(left, right) {
      return left > right ? left : right
    }
    function valid_text_value(value) {
      return value != "" &&
             value !~ /^[[:space:]]/ &&
             value !~ /[[:space:]]$/ &&
             value != "missing" &&
             value != "__DUPLICATE_KEY__" &&
             value !~ /;/ &&
             value !~ /\r/
    }
    function valid_session_token(value) {
      return valid_text_value(value) &&
             value != "." &&
             value != ".." &&
             value ~ /^[A-Za-z0-9_.-]+$/
    }
    function valid_scenario_token(value) {
      return valid_text_value(value) &&
             value ~ /^[A-Z0-9_]+$/
    }
    BEGIN {
      valid = 1
      static_drift_max = 0.0
      length_error_max = 0.0
      recovery_time_max = 0.0
      wrong_loop_sum = 0
      queue_backlog_max = 0
      pps_jitter_max = 0.0
      last_power_loss_time = ""
    }
    skippable_line(\$0) { next }
    {
      if (!parse_record(\$0) ||
          !valid_text_value(values["event"]) ||
          !valid_session_token(values["session_id"]) ||
          !valid_scenario_token(values["scenario"]) ||
          values["t"] == "" ||
          !is_nonnegative_number(values["t"])) {
        valid = 0
        next
      }
      if (values["session_id"] == expected_session &&
          values["scenario"] == expected_scenario) {
        matching = 1
        has_chainage = ("chainage_m" in values)
        has_reference_chainage = ("reference_chainage_m" in values)
        if ((("static_drift_m" in values) &&
             !is_nonnegative_number(values["static_drift_m"])) ||
            (("length_error_percent" in values) &&
             !is_nonnegative_number(values["length_error_percent"])) ||
            ((has_chainage || has_reference_chainage) &&
             (!(has_chainage && has_reference_chainage) ||
              !is_number(values["chainage_m"]) ||
              !is_number(values["reference_chainage_m"]))) ||
            (("wrong_loop" in values) &&
             !is_nonnegative_integer(values["wrong_loop"])) ||
            (("queue_backlog" in values) &&
             !is_nonnegative_integer(values["queue_backlog"])) ||
            (("pps_jitter_ms" in values) &&
             !is_nonnegative_number(values["pps_jitter_ms"]))) {
          valid = 0
          next
        }
        if (("static_drift_m" in values) ||
            ("length_error_percent" in values) ||
            (has_chainage && has_reference_chainage) ||
            ("wrong_loop" in values) ||
            ("queue_backlog" in values) ||
            ("pps_jitter_ms" in values)) {
          has_evidence = 1
        }
        if (values["event"] == "power_loss") {
          has_power_loss = 1
          last_power_loss_time = values["t"] + 0
        } else if ((values["event"] == "recovered" ||
                    values["event"] == "recovery_complete") &&
                   has_power_loss) {
          has_evidence = 1
          recovery_delta = values["t"] + 0 - last_power_loss_time
          recovery_time_max = max_value(recovery_time_max, recovery_delta)
        }
        if ("static_drift_m" in values) {
          static_drift_value = abs_value(values["static_drift_m"] + 0)
          static_drift_max = max_value(static_drift_max, static_drift_value)
        }
        if ("length_error_percent" in values) {
          length_error_value = abs_value(values["length_error_percent"] + 0)
          length_error_max = max_value(length_error_max, length_error_value)
        } else if (has_chainage && has_reference_chainage) {
          chainage_value = values["chainage_m"] + 0
          reference_value = values["reference_chainage_m"] + 0
          reference_abs = abs_value(reference_value)
          denominator = reference_abs > 1.0 ? reference_abs : 1.0
          chainage_error = abs_value(chainage_value - reference_value) / denominator * 100.0
          length_error_max = max_value(length_error_max, chainage_error)
        }
        if ("wrong_loop" in values) {
          wrong_loop_sum += values["wrong_loop"] + 0
        }
        if ("queue_backlog" in values) {
          queue_backlog_value = values["queue_backlog"] + 0
          queue_backlog_max = max_value(queue_backlog_max, queue_backlog_value)
        }
        if ("pps_jitter_ms" in values) {
          pps_jitter_value = abs_value(values["pps_jitter_ms"] + 0)
          pps_jitter_max = max_value(pps_jitter_max, pps_jitter_value)
        }
      }
    }
    END {
      thresholds_ok = 1
      if (static_drift_max > 0.05) {
        thresholds_ok = 0
      }
      if (length_error_max > 0.5) {
        thresholds_ok = 0
      }
      if (recovery_time_max > 45.0) {
        thresholds_ok = 0
      }
      if (wrong_loop_sum > 0) {
        thresholds_ok = 0
      }
      if (queue_backlog_max > 10) {
        thresholds_ok = 0
      }
      if (pps_jitter_max > 2.0) {
        thresholds_ok = 0
      }
      if (valid && matching && has_evidence && thresholds_ok) {
        exit 0
      }
      exit 1
    }
  ' "\${file}"
}

pps_jitter_ms=\$(line_value "\${time_sync_report}" "pps_jitter_ms")
mean_offset_ms=\$(line_value "\${time_sync_report}" "mean_offset_ms")
time_sync_timestamp=\$(line_value "\${time_sync_report}" "timestamp")
time_status_topic=\$(line_value "\${time_sync_report}" "time_status_topic")
pps_topic=\$(line_value "\${time_sync_report}" "pps_topic")
time_sync_reported_status=\$(line_value "\${time_sync_report}" "time_sync_status")
time_capture_status=\$(line_value "\${time_sync_report}" "capture_status")
time_pps_status=\$(line_value "\${time_sync_report}" "pps_status")
time_clock_offset_status=\$(line_value "\${time_sync_report}" "clock_offset_status")
time_sync_raw=\$(line_value "\${time_sync_report}" "raw")
[[ -n "\${pps_jitter_ms}" ]] || pps_jitter_ms="missing"
[[ -n "\${mean_offset_ms}" ]] || mean_offset_ms="missing"
[[ -n "\${time_sync_timestamp}" ]] || time_sync_timestamp="missing"
[[ -n "\${time_status_topic}" ]] || time_status_topic="missing"
[[ -n "\${pps_topic}" ]] || pps_topic="missing"
[[ -n "\${time_sync_reported_status}" ]] || time_sync_reported_status="missing"
[[ -n "\${time_capture_status}" ]] || time_capture_status="missing"
[[ -n "\${time_pps_status}" ]] || time_pps_status="missing"
[[ -n "\${time_clock_offset_status}" ]] || time_clock_offset_status="missing"
[[ -n "\${time_sync_raw}" ]] || time_sync_raw="missing"
time_sync_keys_status="FAIL"
if line_keys_unique "\${time_sync_report}"; then
  time_sync_keys_status="PASS"
fi
time_sync_raw_status="FAIL"
time_sync_raw_path=""
if time_sync_raw_path=\$(resolve_time_sync_raw_path "\${time_sync_raw}") && \\
   path_within_session "\${time_sync_raw_path}" && \\
   [[ -s "\${time_sync_raw_path}" ]]; then
  time_sync_raw_status="PASS"
fi
time_sync_status="FAIL"
if [[ -s "\${time_sync_report}" ]] && \\
   [[ "\${time_sync_keys_status}" == "PASS" ]] && \\
   [[ "\${time_sync_raw_status}" == "PASS" ]] && \\
   [[ "\${time_sync_reported_status}" == "PASS" ]] && \\
   is_iso8601_seconds_timestamp "\${time_sync_timestamp}" && \\
   is_valid_ros_topic_name "\${time_status_topic}" && \\
   is_valid_ros_topic_name "\${pps_topic}" && \\
   [[ "\${time_capture_status}" == "CAPTURED" ]] && \\
   [[ "\${time_pps_status}" == "PASS" ]] && \\
   [[ "\${time_clock_offset_status}" == "PASS" ]] && \\
   is_nonnegative_number "\${pps_jitter_ms}" && \\
   is_number "\${mean_offset_ms}"; then
  time_sync_status="PASS"
fi

deployment_keys_status="FAIL"
if line_keys_unique "\${deployment_report}"; then
  deployment_keys_status="PASS"
fi
deployment_timestamp=\$(line_value "\${deployment_report}" "timestamp")
deployment_overall=\$(line_value "\${deployment_report}" "deployment_status")
deployment_runtime_dir=\$(line_value "\${deployment_report}" "runtime_dir")
systemd_unit_file=\$(line_value "\${deployment_report}" "systemd_unit_file")
systemd_env_file=\$(line_value "\${deployment_report}" "systemd_env_file")
systemd_active=\$(line_value "\${deployment_report}" "systemd_active")
systemd_active_source=\$(line_value "\${deployment_report}" "systemd_active_source")
docker_compose_file=\$(line_value "\${deployment_report}" "docker_compose_file")
docker_env_file=\$(line_value "\${deployment_report}" "docker_env_file")
docker_container_status=\$(line_value "\${deployment_report}" "docker_container_status")
docker_container_status_source=\$(line_value "\${deployment_report}" "docker_container_status_source")
runtime_process_status=\$(line_value "\${deployment_report}" "runtime_process_status")
start_command=\$(line_value "\${deployment_report}" "start_command")
runtime_deployment_status="FAIL"
if [[ -s "\${deployment_report}" ]] && \\
   [[ "\${deployment_keys_status}" == "PASS" ]] && \\
   is_iso8601_seconds_timestamp "\${deployment_timestamp}" && \\
   [[ "\${deployment_overall}" == "PASS" ]] && \\
   is_valid_text_value "\${deployment_runtime_dir}" && \\
   [[ "\${deployment_runtime_dir}" == "${RUNTIME_DIR}" ]] && \\
   [[ "\${systemd_unit_file}" == "PASS" ]] && \\
   [[ "\${systemd_env_file}" == "PASS" ]] && \\
   [[ "\${systemd_active}" == "active" ]] && \\
   [[ "\${systemd_active_source}" == "systemctl" ]] && \\
   [[ "\${docker_compose_file}" == "PASS" ]] && \\
   [[ "\${docker_env_file}" == "PASS" ]] && \\
   [[ "\${docker_container_status}" == "running" ]] && \\
   [[ "\${docker_container_status_source}" == "docker_inspect" ]] && \\
   [[ "\${runtime_process_status}" == "PASS" ]] && \\
   [[ "\${start_command}" == "PASS" ]]; then
  runtime_deployment_status="PASS"
fi
[[ -n "\${deployment_timestamp}" ]] || deployment_timestamp="missing"
[[ -n "\${deployment_overall}" ]] || deployment_overall="missing"
[[ -n "\${deployment_runtime_dir}" ]] || deployment_runtime_dir="missing"
[[ -n "\${systemd_unit_file}" ]] || systemd_unit_file="missing"
[[ -n "\${systemd_env_file}" ]] || systemd_env_file="missing"
[[ -n "\${systemd_active}" ]] || systemd_active="missing"
[[ -n "\${systemd_active_source}" ]] || systemd_active_source="missing"
[[ -n "\${docker_compose_file}" ]] || docker_compose_file="missing"
[[ -n "\${docker_env_file}" ]] || docker_env_file="missing"
[[ -n "\${docker_container_status}" ]] || docker_container_status="missing"
[[ -n "\${docker_container_status_source}" ]] || docker_container_status_source="missing"
[[ -n "\${runtime_process_status}" ]] || runtime_process_status="missing"
[[ -n "\${start_command}" ]] || start_command="missing"

runtime_health_keys_status="FAIL"
if line_keys_unique "\${runtime_health_report}"; then
  runtime_health_keys_status="PASS"
fi
runtime_health_timestamp=\$(line_value "\${runtime_health_report}" "timestamp")
runtime_health_runtime_dir=\$(line_value "\${runtime_health_report}" "runtime_dir")
runtime_health_disk_available_gb=\$(line_value "\${runtime_health_report}" "disk_available_gb")
runtime_health_pid=\$(line_value "\${runtime_health_report}" "runtime_pid")
runtime_health_systemd_active=\$(line_value "\${runtime_health_report}" "systemd_active")
runtime_health_systemd_active_source=\$(line_value "\${runtime_health_report}" "systemd_active_source")
runtime_health_docker_status=\$(line_value "\${runtime_health_report}" "docker_container_status")
runtime_health_docker_status_source=\$(line_value "\${runtime_health_report}" "docker_container_status_source")
runtime_health_status="FAIL"
if [[ -s "\${runtime_health_report}" ]] && \\
   [[ "\${runtime_health_keys_status}" == "PASS" ]] && \\
   is_iso8601_seconds_timestamp "\${runtime_health_timestamp}" && \\
   is_valid_text_value "\${runtime_health_runtime_dir}" && \\
   [[ "\${runtime_health_runtime_dir}" == "${RUNTIME_DIR}" ]] && \\
   is_nonnegative_number "\${runtime_health_disk_available_gb}" && \\
   is_positive_integer "\${runtime_health_pid}" && \\
   [[ "\${runtime_health_systemd_active}" == "active" ]] && \\
   [[ "\${runtime_health_systemd_active_source}" == "systemctl" ]] && \\
   [[ "\${runtime_health_docker_status}" == "running" ]] && \\
   [[ "\${runtime_health_docker_status_source}" == "docker_inspect" ]]; then
  runtime_health_status="PASS"
fi
[[ -n "\${runtime_health_timestamp}" ]] || runtime_health_timestamp="missing"
[[ -n "\${runtime_health_runtime_dir}" ]] || runtime_health_runtime_dir="missing"
[[ -n "\${runtime_health_disk_available_gb}" ]] || runtime_health_disk_available_gb="missing"
[[ -n "\${runtime_health_pid}" ]] || runtime_health_pid="missing"
[[ -n "\${runtime_health_systemd_active}" ]] || runtime_health_systemd_active="missing"
[[ -n "\${runtime_health_systemd_active_source}" ]] || runtime_health_systemd_active_source="missing"
[[ -n "\${runtime_health_docker_status}" ]] || runtime_health_docker_status="missing"
[[ -n "\${runtime_health_docker_status_source}" ]] || runtime_health_docker_status_source="missing"

runtime_stability_summary_keys_status="FAIL"
if line_keys_unique "\${stability_summary}"; then
  runtime_stability_summary_keys_status="PASS"
fi
samples=\$(line_value "\${stability_summary}" "samples")
interval_s=\$(line_value "\${stability_summary}" "interval_s")
stability_overall=\$(line_value "\${stability_summary}" "overall")
runtime_stability_summary_timestamp=\$(line_value "\${stability_summary}" "timestamp")
disk_failures=\$(line_value "\${stability_summary}" "disk_failures")
watchdog_failures=\$(line_value "\${stability_summary}" "watchdog_failures")
watchdog_skipped=\$(line_value "\${stability_summary}" "watchdog_skipped")
health_failures=\$(line_value "\${stability_summary}" "health_failures")
[[ -n "\${samples}" ]] || samples="missing"
[[ -n "\${interval_s}" ]] || interval_s="missing"
[[ -n "\${stability_overall}" ]] || stability_overall="missing"
[[ -n "\${runtime_stability_summary_timestamp}" ]] || runtime_stability_summary_timestamp="missing"
[[ -n "\${disk_failures}" ]] || disk_failures="missing"
[[ -n "\${watchdog_failures}" ]] || watchdog_failures="missing"
[[ -n "\${watchdog_skipped}" ]] || watchdog_skipped="missing"
[[ -n "\${health_failures}" ]] || health_failures="missing"
runtime_stability_csv_samples="missing"
runtime_stability_csv_status="FAIL"
runtime_stability_sample_count_match="FAIL"
runtime_stability_csv_first_timestamp="missing"
runtime_stability_csv_last_timestamp="missing"
if [[ -s "\${stability_csv}" ]]; then
  runtime_stability_csv_status="PASS"
  runtime_stability_csv_samples=0
  runtime_stability_csv_header_seen=0
  runtime_stability_csv_previous_timestamp_epoch="missing"
  while IFS= read -r runtime_stability_csv_line || [[ -n "\${runtime_stability_csv_line}" ]]; do
    [[ "\${runtime_stability_csv_line}" =~ ^[[:space:]]*\$ ]] && continue
    if ((runtime_stability_csv_header_seen == 0)); then
      if [[ "\${runtime_stability_csv_line}" != "sample,timestamp,disk_guard_status,watchdog_status,health_report" ]]; then
        runtime_stability_csv_status="FAIL"
        break
      fi
      runtime_stability_csv_header_seen=1
      continue
    fi
    runtime_stability_csv_commas="\${runtime_stability_csv_line//[^,]/}"
    if [[ \${#runtime_stability_csv_commas} -ne 4 ]]; then
      runtime_stability_csv_status="FAIL"
      break
    fi
    runtime_stability_csv_samples=\$((runtime_stability_csv_samples + 1))
    IFS=, read -r runtime_stability_csv_sample \\
                  runtime_stability_csv_timestamp \\
                  runtime_stability_csv_disk_status \\
                  runtime_stability_csv_watchdog_status \\
                  runtime_stability_csv_health_report \\
                  runtime_stability_csv_extra <<< "\${runtime_stability_csv_line}"
    runtime_stability_csv_timestamp_epoch=\$(iso8601_seconds_epoch "\${runtime_stability_csv_timestamp}") || runtime_stability_csv_timestamp_epoch="missing"
    if [[ "\${runtime_stability_csv_timestamp_epoch}" =~ ^-?[0-9]+\$ ]]; then
      if [[ "\${runtime_stability_csv_first_timestamp}" == "missing" ]]; then
        runtime_stability_csv_first_timestamp="\${runtime_stability_csv_timestamp}"
      fi
      runtime_stability_csv_last_timestamp="\${runtime_stability_csv_timestamp}"
    fi
    for runtime_stability_csv_field in \\
        "\${runtime_stability_csv_sample}" \\
        "\${runtime_stability_csv_timestamp}" \\
        "\${runtime_stability_csv_disk_status}" \\
        "\${runtime_stability_csv_watchdog_status}" \\
        "\${runtime_stability_csv_health_report}"; do
      if [[ -z "\${runtime_stability_csv_field}" || \\
            "\${runtime_stability_csv_field}" =~ ^[[:space:]] || \\
            "\${runtime_stability_csv_field}" =~ [[:space:]]\$ ]]; then
        runtime_stability_csv_status="FAIL"
      fi
    done
    if [[ -n "\${runtime_stability_csv_extra}" ]] || \\
       ! is_positive_integer "\${runtime_stability_csv_sample}" || \\
       ((10#\${runtime_stability_csv_sample} != runtime_stability_csv_samples)) || \\
       ! is_iso8601_seconds_timestamp "\${runtime_stability_csv_timestamp}" || \\
       ! [[ "\${runtime_stability_csv_timestamp_epoch}" =~ ^-?[0-9]+\$ ]] || \\
       ([[ "\${runtime_stability_csv_previous_timestamp_epoch}" =~ ^-?[0-9]+\$ ]] && \\
        ((runtime_stability_csv_timestamp_epoch < runtime_stability_csv_previous_timestamp_epoch))) || \\
       [[ "\${runtime_stability_csv_disk_status}" != "PASS" ]] || \\
       [[ "\${runtime_stability_csv_watchdog_status}" != "PASS" ]] || \\
       ! is_valid_text_value "\${runtime_stability_csv_health_report}" || \\
       ! runtime_stability_csv_health_report_passes "\${runtime_stability_csv_health_report}"; then
      runtime_stability_csv_status="FAIL"
    fi
    if [[ "\${runtime_stability_csv_status}" != "PASS" ]]; then
      break
    fi
    runtime_stability_csv_previous_timestamp_epoch="\${runtime_stability_csv_timestamp_epoch}"
  done < "\${stability_csv}"
  if ((runtime_stability_csv_header_seen == 0 || runtime_stability_csv_samples == 0)); then
    runtime_stability_csv_status="FAIL"
  fi
  if [[ -z "\${runtime_stability_csv_samples}" ]]; then
    runtime_stability_csv_samples="missing"
  fi
  if [[ "\${runtime_stability_csv_status}" != "PASS" ]]; then
    runtime_stability_csv_status="FAIL"
  fi
  runtime_stability_sample_count_match="FAIL"
  if is_positive_integer "\${samples}" && \\
     is_positive_integer "\${runtime_stability_csv_samples}" && \\
     [[ "\${samples}" == "\${runtime_stability_csv_samples}" ]]; then
    runtime_stability_sample_count_match="PASS"
  fi
fi
runtime_stability_duration_h="0.00"
if is_positive_integer "\${samples}" && is_positive_integer "\${interval_s}"; then
  runtime_stability_duration_h=\$(awk -v samples="\${samples}" -v interval_s="\${interval_s}" 'BEGIN { printf "%.2f", (samples * interval_s) / 3600.0 }')
fi
runtime_stability_run_log_keys_status="FAIL"
if line_keys_unique "\${stability_run_log}"; then
  runtime_stability_run_log_keys_status="PASS"
fi
runtime_stability_run_log_started_at=\$(line_value "\${stability_run_log}" "started_at")
runtime_stability_run_log_finished_at=\$(line_value "\${stability_run_log}" "finished_at")
runtime_stability_run_log_runtime_dir=\$(line_value "\${stability_run_log}" "runtime_dir")
runtime_stability_run_log_samples=\$(line_value "\${stability_run_log}" "samples")
runtime_stability_run_log_interval=\$(line_value "\${stability_run_log}" "interval")
runtime_stability_run_log_exit_status=\$(line_value "\${stability_run_log}" "exit_status")
runtime_stability_run_log_capture_exit_status=\$(line_value "\${stability_run_log}" "capture_exit_status")
[[ -n "\${runtime_stability_run_log_started_at}" ]] || runtime_stability_run_log_started_at="missing"
[[ -n "\${runtime_stability_run_log_finished_at}" ]] || runtime_stability_run_log_finished_at="missing"
[[ -n "\${runtime_stability_run_log_runtime_dir}" ]] || runtime_stability_run_log_runtime_dir="missing"
[[ -n "\${runtime_stability_run_log_samples}" ]] || runtime_stability_run_log_samples="missing"
[[ -n "\${runtime_stability_run_log_interval}" ]] || runtime_stability_run_log_interval="missing"
[[ -n "\${runtime_stability_run_log_exit_status}" ]] || runtime_stability_run_log_exit_status="missing"
[[ -n "\${runtime_stability_run_log_capture_exit_status}" ]] || runtime_stability_run_log_capture_exit_status="missing"
runtime_stability_run_log_elapsed_s="missing"
runtime_stability_run_log_required_elapsed_s="missing"
runtime_stability_run_log_duration_status="FAIL"
if is_iso8601_seconds_timestamp "\${runtime_stability_run_log_started_at}" && \\
   is_iso8601_seconds_timestamp "\${runtime_stability_run_log_finished_at}"; then
  if runtime_stability_run_log_elapsed_s=\$(iso8601_seconds_elapsed "\${runtime_stability_run_log_started_at}" "\${runtime_stability_run_log_finished_at}"); then
    :
  else
    runtime_stability_run_log_elapsed_s="missing"
  fi
fi
if is_positive_integer "\${runtime_stability_run_log_samples}" && \\
   is_positive_integer "\${runtime_stability_run_log_interval}"; then
  if runtime_stability_run_log_required_elapsed_s=\$(runtime_stability_required_elapsed_s "\${runtime_stability_run_log_samples}" "\${runtime_stability_run_log_interval}"); then
    :
  else
    runtime_stability_run_log_required_elapsed_s="missing"
  fi
fi
if is_nonnegative_integer "\${runtime_stability_run_log_elapsed_s}" && \\
   is_nonnegative_integer "\${runtime_stability_run_log_required_elapsed_s}" && \\
   ((10#\${runtime_stability_run_log_elapsed_s} >= 10#\${runtime_stability_run_log_required_elapsed_s})); then
  runtime_stability_run_log_duration_status="PASS"
fi
runtime_stability_run_log_status="FAIL"
if [[ -s "\${stability_run_log}" ]] && \\
   [[ "\${runtime_stability_run_log_keys_status}" == "PASS" ]] && \\
   is_iso8601_seconds_timestamp "\${runtime_stability_run_log_started_at}" && \\
   is_iso8601_seconds_timestamp "\${runtime_stability_run_log_finished_at}" && \\
   iso8601_seconds_finished_not_before_started "\${runtime_stability_run_log_started_at}" "\${runtime_stability_run_log_finished_at}" && \\
   [[ "\${runtime_stability_run_log_duration_status}" == "PASS" ]] && \\
   is_valid_text_value "\${runtime_stability_run_log_runtime_dir}" && \\
   [[ "\${runtime_stability_run_log_runtime_dir}" == "${RUNTIME_DIR}" ]] && \\
   is_positive_integer "\${runtime_stability_run_log_samples}" && \\
   [[ "\${runtime_stability_run_log_samples}" == "\${samples}" ]] && \\
   is_positive_integer "\${runtime_stability_run_log_interval}" && \\
   [[ "\${runtime_stability_run_log_interval}" == "\${interval_s}" ]] && \\
   is_zero_integer "\${runtime_stability_run_log_exit_status}" && \\
   is_zero_integer "\${runtime_stability_run_log_capture_exit_status}"; then
  runtime_stability_run_log_status="PASS"
fi
if [[ "\${runtime_stability_csv_status}" == "PASS" && \\
      "\${runtime_stability_run_log_status}" == "PASS" ]]; then
  runtime_stability_run_log_started_epoch=\$(iso8601_seconds_epoch "\${runtime_stability_run_log_started_at}") || runtime_stability_run_log_started_epoch="missing"
  runtime_stability_run_log_finished_epoch=\$(iso8601_seconds_epoch "\${runtime_stability_run_log_finished_at}") || runtime_stability_run_log_finished_epoch="missing"
  if ! [[ "\${runtime_stability_run_log_started_epoch}" =~ ^-?[0-9]+\$ && \\
          "\${runtime_stability_run_log_finished_epoch}" =~ ^-?[0-9]+\$ ]]; then
    runtime_stability_csv_status="FAIL"
  else
    runtime_stability_csv_header_seen=0
    while IFS= read -r runtime_stability_csv_line || [[ -n "\${runtime_stability_csv_line}" ]]; do
      [[ "\${runtime_stability_csv_line}" =~ ^[[:space:]]*\$ ]] && continue
      if ((runtime_stability_csv_header_seen == 0)); then
        runtime_stability_csv_header_seen=1
        continue
      fi
      IFS=, read -r runtime_stability_csv_sample \\
                    runtime_stability_csv_timestamp \\
                    runtime_stability_csv_disk_status \\
                    runtime_stability_csv_watchdog_status \\
                    runtime_stability_csv_health_report \\
                    runtime_stability_csv_extra <<< "\${runtime_stability_csv_line}"
      runtime_stability_csv_timestamp_epoch=\$(iso8601_seconds_epoch "\${runtime_stability_csv_timestamp}") || runtime_stability_csv_timestamp_epoch="missing"
      if ! [[ "\${runtime_stability_csv_timestamp_epoch}" =~ ^-?[0-9]+\$ ]] || \\
         ((runtime_stability_csv_timestamp_epoch < runtime_stability_run_log_started_epoch)) || \\
         ((runtime_stability_csv_timestamp_epoch > runtime_stability_run_log_finished_epoch)); then
        runtime_stability_csv_status="FAIL"
        break
      fi
    done < "\${stability_csv}"
  fi
fi
runtime_stability_status="FAIL"
if [[ -s "\${stability_summary}" ]] && \\
   [[ "\${runtime_stability_summary_keys_status}" == "PASS" ]] && \\
   [[ "\${stability_overall}" == "PASS" ]] && \\
   [[ "\${min_duration_status}" == "PASS" ]] && \\
   is_iso8601_seconds_timestamp "\${runtime_stability_summary_timestamp}" && \\
   iso8601_seconds_finished_not_before_started "\${runtime_stability_run_log_started_at}" "\${runtime_stability_summary_timestamp}" && \\
   iso8601_seconds_finished_not_before_started "\${runtime_stability_summary_timestamp}" "\${runtime_stability_run_log_finished_at}" && \\
   is_iso8601_seconds_timestamp "\${runtime_stability_csv_first_timestamp}" && \\
   is_iso8601_seconds_timestamp "\${runtime_stability_csv_last_timestamp}" && \\
   iso8601_seconds_finished_not_before_started "\${runtime_stability_csv_first_timestamp}" "\${runtime_stability_csv_last_timestamp}" && \\
   is_positive_integer "\${samples}" && \\
   is_positive_integer "\${interval_s}" && \\
   is_zero_integer "\${disk_failures}" && \\
   is_zero_integer "\${watchdog_failures}" && \\
   is_zero_integer "\${watchdog_skipped}" && \\
   is_zero_integer "\${health_failures}" && \\
   [[ "\${runtime_stability_csv_status}" == "PASS" ]] && \\
   [[ "\${runtime_stability_sample_count_match}" == "PASS" ]] && \\
   [[ "\${runtime_stability_run_log_status}" == "PASS" ]] && \\
   awk -v duration="\${runtime_stability_duration_h}" -v min_duration="\${min_duration_h}" 'BEGIN { exit(duration >= min_duration ? 0 : 1) }'; then
  runtime_stability_status="PASS"
fi

power_loss_resume_status="FAIL"
power_loss_resume_source="missing"
recovery_time_s="missing"
power_loss_max_recovery_time_s="\${max_recovery_time_s}"
power_loss_resume_timestamp="missing"
resume_verified_by="missing"
resume_verified_at="missing"
power_loss_resume_confirmation_overall="missing"
power_loss_resume_confirmation_keys_status="missing"
verified_metrics_report="missing"
power_loss_report_keys_status="missing"
power_loss_resume_overall=\$(line_value "\${power_loss_report}" "power_loss_resume_status")
[[ -n "\${power_loss_resume_overall}" ]] || power_loss_resume_overall="missing"
if [[ "\${POWER_LOSS_RESUME_VERIFIED:-FAIL}" == "PASS" ]]; then
  power_loss_resume_source="manual_env"
fi
if [[ -s "\${power_loss_report}" ]]; then
  power_loss_report_keys_status="FAIL"
  if line_keys_unique "\${power_loss_report}"; then
    power_loss_report_keys_status="PASS"
  fi
  power_loss_resume_timestamp=\$(line_value "\${power_loss_report}" "timestamp")
  power_loss_resume_source=\$(line_value "\${power_loss_report}" "power_loss_resume_source")
  recovery_time_s=\$(line_value "\${power_loss_report}" "recovery_time_s")
  verified_max_recovery_time_s=\$(line_value "\${power_loss_report}" "max_recovery_time_s")
  resume_verified_by=\$(line_value "\${power_loss_report}" "resume_verified_by")
  resume_verified_at=\$(line_value "\${power_loss_report}" "resume_verified_at")
  power_loss_resume_confirmation_overall=\$(line_value "\${power_loss_report}" "power_loss_resume_confirmation_overall")
  power_loss_resume_confirmation_keys_status=\$(line_value "\${power_loss_report}" "power_loss_resume_confirmation_keys_status")
  verified_metrics_report=\$(line_value "\${power_loss_report}" "metrics_report")
  [[ -n "\${power_loss_resume_timestamp}" ]] || power_loss_resume_timestamp="missing"
  [[ -n "\${power_loss_resume_source}" ]] || power_loss_resume_source="verified_file"
  [[ -n "\${recovery_time_s}" ]] || recovery_time_s="missing"
  [[ -n "\${verified_max_recovery_time_s}" ]] || verified_max_recovery_time_s="\${max_recovery_time_s}"
  [[ -n "\${resume_verified_by}" ]] || resume_verified_by="missing"
  [[ -n "\${resume_verified_at}" ]] || resume_verified_at="missing"
  [[ -n "\${power_loss_resume_confirmation_overall}" ]] || power_loss_resume_confirmation_overall="missing"
  [[ -n "\${power_loss_resume_confirmation_keys_status}" ]] || power_loss_resume_confirmation_keys_status="missing"
  [[ -n "\${verified_metrics_report}" ]] || verified_metrics_report="missing"
  power_loss_max_recovery_time_s="\${verified_max_recovery_time_s}"
  if [[ "\${power_loss_report_keys_status}" != "PASS" ]] || \\
     [[ "\${power_loss_resume_overall}" != "PASS" ]] || \\
     ! is_iso8601_seconds_timestamp "\${power_loss_resume_timestamp}"; then
    power_loss_resume_status="FAIL"
  elif [[ "\${power_loss_resume_source}" == "metrics_report" ]]; then
    metrics_recovery_time_s=\$(metrics_report_recovery_time "\${metrics_report}" || true)
    if [[ "\${recovery_time_s}" == "missing" ]] || \\
       ! is_valid_text_value "\${verified_metrics_report}" || \\
       [[ "\${verified_metrics_report}" != "\${metrics_report}" ]] || \\
       [[ -z "\${metrics_recovery_time_s}" ]] || \\
       ! metrics_report_passed "\${metrics_report}" || \\
       ! numbers_equal "\${recovery_time_s}" "\${metrics_recovery_time_s}" || \\
       ! recovery_time_within_limit "\${recovery_time_s}" "\${verified_max_recovery_time_s}"; then
      power_loss_resume_status="FAIL"
    else
      power_loss_resume_status="PASS"
    fi
  elif [[ "\${power_loss_resume_source}" == "manual_file" ]]; then
    if [[ "\${recovery_time_s}" == "missing" ]] || \\
       [[ "\${power_loss_resume_confirmation_overall}" != "PASS" ]] || \\
       [[ "\${power_loss_resume_confirmation_keys_status}" != "PASS" ]] || \\
       ! is_valid_text_value "\${resume_verified_by}" || \\
       ! is_iso8601_seconds_timestamp "\${resume_verified_at}" || \\
       ! recovery_time_within_limit "\${recovery_time_s}" "\${verified_max_recovery_time_s}"; then
      power_loss_resume_status="FAIL"
    else
      power_loss_resume_status="PASS"
    fi
  else
    power_loss_resume_status="FAIL"
  fi
elif [[ -s "\${metrics_report}" ]]; then
  power_loss_resume_source="metrics_report"
  verified_metrics_report="\${metrics_report}"
  recovery_time_s=\$(metrics_report_recovery_time "\${metrics_report}" || true)
  if metrics_report_passed "\${metrics_report}" && \\
     [[ -n "\${recovery_time_s}" ]] && \\
     recovery_time_within_limit "\${recovery_time_s}" "\${max_recovery_time_s}"; then
    power_loss_resume_status="PASS"
  else
    [[ -n "\${recovery_time_s}" ]] || recovery_time_s="missing"
  fi
fi

pps_ptp_wiring_verified="FAIL"
wiring_confirmation="missing"
wiring_confirmation_overall="missing"
wiring_confirmation_keys_status="missing"
wiring_confirmation_source="missing"
pps_wiring_verified="missing"
ptp_wiring_verified="missing"
wiring_verified_by="missing"
wiring_verified_at="missing"
pps_ptp_wiring_time_status_topic="missing"
pps_ptp_wiring_pps_topic="missing"
pps_ptp_wiring_pps_jitter_ms="missing"
pps_ptp_wiring_mean_offset_ms="missing"
pps_ptp_wiring_time_sync_status="missing"
pps_ptp_wiring_capture_status="missing"
pps_ptp_wiring_pps_status="missing"
pps_ptp_wiring_clock_offset_status="missing"
pps_ptp_wiring_time_sync_report="missing"
pps_ptp_wiring_time_sync_raw="missing"
pps_ptp_wiring_time_sync_raw_status="missing"
pps_ptp_wiring_time_sync_timestamp="missing"
pps_ptp_wiring_timestamp="missing"
pps_ptp_wiring_report="${REPORT_DIR}/pps_ptp_wiring_verified.txt"
pps_ptp_wiring_keys_status="FAIL"
if line_keys_unique "\${pps_ptp_wiring_report}"; then
  pps_ptp_wiring_keys_status="PASS"
fi
pps_ptp_wiring_overall=\$(line_value "\${pps_ptp_wiring_report}" "pps_ptp_wiring_verified")
[[ -n "\${pps_ptp_wiring_overall}" ]] || pps_ptp_wiring_overall="missing"
if [[ "\${PPS_PTP_WIRING_VERIFIED:-FAIL}" == "PASS" ]]; then
  wiring_confirmation_source="manual_env"
fi
if [[ -s "\${pps_ptp_wiring_report}" ]]; then
  pps_ptp_wiring_timestamp=\$(line_value "\${pps_ptp_wiring_report}" "timestamp")
  pps_ptp_wiring_time_sync_status=\$(line_value "\${pps_ptp_wiring_report}" "time_sync_status")
  pps_ptp_wiring_capture_status=\$(line_value "\${pps_ptp_wiring_report}" "capture_status")
  pps_ptp_wiring_time_status_topic=\$(line_value "\${pps_ptp_wiring_report}" "time_status_topic")
  pps_ptp_wiring_pps_topic=\$(line_value "\${pps_ptp_wiring_report}" "pps_topic")
  pps_ptp_wiring_pps_status=\$(line_value "\${pps_ptp_wiring_report}" "pps_status")
  pps_ptp_wiring_clock_offset_status=\$(line_value "\${pps_ptp_wiring_report}" "clock_offset_status")
  pps_ptp_wiring_pps_jitter_ms=\$(line_value "\${pps_ptp_wiring_report}" "pps_jitter_ms")
  pps_ptp_wiring_mean_offset_ms=\$(line_value "\${pps_ptp_wiring_report}" "mean_offset_ms")
  pps_ptp_wiring_time_sync_report=\$(line_value "\${pps_ptp_wiring_report}" "time_sync_report")
  pps_ptp_wiring_time_sync_raw=\$(line_value "\${pps_ptp_wiring_report}" "time_sync_raw")
  pps_ptp_wiring_time_sync_raw_status=\$(line_value "\${pps_ptp_wiring_report}" "time_sync_raw_status")
  pps_ptp_wiring_time_sync_timestamp=\$(line_value "\${pps_ptp_wiring_report}" "time_sync_timestamp")
  wiring_confirmation=\$(line_value "\${pps_ptp_wiring_report}" "wiring_confirmation")
  wiring_confirmation_overall=\$(line_value "\${pps_ptp_wiring_report}" "wiring_confirmation_overall")
  wiring_confirmation_keys_status=\$(line_value "\${pps_ptp_wiring_report}" "wiring_confirmation_keys_status")
  wiring_confirmation_source=\$(line_value "\${pps_ptp_wiring_report}" "wiring_confirmation_source")
  pps_wiring_verified=\$(line_value "\${pps_ptp_wiring_report}" "pps_wiring_verified")
  ptp_wiring_verified=\$(line_value "\${pps_ptp_wiring_report}" "ptp_wiring_verified")
  wiring_verified_by=\$(line_value "\${pps_ptp_wiring_report}" "wiring_verified_by")
  wiring_verified_at=\$(line_value "\${pps_ptp_wiring_report}" "wiring_verified_at")
  [[ -n "\${pps_ptp_wiring_timestamp}" ]] || pps_ptp_wiring_timestamp="missing"
  [[ -n "\${pps_ptp_wiring_time_sync_status}" ]] || pps_ptp_wiring_time_sync_status="missing"
  [[ -n "\${pps_ptp_wiring_capture_status}" ]] || pps_ptp_wiring_capture_status="missing"
  [[ -n "\${pps_ptp_wiring_time_status_topic}" ]] || pps_ptp_wiring_time_status_topic="missing"
  [[ -n "\${pps_ptp_wiring_pps_topic}" ]] || pps_ptp_wiring_pps_topic="missing"
  [[ -n "\${pps_ptp_wiring_pps_status}" ]] || pps_ptp_wiring_pps_status="missing"
  [[ -n "\${pps_ptp_wiring_clock_offset_status}" ]] || pps_ptp_wiring_clock_offset_status="missing"
  [[ -n "\${pps_ptp_wiring_pps_jitter_ms}" ]] || pps_ptp_wiring_pps_jitter_ms="missing"
  [[ -n "\${pps_ptp_wiring_mean_offset_ms}" ]] || pps_ptp_wiring_mean_offset_ms="missing"
  [[ -n "\${pps_ptp_wiring_time_sync_report}" ]] || pps_ptp_wiring_time_sync_report="missing"
  [[ -n "\${pps_ptp_wiring_time_sync_raw}" ]] || pps_ptp_wiring_time_sync_raw="missing"
  [[ -n "\${pps_ptp_wiring_time_sync_raw_status}" ]] || pps_ptp_wiring_time_sync_raw_status="missing"
  [[ -n "\${pps_ptp_wiring_time_sync_timestamp}" ]] || pps_ptp_wiring_time_sync_timestamp="missing"
  [[ -n "\${wiring_confirmation}" ]] || wiring_confirmation="missing"
  [[ -n "\${wiring_confirmation_overall}" ]] || wiring_confirmation_overall="missing"
  [[ -n "\${wiring_confirmation_keys_status}" ]] || wiring_confirmation_keys_status="missing"
  [[ -n "\${wiring_confirmation_source}" ]] || wiring_confirmation_source="missing"
  [[ -n "\${pps_wiring_verified}" ]] || pps_wiring_verified="missing"
  [[ -n "\${ptp_wiring_verified}" ]] || ptp_wiring_verified="missing"
  [[ -n "\${wiring_verified_by}" ]] || wiring_verified_by="missing"
  [[ -n "\${wiring_verified_at}" ]] || wiring_verified_at="missing"
  if [[ "\${time_sync_status}" == "PASS" && \\
        "\${pps_ptp_wiring_keys_status}" == "PASS" && \\
        "\${pps_ptp_wiring_overall}" == "PASS" && \\
        "\${pps_ptp_wiring_time_sync_status}" == "PASS" && \\
        "\${pps_ptp_wiring_capture_status}" == "CAPTURED" && \\
        "\${pps_ptp_wiring_time_status_topic}" == "\${time_status_topic}" && \\
        "\${pps_ptp_wiring_pps_topic}" == "\${pps_topic}" && \\
        "\${pps_ptp_wiring_pps_status}" == "PASS" && \\
        "\${pps_ptp_wiring_clock_offset_status}" == "PASS" && \\
        "\${pps_ptp_wiring_pps_jitter_ms}" == "\${pps_jitter_ms}" && \\
        "\${pps_ptp_wiring_mean_offset_ms}" == "\${mean_offset_ms}" && \\
        "\${pps_ptp_wiring_time_sync_report}" == "\${time_sync_report}" && \\
        "\${pps_ptp_wiring_time_sync_raw}" == "\${time_sync_raw}" && \\
        "\${pps_ptp_wiring_time_sync_raw_status}" == "PASS" && \\
        "\${pps_ptp_wiring_time_sync_timestamp}" == "\${time_sync_timestamp}" && \\
        "\${wiring_confirmation}" == "PASS" && \\
        "\${wiring_confirmation_overall}" == "PASS" && \\
        "\${wiring_confirmation_keys_status}" == "PASS" && \\
        "\${wiring_confirmation_source}" == "manual_file" && \\
        "\${pps_wiring_verified}" == "PASS" && \\
        "\${ptp_wiring_verified}" == "PASS" ]] && \\
     is_valid_ros_topic_name "\${pps_ptp_wiring_time_status_topic}" && \\
     is_valid_ros_topic_name "\${pps_ptp_wiring_pps_topic}" && \\
     is_nonnegative_number "\${pps_ptp_wiring_pps_jitter_ms}" && \\
     is_number "\${pps_ptp_wiring_mean_offset_ms}" && \\
     is_valid_text_value "\${pps_ptp_wiring_time_sync_report}" && \\
     is_valid_text_value "\${pps_ptp_wiring_time_sync_raw}" && \\
     is_iso8601_seconds_timestamp "\${pps_ptp_wiring_time_sync_timestamp}" && \\
     is_iso8601_seconds_timestamp "\${pps_ptp_wiring_timestamp}" && \\
     is_valid_text_value "\${wiring_verified_by}" && \\
     is_iso8601_seconds_timestamp "\${wiring_verified_at}"; then
    pps_ptp_wiring_verified="PASS"
  fi
fi

section_export_status="FAIL"
if [[ -s "\${section_export_report}" ]] && awk -F, -v expected_session="${NAME}" '
  NR == 1 {
    if (\$0 != "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points") {
      invalid = 1
      exit
    }
    next
  }
  \$0 !~ /^[[:space:]]*$/ {
    ++count
    for (i = 1; i <= NF; ++i) {
      if (\$i == "" || \$i ~ /^[[:space:]]/ || \$i ~ /[[:space:]]$/) {
        bad = 1
      }
    }
    if (NF != 7 || \$1 != expected_session ||
        \$3 !~ /^(IDLE_STATIC|CUTTING_STATIC|FWD_MOVE|REV_MOVE|TURNING|CMD_MOVE_NO_DISP|CONFLICT|RELOCALIZING)\$/ ||
        \$4 !~ /^(A|B|C)\$/ ||
        \$2 !~ /^-?[0-9]+(\\.[0-9]+)?\$/ ||
        \$5 !~ /^[0-9]+(\\.[0-9]+)?\$/ ||
        \$6 !~ /^[0-9]+(\\.[0-9]+)?\$/ ||
        \$7 !~ /^[1-9][0-9]*\$/ ||
        \$5 + 0 < 0 || \$5 + 0 > 1) {
      bad = 1
    }
  }
  END { exit(!invalid && !bad && count > 0 ? 0 : 1) }
' "\${section_export_report}"; then
  section_export_status="PASS"
fi

metrics_status="FAIL"
if [[ -s "\${metrics_report}" ]] && metrics_report_passed "\${metrics_report}"; then
  metrics_status="PASS"
fi

	event_file_status="FAIL"
	if replay_event_file_passed "\${event_file}"; then
	  event_file_status="PASS"
	fi

	field_acceptance_timestamp="\$(date --iso-8601=seconds)"
	field_acceptance_timestamp_status="FAIL"
	if is_iso8601_seconds_timestamp "\${field_acceptance_timestamp}" && \\
	   is_iso8601_seconds_timestamp "\${time_sync_timestamp}" && \\
	   iso8601_seconds_finished_not_before_started "\${time_sync_timestamp}" "\${field_acceptance_timestamp}" && \\
	   is_iso8601_seconds_timestamp "\${deployment_timestamp}" && \\
	   iso8601_seconds_finished_not_before_started "\${deployment_timestamp}" "\${field_acceptance_timestamp}" && \\
	   is_iso8601_seconds_timestamp "\${runtime_health_timestamp}" && \\
	   iso8601_seconds_finished_not_before_started "\${runtime_health_timestamp}" "\${field_acceptance_timestamp}" && \\
	   is_iso8601_seconds_timestamp "\${runtime_stability_run_log_finished_at}" && \\
	   iso8601_seconds_finished_not_before_started "\${runtime_stability_run_log_finished_at}" "\${field_acceptance_timestamp}" && \\
	   is_iso8601_seconds_timestamp "\${runtime_stability_summary_timestamp}" && \\
	   iso8601_seconds_finished_not_before_started "\${runtime_stability_summary_timestamp}" "\${field_acceptance_timestamp}" && \\
	   is_iso8601_seconds_timestamp "\${runtime_stability_csv_last_timestamp}" && \\
	   iso8601_seconds_finished_not_before_started "\${runtime_stability_csv_last_timestamp}" "\${field_acceptance_timestamp}" && \\
	   is_iso8601_seconds_timestamp "\${pps_ptp_wiring_timestamp}" && \\
	   iso8601_seconds_finished_not_before_started "\${pps_ptp_wiring_timestamp}" "\${field_acceptance_timestamp}" && \\
	   ([[ "\${power_loss_resume_timestamp}" == "missing" ]] || \\
	    (is_iso8601_seconds_timestamp "\${power_loss_resume_timestamp}" && \\
	     iso8601_seconds_finished_not_before_started "\${power_loss_resume_timestamp}" "\${field_acceptance_timestamp}")) && \\
	   is_iso8601_seconds_timestamp "\${wiring_verified_at}" && \\
	   iso8601_seconds_finished_not_before_started "\${wiring_verified_at}" "\${field_acceptance_timestamp}"; then
	  if [[ "\${power_loss_resume_source}" != "manual_file" ]]; then
	    field_acceptance_timestamp_status="PASS"
	  elif is_iso8601_seconds_timestamp "\${resume_verified_at}" && \\
	       iso8601_seconds_finished_not_before_started "\${resume_verified_at}" "\${field_acceptance_timestamp}"; then
	    field_acceptance_timestamp_status="PASS"
	  fi
	fi

	field_acceptance_status="FAIL"
	if [[ "\${event_file_status}" == "PASS" && \\
	      "\${field_acceptance_timestamp_status}" == "PASS" && \\
	      "\${time_sync_status}" == "PASS" && \\
	      "\${metrics_status}" == "PASS" && \\
	      "\${runtime_health_status}" == "PASS" && \\
      "\${runtime_deployment_status}" == "PASS" && \\
      "\${runtime_stability_status}" == "PASS" && \\
      "\${power_loss_resume_status}" == "PASS" && \\
      "\${section_export_status}" == "PASS" && \\
      "\${pps_ptp_wiring_verified}" == "PASS" ]]; then
  field_acceptance_status="PASS"
fi

	{
	  echo "timestamp=\${field_acceptance_timestamp}"
	  echo "field_acceptance_timestamp_status=\${field_acceptance_timestamp_status}"
	  echo "session_id=${NAME}"
	  echo "scenario=${SCENARIO}"
	  echo "field_acceptance_status=\${field_acceptance_status}"
  echo "event_file_status=\${event_file_status}"
  echo "event_file_report=\${event_file}"
  echo "time_sync_status=\${time_sync_status}"
  echo "time_sync_keys_status=\${time_sync_keys_status}"
  echo "time_sync_report=\${time_sync_report}"
  echo "time_sync_timestamp=\${time_sync_timestamp}"
  echo "time_sync_raw=\${time_sync_raw}"
  echo "time_sync_raw_status=\${time_sync_raw_status}"
  echo "time_status_topic=\${time_status_topic}"
  echo "pps_topic=\${pps_topic}"
  echo "time_capture_status=\${time_capture_status}"
  echo "time_pps_status=\${time_pps_status}"
  echo "time_clock_offset_status=\${time_clock_offset_status}"
  echo "pps_jitter_ms=\${pps_jitter_ms}"
  echo "mean_offset_ms=\${mean_offset_ms}"
  echo "runtime_deployment_status=\${runtime_deployment_status}"
  echo "runtime_deployment_keys_status=\${deployment_keys_status}"
  echo "runtime_deployment_report=\${deployment_report}"
  echo "runtime_deployment_timestamp=\${deployment_timestamp}"
  echo "deployment_overall=\${deployment_overall}"
  echo "deployment_status=\${deployment_overall}"
  echo "deployment_runtime_dir=\${deployment_runtime_dir}"
  echo "systemd_unit_file=\${systemd_unit_file}"
  echo "systemd_env_file=\${systemd_env_file}"
  echo "docker_compose_file=\${docker_compose_file}"
  echo "docker_env_file=\${docker_env_file}"
  echo "runtime_process_status=\${runtime_process_status}"
  echo "start_command=\${start_command}"
  echo "runtime_health_status=\${runtime_health_status}"
  echo "runtime_health_keys_status=\${runtime_health_keys_status}"
  echo "runtime_health_report=\${runtime_health_report}"
  echo "runtime_health_timestamp=\${runtime_health_timestamp}"
  echo "runtime_health_runtime_dir=\${runtime_health_runtime_dir}"
  echo "runtime_health_disk_available_gb=\${runtime_health_disk_available_gb}"
  echo "runtime_health_pid=\${runtime_health_pid}"
  echo "runtime_health_systemd_active=\${runtime_health_systemd_active}"
  echo "runtime_health_systemd_active_source=\${runtime_health_systemd_active_source}"
  echo "runtime_health_docker_container_status=\${runtime_health_docker_status}"
  echo "runtime_health_docker_container_status_source=\${runtime_health_docker_status_source}"
  echo "runtime_stability_status=\${runtime_stability_status}"
  echo "runtime_stability_min_duration_h=\${min_duration_h}"
  echo "runtime_stability_min_duration_status=\${min_duration_status}"
  echo "runtime_stability_csv_report=\${stability_csv}"
  echo "runtime_stability_csv_first_timestamp=\${runtime_stability_csv_first_timestamp}"
  echo "runtime_stability_csv_last_timestamp=\${runtime_stability_csv_last_timestamp}"
  echo "runtime_stability_summary_report=\${stability_summary}"
  echo "runtime_stability_summary_timestamp=\${runtime_stability_summary_timestamp}"
  echo "runtime_stability_run_log_report=\${stability_run_log}"
  echo "runtime_stability_overall=\${stability_overall}"
  echo "runtime_stability_summary_keys_status=\${runtime_stability_summary_keys_status}"
  echo "runtime_stability_csv_status=\${runtime_stability_csv_status}"
  echo "runtime_stability_run_log_status=\${runtime_stability_run_log_status}"
  echo "runtime_stability_run_log_keys_status=\${runtime_stability_run_log_keys_status}"
  echo "runtime_stability_run_log_started_at=\${runtime_stability_run_log_started_at}"
  echo "runtime_stability_run_log_finished_at=\${runtime_stability_run_log_finished_at}"
  echo "runtime_stability_run_log_runtime_dir=\${runtime_stability_run_log_runtime_dir}"
  echo "runtime_stability_run_log_samples=\${runtime_stability_run_log_samples}"
  echo "runtime_stability_run_log_interval=\${runtime_stability_run_log_interval}"
  echo "runtime_stability_run_log_elapsed_s=\${runtime_stability_run_log_elapsed_s}"
  echo "runtime_stability_run_log_required_elapsed_s=\${runtime_stability_run_log_required_elapsed_s}"
  echo "runtime_stability_run_log_duration_status=\${runtime_stability_run_log_duration_status}"
  echo "runtime_stability_run_log_exit_status=\${runtime_stability_run_log_exit_status}"
  echo "runtime_stability_run_log_capture_exit_status=\${runtime_stability_run_log_capture_exit_status}"
  echo "power_loss_resume_status=\${power_loss_resume_status}"
  echo "power_loss_resume_keys_status=\${power_loss_report_keys_status}"
  echo "power_loss_resume_report=\${power_loss_report}"
  echo "power_loss_resume_timestamp=\${power_loss_resume_timestamp}"
  echo "power_loss_resume_overall=\${power_loss_resume_overall}"
  echo "power_loss_resume_source=\${power_loss_resume_source}"
  echo "power_loss_resume_confirmation_overall=\${power_loss_resume_confirmation_overall}"
  echo "power_loss_resume_confirmation_keys_status=\${power_loss_resume_confirmation_keys_status}"
  echo "recovery_time_s=\${recovery_time_s}"
  echo "max_recovery_time_s=\${power_loss_max_recovery_time_s}"
  echo "resume_verified_by=\${resume_verified_by}"
  echo "resume_verified_at=\${resume_verified_at}"
  echo "pps_ptp_wiring_verified=\${pps_ptp_wiring_verified}"
  echo "pps_ptp_wiring_keys_status=\${pps_ptp_wiring_keys_status}"
  echo "pps_ptp_wiring_report=\${pps_ptp_wiring_report}"
  echo "pps_ptp_wiring_timestamp=\${pps_ptp_wiring_timestamp}"
  echo "pps_ptp_wiring_overall=\${pps_ptp_wiring_overall}"
  echo "pps_ptp_wiring_time_sync_status=\${pps_ptp_wiring_time_sync_status}"
  echo "pps_ptp_wiring_capture_status=\${pps_ptp_wiring_capture_status}"
  echo "pps_ptp_wiring_time_status_topic=\${pps_ptp_wiring_time_status_topic}"
  echo "pps_ptp_wiring_pps_topic=\${pps_ptp_wiring_pps_topic}"
  echo "pps_ptp_wiring_pps_status=\${pps_ptp_wiring_pps_status}"
  echo "pps_ptp_wiring_clock_offset_status=\${pps_ptp_wiring_clock_offset_status}"
  echo "pps_ptp_wiring_pps_jitter_ms=\${pps_ptp_wiring_pps_jitter_ms}"
  echo "pps_ptp_wiring_mean_offset_ms=\${pps_ptp_wiring_mean_offset_ms}"
  echo "pps_ptp_wiring_time_sync_report=\${pps_ptp_wiring_time_sync_report}"
  echo "pps_ptp_wiring_time_sync_raw=\${pps_ptp_wiring_time_sync_raw}"
  echo "pps_ptp_wiring_time_sync_raw_status=\${pps_ptp_wiring_time_sync_raw_status}"
  echo "pps_ptp_wiring_time_sync_timestamp=\${pps_ptp_wiring_time_sync_timestamp}"
  echo "wiring_confirmation=\${wiring_confirmation}"
  echo "wiring_confirmation_overall=\${wiring_confirmation_overall}"
  echo "wiring_confirmation_keys_status=\${wiring_confirmation_keys_status}"
  echo "wiring_confirmation_source=\${wiring_confirmation_source}"
  echo "pps_wiring_verified=\${pps_wiring_verified}"
  echo "ptp_wiring_verified=\${ptp_wiring_verified}"
  echo "wiring_verified_by=\${wiring_verified_by}"
  echo "wiring_verified_at=\${wiring_verified_at}"
  echo "systemd_active=\${systemd_active}"
  echo "systemd_active_source=\${systemd_active_source}"
  echo "docker_container_status=\${docker_container_status}"
  echo "docker_container_status_source=\${docker_container_status_source}"
  echo "runtime_stability_duration_h=\${runtime_stability_duration_h}"
  echo "runtime_stability_samples=\${samples}"
  echo "runtime_stability_csv_samples=\${runtime_stability_csv_samples}"
  echo "runtime_stability_sample_count_match=\${runtime_stability_sample_count_match}"
  echo "runtime_stability_interval_s=\${interval_s}"
  echo "runtime_stability_disk_failures=\${disk_failures}"
  echo "runtime_stability_watchdog_failures=\${watchdog_failures}"
  echo "runtime_stability_watchdog_skipped=\${watchdog_skipped}"
  echo "runtime_stability_health_failures=\${health_failures}"
  echo "section_export_status=\${section_export_status}"
  echo "section_export_report=\${section_export_report}"
  echo "metrics_report=\${metrics_report}"
  echo "metrics_status=\${metrics_status}"
} > "\${target}"
echo "\${target}"
if [[ "\${field_acceptance_status}" != "PASS" ]]; then
  exit 5
fi
EOF
chmod +x "${COMMAND_DIR}/capture_field_acceptance.sh"

cat > "${COMMAND_DIR}/validate_evidence.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd "${SESSION_DIR}"

METRICS_FILE="${REPORT_DIR}/validation_metrics.txt"
EVENT_FILE="${REPORT_DIR}/replay_events.txt"
METRICS_REPORT="${REPORT_DIR}/validation_metrics_report.txt"
EVIDENCE_REPORT="${REPORT_DIR}/evidence_validation_report.txt"

"${COMMAND_DIR}/capture_runtime_health.sh" || true
"${COMMAND_DIR}/capture_time_sync.sh" || true
"${COMMAND_DIR}/capture_pps_ptp_wiring.sh" || true
"${COMMAND_DIR}/capture_runtime_deployment.sh" || true
"${COMMAND_DIR}/capture_runtime_stability.sh" || true
"${COMMAND_DIR}/capture_section_export.sh" || true

if [[ -s "\${EVENT_FILE}" ]]; then
  roslaunch lio_eval_tools validation_report.launch \\
    event_file:="\${EVENT_FILE}" \\
    report_file:="\${METRICS_REPORT}"
  "${COMMAND_DIR}/capture_power_loss_resume.sh" || true
  "${COMMAND_DIR}/capture_field_acceptance.sh" || true
  exec roslaunch lio_eval_tools validation_report.launch \\
    event_file:="\${EVENT_FILE}" \\
    evidence_manifest_file:="${EVIDENCE_MANIFEST_PATH}" \\
    evidence_base_dir:="${SESSION_DIR}" \\
    report_file:="\${EVIDENCE_REPORT}"
fi

roslaunch lio_eval_tools validation_report.launch \\
  metrics_file:="\${METRICS_FILE}" \\
  report_file:="\${METRICS_REPORT}"
"${COMMAND_DIR}/capture_power_loss_resume.sh" || true
"${COMMAND_DIR}/capture_field_acceptance.sh" || true
"${COMMAND_DIR}/capture_section_export.sh" || true
exec roslaunch lio_eval_tools validation_report.launch \\
  metrics_file:="\${METRICS_FILE}" \\
  evidence_manifest_file:="${EVIDENCE_MANIFEST_PATH}" \\
  evidence_base_dir:="${SESSION_DIR}" \\
  report_file:="\${EVIDENCE_REPORT}"
EOF
chmod +x "${COMMAND_DIR}/validate_evidence.sh"

echo "Session directory: ${SESSION_DIR}"
echo "Topics: ${TOPICS}"

if [[ "$DRY_RUN" -eq 1 ]]; then
  echo "Dry run requested; rosbag record not started."
  exit 0
fi

"${COMMAND_DIR}/snapshot_ros_state.sh"
"${COMMAND_DIR}/capture_time_sync.sh" || true
"${COMMAND_DIR}/capture_pps_ptp_wiring.sh" || true
"${COMMAND_DIR}/capture_runtime_health.sh" || true
"${COMMAND_DIR}/capture_runtime_deployment.sh" || true
"${COMMAND_DIR}/capture_runtime_stability.sh" || true
"${COMMAND_DIR}/capture_section_export.sh" || true
if [[ "$START_PCAP" -eq 1 ]]; then
  "${COMMAND_DIR}/record_pcap.sh" > "${LOG_DIR}/record_pcap.log" 2>&1 &
  echo "$!" > "${LOG_DIR}/record_pcap.pid"
fi
exec "${COMMAND_DIR}/record_rosbag.sh"
