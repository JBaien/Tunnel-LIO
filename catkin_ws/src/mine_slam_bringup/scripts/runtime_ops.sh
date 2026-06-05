#!/usr/bin/env bash
set -euo pipefail

ROOT="/tmp/tunnel_lio_runtime"
NAME=""
WORKSPACE="/home/bai/Desktop/Tunnel-LIO/catkin_ws"
LAUNCH_PACKAGE="mine_slam_bringup"
LAUNCH_FILE="bringup_sensors.launch"
SECTION_SESSION_ID=""
MIN_FREE_GB=10
LOG_RETENTION_DAYS=7
WATCHDOG_TOPIC="/time/status"
WATCHDOG_TIMEOUT=10
CPU_SET=""
DRY_RUN=0

usage() {
  cat <<'USAGE'
Usage: runtime_ops.sh [--root DIR] [--name NAME] [--workspace CATKIN_WS]
                      [--launch-package PKG] [--launch-file FILE]
                      [--section-session-id ID]
                      [--min-free-gb GB] [--log-retention-days DAYS]
                      [--watchdog-topic TOPIC] [--watchdog-timeout SEC]
                      [--cpu-set CPUSET]
                      [--dry-run]

Creates a board-side Tunnel-LIO runtime plan with reproducible start, disk
guard, and watchdog commands. Without --dry-run it checks disk space and then
executes the generated roslaunch command with ROS_LOG_DIR pinned to the runtime
log directory.
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
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --launch-package)
      LAUNCH_PACKAGE="$2"
      shift 2
      ;;
    --launch-file)
      LAUNCH_FILE="$2"
      shift 2
      ;;
    --section-session-id)
      SECTION_SESSION_ID="$2"
      shift 2
      ;;
    --min-free-gb)
      MIN_FREE_GB="$2"
      shift 2
      ;;
    --log-retention-days)
      LOG_RETENTION_DAYS="$2"
      shift 2
      ;;
    --watchdog-topic)
      WATCHDOG_TOPIC="$2"
      shift 2
      ;;
    --watchdog-timeout)
      WATCHDOG_TIMEOUT="$2"
      shift 2
      ;;
    --cpu-set)
      CPU_SET="$2"
      shift 2
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

if [[ -z "$SECTION_SESSION_ID" && "$LAUNCH_FILE" == bringup_fusion_*.launch ]]; then
  SECTION_SESSION_ID="$NAME"
fi

validate_runtime_value() {
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
    echo "${field_name} must not contain metadata separators" >&2
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

validate_roslaunch_token() {
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
  if [[ ! "$value" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "${field_name} must be a safe roslaunch token" >&2
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
        "$value" == *'<'* || "$value" == *'>'* ]]; then
    echo "${field_name} must not contain shell metacharacters" >&2
    return 2
  fi
}

validate_docker_volume_source() {
  local field_name="$1"
  local value="$2"
  if [[ "$value" == *":"* ]]; then
    echo "${field_name} must not contain Docker volume separators" >&2
    return 2
  fi
}

validate_csv_safe_value() {
  local field_name="$1"
  local value="$2"
  if [[ "$value" == *","* ]]; then
    echo "${field_name} must not contain CSV separators" >&2
    return 2
  fi
}

validate_absolute_path() {
  local field_name="$1"
  local value="$2"
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

validate_cpu_set() {
  local value="$1"
  local token=""
  local start=""
  local end=""
  if [[ -z "$value" ]]; then
    return 0
  fi
  if [[ ! "$value" =~ ^[0-9]+(-[0-9]+)?(,[0-9]+(-[0-9]+)?)*$ ]]; then
    echo "cpu_set must be a taskset-compatible CPU list" >&2
    return 2
  fi
  IFS=',' read -ra cpu_set_tokens <<< "$value"
  for token in "${cpu_set_tokens[@]}"; do
    if [[ "$token" =~ ^([0-9]+)-([0-9]+)$ ]]; then
      start="${BASH_REMATCH[1]}"
      end="${BASH_REMATCH[2]}"
      if (( 10#${start} > 10#${end} )); then
        echo "cpu_set ranges must be ascending" >&2
        return 2
      fi
    fi
  done
}

validate_positive_integer() {
  local field_name="$1"
  local value="$2"
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${field_name} must be a positive integer" >&2
    return 2
  fi
}

validate_runtime_value "runtime_name" "$NAME" false
validate_runtime_value "runtime_root" "$ROOT" false
validate_runtime_value "section_session_id" "$SECTION_SESSION_ID" true
validate_path_segment "runtime_name" "$NAME"
validate_roslaunch_token "runtime_name" "$NAME" false
validate_roslaunch_token "section_session_id" "$SECTION_SESSION_ID" true
validate_generated_script_literal "runtime_root" "$ROOT" false
validate_csv_safe_value "runtime_root" "$ROOT"
validate_absolute_path "runtime_root" "$ROOT"
validate_docker_volume_source "runtime_root" "$ROOT"
validate_runtime_value "workspace" "$WORKSPACE" false
validate_runtime_value "launch_package" "$LAUNCH_PACKAGE" false
validate_runtime_value "launch_file" "$LAUNCH_FILE" false
validate_roslaunch_token "launch_package" "$LAUNCH_PACKAGE" false
validate_roslaunch_token "launch_file" "$LAUNCH_FILE" false
validate_runtime_value "watchdog_topic" "$WATCHDOG_TOPIC" false
validate_runtime_value "cpu_set" "$CPU_SET" true
validate_generated_script_literal "workspace" "$WORKSPACE" false
validate_absolute_path "workspace" "$WORKSPACE"
validate_docker_volume_source "workspace" "$WORKSPACE"
validate_generated_script_literal "watchdog_topic" "$WATCHDOG_TOPIC" false
validate_ros_topic_name "watchdog_topic" "$WATCHDOG_TOPIC"
validate_generated_script_literal "cpu_set" "$CPU_SET" true
validate_cpu_set "$CPU_SET"
validate_positive_integer "min_free_gb" "$MIN_FREE_GB"
validate_positive_integer "log_retention_days" "$LOG_RETENTION_DAYS"
validate_positive_integer "watchdog_timeout_s" "$WATCHDOG_TIMEOUT"

ROSLAUNCH_ARGS=""
if [[ -n "$SECTION_SESSION_ID" ]]; then
  ROSLAUNCH_ARGS="section_session_id:=${SECTION_SESSION_ID}"
fi

RUNTIME_DIR="${ROOT}/${NAME}"
LOG_DIR="${RUNTIME_DIR}/logs"
STATE_DIR="${RUNTIME_DIR}/state"
COMMAND_DIR="${RUNTIME_DIR}/commands"
SYSTEMD_DIR="${RUNTIME_DIR}/systemd"
DOCKER_DIR="${RUNTIME_DIR}/docker"
SYSTEMD_UNIT_NAME="tunnel-lio.service"
SYSTEMD_UNIT_PATH="${SYSTEMD_DIR}/${SYSTEMD_UNIT_NAME}"
SYSTEMD_ENV_PATH="${SYSTEMD_DIR}/tunnel-lio.env"
DOCKER_COMPOSE_PATH="${DOCKER_DIR}/docker-compose.yaml"
DOCKER_ENV_PATH="${DOCKER_DIR}/tunnel-lio.env"

mkdir -p "$LOG_DIR" "$STATE_DIR" "$COMMAND_DIR" "$SYSTEMD_DIR" "$DOCKER_DIR"

cat > "${RUNTIME_DIR}/runtime.env" <<EOF
runtime_name=${NAME}
runtime_dir=${RUNTIME_DIR}
created_at=$(date --iso-8601=seconds)
workspace=${WORKSPACE}
launch=${LAUNCH_PACKAGE} ${LAUNCH_FILE}
section_session_id=${SECTION_SESSION_ID}
roslaunch_args=${ROSLAUNCH_ARGS}
log_dir=${LOG_DIR}
state_dir=${STATE_DIR}
min_free_gb=${MIN_FREE_GB}
log_retention_days=${LOG_RETENTION_DAYS}
watchdog_topic=${WATCHDOG_TOPIC}
watchdog_timeout_s=${WATCHDOG_TIMEOUT}
cpu_set=${CPU_SET}
systemd_unit=${SYSTEMD_UNIT_PATH}
systemd_env=${SYSTEMD_ENV_PATH}
docker_compose=${DOCKER_COMPOSE_PATH}
EOF

cat > "${COMMAND_DIR}/start_runtime.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
mkdir -p "${LOG_DIR}" "${STATE_DIR}"
export ROS_LOG_DIR="${LOG_DIR}"
source "${WORKSPACE}/devel/setup.bash"
echo "\$\$" > "${STATE_DIR}/runtime.pid"
if [[ -n "${CPU_SET}" ]]; then
  exec taskset -c "${CPU_SET}" roslaunch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ${ROSLAUNCH_ARGS}
fi
exec roslaunch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ${ROSLAUNCH_ARGS}
EOF
chmod +x "${COMMAND_DIR}/start_runtime.sh"

cat > "${COMMAND_DIR}/disk_guard.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="${RUNTIME_DIR}"
min_free_gb=${MIN_FREE_GB}
log_retention_days=${LOG_RETENTION_DAYS}
available_gb=\$(df -BG "\${runtime_dir}" | awk 'NR==2 {gsub("G","",\$4); print \$4}')
find "\${runtime_dir}/logs" -type f -mtime +"${LOG_RETENTION_DAYS}" -delete || true
if [[ "\${available_gb}" -lt "\${min_free_gb}" ]]; then
  echo "disk free \${available_gb}GB below required \${min_free_gb}GB" >&2
  exit 3
fi
EOF
chmod +x "${COMMAND_DIR}/disk_guard.sh"

cat > "${COMMAND_DIR}/watchdog_check.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
topic="${WATCHDOG_TOPIC}"
timeout_s=${WATCHDOG_TIMEOUT}
timeout "\${timeout_s}" rostopic echo -n1 "\${topic}" >/dev/null
EOF
chmod +x "${COMMAND_DIR}/watchdog_check.sh"

cat > "${COMMAND_DIR}/runtime_health.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="${RUNTIME_DIR}"
log_dir="${LOG_DIR}"
state_dir="${STATE_DIR}"
unit_name="${SYSTEMD_UNIT_NAME}"
container_name="tunnel-lio-runtime"
mkdir -p "\${log_dir}"
timestamp=\$(date --iso-8601=seconds)
stamp=\$(date +%Y%m%d_%H%M%S)
report_path="\${log_dir}/runtime_health_\${stamp}.txt"
available_gb=\$(df -BG "\${runtime_dir}" | awk 'NR==2 {gsub("G","",\$4); print \$4}')
runtime_pid="missing"
if [[ -s "\${state_dir}/runtime.pid" ]]; then
  runtime_pid=\$(cat "\${state_dir}/runtime.pid")
fi
systemd_active="unavailable"
systemd_active_source="unavailable"
if command -v systemctl >/dev/null 2>&1; then
  systemd_active_source="systemctl"
  systemd_active=\$(timeout 2 systemctl is-active "\${unit_name}" 2>/dev/null || true)
  if [[ -z "\${systemd_active}" ]]; then
    systemd_active="unavailable"
  fi
fi
docker_container_status="unavailable"
docker_container_status_source="unavailable"
if command -v docker >/dev/null 2>&1; then
  docker_container_status_source="docker_inspect"
  docker_container_status=\$(timeout 2 docker inspect -f '{{.State.Status}}' "\${container_name}" 2>/dev/null || true)
  if [[ -z "\${docker_container_status}" ]]; then
    docker_container_status="unavailable"
  fi
fi
{
  echo "timestamp=\${timestamp}"
  echo "runtime_dir=\${runtime_dir}"
  echo "log_dir=\${log_dir}"
  echo "state_dir=\${state_dir}"
  echo "disk_available_gb=\${available_gb}"
  echo "runtime_pid=\${runtime_pid}"
  echo "systemd_unit=\${unit_name}"
  echo "systemd_active=\${systemd_active}"
  echo "systemd_active_source=\${systemd_active_source}"
  echo "docker_container=\${container_name}"
  echo "docker_container_status=\${docker_container_status}"
  echo "docker_container_status_source=\${docker_container_status_source}"
} | tee "\${report_path}" >/dev/null
ln -sfn "\${report_path}" "\${log_dir}/runtime_health_latest.txt"
echo "\${report_path}"
EOF
chmod +x "${COMMAND_DIR}/runtime_health.sh"

cat > "${COMMAND_DIR}/runtime_deployment_check.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="${RUNTIME_DIR}"
log_dir="${LOG_DIR}"
command_dir="${COMMAND_DIR}"
systemd_unit_path="${SYSTEMD_UNIT_PATH}"
systemd_env_path="${SYSTEMD_ENV_PATH}"
docker_compose_path="${DOCKER_COMPOSE_PATH}"
docker_env_path="${DOCKER_ENV_PATH}"
unit_name="${SYSTEMD_UNIT_NAME}"
container_name="tunnel-lio-runtime"
report_path="\${log_dir}/runtime_deployment_check.txt"
mkdir -p "\${log_dir}"

check_file() {
  local path="\$1"
  if [[ -s "\${path}" ]]; then
    echo "PASS"
  else
    echo "FAIL"
  fi
}

check_executable() {
  local path="\$1"
  if [[ -x "\${path}" ]]; then
    echo "PASS"
  else
    echo "FAIL"
  fi
}

systemd_active="\${TUNNEL_LIO_SYSTEMD_ACTIVE:-unavailable}"
systemd_active_source="env_override"
systemd_enabled="\${TUNNEL_LIO_SYSTEMD_ENABLED:-unavailable}"
if [[ -z "\${TUNNEL_LIO_SYSTEMD_ACTIVE:-}" ]] && command -v systemctl >/dev/null 2>&1; then
  systemd_active_source="systemctl"
  systemd_active=\$(timeout 2 systemctl is-active "\${unit_name}" 2>/dev/null || true)
  systemd_enabled=\$(timeout 2 systemctl is-enabled "\${unit_name}" 2>/dev/null || true)
  [[ -n "\${systemd_active}" ]] || systemd_active="unavailable"
  [[ -n "\${systemd_enabled}" ]] || systemd_enabled="unavailable"
elif [[ -z "\${TUNNEL_LIO_SYSTEMD_ACTIVE:-}" ]]; then
  systemd_active_source="unavailable"
fi

docker_compose_available="unavailable"
docker_container_status="\${TUNNEL_LIO_DOCKER_STATUS:-unavailable}"
docker_container_status_source="env_override"
if [[ -z "\${TUNNEL_LIO_DOCKER_STATUS:-}" ]] && command -v docker >/dev/null 2>&1; then
  docker_container_status_source="docker_inspect"
  docker_compose_available="missing"
  if docker compose version >/dev/null 2>&1; then
    docker_compose_available="available"
  fi
  docker_container_status=\$(timeout 2 docker inspect -f '{{.State.Status}}' "\${container_name}" 2>/dev/null || true)
  [[ -n "\${docker_container_status}" ]] || docker_container_status="unavailable"
elif [[ -z "\${TUNNEL_LIO_DOCKER_STATUS:-}" ]]; then
  docker_container_status_source="unavailable"
fi

systemd_unit_file=\$(check_file "\${systemd_unit_path}")
systemd_env_file=\$(check_file "\${systemd_env_path}")
docker_compose_file=\$(check_file "\${docker_compose_path}")
docker_env_file=\$(check_file "\${docker_env_path}")
start_command=\$(check_executable "\${command_dir}/start_runtime.sh")
disk_guard_command=\$(check_executable "\${command_dir}/disk_guard.sh")
watchdog_command=\$(check_executable "\${command_dir}/watchdog_check.sh")
health_command=\$(check_executable "\${command_dir}/runtime_health.sh")
stability_command=\$(check_executable "\${command_dir}/runtime_stability_check.sh")

deployment_status="PASS"
for status in "\${systemd_unit_file}" "\${systemd_env_file}" "\${docker_compose_file}" "\${docker_env_file}" "\${start_command}" "\${disk_guard_command}" "\${watchdog_command}" "\${health_command}" "\${stability_command}"; do
  if [[ "\${status}" != "PASS" ]]; then
    deployment_status="FAIL"
  fi
done
runtime_process_status="FAIL"
if [[ "\${systemd_active}" == "active" && \\
      "\${systemd_active_source}" == "systemctl" && \\
      "\${docker_container_status}" == "running" && \\
      "\${docker_container_status_source}" == "docker_inspect" ]]; then
  runtime_process_status="PASS"
fi
if [[ "\${runtime_process_status}" != "PASS" ]]; then
  deployment_status="FAIL"
fi

{
  echo "timestamp=\$(date --iso-8601=seconds)"
  echo "runtime_dir=\${runtime_dir}"
  echo "systemd_unit_file=\${systemd_unit_file}"
  echo "systemd_env_file=\${systemd_env_file}"
  echo "systemd_active=\${systemd_active}"
  echo "systemd_active_source=\${systemd_active_source}"
  echo "systemd_enabled=\${systemd_enabled}"
  echo "docker_compose_file=\${docker_compose_file}"
  echo "docker_env_file=\${docker_env_file}"
  echo "docker_compose_available=\${docker_compose_available}"
  echo "docker_container_status=\${docker_container_status}"
  echo "docker_container_status_source=\${docker_container_status_source}"
  echo "runtime_process_status=\${runtime_process_status}"
  echo "start_command=\${start_command}"
  echo "disk_guard_command=\${disk_guard_command}"
  echo "watchdog_command=\${watchdog_command}"
  echo "health_command=\${health_command}"
  echo "stability_command=\${stability_command}"
  echo "deployment_status=\${deployment_status}"
} > "\${report_path}"
echo "\${report_path}"
if [[ "\${deployment_status}" != "PASS" ]]; then
  exit 4
fi
EOF
chmod +x "${COMMAND_DIR}/runtime_deployment_check.sh"

cat > "${COMMAND_DIR}/runtime_stability_check.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
runtime_dir="${RUNTIME_DIR}"
log_dir="${LOG_DIR}"
command_dir="${COMMAND_DIR}"
samples=1440
interval_s=60

usage() {
  cat <<'STABILITY_USAGE'
Usage: runtime_stability_check.sh [--samples N] [--interval SEC]

Runs repeated board-side stability checks for long-duration evidence. Set
TUNNEL_LIO_SKIP_WATCHDOG=1 for offline dry-run validation without a ROS master.
STABILITY_USAGE
}

while [[ \$# -gt 0 ]]; do
  case "\$1" in
    --samples)
      samples="\$2"
      shift 2
      ;;
    --interval)
      interval_s="\$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: \$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "\${samples}" =~ ^[1-9][0-9]*\$ ]]; then
  echo "samples must be a positive integer" >&2
  exit 2
fi
if [[ ! "\${interval_s}" =~ ^(0|[1-9][0-9]*)\$ ]]; then
  echo "interval must be a nonnegative integer" >&2
  exit 2
fi

mkdir -p "\${log_dir}"
csv="\${log_dir}/runtime_stability.csv"
summary="\${log_dir}/runtime_stability_summary.txt"
echo "sample,timestamp,disk_guard_status,watchdog_status,health_report" > "\${csv}"

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

is_number() {
  local value="\$1"
  [[ "\${value}" =~ ^[+-]?(([0-9]+(\\.[0-9]*)?)|(\\.[0-9]+))([eE][+-]?[0-9]+)?\$ ]]
}

is_nonnegative_number() {
  local value="\$1"
  is_number "\${value}" && awk -v value="\${value}" 'BEGIN { exit(value >= 0 ? 0 : 1) }'
}

is_csv_safe_field() {
  local value="\$1"
  [[ -n "\${value}" && "\${value}" != *","* && \\
    "\${value}" != *";"* && \\
    "\${value}" != *$'\n'* && "\${value}" != *$'\r'* ]]
}

health_report_passed() {
  local report="\$1"
  [[ -s "\${report}" ]] || return 1
  line_keys_unique "\${report}" || return 1

  local health_runtime_dir
  local health_disk_available_gb
  local health_pid
  local health_systemd_active
  local health_systemd_active_source
  local health_docker_status
  local health_docker_status_source
  health_runtime_dir=\$(line_value "\${report}" "runtime_dir")
  health_disk_available_gb=\$(line_value "\${report}" "disk_available_gb")
  health_pid=\$(line_value "\${report}" "runtime_pid")
  health_systemd_active=\$(line_value "\${report}" "systemd_active")
  health_systemd_active_source=\$(line_value "\${report}" "systemd_active_source")
  health_docker_status=\$(line_value "\${report}" "docker_container_status")
  health_docker_status_source=\$(line_value "\${report}" "docker_container_status_source")

  [[ "\${health_runtime_dir}" == "\${runtime_dir}" ]] && \\
    is_nonnegative_number "\${health_disk_available_gb}" && \\
    is_positive_integer "\${health_pid}" && \\
    [[ "\${health_systemd_active}" == "active" ]] && \\
    [[ "\${health_systemd_active_source}" == "systemctl" ]] && \\
    [[ "\${health_docker_status}" == "running" ]] && \\
    [[ "\${health_docker_status_source}" == "docker_inspect" ]]
}

disk_failures=0
watchdog_failures=0
watchdog_skipped=0
health_failures=0

for ((sample = 1; sample <= samples; sample++)); do
  timestamp=\$(date --iso-8601=seconds)
  disk_status="PASS"
  if ! "\${command_dir}/disk_guard.sh" >/dev/null 2>&1; then
    disk_status="FAIL"
    disk_failures=\$((disk_failures + 1))
  fi

  watchdog_status="PASS"
  if [[ "\${TUNNEL_LIO_SKIP_WATCHDOG:-0}" == "1" ]]; then
    watchdog_status="SKIP"
    watchdog_skipped=\$((watchdog_skipped + 1))
  elif ! "\${command_dir}/watchdog_check.sh" >/dev/null 2>&1; then
    watchdog_status="FAIL"
    watchdog_failures=\$((watchdog_failures + 1))
  fi

  health_report="missing"
  if health_report=\$("\${command_dir}/runtime_health.sh" 2>/dev/null); then
    if ! is_csv_safe_field "\${health_report}"; then
      health_report="missing"
      health_failures=\$((health_failures + 1))
    elif ! health_report_passed "\${health_report}"; then
      health_failures=\$((health_failures + 1))
    fi
  else
    health_report="missing"
    health_failures=\$((health_failures + 1))
  fi

  echo "\${sample},\${timestamp},\${disk_status},\${watchdog_status},\${health_report}" >> "\${csv}"

  if [[ "\${sample}" -lt "\${samples}" && "\${interval_s}" != "0" ]]; then
    sleep "\${interval_s}"
  fi
done

overall="PASS"
if [[ "\${disk_failures}" -gt 0 || "\${watchdog_failures}" -gt 0 || \\
      "\${watchdog_skipped}" -gt 0 || "\${health_failures}" -gt 0 ]]; then
  overall="FAIL"
fi

{
  echo "timestamp=\$(date --iso-8601=seconds)"
  echo "overall=\${overall}"
  echo "runtime_dir=\${runtime_dir}"
  echo "samples=\${samples}"
  echo "interval_s=\${interval_s}"
  echo "disk_failures=\${disk_failures}"
  echo "watchdog_failures=\${watchdog_failures}"
  echo "watchdog_skipped=\${watchdog_skipped}"
  echo "health_failures=\${health_failures}"
  echo "csv=\${csv}"
} > "\${summary}"
echo "\${summary}"
EOF
chmod +x "${COMMAND_DIR}/runtime_stability_check.sh"

cat > "${DOCKER_DIR}/Dockerfile" <<EOF
FROM ros:noetic-ros-core

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/tunnel_lio/catkin_ws

SHELL ["/bin/bash", "-lc"]
RUN apt-get update && apt-get install -y --no-install-recommends \\
    bash \\
    coreutils \\
    iproute2 \\
    procps \\
    util-linux \\
    ros-noetic-roslaunch \\
    ros-noetic-rosbash \\
  && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["/runtime/commands/start_runtime.sh"]
EOF

cat > "${DOCKER_ENV_PATH}" <<EOF
TUNNEL_LIO_RUNTIME_DIR=/runtime
TUNNEL_LIO_WORKSPACE=/opt/tunnel_lio/catkin_ws
ROS_LOG_DIR=/runtime/logs
TUNNEL_LIO_CPU_SET=${CPU_SET}
EOF

cat > "${DOCKER_COMPOSE_PATH}" <<EOF
services:
  tunnel-lio-runtime:
    build:
      context: ${DOCKER_DIR}
      dockerfile: Dockerfile
    image: tunnel-lio-runtime:latest
    container_name: tunnel-lio-runtime
    network_mode: host
    privileged: true
    ipc: host
    pid: host
    working_dir: /runtime
    env_file:
      - ${DOCKER_ENV_PATH}
    volumes:
      - ${RUNTIME_DIR}:/runtime
      - ${WORKSPACE}:/opt/tunnel_lio/catkin_ws:ro
      - /dev:/dev
    command: /runtime/commands/start_runtime.sh
EOF

cat > "${COMMAND_DIR}/docker_build.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
compose_file="${DOCKER_COMPOSE_PATH}"
exec docker compose -f "\${compose_file}" build
EOF
chmod +x "${COMMAND_DIR}/docker_build.sh"

cat > "${COMMAND_DIR}/docker_up.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
compose_file="${DOCKER_COMPOSE_PATH}"
exec docker compose -f "\${compose_file}" up -d
EOF
chmod +x "${COMMAND_DIR}/docker_up.sh"

cat > "${COMMAND_DIR}/docker_down.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
compose_file="${DOCKER_COMPOSE_PATH}"
exec docker compose -f "\${compose_file}" down
EOF
chmod +x "${COMMAND_DIR}/docker_down.sh"

cat > "${COMMAND_DIR}/docker_logs.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
compose_file="${DOCKER_COMPOSE_PATH}"
exec docker compose -f "\${compose_file}" logs --tail=200 tunnel-lio-runtime
EOF
chmod +x "${COMMAND_DIR}/docker_logs.sh"

cat > "${SYSTEMD_ENV_PATH}" <<EOF
TUNNEL_LIO_RUNTIME_DIR=${RUNTIME_DIR}
TUNNEL_LIO_WORKSPACE=${WORKSPACE}
ROS_LOG_DIR=${LOG_DIR}
TUNNEL_LIO_CPU_SET=${CPU_SET}
EOF

cat > "${SYSTEMD_UNIT_PATH}" <<EOF
[Unit]
Description=Tunnel-LIO runtime
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
EnvironmentFile=${SYSTEMD_ENV_PATH}
WorkingDirectory=${RUNTIME_DIR}
ExecStartPre=${COMMAND_DIR}/disk_guard.sh
ExecStart=${COMMAND_DIR}/start_runtime.sh
Restart=always
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
EOF

cat > "${COMMAND_DIR}/install_systemd.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
unit_name="${SYSTEMD_UNIT_NAME}"
unit_source="${SYSTEMD_UNIT_PATH}"
unit_target="/etc/systemd/system/\${unit_name}"
install -m 0644 "\${unit_source}" "\${unit_target}"
systemctl daemon-reload
systemctl enable tunnel-lio.service
echo "Installed \${unit_target}"
EOF
chmod +x "${COMMAND_DIR}/install_systemd.sh"

echo "Runtime directory: ${RUNTIME_DIR}"
echo "Launch: ${LAUNCH_PACKAGE} ${LAUNCH_FILE}"
echo "Watchdog topic: ${WATCHDOG_TOPIC}"

if [[ "$DRY_RUN" -eq 1 ]]; then
  echo "Dry run requested; runtime not started."
  exit 0
fi

"${COMMAND_DIR}/disk_guard.sh"
exec "${COMMAND_DIR}/start_runtime.sh"
