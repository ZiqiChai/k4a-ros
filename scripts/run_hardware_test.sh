#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_DIR
LOG_FILE="$(mktemp /tmp/k4a_ros_hardware_test.XXXXXX.log)"
readonly LOG_FILE

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "${PROJECT_DIR}/install/setup.bash" ]]; then
  echo "ROS 2 Humble or the built workspace is missing. Run scripts/build_ros.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"
# shellcheck disable=SC1091
set +u
source "${PROJECT_DIR}/install/setup.bash"
set -u

ros2 launch k4a_ros k4a.launch.py use_rviz:=false >"${LOG_FILE}" 2>&1 &
driver_pid=$!

stop_driver() {
  # Background jobs inherit SIGINT ignored from non-interactive Bash; SIGTERM is
  # handled by ros2 launch and is forwarded to the driver process.
  kill -TERM "${driver_pid}" 2>/dev/null || true
  wait "${driver_pid}" 2>/dev/null || true
}
trap stop_driver EXIT INT TERM

for _ in $(seq 1 30); do
  if ros2 node list 2>/dev/null | grep -qx '/k4a/driver'; then
    break
  fi
  if ! kill -0 "${driver_pid}" 2>/dev/null; then
    echo "Driver exited before publishing data. Log: ${LOG_FILE}" >&2
    exit 1
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -qx '/k4a/driver'; then
  echo "Timed out waiting for /k4a/driver. Log: ${LOG_FILE}" >&2
  exit 1
fi

"${SCRIPT_DIR}/validate_streams.py" --namespace /k4a --timeout 20
echo "Hardware stream test passed. Driver log: ${LOG_FILE}"
