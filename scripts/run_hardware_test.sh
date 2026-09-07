#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="$(mktemp /tmp/k4a_ros_hardware_test.XXXXXX.log)"
readonly LOG_FILE

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1 || ! ros2 pkg prefix k4a_ros >/dev/null 2>&1; then
  echo "k4a_ros is not in the current ROS environment." >&2
  echo "Source ROS 2 and the workspace before running this script:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  source ~/ros_ws/install/setup.bash" >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/k4a/driver'; then
  echo "A /k4a/driver node is already running. Stop it before running this test." >&2
  exit 1
fi

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
  if ! kill -0 "${driver_pid}" 2>/dev/null; then
    echo "Driver exited before publishing data. Log: ${LOG_FILE}" >&2
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -qx '/k4a/driver'; then
    break
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -qx '/k4a/driver'; then
  echo "Timed out waiting for /k4a/driver. Log: ${LOG_FILE}" >&2
  exit 1
fi

ros2 run k4a_ros validate_streams.py --namespace /k4a --timeout 20
echo "Hardware stream test passed. Driver log: ${LOG_FILE}"
