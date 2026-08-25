#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_DIR

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

cd "${PROJECT_DIR}"
colcon build --base-paths "${PROJECT_DIR}/k4a_ros" --packages-select k4a_ros \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"

echo "Build completed. Run: source ${PROJECT_DIR}/install/setup.bash"
