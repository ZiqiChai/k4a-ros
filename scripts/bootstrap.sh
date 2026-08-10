#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_DIR

sdk_method="binary"
offline="false"
accept_license="false"
skip_system_deps="false"
skip_rosdep="false"
skip_udev="false"
cache_dir=""
source_dir=""

usage() {
  cat <<'EOF'
Usage: scripts/bootstrap.sh [options]

Install dependencies, deploy the SDK, install udev rules, and build the ROS package.
ROS 2 Humble must already be installed in /opt/ros/humble.

Options:
  --sdk-method binary|source
  --cache-dir PATH
  --source-dir PATH
  --offline
  --accept-sdk-license
  --skip-system-deps
  --skip-rosdep
  --skip-udev
  -h, --help
EOF
}

while (($#)); do
  case "$1" in
    --sdk-method)
      [[ $# -ge 2 ]] || { echo "--sdk-method requires a value" >&2; exit 2; }
      sdk_method="$2"
      shift 2
      ;;
    --cache-dir)
      [[ $# -ge 2 ]] || { echo "--cache-dir requires a path" >&2; exit 2; }
      cache_dir="$2"
      shift 2
      ;;
    --source-dir)
      [[ $# -ge 2 ]] || { echo "--source-dir requires a path" >&2; exit 2; }
      source_dir="$2"
      shift 2
      ;;
    --offline)
      offline="true"
      shift
      ;;
    --accept-sdk-license)
      accept_license="true"
      shift
      ;;
    --skip-system-deps)
      skip_system_deps="true"
      shift
      ;;
    --skip-rosdep)
      skip_rosdep="true"
      shift
      ;;
    --skip-udev)
      skip_udev="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${sdk_method}" != "binary" && "${sdk_method}" != "source" ]]; then
  echo "Unsupported SDK method '${sdk_method}'." >&2
  exit 2
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "22.04" ]]; then
  echo "This release supports Ubuntu 22.04; detected ${PRETTY_NAME}." >&2
  exit 1
fi
if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "The Azure Kinect Depth Engine for this release requires amd64." >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found. Install it before running bootstrap." >&2
  exit 1
fi

if [[ "${skip_system_deps}" == "false" ]]; then
  packages=(
    build-essential
    cmake
    curl
    git
    python3-colcon-common-extensions
    python3-rosdep
  )
  if [[ "${sdk_method}" == "source" ]]; then
    packages+=(
      libgl1-mesa-dev
      libopencv-dev
      libsoundio-dev
      libudev-dev
      libusb-1.0-0-dev
      libvulkan-dev
      libx11-dev
      libxcursor-dev
      libxinerama-dev
      libxrandr-dev
      libxxf86vm-dev
      pkg-config
    )
  fi
  sudo apt-get update
  sudo apt-get install -y "${packages[@]}"
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "${skip_rosdep}" == "false" ]]; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
  rosdep install --from-paths "${PROJECT_DIR}" --ignore-src --rosdistro humble -r -y
fi

sdk_args=(--method "${sdk_method}")
[[ "${offline}" == "false" ]] || sdk_args+=(--offline)
[[ "${accept_license}" == "false" ]] || sdk_args+=(--accept-sdk-license)
[[ -z "${cache_dir}" ]] || sdk_args+=(--cache-dir "${cache_dir}")
[[ -z "${source_dir}" ]] || sdk_args+=(--source-dir "${source_dir}")
"${SCRIPT_DIR}/setup_sdk.sh" "${sdk_args[@]}"

if [[ "${skip_udev}" == "false" ]]; then
  "${SCRIPT_DIR}/install_udev_rules.sh"
fi

"${SCRIPT_DIR}/build_ros.sh"

echo
echo "Bootstrap completed. In this terminal run:"
echo "  source ${PROJECT_DIR}/scripts/env.sh"
echo "  source ${PROJECT_DIR}/install/setup.bash"
echo "  ros2 launch k4a_ros k4a.launch.py use_rviz:=true"
