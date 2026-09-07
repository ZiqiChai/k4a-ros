#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_DIR
readonly LOCK_FILE="${PROJECT_DIR}/dependencies/azure-kinect-sdk-1.4.2.env"

# shellcheck source=../dependencies/azure-kinect-sdk-1.4.2.env
source "${LOCK_FILE}"

license_accepted="false"
cache_dir="${XDG_CACHE_HOME:-${HOME}/.cache}/k4a-ros"
bundle_cache_dir="${PROJECT_DIR}/dependencies/cache"

usage() {
  cat <<'EOF'
Usage: scripts/setup_sdk.sh [--accept-sdk-license]

Install the pinned Azure Kinect Sensor SDK runtime and development packages
system-wide on Ubuntu 22.04 amd64. The script also installs the udev rule.

Options:
  --accept-sdk-license  Accept the Microsoft SDK runtime license non-interactively.
  -h, --help            Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --accept-sdk-license)
      license_accepted="true"
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

if [[ "${ACCEPT_EULA:-}" == "Y" || "${ACCEPT_EULA:-}" == "y" ]]; then
  license_accepted="true"
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "22.04" ]]; then
  echo "This installer supports Ubuntu 22.04; detected ${PRETTY_NAME}." >&2
  exit 1
fi
if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "Azure Kinect SDK 1.4.2 and its Depth Engine require amd64." >&2
  exit 1
fi

run_root() {
  if ((EUID == 0)); then
    "$@"
  else
    sudo "$@"
  fi
}

verify_file() {
  local file="$1"
  local expected_sha256="$2"
  echo "${expected_sha256}  ${file}" | sha256sum --check --status
}

fetch_package() {
  local relative_url="$1"
  local filename="$2"
  local expected_sha256="$3"
  local destination="${cache_dir}/${filename}"
  local bundled="${bundle_cache_dir}/${filename}"

  if [[ -f "${destination}" ]] && verify_file "${destination}" "${expected_sha256}"; then
    echo "Using verified cache: ${destination}"
    return
  fi

  if [[ -f "${bundled}" ]] && verify_file "${bundled}" "${expected_sha256}"; then
    cp "${bundled}" "${destination}"
    echo "Using verified deployment bundle: ${bundled}"
    return
  fi

  if ! command -v curl >/dev/null 2>&1; then
    run_root apt-get update
    run_root env DEBIAN_FRONTEND=noninteractive apt-get install -y ca-certificates curl
  fi

  curl -fL --retry 3 --output "${destination}.part" \
    "${SDK_PACKAGE_BASE_URL}/${relative_url}"
  if ! verify_file "${destination}.part" "${expected_sha256}"; then
    echo "SHA-256 verification failed for ${filename}." >&2
    exit 1
  fi
  mv "${destination}.part" "${destination}"
}

require_sdk_license() {
  local answer
  if [[ "${license_accepted}" == "true" ]]; then
    return
  fi

  echo "Microsoft Azure Kinect Sensor SDK uses separate runtime license terms." >&2
  echo "The license is contained in ${cache_dir}/${SDK_RUNTIME_FILENAME}." >&2
  echo "After installation: /usr/share/doc/libk4a1.4/LICENSE.txt" >&2
  if [[ ! -t 0 ]]; then
    echo "Non-interactive installation requires --accept-sdk-license or ACCEPT_EULA=Y." >&2
    exit 1
  fi
  read -r -p "Do you accept the Microsoft Azure Kinect Sensor SDK license? [y/N] " answer
  if [[ "${answer}" != "Y" && "${answer}" != "y" ]]; then
    echo "SDK license was not accepted; installation stopped." >&2
    exit 1
  fi
}

mkdir -p "${cache_dir}"

fetch_package "${SDK_RUNTIME_RELATIVE_URL}" "${SDK_RUNTIME_FILENAME}" \
  "${SDK_RUNTIME_SHA256}"
fetch_package "${SDK_DEVEL_RELATIVE_URL}" "${SDK_DEVEL_FILENAME}" \
  "${SDK_DEVEL_SHA256}"
require_sdk_license

run_root env ACCEPT_EULA=Y DEBIAN_FRONTEND=noninteractive apt-get install -y \
  "${cache_dir}/${SDK_RUNTIME_FILENAME}" \
  "${cache_dir}/${SDK_DEVEL_FILENAME}"

run_root install -m 0644 "${SCRIPT_DIR}/99-k4a.rules" /etc/udev/rules.d/99-k4a.rules
if command -v udevadm >/dev/null 2>&1 && run_root udevadm control --reload-rules; then
  run_root udevadm trigger --subsystem-match=usb
else
  echo "udev daemon is unavailable; the rule is installed and will apply after reboot." >&2
fi

installed_runtime="$(dpkg-query -W -f='${Version}' libk4a1.4)"
installed_devel="$(dpkg-query -W -f='${Version}' libk4a1.4-dev)"
if [[ "${installed_runtime}" != "${SDK_VERSION}" || "${installed_devel}" != "${SDK_VERSION}" ]]; then
  echo "Unexpected installed SDK version: runtime=${installed_runtime}, dev=${installed_devel}." >&2
  exit 1
fi

test -f /usr/lib/x86_64-linux-gnu/cmake/k4a/k4aConfig.cmake
test -f /usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0

echo "Azure Kinect SDK ${SDK_VERSION} installed system-wide."
echo "udev rule installed; replug the camera if it is not detected."
echo "No SDK environment script is required for build or runtime."
