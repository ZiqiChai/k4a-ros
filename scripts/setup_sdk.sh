#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_DIR
readonly LOCK_FILE="${PROJECT_DIR}/dependencies/azure-kinect-sdk-1.4.2.env"
readonly PATCH_FILE="${PROJECT_DIR}/dependencies/patches/0001-disable-unused-tls-backend.patch"

# shellcheck source=../dependencies/azure-kinect-sdk-1.4.2.env
source "${LOCK_FILE}"

method="binary"
offline="false"
license_accepted="false"
cache_dir="${K4A_CACHE_DIR:-${PROJECT_DIR}/.deps/cache}"
source_dir="${K4A_SDK_SOURCE_DIR:-}"
package_base_url="${K4A_PACKAGE_BASE_URL:-${SDK_PACKAGE_BASE_URL}}"
git_repository="${K4A_SDK_GIT_REPOSITORY:-${SDK_GIT_REPOSITORY}}"
sdk_root="${PROJECT_DIR}/.deps/k4a-sdk"

usage() {
  cat <<'EOF'
Usage: scripts/setup_sdk.sh [options]

Options:
  --method binary|source  Install verified official binaries (default) or
                          compile libk4a from the pinned source revision.
  --cache-dir PATH        Read/write dependency packages in PATH.
  --source-dir PATH       Use an existing recursive SDK checkout for source mode.
  --offline               Disable downloads; require a complete local cache/checkout.
  --accept-sdk-license    Accept the Microsoft Azure Kinect Sensor SDK license.
  -h, --help              Show this help.

Mirrors can be selected with K4A_PACKAGE_BASE_URL and K4A_SDK_GIT_REPOSITORY.
EOF
}

while (($#)); do
  case "$1" in
    --method)
      [[ $# -ge 2 ]] || { echo "--method requires a value" >&2; exit 2; }
      method="$2"
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

if [[ "${method}" != "binary" && "${method}" != "source" ]]; then
  echo "Unsupported method '${method}'; choose binary or source." >&2
  exit 2
fi

if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "Azure Kinect SDK 1.4.2 official runtime and Depth Engine require amd64." >&2
  exit 1
fi

mkdir -p "${cache_dir}" "${sdk_root}"
cache_dir="$(cd -- "${cache_dir}" && pwd)"

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

  if [[ -f "${destination}" ]] && verify_file "${destination}" "${expected_sha256}"; then
    echo "Using verified cache: ${destination}"
    return
  fi

  if [[ "${offline}" == "true" ]]; then
    echo "Offline cache is missing or invalid: ${destination}" >&2
    echo "Expected SHA-256: ${expected_sha256}" >&2
    exit 1
  fi

  curl -fL --retry 3 --output "${destination}.part" \
    "${package_base_url}/${relative_url}"
  if ! verify_file "${destination}.part" "${expected_sha256}"; then
    echo "SHA-256 verification failed for ${filename}." >&2
    exit 1
  fi
  mv "${destination}.part" "${destination}"
}

fetch_official_packages() {
  fetch_package "${SDK_RUNTIME_RELATIVE_URL}" "${SDK_RUNTIME_FILENAME}" \
    "${SDK_RUNTIME_SHA256}"
  fetch_package "${SDK_DEVEL_RELATIVE_URL}" "${SDK_DEVEL_FILENAME}" \
    "${SDK_DEVEL_SHA256}"
  fetch_package "${SDK_TOOLS_RELATIVE_URL}" "${SDK_TOOLS_FILENAME}" \
    "${SDK_TOOLS_SHA256}"
}

extract_official_packages() {
  dpkg-deb --extract "${cache_dir}/${SDK_RUNTIME_FILENAME}" "${sdk_root}"
  dpkg-deb --extract "${cache_dir}/${SDK_DEVEL_FILENAME}" "${sdk_root}"
  dpkg-deb --extract "${cache_dir}/${SDK_TOOLS_FILENAME}" "${sdk_root}"
}

require_sdk_license() {
  local answer
  if [[ "${license_accepted}" == "true" ]]; then
    return
  fi

  echo "Microsoft Azure Kinect Sensor SDK uses separate runtime license terms." >&2
  echo "The license is contained in ${cache_dir}/${SDK_RUNTIME_FILENAME}." >&2
  echo "After installation it is available at:" >&2
  echo "  ${sdk_root}/usr/share/doc/libk4a1.4/LICENSE.txt" >&2

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

prepare_source() {
  local checkout
  if [[ -n "${source_dir}" ]]; then
    checkout="$(cd -- "${source_dir}" && pwd)"
  else
    checkout="${PROJECT_DIR}/.deps/src/Azure-Kinect-Sensor-SDK"
    if [[ ! -d "${checkout}/.git" ]]; then
      if [[ "${offline}" == "true" ]]; then
        echo "Offline source checkout not found: ${checkout}" >&2
        echo "Provide one with --source-dir PATH." >&2
        exit 1
      fi
      mkdir -p "$(dirname -- "${checkout}")"
      git clone --branch "${SDK_GIT_TAG}" --recursive "${git_repository}" "${checkout}"
    fi
  fi

  if [[ ! -d "${checkout}/.git" ]]; then
    echo "SDK source must be a Git checkout with initialized submodules: ${checkout}" >&2
    exit 1
  fi
  if [[ "$(git -C "${checkout}" rev-parse HEAD)" != "${SDK_GIT_COMMIT}" ]]; then
    echo "Unexpected SDK revision in ${checkout}." >&2
    echo "Expected: ${SDK_GIT_COMMIT}" >&2
    exit 1
  fi
  if git -C "${checkout}" submodule status --recursive | grep -Eq '^[+-]'; then
    if [[ "${offline}" == "true" ]]; then
      echo "SDK submodules are incomplete or at unexpected revisions." >&2
      exit 1
    fi
    git -C "${checkout}" submodule update --init --recursive >&2
  fi

  if git -C "${checkout}" apply --reverse --check "${PATCH_FILE}" 2>/dev/null; then
    : # Patch is already applied.
  elif git -C "${checkout}" apply --check "${PATCH_FILE}"; then
    git -C "${checkout}" apply "${PATCH_FILE}"
  else
    echo "The Ubuntu 22.04 compatibility patch does not apply cleanly." >&2
    exit 1
  fi

  printf '%s\n' "${checkout}"
}

build_source_library() {
  local checkout="$1"
  local build_dir="${PROJECT_DIR}/.deps/build/Azure-Kinect-Sensor-SDK-${SDK_VERSION}"
  local built_library
  local target_library="${sdk_root}/usr/lib/x86_64-linux-gnu/libk4a.so.${SDK_VERSION}"

  cmake -S "${checkout}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DVERSION_STR="${SDK_VERSION}" \
    -DK4A_VALIDATE_CLANG_FORMAT=OFF \
    -Duse_openssl=OFF \
    -DCMAKE_C_FLAGS=-Wno-error=array-parameter
  cmake --build "${build_dir}" --target k4a --parallel "$(nproc)"

  built_library="$(readlink -f "${build_dir}/bin/libk4a.so")"
  install -m 0755 "${built_library}" "${target_library}"
  echo "Source-built libk4a deployed to ${target_library}."
}

fetch_official_packages
require_sdk_license
extract_official_packages

if [[ "${method}" == "source" ]]; then
  checkout="$(prepare_source)"
  build_source_library "${checkout}"
fi

printf '%s\n' "${method}" > "${sdk_root}/.install-method"
echo "Azure Kinect SDK ${SDK_VERSION} (${method}) is ready in ${sdk_root}."
echo "Run: source ${SCRIPT_DIR}/env.sh"
