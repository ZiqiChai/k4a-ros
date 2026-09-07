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

license_accepted="false"

usage() {
  cat <<'EOF'
Usage: tools/package_sdk_recovery_bundle.sh [--accept-sdk-license]

Create an SDK-only recovery archive for an authorized offline deployment.
The archive contains the pinned recursive source checkout, compatibility
patch, lock files, and verified Microsoft runtime/development packages.
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

runtime_deb="${PROJECT_DIR}/dependencies/cache/${SDK_RUNTIME_FILENAME}"
devel_deb="${PROJECT_DIR}/dependencies/cache/${SDK_DEVEL_FILENAME}"
source_checkout="${PROJECT_DIR}/dependencies/source/Azure-Kinect-Sensor-SDK"

if [[ "${license_accepted}" != "true" ]]; then
  if [[ ! -t 0 ]]; then
    echo "Non-interactive packaging requires --accept-sdk-license or ACCEPT_EULA=Y." >&2
    exit 1
  fi
  read -r -p \
    "Have you accepted the Microsoft SDK license for this recovery bundle? [y/N] " answer
  if [[ "${answer}" != "Y" && "${answer}" != "y" ]]; then
    echo "SDK license was not accepted; packaging stopped." >&2
    exit 1
  fi
fi

echo "${SDK_RUNTIME_SHA256}  ${runtime_deb}" | sha256sum --check --status
echo "${SDK_DEVEL_SHA256}  ${devel_deb}" | sha256sum --check --status
if [[ ! -d "${source_checkout}/.git" ]]; then
  echo "Pinned recursive SDK checkout is missing: ${source_checkout}" >&2
  exit 1
fi
if [[ "$(git -C "${source_checkout}" rev-parse HEAD)" != "${SDK_GIT_COMMIT}" ]]; then
  echo "Unexpected SDK source revision: ${source_checkout}" >&2
  exit 1
fi
if git -C "${source_checkout}" submodule status --recursive | grep -Eq '^[-+U]'; then
  echo "SDK source submodules are missing or at unexpected revisions." >&2
  exit 1
fi
if [[ -n "$(git -C "${source_checkout}" status --porcelain --untracked-files=no)" ]]; then
  echo "SDK source archive has tracked modifications: ${source_checkout}" >&2
  exit 1
fi
git -C "${source_checkout}" apply --check "${PATCH_FILE}"

output_dir="${PROJECT_DIR}/dist/sdk-recovery"
archive_base="k4a-sdk-recovery-${SDK_VERSION}-ubuntu22.04-amd64"
mkdir -p "${output_dir}"
stage_root="$(mktemp -d)"
trap 'rm -rf -- "${stage_root}"' EXIT
mkdir -p "${stage_root}/dependencies"
manifest="${stage_root}/dependencies/SDK_RECOVERY_MANIFEST.txt"
{
  echo "Azure Kinect Sensor SDK recovery bundle"
  echo "SDK version: ${SDK_VERSION}"
  echo "SDK source commit: ${SDK_GIT_COMMIT}"
  echo "Runtime SHA-256: ${SDK_RUNTIME_SHA256}"
  echo "Development SHA-256: ${SDK_DEVEL_SHA256}"
  echo
  echo "Recursive submodules:"
  git -C "${source_checkout}" submodule status --recursive
} > "${manifest}"

archive_tar="${output_dir}/${archive_base}.tar"
archive_gz="${archive_tar}.gz"
tar --create --file "${archive_tar}" --owner=0 --group=0 --numeric-owner \
  --directory "${PROJECT_DIR}" \
  dependencies/azure-kinect-sdk-1.4.2.env \
  dependencies/SHA256SUMS \
  dependencies/README.md \
  dependencies/cache/README.md \
  "dependencies/cache/${SDK_RUNTIME_FILENAME}" \
  "dependencies/cache/${SDK_DEVEL_FILENAME}" \
  dependencies/source/README.md \
  dependencies/source/Azure-Kinect-Sensor-SDK \
  dependencies/patches/0001-disable-unused-tls-backend.patch
tar --append --file "${archive_tar}" --owner=0 --group=0 --numeric-owner \
  --directory "${stage_root}" dependencies/SDK_RECOVERY_MANIFEST.txt
gzip --no-name --force "${archive_tar}"
(
  cd "${output_dir}"
  sha256sum "$(basename -- "${archive_gz}")"
) > "${archive_gz}.sha256"

echo "SDK recovery bundle created:"
echo "  ${archive_gz}"
echo "  ${archive_gz}.sha256"
echo "Extract it at the k4a-ros repository root before running setup_sdk.sh."
echo "Store or distribute it only as permitted by the included SDK license."
