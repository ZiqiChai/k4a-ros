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

cache_dir="${XDG_CACHE_HOME:-${HOME}/.cache}/k4a-ros"
bundled_source="${PROJECT_DIR}/dependencies/source/Azure-Kinect-Sensor-SDK"
checkout="${K4A_SDK_SOURCE_DIR:-${bundled_source}}"
output_dir="${PROJECT_DIR}/dist/sdk-system"

if [[ ! -d "${checkout}/.git" ]]; then
  checkout="${cache_dir}/source/Azure-Kinect-Sensor-SDK"
fi
if [[ ! -d "${checkout}/.git" ]]; then
  mkdir -p "$(dirname -- "${checkout}")"
  git clone --branch "${SDK_GIT_TAG}" --recursive "${SDK_GIT_REPOSITORY}" "${checkout}"
fi

if [[ "$(git -C "${checkout}" rev-parse HEAD)" != "${SDK_GIT_COMMIT}" ]]; then
  echo "Unexpected SDK source revision: ${checkout}" >&2
  echo "Expected ${SDK_GIT_COMMIT}." >&2
  exit 1
fi
if git -C "${checkout}" submodule status --recursive | grep -Eq '^[-+U]'; then
  git -C "${checkout}" submodule update --init --recursive
fi
if [[ -n "$(git -C "${checkout}" status --porcelain --untracked-files=no)" ]]; then
  echo "SDK source archive has tracked modifications: ${checkout}" >&2
  exit 1
fi

runtime_deb="${PROJECT_DIR}/dependencies/cache/${SDK_RUNTIME_FILENAME}"
devel_deb="${PROJECT_DIR}/dependencies/cache/${SDK_DEVEL_FILENAME}"
if [[ ! -f "${runtime_deb}" ]]; then
  runtime_deb="${cache_dir}/${SDK_RUNTIME_FILENAME}"
fi
if [[ ! -f "${devel_deb}" ]]; then
  devel_deb="${cache_dir}/${SDK_DEVEL_FILENAME}"
fi
if [[ ! -f "${runtime_deb}" || ! -f "${devel_deb}" ]]; then
  echo "Verified runtime/dev packages are missing. Run scripts/setup_sdk.sh first." >&2
  exit 1
fi
echo "${SDK_RUNTIME_SHA256}  ${runtime_deb}" | sha256sum --check --status
echo "${SDK_DEVEL_SHA256}  ${devel_deb}" | sha256sum --check --status

stage_root="$(mktemp -d)"
trap 'rm -rf -- "${stage_root}"' EXIT
source_stage="${stage_root}/source"
build_dir="${stage_root}/build"
cp -a "${checkout}" "${source_stage}"
if git -C "${source_stage}" apply --check "${PATCH_FILE}"; then
  git -C "${source_stage}" apply "${PATCH_FILE}"
else
  echo "Ubuntu 22.04 compatibility patch does not apply cleanly." >&2
  exit 1
fi

cmake -S "${source_stage}" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DVERSION_STR="${SDK_VERSION}" \
  -DK4A_VALIDATE_CLANG_FORMAT=OFF \
  -Duse_openssl=OFF \
  -DCMAKE_C_FLAGS=-Wno-error=array-parameter
cmake --build "${build_dir}" --target k4a --parallel "$(nproc)"
built_library="$(readlink -f "${build_dir}/bin/libk4a.so")"

runtime_stage="${stage_root}/runtime"
devel_stage="${stage_root}/devel"
dpkg-deb --raw-extract "${runtime_deb}" "${runtime_stage}"
dpkg-deb --raw-extract "${devel_deb}" "${devel_stage}"
install -m 0755 "${built_library}" \
  "${runtime_stage}/usr/lib/x86_64-linux-gnu/libk4a.so.${SDK_VERSION}"

package_version="${SDK_VERSION}+source.${SDK_GIT_COMMIT:0:8}"
sed -i "s/^Version: .*/Version: ${package_version}/" "${runtime_stage}/DEBIAN/control"
sed -i "s/^Version: .*/Version: ${package_version}/" "${devel_stage}/DEBIAN/control"
sed -Ei \
  "s/libk4a1\.4 \(= [^)]+\)/libk4a1.4 (= ${package_version})/" \
  "${devel_stage}/DEBIAN/control"
grep -Fq "libk4a1.4 (= ${package_version})" "${devel_stage}/DEBIAN/control"

refresh_package_metadata() {
  local stage="$1"
  local installed_size
  installed_size="$(du -sk "${stage}/usr" | cut -f1)"
  sed -i "s/^Installed-Size: .*/Installed-Size: ${installed_size}/" \
    "${stage}/DEBIAN/control"
  (
    cd "${stage}"
    find . -type f ! -path './DEBIAN/*' -printf '%P\0' \
      | sort -z \
      | xargs -0 md5sum
  ) > "${stage}/DEBIAN/md5sums"
}

refresh_package_metadata "${runtime_stage}"
refresh_package_metadata "${devel_stage}"

mkdir -p "${output_dir}"
runtime_output="${output_dir}/libk4a1.4_${package_version}_amd64.deb"
devel_output="${output_dir}/libk4a1.4-dev_${package_version}_amd64.deb"
dpkg-deb --build --root-owner-group "${runtime_stage}" "${runtime_output}"
dpkg-deb --build --root-owner-group "${devel_stage}" "${devel_output}"
(
  cd "${output_dir}"
  sha256sum "$(basename -- "${runtime_output}")" "$(basename -- "${devel_output}")"
) > "${output_dir}/SHA256SUMS"

echo "Source-rebuilt system packages created:"
echo "  ${runtime_output}"
echo "  ${devel_output}"
echo "Install locally after license review with:"
echo "  sudo env ACCEPT_EULA=Y apt-get install '${runtime_output}' '${devel_output}'"
