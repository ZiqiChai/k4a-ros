#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_DIR
readonly RELEASE_ROOT="${PROJECT_DIR}/.release"
readonly DIST_DIR="${PROJECT_DIR}/dist"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi
if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "This precompiled artifact target is amd64." >&2
  exit 1
fi

version="$(sed -n 's:.*<version>\([^<]*\)</version>.*:\1:p' "${PROJECT_DIR}/k4a_ros/package.xml" | head -n 1)"
if [[ -z "${version}" ]]; then
  echo "Unable to read package version." >&2
  exit 1
fi

artifact_name="k4a_ros-${version}-ubuntu22.04-ros2-humble-amd64"
stage_dir="${RELEASE_ROOT}/stage/${artifact_name}"
archive="${DIST_DIR}/${artifact_name}.tar.gz"
git_dirty="true"
if [[ -z "$(git -C "${PROJECT_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  git_dirty="false"
fi

set +u
source /opt/ros/humble/setup.bash
set -u
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

cmake -E remove_directory "${RELEASE_ROOT}/build"
cmake -E remove_directory "${RELEASE_ROOT}/install"
cmake -E remove_directory "${RELEASE_ROOT}/log"
cmake -E remove_directory "${stage_dir}"
mkdir -p "${DIST_DIR}" "${stage_dir}"

colcon --log-base "${RELEASE_ROOT}/log" build \
  --base-paths "${PROJECT_DIR}/k4a_ros" \
  --packages-select k4a_ros \
  --build-base "${RELEASE_ROOT}/build" \
  --install-base "${RELEASE_ROOT}/install" \
  --merge-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_CXX_FLAGS=-ffile-prefix-map=${PROJECT_DIR}=."

cp -a "${RELEASE_ROOT}/install" "${stage_dir}/ros2_install"
# POSIX sh cannot discover the path of a sourced file. Replace colcon's local
# build-time fallback with an intentionally absent path; setup.bash/setup.zsh
# still calculate the extracted prefix and pass it through normally.
while IFS= read -r generated_file; do
  sed -i "s|${RELEASE_ROOT}/install|/__k4a_ros_relocatable_prefix__|g" \
    "${generated_file}"
done < <(grep -IlR -F "${RELEASE_ROOT}/install" "${stage_dir}/ros2_install")
install -m 0644 "${PROJECT_DIR}/README.md" "${stage_dir}/README.md"
install -m 0644 "${PROJECT_DIR}/LICENSE" "${stage_dir}/LICENSE"
install -m 0644 "${PROJECT_DIR}/CHANGELOG.md" "${stage_dir}/CHANGELOG.md"
mkdir -p "${stage_dir}/scripts"
install -m 0755 \
  "${PROJECT_DIR}/scripts/setup_sdk.sh" \
  "${PROJECT_DIR}/scripts/env.sh" \
  "${PROJECT_DIR}/scripts/install_udev_rules.sh" \
  "${stage_dir}/scripts/"
install -m 0644 "${PROJECT_DIR}/scripts/99-k4a.rules" "${stage_dir}/scripts/99-k4a.rules"
cp -a "${PROJECT_DIR}/dependencies" "${stage_dir}/dependencies"
cp -a "${PROJECT_DIR}/docs" "${stage_dir}/docs"

{
  echo "package=k4a_ros"
  echo "version=${version}"
  echo "platform=ubuntu-22.04-amd64"
  echo "ros_distro=humble"
  echo "sdk=Azure-Kinect-Sensor-SDK-1.4.2-external"
  echo "sdk_lock_sha256=$(sha256sum "${PROJECT_DIR}/dependencies/azure-kinect-sdk-1.4.2.env" | cut -d ' ' -f 1)"
  echo "git_commit=$(git -C "${PROJECT_DIR}" rev-parse HEAD)"
  echo "git_dirty=${git_dirty}"
} > "${stage_dir}/BUILD_INFO"

(
  cd "${stage_dir}"
  manifest_tmp="$(mktemp)"
  find . -type f -print0 | sort -z | xargs -0 sha256sum > "${manifest_tmp}"
  mv "${manifest_tmp}" MANIFEST.sha256
)

tar --sort=name --mtime='@0' --owner=0 --group=0 --numeric-owner \
  -C "${RELEASE_ROOT}/stage" -cf - "${artifact_name}" | gzip -n > "${archive}"
(
  cd "${DIST_DIR}"
  sha256sum "$(basename "${archive}")" > "$(basename "${archive}").sha256"
)

echo "Created ${archive}"
echo "Azure Kinect SDK binaries are intentionally not included."
