#!/usr/bin/env bash

_k4a_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_k4a_project_dir="$(cd -- "${_k4a_script_dir}/.." && pwd)"
_k4a_lock_file="${_k4a_project_dir}/dependencies/azure-kinect-sdk-1.4.2.env"

# shellcheck source=../dependencies/azure-kinect-sdk-1.4.2.env
source "${_k4a_lock_file}"

_k4a_private_root="${_k4a_project_dir}/.deps/k4a-sdk/usr"
_k4a_lib_dir="${_k4a_private_root}/lib/x86_64-linux-gnu"

if [[ ! -f "${_k4a_lib_dir}/libk4a.so.${SDK_VERSION}" ]]; then
  echo "Azure Kinect SDK not found. Run ${_k4a_script_dir}/setup_sdk.sh first." >&2
  return 1 2>/dev/null || exit 1
fi

export K4A_SDK_ROOT="${_k4a_private_root}"
export CMAKE_PREFIX_PATH="${_k4a_private_root}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_k4a_lib_dir}:${_k4a_lib_dir}/libk4a1.4${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PATH="${_k4a_private_root}/bin:${PATH}"

unset SDK_VERSION SDK_GIT_TAG SDK_GIT_COMMIT SDK_GIT_REPOSITORY
unset SDK_PACKAGE_BASE_URL SDK_RUNTIME_RELATIVE_URL SDK_RUNTIME_FILENAME
unset SDK_RUNTIME_SHA256 SDK_DEVEL_RELATIVE_URL SDK_DEVEL_FILENAME
unset SDK_DEVEL_SHA256 SDK_TOOLS_RELATIVE_URL SDK_TOOLS_FILENAME SDK_TOOLS_SHA256
unset _k4a_script_dir _k4a_project_dir _k4a_lock_file _k4a_private_root _k4a_lib_dir
