#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

BUILD_DIR="${REPO_ROOT}/build"
JOBS="$(nproc)"
INSTALL_APT=1
DO_BUILD=1
WITH_OPENVINO=0
WITH_ROS2_MSGS=0
FORCE_OPENCV_APT=0
CONFIGURE_CAMERA_SDK=0
CLEAN_BUILD=0
PERSIST_ENV=1
SKIP_CHECKS=0
CHECK_ONLY=0
APT_UPDATED=0

TARGET_USER="${SUDO_USER:-$(id -un)}"
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"

usage() {
  cat <<'USAGE'
Usage: scripts/setup_env.sh [options]

One-command environment setup for sp_vision_25_rbclone.

Default behavior:
  - Install project apt dependencies.
  - Keep Jetson OpenCV untouched unless --force-opencv-apt is set.
  - Check industrial camera SDK libraries without changing linker config.
  - Verify CUDA, TensorRT, OpenCV, Ceres, and camera SDK libraries for the current
    default TensorRT build in CMakeLists.txt.
  - Configure and build ./build.

Options:
  --no-apt              Do not install apt packages.
  --no-build            Configure environment only; do not run cmake/build.
  --build-dir DIR       Build directory. Default: ./build.
  --jobs N              Parallel build jobs. Default: nproc.
  --clean-build         Remove build directory before configuring.
  --with-openvino       Also install/configure OpenVINO 2024.6 apt package.
  --with-ros2-msgs      Build sp_ws/src ROS2 message package if ROS2 is present.
  --force-opencv-apt    Install libopencv-dev even on Jetson/aarch64.
  --configure-camera-sdk
                        Register bundled camera SDK library paths with ldconfig.
  --no-persist          Do not append CUDA/OpenVINO env lines to ~/.bashrc.
  --skip-checks         Skip dependency verification before build.
  --check-only          Only verify dependencies; make no system changes.
  -h, --help            Show this help.

Notes:
  The repository currently hard-codes TENSOR_RT_MAKE=ON and OPENVINO_MAKE=OFF
  in top-level CMakeLists.txt, so the default build requires CUDA + TensorRT.
  Daheng Galaxy runtime can be provided system-wide or vendored as
  io/daheng/lib/<amd64|arm64>/libgxiapi.so.
USAGE
}

log() {
  printf '\033[1;32m[setup]\033[0m %s\n' "$*"
}

warn() {
  printf '\033[1;33m[warn]\033[0m %s\n' "$*" >&2
}

die() {
  printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2
  exit 1
}

run_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --no-apt)
        INSTALL_APT=0
        ;;
      --no-build)
        DO_BUILD=0
        ;;
      --build-dir)
        [[ $# -ge 2 ]] || die "--build-dir requires a value"
        BUILD_DIR="$2"
        shift
        ;;
      --jobs)
        [[ $# -ge 2 ]] || die "--jobs requires a value"
        JOBS="$2"
        shift
        ;;
      --clean-build)
        CLEAN_BUILD=1
        ;;
      --with-openvino)
        WITH_OPENVINO=1
        ;;
      --with-ros2-msgs)
        WITH_ROS2_MSGS=1
        ;;
      --force-opencv-apt)
        FORCE_OPENCV_APT=1
        ;;
      --configure-camera-sdk)
        CONFIGURE_CAMERA_SDK=1
        ;;
      --no-persist)
        PERSIST_ENV=0
        ;;
      --skip-checks)
        SKIP_CHECKS=1
        ;;
      --check-only)
        CHECK_ONLY=1
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        die "Unknown option: $1"
        ;;
    esac
    shift
  done
}

source_os_release() {
  OS_ID="unknown"
  OS_VERSION_ID="unknown"
  OS_CODENAME="unknown"

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    OS_ID="${ID:-unknown}"
    OS_VERSION_ID="${VERSION_ID:-unknown}"
    OS_CODENAME="${VERSION_CODENAME:-unknown}"
  fi
}

is_jetson() {
  [[ -f /etc/nv_tegra_release ]] && return 0
  [[ -r /proc/device-tree/model ]] && grep -qiE 'jetson|nvidia' /proc/device-tree/model
}

apt_update_once() {
  [[ "${INSTALL_APT}" -eq 1 ]] || return 0
  if [[ "${APT_UPDATED}" -eq 0 ]]; then
    log "Updating apt indexes"
    run_sudo env DEBIAN_FRONTEND=noninteractive apt-get update
    APT_UPDATED=1
  fi
}

apt_install() {
  [[ "${INSTALL_APT}" -eq 1 ]] || {
    log "Skipping apt install: $*"
    return 0
  }

  local missing=()
  local pkg
  for pkg in "$@"; do
    if ! dpkg-query -W -f='${Status}' "${pkg}" 2>/dev/null | grep -q "install ok installed"; then
      missing+=("${pkg}")
    fi
  done

  if [[ "${#missing[@]}" -eq 0 ]]; then
    log "Apt packages already installed: $*"
    return 0
  fi

  apt_update_once
  log "Installing apt packages: ${missing[*]}"
  run_sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}"
}

apt_install_if_available() {
  [[ "${INSTALL_APT}" -eq 1 ]] || {
    log "Skipping optional apt install: $*"
    return 0
  }

  apt_update_once
  local available=()
  local unavailable=()
  local pkg
  for pkg in "$@"; do
    if apt-cache show "${pkg}" >/dev/null 2>&1; then
      available+=("${pkg}")
    else
      unavailable+=("${pkg}")
    fi
  done

  if [[ "${#available[@]}" -gt 0 ]]; then
    apt_install "${available[@]}"
  fi
  if [[ "${#unavailable[@]}" -gt 0 ]]; then
    warn "Packages not found in current apt sources: ${unavailable[*]}"
  fi
}

append_line_once() {
  local file="$1"
  local line="$2"

  [[ "${PERSIST_ENV}" -eq 1 ]] || return 0
  mkdir -p "$(dirname "${file}")"
  touch "${file}"
  if ! grep -Fxq "${line}" "${file}"; then
    printf '%s\n' "${line}" >> "${file}"
  fi
}

detect_cuda_home() {
  if [[ -n "${CUDA_HOME:-}" && -x "${CUDA_HOME}/bin/nvcc" ]]; then
    printf '%s\n' "${CUDA_HOME}"
    return 0
  fi
  if [[ -x /usr/local/cuda/bin/nvcc ]]; then
    printf '%s\n' /usr/local/cuda
    return 0
  fi
  local nvcc_path
  nvcc_path="$(find /usr/local -maxdepth 3 -path '*/bin/nvcc' 2>/dev/null | sort -V | tail -n 1 || true)"
  if [[ -n "${nvcc_path}" ]]; then
    dirname "$(dirname "${nvcc_path}")"
    return 0
  fi
  if have_cmd nvcc; then
    dirname "$(dirname "$(command -v nvcc)")"
    return 0
  fi
  return 1
}

configure_cuda_env() {
  local cuda_home
  cuda_home="$(detect_cuda_home || true)"
  [[ -n "${cuda_home}" ]] || return 0

  export CUDA_HOME="${cuda_home}"
  export PATH="${CUDA_HOME}/bin:${PATH}"
  case "$(uname -m)" in
    aarch64)
      export LD_LIBRARY_PATH="${CUDA_HOME}/targets/aarch64-linux/lib:${LD_LIBRARY_PATH:-}"
      ;;
    x86_64)
      export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH:-}"
      ;;
  esac

  local bashrc="${TARGET_HOME}/.bashrc"
  append_line_once "${bashrc}" "# sp_vision CUDA environment"
  append_line_once "${bashrc}" "export CUDA_HOME=${CUDA_HOME}"
  append_line_once "${bashrc}" 'export PATH=$CUDA_HOME/bin:$PATH'
  if [[ "$(uname -m)" == "aarch64" ]]; then
    append_line_once "${bashrc}" 'export LD_LIBRARY_PATH=$CUDA_HOME/targets/aarch64-linux/lib:$LD_LIBRARY_PATH'
  else
    append_line_once "${bashrc}" 'export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH'
  fi

  log "CUDA detected at ${CUDA_HOME}"
}

install_base_packages() {
  source_os_release
  [[ "${OS_ID}" == "ubuntu" || "${INSTALL_APT}" -eq 0 ]] || warn "This script is tuned for Ubuntu; detected ${OS_ID} ${OS_VERSION_ID}"

  apt_install \
    git \
    g++ \
    build-essential \
    cmake \
    pkg-config \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \
    can-utils \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    libceres-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen \
    v4l-utils \
    usbutils

  if is_jetson && [[ "${FORCE_OPENCV_APT}" -eq 0 ]]; then
    log "Jetson/aarch64 detected; leaving OpenCV as provided by JetPack/system"
  else
    apt_install libopencv-dev
  fi
}

install_tensorrt_if_available() {
  if [[ -n "$(find_shared_lib libnvinfer.so)" ]]; then
    log "TensorRT runtime found"
    return 0
  fi

  warn "TensorRT was not found by ldconfig"
  if is_jetson; then
    warn "On Jetson, install JetPack/NVIDIA runtime first. If apt sources are configured, nvidia-jetpack can provide CUDA + TensorRT."
    apt_install_if_available nvidia-jetpack
  else
    warn "On x86, install CUDA and TensorRT from NVIDIA's official repository, then rerun this script."
    apt_install_if_available libnvinfer-dev libnvinfer-plugin-dev libnvonnxparsers-dev
  fi
}

install_openvino() {
  [[ "${WITH_OPENVINO}" -eq 1 ]] || return 0

  source_os_release
  if [[ "${OS_ID}" != "ubuntu" ]]; then
    warn "Skipping OpenVINO apt setup on non-Ubuntu system"
    return 0
  fi

  local repo_suite=""
  case "${OS_VERSION_ID}" in
    22.04) repo_suite="ubuntu22" ;;
    24.04) repo_suite="ubuntu24" ;;
    *)
      warn "OpenVINO apt repo mapping is not configured for Ubuntu ${OS_VERSION_ID}; skipping repo setup"
      return 0
      ;;
  esac

  local keyring="/usr/share/keyrings/intel-openvino-archive-keyring.gpg"
  if [[ ! -f "${keyring}" ]]; then
    log "Installing Intel OpenVINO apt key"
    curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
      | run_sudo gpg --dearmor -o "${keyring}"
  fi

  local source_file="/etc/apt/sources.list.d/intel-openvino.list"
  local source_line="deb [signed-by=${keyring}] https://apt.repos.intel.com/openvino ${repo_suite} main"
  if [[ ! -f "${source_file}" ]] || ! grep -Fxq "${source_line}" "${source_file}"; then
    log "Adding OpenVINO apt source"
    printf '%s\n' "${source_line}" | run_sudo tee "${source_file}" >/dev/null
    APT_UPDATED=0
  fi

  apt_install_if_available openvino-2024.6.0

  local openvino_dir=""
  local candidate
  for candidate in \
    /opt/intel/openvino_2024.6.0/runtime/cmake \
    /usr/lib/cmake/openvino2024.6.0 \
    /opt/intel/openvino/runtime/cmake; do
    if [[ -d "${candidate}" ]]; then
      openvino_dir="${candidate}"
      break
    fi
  done

  if [[ -n "${openvino_dir}" ]]; then
    export OpenVINO_DIR="${openvino_dir}"
    append_line_once "${TARGET_HOME}/.bashrc" "# sp_vision OpenVINO environment"
    append_line_once "${TARGET_HOME}/.bashrc" "export OpenVINO_DIR=${openvino_dir}"
    if [[ -f /opt/intel/openvino_2024.6.0/setupvars.sh ]]; then
      append_line_once "${TARGET_HOME}/.bashrc" "source /opt/intel/openvino_2024.6.0/setupvars.sh >/dev/null 2>&1 || true"
    fi
    log "OpenVINO detected at ${openvino_dir}"
  else
    warn "OpenVINO package was requested, but no OpenVINO CMake directory was found"
  fi
}

configure_camera_sdk_paths() {
  local sdk_arch
  case "$(uname -m)" in
    x86_64) sdk_arch="amd64" ;;
    aarch64) sdk_arch="arm64" ;;
    *) die "Unsupported architecture for bundled camera SDKs: $(uname -m)" ;;
  esac

  local paths=()
  [[ -d "${REPO_ROOT}/io/daheng/lib/${sdk_arch}" ]] && paths+=("${REPO_ROOT}/io/daheng/lib/${sdk_arch}")
  [[ -d "${REPO_ROOT}/io/hikrobot/lib/${sdk_arch}" ]] && paths+=("${REPO_ROOT}/io/hikrobot/lib/${sdk_arch}")
  [[ -d "${REPO_ROOT}/io/mindvision/lib/${sdk_arch}" ]] && paths+=("${REPO_ROOT}/io/mindvision/lib/${sdk_arch}")

  local cuda_home
  cuda_home="$(detect_cuda_home || true)"
  if [[ -n "${cuda_home}" ]]; then
    case "$(uname -m)" in
      aarch64) [[ -d "${cuda_home}/targets/aarch64-linux/lib" ]] && paths+=("${cuda_home}/targets/aarch64-linux/lib") ;;
      x86_64) [[ -d "${cuda_home}/lib64" ]] && paths+=("${cuda_home}/lib64") ;;
    esac
  fi

  [[ "${#paths[@]}" -gt 0 ]] || return 0

  local tmp
  tmp="$(mktemp)"
  printf '%s\n' "${paths[@]}" > "${tmp}"
  run_sudo install -m 0644 "${tmp}" /etc/ld.so.conf.d/sp_vision_25_rbclone.conf
  rm -f "${tmp}"
  run_sudo ldconfig
  log "Registered runtime library paths in /etc/ld.so.conf.d/sp_vision_25_rbclone.conf"
}

configure_user_permissions() {
  local group
  for group in dialout video plugdev; do
    if getent group "${group}" >/dev/null 2>&1; then
      run_sudo usermod -aG "${group}" "${TARGET_USER}" || warn "Could not add ${TARGET_USER} to ${group}"
    fi
  done
  log "Added ${TARGET_USER} to common device groups where available"
  warn "Group changes require logging out and back in to affect new shells"
}

find_shared_lib() {
  local lib_name="$1"
  local ld_cache
  local found
  local sdk_arch=""
  local search_roots=()

  ld_cache="$(ldconfig -p 2>/dev/null || true)"
  found="$(grep -F "${lib_name}" <<<"${ld_cache}" | head -n 1 | sed -E 's/.* => //' || true)"
  if [[ -n "${found}" ]]; then
    printf '%s\n' "${found}"
    return 0
  fi

  case "$(uname -m)" in
    x86_64) sdk_arch="amd64" ;;
    aarch64) sdk_arch="arm64" ;;
  esac
  if [[ -n "${sdk_arch}" ]]; then
    search_roots+=("${REPO_ROOT}/io/daheng/lib/${sdk_arch}")
    search_roots+=("${REPO_ROOT}/io/hikrobot/lib/${sdk_arch}")
    search_roots+=("${REPO_ROOT}/io/mindvision/lib/${sdk_arch}")
  fi
  search_roots+=(/usr/lib /usr/local/lib /lib /opt)

  found="$(
    find "${search_roots[@]}" \
      -name "${lib_name}*" -print -quit 2>/dev/null || true
  )"
  if [[ -n "${found}" ]]; then
    printf '%s\n' "${found}"
    return 0
  fi

  return 1
}

check_lib() {
  local lib_name="$1"
  local message="$2"
  local found
  found="$(find_shared_lib "${lib_name}" || true)"
  if [[ -n "${found}" ]]; then
    log "${lib_name} found: ${found}"
    return 0
  fi
  warn "${message}"
  return 1
}

check_opencv() {
  if have_cmd opencv_version; then
    log "OpenCV detected: $(opencv_version)"
    return 0
  fi
  if pkg-config --exists opencv4 2>/dev/null; then
    log "OpenCV detected by pkg-config: $(pkg-config --modversion opencv4)"
    return 0
  fi
  local opencv_config
  opencv_config="$(find /usr /usr/local /opt -name OpenCVConfig.cmake -print -quit 2>/dev/null || true)"
  if [[ -n "${opencv_config}" ]]; then
    log "OpenCVConfig.cmake found: ${opencv_config}"
    return 0
  fi
  warn "OpenCV was not found. This project needs OpenCV 4.5.4 or newer for the TensorRT build."
  return 1
}

check_cuda() {
  local cuda_home
  cuda_home="$(detect_cuda_home || true)"
  if [[ -n "${cuda_home}" ]]; then
    log "CUDA toolkit found: ${cuda_home}"
    return 0
  fi
  warn "CUDA toolkit was not found. Install JetPack on Jetson or CUDA Toolkit on x86."
  return 1
}

check_ceres() {
  if pkg-config --exists ceres-solver 2>/dev/null; then
    log "Ceres detected by pkg-config: $(pkg-config --modversion ceres-solver)"
    return 0
  fi

  local ceres_config
  ceres_config="$(find /usr /usr/local /opt -name CeresConfig.cmake -print -quit 2>/dev/null || true)"
  if [[ -n "${ceres_config}" ]]; then
    log "CeresConfig.cmake found: ${ceres_config}"
    return 0
  fi

  if dpkg-query -W -f='${Status}' libceres-dev 2>/dev/null | grep -q "install ok installed"; then
    log "libceres-dev is installed"
    return 0
  fi

  warn "Ceres was not found. Install libceres-dev, then rerun this script."
  return 1
}

verify_dependencies() {
  [[ "${SKIP_CHECKS}" -eq 0 ]] || {
    warn "Skipping dependency checks"
    return 0
  }

  local failures=0

  have_cmd cmake || { warn "cmake not found"; failures=$((failures + 1)); }
  have_cmd g++ || { warn "g++ not found"; failures=$((failures + 1)); }
  have_cmd make || { warn "make not found"; failures=$((failures + 1)); }

  check_opencv || failures=$((failures + 1))
  check_cuda || failures=$((failures + 1))
  check_ceres || failures=$((failures + 1))
  check_lib libnvinfer.so "TensorRT libnvinfer.so was not found. The current CMake default requires TensorRT." || failures=$((failures + 1))
  check_lib libgxiapi.so "Daheng Galaxy runtime libgxiapi.so was not found. Install Galaxy_camera.run or place libgxiapi.so under io/daheng/lib/<amd64|arm64>." || failures=$((failures + 1))
  check_lib libMvCameraControl.so "HikRobot runtime library was not found in ldconfig/common paths." || failures=$((failures + 1))
  check_lib libMVSDK.so "MindVision runtime library was not found in ldconfig/common paths." || failures=$((failures + 1))

  if [[ "${failures}" -gt 0 ]]; then
    if [[ "${DO_BUILD}" -eq 1 ]]; then
      die "${failures} required dependency check(s) failed; fix them and rerun, or use --no-build/--skip-checks intentionally."
    fi
    warn "${failures} dependency check(s) failed; continuing because --no-build is set"
  else
    log "Dependency checks passed"
  fi
}

build_ros2_msgs() {
  [[ "${WITH_ROS2_MSGS}" -eq 1 ]] || return 0

  local setup=""
  local candidate
  for candidate in /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/rolling/setup.bash; do
    if [[ -f "${candidate}" ]]; then
      setup="${candidate}"
      break
    fi
  done

  if [[ -z "${setup}" ]]; then
    warn "ROS2 setup.bash was not found; skipping sp_msgs build"
    return 0
  fi

  apt_install_if_available python3-colcon-common-extensions
  log "Building ROS2 message package with ${setup}"
  (
    # shellcheck disable=SC1090
    source "${setup}"
    cd "${REPO_ROOT}/sp_ws"
    colcon build --packages-select sp_msgs
  )
  append_line_once "${TARGET_HOME}/.bashrc" "# sp_vision ROS2 message environment"
  append_line_once "${TARGET_HOME}/.bashrc" "source ${setup}"
  append_line_once "${TARGET_HOME}/.bashrc" "source ${REPO_ROOT}/sp_ws/install/setup.bash"
}

run_build() {
  [[ "${DO_BUILD}" -eq 1 ]] || {
    log "Skipping build"
    return 0
  }

  if [[ "${CLEAN_BUILD}" -eq 1 ]]; then
    [[ "${BUILD_DIR}" == "${REPO_ROOT}"/* ]] || die "--clean-build refuses to remove a directory outside the repository"
    log "Removing build directory: ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
  fi

  log "Configuring CMake: ${BUILD_DIR}"
  cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release

  log "Building with ${JOBS} job(s)"
  cmake --build "${BUILD_DIR}" -j "${JOBS}"
}

main() {
  parse_args "$@"
  cd "${REPO_ROOT}"

  if [[ "${CHECK_ONLY}" -eq 1 ]]; then
    INSTALL_APT=0
    DO_BUILD=0
    PERSIST_ENV=0
  fi

  log "Repository: ${REPO_ROOT}"
  log "Target user: ${TARGET_USER}"

  if [[ "${CHECK_ONLY}" -eq 1 ]]; then
    log "Check-only mode: no apt, no ldconfig, no group changes, no build"
    configure_cuda_env
    verify_dependencies
    log "Check-only run finished"
    return 0
  fi

  if [[ "${INSTALL_APT}" -eq 1 && "${EUID}" -ne 0 ]]; then
    sudo -v
  fi

  install_base_packages
  install_tensorrt_if_available
  install_openvino
  configure_cuda_env
  if [[ "${CONFIGURE_CAMERA_SDK}" -eq 1 ]]; then
    configure_camera_sdk_paths
  else
    log "Skipping camera SDK linker configuration; dependency check will only report SDK library availability"
  fi
  configure_user_permissions
  build_ros2_msgs
  verify_dependencies
  run_build

  log "Done"
  log "Try: ${BUILD_DIR}/auto_aim_test"
}

main "$@"
