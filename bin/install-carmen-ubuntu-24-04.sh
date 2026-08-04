#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  install-carmen-ubuntu-24-04.sh [--dry-run] [--apply] [--with-pcl] [--with-caffe-deps]

Default behavior:
  --dry-run is the default. The script only simulates apt changes.

Flags:
  --dry-run          Simulate package installation only.
  --apply            Perform the apt installation.
  --with-pcl         Install the PCL/VTK toolchain for point cloud modules.
  --with-caffe-deps  Install Caffe/ENet-like dependencies.

Notes:
  - This script is intended for Ubuntu 24.04.
  - It does not install or modify NVIDIA drivers, CUDA, or cuDNN.
  - It does not downgrade system packages.
EOF
}

MODE="dry-run"
WITH_PCL=0
WITH_CAFFE_DEPS=0

for arg in "$@"; do
  case "$arg" in
    --dry-run)
      MODE="dry-run"
      ;;
    --apply)
      MODE="apply"
      ;;
    --with-pcl)
      WITH_PCL=1
      ;;
    --with-caffe-deps)
      WITH_CAFFE_DEPS=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot verify OS version: /etc/os-release not found." >&2
  exit 1
fi

. /etc/os-release

if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "This script targets Ubuntu 24.04. Detected: ${PRETTY_NAME:-unknown}." >&2
  exit 1
fi

BASELINE_PACKAGES=(
  build-essential g++ git swig byacc flex doxygen
  cmake cmake-curses-gui cmake-qt-gui pkg-config
  gimp meld vim tcsh wget
  freeglut3-dev
  libgtk2.0-dev libgtk-3-dev libgtkglext1 libgtkglext1-dev
  libimlib2 libimlib2-dev imagemagick libmagick++-dev
  libwrap0 libwrap0-dev tcpd
  libncurses-dev libgsl-dev
  libdc1394-dev libdc1394-utils libraw1394-11 libraw1394-dev
  libglade2-0 libglade2-dev
  libcurl4-openssl-dev
  libkml-dev liburiparser1 liburiparser-dev
  libusb-1.0-0 libusb-1.0-0-dev libusb-dev
  libxi-dev libxi6 libxmu-dev libxmu6
  libforms-dev libgflags-dev
  libespeak-dev libfftw3-dev
  libavcodec-dev libavformat-dev libswscale-dev
  libgstreamer-plugins-base1.0-dev
  libjpeg-dev libpng-dev libpng++-dev libtiff5-dev
  libeigen3-dev libboost-all-dev libflann-dev
  mpi-default-dev openmpi-bin openmpi-common
  libproj-dev libsuitesparse-dev libgtest-dev
  qtbase5-dev qtbase5-dev-tools qtchooser qttools5-dev
  libasound2-dev mpg123 portaudio19-dev libjsoncpp-dev
  libglew-dev libudev-dev
)

PCL_PACKAGES=(
  libpcl-dev
  pcl-tools
)

CAFFE_PACKAGES=(
  protobuf-compiler
  libgoogle-glog-dev
  liblmdb-dev
  libleveldb-dev
  libsnappy-dev
  libhdf5-dev
  libopenblas-dev
  libatlas-base-dev
)

PACKAGES=("${BASELINE_PACKAGES[@]}")

if [[ "$WITH_PCL" -eq 1 ]]; then
  PACKAGES+=("${PCL_PACKAGES[@]}")
fi

if [[ "$WITH_CAFFE_DEPS" -eq 1 ]]; then
  PACKAGES+=("${CAFFE_PACKAGES[@]}")
fi

is_installed() {
  dpkg-query -W -f='${Status}\n' "$1" 2>/dev/null | grep -qx 'install ok installed'
}

declare -A SEEN_PACKAGES=()
MISSING_PACKAGES=()

for package in "${PACKAGES[@]}"; do
  if [[ -n "${SEEN_PACKAGES[$package]:-}" ]]; then
    continue
  fi
  SEEN_PACKAGES["$package"]=1

  if ! is_installed "$package"; then
    MISSING_PACKAGES+=("$package")
  fi
done

APT_GET=(apt-get)
APT_INSTALL_FLAGS=(--no-install-recommends)
SUDO_CMD=()

if [[ "${EUID}" -ne 0 ]]; then
  SUDO_CMD=(sudo)
fi

if [[ "$MODE" == "dry-run" ]]; then
  APT_GET+=( -s )
fi

echo "Ubuntu: ${PRETTY_NAME}"
echo "Mode: $MODE"
echo "Include PCL stack: $WITH_PCL"
echo "Include Caffe-like dependencies: $WITH_CAFFE_DEPS"
echo
echo "Packages:"
printf '  %s\n' "${PACKAGES[@]}"
echo
echo "Already installed: $(( ${#PACKAGES[@]} - ${#MISSING_PACKAGES[@]} ))"
echo "Missing packages: ${#MISSING_PACKAGES[@]}"

if [[ "${#MISSING_PACKAGES[@]}" -gt 0 ]]; then
  echo
  echo "Packages that would be installed:"
  printf '  %s\n' "${MISSING_PACKAGES[@]}"
else
  echo
  echo "No missing packages detected. Nothing to do."
  exit 0
fi
echo

if [[ "$MODE" == "apply" ]]; then
  echo "Applying package installation."
  "${SUDO_CMD[@]}" apt-get update
  "${SUDO_CMD[@]}" apt-get install -y "${APT_INSTALL_FLAGS[@]}" "${MISSING_PACKAGES[@]}"
else
  echo "Simulating package installation."
  "${APT_GET[@]}" install "${APT_INSTALL_FLAGS[@]}" "${MISSING_PACKAGES[@]}"
fi
