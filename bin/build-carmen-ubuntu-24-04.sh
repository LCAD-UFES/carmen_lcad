#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  build-carmen-ubuntu-24-04.sh [options]

Options:
  --configure        Run ./configure --nocuda --nopython before build.
  --clean            Run make clean in src before build.
  --jobs N           Number of parallel jobs. Default: nproc.
  --target NAME      Make target to run inside src. Default: total.
  --keep-going       Use make -k. Default: enabled.
  --stop-on-error    Disable make -k.
  -h, --help         Show this help.

Notes:
  - This script targets the Ubuntu 24.04 flow validated in this repository.
  - It does not install packages.
  - It does not touch NVIDIA drivers, CUDA, or cuDNN.
  - It prefers /usr/bin/python3-config over any Linuxbrew Python in PATH.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${REPO_ROOT}/src"

RUN_CONFIGURE=0
RUN_CLEAN=0
KEEP_GOING=1
TARGET="total"
JOBS="$(nproc)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --configure)
      RUN_CONFIGURE=1
      ;;
    --clean)
      RUN_CLEAN=1
      ;;
    --jobs)
      shift
      JOBS="${1:-}"
      ;;
    --target)
      shift
      TARGET="${1:-}"
      ;;
    --keep-going)
      KEEP_GOING=1
      ;;
    --stop-on-error)
      KEEP_GOING=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
  shift
done

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "Cannot find src directory: ${SRC_DIR}" >&2
  exit 1
fi

if [[ -z "${JOBS}" || ! "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Invalid value for --jobs: ${JOBS}" >&2
  exit 1
fi

export CARMEN_HOME="${REPO_ROOT}"
export PYTHON_CONFIG="/usr/bin/python3-config"

if [[ -d /usr/local/carmen_opencv_3_2/lib/pkgconfig ]]; then
  export PKG_CONFIG_PATH="/usr/local/carmen_opencv_3_2/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
fi

if [[ -d /usr/local/carmen_opencv_3_2/lib ]]; then
  export LD_LIBRARY_PATH="/usr/local/carmen_opencv_3_2/lib:/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
else
  export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

MAKE_ARGS=(-C "${SRC_DIR}")
if [[ "${KEEP_GOING}" -eq 1 ]]; then
  MAKE_ARGS+=(-k)
fi
MAKE_ARGS+=(-j"${JOBS}" "${TARGET}")

echo "CARMEN_HOME=${CARMEN_HOME}"
echo "PYTHON_CONFIG=${PYTHON_CONFIG}"
echo "PKG_CONFIG_PATH=${PKG_CONFIG_PATH:-}"
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}"
echo "Jobs=${JOBS}"
echo "Target=${TARGET}"
echo

if [[ "${RUN_CONFIGURE}" -eq 1 ]]; then
  echo "Running configure for Ubuntu 24.04 CPU baseline..."
  (
    cd "${SRC_DIR}"
    ./configure --nocuda --nopython
  )
  echo
fi

if [[ "${RUN_CLEAN}" -eq 1 ]]; then
  echo "Running make clean..."
  make -C "${SRC_DIR}" clean
  echo
fi

echo "Running make ${MAKE_ARGS[*]#-C ${SRC_DIR} }"
make "${MAKE_ARGS[@]}"
