#!/usr/bin/env bash

# Source this file before running Carmen on Ubuntu 24.04.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced, not executed." >&2
  echo "Use: source $(basename "$0")" >&2
  exit 1
fi

_carmen_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_carmen_repo_root="$(cd "${_carmen_env_dir}/.." && pwd)"

export CARMEN_HOME="${_carmen_repo_root}"
export PYTHON_CONFIG="/usr/bin/python3-config"
export PATH="${_carmen_env_dir}:${PATH}"

if [[ -d /usr/local/carmen_opencv_3_2/lib/pkgconfig ]]; then
  case ":${PKG_CONFIG_PATH:-}:" in
    *":/usr/local/carmen_opencv_3_2/lib/pkgconfig:"*) ;;
    *) export PKG_CONFIG_PATH="/usr/local/carmen_opencv_3_2/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}" ;;
  esac
fi

if [[ -d /usr/local/carmen_opencv_3_2/lib ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":/usr/local/carmen_opencv_3_2/lib:"*)
      ;;
    *)
      export LD_LIBRARY_PATH="/usr/local/carmen_opencv_3_2/lib:/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
      ;;
  esac
else
  case ":${LD_LIBRARY_PATH:-}:" in
    *":/usr/local/lib:"*) ;;
    *) export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
  esac
fi

unset _carmen_env_dir
unset _carmen_repo_root
