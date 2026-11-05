# Source this file before running native ROS2 commands.
_sc_lio_sam_setup()
{
    local workspace
    workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" || return
    local distro="${SC_LIO_SAM_ROS_DISTRO:-}"
    local candidate

    if [[ -z "$distro" && "${ROS_VERSION:-}" == 2 ]]; then
        distro="${ROS_DISTRO:-}"
    fi
    if [[ -z "$distro" ]]; then
        for candidate in foxy lyrical; do
            if [[ -f "/opt/ros/$candidate/setup.bash" ]]; then
                if [[ -n "$distro" ]]; then
                    echo 'Multiple ROS 2 installations: set SC_LIO_SAM_ROS_DISTRO to foxy or lyrical.' >&2
                    return 1
                fi
                distro="$candidate"
            fi
        done
    fi
    case "$distro" in
        foxy|lyrical) ;;
        *) echo 'Select ROS 2 foxy or lyrical with SC_LIO_SAM_ROS_DISTRO.' >&2; return 1 ;;
    esac
    if [[ ! -f "/opt/ros/$distro/setup.bash" || ! -f "$workspace/install/local_setup.bash" ]]; then
        echo "Missing ROS 2 $distro or workspace installation; build for this distribution first." >&2
        return 1
    fi

    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
    source "/opt/ros/$distro/setup.bash" || return
    source "$workspace/install/local_setup.bash" || return
}

if _sc_lio_sam_setup; then
    unset -f _sc_lio_sam_setup
else
    unset -f _sc_lio_sam_setup
    return 1
fi
