#pragma once
#include <carmen/carmen.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace carmen_bridge
{
// Preserve ROS1 fromSec rounding for IMU messages.
inline builtin_interfaces::msg::Time carmen_to_ros_time(double timestamp)
{
    if (!std::isfinite(timestamp) || timestamp < 0.0 ||
        timestamp >= static_cast<double>(std::numeric_limits<int32_t>::max()))
        throw std::runtime_error("Invalid CARMEN timestamp");
    int64_t seconds = static_cast<int64_t>(std::floor(timestamp));
    int64_t nanoseconds = static_cast<int64_t>(std::round((timestamp - seconds) * 1e9));
    return rclcpp::Time(seconds * 1000000000LL + nanoseconds);
}

inline int64_t carmen_to_ros_ns(double timestamp)
{
    return static_cast<int64_t>(timestamp * 1e9);
}

inline builtin_interfaces::msg::Time carmen_ns_to_ros_time(int64_t timestamp)
{
    return rclcpp::Time(std::max<int64_t>(timestamp, 0));
}

inline builtin_interfaces::msg::Time tf_stamp(
    const rclcpp::Node::SharedPtr &node, const builtin_interfaces::msg::Time &stamp)
{
    if (node->get_parameter("use_sim_time").as_bool())
        return stamp;
    return rclcpp::Clock(RCL_SYSTEM_TIME).now();
}

// Strip ROS arguments before connecting to CARMEN IPC.
inline void initialize_ipc(int argc, char **argv)
{
    auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char *> ipc_argv;
    for (auto &argument : arguments)
        ipc_argv.push_back(argument.data());
    ipc_argv.push_back(nullptr);
    carmen_ipc_initialize(static_cast<int>(arguments.size()), ipc_argv.data());
}
}
