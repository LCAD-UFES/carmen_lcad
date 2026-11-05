#pragma once
#include "carmen_ros_bridge/sensor_support.hpp"
#include <rosgraph_msgs/msg/clock.hpp>
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#endif
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

static rclcpp::Node::SharedPtr bridge_node;
static rclcpp::Clock log_clock(RCL_STEADY_TIME);

namespace carmen_bridge
{
template<class T> void parameter(const std::string &key, T &value, const T &fallback)
{
    value = bridge_node->declare_parameter<T>(key, fallback);
}
inline void parameter(const std::string &key, std::vector<int> &value, const std::vector<int> &fallback)
{
    auto result = bridge_node->declare_parameter<std::vector<int64_t>>(
        key, std::vector<int64_t>(fallback.begin(), fallback.end()));
    value.assign(result.begin(), result.end());
}
template<class T>
bool
required_parameter(const std::string &key, T &value)
{
    if (!bridge_node->has_parameter(key))
    {
        const auto &overrides = bridge_node->get_node_parameters_interface()->get_parameter_overrides();
        const auto entry = overrides.find(key);
        if (entry == overrides.end())
            return false;
        bridge_node->declare_parameter(key, entry->second);
    }
    return bridge_node->get_parameter(key, value);
}
inline bool use_sim_time() { return bridge_node->get_parameter("use_sim_time").as_bool(); }
inline builtin_interfaces::msg::Time tf_stamp(const rclcpp::Time &stamp)
{
    if (use_sim_time()) return stamp;
    return rclcpp::Clock(RCL_SYSTEM_TIME).now();
}
inline void publish_clock(const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr &pub,
                          const rclcpp::Time &stamp)
{
    rosgraph_msgs::msg::Clock msg;
    msg.clock = stamp;
    pub->publish(msg);
}
inline auto
static_broadcaster()
{
    return std::make_shared<tf2_ros::StaticTransformBroadcaster>(*bridge_node);
}
}
