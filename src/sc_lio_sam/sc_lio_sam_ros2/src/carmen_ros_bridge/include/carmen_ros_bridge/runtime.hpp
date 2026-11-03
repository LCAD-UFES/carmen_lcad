#pragma once
#include "carmen_ros_bridge/sensor_support.hpp"
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
template<class T> bool required_parameter(const std::string &key, T &value)
{
    if (!bridge_node->has_parameter(key))
        bridge_node->declare_parameter(key, rclcpp::ParameterValue(T{}).get_type());
    try { return bridge_node->get_parameter(key, value); }
    catch (const rclcpp::exceptions::ParameterUninitializedException &) { return false; }
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
inline auto static_broadcaster()
{
    return std::make_shared<tf2_ros::StaticTransformBroadcaster>(
        tf2_ros::StaticTransformBroadcaster::RequiredInterfaces(
            bridge_node->get_node_parameters_interface(), bridge_node->get_node_topics_interface()));
}
}
