#include "carmen_ros_bridge/sensor_support.hpp"
#include <carmen/gps_nmea_interface.h>

#include <csignal>
#include <string>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif

// ROS output and private configuration.
static rclcpp::Node::SharedPtr node;
static rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr gps_nmea_publisher;
static std::string gps_frame_id;
static bool publish_gps = true;
static const int publisher_queue_size = 50;
static const int ipc_wait_ms = 10;
// Set by SIGINT or SIGTERM; cleanup runs in main.
static volatile sig_atomic_t shutdown_requested = 0;

// Convert NMEA HDT heading (radians) to a quaternion, preserving timestamp.
static geometry_msgs::msg::QuaternionStamped
gps_nmea_to_msg(const carmen_gps_gphdt_message &message)
{
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp = carmen_bridge::carmen_ns_to_ros_time(
        carmen_bridge::carmen_to_ros_ns(message.timestamp));
    msg.header.frame_id = gps_frame_id;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, message.heading);
    msg.quaternion.x = quaternion.x();
    msg.quaternion.y = quaternion.y();
    msg.quaternion.z = quaternion.z();
    msg.quaternion.w = quaternion.w();
    return msg;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Publishers

// Publish only valid HDT measurements.
static void
publish_gps_nmea_message(const carmen_gps_gphdt_message &message)
{
    if (publish_gps && message.valid != 0)
        gps_nmea_publisher->publish(gps_nmea_to_msg(message));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers

// Handle the latest IPC HDT sample.
static void
gps_nmea_handler(carmen_gps_gphdt_message *message)
{
    if (message != nullptr)
        publish_gps_nmea_message(*message);
}

// Request shutdown without calling IPC from a signal handler.
static void
shutdown_module(int signal_number)
{
    shutdown_requested = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations

// Read output enable and frame settings.
static void
read_parameters()
{
    gps_frame_id = node->declare_parameter<std::string>("gps_frame_id", "utm");
    publish_gps = node->declare_parameter<bool>("publish_gps", true);
}

// Keep the original latest-message subscription policy.
static void
subscribe_to_ipc_message()
{
    if (publish_gps)
        carmen_gps_subscribe_nmea_hdt_message(
            nullptr, reinterpret_cast<carmen_handler_t>(gps_nmea_handler),
            CARMEN_SUBSCRIBE_LATEST);
}

int
main(int argc, char **argv)
{
    // The module owns SIGINT/SIGTERM; initialize ROS without installing handlers.
    rclcpp::contexts::get_global_default_context()->init(argc, argv);
    node = std::make_shared<rclcpp::Node>("carmen_to_ros_gps_nmea");
    read_parameters();

    carmen_bridge::initialize_ipc(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);

    gps_nmea_publisher = node->create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "/gps/heading_raw", rclcpp::QoS(publisher_queue_size));
    subscribe_to_ipc_message();

    // No ROS callbacks; only IPC needs dispatching.
    while (rclcpp::ok() && !shutdown_requested)
        IPC_listenWait(ipc_wait_ms);

    IPC_disconnect();
    rclcpp::shutdown();
    return 0;
}
