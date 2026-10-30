#include "carmen_ros_bridge/platform.hpp"
#include "carmen_ros_bridge/carmen_time.hpp"

#include <csignal>
#include <string>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// ROS output and private configuration.
static ros::Publisher gps_nmea_publisher;
static std::string gps_frame_id;
static bool publish_gps = true;
static const int publisher_queue_size = 50;
static const int ipc_wait_ms = 10;
// Set by SIGINT or SIGTERM; cleanup runs in main.
static volatile sig_atomic_t shutdown_requested = 0;

// Convert NMEA HDT heading (radians) to a quaternion, preserving timestamp.
static geometry_msgs::QuaternionStamped
gps_nmea_to_msg(const carmen_gps_gphdt_message &message)
{
    geometry_msgs::QuaternionStamped msg;
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
        gps_nmea_publisher.publish(gps_nmea_to_msg(message));
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
read_parameters(ros::NodeHandle &private_node)
{
    private_node.param<std::string>("gps_frame_id", gps_frame_id, "utm");
    private_node.param("publish_gps", publish_gps, true);
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
    ros::init(argc, argv, "carmen_to_ros_gps_nmea", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    read_parameters(private_node);

    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);

    gps_nmea_publisher = node.advertise<geometry_msgs::QuaternionStamped>(
        "/gps/heading_raw", publisher_queue_size);
    subscribe_to_ipc_message();

    // No ROS callbacks; only IPC needs dispatching.
    while (ros::ok() && !shutdown_requested)
        IPC_listenWait(ipc_wait_ms);

    IPC_disconnect();
    ros::shutdown();
    return 0;
}
