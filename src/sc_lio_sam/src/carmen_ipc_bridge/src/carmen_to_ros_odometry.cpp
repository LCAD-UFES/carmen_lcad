#include "carmen_ipc_bridge/platform.hpp"
#include "carmen_ipc_bridge/carmen_time.hpp"

#include <csignal>
#include <string>
#include <geometry_msgs/TwistStamped.h>

// ROS output and private configuration.
static ros::Publisher odometry_publisher;
static std::string base_frame_id;
static bool publish_ackermann = true;
static const int publisher_queue_size = 200;
static const int ipc_wait_ms = 10;
// Set by SIGINT or SIGTERM; cleanup runs in main.
static volatile sig_atomic_t shutdown_requested = 0;

// Preserve raw speed (m/s), steering angle (rad) and timestamp.
static geometry_msgs::TwistStamped
odometry_to_msg(const carmen_base_ackerman_odometry_message &message)
{
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = carmen_bridge::carmen_ns_to_ros_time(
        carmen_bridge::carmen_to_ros_ns(message.timestamp));
    msg.header.frame_id = base_frame_id;
    msg.twist.linear.x = message.v;
    // angular.z carries steering angle, not yaw rate.
    msg.twist.angular.z = message.phi;
    return msg;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Publishers

// Forward raw odometry; SC-LIO-SAM integrates the bicycle model.
static void
publish_odometry_message(const carmen_base_ackerman_odometry_message &message)
{
    if (publish_ackermann)
        odometry_publisher.publish(odometry_to_msg(message));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers

// Handle the latest IPC odometry sample.
static void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *message)
{
    if (message != nullptr)
        publish_odometry_message(*message);
}

// Request shutdown without calling IPC from a signal handler.
static void
shutdown_module(int signal_number)
{
    shutdown_requested = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations

// Read the same output settings previously used by ipc_bridge_node.
static void
read_parameters(ros::NodeHandle &private_node)
{
    private_node.param<std::string>("base_frame_id", base_frame_id, "base_link");
    private_node.param("publish_ackermann", publish_ackermann, true);
}

// Keep the original latest-message subscription policy.
static void
subscribe_to_ipc_message()
{
    if (publish_ackermann)
        carmen_base_ackerman_subscribe_odometry_message(
            nullptr, reinterpret_cast<carmen_handler_t>(base_ackerman_odometry_handler),
            CARMEN_SUBSCRIBE_LATEST);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "carmen_to_ros_odometry", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    read_parameters(private_node);

    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);

    odometry_publisher = node.advertise<geometry_msgs::TwistStamped>(
        "/ackermann/odom_raw", publisher_queue_size);
    subscribe_to_ipc_message();

    // No ROS callbacks; only IPC needs dispatching.
    while (ros::ok() && !shutdown_requested)
        IPC_listenWait(ipc_wait_ms);

    IPC_disconnect();
    ros::shutdown();
    return 0;
}
