#include <carmen/carmen.h>
#include <carmen/base_ackerman_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <string>
#include <vector>
#include <geometry_msgs/msg/twist_stamped.hpp>

// ROS output and private configuration.
static rclcpp::Node::SharedPtr node;
static rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odometry_publisher;
static std::string base_frame_id;
static bool publish_ackermann = true;
static const int publisher_queue_size = 200;
static const int ipc_wait_ms = 10;
// Set by SIGINT or SIGTERM; cleanup runs in main.
static volatile sig_atomic_t shutdown_requested = 0;

// Preserve raw speed (m/s), steering angle (rad) and timestamp.
static geometry_msgs::msg::TwistStamped
odometry_to_msg(const carmen_base_ackerman_odometry_message &message)
{
    geometry_msgs::msg::TwistStamped msg;
    int64_t stamp_ns = static_cast<int64_t>(message.timestamp * 1e9);
    if (stamp_ns < 0)
        stamp_ns = 0;
    msg.header.stamp = rclcpp::Time(stamp_ns);
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
        odometry_publisher->publish(odometry_to_msg(message));
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
read_parameters()
{
    base_frame_id = node->declare_parameter<std::string>("base_frame_id", "base_link");
    publish_ackermann = node->declare_parameter<bool>("publish_ackermann", true);
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
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    node = std::make_shared<rclcpp::Node>("carmen_to_ros_odometry");
    read_parameters();

    // Keep ROS arguments out of the CARMEN parameter parser.
    auto ipc_args = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char *> ipc_argv;
    for (auto &arg : ipc_args)
        ipc_argv.push_back(arg.data());
    ipc_argv.push_back(nullptr);
    carmen_ipc_initialize(static_cast<int>(ipc_args.size()), ipc_argv.data());
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);

    odometry_publisher = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ackermann/odom_raw", rclcpp::QoS(publisher_queue_size).reliable().durability_volatile());
    subscribe_to_ipc_message();

    // No ROS callbacks; only IPC needs dispatching.
    while (rclcpp::ok() && !shutdown_requested)
        IPC_listenWait(ipc_wait_ms);

    IPC_disconnect();
    odometry_publisher.reset();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
