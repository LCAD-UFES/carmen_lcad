#include "carmen_ros_bridge/sensor_support.hpp"
#include <carmen/gps_xyz_interface.h>

#include <csignal>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <utility>
#include <map>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif

// One ROS publisher per receiver, created on the first valid fix.
static rclcpp::Node::SharedPtr node;
static std::map<int, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> gps_xyz_publishers;
static std::string gps_frame_id;
static bool publish_gps = true;
// Position standard deviations in meters; the vertical multiplier is dimensionless.
static double gps_sigma_xy_quality_1 = 5.0;
static double gps_sigma_xy_quality_2 = 1.0;
static double gps_sigma_xy_quality_4 = 0.02;
static double gps_sigma_xy_quality_5 = 0.20;
static double gps_sigma_xy_default = 50.0;
static double gps_sigma_z_multiplier = 3.0;
static const int publisher_queue_size = 50;
static const int ipc_wait_ms = 10;
// Signal handlers only request cleanup in main.
static volatile sig_atomic_t shutdown_requested = 0;

// Read startup-only uncertainty settings before connecting to IPC.
static double
read_positive_parameter(const std::string &name, double fallback)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const double value = node->declare_parameter<double>(name, fallback, descriptor);
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(name + " must be finite and positive");
    return value;
}

// Ensure the configured standard deviations produce usable variances.
static void
validate_position_variances()
{
    const std::pair<const char *, double> sigmas[] = {
        {"gps_sigma_xy_quality_1", gps_sigma_xy_quality_1},
        {"gps_sigma_xy_quality_2", gps_sigma_xy_quality_2},
        {"gps_sigma_xy_quality_4", gps_sigma_xy_quality_4},
        {"gps_sigma_xy_quality_5", gps_sigma_xy_quality_5},
        {"gps_sigma_xy_default", gps_sigma_xy_default},
    };
    for (const auto &entry : sigmas)
    {
        const double sigma_xy = entry.second;
        const double sigma_z = sigma_xy * gps_sigma_z_multiplier;
        const double variance_xy = sigma_xy * sigma_xy;
        const double variance_z = sigma_z * sigma_z;
        if (!std::isfinite(variance_xy) || variance_xy <= 0.0 ||
            !std::isfinite(variance_z) || variance_z <= 0.0)
            throw std::invalid_argument(std::string(entry.first) +
                " and gps_sigma_z_multiplier must produce finite, positive variances");
    }
}

// Preserve position and timestamp; assign covariance from the configured quality table.
static nav_msgs::msg::Odometry
gps_xyz_to_msg(const carmen_gps_xyz_message &message)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = carmen_bridge::carmen_ns_to_ros_time(carmen_bridge::carmen_to_ros_ns(message.timestamp));
    msg.header.frame_id = gps_frame_id;
    msg.child_frame_id  = "gps_" + std::to_string(message.nr);

    msg.pose.pose.position.x = message.x;
    msg.pose.pose.position.y = message.y;
    msg.pose.pose.position.z = message.z;

#ifdef MAKE_IN_CARMEn
    const double theta = 0.0;
#else
    const double theta = message.theta;
#endif
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    double sigma_xy;
    switch (message.gps_quality)
    {
        case 4:  sigma_xy = gps_sigma_xy_quality_4; break;
        case 5:  sigma_xy = gps_sigma_xy_quality_5; break;
        case 2:  sigma_xy = gps_sigma_xy_quality_2; break;
        case 1:  sigma_xy = gps_sigma_xy_quality_1; break;
        default: sigma_xy = gps_sigma_xy_default; break;
    }
    for (auto &c : msg.pose.covariance) c = 0.0;
    msg.pose.covariance[0]  = sigma_xy * sigma_xy;
    msg.pose.covariance[7]  = sigma_xy * sigma_xy;
    const double sigma_z = sigma_xy * gps_sigma_z_multiplier;
    msg.pose.covariance[14] = sigma_z * sigma_z;
#ifdef MAKE_IN_CARMEn

    msg.pose.covariance[21] = 1e6;
    msg.pose.covariance[28] = 1e6;
    msg.pose.covariance[35] = 1e6;
#else
    msg.pose.covariance[35] = 0.05;
#endif

    return msg;
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Publishers

// Publish raw position independently for each receiver.
static void
publish_gps_xyz_message(const carmen_gps_xyz_message &message)
{
    if (!publish_gps || message.gps_quality < 1)
        return;

    auto publisher = gps_xyz_publishers.find(message.nr);
    if (publisher == gps_xyz_publishers.end())
    {
        std::string topic = "/gps/xyz_raw/gps_" + std::to_string(message.nr);
        publisher = gps_xyz_publishers.emplace(
            message.nr, node->create_publisher<nav_msgs::msg::Odometry>(topic, rclcpp::QoS(publisher_queue_size))).first;
        RCLCPP_INFO(node->get_logger(), "Novo publisher GPS criado: %s (nr=%d)", topic.c_str(), message.nr);
    }
    publisher->second->publish(gps_xyz_to_msg(message));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers

// Ignore missing samples; fix validity is checked before publishing.
static void
gps_xyz_handler(carmen_gps_xyz_message *message)
{
    if (message != nullptr)
        publish_gps_xyz_message(*message);
}

// Defer IPC and ROS cleanup to the main loop.
static void
shutdown_module(int signal_number)
{
    shutdown_requested = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations

// Read output settings and the GPS quality uncertainty table.
static void
read_parameters()
{
    gps_frame_id = node->declare_parameter<std::string>("gps_frame_id", "utm");
    publish_gps = node->declare_parameter<bool>("publish_gps", true);

    gps_sigma_xy_quality_1 = read_positive_parameter("gps_sigma_xy_quality_1", gps_sigma_xy_quality_1);
    gps_sigma_xy_quality_2 = read_positive_parameter("gps_sigma_xy_quality_2", gps_sigma_xy_quality_2);
    gps_sigma_xy_quality_4 = read_positive_parameter("gps_sigma_xy_quality_4", gps_sigma_xy_quality_4);
    gps_sigma_xy_quality_5 = read_positive_parameter("gps_sigma_xy_quality_5", gps_sigma_xy_quality_5);
    gps_sigma_xy_default = read_positive_parameter("gps_sigma_xy_default", gps_sigma_xy_default);
    gps_sigma_z_multiplier = read_positive_parameter("gps_sigma_z_multiplier", gps_sigma_z_multiplier);
    validate_position_variances();
}

// Keep the original latest-message subscription policy.
static void
subscribe_to_ipc_message()
{
    if (publish_gps)
        carmen_gps_xyz_subscribe_message(
            nullptr, reinterpret_cast<carmen_handler_t>(gps_xyz_handler), CARMEN_SUBSCRIBE_LATEST);
}

int
main(int argc, char **argv)
{
    // The module owns SIGINT/SIGTERM; initialize ROS without installing handlers.
    rclcpp::contexts::get_global_default_context()->init(argc, argv);
    node = std::make_shared<rclcpp::Node>("carmen_to_ros_gps_xyz");
    try
    {
        read_parameters();
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(node->get_logger(), "Invalid GPS parameters: %s", error.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    carmen_bridge::initialize_ipc(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);
    subscribe_to_ipc_message();

    // No ROS callbacks; IPC handles all incoming samples.
    while (rclcpp::ok() && !shutdown_requested)
        IPC_listenWait(ipc_wait_ms);

    IPC_disconnect();
    rclcpp::shutdown();
    return 0;
}
