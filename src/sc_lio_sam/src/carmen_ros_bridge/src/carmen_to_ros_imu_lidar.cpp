#include "carmen_ros_bridge/sensor_support.hpp"
#include <carmen/imu_interface.h>

#include <csignal>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#endif

// ROS output and private configuration.
static rclcpp::Node::SharedPtr node;
static rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
static rclcpp::Clock warning_clock(RCL_STEADY_TIME);
static bool publish_imu = true;
static std::string base_frame_id, imu_frame_id;
static double imu_rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
// Set by SIGINT or SIGTERM.
static volatile sig_atomic_t shutdown_requested = 0;
// Warning state is accessed only by the main loop.
static bool warn_on_xsens = true;
static bool lidar_imu_published = false;
static int lidar_imu_sensor_id = 0;
static std::string xsens_node_name;
static const double angular_covariance = 0.01;
static const double acceleration_covariance = 0.1;
static const int publisher_queue_size = 2000;

// Convert sensor values, preserving the original quaternion and timestamp.
static sensor_msgs::msg::Imu
imu_to_msg(const carmen_imu_message &message)
{
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = carmen_bridge::carmen_to_ros_time(message.timestamp);
    msg.header.frame_id = imu_frame_id;
    const double *rotation = imu_rotation;
    msg.linear_acceleration.x = 
        rotation[0] * message.accX + rotation[1] * message.accY + rotation[2] * message.accZ;
    msg.linear_acceleration.y = 
        rotation[3] * message.accX + rotation[4] * message.accY + rotation[5] * message.accZ;
    msg.linear_acceleration.z = 
        rotation[6] * message.accX + rotation[7] * message.accY + rotation[8] * message.accZ;
    msg.angular_velocity.x = 
        rotation[0] * message.gyroX + rotation[1] * message.gyroY + rotation[2] * message.gyroZ;
    msg.angular_velocity.y = 
        rotation[3] * message.gyroX + rotation[4] * message.gyroY + rotation[5] * message.gyroZ;
    msg.angular_velocity.z = 
        rotation[6] * message.gyroX + rotation[7] * message.gyroY + rotation[8] * message.gyroZ;
    msg.orientation.w = message.q0;
    msg.orientation.x = message.q1;
    msg.orientation.y = message.q2;
    msg.orientation.z = message.q3;
    msg.orientation_covariance[0] = angular_covariance;
    msg.orientation_covariance[4] = angular_covariance;
    msg.orientation_covariance[8] = angular_covariance;
    msg.angular_velocity_covariance[0]    = angular_covariance;
    msg.angular_velocity_covariance[4]    = angular_covariance;
    msg.angular_velocity_covariance[8]    = angular_covariance;
    msg.linear_acceleration_covariance[0] = acceleration_covariance;
    msg.linear_acceleration_covariance[4] = acceleration_covariance;
    msg.linear_acceleration_covariance[8] = acceleration_covariance;
    return msg;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Publishers

// Publish IMU data and send the static transform once.
static void
publish_imu_message(const carmen_imu_message &message)
{
    static tf2_ros::StaticTransformBroadcaster broadcaster(
        *node);
    static bool static_tf_sent = false;

    if (!static_tf_sent)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = carmen_bridge::tf_stamp(node,
            carmen_bridge::carmen_ns_to_ros_time(carmen_bridge::carmen_to_ros_ns(message.timestamp)));
        transform.header.frame_id = base_frame_id;
        transform.child_frame_id = imu_frame_id;
        transform.transform.rotation.w = 1.0;
        broadcaster.sendTransform(transform);
        static_tf_sent = true;
    }

    if (publish_imu)
    {
        imu_publisher->publish(imu_to_msg(message));
        lidar_imu_published = true;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers

// Forward every LiDAR IMU sample without sensor arbitration.
static void
lidar_imu_handler(carmen_imu_message *message)
{
    if (message != nullptr)
        publish_imu_message(*message);
}

// Match the DDS publisher identity to the configured Xsens node.
static void
ros_imu_handler(sensor_msgs::msg::Imu::ConstSharedPtr, const rclcpp::MessageInfo &info)
{
    if (!warn_on_xsens || !lidar_imu_published)
        return;
    const auto &gid = info.get_rmw_message_info().publisher_gid;
    for (const auto &publisher : node->get_publishers_info_by_topic("/imu_raw"))
    {
        std::string name = publisher.node_namespace();
        if (name != "/")
            name += "/";
        name += publisher.node_name();
        const auto &endpoint_gid = publisher.endpoint_gid();
        if (name == xsens_node_name && std::equal(endpoint_gid.begin(), endpoint_gid.end(), gid.data))
        {
            RCLCPP_WARN_THROTTLE(node->get_logger(), warning_clock, 5000,
                "IMU do LiDAR ja publicou e Xsens esta publicando no mesmo topico ROS. "
                "Use warn_on_xsens:=false para silenciar ou desative um dos publishs.");
            break;
        }
    }
}

// Request shutdown; IPC cleanup runs outside the signal handler.
static void
shutdown_module(int signal_number)
{
    shutdown_requested = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations

// Read private ROS parameters before connecting to IPC.
static void
read_parameters()
{
    base_frame_id = node->declare_parameter<std::string>("base_frame_id", "base_link");
    imu_frame_id = node->declare_parameter<std::string>("imu_frame_id", "imu_link");

    publish_imu = node->declare_parameter<bool>("publish_imu", true);
    warn_on_xsens = node->declare_parameter<bool>("warn_on_xsens", true);
    lidar_imu_sensor_id = node->declare_parameter<int>("lidar_imu_sensor_id", 0);
    xsens_node_name = node->declare_parameter<std::string>("xsens_node_name", "carmen_to_ros_imu_xsens");
    if (!xsens_node_name.empty() && xsens_node_name[0] != '/')
        xsens_node_name = std::string(node->get_namespace()) +
            (std::string(node->get_namespace()) == "/" ? "" : "/") + xsens_node_name;

    std::vector<double> rotation;
    rotation = node->declare_parameter<std::vector<double>>("imu_rotation", {1, 0, 0, 0, 1, 0, 0, 0, 1});
    if (rotation.size() == 9)
        for (int i = 0; i < 9; i++)
            imu_rotation[i] = rotation[i];
    else
        RCLCPP_WARN(node->get_logger(), "imu_rotation deve conter 9 valores; usando identidade.");
}

// Subscribe to the selected LiDAR IMU.
static void
subscribe_to_ipc_message()
{
    carmen_imu_subscribe_imu_lidar_message(
        nullptr, reinterpret_cast<carmen_handler_t>(lidar_imu_handler),
        CARMEN_SUBSCRIBE_LATEST, lidar_imu_sensor_id);
}

int
main(int argc, char **argv)
{
    // The module owns SIGINT/SIGTERM; initialize ROS without installing handlers.
    rclcpp::contexts::get_global_default_context()->init(argc, argv);
    node = std::make_shared<rclcpp::Node>("carmen_to_ros_imu_lidar");
    read_parameters();
    imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("/imu_raw", rclcpp::QoS(publisher_queue_size));


    carmen_bridge::initialize_ipc(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);

    subscribe_to_ipc_message();
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    if (warn_on_xsens)
        imu_subscriber = node->create_subscription<sensor_msgs::msg::Imu>("/imu_raw", 100, ros_imu_handler);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // IPC and ROS callbacks run on the same thread.
    while (rclcpp::ok() && !shutdown_requested)
    {
        IPC_listenWait(10);
        executor.spin_some();
    }
    IPC_disconnect();

    rclcpp::shutdown();
    return 0;
}
