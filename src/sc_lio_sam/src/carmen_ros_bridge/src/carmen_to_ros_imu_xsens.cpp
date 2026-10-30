#include "carmen_ros_bridge/platform.hpp"
#include "carmen_ros_bridge/carmen_time.hpp"

#include <csignal>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ROS output and private configuration.
static ros::Publisher imu_publisher;
static bool publish_imu = true;
static std::string base_frame_id, imu_frame_id, ipc_host;
static double imu_rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
// Set by SIGINT or SIGTERM.
static volatile sig_atomic_t shutdown_requested = 0;
// Threshold for a stalled Xsens sample.
static const double xsens_zero_epsilon = 1e-6;
static const double angular_covariance = 0.01;
static const double acceleration_covariance = 0.1;
static const int publisher_queue_size = 2000;

// Convert sensor values, preserving the original quaternion and timestamp.
static sensor_msgs::Imu
imu_to_msg(const carmen_xsens_global_quat_message &message)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = carmen_bridge::carmen_to_ros_time(message.timestamp);
    msg.header.frame_id = imu_frame_id;
    const double *rotation = imu_rotation;
    msg.linear_acceleration.x = 
        rotation[0] * message.m_acc.x + rotation[1] * message.m_acc.y + rotation[2] * message.m_acc.z;
    msg.linear_acceleration.y = 
        rotation[3] * message.m_acc.x + rotation[4] * message.m_acc.y + rotation[5] * message.m_acc.z;
    msg.linear_acceleration.z = 
        rotation[6] * message.m_acc.x + rotation[7] * message.m_acc.y + rotation[8] * message.m_acc.z;
    msg.angular_velocity.x = 
        rotation[0] * message.m_gyr.x + rotation[1] * message.m_gyr.y + rotation[2] * message.m_gyr.z;
    msg.angular_velocity.y = 
        rotation[3] * message.m_gyr.x + rotation[4] * message.m_gyr.y + rotation[5] * message.m_gyr.z;
    msg.angular_velocity.z = 
        rotation[6] * message.m_gyr.x + rotation[7] * message.m_gyr.y + rotation[8] * message.m_gyr.z;
    msg.orientation.w = message.quat_data.m_data[0];
    msg.orientation.x = message.quat_data.m_data[1];
    msg.orientation.y = message.quat_data.m_data[2];
    msg.orientation.z = message.quat_data.m_data[3];
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
publish_imu_message(const carmen_xsens_global_quat_message &message)
{
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    static bool static_tf_sent = false;

    if (!static_tf_sent)
    {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = carmen_bridge::tf_stamp(
            carmen_bridge::carmen_ns_to_ros_time(carmen_bridge::carmen_to_ros_ns(message.timestamp)));
        transform.header.frame_id = base_frame_id;
        transform.child_frame_id = imu_frame_id;
        transform.transform.rotation.w = 1.0;
        broadcaster.sendTransform(transform);
        static_tf_sent = true;
    }

    if (publish_imu)
        imu_publisher.publish(imu_to_msg(message));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers

static void
xsens_handler(carmen_xsens_global_quat_message *message)
{
    if (message == nullptr)
        return;

    bool is_zeroed =
        std::fabs(message->m_acc.x) < xsens_zero_epsilon &&
        std::fabs(message->m_acc.y) < xsens_zero_epsilon &&
        std::fabs(message->m_acc.z) < xsens_zero_epsilon &&
        std::fabs(message->m_gyr.x) < xsens_zero_epsilon &&
        std::fabs(message->m_gyr.y) < xsens_zero_epsilon &&
        std::fabs(message->m_gyr.z) < xsens_zero_epsilon;
    if (is_zeroed) {
        ROS_WARN_THROTTLE(5.0,
        "Mensagem do IMU esta com valores zerados");
    }

    publish_imu_message(*message);
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
read_parameters(ros::NodeHandle &private_node)
{
    private_node.param<std::string>("base_frame_id", base_frame_id, "base_link");
    private_node.param<std::string>("imu_frame_id", imu_frame_id, "imu_link");
    private_node.param<std::string>("ipc_host", ipc_host, "localhost");

    private_node.param("publish_imu", publish_imu, true);

    std::vector<double> rotation;
    private_node.param("imu_rotation", rotation, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    if (rotation.size() == 9)
        for (int i = 0; i < 9; i++)
            imu_rotation[i] = rotation[i];
    else
        ROS_WARN("imu_rotation deve conter 9 valores; usando identidade.");
}

// Subscribe to Xsens using the existing latest-message policy.
static void
subscribe_to_ipc_message()
{
    carmen_xsens_subscribe_xsens_global_quat_message(
        nullptr, reinterpret_cast<carmen_handler_t>(xsens_handler), CARMEN_SUBSCRIBE_LATEST);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "carmen_to_ros_imu_xsens", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    read_parameters(private_node);
    imu_publisher = node.advertise<sensor_msgs::Imu>("/imu_raw", publisher_queue_size);

    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);

    subscribe_to_ipc_message();

    // A bounded wait also handles ROS shutdown requests.
    while (ros::ok() && !shutdown_requested)
        IPC_listenWait(10);
    IPC_disconnect();

    ros::shutdown();
    return 0;
}
