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
static sensor_msgs::Imu
imu_to_msg(const carmen_imu_message &message)
{
    sensor_msgs::Imu msg;
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
    {
        imu_publisher.publish(imu_to_msg(message));
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

// Inspect ROS publisher identity to ignore our own IMU messages.
static void
ros_imu_handler(const ros::MessageEvent<sensor_msgs::Imu const> &event)
{
    if (warn_on_xsens && lidar_imu_published && event.getPublisherName() == xsens_node_name)
        ROS_WARN_THROTTLE(5.0,
            "IMU do LiDAR ja publicou e Xsens esta publicando no mesmo topico ROS. "
            "Desative um nodo, use warn_on_xsens:=false para silenciar ou desative um dos publishs.");
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

    private_node.param("publish_imu", publish_imu, true);
    private_node.param("warn_on_xsens", warn_on_xsens, true);
    private_node.param("lidar_imu_sensor_id", lidar_imu_sensor_id, 0);
    private_node.param<std::string>("xsens_node_name", xsens_node_name, "carmen_to_ros_imu_xsens");
    xsens_node_name = ros::names::resolve(xsens_node_name);

    std::vector<double> rotation;
    private_node.param("imu_rotation", rotation, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    if (rotation.size() == 9)
        for (int i = 0; i < 9; i++)
            imu_rotation[i] = rotation[i];
    else
        ROS_WARN("imu_rotation deve conter 9 valores; usando identidade.");
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
    ros::init(argc, argv, "carmen_to_ros_imu_lidar", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    read_parameters(private_node);
    imu_publisher = node.advertise<sensor_msgs::Imu>("/imu_raw", publisher_queue_size);


    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);

    subscribe_to_ipc_message();
    ros::Subscriber imu_subscriber;
    if (warn_on_xsens)
        imu_subscriber = node.subscribe("/imu_raw", 100, ros_imu_handler);

    // A bounded wait also handles ROS shutdown requests.
    while (ros::ok() && !shutdown_requested)
    {
        IPC_listenWait(10);
    }
    IPC_disconnect();

    ros::shutdown();
    return 0;
}
