#include "carmen_ros_bridge/platform.hpp"
#include "carmen_ros_bridge/carmen_time.hpp"

#include <csignal>
#include <map>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

// One ROS publisher per receiver, created on the first valid fix.
static std::map<int, ros::Publisher> gps_xyz_publishers;
static std::string gps_frame_id;
static bool publish_gps = true;
static const int publisher_queue_size = 50;
static const int ipc_wait_ms = 10;
// Signal handlers only request cleanup in main.
static volatile sig_atomic_t shutdown_requested = 0;

// Preserve position, timestamp and the receiver-quality covariance mapping.
static nav_msgs::Odometry
gps_xyz_to_msg(const carmen_gps_xyz_message &message)
{
    nav_msgs::Odometry msg;
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
        case 4:  sigma_xy = 0.02; break;
        case 5:  sigma_xy = 0.20; break;
        case 2:  sigma_xy = 1.00; break;
        case 1:  sigma_xy = 5.00; break;
        default: sigma_xy = 50.0; break;
    }
    for (auto &c : msg.pose.covariance) c = 0.0;
    msg.pose.covariance[0]  = sigma_xy * sigma_xy;
    msg.pose.covariance[7]  = sigma_xy * sigma_xy;
    msg.pose.covariance[14] = (sigma_xy * 3.0) * (sigma_xy * 3.0);
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
        ros::NodeHandle node;
        std::string topic = "/gps/xyz_raw/gps_" + std::to_string(message.nr);
        publisher = gps_xyz_publishers.emplace(
            message.nr, node.advertise<nav_msgs::Odometry>(topic, publisher_queue_size)).first;
        ROS_INFO("Novo publisher GPS criado: %s (nr=%d)", topic.c_str(), message.nr);
    }
    publisher->second.publish(gps_xyz_to_msg(message));
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

// Read the same private settings previously used by ipc_bridge_node.
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
        carmen_gps_xyz_subscribe_message(
            nullptr, reinterpret_cast<carmen_handler_t>(gps_xyz_handler), CARMEN_SUBSCRIBE_LATEST);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "carmen_to_ros_gps_xyz", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    read_parameters(private_node);

    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    carmen_param_check_version(argv[0]);
    subscribe_to_ipc_message();

    // No ROS callbacks; IPC handles all incoming samples.
    while (ros::ok() && !shutdown_requested)
        IPC_listenWait(ipc_wait_ms);

    IPC_disconnect();
    ros::shutdown();
    return 0;
}
