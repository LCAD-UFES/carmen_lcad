#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#if __has_include(<tf2/LinearMath/Transform.hpp>)
#include <tf2/LinearMath/Transform.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#endif
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif
#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif
#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif
#include <pcl/common/transforms.h>
#include <deque>
#include <mutex>
#include <memory>
using namespace std;

inline rclcpp::Node::SharedPtr node;
inline rclcpp::Clock log_clock(RCL_STEADY_TIME);
inline double stamp_seconds(const builtin_interfaces::msg::Time &stamp)
{
    return double(stamp.sec) + double(stamp.nanosec) * 1e-9;
}
template<typename T> double message_time(T msg)
{
    return stamp_seconds(msg->header.stamp);
}
inline tf2::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}
inline geometry_msgs::msg::TransformStamped stamped_transform(
    const tf2::Transform &transform, const builtin_interfaces::msg::Time &stamp,
    const string &parent, const string &child)
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.transform = tf2::toMsg(transform);
    return msg;
}

class ParamServer
{
public:
    std::string imuTopic;
    std::string odomTopic;
    std::string ackermannTopic;
    std::string odometrySource;
    float wheelbase;
    float ackermannVelNoise;
    float ackermannSteerNoise;
    float ackermannSlipNoiseX;
    float ackermannSlipNoiseY;
    float ackermannSlipNoiseTheta;
    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    Eigen::Matrix3d extRot;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    std::string pointCloudTopic;
    int slamQueueSize;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int lidarJumpSize;
    float lidarMaxRange;

    float edgeThreshold;
    float surfThreshold;
    float odometrySurfLeafSize;

    ParamServer()
    {
        if (!node->has_parameter("edgeThreshold"))
            node->declare_parameter<double>("edgeThreshold", 0.1);
        edgeThreshold = node->get_parameter("edgeThreshold").as_double();
        if (!node->has_parameter("surfThreshold"))
            node->declare_parameter<double>("surfThreshold", 0.1);
        surfThreshold = node->get_parameter("surfThreshold").as_double();
        if (!node->has_parameter("odometrySurfLeafSize"))
            node->declare_parameter<double>("odometrySurfLeafSize", 0.2);
        odometrySurfLeafSize = node->get_parameter("odometrySurfLeafSize").as_double();
        if (!node->has_parameter("pointCloudTopic"))
            node->declare_parameter<std::string>("pointCloudTopic", "points_raw");
        pointCloudTopic = node->get_parameter("pointCloudTopic").get_value<std::string>();
        if (!node->has_parameter("slamQueueSize"))
            node->declare_parameter<int>("slamQueueSize", 1);
        slamQueueSize = node->get_parameter("slamQueueSize").get_value<int>();
        if (!node->has_parameter("N_SCAN"))
            node->declare_parameter<int>("N_SCAN", 16);
        N_SCAN = node->get_parameter("N_SCAN").get_value<int>();
        if (!node->has_parameter("Horizon_SCAN"))
            node->declare_parameter<int>("Horizon_SCAN", 1800);
        Horizon_SCAN = node->get_parameter("Horizon_SCAN").get_value<int>();
        if (!node->has_parameter("downsampleRate"))
            node->declare_parameter<int>("downsampleRate", 1);
        downsampleRate = node->get_parameter("downsampleRate").get_value<int>();
        if (!node->has_parameter("lidarJumpSize"))
            node->declare_parameter<int>("lidarJumpSize", 1);
        lidarJumpSize = node->get_parameter("lidarJumpSize").get_value<int>();
        if (!node->has_parameter("lidarMaxRange"))
            node->declare_parameter<double>("lidarMaxRange", 1000.0);
        lidarMaxRange = node->get_parameter("lidarMaxRange").get_value<double>();
        if (lidarJumpSize < 1)
        {
            RCLCPP_WARN(node->get_logger(), "lidarJumpSize=%d invalido, usando 1 (todos os raios).", lidarJumpSize);
            lidarJumpSize = 1;
        }

        if (!node->has_parameter("imuTopic"))
            node->declare_parameter<std::string>("imuTopic", "imu_correct");
        imuTopic = node->get_parameter("imuTopic").get_value<std::string>();
        if (!node->has_parameter("odomTopic"))
            node->declare_parameter<std::string>("odomTopic", "odometry/imu");
        odomTopic = node->get_parameter("odomTopic").get_value<std::string>();
        if (!node->has_parameter("ackermannTopic"))
            node->declare_parameter<std::string>("ackermannTopic", "/ackermann/odom_raw");
        ackermannTopic = node->get_parameter("ackermannTopic").get_value<std::string>();
        if (!node->has_parameter("odometrySource"))
            node->declare_parameter<std::string>("odometrySource", "imu");
        odometrySource = node->get_parameter("odometrySource").get_value<std::string>();
        if (!node->has_parameter("wheelbase"))
            node->declare_parameter<double>("wheelbase", 2.625);
        wheelbase = node->get_parameter("wheelbase").get_value<double>();
        if (!node->has_parameter("ackermannVelNoise"))
            node->declare_parameter<double>("ackermannVelNoise", 0.1);
        ackermannVelNoise = node->get_parameter("ackermannVelNoise").get_value<double>();
        if (!node->has_parameter("ackermannSteerNoise"))
            node->declare_parameter<double>("ackermannSteerNoise", 0.02);
        ackermannSteerNoise = node->get_parameter("ackermannSteerNoise").get_value<double>();
        if (!node->has_parameter("ackermannSlipNoiseX"))
            node->declare_parameter<double>("ackermannSlipNoiseX", 0.02);
        ackermannSlipNoiseX = node->get_parameter("ackermannSlipNoiseX").get_value<double>();
        if (!node->has_parameter("ackermannSlipNoiseY"))
            node->declare_parameter<double>("ackermannSlipNoiseY", 0.05);
        ackermannSlipNoiseY = node->get_parameter("ackermannSlipNoiseY").get_value<double>();
        if (!node->has_parameter("ackermannSlipNoiseTheta"))
            node->declare_parameter<double>("ackermannSlipNoiseTheta", 0.01);
        ackermannSlipNoiseTheta = node->get_parameter("ackermannSlipNoiseTheta").get_value<double>();
        if (!node->has_parameter("lidarFrame"))
            node->declare_parameter<std::string>("lidarFrame", "base_link");
        lidarFrame = node->get_parameter("lidarFrame").get_value<std::string>();
        if (!node->has_parameter("baselinkFrame"))
            node->declare_parameter<std::string>("baselinkFrame", "base_link");
        baselinkFrame = node->get_parameter("baselinkFrame").get_value<std::string>();
        if (!node->has_parameter("odometryFrame"))
            node->declare_parameter<std::string>("odometryFrame", "odom");
        odometryFrame = node->get_parameter("odometryFrame").get_value<std::string>();
        if (!node->has_parameter("mapFrame"))
            node->declare_parameter<std::string>("mapFrame", "map");
        mapFrame = node->get_parameter("mapFrame").get_value<std::string>();
        if (!node->has_parameter("imuAccNoise"))
            node->declare_parameter<double>("imuAccNoise", 0.01);
        imuAccNoise = node->get_parameter("imuAccNoise").get_value<double>();
        if (!node->has_parameter("imuGyrNoise"))
            node->declare_parameter<double>("imuGyrNoise", 0.001);
        imuGyrNoise = node->get_parameter("imuGyrNoise").get_value<double>();
        if (!node->has_parameter("imuAccBiasN"))
            node->declare_parameter<double>("imuAccBiasN", 0.0002);
        imuAccBiasN = node->get_parameter("imuAccBiasN").get_value<double>();
        if (!node->has_parameter("imuGyrBiasN"))
            node->declare_parameter<double>("imuGyrBiasN", 0.00003);
        imuGyrBiasN = node->get_parameter("imuGyrBiasN").get_value<double>();
        if (!node->has_parameter("imuGravity"))
            node->declare_parameter<double>("imuGravity", 9.80511);
        imuGravity = node->get_parameter("imuGravity").get_value<double>();
        if (node->get_node_parameters_interface()->get_parameter_overrides().count("extrinsicRot"))
            throw std::runtime_error("extrinsicRot is deprecated; use extrinsicRPY [roll, pitch, yaw]");
        if (!node->has_parameter("extrinsicRPY"))
            node->declare_parameter<vector<double>>("extrinsicRPY", {0.0, 0.0, 0.0});
        if (!node->has_parameter("extrinsicTrans"))
            node->declare_parameter<vector<double>>("extrinsicTrans", vector<double>{});
        const auto rpy = node->get_parameter("extrinsicRPY").as_double_array();
        const auto trans = node->get_parameter("extrinsicTrans").as_double_array();
        if (rpy.size() != 3 || trans.size() != 3)
            throw std::runtime_error("extrinsicRPY and extrinsicTrans require three values");
        tf2::Matrix3x3 rotation(quaternion_from_rpy(rpy[0], rpy[1], rpy[2]));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                extRot(i,j) = rotation[i][j];
        extTrans = Eigen::Vector3d(trans[0], trans[1], trans[2]);
        extQRPY = Eigen::Quaterniond(extRot);
    }
    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }
};


