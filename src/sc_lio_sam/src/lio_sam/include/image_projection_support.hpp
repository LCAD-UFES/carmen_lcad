#pragma once
#include "parameters.hpp"
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <atomic>
#include <cfloat>

using PointType = pcl::PointXYZI;

class RateCounter
{
public:
    void tick() { ++count; }
    double rate()
    {
        const auto now = std::chrono::steady_clock::now();
        const int n = count.exchange(0);
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;
        return dt > 1e-3 ? n / dt : 0.0;
    }
private:
    std::atomic<int> count{0};
    std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
};

inline sensor_msgs::msg::PointCloud2 publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr *publisher,
    pcl::PointCloud<PointType>::Ptr cloud, const builtin_interfaces::msg::Time &stamp,
    const std::string &frame)
{
    sensor_msgs::msg::PointCloud2 msg;
    // Preserve the ROS1 PCL point layout, including alignment padding.
    pcl::PCLPointCloud2 pcl_cloud;
    pcl::toPCLPointCloud2(*cloud, pcl_cloud);
    pcl_conversions::moveFromPCL(pcl_cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    if ((*publisher)->get_subscription_count() != 0)
        (*publisher)->publish(msg);
    return msg;
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *imu, T *x, T *y, T *z)
{
    *x = imu->angular_velocity.x;
    *y = imu->angular_velocity.y;
    *z = imu->angular_velocity.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *imu, T *roll, T *pitch, T *yaw)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(imu->orientation, orientation);
    double r, p, y;
    tf2::Matrix3x3(orientation).getRPY(r, p, y);
    *roll = r;
    *pitch = p;
    *yaw = y;
}

inline float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}
