#pragma once

// Convert ROS2 GPS positions and heading to vehicle odometry in UTM.
// Load antenna mounting offsets from CARMEN during initialization.

#include "carmen_ros_bridge/runtime.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#endif
#if __has_include(<tf2/LinearMath/Transform.hpp>)
#include <tf2/LinearMath/Transform.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#endif
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif

#include <vector>
#include <string>
#include <memory>
#include <atomic>

namespace carmen_bridge
{

// Fixed mounting and latest measurement for one antenna.
struct GpsAntenna
{
    int nr = -1;

    // Vehicle-to-antenna transforms include sensor_board_1.
    tf2::Transform T_car_gps;
    tf2::Transform T_gps_car;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;

    bool      has_sample = false;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double    x = 0.0, y = 0.0, z = 0.0;
    double    theta = 0.0;
    double    cov_xx = 1e6, cov_yy = 1e6, cov_zz = 1e6;
};

class GpsMsgToGpsOdometryNode
{
public:
    GpsMsgToGpsOdometryNode(const rclcpp::Node::SharedPtr &nh);

private:
    void declare_parameters();
    void load_antenna_offsets_from_carmen();
    void setup_subscribers();
    void publish_antenna_tfs(const rclcpp::Time &stamp);

    void on_gps_xyz(int nr, nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void on_heading(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg);
    void on_ackermann(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

    void try_publish_fused(const rclcpp::Time &trigger_stamp);
    bool current_heading(const rclcpp::Time &now, double &theta_out) const;

    void update_heading_offset_estimate(const rclcpp::Time &stamp,
                                        double raw_theta,
                                        double x, double y);

    rclcpp::Node::SharedPtr nh_;

    std::vector<GpsAntenna> antennas_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ackermann_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_odom_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // Latest speed and heading samples.
    double    last_v_ = 0.0;
    rclcpp::Time last_v_stamp_{0, 0, RCL_ROS_TIME};

    double    last_heading_ = 0.0;
    bool      heading_from_hdt_ = false;
    rclcpp::Time last_heading_stamp_{0, 0, RCL_ROS_TIME};

    std::vector<int> gps_nrs_;
    std::string      lio_sam_gps_topic_;
    std::string      gps_output_frame_id_;
    std::string      car_frame_id_;
    std::string      ipc_host_;
    double gps_max_age_;
    double heading_max_age_;
    double v_max_age_;
    bool   use_latency_compensation_;
    std::atomic<bool> antenna_tfs_sent_{false};

    // Applied heading correction, in radians.
    double heading_offset_;
    // Enable diagnostic estimation without changing the output.
    bool   estimate_heading_offset_;

    // Distance-weighted circular mean of the heading offset.
    double    hoff_sum_sin_ = 0.0, hoff_sum_cos_ = 0.0, hoff_wsum_ = 0.0;
    size_t    hoff_samples_ = 0;
    bool      hoff_have_prev_ = false;
    double    hoff_prev_x_ = 0.0, hoff_prev_y_ = 0.0, hoff_prev_theta_ = 0.0;
    rclcpp::Time hoff_prev_stamp_{0, 0, RCL_ROS_TIME};
    // Minimum segment displacement, in meters.
    double    hoff_min_baseline_;
};

}
