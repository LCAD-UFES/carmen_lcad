// Convert ROS2 GPS position and heading messages to GPS odometry of base_link.
// Correct antenna mounting, align sample timestamps and combine positions by covariance.
// Publish the global UTM pose on /odometry/gps.

#include "carmen_ros_bridge/platform.hpp"
#include "carmen_ros_bridge/ros_to_ros_gps_msg_to_gps_odometry.hpp"
#include "carmen_ros_bridge/runtime.hpp"
#if __has_include(<tf2/utils.hpp>)
#include <tf2/utils.hpp>
#else
#include <tf2/utils.h>
#endif
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>

using carmen_bridge::GpsMsgToGpsOdometryNode;

// Utilities

// Select a recent heading; CARMEN builds require HDT.
bool
GpsMsgToGpsOdometryNode::current_heading(const rclcpp::Time &now, double &theta_out) const
{
    bool found = false;

    if (heading_from_hdt_ && (now - last_heading_stamp_).seconds() < heading_max_age_)
    {
        theta_out = last_heading_;
        found = true;
    }
#ifndef MAKE_IN_CARMEn
    else
    {
        rclcpp::Time newest(0, 0, RCL_ROS_TIME);
        for (const auto &a : antennas_)
        {
            if (!a.has_sample) continue;
            if ((now - a.stamp).seconds() >= gps_max_age_) continue;
            if (!found || a.stamp > newest)
            {
                newest = a.stamp;
                theta_out = a.theta;
                found = true;
            }
        }
    }
#endif

    if (!found)
        return false;

    theta_out += heading_offset_;
    while (theta_out >   M_PI) theta_out -= 2.0 * M_PI;
    while (theta_out <= -M_PI) theta_out += 2.0 * M_PI;
    return true;
}

// Estimate the heading offset on straight segments; only report it in the log.
void
GpsMsgToGpsOdometryNode::update_heading_offset_estimate(const rclcpp::Time &stamp,
                                                         double raw_theta,
                                                         double x, double y)
{
    if (!estimate_heading_offset_)
        return;
    if (!hoff_have_prev_)
    {
        hoff_have_prev_  = true;
        hoff_prev_x_     = x;
        hoff_prev_y_     = y;
        hoff_prev_theta_ = raw_theta;
        hoff_prev_stamp_ = stamp;
        return;
    }

    const double dx = x - hoff_prev_x_;
    const double dy = y - hoff_prev_y_;
    const double d  = std::hypot(dx, dy);

    if (d < hoff_min_baseline_)
        return;
    double dtheta = raw_theta - hoff_prev_theta_;
    while (dtheta >   M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta <= -M_PI) dtheta += 2.0 * M_PI;

    if (std::fabs(dtheta) < 10.0 * M_PI / 180.0)
    {
        const double cog = std::atan2(dy, dx);
        double diff = 0.5 * (raw_theta + hoff_prev_theta_) - cog;
        while (diff >   M_PI) diff -= 2.0 * M_PI;
        while (diff <= -M_PI) diff += 2.0 * M_PI;

        const double w = d;
        hoff_sum_sin_ += w * std::sin(diff);
        hoff_sum_cos_ += w * std::cos(diff);
        hoff_wsum_    += w;
        ++hoff_samples_;

        if ((hoff_samples_ % 10) == 0)
        {
            const double est = std::atan2(hoff_sum_sin_, hoff_sum_cos_);
            const double R   = std::hypot(hoff_sum_sin_, hoff_sum_cos_) / hoff_wsum_;
            const double sigma_deg = (R > 0.0 && R < 1.0)
                                   ? std::sqrt(-2.0 * std::log(R)) * 180.0 / M_PI
                                   : 0.0;
            RCLCPP_INFO(bridge_node->get_logger(), "\033[1;33m[calib rumo] %zu trechos: rumo reportado esta' %.2f deg "
                     "adiante da direcao de percurso (sigma=%.2f deg). "
                     "Para corrigir use ~heading_offset_deg:=%.2f\033[0m",
                     hoff_samples_, est * 180.0 / M_PI, sigma_deg,
                     -est * 180.0 / M_PI);
        }
    }

    hoff_prev_x_     = x;
    hoff_prev_y_     = y;
    hoff_prev_theta_ = raw_theta;
    hoff_prev_stamp_ = stamp;
}

// Publishers

// Publish the fixed vehicle-to-antenna transforms.
void
GpsMsgToGpsOdometryNode::publish_antenna_tfs(const rclcpp::Time &stamp)
{
    if (!static_tf_broadcaster_)
        static_tf_broadcaster_ = carmen_bridge::static_broadcaster();

    const rclcpp::Time tf_time = carmen_bridge::tf_stamp(stamp);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto &a : antennas_)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = tf_time;
        t.header.frame_id = car_frame_id_;
        t.child_frame_id  = "gps_" + std::to_string(a.nr);

        const tf2::Vector3 &origin = a.T_car_gps.getOrigin();
        t.transform.translation.x = origin.x();
        t.transform.translation.y = origin.y();
        t.transform.translation.z = origin.z();
        t.transform.rotation = tf2::toMsg(a.T_car_gps.getRotation());

        transforms.push_back(t);
        RCLCPP_INFO(bridge_node->get_logger(), "TF estatica %s->%s publicada (braco de alavanca da antena)",
                 car_frame_id_.c_str(), t.child_frame_id.c_str());
    }

    if (!transforms.empty())
        static_tf_broadcaster_->sendTransform(transforms);
}

// Correct and combine antenna positions at the triggering GPS timestamp.
void
GpsMsgToGpsOdometryNode::try_publish_fused(const rclcpp::Time &trigger_stamp)
{
    double theta;
    if (!current_heading(trigger_stamp, theta))
        return;
    double v = 0.0;
    if (use_latency_compensation_ && (trigger_stamp - last_v_stamp_).seconds() < v_max_age_)
        v = last_v_;

    tf2::Quaternion q_heading;
    q_heading.setRPY(0.0, 0.0, theta);

    double sum_wx = 0.0, sum_wx_x = 0.0;
    double sum_wy = 0.0, sum_wy_y = 0.0;
    double sum_wz = 0.0, sum_wz_z = 0.0;
    int n_used = 0;

    for (const auto &a : antennas_)
    {
        if (!a.has_sample) continue;
        double age = (trigger_stamp - a.stamp).seconds();
        if (age >= gps_max_age_) continue;

        // Convert the antenna position to the vehicle origin.
        tf2::Transform global_to_gps(q_heading, tf2::Vector3(a.x, a.y, a.z));
        tf2::Transform global_to_car = global_to_gps * a.T_gps_car;
        tf2::Vector3   car_pos = global_to_car.getOrigin();

        // Propagate older samples to the triggering GPS timestamp.
        double dt = use_latency_compensation_ ? std::max(0.0, std::min(age, 1.0)) : 0.0;
        double cx = car_pos.x() + v * dt * std::cos(theta);
        double cy = car_pos.y() + v * dt * std::sin(theta);
        double cz = car_pos.z();

        // Give higher weights to antenna measurements with lower variance.
        double wx = 1.0 / a.cov_xx;
        double wy = 1.0 / a.cov_yy;
        double wz = 1.0 / a.cov_zz;
        sum_wx += wx; sum_wx_x += wx * cx;
        sum_wy += wy; sum_wy_y += wy * cy;
        sum_wz += wz; sum_wz_z += wz * cz;
        ++n_used;
    }

    if (n_used == 0) return;

    double fx = sum_wx_x / sum_wx;
    double fy = sum_wy_y / sum_wy;
    double fz = sum_wz_z / sum_wz;
    double var_x = 1.0 / sum_wx;
    double var_y = 1.0 / sum_wy;
    double var_z = 1.0 / sum_wz;

    // SC-LIO-SAM handles the conversion from UTM to map coordinates.
    tf2::Transform T_utm_car(q_heading, tf2::Vector3(fx, fy, fz));

    nav_msgs::msg::Odometry out;
    out.header.stamp    = trigger_stamp;
    out.header.frame_id = gps_output_frame_id_;
    out.child_frame_id  = car_frame_id_;

    const tf2::Vector3 &pos = T_utm_car.getOrigin();
    out.pose.pose.position.x = pos.x();
    out.pose.pose.position.y = pos.y();
    out.pose.pose.position.z = pos.z();
    out.pose.pose.orientation = tf2::toMsg(T_utm_car.getRotation());

    for (auto &cov : out.pose.covariance) cov = 0.0;
    out.pose.covariance[0]  = var_x;
    out.pose.covariance[7]  = var_y;
    out.pose.covariance[14] = var_z;

    out.pose.covariance[35] = heading_from_hdt_ ? 0.01 : 0.10;

    gps_odom_pub_->publish(out);
}

// Handlers

void
GpsMsgToGpsOdometryNode::on_gps_xyz(int nr, nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    auto it = std::find_if(antennas_.begin(), antennas_.end(),
        [nr](const GpsAntenna &a)
        {
            return a.nr == nr;
        });
    if (it == antennas_.end()) return;

    if (!antenna_tfs_sent_.exchange(true))
        publish_antenna_tfs(msg->header.stamp);

    it->has_sample = true;
    it->stamp = msg->header.stamp;
    it->x = msg->pose.pose.position.x;
    it->y = msg->pose.pose.position.y;
    it->z = msg->pose.pose.position.z;
#ifndef MAKE_IN_CARMEn
    it->theta = tf2::getYaw(msg->pose.pose.orientation);
#endif
    it->cov_xx = std::max(msg->pose.covariance[0],  1e-6);
    it->cov_yy = std::max(msg->pose.covariance[7],  1e-6);
    it->cov_zz = std::max(msg->pose.covariance[14], 1e-6);

#ifdef MAKE_IN_CARMEn
    // CARMEN GPS XYZ has no heading; calibration uses a recent HDT sample.
    if (!gps_nrs_.empty() && nr == gps_nrs_.front() && heading_from_hdt_ &&
        std::fabs((rclcpp::Time(msg->header.stamp) - last_heading_stamp_).seconds()) < heading_max_age_)
        update_heading_offset_estimate(msg->header.stamp, last_heading_, it->x, it->y);
#else
    if (!gps_nrs_.empty() && nr == gps_nrs_.front())
        update_heading_offset_estimate(msg->header.stamp, it->theta, it->x, it->y);
#endif

    try_publish_fused(msg->header.stamp);
}

void
GpsMsgToGpsOdometryNode::on_heading(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg)
{
    last_heading_ = tf2::getYaw(msg->quaternion);
    last_heading_stamp_ = msg->header.stamp;
    heading_from_hdt_ = true;
}

void
GpsMsgToGpsOdometryNode::on_ackermann(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
    last_v_ = msg->twist.linear.x;
    last_v_stamp_ = msg->header.stamp;
}

// Initializations

// Read antenna mounting once through IPC, then process ROS2 messages.
GpsMsgToGpsOdometryNode::GpsMsgToGpsOdometryNode(const rclcpp::Node::SharedPtr &nh)
    : nh_(nh)
{
    declare_parameters();

    {
        std::string prog = "ros_to_ros_gps_msg_to_gps_odometry";
        std::string flag = "-central_host";
        std::vector<char*> argv_vec = {
            const_cast<char*>(prog.c_str()),
            const_cast<char*>(flag.c_str()),
            const_cast<char*>(ipc_host_.c_str())
        };
        carmen_ipc_initialize(static_cast<int>(argv_vec.size()), argv_vec.data());
    }

    load_antenna_offsets_from_carmen();

    IPC_disconnect();

    gps_odom_pub_ = bridge_node->create_publisher<nav_msgs::msg::Odometry>(lio_sam_gps_topic_, 50);

    setup_subscribers();

    RCLCPP_INFO(bridge_node->get_logger(),
        "ros_to_ros_gps_msg_to_gps_odometry iniciado: %zu antena(s), publicando em %s (frame=%s, child_frame=%s), "
        "sem ancoragem -- quem converte pro frame do mapa agora e' o SC-LIO-SAM "
        "(gps_utm_anchor.txt, salvo/lido junto com o mapa).",
        antennas_.size(), lio_sam_gps_topic_.c_str(),
        gps_output_frame_id_.c_str(), car_frame_id_.c_str());
}

void
GpsMsgToGpsOdometryNode::declare_parameters()
{
    std::vector<int> default_nrs = {1};
    carmen_bridge::parameter("gps_nrs", gps_nrs_, default_nrs);

    carmen_bridge::parameter("lio_sam_gps_topic",   lio_sam_gps_topic_,   std::string("/odometry/gps"));
    carmen_bridge::parameter("gps_output_frame_id", gps_output_frame_id_, std::string("utm"));
    carmen_bridge::parameter("car_frame_id",        car_frame_id_,        std::string("base_link"));
    carmen_bridge::parameter("ipc_host",            ipc_host_,            std::string("localhost"));

    carmen_bridge::parameter("gps_max_age",     gps_max_age_,     0.3);
    carmen_bridge::parameter("heading_max_age", heading_max_age_, 1.0);
    carmen_bridge::parameter("v_max_age",       v_max_age_,       1.0);
    carmen_bridge::parameter("use_latency_compensation", use_latency_compensation_, true);

    // Apply a fixed heading correction; optional estimation only reports a suggested value.
    double heading_offset_deg = 0.0;
    carmen_bridge::parameter("heading_offset_deg", heading_offset_deg, 0.0);
    heading_offset_ = heading_offset_deg * M_PI / 180.0;

    carmen_bridge::parameter("estimate_heading_offset", estimate_heading_offset_, false);
    carmen_bridge::parameter("heading_offset_min_baseline", hoff_min_baseline_, 5.0);

    if (heading_offset_deg != 0.0)
        RCLCPP_INFO(bridge_node->get_logger(), "\033[1;32mheading_offset_deg = %.2f (%.4f rad) -- somado ao rumo cru "
                 "antes de rotacionar o braco de alavanca.\033[0m",
                 heading_offset_deg, heading_offset_);
    else
        RCLCPP_INFO(bridge_node->get_logger(), "heading_offset_deg = 0 -- rumo do GPS usado cru. Em veiculo de DUAS "
                 "antenas confira com ~estimate_heading_offset:=true.");

    if (gps_nrs_.empty())
    {
        gps_nrs_ = {1};
    }
}

// Antenna offsets are relative to sensor_board_1, which is relative to the vehicle.
void
GpsMsgToGpsOdometryNode::load_antenna_offsets_from_carmen()
{
    carmen_param_allow_unfound_variables(1);

    double sb_x = 0.0, sb_y = 0.0, sb_z = 0.0;
    double sb_roll = 0.0, sb_pitch = 0.0, sb_yaw = 0.0;
    {
        carmen_param_t sb_params[] = {
            {(char *) "sensor_board_1", (char *) "x",     CARMEN_PARAM_DOUBLE, &sb_x,     0, NULL},
            {(char *) "sensor_board_1", (char *) "y",     CARMEN_PARAM_DOUBLE, &sb_y,     0, NULL},
            {(char *) "sensor_board_1", (char *) "z",     CARMEN_PARAM_DOUBLE, &sb_z,     0, NULL},
            {(char *) "sensor_board_1", (char *) "roll",  CARMEN_PARAM_DOUBLE, &sb_roll,  0, NULL},
            {(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &sb_pitch, 0, NULL},
            {(char *) "sensor_board_1", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &sb_yaw,   0, NULL},
        };
        carmen_param_install_params(0, nullptr, sb_params,
                                   sizeof(sb_params) / sizeof(sb_params[0]));
    }
    tf2::Quaternion q_sb;
    q_sb.setRPY(sb_roll, sb_pitch, sb_yaw);
    const tf2::Transform T_car_board(q_sb, tf2::Vector3(sb_x, sb_y, sb_z));
    RCLCPP_INFO(bridge_node->get_logger(), "sensor_board_1 = (%.3f, %.3f, %.3f) rpy=(%.4f, %.4f, %.4f) -- "
             "composta no braco de alavanca de cada antena.",
             sb_x, sb_y, sb_z, sb_roll, sb_pitch, sb_yaw);

    for (int nr : gps_nrs_)
    {
        double gx = 0.0, gy = 0.0, gz = 0.0;
        double groll = 0.0, gpitch = 0.0, gyaw = 0.0;

        std::string ros_prefix = "gps" + std::to_string(nr) + "_offset_";
        carmen_bridge::parameter(ros_prefix + "x",     gx,     0.0);
        carmen_bridge::parameter(ros_prefix + "y",     gy,     0.0);
        carmen_bridge::parameter(ros_prefix + "z",     gz,     0.0);
        carmen_bridge::parameter(ros_prefix + "roll",  groll,  0.0);
        carmen_bridge::parameter(ros_prefix + "pitch", gpitch, 0.0);
        carmen_bridge::parameter(ros_prefix + "yaw",   gyaw,   0.0);

        std::string carmen_prefix = "nmea_" + std::to_string(nr) + "_";
        std::string key_x = carmen_prefix + "x";
        std::string key_y = carmen_prefix + "y";
        std::string key_z = carmen_prefix + "z";
        std::string key_roll  = carmen_prefix + "roll";
        std::string key_pitch = carmen_prefix + "pitch";
        std::string key_yaw   = carmen_prefix + "yaw";

        carmen_param_t gps_params[] = {
            {(char *) "gps", const_cast<char *>(key_x.c_str()),     CARMEN_PARAM_DOUBLE, &gx,     0, NULL},
            {(char *) "gps", const_cast<char *>(key_y.c_str()),     CARMEN_PARAM_DOUBLE, &gy,     0, NULL},
            {(char *) "gps", const_cast<char *>(key_z.c_str()),     CARMEN_PARAM_DOUBLE, &gz,     0, NULL},
            {(char *) "gps", const_cast<char *>(key_roll.c_str()),  CARMEN_PARAM_DOUBLE, &groll,  0, NULL},
            {(char *) "gps", const_cast<char *>(key_pitch.c_str()), CARMEN_PARAM_DOUBLE, &gpitch, 0, NULL},
            {(char *) "gps", const_cast<char *>(key_yaw.c_str()),   CARMEN_PARAM_DOUBLE, &gyaw,   0, NULL},
        };
        carmen_param_install_params(0, nullptr, gps_params,
                                   sizeof(gps_params) / sizeof(gps_params[0]));

        GpsAntenna ant;
        ant.nr = nr;

        tf2::Quaternion q_bg;
        q_bg.setRPY(groll, gpitch, gyaw);
        const tf2::Transform T_board_gps(q_bg, tf2::Vector3(gx, gy, gz));
        ant.T_car_gps = T_car_board * T_board_gps;
        ant.T_gps_car = ant.T_car_gps.inverse();

        double dist = ant.T_car_gps.getOrigin().length();
        RCLCPP_INFO(bridge_node->get_logger(),
            "GPS nr=%d: board->antena = (%.3f, %.3f, %.3f) | car->antena = (%.3f, %.3f, %.3f) m, distancia=%.3f m",
            nr, gx, gy, gz,
            ant.T_car_gps.getOrigin().x(), ant.T_car_gps.getOrigin().y(),
            ant.T_car_gps.getOrigin().z(), dist);

        antennas_.push_back(std::move(ant));
    }
}

void
GpsMsgToGpsOdometryNode::setup_subscribers()
{
    for (size_t i = 0; i < antennas_.size(); ++i)
    {
        int nr = antennas_[i].nr;
        std::string topic = "/gps/xyz_raw/gps_" + std::to_string(nr);
        antennas_[i].sub = bridge_node->create_subscription<nav_msgs::msg::Odometry>(
            topic, 20,
            [this, nr](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                on_gps_xyz(nr, msg);
            });
        RCLCPP_INFO(bridge_node->get_logger(), "Assinando %s (antena nr=%d)", topic.c_str(), nr);
    }

    heading_sub_ = bridge_node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/gps/heading_raw", 20,
        [this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg)
        {
            on_heading(msg);
        });
    ackermann_sub_ = bridge_node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/ackermann/odom_raw", 20,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
        {
            on_ackermann(msg);
        });
}

// Main

int
main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    bridge_node = std::make_shared<rclcpp::Node>("ros_to_ros_gps_msg_to_gps_odometry");
    GpsMsgToGpsOdometryNode node(bridge_node);
    rclcpp::spin(bridge_node);
    return 0;
}
