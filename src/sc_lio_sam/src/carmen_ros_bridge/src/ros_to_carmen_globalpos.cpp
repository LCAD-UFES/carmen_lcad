// Convert ROS SLAM odometry and Ackermann data to CARMEN fused_odometry.
// localize_ackerman publishes globalpos from this data with mapping_mode on.
// Relay CARMEN positioning requests to ROS /initialpose.

#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif
#include "carmen_ros_bridge/platform.hpp"

#include "carmen_ros_bridge/runtime.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif
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
#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>

#include "carmen_ros_bridge/runtime.hpp"

// Route the C-style IPC callback to the active node instance.
class RosToCarmenGlobalpos;
static RosToCarmenGlobalpos *g_fused_odom_node = nullptr;
static void
carmenInitializeMessageHandler(carmen_localize_ackerman_initialize_message *msg);

class RosToCarmenGlobalpos
{
public:
    RosToCarmenGlobalpos(const rclcpp::Node::SharedPtr &nh)
        : nh_(nh), tf_buffer_(nh->get_clock())
    {
        carmen_bridge::parameter("ipc_host", ipc_host_, std::string("localhost"));
        carmen_bridge::parameter("base_frame_id",  base_frame_id_,  std::string("base_link"));
        carmen_bridge::parameter("lidar_frame_id", lidar_frame_id_, std::string("velodyne"));

        // The default /odometry/imu input describes the LiDAR pose.
        carmen_bridge::parameter("slam_odom_topic", slam_odom_topic_, std::string("/odometry/imu"));
        // Limit IPC publication to 50 Hz by default; zero disables the limit.
        carmen_bridge::parameter("max_publish_rate", max_publish_rate_, 50.0);
        carmen_bridge::parameter("ackermann_topic", ackermann_topic_, std::string("/ackermann/odom_raw"));
        carmen_bridge::parameter("world_origin_file", world_origin_file_, std::string(""));
        carmen_bridge::parameter("origin_x",   origin_x_,   0.0);
        carmen_bridge::parameter("origin_y",   origin_y_,   0.0);
        carmen_bridge::parameter("origin_z",   origin_z_,   0.0);
        carmen_bridge::parameter("origin_yaw", origin_yaw_, 0.0);
        // Accept Ackermann samples within 0.2 seconds of the SLAM timestamp.
        carmen_bridge::parameter("ackermann_max_dt", ackermann_max_dt_, 0.2);

        carmen_bridge::parameter("initialpose_topic", initialpose_topic_, std::string("/initialpose"));

        // Load the origin once; a valid file overrides origin_* parameters.
        if (!world_origin_file_.empty())
            loadWorldOrigin();

        // Apply origin translation and yaw to transform map poses into world coordinates.
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, origin_yaw_);
        T_world_map_ = tf2::Transform(q, tf2::Vector3(origin_x_, origin_y_, origin_z_));

        RCLCPP_INFO(bridge_node->get_logger(), "ros_to_carmen_globalpos: origem do mundo x=%.3f y=%.3f z=%.3f yaw=%.6f",
                 origin_x_, origin_y_, origin_z_, origin_yaw_);

        if (std::fabs(origin_x_) < 1000000.0)
            RCLCPP_WARN(bridge_node->get_logger(), "\033[1;31mORIGEM x=%.3f tem |x| < 1.000.000 -- o localize_ackerman "
                     "VAI DESCARTAR o globalpos em silencio (main.cpp:3426). "
                     "Ajuste a origem antes de rodar.\033[0m", origin_x_);

        // Supply a module name when opening the IPC connection.
        std::string prog = "ros_to_carmen_globalpos";
        std::string flag = "-central_host";
        std::vector<char *> argv_vec = {
            const_cast<char *>(prog.c_str()),
            const_cast<char *>(flag.c_str()),
            const_cast<char *>(ipc_host_.c_str())
        };
        carmen_ipc_initialize(static_cast<int>(argv_vec.size()), argv_vec.data());

        IPC_RETURN_TYPE err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_NAME,
                                            IPC_VARIABLE_LENGTH,
                                            CARMEN_FUSED_ODOMETRY_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_NAME);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        sub_ack_ = bridge_node->create_subscription<geometry_msgs::msg::TwistStamped>(ackermann_topic_, 200, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { onAckermann(msg); });
        sub_slam_ = bridge_node->create_subscription<nav_msgs::msg::Odometry>(slam_odom_topic_, 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onSlamOdom(msg); });
        sub_initialpose_ = bridge_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic_, 1, [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) { onInitialPoseTrigger(msg); });
        pub_initialpose_ = bridge_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic_, 1);

        // Forward CARMEN positioning requests to the SLAM initial-pose topic.
        g_fused_odom_node = this;
        carmen_localize_ackerman_subscribe_initialize_message(
            nullptr, (carmen_handler_t) carmenInitializeMessageHandler, CARMEN_SUBSCRIBE_LATEST);

        // Poll IPC without blocking the single ROS callback thread.
        ipc_poll_timer_ = bridge_node->create_wall_timer(std::chrono::milliseconds(20),
                []() { IPC_listen(0); });

        RCLCPP_INFO(bridge_node->get_logger(), "ros_to_carmen_globalpos pronto: %s + %s -> %s (IPC)",
                 slam_odom_topic_.c_str(), ackermann_topic_.c_str(), CARMEN_FUSED_ODOMETRY_NAME);
        RCLCPP_INFO(bridge_node->get_logger(), "Lembre: localize_ackerman -mapping_mode on, e o modulo fused_odometry DESLIGADO.");
        RCLCPP_INFO(bridge_node->get_logger(), "Snap do(s) trailer_theta[]: publique em '%s' (ex.: '2D Pose Estimate' do "
                 "RViz) para realinhar o(s) reboque(s) com o yaw ATUAL do cavalo -- um snap "
                 "por clique, nao repete sozinho.", initialpose_topic_.c_str());
    }

private:
    // Read the first valid x y z roll pitch yaw row; only translation and yaw are used.
    void
    loadWorldOrigin()
    {
        std::ifstream f(world_origin_file_);
        if (!f.is_open())
        {
            RCLCPP_WARN(bridge_node->get_logger(), "world_origin_file '%s' nao abriu -- usando os parametros origin_*.",
                     world_origin_file_.c_str());
            return;
        }
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#') continue;
            if (line.compare(0, 6, "pesos ") == 0) continue;
            std::istringstream ss(line);
            double x, y, z, roll, pitch, yaw;
            if (!(ss >> x >> y >> z >> roll >> pitch >> yaw)) continue;
            origin_x_ = x; origin_y_ = y; origin_z_ = z; origin_yaw_ = yaw;
            RCLCPP_INFO(bridge_node->get_logger(), "Origem do mundo lida de '%s'.", world_origin_file_.c_str());
            return;
        }
        RCLCPP_WARN(bridge_node->get_logger(), "world_origin_file '%s' sem linha valida -- usando os parametros origin_*.",
                 world_origin_file_.c_str());
    }

    // Cache the base_link-to-LiDAR transform when it becomes available.
    bool
    ensureBaseLidar()
    {
        if (have_base_lidar_) return true;

        if (base_frame_id_ == lidar_frame_id_)
        {
            T_base_lidar_.setIdentity();
            have_base_lidar_ = true;
            return true;
        }

        try
        {
            geometry_msgs::msg::TransformStamped t =
                tf_buffer_.lookupTransform(base_frame_id_, lidar_frame_id_, rclcpp::Time(0, 0, RCL_ROS_TIME));
            tf2::fromMsg(t.transform, T_base_lidar_);
            have_base_lidar_ = true;

            double r, p, y;
            tf2::Matrix3x3(T_base_lidar_.getRotation()).getRPY(r, p, y);
            RCLCPP_INFO(bridge_node->get_logger(), "Extrinseca %s -> %s: t=(%.3f, %.3f, %.3f) rpy=(%.4f, %.4f, %.4f) "
                     "[yaw %.1f deg]",
                     base_frame_id_.c_str(), lidar_frame_id_.c_str(),
                     T_base_lidar_.getOrigin().x(), T_base_lidar_.getOrigin().y(),
                     T_base_lidar_.getOrigin().z(), r, p, y, y * 180.0 / M_PI);
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(bridge_node->get_logger(), log_clock, 2000,
                "TF %s -> %s indisponivel (%s) -- sem ela a pose sairia no frame do "
                "LIDAR, com o yaw errado. Nao publicando.",
                base_frame_id_.c_str(), lidar_frame_id_.c_str(), ex.what());
            return false;
        }
    }

    // CARMEN uses linear.x for speed and angular.z for steering angle, in m/s and radians.
    void
    onAckermann(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
    {

        last_v_        = msg->twist.linear.x;
        last_phi_      = msg->twist.angular.z;
        last_ack_time_ = rclcpp::Time(msg->header.stamp).seconds();
        have_ack_      = true;
    }

    // Ask localize_ackerman to initialize globalpos at the supplied world pose.
    void
    snapGlobalpos(double x, double y, double yaw, const char *host)
    {
        carmen_point_t mean = {x, y, yaw};
        carmen_point_t std  = {0.05, 0.05, 0.05};
        double trailer_theta[MAX_NUM_TRAILERS];
        for (int i = 0; i < num_trailers_; ++i)
            trailer_theta[i] = yaw;

#ifdef MAKE_IN_CARMEn
        // Preserve the source tag so the relayed command is not echoed back to ROS.
        carmen_localize_ackerman_initialize_message init{};
        IPC_RETURN_TYPE err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME,
            IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
        carmen_test_ipc_exit(err, "Could not define message",
            CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
        init.timestamp = carmen_get_time();
        init.host = const_cast<char *>(host);
        init.distribution = CARMEN_INITIALIZE_GAUSSIAN;
        init.num_modes = 1;
        init.mean = &mean;
        init.std = &std;
        init.num_trailers = num_trailers_;
        for (int i = 0; i < num_trailers_; ++i)
            init.trailer_theta[i] = trailer_theta[i];
        err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, &init);
        carmen_test_ipc(err, "Could not publish",
            CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
#else
        carmen_localize_ackerman_initialize_gaussian_command_host(
            mean, std, trailer_theta, num_trailers_, (char *) host);
#endif
    }

public:

    // Convert CARMEN world positioning requests back to the SLAM map frame.
    void
    onCarmenInitializeMessage(carmen_localize_ackerman_initialize_message *msg)
    {

        // Ignore initialization commands emitted by this module, including startup.
        const std::string host = msg->host ? msg->host : "";
        if (host.rfind("ros_to_carmen_globalpos", 0) == 0)
            return;

        if (!msg->mean)
        {
            RCLCPP_WARN(bridge_node->get_logger(), "Clique de posicionamento (host=%s) sem 'mean' -- ignorando.",
                     msg->host ? msg->host : "?");
            return;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, msg->mean->theta);
        tf2::Transform T_world_base(q, tf2::Vector3(msg->mean->x, msg->mean->y, 0.0));
        tf2::Transform T_map_base = T_world_map_.inverse() * T_world_base;

        geometry_msgs::msg::PoseWithCovarianceStamped out;
        out.header.stamp    = bridge_node->now();
        out.header.frame_id = last_slam_frame_id_.empty() ? "map" : last_slam_frame_id_;
        out.pose.pose.position.x    = T_map_base.getOrigin().x();
        out.pose.pose.position.y    = T_map_base.getOrigin().y();
        out.pose.pose.position.z    = T_map_base.getOrigin().z();
        out.pose.pose.orientation.x = T_map_base.getRotation().x();
        out.pose.pose.orientation.y = T_map_base.getRotation().y();
        out.pose.pose.orientation.z = T_map_base.getRotation().z();
        out.pose.pose.orientation.w = T_map_base.getRotation().w();

        pub_initialpose_->publish(out);

        RCLCPP_INFO(bridge_node->get_logger(), "Clique de posicionamento (host=%s) repassado pro SC-LIO-SAM em '%s' "
                 "(frame '%s'): mundo x=%.3f y=%.3f yaw=%.4f.",
                 msg->host ? msg->host : "?", initialpose_topic_.c_str(),
                 out.header.frame_id.c_str(), msg->mean->x, msg->mean->y, msg->mean->theta);
    }

private:

    // Request trailer alignment only when trailers and a current world pose exist.
    void
    onInitialPoseTrigger(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr )
    {
        if (!have_world_pose_)
        {
            RCLCPP_WARN(bridge_node->get_logger(), "snap de trailer pedido, mas ainda nao recebi nenhuma pose do SLAM -- ignorando.");
            return;
        }
        if (num_trailers_ <= 0)
        {
            RCLCPP_WARN(bridge_node->get_logger(), "snap de trailer pedido, mas num_trailers=0 (sem reboque configurado) -- ignorando.");
            return;
        }

        snapGlobalpos(last_world_x_, last_world_y_, last_world_yaw_, "ros_to_carmen_globalpos");

        RCLCPP_INFO(bridge_node->get_logger(), "snap: trailer_theta alinhado ao yaw do cavalo (%.4f rad) em x=%.3f y=%.3f",
                 last_world_yaw_, last_world_x_, last_world_y_);
    }

    void
    onSlamOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();

        // Keep the input timestamp when enforcing the publication limit.
        if (max_publish_rate_ > 0.0)
        {
            const double min_dt = 1.0 / max_publish_rate_;
            if (last_pub_stamp_ > 0.0 && (stamp - last_pub_stamp_) < min_dt)
                return;
            last_pub_stamp_ = stamp;
        }

        // Convert the input LiDAR pose to base_link before applying the world origin.
        if (!ensureBaseLidar())
            return;

        tf2::Transform T_map_lidar;
        tf2::fromMsg(msg->pose.pose, T_map_lidar);
        const tf2::Transform T_map_base   = T_map_lidar * T_base_lidar_.inverse();
        const tf2::Transform T_world_base = T_world_map_ * T_map_base;

        double roll, pitch, yaw;
        tf2::Matrix3x3(T_world_base.getRotation()).getRPY(roll, pitch, yaw);

        // Use measured speed and steering for CARMEN scan-time extrapolation.
        double v = 0.0, phi = 0.0;
        if (have_ack_ && std::fabs(stamp - last_ack_time_) <= ackermann_max_dt_)
        {
            v   = last_v_;
            phi = last_phi_;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(bridge_node->get_logger(), log_clock, 5000,
                "sem Ackermann proximo (dt=%.3f > %.3f) -- publicando v=0, phi=0. "
                "A extrapolacao ate' o scan fica sem efeito.",
                have_ack_ ? std::fabs(stamp - last_ack_time_) : -1.0, ackermann_max_dt_);
        }

        carmen_fused_odometry_message m;
        memset(&m, 0, sizeof(m));

        m.pose.position.x = T_world_base.getOrigin().x();
        m.pose.position.y = T_world_base.getOrigin().y();
        m.pose.position.z = T_world_base.getOrigin().z();
        m.pose.orientation.roll  = roll;
        m.pose.orientation.pitch = pitch;
        m.pose.orientation.yaw   = yaw;

        m.xsens_yaw_bias = 0.0;

        m.velocity.x = v;
        m.velocity.y = 0.0;
        m.velocity.z = 0.0;

        m.angular_velocity.roll  = msg->twist.twist.angular.x;
        m.angular_velocity.pitch = msg->twist.twist.angular.y;
        m.angular_velocity.yaw   = msg->twist.twist.angular.z;

        m.phi = phi;

        m.num_trailers = num_trailers_;
        for (int i = 0; i < MAX_NUM_TRAILERS; ++i)
            m.trailer_theta[i] = (i < num_trailers_) ? yaw : 0.0;

        last_world_x_ = m.pose.position.x;
        last_world_y_ = m.pose.position.y;
        last_world_yaw_ = yaw;
        last_slam_frame_id_ = msg->header.frame_id;
        have_world_pose_ = true;

        // Initialize globalpos on the first SLAM pose, before the next LiDAR scan.
        if (!have_published_initial_globalpos_)
        {
            have_published_initial_globalpos_ = true;
            snapGlobalpos(last_world_x_, last_world_y_, last_world_yaw_, "ros_to_carmen_globalpos_startup");
            RCLCPP_INFO(bridge_node->get_logger(), "globalpos inicial publicado (snap) em x=%.3f y=%.3f yaw=%.4f -- "
                     "antes do 1o scan do velodyne, pra' reduzir a janela do bug de "
                     "startup do navigator_gui2.", last_world_x_, last_world_y_, last_world_yaw_);
        }

        m.gps_position_at_turn_on.x = 0.0;
        m.gps_position_at_turn_on.y = 0.0;
        m.gps_position_at_turn_on.z = 0.0;

        // Preserve the acquisition timestamp shared with CARMEN.
        m.timestamp = stamp;
        m.host      = carmen_get_host();

        carmen_fused_odometry_publish_message(&m);

        if ((++published_ % 200) == 0)
            RCLCPP_INFO(bridge_node->get_logger(), "fused_odometry: %zu publicadas | x=%.3f y=%.3f yaw=%.4f v=%.2f phi=%.4f t=%.6f",
                     published_, m.pose.position.x, m.pose.position.y, yaw, v, phi, m.timestamp);
    }

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_slam_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_ack_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
    rclcpp::TimerBase::SharedPtr ipc_poll_timer_;
    std::string     last_slam_frame_id_;

    std::string slam_odom_topic_, ackermann_topic_, world_origin_file_;
    double origin_x_{0.0}, origin_y_{0.0}, origin_z_{0.0}, origin_yaw_{0.0};
    double ackermann_max_dt_{0.2};
    double max_publish_rate_{50.0};
    double last_pub_stamp_{0.0};
    std::string ipc_host_, base_frame_id_, lidar_frame_id_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform T_base_lidar_;
    bool have_base_lidar_{false};
    tf2::Transform T_world_map_;

    bool   have_ack_{false};
    double last_v_{0.0}, last_phi_{0.0}, last_ack_time_{0.0};
    size_t published_{0};
    // No trailer count is loaded; this value currently remains zero.
    int    num_trailers_{0};
    std::string initialpose_topic_;
    double last_world_x_{0.0}, last_world_y_{0.0}, last_world_yaw_{0.0};
    bool   have_world_pose_{false};
    bool   have_published_initial_globalpos_{false};
};

// Adapt the IPC free-function callback to the node method.
static void
carmenInitializeMessageHandler(carmen_localize_ackerman_initialize_message *msg)
{
    if (g_fused_odom_node)
        g_fused_odom_node->onCarmenInitializeMessage(msg);
}

int
main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    bridge_node = std::make_shared<rclcpp::Node>("ros_to_carmen_globalpos");
    RosToCarmenGlobalpos node(bridge_node);
    rclcpp::spin(bridge_node);
    return 0;
}
