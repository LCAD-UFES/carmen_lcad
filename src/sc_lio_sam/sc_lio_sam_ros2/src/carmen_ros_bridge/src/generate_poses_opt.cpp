#include <tf2_ros/buffer.hpp>
/*
 * generate_poses_opt.cpp  (ROS1 / Noetic)
 *
 * Grava poses do robô em poses_opt.dat (formato graphslam), disparado a cada
 * scan de LiDAR. A pose vem da TF base_link -> map_base_link; z, roll e pitch
 * são fixados em zero na gravação.
 */

#include "carmen_ros_bridge/runtime.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/utils.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <boost/filesystem.hpp>

#include <fstream>
#include <string>
#include <mutex>

namespace {

double stamp_to_sec(const rclcpp::Time &stamp)
{
    return stamp.seconds();
}

bool ensure_parent_directory(const std::string &file_path)
{
    const boost::filesystem::path parent = boost::filesystem::path(file_path).parent_path();
    if (parent.empty()) return true;
    boost::system::error_code ec;
    boost::filesystem::create_directories(parent, ec);
    if (ec) {
        RCLCPP_ERROR(bridge_node->get_logger(), "Nao foi possivel criar diretorio '%s': %s",
                  parent.string().c_str(), ec.message().c_str());
        return false;
    }
    return true;
}

// CARMEN usa timestamp no fim do scan; ROS header.stamp e o inicio.
// carmen_ts = ros_header_stamp + max(point.time)
double ros_to_carmen_lidar_timestamp(const sensor_msgs::msg::PointCloud2 &cloud)
{
    bool has_time_field = false;
    for (const auto &field : cloud.fields) {
        if (field.name == "time") {
            has_time_field = true;
            break;
        }
    }
    if (!has_time_field || cloud.width == 0) {
        RCLCPP_WARN_THROTTLE(bridge_node->get_logger(), log_clock, 5000,
            "Nuvem sem campo 'time' ou vazia; usando header.stamp ROS como timestamp CARMEN.");
        return rclcpp::Time(cloud.header.stamp).seconds();
    }

    float max_t = 0.0f;
    sensor_msgs::PointCloud2ConstIterator<float> it_time(cloud, "time");
    for (; it_time != it_time.end(); ++it_time) {
        if (*it_time > max_t) max_t = *it_time;
    }
    return rclcpp::Time(cloud.header.stamp).seconds() + static_cast<double>(max_t);
}

}  // namespace

class GeneratePosesOptNode
{
public:
    GeneratePosesOptNode(const rclcpp::Node::SharedPtr &nh, const std::string &output_path)
        : nh_(nh), output_path_(output_path), tf_buffer_(nh->get_clock()), tf_listener_(tf_buffer_)
    {
        carmen_bridge::parameter("base_frame_id", base_frame_id_, std::string("base_link"));
        carmen_bridge::parameter("map_base_link_frame_id", map_base_link_frame_id_, std::string("map_base_link"));
        carmen_bridge::parameter("lidar_topic", lidar_topic_, std::string("/points_raw"));
        carmen_bridge::parameter("odometry_topic", odometry_topic_, std::string("/lio_sam/mapping/odometry"));
        carmen_bridge::parameter("gps_topic", gps_topic_, std::string("/odometry/gps"));
        carmen_bridge::parameter("tf_timeout", tf_timeout_, 0.1);

        if (!ensure_parent_directory(output_path_)) {
            throw std::runtime_error("Falha ao criar diretorio de saida para poses_opt.dat");
        }

        ofs_.open(output_path_, std::ofstream::out | std::ofstream::trunc);
        if (!ofs_.good()) {
            throw std::runtime_error("Nao foi possivel abrir arquivo de saida: " + output_path_);
        }

        lidar_sub_ = bridge_node->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_, 10, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_lidar(msg); });
        odom_sub_ = bridge_node->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odometry(msg); });
        gps_sub_ = bridge_node->create_subscription<nav_msgs::msg::Odometry>(gps_topic_, 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_gps(msg); });

        RCLCPP_INFO(bridge_node->get_logger(), "generate_poses_opt iniciado: gravando em '%s'", output_path_.c_str());
        RCLCPP_INFO(bridge_node->get_logger(), "  TF: %s -> %s", base_frame_id_.c_str(), map_base_link_frame_id_.c_str());
        RCLCPP_INFO(bridge_node->get_logger(), "  lidar=%s  odom=%s  gps=%s",
                 lidar_topic_.c_str(), odometry_topic_.c_str(), gps_topic_.c_str());
    }

    ~GeneratePosesOptNode()
    {
        if (ofs_.is_open()) {
            ofs_.flush();
            ofs_.close();
        }
        RCLCPP_INFO(bridge_node->get_logger(), "generate_poses_opt encerrado: %zu poses gravadas em '%s'",
                 pose_count_, output_path_.c_str());
    }

private:
    void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_odometry_timestamp_ = stamp_to_sec(msg->header.stamp);
        has_odometry_ = true;
    }

    void on_gps(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_gps_timestamp_ = stamp_to_sec(msg->header.stamp);
        has_gps_ = true;
    }

    void on_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        const rclcpp::Time stamp = msg->header.stamp;
        geometry_msgs::msg::TransformStamped tf_msg;

        try {
            tf_msg = tf_buffer_.lookupTransform(
                base_frame_id_, map_base_link_frame_id_, stamp,
                rclcpp::Duration::from_seconds(tf_timeout_));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(bridge_node->get_logger(), log_clock, 2000, "TF %s -> %s indisponivel: %s",
                              base_frame_id_.c_str(), map_base_link_frame_id_.c_str(), ex.what());
            return;
        }

        const tf2::Transform tf(
            tf2::Quaternion(
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w),
            tf2::Vector3(
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z));

        const double x   = tf.getOrigin().x();
        const double y   = tf.getOrigin().y();
        const double yaw = tf2::getYaw(tf.getRotation());

        const double carmen_lidar_ts = ros_to_carmen_lidar_timestamp(*msg);
        double pose_ts  = carmen_lidar_ts;
        double lidar_ts = carmen_lidar_ts;
        double odom_ts = -1.0;
        double gps_ts = -1.0;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (has_odometry_) odom_ts = last_odometry_timestamp_;
            if (has_gps_)      gps_ts  = last_gps_timestamp_;
        }

        ofs_ << std::fixed
             << x << " " << y << " " << 0.0 << " "
             << 0.0 << " " << 0.0 << " " << yaw << " "
             << pose_ts << " " << lidar_ts << " " << odom_ts << " " << gps_ts << "\n";
        ofs_.flush();
        ++pose_count_;
    }

    rclcpp::Node::SharedPtr nh_;
    std::string output_path_;

    std::string base_frame_id_;
    std::string map_base_link_frame_id_;
    std::string lidar_topic_;
    std::string odometry_topic_;
    std::string gps_topic_;
    double tf_timeout_{0.1};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::ofstream ofs_;
    std::mutex mutex_;
    double last_odometry_timestamp_{-1.0};
    double last_gps_timestamp_{-1.0};
    bool has_odometry_{false};
    bool has_gps_{false};
    size_t pose_count_{0};
};

static void print_usage(const char *prog)
{
    fprintf(stderr,
            "Uso: %s <caminho/poses_opt.dat>\n"
            "\n"
            "Grava poses do robo em poses_opt.dat (formato graphslam).\n"
            "A pose vem da TF base_link -> map_base_link; z, roll e pitch sao zero.\n",
            prog);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    const std::string output_path = argv[1];

    rclcpp::init(argc, argv);
    bridge_node = std::make_shared<rclcpp::Node>("generate_poses_opt");

    try {
        GeneratePosesOptNode node(bridge_node, output_path);
        rclcpp::spin(bridge_node);
    } catch (const std::exception &ex) {
        RCLCPP_FATAL(bridge_node->get_logger(), "generate_poses_opt: %s", ex.what());
        return 1;
    }

    return 0;
}
