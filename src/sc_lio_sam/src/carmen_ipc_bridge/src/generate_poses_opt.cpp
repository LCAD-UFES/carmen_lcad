/*
 * generate_poses_opt.cpp  (ROS1 / Noetic)
 *
 * Grava poses do robô em poses_opt.dat (formato graphslam), disparado a cada
 * scan de LiDAR. A pose vem da TF base_link -> map_base_link; z, roll e pitch
 * são fixados em zero na gravação.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <string>
#include <mutex>

namespace {

double stamp_to_sec(const ros::Time &stamp)
{
    return stamp.toSec();
}

bool ensure_parent_directory(const std::string &file_path)
{
    const boost::filesystem::path parent = boost::filesystem::path(file_path).parent_path();
    if (parent.empty()) return true;
    boost::system::error_code ec;
    boost::filesystem::create_directories(parent, ec);
    if (ec) {
        ROS_ERROR("Nao foi possivel criar diretorio '%s': %s",
                  parent.string().c_str(), ec.message().c_str());
        return false;
    }
    return true;
}

// CARMEN usa timestamp no fim do scan; ROS header.stamp e o inicio.
// carmen_ts = ros_header_stamp + max(point.time)
double ros_to_carmen_lidar_timestamp(const sensor_msgs::PointCloud2 &cloud)
{
    bool has_time_field = false;
    for (const auto &field : cloud.fields) {
        if (field.name == "time") {
            has_time_field = true;
            break;
        }
    }
    if (!has_time_field || cloud.width == 0) {
        ROS_WARN_THROTTLE(5.0,
            "Nuvem sem campo 'time' ou vazia; usando header.stamp ROS como timestamp CARMEN.");
        return cloud.header.stamp.toSec();
    }

    float max_t = 0.0f;
    sensor_msgs::PointCloud2ConstIterator<float> it_time(cloud, "time");
    for (; it_time != it_time.end(); ++it_time) {
        if (*it_time > max_t) max_t = *it_time;
    }
    return cloud.header.stamp.toSec() + static_cast<double>(max_t);
}

}  // namespace

class GeneratePosesOptNode
{
public:
    GeneratePosesOptNode(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string &output_path)
        : nh_(nh), pnh_(pnh), output_path_(output_path), tf_listener_(tf_buffer_)
    {
        pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        pnh_.param<std::string>("map_base_link_frame_id", map_base_link_frame_id_, "map_base_link");
        pnh_.param<std::string>("lidar_topic", lidar_topic_, "/points_raw");
        pnh_.param<std::string>("odometry_topic", odometry_topic_, "/lio_sam/mapping/odometry");
        pnh_.param<std::string>("gps_topic", gps_topic_, "/odometry/gps");
        pnh_.param<double>("tf_timeout", tf_timeout_, 0.1);

        if (!ensure_parent_directory(output_path_)) {
            throw std::runtime_error("Falha ao criar diretorio de saida para poses_opt.dat");
        }

        ofs_.open(output_path_, std::ofstream::out | std::ofstream::trunc);
        if (!ofs_.good()) {
            throw std::runtime_error("Nao foi possivel abrir arquivo de saida: " + output_path_);
        }

        lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &GeneratePosesOptNode::on_lidar, this);
        odom_sub_  = nh_.subscribe(odometry_topic_, 50, &GeneratePosesOptNode::on_odometry, this);
        gps_sub_   = nh_.subscribe(gps_topic_, 50, &GeneratePosesOptNode::on_gps, this);

        ROS_INFO("generate_poses_opt iniciado: gravando em '%s'", output_path_.c_str());
        ROS_INFO("  TF: %s -> %s", base_frame_id_.c_str(), map_base_link_frame_id_.c_str());
        ROS_INFO("  lidar=%s  odom=%s  gps=%s",
                 lidar_topic_.c_str(), odometry_topic_.c_str(), gps_topic_.c_str());
    }

    ~GeneratePosesOptNode()
    {
        if (ofs_.is_open()) {
            ofs_.flush();
            ofs_.close();
        }
        ROS_INFO("generate_poses_opt encerrado: %zu poses gravadas em '%s'",
                 pose_count_, output_path_.c_str());
    }

private:
    void on_odometry(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_odometry_timestamp_ = stamp_to_sec(msg->header.stamp);
        has_odometry_ = true;
    }

    void on_gps(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_gps_timestamp_ = stamp_to_sec(msg->header.stamp);
        has_gps_ = true;
    }

    void on_lidar(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        const ros::Time stamp = msg->header.stamp;
        geometry_msgs::TransformStamped tf_msg;

        try {
            tf_msg = tf_buffer_.lookupTransform(
                base_frame_id_, map_base_link_frame_id_, stamp,
                ros::Duration(tf_timeout_));
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0, "TF %s -> %s indisponivel: %s",
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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string output_path_;

    std::string base_frame_id_;
    std::string map_base_link_frame_id_;
    std::string lidar_topic_;
    std::string odometry_topic_;
    std::string gps_topic_;
    double tf_timeout_{0.1};

    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gps_sub_;

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

    ros::init(argc, argv, "generate_poses_opt");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        GeneratePosesOptNode node(nh, pnh, output_path);
        ros::spin();
    } catch (const std::exception &ex) {
        ROS_FATAL("generate_poses_opt: %s", ex.what());
        return 1;
    }

    return 0;
}
