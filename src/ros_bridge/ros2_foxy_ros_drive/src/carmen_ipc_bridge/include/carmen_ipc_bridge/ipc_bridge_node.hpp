#pragma once

/*
 * ipc_bridge_node.hpp
 *
 * Nó ROS2 standalone que lê dados do CARMEN IPC e publica tópicos
 * padrão consumíveis pelo LIO-SAM ou qualquer outro pipeline.
 *
 * Tópicos publicados:
 *   /velodyne_points  (sensor_msgs/PointCloud2)  — LIO-SAM usa este
 *   /scan             (sensor_msgs/LaserScan)     — visualização / 2D SLAM
 *   /odom             (nav_msgs/Odometry)
 *   /imu/data         (sensor_msgs/Imu)
 *   TF: odom → base_link  (dinâmico)
 *   TF: base_link → velodyne  (estático)
 *
 * Sem assinaturas de slam_toolbox. Sem publicação de volta ao IPC.
 */

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#ifdef HAVE_TF2_GEOMETRY_MSGS
#  include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
namespace tf2 {
inline geometry_msgs::msg::Quaternion toMsg(const tf2::Quaternion &q) {
    geometry_msgs::msg::Quaternion m;
    m.x = q.x(); m.y = q.y(); m.z = q.z(); m.w = q.w();
    return m;
}
inline void fromMsg(const geometry_msgs::msg::Quaternion &m, tf2::Quaternion &q) {
    q.setValue(m.x, m.y, m.z, m.w);
}
} // namespace tf2
#endif

// ─── CARMEN headers ──────────────────────────────────────────────────────────
#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>

// ─── Thread-safe bounded queue ───────────────────────────────────────────────
#include <deque>
#include <condition_variable>

template<typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t max_size) : max_size_(max_size) {}

    // Recebe por valor e move (zero-copy)
    void push(T item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.size() >= max_size_) queue_.pop_front();
        queue_.push_back(std::move(item));
    }

    // Retorna O(1) o último item e descarta o lixo acumulado
    bool pop_latest(T &item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return false;
        item = std::move(queue_.back()); 
        queue_.clear(); 
        return true;
    }

    bool pop(T &item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

private:
    std::deque<T> queue_;
    std::mutex mtx_;
    size_t max_size_;
};

// ─── Data frames (IPC thread → ROS2 thread) ──────────────────────────────────
namespace carmen_bridge {

struct VelodyneFrame {
    double timestamp = 0.0;
    // Dados 2D (LaserScan)
    std::vector<float> ranges;
    std::vector<float> angles;
    // Dados 3D (PointCloud2) — x,y,z,intensity por ponto
    struct Point3D { float x, y, z, intensity; uint16_t ring; };
    std::vector<Point3D> points;
    // Ângulos de elevação por anel (para XT-32)
    std::vector<float> elevations;
};

struct OdometryFrame {
    double x = 0.0, y = 0.0, theta = 0.0;
    double v = 0.0, phi = 0.0;
    double abs_x = 0.0, abs_y = 0.0, abs_theta = 0.0;
    double timestamp = 0.0;
};

struct ImuFrame {
    double timestamp = 0.0;
    // Aceleração linear [m/s²]
    double ax = 0.0, ay = 0.0, az = 0.0;
    // Velocidade angular [rad/s]
    double gx = 0.0, gy = 0.0, gz = 0.0;
    // Orientação (quaternion) — se disponível
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    bool has_orientation = false;
};

// ─── Nó principal ─────────────────────────────────────────────────────────────
class IpcBridgeNode : public rclcpp::Node {
public:
    explicit IpcBridgeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~IpcBridgeNode();

    // Chamados pelos handlers IPC estáticos
    void on_velodyne_scan(VelodyneFrame frame);
    void on_odometry(const OdometryFrame &frame);
    void on_imu(const ImuFrame &frame);

    // ── Parâmetros lidos do CARMEN (lidar_config) — public para handlers estáticos
    int    lidar_sensor_id_{5};
    double lidar_range_division_factor_{500.0};
    double lidar_max_range_{120.0};
    int    lidar_shot_size_{32};
    double lidar_x_{0.0}, lidar_y_{0.0}, lidar_z_{1.8};
    double lidar_roll_{0.0}, lidar_pitch_{0.0}, lidar_yaw_{0.0};
    std::atomic<bool> lidar_config_loaded_{false};

private:
    // ── Inicialização ──────────────────────────────────────────────────────
    void declare_parameters();
    void setup_publishers();
    void setup_tf();
    void setup_tf_from_carmen();   // Re-emite o TF estático após carregar lidar_config
    void load_carmen_lidar_config(int argc, char **argv);
    void start_ipc_thread();

    // ── Timers (drain das filas) ───────────────────────────────────────────
    void timer_publish_scan();
    void timer_publish_odom();
    void timer_publish_imu();

    // ── Conversão ─────────────────────────────────────────────────────────
    sensor_msgs::msg::LaserScan   velodyne_to_laserscan(const VelodyneFrame &f);
    sensor_msgs::msg::PointCloud2 velodyne_to_pointcloud2(const VelodyneFrame &f);
    nav_msgs::msg::Odometry       odometry_to_msg(const OdometryFrame &f);
    sensor_msgs::msg::Imu         imu_to_msg(const ImuFrame &f);

    // ── TF ────────────────────────────────────────────────────────────────
    void publish_tf_odom_base(double x, double y, double theta, rclcpp::Time stamp);

    // ── Publishers ────────────────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr          imu_pub_;

    // ── TF broadcasters ───────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // ── Timers ────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr scan_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;

    // ── Filas thread-safe ─────────────────────────────────────────────────
    BoundedQueue<VelodyneFrame>  scan_queue_;
    BoundedQueue<OdometryFrame>  odom_queue_;
    BoundedQueue<ImuFrame>       imu_queue_;

    // ── Estado compartilhado ──────────────────────────────────────────────
    std::mutex        state_mutex_;
    OdometryFrame     latest_odom_;
    bool              odom_initialized_ = false;

    // ── Thread IPC ────────────────────────────────────────────────────────
    std::thread       ipc_thread_;
    std::atomic<bool> running_{true};

    // ── Parâmetros ────────────────────────────────────────────────────────
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string laser_frame_id_;
    std::string imu_frame_id_;

    double max_laser_range_;
    double laser_min_angle_;
    double laser_max_angle_;
    int    laser_num_beams_;

    // Ângulos de elevação do Hesai XT-32 (32 anéis, em graus convertidos)
    // Ordem do datasheet: anel 0 (mais baixo) → anel 31 (mais alto)
    static constexpr int NUM_RINGS = 32;
    float ring_elevation_rad_[NUM_RINGS];

    // Wheelbase para integração Ackermann
    double wheelbase_;

    bool publish_pointcloud_;
    bool publish_laserscan_;
    bool publish_imu_;

    // ── Parâmetros lidos do CARMEN (lidar_config) ─────────────────────────
    // (declarados em public para acesso pelos handlers IPC estáticos)
};

// Ponteiro global para callbacks IPC estáticos
extern IpcBridgeNode *g_bridge_node;

} // namespace carmen_bridge