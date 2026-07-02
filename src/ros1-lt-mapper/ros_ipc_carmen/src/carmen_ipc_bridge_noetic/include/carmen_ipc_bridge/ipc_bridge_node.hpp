#pragma once

/*
 * ipc_bridge_node.hpp  (ROS1 / Noetic)
 *
 * Nó ROS1 standalone que lê dados do CARMEN IPC e publica:
 *   /imu_raw          (sensor_msgs/Imu)
 *   /velodyne_raw_ipc (sensor_msgs/PointCloud2, campos brutos) — para pointcloud_node
 *   /scan             (sensor_msgs/LaserScan)
 *   TF: base_link → velodyne  (estático)
 *   TF: base_link → imu_link  (estático)
 *
 * Porte direto da versão ROS2: a única diferença estrutural é que aqui
 * o nó não herda de nada (sem rclcpp::Node) — guarda um ros::NodeHandle
 * público e um privado ("~"), e os publishers são ros::Publisher comuns
 * (sem QoS, só profundidade de fila).
 */

#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>

// ─── Thread-safe bounded queue ───────────────────────────────────────────────
template<typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t max_size) : max_size_(max_size) {}
    void push(T item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.size() >= max_size_) queue_.pop_front();
        queue_.push_back(std::move(item));
    }
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

// ─── Data frames ─────────────────────────────────────────────────────────────
namespace carmen_bridge {

// Frame bruto do velodyne — publicado em /velodyne_raw_ipc via PointCloud2
// com campos extras. O pointcloud_node faz a conversão 3D.
struct VelodyneFrame {
    double timestamp    = 0.0;
    int64_t ros_stamp_ns = 0;
    // Dados brutos de distância+intensidade por shot×anel
    // Empacotados como PointCloud2 com campos: shot, ring, distance, intensity
    int num_shots       = 0;
    int shot_size       = 0;
    double range_div    = 500.0;
    double max_range    = 70.0;
    // Pontos 3D já calculados (mantido para compatibilidade com LaserScan)
    std::vector<float> ranges;
    std::vector<float> angles;
    // Dados brutos para o pointcloud_node
    struct RawPoint { uint16_t distance; uint8_t intensity; uint8_t ring; };
    std::vector<RawPoint> raw_points;  // num_shots × shot_size
    std::vector<int>      shot_sizes;  // tamanho de cada shot (variable_scan)
    // cos/sin do azimute REAL do sensor (msg->partial_scan[i].angle),
    // convertido para radianos anti-horários (ROS REP-103).
    // Preenchido no handler, consumido em velodyneframe_to_raw_cloud.
    std::vector<float>    shot_cos_az;
    std::vector<float>    shot_sin_az;
};

struct ImuFrame {
    double timestamp    = 0.0;
    int64_t ros_stamp_ns = 0;
    double ax = 0.0, ay = 0.0, az = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    bool has_orientation = false;
};

// ─── Nó IPC bridge (só IPC + IMU + LaserScan) ────────────────────────────────
class IpcBridgeNode {
public:
    IpcBridgeNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~IpcBridgeNode();

    void on_velodyne_scan(VelodyneFrame frame);
    void on_imu(const ImuFrame &frame);

    // Público para handlers IPC estáticos
    int    lidar_sensor_id_{5};
    double lidar_range_division_factor_{500.0};
    double lidar_max_range_{70.0};
    int    lidar_shot_size_{32};
    double lidar_x_{0.0}, lidar_y_{0.0}, lidar_z_{1.8};
    double lidar_roll_{0.0}, lidar_pitch_{0.0}, lidar_yaw_{0.0};
    std::atomic<bool> lidar_config_loaded_{false};

private:
    void declare_parameters();
    void setup_publishers();
    void setup_tf();
    void setup_tf_from_carmen();
    void load_carmen_lidar_config(int argc, char **argv);
    void start_ipc_thread();
    void thread_publish_scan();

    sensor_msgs::LaserScan   velodyne_to_laserscan(const VelodyneFrame &f);
    sensor_msgs::PointCloud2 velodyneframe_to_raw_cloud(const VelodyneFrame &f);
    sensor_msgs::Imu         imu_to_msg(const ImuFrame &f);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher scan_pub_;
    ros::Publisher raw_cloud_pub_;
    ros::Publisher imu_pub_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::thread       cloud_thread_;
    std::thread       ipc_thread_;
    std::atomic<bool> running_{true};

    BoundedQueue<VelodyneFrame> scan_queue_;

    std::string base_frame_id_;
    std::string laser_frame_id_;
    std::string imu_frame_id_;
    std::string ipc_host_;

    double max_laser_range_;
    double laser_min_angle_;
    double laser_max_angle_;
    int    laser_num_beams_;

    static constexpr int NUM_RINGS = 32;
    float ring_elevation_rad_[NUM_RINGS];

    bool publish_raw_cloud_;
    bool publish_laserscan_;
    bool publish_imu_;

    double imu_rotation_[9] = {1,0,0, 0,1,0, 0,0,1};
};

extern IpcBridgeNode *g_bridge_node;

} // namespace carmen_bridge
