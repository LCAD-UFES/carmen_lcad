#pragma once

/*
 * ipc_bridge_node.hpp  (ROS1 / Noetic)
 *
 * Nó ROS1 standalone que lê dados do CARMEN IPC e publica:
 *   /imu_raw            (sensor_msgs/Imu)
 *   /ackermann/odom_raw (geometry_msgs/TwistStamped)
 *   /gps/xyz_raw/gps_<nr> (nav_msgs/Odometry)  — uma por antena/GPS
 *   /gps/heading_raw    (geometry_msgs/QuaternionStamped)
 *   TF: base_link → imu_link  (estático)
 *
 * O caminho do LiDAR (leitura do velodyne, conversão 3D, /points_raw e o
 * filtro de pontos sobre o veículo) vive agora no pointcloud_node — este nó
 * não toca mais na nuvem de pontos nem no /scan.
 */

#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <carmen/carmen.h>

// ─── Data frames ─────────────────────────────────────────────────────────────
namespace carmen_bridge {

struct ImuFrame {
    double timestamp    = 0.0;
    int64_t ros_stamp_ns = 0;
    double ax = 0.0, ay = 0.0, az = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    bool has_orientation = false;
};

// Frame de odometria Ackermann — publicado em /ackermann/odom_raw via
// geometry_msgs/TwistStamped (linear.x = v [m/s], angular.z = phi [rad]).
// A integração do modelo de bicicleta (com a distância entre eixos) é feita
// no SC-LIO-SAM, então o bridge só encaminha os valores crus de v e phi.
struct AckermannFrame {
    double timestamp    = 0.0;
    int64_t ros_stamp_ns = 0;
    double v   = 0.0;   // velocidade linear (m/s)
    double phi = 0.0;   // ângulo de esterçamento (rad)
};

// Frame bruto de GPS (posição). Publicado em /gps/xyz_raw/gps_<nr> via
// nav_msgs/Odometry. "nr" identifica a antena/receptor (multi-GPS). Nenhuma
// fusão/correção acontece aqui — isso fica pro nó de correção separado.
struct GpsXyzFrame {
    double  timestamp    = 0.0;
    int64_t ros_stamp_ns = 0;
    int     nr           = -1;
    double  x = 0.0, y = 0.0, z = 0.0;
    double  theta        = 0.0;   // heading cru reportado pelo próprio GPS
    int     gps_quality  = 0;     // indicador de fix (estilo NMEA GGA)
    bool    valid        = false;
};

// Frame bruto de heading GPS (NMEA HDT, dupla antena). Publicado em
// /gps/heading_raw via geometry_msgs/QuaternionStamped.
struct GpsHdtFrame {
    double  timestamp    = 0.0;
    int64_t ros_stamp_ns = 0;
    double  heading      = 0.0;
    bool    valid        = false;
};

// ─── Nó IPC bridge (IMU + Ackermann + GPS + TF imu_link) ─────────────────────
class IpcBridgeNode {
public:
    IpcBridgeNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~IpcBridgeNode();

    void on_imu(const ImuFrame &frame);
    void on_ackermann(const AckermannFrame &frame);
    void on_gps_xyz(const GpsXyzFrame &frame);
    void on_gps_hdt(const GpsHdtFrame &frame);

private:
    void declare_parameters();
    void setup_publishers();
    void setup_tf();
    void publish_imu_static_tf(const ros::Time &sensor_stamp);
    void start_ipc_thread();

    sensor_msgs::Imu         imu_to_msg(const ImuFrame &f);
    nav_msgs::Odometry       gps_xyz_to_msg(const GpsXyzFrame &f);
    geometry_msgs::QuaternionStamped gps_hdt_to_msg(const GpsHdtFrame &f);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher imu_pub_;
    ros::Publisher ackermann_pub_;
    ros::Publisher gps_hdt_pub_;
    // Uma publisher por antena/GPS (campo "nr"), criada sob demanda em
    // on_gps_xyz() — não sabemos de antemão quantas vão estar ativas.
    std::map<int, ros::Publisher> gps_xyz_pubs_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::thread       ipc_thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> static_imu_tf_sent_{false};

    std::string base_frame_id_;
    std::string imu_frame_id_;
    std::string gps_frame_id_;
    std::string ipc_host_;

    bool publish_imu_;
    bool publish_ackermann_;
    bool publish_gps_;

    double imu_rotation_[9] = {1,0,0, 0,1,0, 0,0,1};
};

extern IpcBridgeNode *g_bridge_node;

} // namespace carmen_bridge