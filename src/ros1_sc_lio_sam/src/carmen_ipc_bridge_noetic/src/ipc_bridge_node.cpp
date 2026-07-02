/*
 * ipc_bridge_node.cpp  (ROS1 / Noetic)
 *
 * Responsabilidades:
 *   1. Lê velodyne (partial/variable) do CARMEN IPC
 *   2. Publica /velodyne_raw_ipc (PointCloud2 com campos brutos)
 *   3. Publica /scan (LaserScan 2D)
 *   4. Lê XSens do CARMEN IPC e publica /imu_raw
 *   5. Publica TF estáticos: base_link→velodyne, base_link→imu_link
 *
 * A conversão 3D (trigonometria pesada) foi movida para o pointcloud_node.
 *
 * Porte de ROS2: rclcpp::Node vira NodeHandle, RCLCPP_* vira ROS_*,
 * rclcpp::Time(ns) vira ros::Time(sec, nsec) via helper ns_to_rostime().
 */

#include "carmen_ipc_bridge/ipc_bridge_node.hpp"
#include <carmen/xsens_interface.h>
#include <cmath>
#include <chrono>
#include <cstring>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using carmen_bridge::IpcBridgeNode;

carmen_bridge::IpcBridgeNode *carmen_bridge::g_bridge_node = nullptr;
static constexpr int NUM_RINGS = 32;

// ─── Ângulos de elevação Hesai XT-32 ─────────────────────────────────────────
static const float HESAI_XT32_ELEVATIONS_DEG[32] = {
     15.0f,  14.0f,  13.0f,  12.0f,
     11.0f,  10.0f,   9.0f,   8.0f,
      7.0f,   6.0f,   5.0f,   4.0f,
      3.0f,   2.0f,   1.0f,   0.0f,
     -1.0f,  -2.0f,  -3.0f,  -4.0f,
     -5.0f,  -6.0f,  -7.0f,  -8.0f,
     -9.0f, -10.0f, -11.0f, -12.0f,
    -13.0f, -14.0f, -15.0f, -16.0f
};

// ─── Helper: nanossegundos (epoch) → ros::Time ───────────────────────────────
static ros::Time ns_to_rostime(int64_t ns)
{
    if (ns < 0) ns = 0;
    uint32_t sec  = static_cast<uint32_t>(ns / 1000000000LL);
    uint32_t nsec = static_cast<uint32_t>(ns % 1000000000LL);
    return ros::Time(sec, nsec);
}

// ─── Estimativa dinâmica do período de scan / correção de timestamp ─────────
//
// O IPC só entrega a mensagem de scan DEPOIS que o LiDAR terminou de girar:
// ros::Time::now() no momento do handler é o FIM do scan, não o início.
// O LIO-SAM (e o point cloud "time" field) assume que header.stamp é o
// INÍCIO do scan. Sem essa correção, timeScanEnd fica sistematicamente
// ~1 período de scan no "futuro", e o IMU nunca alcança a tempo.
//
// O período real é medido pelo intervalo entre chegadas consecutivas
// (EMA), em vez de hardcoded — funciona para qualquer Hz real do sensor.
// ─── Lógica de Sincronização CARMEN -> ROS1 ─────────────────────────────────
static double g_scan_period_ema_s = 0.05;
static double g_time_offset = 0.0;
static bool   g_offset_initialized = false;

static int64_t scan_start_stamp_ns(double carmen_ts)
{
    // 1. Trava a diferença exata entre o relógio do hardware e o ROS1
    if (!g_offset_initialized) {
        g_time_offset = ros::Time::now().toSec() - carmen_ts;
        g_offset_initialized = true;
    }

    // 2. Calcula o EMA real sem a interferência do lag de rede
    static double last_carmen_ts = 0.0;
    if (last_carmen_ts > 0.0) {
        double delta_s = carmen_ts - last_carmen_ts;
        if (delta_s > 0.005 && delta_s < 0.5)
            g_scan_period_ema_s = 0.9 * g_scan_period_ema_s + 0.1 * delta_s;
    }
    last_carmen_ts = carmen_ts;

    // 3. Aplica o offset no tempo do hardware
    double scan_start = carmen_ts - g_scan_period_ema_s;
    return static_cast<int64_t>((scan_start + g_time_offset) * 1e9);
}
// ─── Handlers IPC ─────────────────────────────────────────────────────────────

static void velodyne_partial_scan_handler(carmen_velodyne_partial_scan_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;
    const int num_shots = msg->number_of_32_laser_shots;
    if (num_shots <= 0 || num_shots > 10000) return;

    carmen_bridge::VelodyneFrame frame;
    frame.timestamp   = msg->timestamp;
    frame.num_shots   = num_shots;
    frame.shot_size   = 32;
    frame.range_div   = carmen_bridge::g_bridge_node->lidar_range_division_factor_;
    frame.max_range   = carmen_bridge::g_bridge_node->lidar_max_range_;

    frame.ranges.reserve(num_shots);
    frame.angles.reserve(num_shots);
    frame.raw_points.reserve(num_shots * 32);
    frame.shot_sizes.assign(num_shots, 32);
    frame.shot_cos_az.resize(num_shots);
    frame.shot_sin_az.resize(num_shots);

    for (int i = 0; i < num_shots; ++i) {
        // Usa o ângulo real do sensor (sentido horário, graus) → anti-horário ROS
        float az_deg  = msg->partial_scan[i].angle;
        float az_rad  = -az_deg * static_cast<float>(M_PI) / 180.0f;
        frame.shot_cos_az[i] = std::cos(az_rad);
        frame.shot_sin_az[i] = std::sin(az_rad);
        float azimuth = az_rad;
        float min_r = 999.0f;

        for (int j = 0; j < 32; ++j) {
            float r = msg->partial_scan[i].distance[j] / static_cast<float>(frame.range_div);
            carmen_bridge::VelodyneFrame::RawPoint rp;
            rp.distance  = msg->partial_scan[i].distance[j];
            rp.intensity = 100;
            rp.ring      = static_cast<uint8_t>(j);
            frame.raw_points.push_back(rp);
            if (r >= 0.1f && r <= static_cast<float>(frame.max_range) && r < min_r)
                min_r = r;
        }
        if (min_r < 999.0f) {
            frame.ranges.push_back(min_r);
            frame.angles.push_back(azimuth);
        }
    }

    frame.ros_stamp_ns = scan_start_stamp_ns(frame.timestamp);
    carmen_bridge::g_bridge_node->on_velodyne_scan(std::move(frame));
}

static void velodyne_variable_scan_handler(carmen_velodyne_variable_scan_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;
    const int num_shots = msg->number_of_shots;

    ROS_INFO_THROTTLE(1.0,
        "[DBG-SCAN-1] CARMEN->bridge: num_shots=%d  ts_carmen=%.3f",
        num_shots, msg->timestamp);

    if (num_shots <= 0 || num_shots > 10000) return;

    carmen_bridge::VelodyneFrame frame;
    frame.timestamp = msg->timestamp;
    frame.num_shots = num_shots;
    frame.range_div = carmen_bridge::g_bridge_node->lidar_range_division_factor_;
    frame.max_range = carmen_bridge::g_bridge_node->lidar_max_range_;

    try {
        // Pré-calcula cos/sin do azimute UMA vez para todos os shots
        // USA o ângulo REAL do sensor (msg->partial_scan[i].angle, em graus, sentido horário)
        // e converte para o sentido anti-horário do ROS (REP-103) negando o valor.
        // NÃO usa mais azimute sintético por índice — que causava drift quando
        // num_shots variava entre frames (o corredor "subindo" no LIO-SAM).
        std::vector<float> cos_az(num_shots), sin_az(num_shots);
        for (int i = 0; i < num_shots; ++i) {
            // ângulo real em graus (sentido horário, 0–360)
            float az_deg = msg->partial_scan[i].angle;
            // converte para radianos e inverte sentido (horário→anti-horário)
            float az_rad = -az_deg * static_cast<float>(M_PI) / 180.0f;
            cos_az[i] = std::cos(az_rad);
            sin_az[i] = std::sin(az_rad);
        }

        frame.ranges.reserve(num_shots);
        frame.angles.reserve(num_shots);
        frame.shot_sizes.reserve(num_shots);
        frame.raw_points.reserve(num_shots * 32);
        // Propaga cos/sin do ângulo real para o velodyneframe_to_raw_cloud
        frame.shot_cos_az.resize(num_shots);
        frame.shot_sin_az.resize(num_shots);
        for (int i = 0; i < num_shots; ++i) {
            frame.shot_cos_az[i] = cos_az[i];
            frame.shot_sin_az[i] = sin_az[i];
        }

        for (int i = 0; i < num_shots; ++i) {
            int shot_size = msg->partial_scan[i].shot_size;
            if (shot_size <= 0 || shot_size > 128) {
                frame.shot_sizes.push_back(0);
                continue;
            }
            frame.shot_sizes.push_back(shot_size);
            if (frame.shot_size == 0) frame.shot_size = shot_size;

            float min_r = 999.0f;
            for (int j = 0; j < shot_size; ++j) {
                float r = msg->partial_scan[i].distance[j] / static_cast<float>(frame.range_div);
                carmen_bridge::VelodyneFrame::RawPoint rp;
                rp.distance  = msg->partial_scan[i].distance[j];
                rp.intensity = (msg->partial_scan[i].intensity)
                               ? static_cast<uint8_t>(msg->partial_scan[i].intensity[j])
                               : 0;
                rp.ring = static_cast<uint8_t>(j < 32 ? j : 31);
                frame.raw_points.push_back(rp);

                if (r >= 0.1f && r <= static_cast<float>(frame.max_range) && r < min_r)
                    min_r = r;
            }

            // azimute real do sensor — já calculado no array cos_az/sin_az acima
            // reconstrói o ângulo em radianos para o LaserScan 2D
            float azimuth = std::atan2(sin_az[i], cos_az[i]);
            if (min_r < 999.0f) {
                frame.ranges.push_back(min_r);
                frame.angles.push_back(azimuth);
            }
        }

        ROS_INFO_THROTTLE(1.0,
            "[DBG-SCAN-2] raw_pts=%zu  ranges_2d=%zu  (shots=%d)",
            frame.raw_points.size(), frame.ranges.size(), num_shots);

        frame.ros_stamp_ns = scan_start_stamp_ns(msg->timestamp);
        carmen_bridge::g_bridge_node->on_velodyne_scan(std::move(frame));

    } catch (const std::exception &e) {
        ROS_ERROR("EXCEÇÃO no variable_scan_handler: %s", e.what());
    }
}

static void xsens_handler(carmen_xsens_global_quat_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;

    static std::atomic<uint64_t> xsens_raw_count{0};
    ++xsens_raw_count;
    ROS_INFO_THROTTLE(1.0,
        "[DBG-XSENS] msgs_carmen/s=%lu  ts_carmen=%.3f",
        static_cast<unsigned long>(xsens_raw_count.exchange(0)), msg->timestamp);

    carmen_bridge::ImuFrame frame;
    frame.timestamp = msg->timestamp;
    frame.ax = msg->m_acc.x;
    frame.ay = msg->m_acc.y;
    frame.az = msg->m_acc.z;
    frame.gx = msg->m_gyr.x;
    frame.gy = msg->m_gyr.y;
    frame.gz = msg->m_gyr.z;
    frame.qw = msg->quat_data.m_data[0];
    frame.qx = msg->quat_data.m_data[1];
    frame.qy = msg->quat_data.m_data[2];
    frame.qz = msg->quat_data.m_data[3];
    frame.has_orientation = true;

    if (!g_offset_initialized) {
        g_time_offset = ros::Time::now().toSec() - msg->timestamp;
        g_offset_initialized = true;
    }
    frame.ros_stamp_ns = static_cast<int64_t>((msg->timestamp + g_time_offset) * 1e9);

    carmen_bridge::g_bridge_node->on_imu(frame);
}

// ─── Constructor / Destructor ─────────────────────────────────────────────────

IpcBridgeNode::IpcBridgeNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), scan_queue_(20)
{
    g_bridge_node = this;

    for (int i = 0; i < NUM_RINGS; ++i)
        ring_elevation_rad_[i] = HESAI_XT32_ELEVATIONS_DEG[i] * static_cast<float>(M_PI) / 180.0f;

    declare_parameters();
    setup_publishers();
    setup_tf();
    start_ipc_thread();

    cloud_thread_ = std::thread([this]() {
        while (running_) {
            thread_publish_scan();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    ROS_INFO("ipc_bridge_node iniciado.");
    ROS_INFO("  RawCloud: %s  LaserScan: %s  IMU: %s",
        publish_raw_cloud_ ? "SIM" : "NAO",
        publish_laserscan_  ? "SIM" : "NAO",
        publish_imu_        ? "SIM" : "NAO");
}

IpcBridgeNode::~IpcBridgeNode()
{
    running_ = false;
    if (cloud_thread_.joinable()) cloud_thread_.join();
    IPC_disconnect();
    if (ipc_thread_.joinable()) ipc_thread_.join();
}

// ─── Inicialização ────────────────────────────────────────────────────────────

void IpcBridgeNode::declare_parameters()
{
    pnh_.param<std::string>("base_frame_id",  base_frame_id_,  "base_link");
    pnh_.param<std::string>("laser_frame_id", laser_frame_id_, "velodyne");
    pnh_.param<std::string>("imu_frame_id",   imu_frame_id_,   "imu_link");
    pnh_.param<double>("max_laser_range",  max_laser_range_, 120.0);
    pnh_.param<double>("laser_min_angle",  laser_min_angle_, -M_PI);
    pnh_.param<double>("laser_max_angle",  laser_max_angle_,  M_PI);
    pnh_.param<int>   ("laser_num_beams",  laser_num_beams_,  360);
    pnh_.param<int>   ("lidar_sensor_id",  lidar_sensor_id_,  5);
    pnh_.param<bool>  ("publish_raw_cloud", publish_raw_cloud_, true);
    pnh_.param<bool>  ("publish_laserscan", publish_laserscan_, true);
    pnh_.param<bool>  ("publish_imu",       publish_imu_,       true);
    pnh_.param<std::string>("ipc_host",    ipc_host_,       "localhost");

    std::vector<double> rot;
    pnh_.param("imu_rotation", rot, std::vector<double>{1,0,0, 0,1,0, 0,0,1});
    if (rot.size() == 9)
        for (int i = 0; i < 9; ++i) imu_rotation_[i] = rot[i];

    ROS_INFO(
        "imu_rotation: [%.0f %.0f %.0f | %.0f %.0f %.0f | %.0f %.0f %.0f]",
        imu_rotation_[0], imu_rotation_[1], imu_rotation_[2],
        imu_rotation_[3], imu_rotation_[4], imu_rotation_[5],
        imu_rotation_[6], imu_rotation_[7], imu_rotation_[8]);

    lidar_z_ = 1.8;
}

void IpcBridgeNode::setup_publishers()
{
    if (publish_raw_cloud_)
        raw_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_raw_ipc", 5);
    if (publish_laserscan_)
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
    if (publish_imu_)
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu_raw", 2000);
}

void IpcBridgeNode::setup_tf()
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
    setup_tf_from_carmen();

    geometry_msgs::TransformStamped tf_imu;
    tf_imu.header.stamp    = ros::Time::now();
    tf_imu.header.frame_id = base_frame_id_;
    tf_imu.child_frame_id  = imu_frame_id_;
    tf_imu.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf_imu);
}

void IpcBridgeNode::setup_tf_from_carmen()
{
    tf2::Quaternion q;
    q.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);
    geometry_msgs::TransformStamped tf_lidar;
    tf_lidar.header.stamp    = ros::Time::now();
    tf_lidar.header.frame_id = base_frame_id_;
    tf_lidar.child_frame_id  = laser_frame_id_;
    tf_lidar.transform.translation.x = lidar_x_;
    tf_lidar.transform.translation.y = lidar_y_;
    tf_lidar.transform.translation.z = lidar_z_;
    tf_lidar.transform.rotation      = tf2::toMsg(q);
    static_tf_broadcaster_->sendTransform(tf_lidar);
    ROS_INFO(
        "TF %s->%s: xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
        base_frame_id_.c_str(), laser_frame_id_.c_str(),
        lidar_x_, lidar_y_, lidar_z_, lidar_roll_, lidar_pitch_, lidar_yaw_);
}

void IpcBridgeNode::load_carmen_lidar_config(int argc, char **argv)
{
    carmen_lidar_config *cfg = static_cast<carmen_lidar_config *>(
        calloc(1, sizeof(carmen_lidar_config)));
    if (!cfg) return;
    carmen_param_allow_unfound_variables(1);
    load_lidar_config(argc, argv, lidar_sensor_id_, &cfg);
    carmen_param_allow_unfound_variables(0);
    if (cfg->range_division_factor <= 0 || cfg->shot_size <= 0) { free(cfg); return; }
    lidar_range_division_factor_ = cfg->range_division_factor;
    lidar_max_range_  = cfg->max_range;
    lidar_shot_size_  = cfg->shot_size;
    lidar_x_    = cfg->pose.position.x;
    lidar_y_    = cfg->pose.position.y;
    lidar_z_    = cfg->pose.position.z;
    lidar_roll_ = cfg->pose.orientation.roll;
    lidar_pitch_= cfg->pose.orientation.pitch;
    lidar_yaw_  = cfg->pose.orientation.yaw;
    max_laser_range_ = lidar_max_range_;
    lidar_config_loaded_ = true;
    ROS_INFO(
        "lidar_config sensor %d: shot_size=%d, range_div=%.0f, max_range=%.1fm",
        lidar_sensor_id_, lidar_shot_size_,
        lidar_range_division_factor_, lidar_max_range_);
    if (cfg->vertical_angles) free(cfg->vertical_angles);
    if (cfg->ray_order)       free(cfg->ray_order);
    free(cfg);
}

void IpcBridgeNode::start_ipc_thread()
{
    std::string ipc_host = ipc_host_;
    ipc_thread_ = std::thread([this, ipc_host]() {
        std::string prog = "carmen_ipc_bridge";
        std::string flag = "-central_host";
        std::vector<char*> argv_vec = {
            const_cast<char*>(prog.c_str()),
            const_cast<char*>(flag.c_str()),
            const_cast<char*>(ipc_host.c_str())
        };
        int argc = static_cast<int>(argv_vec.size());
        carmen_ipc_initialize(argc, argv_vec.data());
        load_carmen_lidar_config(argc, argv_vec.data());
        if (lidar_config_loaded_) setup_tf_from_carmen();

        carmen_velodyne_subscribe_partial_scan_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(velodyne_partial_scan_handler),
            CARMEN_SUBSCRIBE_LATEST);
        carmen_xsens_subscribe_xsens_global_quat_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(xsens_handler),
            CARMEN_SUBSCRIBE_LATEST);
        carmen_velodyne_subscribe_variable_scan_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(velodyne_variable_scan_handler),
            CARMEN_SUBSCRIBE_LATEST, lidar_sensor_id_);

        ROS_INFO(
            "IPC dispatch loop iniciado (host=%s, lidar_sensor_id=%d).",
            ipc_host.c_str(), lidar_sensor_id_);

        while (running_) IPC_listenWait(1);
        IPC_disconnect();
    });
}

// ─── Callbacks ────────────────────────────────────────────────────────────────

void IpcBridgeNode::on_velodyne_scan(VelodyneFrame frame)
{
    scan_queue_.push(std::move(frame));
}

void IpcBridgeNode::on_imu(const ImuFrame &frame)
{
    static std::atomic<uint64_t> imu_count{0};
    static double imu_t0 = 0.0;
    ++imu_count;
    double now_s = ros::Time::now().toSec();
    if (imu_t0 == 0.0) imu_t0 = now_s;
    if (now_s - imu_t0 >= 1.0) {
        double hz = static_cast<double>(imu_count.exchange(0)) / (now_s - imu_t0);
        imu_t0 = now_s;
        ROS_INFO(
            "[DBG-IMU] Hz_real=%.1f  ax=%.3f ay=%.3f az=%.3f  gx=%.3f gy=%.3f gz=%.3f",
            hz, frame.ax, frame.ay, frame.az, frame.gx, frame.gy, frame.gz);
    }
    if (!publish_imu_) return;
    imu_pub_.publish(imu_to_msg(frame));
}

// ─── Publicação da fila de scan ───────────────────────────────────────────────

void IpcBridgeNode::thread_publish_scan()
{
    VelodyneFrame latest;
    if (!scan_queue_.pop_latest(latest)) return;

    if (publish_raw_cloud_)
        raw_cloud_pub_.publish(velodyneframe_to_raw_cloud(latest));

    if (publish_laserscan_)
        scan_pub_.publish(velodyne_to_laserscan(latest));
}

// ─── Conversões ──────────────────────────────────────────────────────────────

// Publica os dados brutos de distância+intensidade como PointCloud2
// com campos customizados que o pointcloud_node vai ler.
// Campos: shot_idx(uint16), ring(uint8), distance(uint16), intensity(uint8), cos_az(float), sin_az(float)
sensor_msgs::PointCloud2
IpcBridgeNode::velodyneframe_to_raw_cloud(const VelodyneFrame &f)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp    = ns_to_rostime(f.ros_stamp_ns);
    cloud.header.frame_id = laser_frame_id_;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense     = true;

    // Campos do frame bruto
    // offset: shot_idx(0,2) ring(2,1) distance(4,2) intensity(6,1) pad(7,1) cos_az(8,4) sin_az(12,4) = 16 bytes
    cloud.fields.resize(6);
    cloud.fields[0].name="shot_idx";  cloud.fields[0].offset=0;  cloud.fields[0].datatype=4; cloud.fields[0].count=1; // UINT16
    cloud.fields[1].name="ring";      cloud.fields[1].offset=2;  cloud.fields[1].datatype=2; cloud.fields[1].count=1; // UINT8
    cloud.fields[2].name="distance";  cloud.fields[2].offset=4;  cloud.fields[2].datatype=4; cloud.fields[2].count=1; // UINT16
    cloud.fields[3].name="intensity"; cloud.fields[3].offset=6;  cloud.fields[3].datatype=2; cloud.fields[3].count=1; // UINT8
    cloud.fields[4].name="cos_az";    cloud.fields[4].offset=8;  cloud.fields[4].datatype=7; cloud.fields[4].count=1; // FLOAT32
    cloud.fields[5].name="sin_az";    cloud.fields[5].offset=12; cloud.fields[5].datatype=7; cloud.fields[5].count=1; // FLOAT32

    cloud.point_step = 16;
    cloud.width      = static_cast<uint32_t>(f.raw_points.size());
    cloud.row_step   = cloud.width * cloud.point_step;
    cloud.data.resize(cloud.row_step, 0);

    // cos/sin do azimute real calculados do ângulo do sensor (msg->partial_scan[i].angle)
    // no handler e propagados via f.shot_cos_az / f.shot_sin_az (um por shot).
    // Se o vetor não estiver disponível (frame antigo), usa fallback sintético.
    const float angle_step_fb = static_cast<float>(2.0 * M_PI / std::max(f.num_shots, 1));
    int raw_idx = 0;
    uint8_t *ptr = cloud.data.data();

    for (int i = 0; i < f.num_shots; ++i) {
        float cos_az, sin_az;
        if (i < (int)f.shot_cos_az.size()) {
            // ângulo real do sensor — preenchido no velodyne_variable_scan_handler
            cos_az = f.shot_cos_az[i];
            sin_az = f.shot_sin_az[i];
        } else {
            // fallback sintético — só ocorre se o frame vier sem shot_cos_az
            float az = -(static_cast<float>(i) * angle_step_fb - static_cast<float>(M_PI));
            cos_az = std::cos(az);
            sin_az = std::sin(az);
        }
        int ss = (i < (int)f.shot_sizes.size()) ? f.shot_sizes[i] : f.shot_size;
        for (int j = 0; j < ss && raw_idx < (int)f.raw_points.size(); ++j, ++raw_idx) {
            const auto &rp = f.raw_points[raw_idx];
            uint16_t shot_i = static_cast<uint16_t>(i);
            std::memcpy(ptr + 0,  &shot_i,      2);
            std::memcpy(ptr + 2,  &rp.ring,     1);
            std::memcpy(ptr + 4,  &rp.distance, 2);
            std::memcpy(ptr + 6,  &rp.intensity,1);
            std::memcpy(ptr + 8,  &cos_az,      4);
            std::memcpy(ptr + 12, &sin_az,      4);
            ptr += cloud.point_step;
        }
    }
    cloud.width = static_cast<uint32_t>((ptr - cloud.data.data()) / cloud.point_step);
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(cloud.row_step);

    return cloud;
}

sensor_msgs::LaserScan
IpcBridgeNode::velodyne_to_laserscan(const VelodyneFrame &f)
{
    sensor_msgs::LaserScan scan;
    scan.header.stamp    = ns_to_rostime(f.ros_stamp_ns);
    scan.header.frame_id = laser_frame_id_;
    scan.angle_min       = static_cast<float>(laser_min_angle_);
    scan.angle_max       = static_cast<float>(laser_max_angle_);
    scan.range_min       = 0.1f;
    scan.range_max       = static_cast<float>(max_laser_range_);
    const int    num_bins = laser_num_beams_;
    const double bin_sz   = (laser_max_angle_ - laser_min_angle_) / num_bins;
    scan.angle_increment = static_cast<float>(bin_sz);
    scan.time_increment  = 0.0f;
    scan.scan_time       = 0.1f;
    scan.ranges.assign(num_bins, scan.range_max);
    scan.intensities.assign(num_bins, 0.0f);
    for (size_t i = 0; i < f.ranges.size(); ++i) {
        float r = f.ranges[i], a = f.angles[i];
        if (r < 0.1f || r > scan.range_max) continue;
        if (a < laser_min_angle_ || a >= laser_max_angle_) continue;
        int bin = static_cast<int>((a - laser_min_angle_) / bin_sz);
        if (bin >= 0 && bin < num_bins && r < scan.ranges[bin])
            scan.ranges[bin] = r;
    }
    return scan;
}

sensor_msgs::Imu
IpcBridgeNode::imu_to_msg(const ImuFrame &f)
{
    sensor_msgs::Imu msg;
    msg.header.stamp    = ns_to_rostime(f.ros_stamp_ns);
    msg.header.frame_id = imu_frame_id_;
    const double *R = imu_rotation_;
    msg.linear_acceleration.x = R[0]*f.ax + R[1]*f.ay + R[2]*f.az;
    msg.linear_acceleration.y = R[3]*f.ax + R[4]*f.ay + R[5]*f.az;
    msg.linear_acceleration.z = R[6]*f.ax + R[7]*f.ay + R[8]*f.az;
    msg.angular_velocity.x = R[0]*f.gx + R[1]*f.gy + R[2]*f.gz;
    msg.angular_velocity.y = R[3]*f.gx + R[4]*f.gy + R[5]*f.gz;
    msg.angular_velocity.z = R[6]*f.gx + R[7]*f.gy + R[8]*f.gz;
    if (f.has_orientation) {
        msg.orientation.x = f.qx;
        msg.orientation.y = f.qy;
        msg.orientation.z = f.qz;
        msg.orientation.w = f.qw;
        msg.orientation_covariance[0] = 0.01;
        msg.orientation_covariance[4] = 0.01;
        msg.orientation_covariance[8] = 0.01;
    } else {
        msg.orientation_covariance[0] = -1.0;
    }
    msg.angular_velocity_covariance[0]    = 0.01;
    msg.angular_velocity_covariance[4]    = 0.01;
    msg.angular_velocity_covariance[8]    = 0.01;
    msg.linear_acceleration_covariance[0] = 0.1;
    msg.linear_acceleration_covariance[4] = 0.1;
    msg.linear_acceleration_covariance[8] = 0.1;
    return msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ipc_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    IpcBridgeNode node(nh, pnh);
    ros::spin();
    return 0;
}
