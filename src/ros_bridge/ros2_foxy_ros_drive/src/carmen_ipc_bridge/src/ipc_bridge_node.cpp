/*
 * ipc_bridge_node.cpp  —  carmen_ipc_bridge (standalone)
 *
 * Lê dados do CARMEN IPC e publica tópicos ROS2 padrão:
 *   /velodyne_points  (sensor_msgs/PointCloud2)  ← LIO-SAM usa este
 *   /scan             (sensor_msgs/LaserScan)     ← opcional, visualização
 *   /odom             (nav_msgs/Odometry)
 *   /imu/data         (sensor_msgs/Imu)
 *
 * SEM dependência de slam_toolbox.
 * SEM publicação de volta ao IPC (bridge unidirecional IPC → ROS2).
 */

#include "carmen_ipc_bridge/ipc_bridge_node.hpp"
#include <carmen/xsens_interface.h>

#include <cmath>
#include <chrono>
#include <cstring>
#include <iostream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using carmen_bridge::IpcBridgeNode;

// ─── Global node ptr (usado pelos handlers IPC estáticos) ────────────────────
carmen_bridge::IpcBridgeNode *carmen_bridge::g_bridge_node = nullptr;
static rclcpp::Logger g_logger = rclcpp::get_logger("ipc_bridge");
static constexpr int NUM_RINGS = 32;

// Clock global — rclcpp::Clock::make_shared() dentro de thread sem contexto
// ROS2 falha no Foxy com "getting current steady time failed".
// Um único clock compartilhado criado na thread principal resolve o problema.
static rclcpp::Clock::SharedPtr g_clock;

// ─────────────────────────────────────────────────────────────────────────────
//  Ângulos de elevação do Hesai Pandar XT-32
//  Fonte: datasheet oficial, ordem do bit-stream (anel 0 = mais baixo)
// ─────────────────────────────────────────────────────────────────────────────
// Ordem do bit-stream CARMEN/Hesai XT-32: ring 0 = topo (+15°), ring 31 = base (-16°)
// (oposto ao datasheet — o driver CARMEN inverte a ordem dos anéis)
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

// ─────────────────────────────────────────────────────────────────────────────
//  Handlers IPC estáticos
//  DEVEM ser definidos ANTES do construtor que os referencia.
// ─────────────────────────────────────────────────────────────────────────────

// Handler: partial_scan (HDL-32E legado — formato 32 feixes por shot)
static void velodyne_partial_scan_handler(carmen_velodyne_partial_scan_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;

    const int num_shots = msg->number_of_32_laser_shots;

    if (num_shots <= 0 || num_shots > 10000) {
        RCLCPP_WARN_THROTTLE(g_logger, *g_clock, 2000,
            "partial_scan: num_shots inválido (%d), descartando frame", num_shots);
        return;
    }

    carmen_bridge::VelodyneFrame frame;
    frame.timestamp = msg->timestamp;

    const float angle_step = static_cast<float>(2.0 * M_PI / num_shots);

    // Reserva para 2D e 3D simultaneamente
    frame.ranges.reserve(num_shots);
    frame.angles.reserve(num_shots);
    frame.points.reserve(num_shots * 32);

    for (int i = 0; i < num_shots; ++i) {
        float azimuth = static_cast<float>(i) * angle_step - static_cast<float>(M_PI);
        float min_r = 999.0f;

        for (int j = 0; j < 32; ++j) {
            // partial_scan legado: divisor padrão Velodyne = 500 (mesmo factor do XT-32)
            float r = msg->partial_scan[i].distance[j]
                      / static_cast<float>(carmen_bridge::g_bridge_node->lidar_range_division_factor_);
            if (r < 0.1f || r > static_cast<float>(carmen_bridge::g_bridge_node->lidar_max_range_)) continue;

            // Ângulo de elevação: usa tabela XT-32 se disponível, fallback linear
            float elev_deg = (j < 32) ? HESAI_XT32_ELEVATIONS_DEG[j]
                                       : (-16.0f + j * 1.0f);
            float elev_rad = elev_deg * static_cast<float>(M_PI) / 180.0f;

            float cos_elev = std::cos(elev_rad);
            float sin_elev = std::sin(elev_rad);
            float cos_az   = std::cos(azimuth);
            float sin_az   = std::sin(azimuth);

            carmen_bridge::VelodyneFrame::Point3D pt;
            pt.x = r * cos_elev * cos_az;
            pt.y = r * cos_elev * sin_az;
            pt.z = r * sin_elev;
            pt.intensity = 100.0f; // partial_scan não tem intensidade
            pt.ring = static_cast<uint16_t>(j);  // j = 0..31
            frame.points.push_back(pt);

            if (r < min_r) min_r = r;
        }

        // Projeção 2D: range mínimo do shot → LaserScan
        if (min_r < 999.0f) {
            frame.ranges.push_back(min_r);
            frame.angles.push_back(azimuth);
        }
    }

    RCLCPP_DEBUG(g_logger, "partial_scan: %d shots, %zu pontos 3D",
                 num_shots, frame.points.size());

    carmen_bridge::g_bridge_node->on_velodyne_scan(frame);
}

static void velodyne_variable_scan_handler(carmen_velodyne_variable_scan_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;

    const int num_shots = msg->number_of_shots;

    // Log para monitorar o tamanho real que o CARMEN está enviando
    RCLCPP_INFO_THROTTLE(g_logger, *g_clock, 1000,
        "variable_scan handler: num_shots=%d", num_shots);

    if (num_shots <= 0 || num_shots > 10000) {
        RCLCPP_WARN_THROTTLE(g_logger, *g_clock, 2000,
            "variable_scan: num_shots inválido (%d), descartando frame", num_shots);
        return;
    }

    carmen_bridge::VelodyneFrame frame;
    frame.timestamp = msg->timestamp;

    try {
        const float angle_step = static_cast<float>(2.0 * M_PI / num_shots);

        frame.ranges.reserve(num_shots);
        frame.angles.reserve(num_shots);
        frame.points.reserve(num_shots * 32);

        for (int i = 0; i < num_shots; ++i) {
            float azimuth = static_cast<float>(i) * angle_step - static_cast<float>(M_PI);
            int shot_size = msg->partial_scan[i].shot_size;

            if (shot_size <= 0 || shot_size > 128) continue;
            float min_r = 999.0f;

            for (int j = 0; j < shot_size; ++j) {
                // Usa range_division_factor lido do carmen.ini (default 500 = XT-32)
                float r = msg->partial_scan[i].distance[j]
                          / static_cast<float>(carmen_bridge::g_bridge_node->lidar_range_division_factor_);
                if (r < 0.1f || r > static_cast<float>(carmen_bridge::g_bridge_node->lidar_max_range_)) continue;

                float elev_deg = (j < NUM_RINGS) ? HESAI_XT32_ELEVATIONS_DEG[j]
                                                 : (-16.0f + j * (32.0f / std::max(shot_size, 1)));
                float elev_rad = elev_deg * static_cast<float>(M_PI) / 180.0f;

                float cos_elev = std::cos(elev_rad);
                float sin_elev = std::sin(elev_rad);

                carmen_bridge::VelodyneFrame::Point3D pt;
                pt.x = r * cos_elev * std::cos(azimuth);
                pt.y = r * cos_elev * std::sin(azimuth);
                pt.z = r * sin_elev;
                pt.intensity = (j < shot_size && msg->partial_scan[i].intensity != nullptr)
                               ? static_cast<float>(msg->partial_scan[i].intensity[j])
                               : 0.0f;
                pt.ring = static_cast<uint16_t>(j);
                frame.points.push_back(pt);

                if (r < min_r) min_r = r;
            }

            if (min_r < 999.0f) {
                frame.ranges.push_back(min_r);
                frame.angles.push_back(azimuth);
            }
        }

        carmen_bridge::g_bridge_node->on_velodyne_scan(std::move(frame));

    } catch (const std::exception &e) {
        RCLCPP_ERROR(g_logger,
            "EXCEÇÃO no variable_scan_handler: %s | num_shots=%d, points_acumulados=%zu",
            e.what(), num_shots, frame.points.size());
    }
}

// Handler: fused_odometry — dead-reckoning Ackermann relativo ao frame odom
static void fused_odometry_handler(carmen_fused_odometry_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;

    static double dr_x = 0.0, dr_y = 0.0, dr_theta = 0.0;
    static double last_ts = 0.0;
    static bool   first   = true;
    static int    count   = 0;
    count++;

    if (first) {
        first   = false;
        last_ts = msg->timestamp;

        carmen_bridge::OdometryFrame frame{};
        frame.v         = msg->velocity.x;
        frame.phi       = msg->phi;
        frame.timestamp = msg->timestamp;
        frame.abs_x     = msg->pose.position.x;
        frame.abs_y     = msg->pose.position.y;
        frame.abs_theta = msg->pose.orientation.yaw;
        // pose relativa começa em zero
        frame.x = 0.0; frame.y = 0.0; frame.theta = 0.0;

        RCLCPP_INFO(g_logger,
            "Primeira odometria (msg #%d): abs=(%.2f, %.2f, %.3f rad)",
            count, frame.abs_x, frame.abs_y, frame.abs_theta);
        carmen_bridge::g_bridge_node->on_odometry(frame);
        return;
    }

    double dt  = msg->timestamp - last_ts;
    last_ts    = msg->timestamp;
    double v   = msg->velocity.x;
    double phi = msg->phi;

    // dt fora de [0, 0.5s]: descarta (pausa no log ou jump)
    if (dt > 0.0 && dt < 0.5) {
        // Modelo Ackermann com wheelbase configurado
        // (valor padrão: 2.625m para o Argos)
        const double L = carmen_bridge::g_bridge_node
                         ? /* lido do param */ 2.625 : 2.625;
        dr_x     += v * dt * std::cos(dr_theta);
        dr_y     += v * dt * std::sin(dr_theta);
        dr_theta += (v / L) * std::tan(phi) * dt;
        // Normaliza [-pi, pi]
        while (dr_theta >  M_PI) dr_theta -= 2.0 * M_PI;
        while (dr_theta < -M_PI) dr_theta += 2.0 * M_PI;
    }

    carmen_bridge::OdometryFrame frame{};
    frame.x         = dr_x;
    frame.y         = dr_y;
    frame.theta     = dr_theta;
    frame.v         = v;
    frame.phi       = phi;
    frame.timestamp = msg->timestamp;
    frame.abs_x     = msg->pose.position.x;
    frame.abs_y     = msg->pose.position.y;
    frame.abs_theta = msg->pose.orientation.yaw;

    carmen_bridge::g_bridge_node->on_odometry(frame);
}

static void xsens_handler(carmen_xsens_global_quat_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;

    carmen_bridge::ImuFrame frame;
    frame.timestamp = msg->timestamp;

    // Aceleração linear [m/s²]
    frame.ax = msg->m_acc.x;
    frame.ay = msg->m_acc.y;
    frame.az = msg->m_acc.z;

    // Velocidade angular [rad/s]
    frame.gx = msg->m_gyr.x;
    frame.gy = msg->m_gyr.y;
    frame.gz = msg->m_gyr.z;

    // Quaternion: XSens quat_data.m_data = [w, x, y, z]
    frame.qw = msg->quat_data.m_data[0];
    frame.qx = msg->quat_data.m_data[1];
    frame.qy = msg->quat_data.m_data[2];
    frame.qz = msg->quat_data.m_data[3];
    frame.has_orientation = true;

    carmen_bridge::g_bridge_node->on_imu(frame);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

IpcBridgeNode::IpcBridgeNode(const rclcpp::NodeOptions &options)
    : Node("ipc_bridge_node", options),
      scan_queue_(20),
      odom_queue_(50),
      imu_queue_(100)
{
    g_bridge_node = this;
    g_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Preenche tabela de elevações em radianos
    for (int i = 0; i < NUM_RINGS; ++i) {
        ring_elevation_rad_[i] = HESAI_XT32_ELEVATIONS_DEG[i]
                                  * static_cast<float>(M_PI) / 180.0f;
    }

    declare_parameters();
    setup_publishers();
    setup_tf();
    start_ipc_thread();

    // Timers de drenagem das filas
    scan_timer_ = create_wall_timer(50ms,  [this]{ timer_publish_scan(); });
    odom_timer_ = create_wall_timer(20ms,  [this]{ timer_publish_odom(); });
    imu_timer_  = create_wall_timer(5ms,   [this]{ timer_publish_imu();  });

    RCLCPP_INFO(get_logger(), "carmen_ipc_bridge iniciado (standalone, sem slam_toolbox).");
    RCLCPP_INFO(get_logger(), "  PointCloud2: %s  LaserScan: %s  IMU: %s",
        publish_pointcloud_ ? "SIM" : "NÃO",
        publish_laserscan_  ? "SIM" : "NÃO",
        publish_imu_        ? "SIM" : "NÃO");
}

IpcBridgeNode::~IpcBridgeNode()
{
    running_ = false;
    IPC_disconnect();
    if (ipc_thread_.joinable()) ipc_thread_.join();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Inicialização
// ─────────────────────────────────────────────────────────────────────────────

void IpcBridgeNode::declare_parameters()
{
    // Frames TF
    declare_parameter<std::string>("base_frame_id",   "base_link");
    declare_parameter<std::string>("odom_frame_id",   "odom");
    declare_parameter<std::string>("laser_frame_id",  "velodyne");
    declare_parameter<std::string>("imu_frame_id",    "imu_link");

    // LiDAR — valores default sobrescritos pelo CARMEN se load_lidar_config funcionar
    declare_parameter<double>("max_laser_range",    120.0);
    declare_parameter<double>("laser_min_angle",    -M_PI);
    declare_parameter<double>("laser_max_angle",     M_PI);
    declare_parameter<int>   ("laser_num_beams",     360);

    // ID do sensor CARMEN (lidar N) — determina a subscription e o param group
    declare_parameter<int>("lidar_sensor_id", 5);

    // Quais tópicos publicar
    declare_parameter<bool>("publish_pointcloud",   true);
    declare_parameter<bool>("publish_laserscan",    true);
    declare_parameter<bool>("publish_imu",          true);

    // Odometria
    declare_parameter<double>("wheelbase", 2.625);

    // IPC
    declare_parameter<std::string>("ipc_host", "localhost");

    // Lê valores
    base_frame_id_      = get_parameter("base_frame_id").as_string();
    odom_frame_id_      = get_parameter("odom_frame_id").as_string();
    laser_frame_id_     = get_parameter("laser_frame_id").as_string();
    imu_frame_id_       = get_parameter("imu_frame_id").as_string();
    max_laser_range_    = get_parameter("max_laser_range").as_double();
    laser_min_angle_    = get_parameter("laser_min_angle").as_double();
    laser_max_angle_    = get_parameter("laser_max_angle").as_double();
    laser_num_beams_    = get_parameter("laser_num_beams").as_int();
    lidar_sensor_id_    = get_parameter("lidar_sensor_id").as_int();
    publish_pointcloud_ = get_parameter("publish_pointcloud").as_bool();
    publish_laserscan_  = get_parameter("publish_laserscan").as_bool();
    publish_imu_        = get_parameter("publish_imu").as_bool();
    wheelbase_          = get_parameter("wheelbase").as_double();
    // lidar_range_division_factor_, lidar_max_range_, lidar_shot_size_ e
    // lidar_x/y/z/roll/pitch/yaw têm defaults inline no hpp e são sobrescritos
    // por load_carmen_lidar_config() na thread IPC.
    lidar_z_ = 1.8;  // fallback de altura até CARMEN responder
}

void IpcBridgeNode::setup_publishers()
{
    // LIO-SAM (Foxy) subscreve /points_raw com SYSTEM_DEFAULT (Reliable, Volatile).
    // SensorDataQoS usa BestEffort — incompatível, a nuvem nunca aparece no LIO-SAM.
    // Usamos keep_last(5) Reliable para o pointcloud e mantemos BestEffort só para scan/IMU.
    // No Foxy, ReliabilityPolicy e DurabilityPolicy ficam em rmw::
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(5))
                            .reliable()
                            .durability_volatile();

    auto qos_sensor = rclcpp::SensorDataQoS().keep_last(1); // BestEffort — scan e IMU

    if (publish_pointcloud_) {
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points_raw", qos_reliable);  // tópico que o LIO-SAM escuta
    }
    if (publish_laserscan_) {
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", qos_sensor);
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    if (publish_imu_) {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "/imu_raw", qos_sensor);  // tópico que o LIO-SAM escuta
    }
}

void IpcBridgeNode::setup_tf()
{
    tf_broadcaster_        = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // TF estático provisório com defaults — será re-emitido após load_carmen_lidar_config
    setup_tf_from_carmen();

    // TF estático: base_link → imu_link
    geometry_msgs::msg::TransformStamped tf_imu;
    tf_imu.header.stamp    = rclcpp::Time(0);
    tf_imu.header.frame_id = base_frame_id_;
    tf_imu.child_frame_id  = imu_frame_id_;
    tf_imu.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf_imu);
}

// Emite (ou re-emite) o TF estático base_link → velodyne usando a pose
// lida do CARMEN. Chamado uma vez no construtor (com defaults) e de novo
// na thread IPC após load_carmen_lidar_config() preencher os campos reais.
void IpcBridgeNode::setup_tf_from_carmen()
{
    tf2::Quaternion q;
    q.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);

    geometry_msgs::msg::TransformStamped tf_lidar;
    // stamp=0 → TF estático válido para qualquer timestamp, incluindo logs antigos
    tf_lidar.header.stamp    = rclcpp::Time(0);
    tf_lidar.header.frame_id = base_frame_id_;
    tf_lidar.child_frame_id  = laser_frame_id_;
    tf_lidar.child_frame_id  = laser_frame_id_;
    tf_lidar.transform.translation.x = lidar_x_;
    tf_lidar.transform.translation.y = lidar_y_;
    tf_lidar.transform.translation.z = lidar_z_;
    tf_lidar.transform.rotation      = tf2::toMsg(q);
    static_tf_broadcaster_->sendTransform(tf_lidar);

    RCLCPP_INFO(get_logger(),
        "TF %s→%s: xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
        base_frame_id_.c_str(), laser_frame_id_.c_str(),
        lidar_x_, lidar_y_, lidar_z_,
        lidar_roll_, lidar_pitch_, lidar_yaw_);
}

// Lê os parâmetros do sensor N do carmen.ini via carmen_param e popula
// os campos lidar_* do node. Chamada dentro da thread IPC, após
// carmen_ipc_initialize() (pré-requisito para carmen_param funcionar).
void IpcBridgeNode::load_carmen_lidar_config(int argc, char **argv)
{
    // load_lidar_config espera que *lidar_config já aponte para memória alocada.
    // Se passarmos nullptr, param_install escreve em ponteiro nulo → segfault.
    carmen_lidar_config *cfg = static_cast<carmen_lidar_config *>(
        calloc(1, sizeof(carmen_lidar_config)));
    if (!cfg) {
        RCLCPP_ERROR(get_logger(), "calloc falhou para carmen_lidar_config.");
        return;
    }

    // Permite que parâmetros ausentes no carmen.ini não matem o processo.
    // Sem isso, carmen_param_install_params chama carmen_die() / exit()
    // se qualquer campo obrigatório (flag=0) estiver faltando no .ini.
    carmen_param_allow_unfound_variables(1);
    load_lidar_config(argc, argv, lidar_sensor_id_, &cfg);
    carmen_param_allow_unfound_variables(0);  // restaura comportamento estrito

    // Sanity-check: range_division_factor=0 indica que o bloco [lidarN]
    // não existe no carmen.ini — usa defaults e avisa.
    if (cfg->range_division_factor <= 0 || cfg->shot_size <= 0) {
        RCLCPP_WARN(get_logger(),
            "Bloco [lidar%d] nao encontrado ou incompleto no carmen.ini. "
            "Usando defaults (range_div=%.0f, max_range=%.1f, shot_size=%d).",
            lidar_sensor_id_,
            lidar_range_division_factor_,
            lidar_max_range_,
            lidar_shot_size_);
        free(cfg);
        return;
    }

    lidar_range_division_factor_ = static_cast<double>(cfg->range_division_factor);
    lidar_max_range_             = cfg->max_range;
    lidar_shot_size_             = cfg->shot_size;

    lidar_x_     = cfg->pose.position.x;
    lidar_y_     = cfg->pose.position.y;
    lidar_z_     = cfg->pose.position.z;
    lidar_roll_  = cfg->pose.orientation.roll;
    lidar_pitch_ = cfg->pose.orientation.pitch;
    lidar_yaw_   = cfg->pose.orientation.yaw;

    max_laser_range_ = lidar_max_range_;
    lidar_config_loaded_ = true;

    RCLCPP_INFO(get_logger(),
        "lidar_config sensor %d: shot_size=%d, range_div=%.0f, "
        "max_range=%.1fm, pose=(%.3f,%.3f,%.3f) rpy=(%.3f,%.3f,%.3f)",
        lidar_sensor_id_, lidar_shot_size_,
        lidar_range_division_factor_, lidar_max_range_,
        lidar_x_, lidar_y_, lidar_z_,
        lidar_roll_, lidar_pitch_, lidar_yaw_);

    if (cfg->vertical_angles) free(cfg->vertical_angles);
    if (cfg->ray_order)       free(cfg->ray_order);
    free(cfg);
}

void IpcBridgeNode::start_ipc_thread()
{
    std::string ipc_host = get_parameter("ipc_host").as_string();

    ipc_thread_ = std::thread([this, ipc_host]() {
        // carmen_ipc_initialize reconhece -central_host (não -ipc_host)
        // para escolher onde conectar ao carmen_central.
        std::string prog        = "carmen_ipc_bridge";
        std::string flag        = "-central_host";
        std::vector<char*> argv_vec = {
            const_cast<char*>(prog.c_str()),
            const_cast<char*>(flag.c_str()),
            const_cast<char*>(ipc_host.c_str())
        };
        int argc = static_cast<int>(argv_vec.size());

        carmen_ipc_initialize(argc, argv_vec.data());

        // ── Carrega parâmetros do sensor do carmen.ini ────────────────────
        // Deve ser chamado APÓS carmen_ipc_initialize (pré-requisito do param).
        // Re-emite o TF estático com a pose real do sensor.
        load_carmen_lidar_config(argc, argv_vec.data());
        if (lidar_config_loaded_) {
            // Re-emite TF agora que temos a pose real
            setup_tf_from_carmen();
        }

        // Subscriptions IPC — apenas leitura, sem publish de volta
        carmen_fused_odometry_subscribe_fused_odometry_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(fused_odometry_handler),
            CARMEN_SUBSCRIBE_LATEST);

        carmen_velodyne_subscribe_partial_scan_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(velodyne_partial_scan_handler),
            CARMEN_SUBSCRIBE_LATEST);

        carmen_xsens_subscribe_xsens_global_quat_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(xsens_handler),
            CARMEN_SUBSCRIBE_LATEST);

        // Usa sensor_id lido do ROS2 param (default 5, configurável)
        carmen_velodyne_subscribe_variable_scan_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(velodyne_variable_scan_handler),
            CARMEN_SUBSCRIBE_LATEST, lidar_sensor_id_);

        RCLCPP_INFO(g_logger,
            "IPC dispatch loop iniciado (host=%s, lidar_sensor_id=%d). "
            "Subscriptions: fused_odometry, velodyne_partial_scan, velodyne_variable_scan.",
            ipc_host.c_str(), lidar_sensor_id_);

        while (running_) {
            IPC_listenWait(50 /* ms */);
        }

        IPC_disconnect();
        RCLCPP_INFO(g_logger, "IPC dispatch loop encerrado.");
    });
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callbacks IPC (chamados pela thread IPC, enfileiram para a thread ROS2)
// ─────────────────────────────────────────────────────────────────────────────

void IpcBridgeNode::on_velodyne_scan(VelodyneFrame frame)
{
    scan_queue_.push(std::move(frame));
}

void IpcBridgeNode::on_odometry(const OdometryFrame &frame)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_odom_      = frame;
        odom_initialized_ = true;
    }
    odom_queue_.push(frame);
}

void IpcBridgeNode::on_imu(const ImuFrame &frame)
{
    imu_queue_.push(frame);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Timers (drenam filas e publicam no ROS2)
// ─────────────────────────────────────────────────────────────────────────────

void IpcBridgeNode::timer_publish_scan()
{
    VelodyneFrame latest;
    
    // Puxa o último frame em O(1) e descarta o lixo acumulado
    if (!scan_queue_.pop_latest(latest)) return;

    if (publish_pointcloud_ && cloud_pub_) {
        auto msg = velodyne_to_pointcloud2(latest);
        cloud_pub_->publish(std::move(msg));
    }
    if (publish_laserscan_ && scan_pub_) {
        auto msg = velodyne_to_laserscan(latest);
        scan_pub_->publish(std::move(msg));
    }
}

void IpcBridgeNode::timer_publish_odom()
{
    OdometryFrame frame;
    while (odom_queue_.pop(frame)) {
        auto msg = odometry_to_msg(frame);
        odom_pub_->publish(msg);
        publish_tf_odom_base(frame.x, frame.y, frame.theta, now());
    }
}

void IpcBridgeNode::timer_publish_imu()
{
    if (!publish_imu_ || !imu_pub_) return;
    ImuFrame frame;
    while (imu_queue_.pop(frame)) {
        auto msg = imu_to_msg(frame);
        imu_pub_->publish(msg);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Conversão: VelodyneFrame → PointCloud2
//  Formato: x, y, z, intensity, ring  (compatível com LIO-SAM)
// ─────────────────────────────────────────────────────────────────────────────

sensor_msgs::msg::PointCloud2
IpcBridgeNode::velodyne_to_pointcloud2(const VelodyneFrame &f)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp    = now();
    cloud.header.frame_id = laser_frame_id_;
    cloud.height          = 1;
    cloud.is_bigendian    = false;

    // Campos: x, y, z, intensity, ring, time
    // O campo 'time' é obrigatório para o deskew do LIO-SAM.
    // Sem ele o imageProjection desativa o deskew e o sistema deriva.
    cloud.fields.resize(6);
    cloud.fields[0].name = "x";         cloud.fields[0].offset = 0;  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[0].count = 1;
    cloud.fields[1].name = "y";         cloud.fields[1].offset = 4;  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[1].count = 1;
    cloud.fields[2].name = "z";         cloud.fields[2].offset = 8;  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[2].count = 1;
    cloud.fields[3].name = "intensity"; cloud.fields[3].offset = 12; cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[3].count = 1;
    cloud.fields[4].name = "ring";      cloud.fields[4].offset = 16; cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;  cloud.fields[4].count = 1;
    cloud.fields[5].name = "time";      cloud.fields[5].offset = 20; cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[5].count = 1;

    // point_step = 32 bytes (alinhamento exigido pelo LIO-SAM/PCL)
    // layout: x(4) y(4) z(4) intensity(4) ring(2) pad(2) time(4) pad(8) = 32
    cloud.point_step = 32;

    // Filtra pontos inválidos antes de alocar — LIO-SAM exige is_dense=true (sem NaN)
    std::vector<const VelodyneFrame::Point3D *> valid;
    valid.reserve(f.points.size());
    for (const auto &pt : f.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            valid.push_back(&pt);
    }

    cloud.width    = static_cast<uint32_t>(valid.size());
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.is_dense = true;  // garantido: filtramos todos os NaN acima

    try {
        cloud.data.resize(cloud.row_step, 0);
    } catch (const std::exception &) {
        RCLCPP_ERROR(g_logger, "bad_alloc: %zu pontos", valid.size());
        return sensor_msgs::msg::PointCloud2{};
    }

    // Tempo relativo por ponto: distribui linearmente de 0 a ~0.1s (1 revolução)
    // O LIO-SAM usa esse campo para compensar o movimento durante o scan (deskew).
    const float scan_period = 0.1f;  // 10 Hz
    const float inv_total   = valid.empty() ? 0.0f
                              : scan_period / static_cast<float>(valid.size());

    uint8_t *ptr = cloud.data.data();
    for (size_t idx = 0; idx < valid.size(); ++idx) {
        const auto &pt = *valid[idx];
        float t = static_cast<float>(idx) * inv_total;
        std::memcpy(ptr + 0,  &pt.x,        4);
        std::memcpy(ptr + 4,  &pt.y,        4);
        std::memcpy(ptr + 8,  &pt.z,        4);
        std::memcpy(ptr + 12, &pt.intensity, 4);
        std::memcpy(ptr + 16, &pt.ring,      2);
        // bytes 18-19: padding (já zerado pelo resize)
        std::memcpy(ptr + 20, &t,            4);
        // bytes 24-31: padding
        ptr += cloud.point_step;
    }

    return cloud;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Conversão: VelodyneFrame → LaserScan (projeção 2D)
// ─────────────────────────────────────────────────────────────────────────────

sensor_msgs::msg::LaserScan
IpcBridgeNode::velodyne_to_laserscan(const VelodyneFrame &f)
{
    sensor_msgs::msg::LaserScan scan;
    int64_t ts_ns = static_cast<int64_t>(f.timestamp * 1e9);
    scan.header.stamp    = rclcpp::Time(ts_ns);
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
        float r = f.ranges[i];
        float a = f.angles[i];
        if (r < 0.1f || r > scan.range_max) continue;
        if (a < laser_min_angle_ || a >= laser_max_angle_) continue;
        int bin = static_cast<int>((a - laser_min_angle_) / bin_sz);
        if (bin < 0 || bin >= num_bins) continue;
        if (r < scan.ranges[bin]) scan.ranges[bin] = r;
    }

    return scan;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Conversão: OdometryFrame → nav_msgs/Odometry
// ─────────────────────────────────────────────────────────────────────────────

nav_msgs::msg::Odometry
IpcBridgeNode::odometry_to_msg(const OdometryFrame &f)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = now();
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id  = base_frame_id_;

    msg.pose.pose.position.x = f.x;
    msg.pose.pose.position.y = f.y;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, f.theta);
    msg.pose.pose.orientation = tf2::toMsg(q);

    // Twist: velocidade linear e angular (aproximação Ackermann)
    msg.twist.twist.linear.x  = f.v;
    msg.twist.twist.angular.z = (wheelbase_ > 0.0)
                                 ? f.v * std::tan(f.phi) / wheelbase_
                                 : 0.0;

    // Covariâncias
    msg.pose.covariance[0]   = 0.01;   // xx
    msg.pose.covariance[7]   = 0.01;   // yy
    msg.pose.covariance[35]  = 0.005;  // yaw-yaw
    msg.twist.covariance[0]  = 0.01;
    msg.twist.covariance[35] = 0.005;

    return msg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Conversão: ImuFrame → sensor_msgs/Imu
// ─────────────────────────────────────────────────────────────────────────────

sensor_msgs::msg::Imu
IpcBridgeNode::imu_to_msg(const ImuFrame &f)
{
    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = now();
    msg.header.frame_id = imu_frame_id_;

    msg.linear_acceleration.x = f.ax;
    msg.linear_acceleration.y = f.ay;
    msg.linear_acceleration.z = f.az;

    msg.angular_velocity.x = f.gx;
    msg.angular_velocity.y = f.gy;
    msg.angular_velocity.z = f.gz;

    if (f.has_orientation) {
        msg.orientation.x = f.qx;
        msg.orientation.y = f.qy;
        msg.orientation.z = f.qz;
        msg.orientation.w = f.qw;
        msg.orientation_covariance[0] = 0.01;
        msg.orientation_covariance[4] = 0.01;
        msg.orientation_covariance[8] = 0.01;
    } else {
        msg.orientation_covariance[0] = -1.0; // indica: sem orientação
    }

    msg.angular_velocity_covariance[0] = 0.01;
    msg.angular_velocity_covariance[4] = 0.01;
    msg.angular_velocity_covariance[8] = 0.01;

    msg.linear_acceleration_covariance[0] = 0.1;
    msg.linear_acceleration_covariance[4] = 0.1;
    msg.linear_acceleration_covariance[8] = 0.1;

    return msg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  TF dinâmico: odom → base_link
// ─────────────────────────────────────────────────────────────────────────────

void IpcBridgeNode::publish_tf_odom_base(double x, double y, double theta,
                                          rclcpp::Time stamp)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = stamp;
    t.header.frame_id = odom_frame_id_;
    t.child_frame_id  = base_frame_id_;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
}

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IpcBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}