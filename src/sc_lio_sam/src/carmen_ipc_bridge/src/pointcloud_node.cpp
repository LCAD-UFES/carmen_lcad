/*
 * pointcloud_node.cpp  (ROS1 / Noetic)
 *
 * Nó dedicado ao caminho do LiDAR, ponta a ponta:
 *   1. Lê o velodyne (partial/variable) DIRETO do CARMEN IPC.
 *   2. Converte bruto -> 3D num único passo e publica /points_raw
 *      (sensor_msgs/PointCloud2 pronta para o SC-LIO-SAM).
 *   3. Assina /points_raw, remove os pontos que caem sobre o modelo de
 *      colisão do veículo e publica /points_raw_no_vehicle.
 *   4. Publica o TF estático base_link -> velodyne, base_link -> sensor_board_1
 *      e os markers do modelo de colisão (/collision_model_markers).
 *
 * Antes essa cadeia era dividida entre o ipc_bridge_node (leitura IPC +
 * /velodyne_raw_ipc + filtro de colisão) e este nó (só a conversão 3D).
 * Agora tudo do LiDAR vive aqui; o ipc_bridge_node não toca mais na nuvem.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/TimeReference.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <memory>
#include <algorithm>

#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/global.h>

#include "carmen_ipc_bridge/carmen_time.hpp"

// Ângulos de elevação Hesai XT-32 (apenas FALLBACK) — usado só quando o config
// do LiDAR não pôde ser carregado, para não regredir o cenário XT-32.
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

// ─── Frame bruto do velodyne (interno) ───────────────────────────────────────
class PointcloudNode;
static PointcloudNode *g_pc_node = nullptr;

struct VelodyneFrame {
    double  timestamp    = 0.0;   // timestamp CARMEN original (FIM da varredura)
    int64_t ros_stamp_ns = 0;     // stamp ROS publicado (INICIO da varredura)
    int     source       = 0;     // 0 = partial_scan (velodyne), 1 = variable_scan (lidar N)
    int     num_shots    = 0;
    int     shot_size    = 0;
    double  range_div    = 0.0;
    double  max_range    = 0.0;
    struct RawPoint { uint32_t distance; uint8_t intensity; uint8_t ring; };
    std::vector<RawPoint> raw_points;  // num_shots × shot_size
    std::vector<int>      shot_sizes;  // tamanho de cada shot (variable_scan)
    std::vector<float>    shot_cos_az; // cos do azimute real (por shot)
    std::vector<float>    shot_sin_az; // sin do azimute real (por shot)
};

class PointcloudNode
{
public:
    PointcloudNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh)
    {
        g_pc_node = this;

        pnh_.param<std::string>("laser_frame_id", laser_frame_id_, "velodyne");
        pnh_.param<std::string>("base_frame_id",  base_frame_id_,  "base_link");
        pnh_.param<std::string>("sensorboard_frame_id",  sensorboard_frame_id_,  "sensor_board_1");
        pnh_.param<std::string>("ground_frame_id",  ground_frame_id_,  "ground_link"); 
        pnh_.param<std::string>("map_ground_frame_id_",  map_ground_frame_id_,  "map_ground_link");
        pnh_.param<std::string>("map_base_link_frame_id", map_base_link_frame_id_, "map_base_link");
        pnh_.param<int>("lidar_sensor_id", lidar_sensor_id_, 5);
        pnh_.param<std::string>("ipc_host", ipc_host_, "localhost");

        // Modelo 3D do robo (mesma tecnica do viewer_3D: le carmodel_* do
        // param_daemon e desenha o .obj preso ao frame do veiculo).
        pnh_.param<bool>("publish_car_model", publish_car_model_, true);
        pnh_.param<std::string>("car_model_file", car_model_file_override_, "");
        pnh_.param<std::string>("car_model_frame_id", car_model_frame_id_, base_frame_id_);

        // LUT de elevação com fallback XT-32 (sobrescrita ao carregar o config).
        lidar_num_rays_ = 32;
        for (int i = 0; i < MAX_RAYS; ++i) {
            float el_deg = (i < 32) ? HESAI_XT32_ELEVATIONS_DEG[i] : 0.0f;
            float el_rad = el_deg * static_cast<float>(M_PI) / 180.0f;
            ring_cos_el_[i] = std::cos(el_rad);
            ring_sin_el_[i] = std::sin(el_rad);
        }

        // Offset de azimute por canal: identidade até o config carregar (e
        // permanece identidade pra sempre, se o LiDAR não tiver essa correção).
        for (int i = 0; i < MAX_RAYS; ++i) {
            ring_cos_az_off_[i] = 1.0f;
            ring_sin_az_off_[i] = 0.0f;
        }

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

        clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);
        pub_          = nh_.advertise<sensor_msgs::PointCloud2>("/points_raw", 5);
        points_no_vehicle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_raw_no_vehicle", 5);
        // Ponte de tempo CARMEN <-> ROS por scan. O stamp ROS que publicamos e' o
        // INICIO da varredura (o LIO-SAM precisa assim, pro deskew), mas o CARMEN
        // carimba o FIM -- e o graphslam_publish casa timestamps com tolerancia de
        // 1 us. Sem essa ponte nao da' pra reconstruir o valor original com
        // precisao suficiente do lado do SC-LIO-SAM.
        //
        // FILA DE 400, nao 20. Do outro lado o mapOptmization assina com fila 400 e roda
        // em ros::spin() SINGLE-THREAD: enquanto o ICP mastiga o mapa (26 M de pontos no
        // ARGOS), o callback deste topico nao roda e as mensagens se acumulam. Com fila
        // 20 -- 1 s a 20 Hz -- elas eram DESCARTADAS aqui no publicador antes de chegar.
        //
        // O estrago era total e silencioso: o lookupCarmenStamp() do SC-LIO-SAM faz busca
        // EXATA por nanossegundo, entao stamp perdido = scan sem timestamp CARMEN = pose
        // descartada. Medido no ARGOS: a nuvem chegava ao mapOptimization 2 s ANTES do
        // TimeReference correspondente, 516 scans descartados, poses_opt.dat com 0 bytes.
        // Na IARA nao aparecia porque o mapa e' menor e o ICP acompanha.
        //
        // 400 casa com a fila do assinante e cobre ~20 s de atraso a 20 Hz. TimeReference
        // tem ~50 bytes: o custo de memoria e' desprezivel perto de perder a pose.
        scan_time_ref_pub_ = nh_.advertise<sensor_msgs::TimeReference>("/carmen/scan_time_reference", 400);
        collision_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/collision_model_markers", 1);
        // Latched: quem assinar depois (RViz reaberto) recebe o modelo na hora.
        car_model_pub_ = nh_.advertise<visualization_msgs::Marker>("/car_model_marker", 1, /*latch=*/true);
        car_model_trailers_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/car_model_trailer_markers", 1, /*latch=*/true);

        // WallTimer de proposito: nao depende de /clock, de playback nem de scan
        // do LiDAR -- o modelo sobe junto com o no'. O republish de 1 Hz cobre
        // RViz reiniciado e tempo simulado andando pra tras.
        if (publish_car_model_) {
            car_model_timer_ = nh_.createWallTimer(
                ros::WallDuration(1.0),
                [this](const ros::WallTimerEvent &) {
                    publish_car_model_marker();
                    publish_trailer_model_markers();   // heartbeat: RViz reaberto
                });
        }

        // Assina a própria /points_raw para o filtro de colisão (conforme o
        // desenho: o mesmo nó publica /points_raw e a consome para filtrar).
        points_raw_sub_ = nh_.subscribe("/points_raw", 5, &PointcloudNode::on_points_raw, this);

        start_ipc_thread();

        ROS_INFO("pointcloud_node iniciado (host=%s, lidar_sensor_id=%d).",
                 ipc_host_.c_str(), lidar_sensor_id_);
    }

    ~PointcloudNode()
    {
        running_ = false;
        if (ipc_thread_.joinable()) ipc_thread_.join();
    }

    // ─── Callbacks IPC (chamados pelas funções estáticas) ────────────────────
    void on_velodyne_scan(VelodyneFrame frame)
    {
        if (frame.raw_points.empty()) return;

        const ros::Time stamp = carmen_bridge::carmen_ns_to_ros_time(frame.ros_stamp_ns);
        carmen_bridge::publish_clock(clock_pub_, stamp);

        if (!static_tfs_sent_.exchange(true))
            publish_static_transforms(stamp);

        last_scan_stamp_ = stamp;

        // Publica a ponte de tempo ANTES da nuvem: quem consome as duas garante
        // que o stamp CARMEN ja' esta' na tabela quando a nuvem chegar.
        {
            sensor_msgs::TimeReference tref;
            tref.header.stamp    = stamp;                 // chave: stamp ROS da nuvem
            tref.header.frame_id = laser_frame_id_;
            tref.time_ref.fromSec(frame.timestamp);       // timestamp CARMEN (fim da varredura)
            tref.source = (frame.source == 1)
                        ? ("variable_scan:" + std::to_string(lidar_sensor_id_))
                        : std::string("partial_scan");
            scan_time_ref_pub_.publish(tref);
        }

        pub_.publish(velodyneframe_to_points_raw(frame));

        static ros::Time last_marker_pub(0);
        const ros::Time ref_now = carmen_bridge::use_sim_time() ? stamp : ros::Time::now();

        // FIX: sob use_sim_time o tempo VOLTA PRA TRAS toda vez que o playback e'
        // reiniciado, e este no' e' de vida longa (roda fora do roslaunch, sobrevive
        // a varias sessoes do lio_sam) -- entao last_marker_pub guardava o stamp
        // ALTO da sessao anterior. Com ref_now < last_marker_pub a diferenca fica
        // negativa e nunca mais alcanca os 0.05 s: os markers de colisao paravam de
        // ser publicados de vez, ate o playback reultrapassar aquele stamp antigo.
        // (A nuvem em /points_raw nao tem essa guarda, por isso so' os markers
        // sumiam.) Detecta o retrocesso e rearma.
        if (ref_now < last_marker_pub) {
            ROS_WARN("Tempo andou pra tras (%.3f < %.3f) -- playback reiniciado? "
                     "Rearmando a publicacao dos markers de colisao.",
                     ref_now.toSec(), last_marker_pub.toSec());
            last_marker_pub = ros::Time(0);
        }

        if ((ref_now - last_marker_pub).toSec() >= 0.05) {
            publish_collision_markers(stamp);
            last_marker_pub = ref_now;
        }
    }

    void on_trailer_pose(const double *betas, int n, bool engaged)
    {
        for (int i = 0; i < MAX_NUM_TRAILERS; ++i)
            trailer_betas_[i].store(i < n ? betas[i] : 0.0);
        trailer_beta_.store(n > 0 ? betas[0] : 0.0);
        trailer_engaged_.store(engaged);

        // Os meshes dos semi-reboques dependem do beta, entao acompanham o
        // globalpos (throttle pra nao inundar o RViz). O do cavalo e' fixo no
        // base_link e continua saindo so' no WallTimer.
        if (!trailer_models_.empty()) {
            const ros::WallTime now = ros::WallTime::now();
            if ((now - last_trailer_model_pub_).toSec() >= 0.05) {
                publish_trailer_model_markers();
                last_trailer_model_pub_ = now;
            }
        }
    }

    // Público para os handlers IPC estáticos.
    int    lidar_sensor_id_{5};
    double lidar_range_division_factor_{0.0};
    double lidar_max_range_{0.0};
    double lidar_time_between_shots_{0.0};
    int    lidar_shot_size_{32};
    double lidar_x_relative_to_sb1_{0.0}, lidar_y_relative_to_sb1_{0.0}, lidar_z_relative_to_sb1_{1.8};
    double lidar_roll_relative_to_sb1_{0.0}, lidar_pitch_relative_to_sb1_{0.0}, lidar_yaw_relative_to_sb1_{0.0};
    std::atomic<bool> lidar_config_loaded_{false};

    static constexpr int MAX_RAYS = 128;
    int   lidar_num_rays_{32};
    float ring_cos_el_[MAX_RAYS];
    float ring_sin_el_[MAX_RAYS];

    // Offset de azimute por canal (correção horizontal), em cos/sin.
    // Só é usado pelos LiDARs cujo driver publica essa correção no
    // param_daemon (ex.: mecânicos multi-linha tipo OT128/Pandar, onde cada
    // laser tem um pequeno desvio de azimute dentro do mesmo shot).
    // Default = identidade (sem correção) — LiDARs que não têm essa info
    // (campo nulo ou tudo 0 porque a chave não existe no param_daemon)
    // continuam se comportando exatamente como antes.
    float ring_cos_az_off_[MAX_RAYS];
    float ring_sin_az_off_[MAX_RAYS];

private:
    // ─── Inicialização / IPC ─────────────────────────────────────────────────
    void start_ipc_thread();
    void load_carmen_lidar_config(int argc, char **argv);
    void setup_collision_filter();
    void setup_car_model();
    void publish_car_model_marker();
    void setup_trailer_models();
    void publish_trailer_model_markers();
    void publish_static_transforms(const ros::Time &stamp);

    // ─── Conversão bruto -> 3D ───────────────────────────────────────────────
    sensor_msgs::PointCloud2 velodyneframe_to_points_raw(const VelodyneFrame &f);
    float update_scan_period(const ros::Time &stamp);

    // ─── Filtro de pontos sobre o veículo ────────────────────────────────────
    void on_points_raw(const sensor_msgs::PointCloud2::ConstPtr &msg);
    bool point_on_vehicle(float x, float y, float z) const;
    bool ensure_collision_heights();
    void publish_collision_markers(const ros::Time &stamp);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher  clock_pub_;
    ros::Publisher  pub_;
    ros::Publisher  points_no_vehicle_pub_;
    ros::Publisher  scan_time_ref_pub_;
    ros::Publisher  collision_markers_pub_;
    ros::Publisher  car_model_pub_;
    ros::Subscriber points_raw_sub_;
    ros::WallTimer  car_model_timer_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::thread       ipc_thread_;
    std::atomic<bool> running_{true};

    std::string laser_frame_id_;
    std::string base_frame_id_;
    std::string sensorboard_frame_id_; 
    std::string ground_frame_id_;
    std::string map_ground_frame_id_;
    std::string map_base_link_frame_id_;
    std::string ipc_host_;

    // Estimativa dinâmica do período de scan (para o campo "time" por ponto).
    double last_stamp_sec_ = -1.0;
    float  scan_period_ema_ = 0.055f;
    ros::Time last_scan_stamp_{0};

    // ─── Filtro de colisão (modelo 3D do veículo) ────────────────────────────
    carmen_collision_config_t *collision_cfg_{nullptr};
    tf2::Transform T_ground_lidar_;
    tf2::Transform T_ground_robot_;
    tf2::Transform T_robot_lidar;
    std::atomic<bool> collision_ready_{false};
    double robot_wheel_radius_{0.0};
    double sb1_x_{0.0}, sb1_y_{0.0}, sb1_z_{0.0};
    double sb1_roll_{0.0}, sb1_pitch_{0.0}, sb1_yaw_{0.0};
    std::atomic<double> trailer_beta_{0.0};
    std::atomic<double> trailer_betas_[MAX_NUM_TRAILERS];
    std::atomic<bool>   trailer_engaged_{false};
    double robot_collision_height_{0.0};
    double semi_trailer_collision_height_{0.0};
    double collision_safety_margin_{0.0};
    std::atomic<bool> collision_heights_loaded_{false};
    std::atomic<bool> static_tfs_sent_{false};

    // ─── Modelo 3D do robô (carmodel_* do param_daemon, igual ao viewer_3D) ──
    bool        publish_car_model_{true};
    std::string car_model_file_override_;
    std::string car_model_frame_id_;
    std::string car_model_uri_;              // file:///... pronto pro RViz
    visualization_msgs::Marker car_model_marker_;
    std::atomic<bool> car_model_ready_{false};

    // Um por semi-reboque: tudo o que nao depende do beta ja' vem mastigado
    // daqui; so' a articulacao e' recalculada a cada globalpos.
    struct TrailerModel {
        std::string  uri;
        tf2::Vector3 scale;        // marker.scale (unitize * size/2)
        tf2::Vector3 bbox_center;  // centro da bbox, em unidades cruas do .obj
        double off_x{0.0}, off_y{0.0}, off_z{0.0};   // semi_trailer_model{N}_{x,y,z}
        double roll{0.0}, pitch{0.0}, yaw{0.0};
        double d{0.0}, M{0.0};                       // semi_trailer{N}_{d,M}
    };
    std::vector<TrailerModel> trailer_models_;
    ros::Publisher car_model_trailers_pub_;
    ros::WallTime  last_trailer_model_pub_{ros::WallTime(0)};
};

// EMA do período real entre scans (só diagnóstico; não altera o stamp publicado).
static double g_scan_period_ema_s = 0.05;

// carmen_ts_end = timestamp CARMEN no FIM do scan; ROS/LIO-SAM esperam o INÍCIO.
static int64_t scan_start_stamp_ns(double carmen_ts_end, int num_shots)
{
    // EMA do período real (só para diagnóstico; não altera o stamp).
    static double last_carmen_ts = 0.0;
    if (last_carmen_ts > 0.0) {
        double delta_s = carmen_ts_end - last_carmen_ts;
        if (delta_s > 0.005 && delta_s < 0.5)
            g_scan_period_ema_s = 0.9 * g_scan_period_ema_s + 0.1 * delta_s;
    }
    last_carmen_ts = carmen_ts_end;

    const double scan_duration =
        g_pc_node->lidar_time_between_shots_ * num_shots;
    const double scan_start = carmen_ts_end - scan_duration;
    ROS_DEBUG_THROTTLE(5.0,
        "scan stamp: end=%.3f duration=%.4f start=%.3f ema=%.4f",
        carmen_ts_end, scan_duration, scan_start, g_scan_period_ema_s);
    return carmen_bridge::carmen_to_ros_ns(scan_start);
}

// ─── Handlers IPC ─────────────────────────────────────────────────────────────
static void velodyne_partial_scan_handler(carmen_velodyne_partial_scan_message *msg)
{
    if (!g_pc_node || !msg) return;
    const int num_shots = msg->number_of_32_laser_shots;
    if (num_shots <= 0 || num_shots > 10000) return;

    VelodyneFrame frame;
    frame.timestamp = msg->timestamp;
    frame.num_shots = num_shots;
    frame.shot_size = 32;
    frame.range_div = g_pc_node->lidar_range_division_factor_;
    frame.max_range = g_pc_node->lidar_max_range_;

    frame.raw_points.reserve(num_shots * 32);
    frame.shot_sizes.assign(num_shots, 32);
    frame.shot_cos_az.resize(num_shots);
    frame.shot_sin_az.resize(num_shots);

    for (int i = 0; i < num_shots; ++i) {
        // Ângulo real do sensor (horário, graus) → anti-horário ROS (REP-103).
        float az_deg = msg->partial_scan[i].angle;
        float az_rad = -az_deg * static_cast<float>(M_PI) / 180.0f;
        frame.shot_cos_az[i] = std::cos(az_rad);
        frame.shot_sin_az[i] = std::sin(az_rad);

        for (int j = 0; j < 32; ++j) {
            VelodyneFrame::RawPoint rp;
            rp.distance  = msg->partial_scan[i].distance[j];
            rp.intensity = 100;
            rp.ring      = static_cast<uint8_t>(j);
            frame.raw_points.push_back(rp);
        }
    }

    frame.source       = 0;   // partial_scan -> graphslam_publish -poses_from velodyne
    frame.ros_stamp_ns = scan_start_stamp_ns(frame.timestamp, num_shots);
    g_pc_node->on_velodyne_scan(std::move(frame));
}

static void velodyne_variable_scan_handler(carmen_velodyne_variable_scan_message *msg)
{
    if (!g_pc_node || !msg) return;
    const int num_shots = msg->number_of_shots;

    ROS_INFO_THROTTLE(1.0,
        "[DBG-SCAN-1] CARMEN->pointcloud: num_shots=%d  ts_carmen=%.3f",
        num_shots, msg->timestamp);

    if (num_shots <= 0 || num_shots > 10000) return;

    VelodyneFrame frame;
    frame.timestamp = msg->timestamp;
    frame.num_shots = num_shots;
    frame.range_div = g_pc_node->lidar_range_division_factor_;
    frame.max_range = g_pc_node->lidar_max_range_;

    try {
        // Pré-calcula cos/sin do azimute REAL (msg->partial_scan[i].angle, em
        // graus, sentido horário), convertido para anti-horário ROS. Não usa
        // azimute sintético por índice (causava drift quando num_shots variava).
        std::vector<float> cos_az(num_shots), sin_az(num_shots);
        for (int i = 0; i < num_shots; ++i) {
            float az_deg = msg->partial_scan[i].angle;
            float az_rad = -az_deg * static_cast<float>(M_PI) / 180.0f;
            cos_az[i] = std::cos(az_rad);
            sin_az[i] = std::sin(az_rad);
        }

        const int rays_hint = (g_pc_node->lidar_shot_size_ > 0)
                                  ? g_pc_node->lidar_shot_size_ : 32;
        frame.shot_sizes.reserve(num_shots);
        frame.raw_points.reserve(num_shots * rays_hint);
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

            for (int j = 0; j < shot_size; ++j) {
                VelodyneFrame::RawPoint rp;
                rp.distance  = msg->partial_scan[i].distance[j];
                rp.intensity = (msg->partial_scan[i].intensity)
                               ? static_cast<uint8_t>(msg->partial_scan[i].intensity[j])
                               : 0;
                rp.ring = static_cast<uint8_t>(j < 128 ? j : 127);
                frame.raw_points.push_back(rp);
            }
        }

        ROS_INFO_THROTTLE(1.0,
            "[DBG-SCAN-2] raw_pts=%zu  (shots=%d)",
            frame.raw_points.size(), num_shots);

        frame.source       = 1;   // variable_scan -> graphslam_publish -poses_from lidar
        frame.ros_stamp_ns = scan_start_stamp_ns(msg->timestamp, num_shots);
        g_pc_node->on_velodyne_scan(std::move(frame));

    } catch (const std::exception &e) {
        ROS_ERROR("EXCEÇÃO no variable_scan_handler: %s", e.what());
    }
}

static void localize_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    if (!g_pc_node || !msg) return;
    // Ângulo de articulação relativo de cada semi-trailer: o do primeiro é
    // relativo ao caminhão, os demais em relação ao reboque da frente.
    double betas[MAX_NUM_TRAILERS];
    double prev_theta = msg->globalpos.theta;
    for (int i = 0; i < MAX_NUM_TRAILERS; ++i) {
        betas[i] = convert_theta1_to_beta(prev_theta, msg->trailer_theta[i]);
        prev_theta = msg->trailer_theta[i];
    }
    g_pc_node->on_trailer_pose(betas, MAX_NUM_TRAILERS, msg->semi_trailer_engaged != 0);
}

// ─── Carregamento do config do LiDAR (range/shot/pose + LUT de elevação) ─────
void PointcloudNode::load_carmen_lidar_config(int argc, char **argv)
{
    carmen_lidar_config *cfg = static_cast<carmen_lidar_config *>(
        calloc(1, sizeof(carmen_lidar_config)));
    if (!cfg) return;
    carmen_param_allow_unfound_variables(1);
    load_lidar_config(argc, argv, lidar_sensor_id_, &cfg);
    carmen_param_allow_unfound_variables(0);

    if (cfg->range_division_factor <= 0 || cfg->shot_size <= 0 || cfg->max_range <= 0) {
        ROS_FATAL(
            "lidar_config sensor %d invalido: range_division_factor=%d, shot_size=%d, max_range=%.1f. "
            "Sem default — encerrando.",
            lidar_sensor_id_, cfg->range_division_factor, cfg->shot_size, cfg->max_range);
        free(cfg);
        ros::shutdown();
        return;
    }

    lidar_range_division_factor_ = cfg->range_division_factor;
    lidar_max_range_ = cfg->max_range;
    lidar_time_between_shots_ = cfg->time_between_shots;
    lidar_shot_size_ = cfg->shot_size;
    lidar_x_relative_to_sb1_    = cfg->pose.position.x;
    lidar_y_relative_to_sb1_    = cfg->pose.position.y;
    lidar_z_relative_to_sb1_    = cfg->pose.position.z;
    lidar_roll_relative_to_sb1_ = cfg->pose.orientation.roll;
    lidar_pitch_relative_to_sb1_= cfg->pose.orientation.pitch;
    lidar_yaw_relative_to_sb1_  = cfg->pose.orientation.yaw;

    // Preenche a LUT de elevação (cos/sin) indexada pelo canal físico do LiDAR:
    // vertical_angles[k] é a elevação do canal ray_order[k].
    int num_rays = cfg->shot_size;
    if (num_rays > MAX_RAYS) {
        //ROS_WARN("lidar_config sensor %d: shot_size=%d > MAX_RAYS=%d, truncando.",
        //         lidar_sensor_id_, num_rays, MAX_RAYS);
        num_rays = MAX_RAYS;
    }
    if (num_rays > 0 && cfg->vertical_angles && cfg->ray_order) {
        for (int c = 0; c < MAX_RAYS; ++c) {
            ring_cos_el_[c] = 1.0f;
            ring_sin_el_[c] = 0.0f;
        }
        for (int k = 0; k < num_rays; ++k) {
            int channel = cfg->ray_order[k];
            if (channel < 0 || channel >= MAX_RAYS) continue;
            float el_rad = static_cast<float>(cfg->vertical_angles[k])
                           * static_cast<float>(M_PI) / 180.0f;
            ring_cos_el_[channel] = std::cos(el_rad);
            ring_sin_el_[channel] = std::sin(el_rad);
        }
        lidar_num_rays_ = num_rays;
    }

    // Offset de azimute por canal (correção horizontal). Nem todo LiDAR tem
    // isso — nesse caso cfg->horizontal_angles vem nulo, ou o driver publica
    // tudo 0 porque a chave não existe no param_daemon. Em ambos os casos o
    // offset fica em 0/identidade (setado no construtor) e o ponto sai
    // exatamente como antes: nenhuma regressão pros LiDARs sem essa correção.
    //
    // LIGADO. O campo horizontal_angles_deltas foi acrescentado ao
    // carmen_lidar_config (velodyne_messages.h) e o load_lidar_config()
    // (velodyne_interface.cpp) passou a ler a chave OPCIONAL
    // lidar<N>_horizontal_angles do param_daemon.
    //
    // Quando a chave nao existe -- ou tem menos valores que shot_size -- o
    // load_lidar_config deixa o vetor todo em zero, e o has_non_zero_deltas
    // abaixo detecta isso: ring_cos_az_off_/ring_sin_az_off_ ficam na identidade
    // posta pelo construtor e o azimute sai do proprio indice do shot, como em
    // qualquer LiDAR sem correcao horizontal. Nenhuma regressao.
#define CARMEN_LIDAR_CONFIG_HAS_HORIZONTAL_DELTAS 1

#if CARMEN_LIDAR_CONFIG_HAS_HORIZONTAL_DELTAS
    if (num_rays > 0 && cfg->horizontal_angles_deltas && cfg->ray_order) {
        bool has_non_zero_deltas = false;
        for (int k = 0; k < num_rays; ++k) {
            int channel = cfg->ray_order[k];
            if (channel < 0 || channel >= MAX_RAYS) continue;

            float off_rad = -static_cast<float>(cfg->horizontal_angles_deltas[k])
                            * static_cast<float>(M_PI) / 180.0f;
            ring_cos_az_off_[channel] = std::cos(off_rad);
            ring_sin_az_off_[channel] = std::sin(off_rad);

            if (std::abs(cfg->horizontal_angles_deltas[k]) > 1e-6) {
                has_non_zero_deltas = true;
            }
        }

        if (has_non_zero_deltas) {
            ROS_INFO("\033[1;32m[LiDAR Config] Correção HORIZONTAL ativada! Modelo: %s. Desvios lidos com sucesso.\033[0m", cfg->model);
        } else {
            ROS_WARN("\033[1;33m[LiDAR Config] O parâmetro horizontal_angles foi lido, mas TODOS os ângulos estão ZERADOS!\033[0m");
        }
    } else {
        ROS_ERROR("\033[1;31m[LiDAR Config] ERRO: horizontal_angles NÃO está sendo lido pelo sistema para o modelo '%s'!\033[0m", 
                cfg->model ? cfg->model : "Desconhecido");
    }
#else
    ROS_INFO("[LiDAR Config] correcao HORIZONTAL de azimute desligada "
             "(carmen_lidar_config do CARMEN nao tem horizontal_angles_deltas); "
             "modelo '%s' segue com offset por canal = 0.",
             cfg->model ? cfg->model : "Desconhecido");
#endif

    lidar_config_loaded_ = true;
    ROS_INFO(
        "lidar_config sensor %d: shot_size=%d, num_rays=%d, range_div=%.0f, max_range=%.1fm",
        lidar_sensor_id_, lidar_shot_size_, lidar_num_rays_,
        lidar_range_division_factor_, lidar_max_range_);
    if (cfg->vertical_angles)   free(cfg->vertical_angles);
#if CARMEN_LIDAR_CONFIG_HAS_HORIZONTAL_DELTAS
    if (cfg->horizontal_angles_deltas) free(cfg->horizontal_angles_deltas);
#endif
    if (cfg->ray_order)         free(cfg->ray_order);
    free(cfg);

    // Prepara o filtro de pontos sobre o veículo (transformação + modelo).
    setup_collision_filter();

    // Modelo 3D do robô para o RViz (precisa do wheel_radius lido acima).
    if (publish_car_model_) setup_car_model();
}

// Lê a pose da sensor_board_1 e o raio das rodas do param_daemon, compõe a
// transformação ponto-do-LiDAR -> frame do caminhão (T_robot_lidar = T_robot_board
// * T_board_lidar) e obtém o modelo de colisão global.
void PointcloudNode::setup_collision_filter()
{
    carmen_param_allow_unfound_variables(1);
    carmen_param_t param_list[] = {
        {(char *) "sensor_board_1", (char *) "x",     CARMEN_PARAM_DOUBLE, &sb1_x_,     0, NULL},
        {(char *) "sensor_board_1", (char *) "y",     CARMEN_PARAM_DOUBLE, &sb1_y_,     0, NULL},
        {(char *) "sensor_board_1", (char *) "z",     CARMEN_PARAM_DOUBLE, &sb1_z_,     0, NULL},
        {(char *) "sensor_board_1", (char *) "roll",  CARMEN_PARAM_DOUBLE, &sb1_roll_,  0, NULL},
        {(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &sb1_pitch_, 0, NULL},
        {(char *) "sensor_board_1", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &sb1_yaw_,   0, NULL},
        {(char *) "robot",  (char *) "wheel_radius",  CARMEN_PARAM_DOUBLE, &robot_wheel_radius_, 0, NULL},
    };
    carmen_param_install_params(0, nullptr, param_list,
                               sizeof(param_list) / sizeof(param_list[0]));
    carmen_param_allow_unfound_variables(0);

    if (sb1_x_ == 0.0 && sb1_y_ == 0.0 && sb1_z_ == 0.0 &&
        sb1_roll_ == 0.0 && sb1_pitch_ == 0.0 && sb1_yaw_ == 0.0)
    {
        //ROS_WARN(
        //    "sensor_board_1 veio toda zerada do param_daemon -- ou a board "
        //    "esta mesmo na origem do /car, ou a chave nao foi encontrada.");
    }

    // T_robot_board: pose da sensor board relativa ao eixo traseiro do caminhão.
    tf2::Quaternion q_sb;
    q_sb.setRPY(sb1_roll_, sb1_pitch_, sb1_yaw_);
    tf2::Transform T_robot_board(q_sb, tf2::Vector3(sb1_x_, sb1_y_, sb1_z_));

    // T_board_lidar: pose do LiDAR relativa à sensor board (cfg->pose).
    tf2::Quaternion q_bl;
    q_bl.setRPY(lidar_roll_relative_to_sb1_, lidar_pitch_relative_to_sb1_, lidar_yaw_relative_to_sb1_);
    tf2::Transform T_board_lidar(q_bl, tf2::Vector3(lidar_x_relative_to_sb1_, lidar_y_relative_to_sb1_, lidar_z_relative_to_sb1_));

    T_robot_lidar = T_robot_board * T_board_lidar;

    // T_ground_robot: pose do eixo traseiro do caminhão relativo ao chao.
    tf2::Quaternion q_gnd;
    q_gnd.setRPY(0, 0, 0);
    T_ground_robot_.setOrigin(tf2::Vector3(0, 0, robot_wheel_radius_));
    T_ground_robot_.setRotation(q_gnd);

    T_ground_lidar_ = T_ground_robot_ * T_robot_board * T_board_lidar;

    collision_cfg_ = carmen_collision_detection_get_global_collision_config();
    if (!collision_cfg_) {
        ROS_ERROR("Nao foi possivel obter o modelo de colisao (collision_config). "
                  "Filtro de pontos sobre o veiculo ficara desativado (pass-through).");
        return;
    }

    collision_ready_ = true;
    ROS_INFO(
        "Filtro de colisao pronto: wheel_radius=%.3fm, board_1=(%.3f, %.3f, %.3f), "
        "n_markers_caminhao=%d, semi_trailer_type=%d",
        robot_wheel_radius_, sb1_x_, sb1_y_, sb1_z_,
        collision_cfg_->n_markers, collision_cfg_->semi_trailer_type);
}

// ─── Modelo 3D do robô ───────────────────────────────────────────────────────
//
// Porte da tecnica do viewer_3D (draw_car.cpp) para o RViz. La' o fluxo e':
//
//   createCarDrawer():  le carmodel_file_name/size_*/x/y/z/yaw do param_daemon,
//                       glmReadOBJ(arquivo)
//                       glmUnitize()       -> centra na origem e normaliza o
//                                             maior lado para 2.0
//                       glmScaleXYZ(sx/2, sy/2, sz/2)
//   draw_car():         glTranslatef(carmodel_x, carmodel_y, carmodel_z + wheel_radius)
//                       glRotatef(90, X) ; glRotatef(-yaw, Y) ; glmDraw()
//   draw_car_at_pose(): tudo isso dentro da pose do globalpos
//
// No RViz o equivalente e' um Marker MESH_RESOURCE preso ao frame do veiculo
// (o "draw_car_at_pose" sai de graca: quem move o frame e' a TF). Como o RViz
// nao tem glmUnitize, o fator dele entra no marker.scale, e o recentramento
// entra na posicao -- por isso o .obj e' lido aqui so' para medir a bounding
// box (as mesmas contas de glmUnitize).
static bool obj_bounding_box(const std::string &path, double bb_min[3], double bb_max[3])
{
    FILE *f = fopen(path.c_str(), "r");
    if (!f) return false;

    bool first = true;
    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        // So' os vertices de geometria ("v "), nunca "vt "/"vn ".
        if (line[0] != 'v' || (line[1] != ' ' && line[1] != '\t')) continue;
        double x, y, z;
        if (sscanf(line + 1, "%lf %lf %lf", &x, &y, &z) != 3) continue;
        const double v[3] = {x, y, z};
        for (int i = 0; i < 3; ++i) {
            if (first || v[i] < bb_min[i]) bb_min[i] = v[i];
            if (first || v[i] > bb_max[i]) bb_max[i] = v[i];
        }
        first = false;
    }
    fclose(f);
    return !first;
}

// Resolve o caminho do .obj e reproduz glmUnitize() + glmScaleXYZ(size/2):
// devolve a URI pro RViz, a escala do marker e o centro da bounding box (em
// unidades cruas do arquivo, que e' o que o unitize usa pra recentrar).
static bool prepare_obj_model(const std::string &raw_file,
                              double size_x, double size_y, double size_z,
                              std::string &uri, tf2::Vector3 &scale,
                              tf2::Vector3 &center, tf2::Vector3 &dims_m,
                              std::string &err)
{
    // O viewer_3D roda com cwd em $CARMEN_HOME/bin e passa o nome direto pro
    // glmReadOBJ, entao caminho relativo e' relativo a bin/. O RViz precisa de
    // uma URI absoluta, e quem abre o arquivo e' o processo do RViz.
    std::string file = raw_file;
    if (!file.empty() && file[0] != '/') {
        const char *carmen_home = getenv("CARMEN_HOME");
        std::string bin_dir = carmen_home ? (std::string(carmen_home) + "/bin") : std::string(".");
        file = bin_dir + "/" + file;
    }

    double bb_min[3] = {0, 0, 0}, bb_max[3] = {0, 0, 0};
    if (!obj_bounding_box(file, bb_min, bb_max)) {
        err = "nao consegui abrir/medir '" + file + "'";
        return false;
    }

    if (size_x <= 0.0 || size_y <= 0.0 || size_z <= 0.0) {
        err = "size_x/y/z invalido";
        return false;
    }

    // glmUnitize: w/h/d sao |max|+|min| (nao max-min) e o scale normaliza o
    // maior deles para 2.0. Reproduzido igual para bater com o viewer.
    const double w = std::abs(bb_max[0]) + std::abs(bb_min[0]);
    const double h = std::abs(bb_max[1]) + std::abs(bb_min[1]);
    const double d = std::abs(bb_max[2]) + std::abs(bb_min[2]);
    const double unit_scale = 2.0 / std::max(w, std::max(h, d));

    // glmScaleXYZ(size/2) por eixo, no referencial do proprio modelo -- que e'
    // exatamente onde o RViz aplica marker.scale (escala do no' antes da pose).
    scale.setValue(unit_scale * size_x / 2.0,
                   unit_scale * size_y / 2.0,
                   unit_scale * size_z / 2.0);
    center.setValue((bb_max[0] + bb_min[0]) / 2.0,
                    (bb_max[1] + bb_min[1]) / 2.0,
                    (bb_max[2] + bb_min[2]) / 2.0);
    dims_m.setValue((bb_max[0] - bb_min[0]) * scale.x(),
                    (bb_max[1] - bb_min[1]) * scale.y(),
                    (bb_max[2] - bb_min[2]) * scale.z());
    uri = "file://" + file;
    return true;
}

void PointcloudNode::setup_car_model()
{
    char  *carmodel_file = nullptr;
    double cm_x = 0.0, cm_y = 0.0, cm_z = 0.0;
    double cm_roll = 0.0, cm_pitch = 0.0, cm_yaw = 0.0;
    double cm_size_x = 0.0, cm_size_y = 0.0, cm_size_z = 0.0;

    carmen_param_allow_unfound_variables(1);
    carmen_param_t param_list[] = {
        {(char *) "carmodel", (char *) "file_name", CARMEN_PARAM_STRING, &carmodel_file, 0, NULL},
        {(char *) "carmodel", (char *) "size_x",    CARMEN_PARAM_DOUBLE, &cm_size_x,     0, NULL},
        {(char *) "carmodel", (char *) "size_y",    CARMEN_PARAM_DOUBLE, &cm_size_y,     0, NULL},
        {(char *) "carmodel", (char *) "size_z",    CARMEN_PARAM_DOUBLE, &cm_size_z,     0, NULL},
        {(char *) "carmodel", (char *) "x",         CARMEN_PARAM_DOUBLE, &cm_x,          0, NULL},
        {(char *) "carmodel", (char *) "y",         CARMEN_PARAM_DOUBLE, &cm_y,          0, NULL},
        {(char *) "carmodel", (char *) "z",         CARMEN_PARAM_DOUBLE, &cm_z,          0, NULL},
        {(char *) "carmodel", (char *) "roll",      CARMEN_PARAM_DOUBLE, &cm_roll,       0, NULL},
        {(char *) "carmodel", (char *) "pitch",     CARMEN_PARAM_DOUBLE, &cm_pitch,      0, NULL},
        {(char *) "carmodel", (char *) "yaw",       CARMEN_PARAM_DOUBLE, &cm_yaw,        0, NULL},
    };
    carmen_param_install_params(0, nullptr, param_list,
                               sizeof(param_list) / sizeof(param_list[0]));
    carmen_param_allow_unfound_variables(0);

    // O rosparam car_model_file vence o param_daemon (util quando o .ini traz
    // um caminho absoluto de outra maquina, como acontece em varios carmen-*.ini).
    std::string file = car_model_file_override_.empty()
                     ? (carmodel_file ? std::string(carmodel_file) : std::string())
                     : car_model_file_override_;
    if (file.empty()) {
        ROS_WARN("carmodel_file_name nao encontrado no param_daemon e nenhum "
                 "~car_model_file definido -- modelo 3D do robo desativado.");
        return;
    }

    if (cm_size_x <= 0.0 || cm_size_y <= 0.0 || cm_size_z <= 0.0) {
        ROS_WARN("carmodel_size_* invalido/ausente (%.3f, %.3f, %.3f) -- usando "
                 "as dimensoes cruas do .obj.", cm_size_x, cm_size_y, cm_size_z);
        cm_size_x = cm_size_y = cm_size_z = 2.0;   // neutro apos o unitize
    }

    tf2::Vector3 scale, bbox_center, dims_m;
    std::string  uri, err;
    if (!prepare_obj_model(file, cm_size_x, cm_size_y, cm_size_z,
                           uri, scale, bbox_center, dims_m, err)) {
        ROS_ERROR("Modelo 3D do robo desativado: %s. (O RViz tambem precisa "
                  "enxergar esse caminho.)", err.c_str());
        return;
    }
    const double sx = scale.x(), sy = scale.y(), sz = scale.z();

    // Rotacao liquida do viewer: glRotatef(90,X) e depois glRotatef(-yaw,Y).
    // Como o eixo Y local vira o Z do mundo apos o primeiro giro, isso equivale
    // a Rz(-yaw) * Rx(+90) -- ou seja, setRPY(pi/2, 0, -yaw).
    // (draw_car() ignora carmodel_roll/pitch; mantido igual de proposito.)
    if (std::abs(cm_roll) > 1e-9 || std::abs(cm_pitch) > 1e-9) {
        ROS_WARN("carmodel_roll/pitch (%.4f, %.4f) sao ignorados -- o viewer_3D "
                 "tambem so' usa carmodel_yaw para o carro.", cm_roll, cm_pitch);
    }
    tf2::Quaternion q;
    q.setRPY(M_PI / 2.0, 0.0, -cm_yaw);
    const tf2::Matrix3x3 R(q);

    // Onde o viewer poe o CENTRO do modelo, relativo ao frame do veiculo:
    // glTranslatef(carmodel_x, carmodel_y, carmodel_z + wheel_radius).
    const tf2::Vector3 p0(cm_x, cm_y, cm_z + robot_wheel_radius_);

    // O RViz nao centra o mesh: a origem dele e' a origem do .obj. Compensa
    // subtraindo o centro da bounding box (ja' escalado e girado):
    //     t = p0 - R * (S * centro)
    const tf2::Vector3 c_scaled(bbox_center.x() * sx,
                                bbox_center.y() * sy,
                                bbox_center.z() * sz);
    const tf2::Vector3 t = p0 - R * c_scaled;

    car_model_uri_ = uri;

    visualization_msgs::Marker &m = car_model_marker_;
    m.header.frame_id = car_model_frame_id_;
    m.header.stamp    = ros::Time(0);   // "use a TF mais recente"
    m.ns              = "car_model";
    m.id              = 0;
    m.type            = visualization_msgs::Marker::MESH_RESOURCE;
    m.action          = visualization_msgs::Marker::ADD;
    m.mesh_resource   = car_model_uri_;
    m.mesh_use_embedded_materials = true;   // usa o .mtl/texturas do proprio .obj
    m.pose.position.x = t.x();
    m.pose.position.y = t.y();
    m.pose.position.z = t.z();
    m.pose.orientation = tf2::toMsg(q);
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    // Cor toda zerada + mesh_use_embedded_materials: o RViz desenha o modelo
    // com os materiais do arquivo, sem tingir.
    m.color.r = m.color.g = m.color.b = m.color.a = 0.0;
    m.lifetime     = ros::Duration(0);
    m.frame_locked = true;   // re-transforma a cada frame, com a TF mais nova

    car_model_ready_ = true;
    ROS_INFO("Modelo 3D do robo pronto: %s | escala=(%.6f, %.6f, %.6f) | "
             "pose no %s=(%.3f, %.3f, %.3f) yaw=%.3f rad | bbox final=(%.2f x %.2f x %.2f) m",
             car_model_uri_.c_str(), sx, sy, sz, car_model_frame_id_.c_str(),
             t.x(), t.y(), t.z(), cm_yaw,
             dims_m.x(), dims_m.y(), dims_m.z());

    publish_car_model_marker();
    setup_trailer_models();
}

void PointcloudNode::publish_car_model_marker()
{
    if (!car_model_ready_.load()) return;
    car_model_pub_.publish(car_model_marker_);
}

// Mesma tecnica do cavalo, para os semi-reboques (draw_car.cpp:632). A
// diferenca e' que a pose depende do beta que vem do localize -- exatamente a
// mesma fonte que ja' alimenta os cilindros de colisao do reboque.
void PointcloudNode::setup_trailer_models()
{
    int num_trailers = 0;
    carmen_param_allow_unfound_variables(1);
    carmen_param_t n_list[] = {
        // Igual ao viewer_3D: ele usa semi_trailer_initial_type como a
        // QUANTIDADE de reboques (0 = nao tem).
        {(char *) "semi_trailer", (char *) "initial_type", CARMEN_PARAM_INT, &num_trailers, 0, NULL},
    };
    carmen_param_install_params(0, nullptr, n_list, 1);
    carmen_param_allow_unfound_variables(0);

    if (num_trailers <= 0) {
        ROS_INFO("semi_trailer_initial_type=%d -- veiculo sem semi-reboque, "
                 "so' o modelo do cavalo sera' publicado.", num_trailers);
        return;
    }
    if (num_trailers > MAX_NUM_TRAILERS) num_trailers = MAX_NUM_TRAILERS;

    for (int id = 1; id <= num_trailers; ++id) {
        char sec_trailer[64], sec_model[64];
        snprintf(sec_trailer, sizeof(sec_trailer), "semi_trailer%d", id);
        snprintf(sec_model,   sizeof(sec_model),   "semi_trailer_model%d", id);

        char  *file = nullptr;
        double d = 0.0, M = 0.0;
        double sx = 0.0, sy = 0.0, sz = 0.0;
        double ox = 0.0, oy = 0.0, oz = 0.0;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;

        carmen_param_allow_unfound_variables(1);
        carmen_param_t p[] = {
            {sec_trailer, (char *) "d",         CARMEN_PARAM_DOUBLE, &d,     0, NULL},
            {sec_trailer, (char *) "M",         CARMEN_PARAM_DOUBLE, &M,     0, NULL},
            {sec_model,   (char *) "file_name", CARMEN_PARAM_STRING, &file,  0, NULL},
            {sec_model,   (char *) "size_x",    CARMEN_PARAM_DOUBLE, &sx,    0, NULL},
            {sec_model,   (char *) "size_y",    CARMEN_PARAM_DOUBLE, &sy,    0, NULL},
            {sec_model,   (char *) "size_z",    CARMEN_PARAM_DOUBLE, &sz,    0, NULL},
            {sec_model,   (char *) "x",         CARMEN_PARAM_DOUBLE, &ox,    0, NULL},
            {sec_model,   (char *) "y",         CARMEN_PARAM_DOUBLE, &oy,    0, NULL},
            {sec_model,   (char *) "z",         CARMEN_PARAM_DOUBLE, &oz,    0, NULL},
            {sec_model,   (char *) "roll",      CARMEN_PARAM_DOUBLE, &roll,  0, NULL},
            {sec_model,   (char *) "pitch",     CARMEN_PARAM_DOUBLE, &pitch, 0, NULL},
            {sec_model,   (char *) "yaw",       CARMEN_PARAM_DOUBLE, &yaw,   0, NULL},
        };
        carmen_param_install_params(0, nullptr, p, sizeof(p) / sizeof(p[0]));
        carmen_param_allow_unfound_variables(0);

        if (!file || file[0] == '\0') {
            ROS_WARN("%s_file_name nao encontrado -- semi-reboque %d nao sera' desenhado.",
                     sec_model, id);
            break;   // sem o modelo, a corrente quebra dai' pra frente
        }

        TrailerModel tm;
        tf2::Vector3 dims_m;
        std::string  err;
        if (!prepare_obj_model(file, sx, sy, sz, tm.uri, tm.scale, tm.bbox_center, dims_m, err)) {
            ROS_ERROR("Semi-reboque %d desativado: %s.", id, err.c_str());
            break;
        }

        tm.off_x = ox; tm.off_y = oy; tm.off_z = oz;
        tm.roll = roll; tm.pitch = pitch; tm.yaw = yaw;
        tm.d = d; tm.M = M;
        trailer_models_.push_back(tm);

        ROS_INFO("Semi-reboque %d pronto: %s | d=%.3f M=%.3f | offset=(%.2f, %.2f, %.2f) "
                 "yaw=%.3f | bbox final=(%.2f x %.2f x %.2f) m",
                 id, tm.uri.c_str(), d, M, ox, oy, oz, yaw,
                 dims_m.x(), dims_m.y(), dims_m.z());
    }

    publish_trailer_model_markers();
}

void PointcloudNode::publish_trailer_model_markers()
{
    if (trailer_models_.empty()) return;

    visualization_msgs::MarkerArray arr;

    // Desengatado: apaga o que estiver desenhado (o viewer simplesmente nao
    // desenha; no RViz marker sem DELETE fica na tela pra sempre).
    if (!trailer_engaged_.load()) {
        for (size_t i = 0; i < trailer_models_.size(); ++i) {
            visualization_msgs::Marker m;
            m.header.frame_id = car_model_frame_id_;
            m.header.stamp    = ros::Time(0);
            m.ns              = "car_model_trailer";
            m.id              = static_cast<int>(i);
            m.action          = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(m);
        }
        car_model_trailers_pub_.publish(arr);
        return;
    }

    double beta[MAX_NUM_TRAILERS];
    for (int i = 0; i < MAX_NUM_TRAILERS; ++i) beta[i] = trailer_betas_[i].load();

    // calculate_trailer_positions() com drawing_model=1 (draw_car.cpp:303).
    // Repare que o viewer usa SEMPRE o offset do modelo 0 como "posicao do
    // carro" -- mantido igual de proposito, senao a corrente nao bate.
    const double car_x = trailer_models_[0].off_x;
    const double car_y = trailer_models_[0].off_y;
    const double car_z = trailer_models_[0].off_z;

    const size_t n = trailer_models_.size();
    std::vector<tf2::Vector3> hop(n);   // translacao de cada elo, ja' no eixo do mundo
    for (size_t i = 0; i < n; ++i) {
        const double cb = std::cos(beta[i]), sb = std::sin(beta[i]);
        const double prev_x = (i == 0) ? car_x : (-car_x * cb) + (car_y * sb) + car_x;
        const double prev_y = (i == 0) ? car_y : (car_x * sb) - (car_y * cb);

        const double st_x = prev_x - (trailer_models_[i].d + trailer_models_[i].M * cb);
        const double st_y = prev_y + trailer_models_[i].M * sb;

        // O viewer translada (x, z, y) num referencial ja' girado por Rx(90).
        // Passando o Rx(90) pra fora, no eixo do veiculo isso vira (x, -y, z).
        hop[i].setValue(st_x, -st_y, car_z);
    }

    for (size_t k = 0; k < n; ++k) {
        const TrailerModel &tm = trailer_models_[k];

        // Corrente: para o reboque k, aplica Rz(-beta_i) e o salto i, i=0..k.
        tf2::Transform T;
        T.setIdentity();
        for (size_t i = 0; i <= k; ++i) {
            tf2::Quaternion qb;
            qb.setRPY(0.0, 0.0, -beta[i]);
            T *= tf2::Transform(qb, tf2::Vector3(0, 0, 0));
            T *= tf2::Transform(tf2::Quaternion(0, 0, 0, 1), hop[i]);
        }

        // Rx(90) * Ry(-yaw) * Rx(-pitch) * Rz(-roll)  ==  Rz(-yaw) * Rx(pi/2 - pitch) * Rz(-roll)
        tf2::Quaternion q_yaw, q_pitch, q_roll;
        q_yaw.setRPY(0.0, 0.0, -tm.yaw);
        q_pitch.setRPY(M_PI / 2.0 - tm.pitch, 0.0, 0.0);
        q_roll.setRPY(0.0, 0.0, -tm.roll);
        T *= tf2::Transform(q_yaw * q_pitch * q_roll, tf2::Vector3(0, 0, 0));

        // Mesma compensacao do glmUnitize usada no cavalo.
        const tf2::Vector3 c_scaled(tm.bbox_center.x() * tm.scale.x(),
                                    tm.bbox_center.y() * tm.scale.y(),
                                    tm.bbox_center.z() * tm.scale.z());
        const tf2::Vector3 pos = T.getOrigin() - T.getBasis() * c_scaled;

        visualization_msgs::Marker m;
        m.header.frame_id = car_model_frame_id_;
        m.header.stamp    = ros::Time(0);
        m.ns              = "car_model_trailer";
        m.id              = static_cast<int>(k);
        m.type            = visualization_msgs::Marker::MESH_RESOURCE;
        m.action          = visualization_msgs::Marker::ADD;
        m.mesh_resource   = tm.uri;
        m.mesh_use_embedded_materials = true;
        m.pose.position.x = pos.x();
        m.pose.position.y = pos.y();
        m.pose.position.z = pos.z();
        m.pose.orientation = tf2::toMsg(T.getRotation());
        m.scale.x = tm.scale.x();
        m.scale.y = tm.scale.y();
        m.scale.z = tm.scale.z();
        m.color.r = m.color.g = m.color.b = m.color.a = 0.0;
        m.lifetime     = ros::Duration(0);
        m.frame_locked = true;
        arr.markers.push_back(m);
    }

    car_model_trailers_pub_.publish(arr);
}

void PointcloudNode::publish_static_transforms(const ros::Time &stamp)
{
    const ros::Time tf_time = carmen_bridge::tf_stamp(stamp);

    tf2::Quaternion q_tf_robot;
    q_tf_robot.setRPY(sb1_roll_, sb1_pitch_, sb1_yaw_);
    geometry_msgs::TransformStamped tf_robot;
    tf_robot.header.stamp    = tf_time;
    tf_robot.header.frame_id = base_frame_id_;
    tf_robot.child_frame_id  = sensorboard_frame_id_;
    tf_robot.transform.translation.x = sb1_x_;
    tf_robot.transform.translation.y = sb1_y_;
    tf_robot.transform.translation.z = sb1_z_;
    tf_robot.transform.rotation      = tf2::toMsg(q_tf_robot);
    static_tf_broadcaster_->sendTransform(tf_robot);

    geometry_msgs::TransformStamped tf_lidar;
    tf_lidar.header.stamp    = tf_time;
    tf_lidar.header.frame_id = base_frame_id_;
    tf_lidar.child_frame_id  = laser_frame_id_;
    tf_lidar.transform.translation.x = T_robot_lidar.getOrigin().x();
    tf_lidar.transform.translation.y = T_robot_lidar.getOrigin().y();
    tf_lidar.transform.translation.z = T_robot_lidar.getOrigin().z();
    tf_lidar.transform.rotation.x    = T_robot_lidar.getRotation().x();
    tf_lidar.transform.rotation.y    = T_robot_lidar.getRotation().y();
    tf_lidar.transform.rotation.z    = T_robot_lidar.getRotation().z();
    tf_lidar.transform.rotation.w    = T_robot_lidar.getRotation().w();
    static_tf_broadcaster_->sendTransform(tf_lidar);

    tf2::Quaternion q_tf_gnd;
    q_tf_gnd.setRPY(0.0, 0.0, 0.0);
    geometry_msgs::TransformStamped tf_gnd;
    tf_gnd.header.stamp    = tf_time;
    tf_gnd.header.frame_id = base_frame_id_;
    tf_gnd.child_frame_id  = ground_frame_id_;
    tf_gnd.transform.translation.x = 0.0;
    tf_gnd.transform.translation.y = 0.0;
    tf_gnd.transform.translation.z = -robot_wheel_radius_;
    tf_gnd.transform.rotation      = tf2::toMsg(q_tf_gnd);
    static_tf_broadcaster_->sendTransform(tf_gnd);

    // NOTA: map_base_link e map_ground_link sao o INVERSO de T_robot_lidar /
    // T_ground_robot_ (a arvore de TF so aceita um pai por frame, entao esses
    // dois viram "ramos" separados enraizados em "map", em vez de pendurar
    // direto embaixo de base_link). O inverso de uma transformacao rigida
    // T=(R,t) e T^-1=(R^-1, -R^-1*t) -- NAO é (R^-1, -t). Negar a translacao
    // direto so da certo quando R é ~identidade; com o lidar da IARA girado
    // ~90 graus em relacao ao veiculo isso saia com posicao e orientacao
    // erradas. Usa tf2::Transform::inverse(), que faz a conta certa.
    const tf2::Transform T_lidar_robot = T_robot_lidar.inverse();
    geometry_msgs::TransformStamped tf_map_base;
    tf_map_base.header.stamp    = tf_time;
    tf_map_base.header.frame_id = "map";
    tf_map_base.child_frame_id  = map_base_link_frame_id_;
    tf_map_base.transform.translation.x = T_lidar_robot.getOrigin().x();
    tf_map_base.transform.translation.y = T_lidar_robot.getOrigin().y();
    tf_map_base.transform.translation.z = T_lidar_robot.getOrigin().z();
    tf_map_base.transform.rotation.x    = T_lidar_robot.getRotation().x();
    tf_map_base.transform.rotation.y    = T_lidar_robot.getRotation().y();
    tf_map_base.transform.rotation.z    = T_lidar_robot.getRotation().z();
    tf_map_base.transform.rotation.w    = T_lidar_robot.getRotation().w();
    static_tf_broadcaster_->sendTransform(tf_map_base);

    const tf2::Transform T_robot_ground = T_ground_robot_.inverse();
    geometry_msgs::TransformStamped tf_base_gnd_map;
    tf_base_gnd_map.header.stamp    = tf_time;
    tf_base_gnd_map.header.frame_id = map_base_link_frame_id_;
    tf_base_gnd_map.child_frame_id  = map_ground_frame_id_;
    tf_base_gnd_map.transform.translation.x = T_robot_ground.getOrigin().x();
    tf_base_gnd_map.transform.translation.y = T_robot_ground.getOrigin().y();
    tf_base_gnd_map.transform.translation.z = T_robot_ground.getOrigin().z();
    tf_base_gnd_map.transform.rotation.x    = T_robot_ground.getRotation().x();
    tf_base_gnd_map.transform.rotation.y    = T_robot_ground.getRotation().y();
    tf_base_gnd_map.transform.rotation.z    = T_robot_ground.getRotation().z();
    tf_base_gnd_map.transform.rotation.w    = T_robot_ground.getRotation().w();
    static_tf_broadcaster_->sendTransform(tf_base_gnd_map);
}

void PointcloudNode::start_ipc_thread()
{
    std::string ipc_host = ipc_host_;
    ipc_thread_ = std::thread([this, ipc_host]() {
        std::string prog = "pointcloud_node";
        std::string flag = "-central_host";
        std::vector<char*> argv_vec = {
            const_cast<char*>(prog.c_str()),
            const_cast<char*>(flag.c_str()),
            const_cast<char*>(ipc_host.c_str())
        };
        int argc = static_cast<int>(argv_vec.size());

        carmen_ipc_initialize(argc, argv_vec.data());
        load_carmen_lidar_config(argc, argv_vec.data());

        carmen_velodyne_subscribe_partial_scan_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(velodyne_partial_scan_handler),
            CARMEN_SUBSCRIBE_LATEST);
        carmen_velodyne_subscribe_variable_scan_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(velodyne_variable_scan_handler),
            CARMEN_SUBSCRIBE_LATEST, lidar_sensor_id_);

        // Estado de articulação do semi-trailer (beta) para o filtro de colisão.
        carmen_localize_ackerman_subscribe_globalpos_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(localize_ackerman_globalpos_handler),
            CARMEN_SUBSCRIBE_LATEST);

        ROS_INFO("IPC dispatch loop iniciado (host=%s, lidar_sensor_id=%d).",
                 ipc_host.c_str(), lidar_sensor_id_);

        while (running_) IPC_listenWait(1);
        IPC_disconnect();
    });
}

// ─── Conversão bruto -> 3D ───────────────────────────────────────────────────

// Mede o período real entre scans consecutivos (EMA) para o campo "time".
float PointcloudNode::update_scan_period(const ros::Time &stamp)
{
    double stamp_sec = stamp.toSec();
    if (last_stamp_sec_ > 0.0) {
        double delta = stamp_sec - last_stamp_sec_;
        if (delta > 0.005 && delta < 0.5)
            scan_period_ema_ = 0.9f * scan_period_ema_ + 0.1f * static_cast<float>(delta);
    }
    last_stamp_sec_ = stamp_sec;
    return scan_period_ema_;
}

// Gera a nuvem /points_raw (32 bytes/ponto: x,y,z,intensity,ring,time) direto
// do frame bruto do CARMEN, sem passar por /velodyne_raw_ipc.
sensor_msgs::PointCloud2
PointcloudNode::velodyneframe_to_points_raw(const VelodyneFrame &f)
{
    const ros::Time stamp = carmen_bridge::carmen_ns_to_ros_time(f.ros_stamp_ns);

    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp    = stamp;
    cloud.header.frame_id = laser_frame_id_;
    cloud.height       = 1;
    cloud.is_bigendian = false;
    cloud.is_dense     = true;
    cloud.point_step   = 32;

    cloud.fields.resize(6);
    cloud.fields[0].name="x";         cloud.fields[0].offset=0;  cloud.fields[0].datatype=7; cloud.fields[0].count=1;
    cloud.fields[1].name="y";         cloud.fields[1].offset=4;  cloud.fields[1].datatype=7; cloud.fields[1].count=1;
    cloud.fields[2].name="z";         cloud.fields[2].offset=8;  cloud.fields[2].datatype=7; cloud.fields[2].count=1;
    cloud.fields[3].name="intensity"; cloud.fields[3].offset=12; cloud.fields[3].datatype=7; cloud.fields[3].count=1;
    cloud.fields[4].name="ring";      cloud.fields[4].offset=16; cloud.fields[4].datatype=4; cloud.fields[4].count=1;
    cloud.fields[5].name="time";      cloud.fields[5].offset=20; cloud.fields[5].datatype=7; cloud.fields[5].count=1;

    const uint32_t n = static_cast<uint32_t>(f.raw_points.size());
    cloud.data.resize(static_cast<size_t>(n) * cloud.point_step, 0);

    const float shot_period = static_cast<float>(lidar_time_between_shots_);

    uint8_t *out = cloud.data.data();
    uint32_t valid = 0;
    int point_idx = 0;

    for (int i = 0; i < f.num_shots; ++i) {
        float cos_az, sin_az;
        if (i < (int)f.shot_cos_az.size()) {
            cos_az = f.shot_cos_az[i];
            sin_az = f.shot_sin_az[i];
        } else {
            const float angle_step_fb =
                static_cast<float>(2.0 * M_PI / std::max(f.num_shots, 1));
            float az = -(static_cast<float>(i) * angle_step_fb - static_cast<float>(M_PI));
            cos_az = std::cos(az);
            sin_az = std::sin(az);
        }

        int ss = (i < (int)f.shot_sizes.size()) ? f.shot_sizes[i] : f.shot_size;
        const float time_per_point = shot_period / static_cast<float>(std::max(ss, 1));
        for (int j = 0; j < ss && point_idx < (int)f.raw_points.size(); ++j, ++point_idx) {
            const auto &rp = f.raw_points[point_idx];

            float r = static_cast<float>(rp.distance) / static_cast<float>(f.range_div);
            if (r > static_cast<float>(f.max_range))
                continue;

            int ring = static_cast<int>(rp.ring);
            if (ring < 0 || ring >= lidar_shot_size_) continue;

            float cos_el = (ring < MAX_RAYS) ? ring_cos_el_[ring] : 1.0f;
            float sin_el = (ring < MAX_RAYS) ? ring_sin_el_[ring] : 0.0f;

            // Rotaciona o azimute do shot pelo offset do canal (soma de
            // ângulos via cos/sin, sem chamar atan2/cos/sin de novo por
            // ponto). Se o canal não tem offset (default), isso é identidade.
            float cos_az_off = (ring < MAX_RAYS) ? ring_cos_az_off_[ring] : 1.0f;
            float sin_az_off = (ring < MAX_RAYS) ? ring_sin_az_off_[ring] : 0.0f;
            float cos_az_pt = cos_az * cos_az_off - sin_az * sin_az_off;
            float sin_az_pt = sin_az * cos_az_off + cos_az * sin_az_off;

            float x = r * cos_el * cos_az_pt;
            float y = r * cos_el * sin_az_pt;
            float z = r * sin_el;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            float intensity = static_cast<float>(rp.intensity);
            uint16_t ring16 = static_cast<uint16_t>(ring);
            float t = static_cast<float>(i) * shot_period
                    + static_cast<float>(j) * time_per_point;

            std::memcpy(out + 0,  &x,         4);
            std::memcpy(out + 4,  &y,         4);
            std::memcpy(out + 8,  &z,         4);
            std::memcpy(out + 12, &intensity, 4);
            std::memcpy(out + 16, &ring16,    2);
            // bytes 18-19: padding (zero)
            std::memcpy(out + 20, &t,         4);
            // bytes 24-31: padding (zero)

            out += cloud.point_step;
            ++valid;
        }
    }

    cloud.width    = valid;
    cloud.row_step = valid * cloud.point_step;
    cloud.data.resize(static_cast<size_t>(valid) * cloud.point_step);

    // [DBG] Hz de publicação (1x/seg)
    static uint64_t cnt = 0; static double t0 = 0.0;
    ++cnt;
    const double now_s = carmen_bridge::use_sim_time()
        ? stamp.toSec() : ros::Time::now().toSec();
    if (t0 == 0.0) t0 = now_s;
    if (now_s - t0 >= 1.0) {
        ROS_INFO("[DBG-CLOUD] Hz=%.1f  pontos_validos=%u  stamp=%.3f",
                 static_cast<double>(cnt) / (now_s - t0), valid, stamp.toSec());
        cnt = 0; t0 = now_s;
    }

    return cloud;
}

// ─── Filtro de pontos sobre o veículo ────────────────────────────────────────

bool PointcloudNode::ensure_collision_heights()
{
    if (collision_heights_loaded_.load()) return true;
    double rh = 0.0, th = 0.0;
    if (ros::param::get("/lio_sam/robot_collision_height", rh) &&
        ros::param::get("/lio_sam/semi_trailer_collision_height", th) &&
        rh > 0.0 && th > 0.0) {
        robot_collision_height_ = rh;
        semi_trailer_collision_height_ = th;
        double margin = 0.0;
        ros::param::get("/lio_sam/collision_safety_margin", margin);
        collision_safety_margin_ = (margin > 0.0) ? margin : 0.0;
        collision_heights_loaded_.store(true);
        ROS_INFO("Alturas de colisao: robot=%.2fm, semi_trailer=%.2fm, margem=%.2fm",
                 rh, th, collision_safety_margin_);
        return true;
    }
    return false;
}

bool PointcloudNode::point_on_vehicle(float x, float y, float z) const
{
    if (!collision_cfg_) return false;

    const tf2::Vector3 p_car = T_ground_lidar_ * tf2::Vector3(x, y, z);
    const double px = p_car.x();
    const double py = p_car.y();
    const double gh = p_car.z();   // altura do chão

    // Cilindros do caminhão.
    if (gh >= 0.0 && gh <= robot_collision_height_) {
        for (int i = 0; i < collision_cfg_->n_markers; ++i) {
            const double dx = px - collision_cfg_->markers[i].x;
            const double dy = py - collision_cfg_->markers[i].y;
            const double r  = collision_cfg_->markers[i].radius + collision_safety_margin_;
            if (dx * dx + dy * dy <= r * r) return true;
        }
    }

    // Cilindros do semi-trailer (se engatado e existente).
    if (trailer_engaged_.load() && collision_cfg_->semi_trailer_type >= 1 &&
        gh >= 0.0 && gh <= semi_trailer_collision_height_) {
        const double beta = trailer_beta_.load();
        const double d = collision_cfg_->semi_trailer_d[0];
        const double M = collision_cfg_->semi_trailer_M[0];
        const double b = -beta;
        const double cb = std::cos(b), sb = std::sin(b);
        const int n = collision_cfg_->n_semi_trailer_markers[0];
        const carmen_collision_marker_t *mk = collision_cfg_->semi_trailer_markers[0];
        for (int i = 0; i < n; ++i) {
            const double cx = (mk[i].x - d) * cb - mk[i].y * sb - M;
            const double cy = (mk[i].x - d) * sb + mk[i].y * cb;
            const double dx = px - cx;
            const double dy = py - cy;
            const double r  = mk[i].radius + collision_safety_margin_;
            if (dx * dx + dy * dy <= r * r) return true;
        }
    }

    return false;
}

void PointcloudNode::publish_collision_markers(const ros::Time &stamp)
{
    if (stamp.isZero()) return;
    if (!collision_ready_.load() || !ensure_collision_heights()) return;

    visualization_msgs::MarkerArray arr;
    int id = 0;

    // Círculos do caminhão.
    for (int i = 0; i < collision_cfg_->n_markers; ++i) {
        visualization_msgs::Marker m;
        m.header.frame_id = base_frame_id_;
        m.header.stamp = stamp;
        m.ns = "collision_truck";
        m.id = id++;
        m.type = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;

        m.pose.position.x = collision_cfg_->markers[i].x;
        m.pose.position.y = collision_cfg_->markers[i].y;
        m.pose.position.z = -robot_wheel_radius_ + (robot_collision_height_ / 2.0);
        m.pose.orientation.w = 1.0;

        double r = collision_cfg_->markers[i].radius + collision_safety_margin_;
        m.scale.x = m.scale.y = 2.0 * r;
        m.scale.z = robot_collision_height_;
        m.color.r = 1.0; m.color.a = 0.35;
        m.lifetime = ros::Duration(0);
        arr.markers.push_back(m);
    }

    // Círculos do semi-trailer (se engatado).
    if (trailer_engaged_.load() && collision_cfg_->semi_trailer_type >= 1) {
        const double beta = trailer_beta_.load();
        const double d = collision_cfg_->semi_trailer_d[0];
        const double M = collision_cfg_->semi_trailer_M[0];
        const double b = -beta;
        const double cb = std::cos(b), sb = std::sin(b);
        const int n = collision_cfg_->n_semi_trailer_markers[0];
        const carmen_collision_marker_t *mk = collision_cfg_->semi_trailer_markers[0];
        for (int i = 0; i < n; ++i) {
            visualization_msgs::Marker m;
            m.header.frame_id = base_frame_id_;
            m.header.stamp = stamp;
            m.ns = "collision_trailer";
            m.id = id++;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;

            m.pose.position.x = (mk[i].x - d) * cb - mk[i].y * sb - M - sb1_x_;
            m.pose.position.y = (mk[i].x - d) * sb + mk[i].y * cb - sb1_y_;
            m.pose.position.z = -robot_wheel_radius_ + (semi_trailer_collision_height_ / 2.0);
            m.pose.orientation.w = 1.0;

            double r = mk[i].radius + collision_safety_margin_;
            m.scale.x = m.scale.y = 2.0 * r;
            m.scale.z = semi_trailer_collision_height_;
            m.color.b = 1.0; m.color.a = 0.35;
            m.lifetime = ros::Duration(0);
            arr.markers.push_back(m);
        }
    }

    collision_markers_pub_.publish(arr);
}

void PointcloudNode::on_points_raw(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    const uint32_t ps = msg->point_step;
    const uint32_t n  = msg->width;

    // Enquanto o modelo/transformação/alturas não estiverem prontos, ou se o
    // layout não tiver ao menos x,y,z, republica sem filtrar (pass-through).
    if (!collision_ready_.load() || !ensure_collision_heights() || ps < 12 || n == 0) {
        //ROS_WARN_THROTTLE(5.0,
        //    "Filtro de colisao em pass-through (config/alturas ainda nao prontos).");
        points_no_vehicle_pub_.publish(*msg);
        return;
    }

    sensor_msgs::PointCloud2 out;
    out.header       = msg->header;
    out.height       = 1;
    out.fields       = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step   = ps;
    out.is_dense     = msg->is_dense;
    out.data.resize(msg->data.size());

    const uint8_t *in  = msg->data.data();
    uint8_t       *o   = out.data.data();
    uint32_t       valid = 0;
    uint32_t       removed = 0;

    for (uint32_t i = 0; i < n; ++i) {
        const uint8_t *p = in + static_cast<size_t>(i) * ps;
        float x, y, z;
        std::memcpy(&x, p + 0, 4);
        std::memcpy(&y, p + 4, 4);
        std::memcpy(&z, p + 8, 4);

        if (point_on_vehicle(x, y, z)) { ++removed; continue; }

        std::memcpy(o, p, ps);
        o += ps;
        ++valid;
    }

    out.width    = valid;
    out.row_step = valid * ps;
    out.data.resize(static_cast<size_t>(valid) * ps);
    points_no_vehicle_pub_.publish(out);

    static uint64_t cnt = 0; static double t0 = 0.0;
    ++cnt;
    const double now_s = carmen_bridge::use_sim_time()
        ? msg->header.stamp.toSec() : ros::Time::now().toSec();
    if (t0 == 0.0) t0 = now_s;
    if (now_s - t0 >= 1.0) {
        ROS_INFO("[DBG-COLLISION] Hz=%.1f  pontos=%u  removidos=%u  trailer_engaged=%d",
                 static_cast<double>(cnt) / (now_s - t0), n, removed,
                 static_cast<int>(trailer_engaged_.load()));
        cnt = 0; t0 = now_s;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PointcloudNode node(nh, pnh);
    ros::spin();
    return 0;
}