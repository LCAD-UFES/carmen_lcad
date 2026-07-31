/*
 * ipc_bridge_node.cpp  (ROS1 / Noetic)
 *
 * Responsabilidades:
 *   1. Lê XSens do CARMEN IPC e publica /imu_raw
 *   2. Lê a odometria Ackermann e publica /ackermann/odom_raw
 *   3. Lê GPS (xyz + heading) e publica /gps/xyz_raw/gps_<nr> e /gps/heading_raw
 *   4. Publica o TF estático base_link → imu_link
 *
 * O caminho do LiDAR (velodyne → /points_raw → /points_raw_no_vehicle) foi
 * movido inteiro para o pointcloud_node; este nó não toca mais na nuvem.
 */

#include "carmen_ipc_bridge/ipc_bridge_node.hpp"
#include "carmen_ipc_bridge/carmen_time.hpp"
#include <carmen/xsens_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/imu_interface.h>
#include <carmen/global.h>
#include <cmath>
#include <chrono>
#include <map>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

using carmen_bridge::IpcBridgeNode;

carmen_bridge::IpcBridgeNode *carmen_bridge::g_bridge_node = nullptr;

// ─── Fallback XSens -> IMU do LiDAR ──────────────────────────────────────────
// Prioridade normal: XSens. Se o XSens nunca publicar (nao existir/nao
// conectado) ou publicar zerado (sensor travado mas IPC ainda de pe), o IMU
// embutido do LiDAR (carmen_imu_message, ver ouster_main.cpp) assume como
// referencia. Volta pro XSens sozinho assim que ele publicar dado valido de
// novo -- nao precisa reiniciar nada.
static int    g_lidar_imu_sensor_id = 0;
static double g_last_xsens_carmen_time = -1.0;  // timestamp CARMEN da ultima msg xsens valida
static bool   g_last_xsens_valid = false;
static const double XSENS_TIMEOUT_S = 1.0;   // sem msg ha mais que isso -> fallback
static const double XSENS_ZERO_EPS  = 1e-6;

static bool xsens_is_active(double ref_carmen_time)
{
    if (g_last_xsens_carmen_time < 0.0) return false;
    if (!g_last_xsens_valid)          return false;
    return (ref_carmen_time - g_last_xsens_carmen_time) < XSENS_TIMEOUT_S;
}

// ─── Handlers IPC ─────────────────────────────────────────────────────────────

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

    // "zerado" = accel e giro tudo zero -- xsens travado/desconectado mas
    // ainda publicando no IPC (falha silenciosa, diferente de topico ausente)
    bool is_zeroed =
        (fabs(frame.ax) < XSENS_ZERO_EPS) && (fabs(frame.ay) < XSENS_ZERO_EPS) && (fabs(frame.az) < XSENS_ZERO_EPS) &&
        (fabs(frame.gx) < XSENS_ZERO_EPS) && (fabs(frame.gy) < XSENS_ZERO_EPS) && (fabs(frame.gz) < XSENS_ZERO_EPS);

    g_last_xsens_carmen_time = msg->timestamp;
    g_last_xsens_valid = !is_zeroed;

    if (is_zeroed)
    {
        //ROS_WARN_THROTTLE(2.0, "[DBG-XSENS] mensagem zerada recebida -- cedendo pro fallback do IMU do lidar");
        return; // nao publica lixo; lidar_imu_handler assume a partir daqui
    }

    frame.ros_stamp_ns = carmen_bridge::carmen_to_ros_ns(msg->timestamp);


    carmen_bridge::g_bridge_node->on_imu(frame);
}

// Fallback: IMU embutido do LiDAR (carmen_imu_message, publicado pelo
// ouster_main.cpp via carmen_imu_publish_imu_lidar_message). So publica
// quando o xsens nao existe ou esta zerado -- ver xsens_is_active().
// Assim que o xsens voltar a publicar dado valido, este handler volta a
// ficar quieto sozinho, sem precisar de nenhuma troca manual.
static void lidar_imu_handler(carmen_imu_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;
    if (xsens_is_active(msg->timestamp)) return; // xsens tem prioridade, ignora o lidar

    static std::atomic<uint64_t> lidar_imu_count{0};
    ++lidar_imu_count;
    //ROS_WARN_THROTTLE(2.0,
    //    "[DBG-IMU-FALLBACK] usando IMU do lidar (xsens ausente/zerado)  msgs/s=%lu  ts_carmen=%.3f",
    //    static_cast<unsigned long>(lidar_imu_count.exchange(0)), msg->timestamp);

    carmen_bridge::ImuFrame frame;
    frame.timestamp = msg->timestamp;
    frame.ax = msg->accX;
    frame.ay = msg->accY;
    frame.az = msg->accZ;
    frame.gx = msg->gyroX;
    frame.gy = msg->gyroY;
    frame.gz = msg->gyroZ;
    frame.qw = msg->q0;
    frame.qx = msg->q1;
    frame.qy = msg->q2;
    frame.qz = msg->q3;
    frame.has_orientation = true;

    frame.ros_stamp_ns = carmen_bridge::carmen_to_ros_ns(msg->timestamp);


    carmen_bridge::g_bridge_node->on_imu(frame);
}

static void base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;

    carmen_bridge::AckermannFrame frame;
    frame.timestamp = msg->timestamp;
    frame.v   = msg->v;
    frame.phi = msg->phi;

    frame.ros_stamp_ns = carmen_bridge::carmen_to_ros_ns(msg->timestamp);

    carmen_bridge::g_bridge_node->on_ackermann(frame);
}

// // GPS bruto (xyz + heading). Nenhuma fusão/correção acontece aqui — isso é
// // responsabilidade de um nó separado (gps_localization_node).
// static void gps_xyz_handler(carmen_gps_xyz_message *msg)
// {
//     if (!carmen_bridge::g_bridge_node || !msg) return;

//     static std::atomic<uint64_t> gps_xyz_count{0};
//     ++gps_xyz_count;
//     ROS_INFO_THROTTLE(1.0,
//         "[DBG-GPS-XYZ] msgs/s=%lu  nr=%d  quality=%d  x=%.3f y=%.3f theta=%.4f  ts_carmen=%.3f",
//         static_cast<unsigned long>(gps_xyz_count.exchange(0)),
//         msg->nr, msg->gps_quality, msg->x, msg->y, msg->theta, msg->timestamp);

//     carmen_bridge::GpsXyzFrame frame;
//     frame.timestamp   = msg->timestamp;
//     frame.nr          = msg->nr;
//     frame.x           = msg->x;
//     frame.y           = msg->y;
//     frame.z           = msg->z;
//     frame.theta       = msg->theta;
//     frame.gps_quality = msg->gps_quality;
//     frame.valid       = (msg->gps_quality >= 1);

//     frame.ros_stamp_ns = carmen_bridge::carmen_to_ros_ns(msg->timestamp);

//     carmen_bridge::g_bridge_node->on_gps_xyz(frame);
// }

static void gps_hdt_handler(carmen_gps_gphdt_message *msg)
{
    if (!carmen_bridge::g_bridge_node || !msg) return;

    carmen_bridge::GpsHdtFrame frame;
    frame.timestamp = msg->timestamp;
    frame.heading    = msg->heading;
    frame.valid      = (msg->valid != 0);

    frame.ros_stamp_ns = carmen_bridge::carmen_to_ros_ns(msg->timestamp);

    carmen_bridge::g_bridge_node->on_gps_hdt(frame);
}

// ─── Constructor / Destructor ─────────────────────────────────────────────────

IpcBridgeNode::IpcBridgeNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh)
{
    g_bridge_node = this;

    declare_parameters();
    setup_publishers();
    setup_tf();
    start_ipc_thread();

    ROS_INFO("ipc_bridge_node iniciado.");
    ROS_INFO("  IMU: %s  Ackermann: %s  GPS: %s",
        publish_imu_        ? "SIM" : "NAO",
        publish_ackermann_  ? "SIM" : "NAO",
        publish_gps_        ? "SIM" : "NAO");
    ROS_INFO("  IMU fallback: lidar_imu_sensor_id=%d (assume se xsens sumir/zerar por %.1fs)",
        g_lidar_imu_sensor_id, XSENS_TIMEOUT_S);
}

IpcBridgeNode::~IpcBridgeNode()
{
    running_ = false;
    if (ipc_thread_.joinable()) ipc_thread_.join();
}

// ─── Inicialização ────────────────────────────────────────────────────────────

void IpcBridgeNode::declare_parameters()
{
    pnh_.param<std::string>("base_frame_id",  base_frame_id_,  "base_link");
    pnh_.param<std::string>("imu_frame_id",   imu_frame_id_,   "imu_link");
    pnh_.param<bool>  ("publish_imu",       publish_imu_,       true);
    pnh_.param<bool>  ("publish_ackermann", publish_ackermann_, true);
    pnh_.param<bool>  ("publish_gps",       publish_gps_,       true);
    pnh_.param<std::string>("gps_frame_id", gps_frame_id_,   "utm");
    pnh_.param<std::string>("ipc_host",    ipc_host_,       "localhost");

    // id do lidar (ouster_sensor_id no carmen.ini dele) cujo IMU embutido
    // serve de fallback quando o xsens some ou vem zerado
    pnh_.param<int>("lidar_imu_sensor_id", g_lidar_imu_sensor_id, 0);

    std::vector<double> rot;
    pnh_.param("imu_rotation", rot, std::vector<double>{1,0,0, 0,1,0, 0,0,1});
    if (rot.size() == 9)
        for (int i = 0; i < 9; ++i) imu_rotation_[i] = rot[i];

    ROS_INFO(
        "imu_rotation: [%.0f %.0f %.0f | %.0f %.0f %.0f | %.0f %.0f %.0f]",
        imu_rotation_[0], imu_rotation_[1], imu_rotation_[2],
        imu_rotation_[3], imu_rotation_[4], imu_rotation_[5],
        imu_rotation_[6], imu_rotation_[7], imu_rotation_[8]);
}

void IpcBridgeNode::setup_publishers()
{
    if (publish_imu_)
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu_raw", 2000);
    if (publish_ackermann_)
        ackermann_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/ackermann/odom_raw", 200);
    if (publish_gps_)
        gps_hdt_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>("/gps/heading_raw", 50);
    // Os publishers de /gps/xyz_raw/gps_<nr> sao criados sob demanda em
    // on_gps_xyz(), pois nao sabemos de antemao quantas antenas/GPS (campo
    // "nr") vao estar ativas.
}

void IpcBridgeNode::setup_tf()
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
}

void IpcBridgeNode::publish_imu_static_tf(const ros::Time &sensor_stamp)
{
    geometry_msgs::TransformStamped tf_imu;
    tf_imu.header.stamp    = carmen_bridge::tf_stamp(sensor_stamp);
    tf_imu.header.frame_id = base_frame_id_;
    tf_imu.child_frame_id  = imu_frame_id_;
    tf_imu.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf_imu);
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

        carmen_xsens_subscribe_xsens_global_quat_message(
            nullptr,
            reinterpret_cast<carmen_handler_t>(xsens_handler),
            CARMEN_SUBSCRIBE_LATEST);

        // // Fallback: IMU do lidar. Fica inscrito o tempo todo mas so publica
        // // quando xsens_is_active() diz que o xsens sumiu/zerou (ver handler).
        // carmen_imu_subscribe_imu_lidar_message(
        //     nullptr,
        //     reinterpret_cast<carmen_handler_t>(lidar_imu_handler),
        //     CARMEN_SUBSCRIBE_LATEST,
        //     g_lidar_imu_sensor_id);

        if (publish_ackermann_)
            carmen_base_ackerman_subscribe_odometry_message(
                nullptr,
                reinterpret_cast<carmen_handler_t>(base_ackerman_odometry_handler),
                CARMEN_SUBSCRIBE_LATEST);

        if (publish_gps_) {
            // carmen_gps_xyz_subscribe_message(
            //     nullptr,
            //     reinterpret_cast<carmen_handler_t>(gps_xyz_handler),
            //     CARMEN_SUBSCRIBE_LATEST);
            carmen_gps_subscribe_nmea_hdt_message(
                nullptr,
                reinterpret_cast<carmen_handler_t>(gps_hdt_handler),
                CARMEN_SUBSCRIBE_LATEST);
        }

        ROS_INFO("IPC dispatch loop iniciado (host=%s).", ipc_host.c_str());

        while (running_) IPC_listenWait(1);
        IPC_disconnect();
    });
}

// ─── Callbacks ────────────────────────────────────────────────────────────────

void IpcBridgeNode::on_imu(const ImuFrame &frame)
{
    const ros::Time stamp = carmen_bridge::carmen_ns_to_ros_time(frame.ros_stamp_ns);
    if (!static_imu_tf_sent_.exchange(true))
        publish_imu_static_tf(stamp);

    static std::atomic<uint64_t> imu_count{0};
    static double imu_t0 = 0.0;
    ++imu_count;
    const double now_s = carmen_bridge::use_sim_time()
        ? stamp.toSec() : ros::Time::now().toSec();
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

void IpcBridgeNode::on_ackermann(const AckermannFrame &frame)
{
    static std::atomic<uint64_t> ack_count{0};
    ++ack_count;
    ROS_INFO_THROTTLE(1.0,
        "[DBG-ACKERMANN] msgs/s=%lu  v=%.3f m/s  phi=%.4f rad  ts_carmen=%.3f",
        static_cast<unsigned long>(ack_count.exchange(0)),
        frame.v, frame.phi, frame.timestamp);

    if (!publish_ackermann_) return;

    // TwistStamped: linear.x = v (m/s), angular.z = phi (rad, esterçamento cru).
    // A integração do modelo de bicicleta (com a distância entre eixos) é feita
    // no SC-LIO-SAM para manter a distância entre eixos num único arquivo de config.
    geometry_msgs::TwistStamped msg;
    msg.header.stamp    = carmen_bridge::carmen_ns_to_ros_time(frame.ros_stamp_ns);
    msg.header.frame_id = base_frame_id_;
    msg.twist.linear.x  = frame.v;
    msg.twist.angular.z = frame.phi;
    ackermann_pub_.publish(msg);
}

void IpcBridgeNode::on_gps_xyz(const GpsXyzFrame &frame)
{
    if (!publish_gps_) return;

    // So descarta o que o proprio receptor ja reporta como sem fix.
    // Filtro/fusao de verdade fica pro no de correcao.
    if (!frame.valid) return;

    // Uma publisher por antena/gps (campo "nr"), criada sob demanda —
    // assim o no de correcao pode assinar cada uma separadamente.
    auto it = gps_xyz_pubs_.find(frame.nr);
    if (it == gps_xyz_pubs_.end())
    {
        std::string topic = "/gps/xyz_raw/gps_" + std::to_string(frame.nr);
        it = gps_xyz_pubs_.emplace(
            frame.nr, nh_.advertise<nav_msgs::Odometry>(topic, 50)).first;
        ROS_INFO("Novo publisher GPS criado: %s (nr=%d)", topic.c_str(), frame.nr);
    }
    it->second.publish(gps_xyz_to_msg(frame));
}

void IpcBridgeNode::on_gps_hdt(const GpsHdtFrame &frame)
{
    if (!publish_gps_ || !frame.valid) return;
    gps_hdt_pub_.publish(gps_hdt_to_msg(frame));
}

// ─── Conversões ──────────────────────────────────────────────────────────────

sensor_msgs::Imu
IpcBridgeNode::imu_to_msg(const ImuFrame &f)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = carmen_bridge::carmen_to_ros_time(f.timestamp);
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

nav_msgs::Odometry
IpcBridgeNode::gps_xyz_to_msg(const GpsXyzFrame &f)
{
    nav_msgs::Odometry msg;
    msg.header.stamp    = carmen_bridge::carmen_ns_to_ros_time(f.ros_stamp_ns);
    msg.header.frame_id = gps_frame_id_;                 // "utm"
    msg.child_frame_id  = "gps_" + std::to_string(f.nr);

    msg.pose.pose.position.x = f.x;
    msg.pose.pose.position.y = f.y;
    msg.pose.pose.position.z = f.z;

    // theta cru do proprio GPS (nao é o heading corrigido do HDT).
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, f.theta);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // Covariancia so reflete o gps_quality que o proprio receptor reporta
    // (indicador de fix tipo NMEA GGA). Isso NAO é fusao/correcao — é so
    // repassar a confianca que o sensor ja da, pro no de correcao usar.
    double sigma_xy;
    switch (f.gps_quality)
    {
        case 4:  sigma_xy = 0.02; break;  // RTK fixed
        case 5:  sigma_xy = 0.20; break;  // RTK float
        case 2:  sigma_xy = 1.00; break;  // DGPS
        case 1:  sigma_xy = 5.00; break;  // GPS simples (sem correcao)
        default: sigma_xy = 50.0; break;  // qualidade desconhecida/baixa
    }
    for (auto &c : msg.pose.covariance) c = 0.0;
    msg.pose.covariance[0]  = sigma_xy * sigma_xy;               // xx
    msg.pose.covariance[7]  = sigma_xy * sigma_xy;               // yy
    msg.pose.covariance[14] = (sigma_xy * 3.0) * (sigma_xy * 3.0); // zz (Z do GPS é sempre pior)
    msg.pose.covariance[35] = 0.05;                              // yaw, fixo por enquanto

    return msg;
}

geometry_msgs::QuaternionStamped
IpcBridgeNode::gps_hdt_to_msg(const GpsHdtFrame &f)
{
    geometry_msgs::QuaternionStamped msg;
    msg.header.stamp    = carmen_bridge::carmen_ns_to_ros_time(f.ros_stamp_ns);
    msg.header.frame_id = gps_frame_id_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, f.heading);
    msg.quaternion.x = q.x();
    msg.quaternion.y = q.y();
    msg.quaternion.z = q.z();
    msg.quaternion.w = q.w();

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