#pragma once

/*
 * gps_localization_node.hpp  (ROS1 / Noetic)
 *
 * Nó separado do ipc_bridge_node. Não fala IPC continuamente — só usa o IPC
 * uma vez, na inicialização, pra ler do param_daemon do CARMEN o offset de
 * cada antena/GPS em relação ao /car (grupo "gps", chaves "nmea_<nr>_*" —
 * já relativo ao centro do veículo, não à sensor_board_1).
 *
 * Depois disso, o nó só fala ROS:
 *   assina  /gps/xyz_raw/gps_<nr>   (nav_msgs/Odometry, N antenas)
 *   assina  /gps/heading_raw        (geometry_msgs/QuaternionStamped)
 *   assina  /ackermann/odom_raw     (geometry_msgs/TwistStamped, só usa v)
 *   assina  ~lio_local_odom_topic   (nav_msgs/Odometry, default "odometry/imu" —
 *                                    odometria local do LIO-SAM, só pra ancoragem)
 *   publica /odometry/gps           (nav_msgs/Odometry) — o que o SC-LIO-SAM consome
 *
 * Algoritmo (por antena, a cada mensagem nova):
 *   0. Na primeira amostra correlacionada com a odometria local do LIO-SAM
 *      (~lio_local_odom_topic, default "odometry/imu"), calcula uma âncora
 *      fixa T_odom_utm alinhando a pose global (UTM) do /car com a pose que
 *      o LIO-SAM já tem pro mesmo instante no frame "odom". Essa âncora fica
 *      travada pelo resto da sessão (mesma lógica do "datum" do
 *      navsat_transform_node do robot_localization) e também é publicada
 *      como TF estática "utm"->"odom", útil pra georreferenciar o mapa salvo
 *      depois. Enquanto a âncora não existe, o nó NÃO publica em
 *      lio_sam_gps_topic_ (evita mandar GPS factor em frame errado pro
 *      LIO-SAM).
 *   1. Pose global da antena (x,y,z do GPS) + heading atual do veículo vira
 *      uma transformação global->gps.
 *   2. Compõe com a transformação fixa gps->car (braço de alavanca, inversa
 *      da pose da antena em relação ao /car) pra obter a posição global do
 *      /car — exatamente o que get_car_pose_from_gps_pose fazia no CARMEN.
 *   3. Compensa a latência entre o timestamp do GPS e agora, propagando a
 *      posição com a velocidade atual (v * dt, linha reta — versão
 *      simplificada do modelo de bicicleta original).
 *   4. Se mais de uma antena estiver ativa, funde as posições já corrigidas
 *      por média ponderada pelo inverso da covariância (a que o próprio
 *      ipc_bridge_node já embute no Odometry cru, a partir do gps_quality).
 *   5. Aplica a âncora (passo 0) pra publicar a pose final já no frame
 *      "odom" do LIO-SAM, não em UTM absoluto.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <string>
#include <memory>
#include <atomic>

namespace carmen_bridge {

// Uma antena/GPS configurada: guarda a transformação fixa (braço de
// alavanca) e a última amostra recebida em /gps/xyz_raw/gps_<nr>.
struct GpsAntenna
{
    int nr = -1;

    // Pose da antena em relação ao /car (braço de alavanca), lida do
    // param_daemon. T_gps_car é a inversa — origem do /car expressa no
    // frame da antena — que é o que entra direto na composição do algoritmo.
    tf2::Transform T_car_gps;
    tf2::Transform T_gps_car;

    ros::Subscriber sub;

    bool      has_sample = false;
    ros::Time stamp;
    double    x = 0.0, y = 0.0, z = 0.0;
    double    theta = 0.0;          // heading cru reportado pela própria mensagem GPS
    double    cov_xx = 1e6, cov_yy = 1e6, cov_zz = 1e6;
};

class GpsLocalizationNode
{
public:
    GpsLocalizationNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
    void declare_parameters();
    void load_antenna_offsets_from_carmen();
    void setup_subscribers();
    void publish_antenna_tfs(const ros::Time &stamp);

    void on_gps_xyz(int nr, const nav_msgs::Odometry::ConstPtr &msg);
    void on_heading(const geometry_msgs::QuaternionStamped::ConstPtr &msg);
    void on_ackermann(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void on_lio_odom(const nav_msgs::Odometry::ConstPtr &msg);

    void try_publish_fused(const ros::Time &trigger_stamp);
    bool current_heading(const ros::Time &now, double &theta_out) const;
    bool try_compute_anchor(const ros::Time &trigger_stamp, const tf2::Transform &T_utm_car);
    void publish_utm_anchor_tf(const ros::Time &stamp);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::vector<GpsAntenna> antennas_;
    ros::Subscriber heading_sub_;
    ros::Subscriber ackermann_sub_;
    ros::Subscriber lio_odom_sub_;
    ros::Publisher  gps_odom_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // Odometria local do LIO-SAM (frame "odom"), usada só uma vez pra travar
    // a âncora UTM->odom. Depois de ancorado, não precisamos mais dela.
    nav_msgs::Odometry last_lio_odom_;
    bool      has_lio_odom_ = false;
    ros::Time last_lio_odom_stamp_;

    // Âncora: pose da origem UTM expressa no frame "odom" do LIO-SAM,
    // calculada uma vez (equivalente ao "datum" do navsat_transform_node) e
    // travada pelo resto da sessão. T_odom_utm_ * T_utm_car = T_odom_car.
    bool           anchor_valid_ = false;
    tf2::Transform T_odom_utm_;

    // Estado dinâmico do veículo (compensação de latência + heading quando
    // não há HDT válido).
    double    last_v_ = 0.0;
    ros::Time last_v_stamp_;

    double    last_heading_ = 0.0;
    bool      heading_from_hdt_ = false;
    ros::Time last_heading_stamp_;

    // ─── Parâmetros ───────────────────────────────────────────────────────
    std::vector<int> gps_nrs_;          // quais antenas assinar (~gps_nrs, default [1])
    std::string      lio_sam_gps_topic_;
    std::string      gps_output_frame_id_;   // nome do frame global/UTM (só usado na TF da âncora)
    std::string      odom_frame_id_;         // frame local do LIO-SAM -- é o que sai no Odometry publicado
    std::string      lio_local_odom_topic_;  // odometria local do LIO-SAM usada pra calcular a âncora
    std::string      car_frame_id_;
    std::string      ipc_host_;
    double gps_max_age_;
    double heading_max_age_;
    double v_max_age_;
    double anchor_max_age_;
    bool   use_latency_compensation_;
    std::atomic<bool> antenna_tfs_sent_{false};
};

} // namespace carmen_bridge