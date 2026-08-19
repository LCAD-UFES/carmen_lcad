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
 *   publica /odometry/gps           (nav_msgs/Odometry) — o que o SC-LIO-SAM consome
 *
 * FIX: esse nó NÃO ancora mais em frame nenhum (não sabe onde fica o "odom"
 * do LIO-SAM, nem tenta). Antes ele correlacionava a pose global (UTM) do
 * /car com a odometria local do LIO-SAM pra travar uma âncora UTM->odom uma
 * vez por sessão — só que essa odometria local só é confiável desde o boot
 * durante MAPEAMENTO (está sendo construída do zero, self-consistente); em
 * localizationMode ela começa errada (semente do lio_sam/initialPose*) até
 * o robô encaixar no mapa (Scan Context / clique no RViz), então a âncora
 * saía errada bem na hora que mais precisava estar certa — um problema de
 * ovo-e-galinha. Quem faz a conversão pro frame do mapa agora é o próprio
 * SC-LIO-SAM (mapOptmization.cpp: gpsHandler + arquivo gps_utm_anchor.txt,
 * calculado contra a pose SLAM — sempre confiável em mapeamento — e salvo
 * junto com o resto do mapa; carregado de volta em localizationMode).
 *
 * Algoritmo (por antena, a cada mensagem nova):
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
 *   5. Publica a pose final CRUA (frame gps_output_frame_id_, "utm"), sem
 *      nenhuma ancoragem.
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

    void try_publish_fused(const ros::Time &trigger_stamp);
    bool current_heading(const ros::Time &now, double &theta_out) const;

    // Diagnostico de calibracao: acumula (rumo reportado - direcao de percurso)
    // enquanto o veiculo anda em linha reta. Ver heading_offset_ no .cpp.
    void update_heading_offset_estimate(const ros::Time &stamp,
                                        double raw_theta,
                                        double x, double y);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::vector<GpsAntenna> antennas_;
    ros::Subscriber heading_sub_;
    ros::Subscriber ackermann_sub_;
    ros::Publisher  gps_odom_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

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
    std::string      gps_output_frame_id_;   // frame_id publicado no Odometry ("utm", cru)
    std::string      car_frame_id_;
    std::string      ipc_host_;
    double gps_max_age_;
    double heading_max_age_;
    double v_max_age_;
    bool   use_latency_compensation_;
    std::atomic<bool> antenna_tfs_sent_{false};

    // ─── Calibracao do rumo (montagem das antenas) ────────────────────────
    //
    // Somado ao rumo cru antes de qualquer uso. Com UMA antena (IARA) o rumo
    // e' course-over-ground e o offset e' zero. Com DUAS antenas o rumo vem da
    // linha de base entre elas e carrega o offset de montagem -- no Ype medimos
    // -142.4 graus constante (+-1.2 graus na corrida inteira).
    double heading_offset_;             // ~heading_offset_deg, em rad
    bool   estimate_heading_offset_;    // ~estimate_heading_offset: so' mede e loga

    // Acumuladores da estimativa (media circular ponderada por distancia).
    double    hoff_sum_sin_ = 0.0, hoff_sum_cos_ = 0.0, hoff_wsum_ = 0.0;
    size_t    hoff_samples_ = 0;
    bool      hoff_have_prev_ = false;
    double    hoff_prev_x_ = 0.0, hoff_prev_y_ = 0.0, hoff_prev_theta_ = 0.0;
    ros::Time hoff_prev_stamp_;
    double    hoff_min_baseline_;        // ~heading_offset_min_baseline [m]
};

} // namespace carmen_bridge
