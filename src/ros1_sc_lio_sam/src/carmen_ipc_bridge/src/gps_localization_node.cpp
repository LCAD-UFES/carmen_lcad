/*
 * gps_localization_node.cpp  (ROS1 / Noetic)
 *
 * Ver gps_localization_node.hpp pra explicação do algoritmo.
 */

#include "carmen_ipc_bridge/gps_localization_node.hpp"
#include "carmen_ipc_bridge/carmen_time.hpp"
#include <carmen/carmen.h>
#include <carmen/global.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

using carmen_bridge::GpsLocalizationNode;

// ─── Constructor ──────────────────────────────────────────────────────────

GpsLocalizationNode::GpsLocalizationNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh)
{
    declare_parameters();

    // O IPC só é usado aqui, uma vez, pra ler os offsets das antenas do
    // param_daemon. Depois disso o nó só fala ROS (sem thread de IPC
    // rodando, sem IPC_listenWait — não assinamos nenhuma mensagem IPC).
    {
        std::string prog = "gps_localization_node";
        std::string flag = "-central_host";
        std::vector<char*> argv_vec = {
            const_cast<char*>(prog.c_str()),
            const_cast<char*>(flag.c_str()),
            const_cast<char*>(ipc_host_.c_str())
        };
        carmen_ipc_initialize(static_cast<int>(argv_vec.size()), argv_vec.data());
    }

    load_antenna_offsets_from_carmen();

    IPC_disconnect();

    gps_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(lio_sam_gps_topic_, 50);

    setup_subscribers();

    ROS_INFO(
        "gps_localization_node iniciado: %zu antena(s), publicando em %s (frame=%s, child_frame=%s) "
        "assim que ancorar com %s",
        antennas_.size(), lio_sam_gps_topic_.c_str(),
        odom_frame_id_.c_str(), car_frame_id_.c_str(), lio_local_odom_topic_.c_str());
}

// ─── Inicialização ────────────────────────────────────────────────────────

void GpsLocalizationNode::declare_parameters()
{
    std::vector<int> default_nrs = {1};
    pnh_.param<std::vector<int>>("gps_nrs", gps_nrs_, default_nrs);

    pnh_.param<std::string>("lio_sam_gps_topic",   lio_sam_gps_topic_,   "/odometry/gps");
    pnh_.param<std::string>("gps_output_frame_id", gps_output_frame_id_, "utm");
    pnh_.param<std::string>("odom_frame_id",        odom_frame_id_,       "odom");
    pnh_.param<std::string>("lio_local_odom_topic", lio_local_odom_topic_, "odometry/imu");
    pnh_.param<std::string>("car_frame_id",        car_frame_id_,        "base_link");
    pnh_.param<std::string>("ipc_host",            ipc_host_,            "localhost");

    pnh_.param<double>("gps_max_age",     gps_max_age_,     0.3);
    pnh_.param<double>("heading_max_age", heading_max_age_, 1.0);
    pnh_.param<double>("v_max_age",       v_max_age_,       1.0);
    pnh_.param<double>("anchor_max_age",  anchor_max_age_,  0.3);
    pnh_.param<bool>  ("use_latency_compensation", use_latency_compensation_, true);

    if (gps_nrs_.empty()) {
        //ROS_WARN("~gps_nrs vazio — usando antena 1 como default.");
        gps_nrs_ = {1};
    }
}

// Lê o offset de cada antena GPS em relação ao /car (centro do veículo).
// Nome de grupo/chave CONFIRMADO em carmen_localize_ackerman_read_parameters()
// (localize_ackerman_core.cpp):
//   antena GPS nr: grupo "gps", chaves "nmea_<nr>_x","nmea_<nr>_y","nmea_<nr>_z",
//                   "nmea_<nr>_roll","nmea_<nr>_pitch","nmea_<nr>_yaw"
//                   (confirmado pra nr=1 e nr=2 — gps_pose_in_the_car / gps2_pose_in_the_car,
//                   ou seja, já é relativo ao carro, NÃO à sensor_board_1;
//                   se usar nr>=3, confirma se existe nmea_3_* etc no param_daemon antes)
void GpsLocalizationNode::load_antenna_offsets_from_carmen()
{
    carmen_param_allow_unfound_variables(1);

    // NOTA: os offsets "gps"/"nmea_<nr>_*" ja vem do param_daemon como pose
    // da antena em relacao ao /car (centro do veiculo / base_link) -- o
    // proprio nome interno do CARMEN pra nr=1 e nr=2 e "gps_pose_in_the_car" /
    // "gps2_pose_in_the_car". Ou seja, NAO sao relativos a sensor_board_1 e
    // NAO devem ser compostos com T_car_board -- isso aplicaria o braco de
    // alavanca da board em cima de um offset que ja e relativo ao carro.

    for (int nr : gps_nrs_)
    {
        double gx = 0.0, gy = 0.0, gz = 0.0;
        double groll = 0.0, gpitch = 0.0, gyaw = 0.0;

        // Fallback via ROS param (~gps<nr>_offset_x etc), sobrescrito pelo
        // param_daemon se a chave "gps"/"nmea_<nr>_*" existir la. Util pra
        // nr>=3 ainda nao confirmado, ou pra testar offset sem mexer no ini.
        std::string ros_prefix = "gps" + std::to_string(nr) + "_offset_";
        pnh_.param<double>(ros_prefix + "x",     gx,     0.0);
        pnh_.param<double>(ros_prefix + "y",     gy,     0.0);
        pnh_.param<double>(ros_prefix + "z",     gz,     0.0);
        pnh_.param<double>(ros_prefix + "roll",  groll,  0.0);
        pnh_.param<double>(ros_prefix + "pitch", gpitch, 0.0);
        pnh_.param<double>(ros_prefix + "yaw",   gyaw,   0.0);

        std::string carmen_prefix = "nmea_" + std::to_string(nr) + "_";
        std::string key_x = carmen_prefix + "x";
        std::string key_y = carmen_prefix + "y";
        std::string key_z = carmen_prefix + "z";
        std::string key_roll  = carmen_prefix + "roll";
        std::string key_pitch = carmen_prefix + "pitch";
        std::string key_yaw   = carmen_prefix + "yaw";

        carmen_param_t gps_params[] = {
            {(char *) "gps", const_cast<char *>(key_x.c_str()),     CARMEN_PARAM_DOUBLE, &gx,     0, NULL},
            {(char *) "gps", const_cast<char *>(key_y.c_str()),     CARMEN_PARAM_DOUBLE, &gy,     0, NULL},
            {(char *) "gps", const_cast<char *>(key_z.c_str()),     CARMEN_PARAM_DOUBLE, &gz,     0, NULL},
            {(char *) "gps", const_cast<char *>(key_roll.c_str()),  CARMEN_PARAM_DOUBLE, &groll,  0, NULL},
            {(char *) "gps", const_cast<char *>(key_pitch.c_str()), CARMEN_PARAM_DOUBLE, &gpitch, 0, NULL},
            {(char *) "gps", const_cast<char *>(key_yaw.c_str()),   CARMEN_PARAM_DOUBLE, &gyaw,   0, NULL},
        };
        carmen_param_install_params(0, nullptr, gps_params,
                                   sizeof(gps_params) / sizeof(gps_params[0]));

        if (gx == 0.0 && gy == 0.0 && gz == 0.0 &&
            groll == 0.0 && gpitch == 0.0 && gyaw == 0.0)
        {
            //ROS_WARN(
            //    "GPS nr=%d: offset veio todo zerado do param_daemon (grupo "
            //    "\"gps\", chave \"nmea_%d_*\") -- confere com: "
            //    "grep gps_nmea_%d_ no seu .ini",
            //    nr, nr, nr);
        }

        GpsAntenna ant;
        ant.nr = nr;

        // T_car_gps: pose da antena relativa ao /car (braço de alavanca),
        // lida direto do param_daemon (grupo "gps", chave "nmea_<nr>_*") --
        // já é relativa ao centro do veículo, não à sensor_board_1.
        // T_gps_car é a inversa, usada direto na composição do algoritmo
        // (equivalente ao "gps_to_car" do CARMEN).
        tf2::Quaternion q_cg;
        q_cg.setRPY(groll, gpitch, gyaw);
        ant.T_car_gps = tf2::Transform(q_cg, tf2::Vector3(gx, gy, gz));
        ant.T_gps_car = ant.T_car_gps.inverse();

        double dist = ant.T_car_gps.getOrigin().length();
        ROS_INFO(
            "GPS nr=%d: offset em relacao ao /car = (%.3f, %.3f, %.3f) m, distancia=%.3f m",
            nr, ant.T_car_gps.getOrigin().x(), ant.T_car_gps.getOrigin().y(),
            ant.T_car_gps.getOrigin().z(), dist);

        antennas_.push_back(std::move(ant));
    }
}

// Publica uma TF estática car_frame_id_ -> "gps_<nr>" por antena, com o
// mesmo braço de alavanca (T_car_gps) usado no algoritmo de correção. Serve
// pra conferir visualmente no RViz se o offset lido do param_daemon bate com
// a posição física real da antena (em vez de só olhar os números no log).
void GpsLocalizationNode::publish_antenna_tfs(const ros::Time &stamp)
{
    if (!static_tf_broadcaster_)
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    const ros::Time tf_time = carmen_bridge::tf_stamp(stamp);
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const auto &a : antennas_)
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp    = tf_time;
        t.header.frame_id = car_frame_id_;                  // "base_link"
        t.child_frame_id  = "gps_" + std::to_string(a.nr);   // mesmo nome do child_frame_id do Odometry cru

        const tf2::Vector3 &origin = a.T_car_gps.getOrigin();
        t.transform.translation.x = origin.x();
        t.transform.translation.y = origin.y();
        t.transform.translation.z = origin.z();
        t.transform.rotation = tf2::toMsg(a.T_car_gps.getRotation());

        transforms.push_back(t);
        ROS_INFO("TF estatica %s->%s publicada (braco de alavanca da antena)",
                 car_frame_id_.c_str(), t.child_frame_id.c_str());
    }

    if (!transforms.empty())
        static_tf_broadcaster_->sendTransform(transforms);
}

void GpsLocalizationNode::setup_subscribers()
{
    for (size_t i = 0; i < antennas_.size(); ++i)
    {
        int nr = antennas_[i].nr;
        std::string topic = "/gps/xyz_raw/gps_" + std::to_string(nr);
        antennas_[i].sub = nh_.subscribe<nav_msgs::Odometry>(
            topic, 20,
            [this, nr](const nav_msgs::Odometry::ConstPtr &msg) { on_gps_xyz(nr, msg); });
        ROS_INFO("Assinando %s (antena nr=%d)", topic.c_str(), nr);
    }

    heading_sub_ = nh_.subscribe("/gps/heading_raw", 20, &GpsLocalizationNode::on_heading, this);
    ackermann_sub_ = nh_.subscribe("/ackermann/odom_raw", 20, &GpsLocalizationNode::on_ackermann, this);
    lio_odom_sub_ = nh_.subscribe(lio_local_odom_topic_, 50, &GpsLocalizationNode::on_lio_odom, this);
    ROS_INFO("Assinando %s (odometria local do LIO-SAM, pra ancoragem UTM->odom)",
             lio_local_odom_topic_.c_str());
}

// ─── Callbacks ────────────────────────────────────────────────────────────

void GpsLocalizationNode::on_gps_xyz(int nr, const nav_msgs::Odometry::ConstPtr &msg)
{
    auto it = std::find_if(antennas_.begin(), antennas_.end(),
        [nr](const GpsAntenna &a) { return a.nr == nr; });
    if (it == antennas_.end()) return;

    if (!antenna_tfs_sent_.exchange(true))
        publish_antenna_tfs(msg->header.stamp);

    it->has_sample = true;
    it->stamp = msg->header.stamp;
    it->x = msg->pose.pose.position.x;
    it->y = msg->pose.pose.position.y;
    it->z = msg->pose.pose.position.z;
    it->theta = tf2::getYaw(msg->pose.pose.orientation);
    it->cov_xx = std::max(msg->pose.covariance[0],  1e-6);
    it->cov_yy = std::max(msg->pose.covariance[7],  1e-6);
    it->cov_zz = std::max(msg->pose.covariance[14], 1e-6);

    try_publish_fused(msg->header.stamp);
}

void GpsLocalizationNode::on_heading(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
    last_heading_ = tf2::getYaw(msg->quaternion);
    last_heading_stamp_ = msg->header.stamp;
    heading_from_hdt_ = true;
}

void GpsLocalizationNode::on_ackermann(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    last_v_ = msg->twist.linear.x;
    last_v_stamp_ = msg->header.stamp;
}

// Guarda a última odometria local do LIO-SAM (frame "odom"). Só serve pra
// calcular a âncora (try_compute_anchor) -- depois de anchor_valid_==true
// não precisamos mais dela, mas não custa nada continuar recebendo.
void GpsLocalizationNode::on_lio_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
    last_lio_odom_ = *msg;
    last_lio_odom_stamp_ = msg->header.stamp;
    has_lio_odom_ = true;
}

// ─── Fusão / publicação ───────────────────────────────────────────────────

// Heading atual do veiculo pra rotacionar o braço de alavanca: preferencia
// pro HDT (heading verdadeiro, nao tem drift), senao cai pro theta cru mais
// recente entre as antenas ativas.
bool GpsLocalizationNode::current_heading(const ros::Time &now, double &theta_out) const
{
    if (heading_from_hdt_ && (now - last_heading_stamp_).toSec() < heading_max_age_)
    {
        theta_out = last_heading_;
        return true;
    }

    bool found = false;
    ros::Time newest(0);
    for (const auto &a : antennas_)
    {
        if (!a.has_sample) continue;
        if ((now - a.stamp).toSec() >= gps_max_age_) continue;
        if (!found || a.stamp > newest)
        {
            newest = a.stamp;
            theta_out = a.theta;
            found = true;
        }
    }
    return found;
}

// Calcula a âncora T_odom_utm_ (pose da origem UTM expressa no frame "odom"
// do LIO-SAM), correlacionando a pose global (UTM) do /car agora com a
// última odometria local recebida de lio_local_odom_topic_. Só roda uma vez
// -- depois de anchor_valid_==true essa função nunca mais é chamada.
// Mesma ideia do "datum" do navsat_transform_node (robot_localization): usa
// a primeira amostra correlacionada entre GPS e odometria local como
// referência, em vez de assumir que ambos começam exatamente na mesma pose.
bool GpsLocalizationNode::try_compute_anchor(const ros::Time &trigger_stamp, const tf2::Transform &T_utm_car)
{
    if (!has_lio_odom_) return false;

    double age = (trigger_stamp - last_lio_odom_stamp_).toSec();
    if (std::fabs(age) > anchor_max_age_) return false;  // odom do LIO-SAM velha/futura demais

    tf2::Quaternion q_odom(
        last_lio_odom_.pose.pose.orientation.x, last_lio_odom_.pose.pose.orientation.y,
        last_lio_odom_.pose.pose.orientation.z, last_lio_odom_.pose.pose.orientation.w);
    tf2::Transform T_odom_car(
        q_odom,
        tf2::Vector3(last_lio_odom_.pose.pose.position.x,
                     last_lio_odom_.pose.pose.position.y,
                     last_lio_odom_.pose.pose.position.z));

    T_odom_utm_ = T_odom_car * T_utm_car.inverse();
    anchor_valid_ = true;

    const tf2::Vector3 &t = T_odom_utm_.getOrigin();
    //ROS_WARN(
    //    "GPS localization: ANCORA calculada (uma vez por sessao) -- "
    //    "offset %s->%s = (%.3f, %.3f, %.3f) m, yaw=%.4f rad (idade da odom do LIO-SAM usada: %.3f s). "
    //    "A partir de agora %s sai no frame '%s'.",
    //    gps_output_frame_id_.c_str(), odom_frame_id_.c_str(),
    //    t.x(), t.y(), t.z(), tf2::getYaw(T_odom_utm_.getRotation()), age,
    //    lio_sam_gps_topic_.c_str(), odom_frame_id_.c_str());

    publish_utm_anchor_tf(trigger_stamp);
    return true;
}

// Publica TF estatica gps_output_frame_id_ ("utm") -> odom_frame_id_
// ("odom") com a ancora calculada. Serve pra: (1) conferir no RViz que a
// ancora bateu com a pose real; (2) georreferenciar o mapa salvo depois --
// qualquer ponto do GlobalMap.pcd (que fica no frame "odom"/"map" do
// LIO-SAM) pode ser levado de volta pra UTM absoluto aplicando essa TF.
void GpsLocalizationNode::publish_utm_anchor_tf(const ros::Time &stamp)
{
    if (!static_tf_broadcaster_)
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    tf2::Transform T_utm_odom = T_odom_utm_.inverse();

    geometry_msgs::TransformStamped t;
    t.header.stamp    = carmen_bridge::tf_stamp(stamp);
    t.header.frame_id = gps_output_frame_id_;   // "utm"
    t.child_frame_id  = odom_frame_id_;         // "odom"
    t.transform.translation.x = T_utm_odom.getOrigin().x();
    t.transform.translation.y = T_utm_odom.getOrigin().y();
    t.transform.translation.z = T_utm_odom.getOrigin().z();
    t.transform.rotation      = tf2::toMsg(T_utm_odom.getRotation());

    static_tf_broadcaster_->sendTransform(t);
    ROS_INFO("TF estatica %s->%s publicada (ancora GPS -- guarda esses numeros pra georreferenciar o mapa depois)",
             gps_output_frame_id_.c_str(), odom_frame_id_.c_str());
}

void GpsLocalizationNode::try_publish_fused(const ros::Time &trigger_stamp)
{
    double theta;
    if (!current_heading(trigger_stamp, theta))
        return;  // sem heading confiavel ainda (nem HDT, nem GPS valido) -- nao publica

    double v = 0.0;
    if (use_latency_compensation_ && (trigger_stamp - last_v_stamp_).toSec() < v_max_age_)
        v = last_v_;

    tf2::Quaternion q_heading;
    q_heading.setRPY(0.0, 0.0, theta);

    double sum_wx = 0.0, sum_wx_x = 0.0;
    double sum_wy = 0.0, sum_wy_y = 0.0;
    double sum_wz = 0.0, sum_wz_z = 0.0;
    int n_used = 0;

    for (const auto &a : antennas_)
    {
        if (!a.has_sample) continue;
        double age = (trigger_stamp - a.stamp).toSec();
        if (age >= gps_max_age_) continue;

        // 1) posicao global da antena -> posicao global do /car (braço de
        //    alavanca rotacionado pelo heading atual).
        tf2::Transform global_to_gps(q_heading, tf2::Vector3(a.x, a.y, a.z));
        tf2::Transform global_to_car = global_to_gps * a.T_gps_car;
        tf2::Vector3   car_pos = global_to_car.getOrigin();

        // 2) compensa a latencia entre o timestamp do GPS e agora, andando
        //    em linha reta com a velocidade atual (versao simplificada do
        //    modelo de bicicleta original).
        double dt = use_latency_compensation_ ? std::max(0.0, std::min(age, 1.0)) : 0.0;
        double cx = car_pos.x() + v * dt * std::cos(theta);
        double cy = car_pos.y() + v * dt * std::sin(theta);
        double cz = car_pos.z();

        // 3) acumula pro fusao ponderada pelo inverso da covariancia.
        double wx = 1.0 / a.cov_xx;
        double wy = 1.0 / a.cov_yy;
        double wz = 1.0 / a.cov_zz;
        sum_wx += wx; sum_wx_x += wx * cx;
        sum_wy += wy; sum_wy_y += wy * cy;
        sum_wz += wz; sum_wz_z += wz * cz;
        ++n_used;
    }

    if (n_used == 0) return;

    double fx = sum_wx_x / sum_wx;
    double fy = sum_wy_y / sum_wy;
    double fz = sum_wz_z / sum_wz;
    double var_x = 1.0 / sum_wx;
    double var_y = 1.0 / sum_wy;
    double var_z = 1.0 / sum_wz;

    // Pose global (UTM) do /car agora -- é o que ia direto no Odometry antes.
    tf2::Transform T_utm_car(q_heading, tf2::Vector3(fx, fy, fz));

    // Sem ancora ainda: nao publica. Publicar em UTM absoluto quebraria o
    // addGPSFactor do LIO-SAM (ele soma esses x,y,z direto na pose local,
    // que comeca perto de 0,0,0 -- ver conversa sobre o gpsTopic).
    if (!anchor_valid_ && !try_compute_anchor(trigger_stamp, T_utm_car))
    {
        //ROS_WARN_THROTTLE(2.0,
        //    "GPS localization: aguardando odometria local do LIO-SAM em '%s' pra ancorar o frame -- "
        //    "%s ainda NAO esta sendo publicado.",
        //    lio_local_odom_topic_.c_str(), lio_sam_gps_topic_.c_str());
        return;
    }

    // Aplica a ancora: pose do /car no frame local "odom" do LIO-SAM.
    tf2::Transform T_odom_car = T_odom_utm_ * T_utm_car;

    // Rotaciona a covariancia x/y (que foi calculada em eixos UTM/east-north)
    // pelo yaw da ancora, senao var_x/var_y ficam com os eixos trocados em
    // relacao ao frame odom sempre que o heading inicial != 0. z nao muda
    // (ancora nao tem roll/pitch, mesma simplificacao ja usada no resto do
    // no -- terreno assumido plano).
    double yaw_offset = tf2::getYaw(T_odom_utm_.getRotation());
    double cos_yaw = std::cos(yaw_offset), sin_yaw = std::sin(yaw_offset);
    double var_x_odom = var_x * cos_yaw * cos_yaw + var_y * sin_yaw * sin_yaw;
    double var_y_odom = var_x * sin_yaw * sin_yaw + var_y * cos_yaw * cos_yaw;

    nav_msgs::Odometry out;
    out.header.stamp    = trigger_stamp;
    out.header.frame_id = odom_frame_id_;   // "odom" -- local, alinhado com o LIO-SAM (nao mais UTM)
    out.child_frame_id  = car_frame_id_;    // "base_link"

    const tf2::Vector3 &pos_odom = T_odom_car.getOrigin();
    out.pose.pose.position.x = pos_odom.x();
    out.pose.pose.position.y = pos_odom.y();
    out.pose.pose.position.z = pos_odom.z();
    out.pose.pose.orientation = tf2::toMsg(T_odom_car.getRotation());

    for (auto &cov : out.pose.covariance) cov = 0.0;
    out.pose.covariance[0]  = var_x_odom;
    out.pose.covariance[7]  = var_y_odom;
    out.pose.covariance[14] = var_z;
    // Variancia do heading: menor (mais confiavel) se veio do HDT.
    out.pose.covariance[35] = heading_from_hdt_ ? 0.01 : 0.10;

    gps_odom_pub_.publish(out);
}

// ─── main ─────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_localization_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    GpsLocalizationNode node(nh, pnh);
    ros::spin();
    return 0;
}