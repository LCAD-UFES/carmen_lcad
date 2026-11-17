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
#include <fstream>
#include <sstream>

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
        "gps_localization_node iniciado: %zu antena(s), publicando em %s (frame=%s, child_frame=%s), "
        "sem ancoragem -- quem converte pro frame do mapa agora e' o SC-LIO-SAM "
        "(gps_utm_anchor.txt, salvo/lido junto com o mapa).",
        antennas_.size(), lio_sam_gps_topic_.c_str(),
        gps_output_frame_id_.c_str(), car_frame_id_.c_str());
}

// ─── Inicialização ────────────────────────────────────────────────────────

void GpsLocalizationNode::declare_parameters()
{
    std::vector<int> default_nrs = {1};
    pnh_.param<std::vector<int>>("gps_nrs", gps_nrs_, default_nrs);

    pnh_.param<std::string>("lio_sam_gps_topic",   lio_sam_gps_topic_,   "/odometry/gps");
    pnh_.param<std::string>("gps_output_frame_id", gps_output_frame_id_, "utm");
    pnh_.param<std::string>("car_frame_id",        car_frame_id_,        "base_link");
    pnh_.param<std::string>("ipc_host",            ipc_host_,            "localhost");

    pnh_.param<double>("gps_max_age",     gps_max_age_,     0.3);
    pnh_.param<double>("heading_max_age", heading_max_age_, 1.0);
    pnh_.param<double>("v_max_age",       v_max_age_,       1.0);
    pnh_.param<bool>  ("use_latency_compensation", use_latency_compensation_, true);

    // ─── Calibracao do rumo ───────────────────────────────────────────────
    //
    // Este no usa o rumo pra DUAS coisas: rotacionar o braco de alavanca
    // antena->base_link (try_publish_fused) e preencher a orientacao da
    // Odometry publicada. Se o rumo estiver com offset de montagem, o braco
    // roda errado e a posicao do carro sai deslocada -- e o deslocamento GIRA
    // junto com o veiculo, o que aparece como "drift na diagonal" rio abaixo,
    // no SC-LIO-SAM.
    //
    // Com UMA antena (IARA) o rumo do gps_xyz e' course-over-ground: bate com a
    // direcao de percurso e o offset e' 0. Com DUAS antenas o rumo vem da linha
    // de base entre elas (gps_xyz.cpp: get_angle_between_gpss(), corrigido por
    // "gps_nmea_<n>_vector_antenna_angle") ou de uma sentenca $GPHDT do
    // receptor -- os dois carregam a orientacao FISICA do par de antenas.
    //
    // Medido no Ype (log de 03/08/2026, Atego 1730, ~4 min): o rumo reportado
    // estava a -142.4 graus da direcao de percurso, com espalhamento de +-1.2
    // graus na corrida inteira. Constante desse jeito = calibracao, nao ruido.
    // O braco horizontal ali e' de 2.84 m, entao 142 graus de erro na rotacao
    // valem ate' 5.4 m de erro de posicao.
    //
    // Default 0.0 = comportamento anterior. Ligue ~estimate_heading_offset pra
    // o no medir sozinho e logar o valor a usar aqui, sem alterar a saida.
    double heading_offset_deg = 0.0;
    pnh_.param<double>("heading_offset_deg", heading_offset_deg, 0.0);
    heading_offset_ = heading_offset_deg * M_PI / 180.0;

    pnh_.param<bool>  ("estimate_heading_offset", estimate_heading_offset_, false);
    pnh_.param<double>("heading_offset_min_baseline", hoff_min_baseline_, 5.0);

    if (heading_offset_deg != 0.0)
        ROS_INFO("\033[1;32mheading_offset_deg = %.2f (%.4f rad) -- somado ao rumo cru "
                 "antes de rotacionar o braco de alavanca.\033[0m",
                 heading_offset_deg, heading_offset_);
    else
        ROS_INFO("heading_offset_deg = 0 -- rumo do GPS usado cru. Em veiculo de DUAS "
                 "antenas confira com ~estimate_heading_offset:=true.");

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

    // CORRECAO: os offsets "gps"/"nmea_<nr>_*" sao relativos a SENSOR_BOARD_1,
    // nao ao /car. O nome interno do CARMEN ("gps_pose_in_the_car") engana; a
    // cadeia real esta em point_cloud_odometry_main.cpp:1418-1431:
    //
    //     car_to_board_pose  <- sensor_board_1_pose
    //     board_to_gps_pose  <- gps_pose_in_the_car     // board -> gps
    //
    // A fisica confirma: gps_nmea_1_z = -0.148. Se fosse relativo ao /car, a
    // antena estaria 15 cm ABAIXO do centro do eixo traseiro, ou seja enterrada.
    // Relativa a board (z = 1.482) da' 1.334 m acima do eixo -- antena de teto.
    //
    // Sem compor com a board o braco de alavanca saia 0.695 m errado em x (o
    // sensor_board_1_x da IARA), o que vira erro sistematico na pose do carro.
    double sb_x = 0.0, sb_y = 0.0, sb_z = 0.0;
    double sb_roll = 0.0, sb_pitch = 0.0, sb_yaw = 0.0;
    {
        carmen_param_t sb_params[] = {
            {(char *) "sensor_board_1", (char *) "x",     CARMEN_PARAM_DOUBLE, &sb_x,     0, NULL},
            {(char *) "sensor_board_1", (char *) "y",     CARMEN_PARAM_DOUBLE, &sb_y,     0, NULL},
            {(char *) "sensor_board_1", (char *) "z",     CARMEN_PARAM_DOUBLE, &sb_z,     0, NULL},
            {(char *) "sensor_board_1", (char *) "roll",  CARMEN_PARAM_DOUBLE, &sb_roll,  0, NULL},
            {(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &sb_pitch, 0, NULL},
            {(char *) "sensor_board_1", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &sb_yaw,   0, NULL},
        };
        carmen_param_install_params(0, nullptr, sb_params,
                                   sizeof(sb_params) / sizeof(sb_params[0]));
    }
    tf2::Quaternion q_sb;
    q_sb.setRPY(sb_roll, sb_pitch, sb_yaw);
    const tf2::Transform T_car_board(q_sb, tf2::Vector3(sb_x, sb_y, sb_z));
    ROS_INFO("sensor_board_1 = (%.3f, %.3f, %.3f) rpy=(%.4f, %.4f, %.4f) -- "
             "composta no braco de alavanca de cada antena.",
             sb_x, sb_y, sb_z, sb_roll, sb_pitch, sb_yaw);

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

        // T_board_gps: pose da antena relativa a sensor_board_1 (e' o que o
        // param_daemon guarda). T_car_gps = T_car_board * T_board_gps.
        // T_gps_car e' a inversa, usada direto na composicao do algoritmo
        // (equivalente ao "gps_to_car" do CARMEN).
        tf2::Quaternion q_bg;
        q_bg.setRPY(groll, gpitch, gyaw);
        const tf2::Transform T_board_gps(q_bg, tf2::Vector3(gx, gy, gz));
        ant.T_car_gps = T_car_board * T_board_gps;
        ant.T_gps_car = ant.T_car_gps.inverse();

        double dist = ant.T_car_gps.getOrigin().length();
        ROS_INFO(
            "GPS nr=%d: board->antena = (%.3f, %.3f, %.3f) | car->antena = (%.3f, %.3f, %.3f) m, distancia=%.3f m",
            nr, gx, gy, gz,
            ant.T_car_gps.getOrigin().x(), ant.T_car_gps.getOrigin().y(),
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

    // Calibracao do rumo: SEMPRE na mesma antena (a primeira de ~gps_nrs). Se
    // misturasse as duas, o passo lateral entre elas (1.44 m no Ype) entraria
    // como zigue-zague na direcao de percurso e sujaria a medida.
    if (!gps_nrs_.empty() && nr == gps_nrs_.front())
        update_heading_offset_estimate(msg->header.stamp, it->theta, it->x, it->y);

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

// ─── Fusão / publicação ───────────────────────────────────────────────────

// Heading atual do veiculo pra rotacionar o braço de alavanca: preferencia
// pro HDT (heading verdadeiro, nao tem drift), senao cai pro theta cru mais
// recente entre as antenas ativas.
bool GpsLocalizationNode::current_heading(const ros::Time &now, double &theta_out) const
{
    bool found = false;

    if (heading_from_hdt_ && (now - last_heading_stamp_).toSec() < heading_max_age_)
    {
        theta_out = last_heading_;
        found = true;
    }
    else
    {
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
    }

    if (!found)
        return false;

    // Offset de montagem das antenas. Aplicado nas DUAS fontes (HDT e theta cru):
    // as duas descrevem a mesma linha de base fisica, entao carregam o mesmo erro.
    theta_out += heading_offset_;
    while (theta_out >   M_PI) theta_out -= 2.0 * M_PI;
    while (theta_out <= -M_PI) theta_out += 2.0 * M_PI;
    return true;
}

// Mede (rumo reportado - direcao de percurso) por media circular ponderada pela
// distancia do trecho. Nao altera nada na saida -- so' loga o numero que voce
// deve pôr em ~heading_offset_deg (com o sinal ja' trocado, veja o log).
//
// Ponderar por distancia importa: com linha de base curta o ruido de posicao do
// GPS vira erro angular grande, entao trecho longo tem que dominar.
void GpsLocalizationNode::update_heading_offset_estimate(const ros::Time &stamp,
                                                         double raw_theta,
                                                         double x, double y)
{
    if (!estimate_heading_offset_)
        return;

    if (!hoff_have_prev_)
    {
        hoff_have_prev_  = true;
        hoff_prev_x_     = x;
        hoff_prev_y_     = y;
        hoff_prev_theta_ = raw_theta;
        hoff_prev_stamp_ = stamp;
        return;
    }

    const double dx = x - hoff_prev_x_;
    const double dy = y - hoff_prev_y_;
    const double d  = std::hypot(dx, dy);

    if (d < hoff_min_baseline_)
        return;   // ainda nao andou o bastante; acumula mais

    // So' vale em trecho reto: se o rumo mudou muito entre as duas pontas, a
    // corda nao representa a direcao instantanea e a amostra sujaria a media.
    double dtheta = raw_theta - hoff_prev_theta_;
    while (dtheta >   M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta <= -M_PI) dtheta += 2.0 * M_PI;

    if (std::fabs(dtheta) < 10.0 * M_PI / 180.0)
    {
        const double cog = std::atan2(dy, dx);
        double diff = 0.5 * (raw_theta + hoff_prev_theta_) - cog;
        while (diff >   M_PI) diff -= 2.0 * M_PI;
        while (diff <= -M_PI) diff += 2.0 * M_PI;

        const double w = d;
        hoff_sum_sin_ += w * std::sin(diff);
        hoff_sum_cos_ += w * std::cos(diff);
        hoff_wsum_    += w;
        ++hoff_samples_;

        if ((hoff_samples_ % 10) == 0)
        {
            const double est = std::atan2(hoff_sum_sin_, hoff_sum_cos_);
            const double R   = std::hypot(hoff_sum_sin_, hoff_sum_cos_) / hoff_wsum_;
            const double sigma_deg = (R > 0.0 && R < 1.0)
                                   ? std::sqrt(-2.0 * std::log(R)) * 180.0 / M_PI
                                   : 0.0;
            ROS_INFO("\033[1;33m[calib rumo] %zu trechos: rumo reportado esta' %.2f deg "
                     "adiante da direcao de percurso (sigma=%.2f deg). "
                     "Para corrigir use ~heading_offset_deg:=%.2f\033[0m",
                     hoff_samples_, est * 180.0 / M_PI, sigma_deg,
                     -est * 180.0 / M_PI);
        }
    }

    hoff_prev_x_     = x;
    hoff_prev_y_     = y;
    hoff_prev_theta_ = raw_theta;
    hoff_prev_stamp_ = stamp;
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

    // Pose global (UTM/CARMEN) do /car agora -- publicada DIRETO, sem ancorar
    // em nada. FIX: a ancoragem em cima da odometria local do LIO-SAM foi
    // removida daqui -- ela dependia de ~lio_local_odom_topic já estar
    // correta, o que só é verdade em mapeamento (odometria sendo construída
    // do zero); em localizationMode a odometria local começa errada (semente
    // do yaml) até o robô encaixar no mapa, então a âncora saía errada bem
    // na hora que mais precisava estar certa. Quem converte pro frame do
    // mapa agora é o SC-LIO-SAM (mapOptmization.cpp: gpsHandler + arquivo
    // gps_utm_anchor.txt salvo/lido junto com o mapa) -- este nó só funde
    // antena(s) + heading + compensação de latência e publica cru.
    tf2::Transform T_utm_car(q_heading, tf2::Vector3(fx, fy, fz));

    nav_msgs::Odometry out;
    out.header.stamp    = trigger_stamp;
    out.header.frame_id = gps_output_frame_id_;   // "utm" -- cru, nao ancorado
    out.child_frame_id  = car_frame_id_;          // "base_link"

    const tf2::Vector3 &pos = T_utm_car.getOrigin();
    out.pose.pose.position.x = pos.x();
    out.pose.pose.position.y = pos.y();
    out.pose.pose.position.z = pos.z();
    out.pose.pose.orientation = tf2::toMsg(T_utm_car.getRotation());

    for (auto &cov : out.pose.covariance) cov = 0.0;
    out.pose.covariance[0]  = var_x;
    out.pose.covariance[7]  = var_y;
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