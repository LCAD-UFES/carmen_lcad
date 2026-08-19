/*
 * fused_odometry_from_slam_node.cpp  (ROS1 / Noetic)
 *
 * CODIGO DE TESTE -- injeta a localizacao do SC-LIO-SAM no CARMEN em tempo real.
 *
 * Como funciona (nada disso exige mudanca no CARMEN):
 *
 *   SC-LIO-SAM  --/lio_sam/mapping/odometry-->  ESTE NO
 *                                                 |
 *                                    carmen_fused_odometry_message (IPC)
 *                                                 |
 *                              localize_ackerman -mapping_mode on
 *                                                 |
 *                                    carmen_localize_ackerman_globalpos
 *                                                 |
 *                              mapper, navigator, mss, viewer_3D, ...
 *
 * Em -mapping_mode on o localize_ackerman NAO roda o filtro de particulas: o
 * handler do velodyne (localize_ackerman_main.cpp:3665) chama
 * publish_globalpos_on_mapping_mode() e da' return. Essa funcao monta o
 * globalpos INTEIRAMENTE a partir da fused_odometry -- pose, velocity e phi --,
 * ainda interpolando a pose ate' o instante do scan com
 * carmen_ackerman_interpolated_robot_position_at_time(). Ou seja: quem publica
 * fused_odometry controla o globalpos.
 *
 * PRE-REQUISITOS (verificados em 03/08/2026):
 *   1. O modulo `fused_odometry` do CARMEN NAO pode estar rodando -- dois
 *      publicadores da mesma mensagem.
 *   2. |x| >= 1.000.000. O localize se recusa a publicar globalpos abaixo disso
 *      (localize_ackerman_main.cpp:3426), sem log nenhum. O x do CARMEN e' o
 *      northing da UTM, que no hemisferio sul fica na casa dos 7,7 milhoes.
 *      Use o mesmo posesOrigin* do poses_opt.dat.
 *   3. localize_ackerman com -mapping_mode on.
 *
 * LIMITACOES CONHECIDAS deste caminho:
 *   - o filtro de particulas nao roda: status/particulas/metricas do localize
 *     deixam de sair;
 *   - publish_globalpos_on_mapping_mode() nao chama update_semi_trailers_thetas(),
 *     entao trailer_theta[] do globalpos nao e' recalculado. Irrelevante na IARA
 *     (sem reboque), buraco real em caminhao com semi-reboque.
 *
 * CONTRATO DO SEMI-REBOQUE (mao unica): o angulo de articulacao pertence ao
 * localize_ackerman. O lado ROS so' CONSOME -- pointcloud_node.cpp assina o
 * globalpos e converte theta -> beta pro filtro de colisao e pros marcadores do
 * RViz. Nenhum no' daqui escreve num_trailers/trailer_theta em mensagem IPC:
 * como o SLAM nao observa a articulacao, qualquer valor que sairia daqui seria
 * constante em relacao ao cavalo e entraria no barramento como se fosse medida.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>

#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>

#include "carmen_ipc_bridge/carmen_time.hpp"

// ─── carmen_localize_ackerman_initialize_gaussian_command_host() ─────────────
//
// O localize_ackerman_interface do fork de origem tem uma variante "_host" do
// initialize_gaussian_command que deixa o CHAMADOR escolher o campo `host` da
// mensagem. E' disso que este no' depende: ele mesmo assina
// carmen_localize_ackerman_initialize_message (o clique de "posicionar robo" do
// navigator_gui/viewer_3D) e precisa distinguir o clique de verdade do ECO do
// seu proprio snap -- o filtro e' justamente pelo host (ver
// onCarmenInitializeMessage()).
//
// O CARMEN so' tem carmen_localize_ackerman_initialize_gaussian_command(), que
// carimba carmen_get_host() e portanto nao da' como filtrar o eco. Em vez de
// mexer no libcarmen (a interface e' compartilhada com todo o resto), a
// variante fica local aqui: e' a MESMA mensagem IPC, mesmos NAME/FMT, so' com o
// host vindo por parametro.
static void
carmen_localize_ackerman_initialize_gaussian_command_host(carmen_point_t mean, carmen_point_t std,
                                                          double *trailer_theta, int num_trailers,
                                                          char *host)
{
    static carmen_localize_ackerman_initialize_message init;
    static int first = 1;
    IPC_RETURN_TYPE err;

    if (first)
    {
        err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, IPC_VARIABLE_LENGTH,
                            CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
        carmen_test_ipc_exit(err, "Could not define message", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
        first = 0;
    }

    init.timestamp = carmen_get_time();
    init.host = host;
    init.distribution = CARMEN_INITIALIZE_GAUSSIAN;
    init.num_modes = 1;
    init.mean = &mean;
    init.std = &std;
    init.num_trailers = num_trailers;
    for (int i = 0; i < init.num_trailers; i++)
        init.trailer_theta[i] = trailer_theta[i];

    err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, &init);
    carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
}

// Forward decl: o handler IPC do clique "posicionar robo" (navigator_gui/
// viewer_3D) e' um free function (CARMEN_EXTERN_FUNCTION/carmen_handler_t nao
// aceita ponteiro-a-membro), mas precisa ser referenciavel de dentro do
// construtor da classe -- ver carmenInitializeMessageHandler() e
// FusedOdometryFromSlam::onCarmenInitializeMessage() logo apos a classe.
class FusedOdometryFromSlam;
static FusedOdometryFromSlam *g_fused_odom_node = nullptr;
static void carmenInitializeMessageHandler(carmen_localize_ackerman_initialize_message *msg);

class FusedOdometryFromSlam
{
public:
    FusedOdometryFromSlam(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh)
    {
        pnh_.param<std::string>("ipc_host", ipc_host_, "localhost");
        pnh_.param<std::string>("base_frame_id",  base_frame_id_,  "base_link");
        pnh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "velodyne");
        // PADRAO = /odometry/imu (saida do TransformFusion, imuPreintegration.cpp:299),
        // NAO /lio_sam/mapping/odometry.
        //
        // O /lio_sam/mapping/odometry sai so' quando o mapOptimization TERMINA um
        // scan (~6 Hz aqui, e trava em transiente tipo buraco/lombada). Como o
        // publish_globalpos_on_mapping_mode() faz
        //     dt = t_velodyne - t_fused
        //     pose = interpolated_robot_position_at_time(pose, dt, v, phi, L)
        // uma fused_odometry atrasada em 200 ms vira 2 m de extrapolacao a 10 m/s.
        // E' o "carro na frente da nuvem" -- o dt e' uma leitura direta do atraso.
        //
        // Ja' o /odometry/imu e' lidarOdomAffine * incremento_ate_agora: sai na taxa
        // do IMU/Ackermann, com latencia ~zero, e e' EXATAMENTE a pose que gera a TF
        // odom->base_link que aparece no RViz. Com ela dt ~ 0 e nao ha' extrapolacao.
        //
        // Mesma convencao de frame nos dois (pose do LIDAR), entao a correcao
        // T_base_lidar continua valendo igual.
        pnh_.param<std::string>("slam_odom_topic", slam_odom_topic_, "/odometry/imu");
        pnh_.param<double>("max_publish_rate", max_publish_rate_, 50.0);
        pnh_.param<std::string>("ackermann_topic", ackermann_topic_, "/ackermann/odom_raw");
        pnh_.param<std::string>("world_origin_file", world_origin_file_, "");
        pnh_.param<double>("origin_x",   origin_x_,   0.0);
        pnh_.param<double>("origin_y",   origin_y_,   0.0);
        pnh_.param<double>("origin_z",   origin_z_,   0.0);
        pnh_.param<double>("origin_yaw", origin_yaw_, 0.0);
        pnh_.param<double>("ackermann_max_dt", ackermann_max_dt_, 0.2);
        // Topico do "snap" de orientacao dos semi-reboques (ver onInitialPoseTrigger()).
        // Padrao = /initialpose, o MESMO topico que a ferramenta "2D Pose Estimate"
        // do RViz ja publica -- reusa esse gatilho em vez de reinventar um botao.
        pnh_.param<std::string>("initialpose_topic", initialpose_topic_, "/initialpose");

        // O arquivo (gravado pelo mapOptmization) vence os parametros, pra a
        // origem ser a MESMA que o poses_opt.dat usou.
        if (!world_origin_file_.empty())
            loadWorldOrigin();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, origin_yaw_);
        T_world_map_ = tf2::Transform(q, tf2::Vector3(origin_x_, origin_y_, origin_z_));

        ROS_INFO("fused_odometry_from_slam: origem do mundo x=%.3f y=%.3f z=%.3f yaw=%.6f",
                 origin_x_, origin_y_, origin_z_, origin_yaw_);

        if (std::fabs(origin_x_) < 1000000.0)
            ROS_WARN("\033[1;31mORIGEM x=%.3f tem |x| < 1.000.000 -- o localize_ackerman "
                     "VAI DESCARTAR o globalpos em silencio (main.cpp:3426). "
                     "Ajuste a origem antes de rodar.\033[0m", origin_x_);

        // carmen_ipc_initialize() faz carmen_ipc_connect(argv[0]) na PRIMEIRA linha
        // (global/ipc_wrapper.c:512) -- passar argv nulo e' segfault imediato,
        // antes de qualquer log sair. Mesmo padrao dos outros nos do bridge:
        // monta um argv minimo com o nome do modulo e o central host.
        std::string prog = "fused_odometry_from_slam";
        std::string flag = "-central_host";
        std::vector<char *> argv_vec = {
            const_cast<char *>(prog.c_str()),
            const_cast<char *>(flag.c_str()),
            const_cast<char *>(ipc_host_.c_str())
        };
        carmen_ipc_initialize(static_cast<int>(argv_vec.size()), argv_vec.data());

        IPC_RETURN_TYPE err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_NAME,
                                            IPC_VARIABLE_LENGTH,
                                            CARMEN_FUSED_ODOMETRY_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_NAME);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        sub_ack_  = nh_.subscribe(ackermann_topic_, 200, &FusedOdometryFromSlam::onAckermann, this);
        sub_slam_ = nh_.subscribe(slam_odom_topic_, 20,  &FusedOdometryFromSlam::onSlamOdom, this);
        sub_initialpose_ = nh_.subscribe(initialpose_topic_, 1,
                &FusedOdometryFromSlam::onInitialPoseTrigger, this);
        pub_initialpose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic_, 1);

        // Repassa o clique "posicionar robo" do navigator_gui/viewer_3D
        // (carmen_localize_ackerman_initialize_message, IPC) pro SC-LIO-SAM via
        // /initialpose -- ver onCarmenInitializeMessage(). O SC-LIO-SAM ja' sabe
        // re-localizar por ICP a partir desse topico (mapOptmization.cpp:861
        // initialPoseHandler() / :921 requestLocSnapFromBaseLinkPose()); so'
        // faltava algo do lado CARMEN publicando nele quando o clique vem do
        // navigator_gui/viewer_3D em vez do "2D Pose Estimate" do RViz.
        g_fused_odom_node = this;
        carmen_localize_ackerman_subscribe_initialize_message(
            nullptr, (carmen_handler_t) carmenInitializeMessageHandler, CARMEN_SUBSCRIBE_LATEST);

        // carmen_ipc_initialize() so' conecta -- precisa bombear o loop de
        // recebimento pra' a subscribe acima disparar o handler. Sem thread
        // dedicada: todo I/O do IPC (publishes inclusive) fica no MESMO thread
        // do ROS, pra' nao introduzir concorrencia na conexao IPC (compare com
        // pointcloud_node.cpp/ipc_bridge_node.cpp, que usam thread dedicada mas
        // tambem fazem TODO o IPC deles -- inclusive publish -- só' nela). Aqui
        // um timer do ROS chama IPC_listen(0) (nao bloqueante) periodicamente.
        ipc_poll_timer_ = nh_.createTimer(ros::Duration(0.02),
                [](const ros::TimerEvent &) { IPC_listen(0); });

        ROS_INFO("fused_odometry_from_slam pronto: %s + %s -> %s (IPC)",
                 slam_odom_topic_.c_str(), ackermann_topic_.c_str(), CARMEN_FUSED_ODOMETRY_NAME);
        ROS_INFO("Lembre: localize_ackerman -mapping_mode on, e o modulo fused_odometry DESLIGADO.");
        ROS_INFO("Snap do(s) trailer_theta[]: publique em '%s' (ex.: '2D Pose Estimate' do "
                 "RViz) para realinhar o(s) reboque(s) com o yaw ATUAL do cavalo -- um snap "
                 "por clique, nao repete sozinho.", initialpose_topic_.c_str());
    }

private:
    void loadWorldOrigin()
    {
        std::ifstream f(world_origin_file_);
        if (!f.is_open())
        {
            ROS_WARN("world_origin_file '%s' nao abriu -- usando os parametros origin_*.",
                     world_origin_file_.c_str());
            return;
        }
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#') continue;
            if (line.compare(0, 6, "pesos ") == 0) continue;
            std::istringstream ss(line);
            double x, y, z, roll, pitch, yaw;
            if (!(ss >> x >> y >> z >> roll >> pitch >> yaw)) continue;
            origin_x_ = x; origin_y_ = y; origin_z_ = z; origin_yaw_ = yaw;
            ROS_INFO("Origem do mundo lida de '%s'.", world_origin_file_.c_str());
            return;
        }
        ROS_WARN("world_origin_file '%s' sem linha valida -- usando os parametros origin_*.",
                 world_origin_file_.c_str());
    }

    // Extrinseca base_link -> lidar, com lookup preguicoso (a TF estatica so'
    // aparece depois que o pointcloud_node publica o primeiro scan).
    bool ensureBaseLidar()
    {
        if (have_base_lidar_) return true;

        if (base_frame_id_ == lidar_frame_id_)
        {
            T_base_lidar_.setIdentity();
            have_base_lidar_ = true;
            return true;
        }

        try {
            geometry_msgs::TransformStamped t =
                tf_buffer_.lookupTransform(base_frame_id_, lidar_frame_id_, ros::Time(0));
            tf2::fromMsg(t.transform, T_base_lidar_);
            have_base_lidar_ = true;

            double r, p, y;
            tf2::Matrix3x3(T_base_lidar_.getRotation()).getRPY(r, p, y);
            ROS_INFO("Extrinseca %s -> %s: t=(%.3f, %.3f, %.3f) rpy=(%.4f, %.4f, %.4f) "
                     "[yaw %.1f deg]",
                     base_frame_id_.c_str(), lidar_frame_id_.c_str(),
                     T_base_lidar_.getOrigin().x(), T_base_lidar_.getOrigin().y(),
                     T_base_lidar_.getOrigin().z(), r, p, y, y * 180.0 / M_PI);
            return true;
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0,
                "TF %s -> %s indisponivel (%s) -- sem ela a pose sairia no frame do "
                "LIDAR, com o yaw errado. Nao publicando.",
                base_frame_id_.c_str(), lidar_frame_id_.c_str(), ex.what());
            return false;
        }
    }

    void onAckermann(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        // Convencao do CARMEN: linear.x = v [m/s], angular.z = phi [rad]
        // (angulo de estercamento, NAO velocidade angular -- ver o modelo
        // bicicleta do AckermannPreintegration).
        last_v_        = msg->twist.linear.x;
        last_phi_      = msg->twist.angular.z;
        last_ack_time_ = msg->header.stamp.toSec();
        have_ack_      = true;
    }

    // Publica carmen_localize_ackerman_initialize_message (mesmo IPC que o
    // navigator_gui publica ao clicar "inicializar pose" --
    // carmen_localize_ackerman_main.cpp:4283 trata sem checar mapping_mode).
    // publish_first_globalpos(), chamado de dentro desse handler, publica um
    // carmen_localize_ackerman_globalpos JA' NESSA CHAMADA -- diferente de
    // publish_globalpos_on_mapping_mode(), que so' publica quando um scan do
    // velodyne chega. E' por isso que usamos isso tanto pro snap manual quanto
    // pro primeiro globalpos do startup (ver onSlamOdom()).
    void snapGlobalpos(double x, double y, double yaw, const char *host)
    {
        carmen_point_t mean = {x, y, yaw};
        carmen_point_t std  = {0.05, 0.05, 0.05};
        double trailer_theta[MAX_NUM_TRAILERS];
        for (int i = 0; i < num_trailers_; ++i)
            trailer_theta[i] = yaw;

        carmen_localize_ackerman_initialize_gaussian_command_host(
            mean, std, trailer_theta, num_trailers_, (char *) host);
    }

public:
    // Chamado por carmenInitializeMessageHandler() (livre, fora da classe --
    // ver o topo do arquivo) quando chega um carmen_localize_ackerman_initialize_message
    // pelo IPC: o clique de "posicionar robo" do navigator_gui/viewer_3D, OU um
    // dos nossos proprios snapGlobalpos() ecoando de volta (filtrado abaixo).
    //
    // mean->x/y/theta chegam no frame MUNDO/UTM do CARMEN (mesma convencao de
    // origin_x_/origin_y_/origin_yaw_/T_world_map_). Aplicamos o INVERSO da
    // MESMA transformacao que onSlamOdom() usa no sentido local->mundo, e
    // publicamos em /initialpose no frame local do SC-LIO-SAM -- que ja' sabe
    // rodar um ICP de re-localizacao a partir dai (mapOptmization.cpp:861
    // initialPoseHandler(), :921 requestLocSnapFromBaseLinkPose()). E' isso que
    // fecha o loop CARMEN -> SC-LIO-SAM: clicar no mapa do navigator_gui/viewer_3D
    // agora re-posiciona o robo de verdade (nao so' o trailer_theta, que e' o
    // que o snap manual via /initialpose, abaixo, ja' fazia).
    void onCarmenInitializeMessage(carmen_localize_ackerman_initialize_message *msg)
    {
        // Ignora o eco dos nossos proprios snaps -- snapGlobalpos() publica
        // esse MESMO tipo de mensagem (com host "fused_odometry_from_slam" ou
        // "fused_odometry_from_slam_startup"). Sem isso, cada snap automatico
        // de trailer dispararia uma re-localizacao desnecessaria no SC-LIO-SAM.
        const std::string host = msg->host ? msg->host : "";
        if (host.rfind("fused_odometry_from_slam", 0) == 0)
            return;

        if (!msg->mean)
        {
            ROS_WARN("Clique de posicionamento (host=%s) sem 'mean' -- ignorando.",
                     msg->host ? msg->host : "?");
            return;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, msg->mean->theta);
        tf2::Transform T_world_base(q, tf2::Vector3(msg->mean->x, msg->mean->y, 0.0));
        tf2::Transform T_map_base = T_world_map_.inverse() * T_world_base;

        geometry_msgs::PoseWithCovarianceStamped out;
        out.header.stamp    = ros::Time::now();
        out.header.frame_id = last_slam_frame_id_.empty() ? "map" : last_slam_frame_id_;
        out.pose.pose.position.x    = T_map_base.getOrigin().x();
        out.pose.pose.position.y    = T_map_base.getOrigin().y();
        out.pose.pose.position.z    = T_map_base.getOrigin().z();
        out.pose.pose.orientation.x = T_map_base.getRotation().x();
        out.pose.pose.orientation.y = T_map_base.getRotation().y();
        out.pose.pose.orientation.z = T_map_base.getRotation().z();
        out.pose.pose.orientation.w = T_map_base.getRotation().w();

        pub_initialpose_.publish(out);

        ROS_INFO("Clique de posicionamento (host=%s) repassado pro SC-LIO-SAM em '%s' "
                 "(frame '%s'): mundo x=%.3f y=%.3f yaw=%.4f.",
                 msg->host ? msg->host : "?", initialpose_topic_.c_str(),
                 out.header.frame_id.c_str(), msg->mean->x, msg->mean->y, msg->mean->theta);
    }

private:
    // Gatilho de snap: dispara UMA VEZ por mensagem recebida em '/initialpose'
    // (o mesmo topico do "2D Pose Estimate" do RViz) -- nao repete sozinho depois.
    // Ignoramos o conteudo da mensagem (a posicao clicada no RViz esta no frame
    // local do SC-LIO-SAM, nao no frame mundo/UTM do CARMEN) e usamos a ULTIMA
    // pose ja convertida por onSlamOdom() -- ela e' a estimativa atual do SLAM,
    // mais precisa que qualquer coisa que se consiga clicar a mao. Depois desse
    // unico publish o localize_ackerman volta a andar sozinho --
    // publish_globalpos_on_mapping_mode() continua atualizando x/y/yaw a cada
    // scan; so' trailer_theta[] fica parado ate' o proximo clique.
    void onInitialPoseTrigger(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & /*msg*/)
    {
        if (!have_world_pose_)
        {
            ROS_WARN("snap de trailer pedido, mas ainda nao recebi nenhuma pose do SLAM -- ignorando.");
            return;
        }
        if (num_trailers_ <= 0)
        {
            ROS_WARN("snap de trailer pedido, mas num_trailers=0 (sem reboque configurado) -- ignorando.");
            return;
        }

        snapGlobalpos(last_world_x_, last_world_y_, last_world_yaw_, "fused_odometry_from_slam");

        ROS_INFO("snap: trailer_theta alinhado ao yaw do cavalo (%.4f rad) em x=%.3f y=%.3f",
                 last_world_yaw_, last_world_x_, last_world_y_);
    }

    void onSlamOdom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        const double stamp = msg->header.stamp.toSec();

        // Limite de taxa: /odometry/imu pode vir a 100+ Hz e o localize so' precisa
        // de uma amostra por scan. 0 desliga o limite.
        if (max_publish_rate_ > 0.0)
        {
            const double min_dt = 1.0 / max_publish_rate_;
            if (last_pub_stamp_ > 0.0 && (stamp - last_pub_stamp_) < min_dt)
                return;
            last_pub_stamp_ = stamp;
        }

        // ATENCAO: /lio_sam/mapping/odometry publica o transformTobeMapped CRU
        // (mapOptmization.cpp:4396), que e' a pose do LIDAR no frame do mapa --
        // NAO a do base_link. Na IARA o base_link->velodyne tem yaw = 180 graus
        // (LiDAR montado girado), entao sem converter a pose sai 180 graus errada.
        // O writeLocalizationPose() do mapOptmization ja' faz essa conversao; aqui
        // faltava.
        //     T_map_base = T_map_lidar * T_lidar_base,  T_lidar_base = T_base_lidar^-1
        if (!ensureBaseLidar())
            return;

        tf2::Transform T_map_lidar;
        tf2::fromMsg(msg->pose.pose, T_map_lidar);
        const tf2::Transform T_map_base   = T_map_lidar * T_base_lidar_.inverse();
        const tf2::Transform T_world_base = T_world_map_ * T_map_base;

        double roll, pitch, yaw;
        tf2::Matrix3x3(T_world_base.getRotation()).getRPY(roll, pitch, yaw);

        // v e phi vem da odometria Ackermann real, nao do SLAM: o
        // publish_globalpos_on_mapping_mode() usa os dois pra extrapolar a pose
        // ate' o instante do scan. Sem eles a extrapolacao vira identidade.
        double v = 0.0, phi = 0.0;
        if (have_ack_ && std::fabs(stamp - last_ack_time_) <= ackermann_max_dt_)
        {
            v   = last_v_;
            phi = last_phi_;
        }
        else
        {
            ROS_WARN_THROTTLE(5.0,
                "sem Ackermann proximo (dt=%.3f > %.3f) -- publicando v=0, phi=0. "
                "A extrapolacao ate' o scan fica sem efeito.",
                have_ack_ ? std::fabs(stamp - last_ack_time_) : -1.0, ackermann_max_dt_);
        }

        carmen_fused_odometry_message m;
        memset(&m, 0, sizeof(m));

        m.pose.position.x = T_world_base.getOrigin().x();
        m.pose.position.y = T_world_base.getOrigin().y();
        m.pose.position.z = T_world_base.getOrigin().z();
        m.pose.orientation.roll  = roll;
        m.pose.orientation.pitch = pitch;
        m.pose.orientation.yaw   = yaw;

        m.xsens_yaw_bias = 0.0;

        m.velocity.x = v;      // publish_globalpos_on_mapping_mode le' velocity.x como v
        m.velocity.y = 0.0;
        m.velocity.z = 0.0;

        m.angular_velocity.roll  = msg->twist.twist.angular.x;
        m.angular_velocity.pitch = msg->twist.twist.angular.y;
        m.angular_velocity.yaw   = msg->twist.twist.angular.z;

        m.phi = phi;

        // ─── Alinhamento dos semi-reboques ────────────────────────────────
        //
        // publish_globalpos_on_mapping_mode() (localize_ackerman_main.cpp:3391)
        // monta o globalpos inteiro a partir desta mensagem SEM rodar o estimador
        // de articulacao (update_semi_trailers_thetas()) e SEM ler os campos
        // trailer_theta[]/num_trailers daqui -- preenche-los na carmen_fused_odometry
        // nao tem efeito nenhum no globalpos publicado (deixamos abaixo so' pra
        // mensagem ficar coerente, caso outro consumidor da fused_odometry exista).
        //
        // Quem efetivamente atualiza o trailer_theta[] do globalpos, mesmo em
        // mapping_mode, e' carmen_localize_ackerman_initialize_message_handler
        // (localize_ackerman_main.cpp:4283) -- o MESMO handler acionado quando um
        // operador clica "inicializar pose" no navigator_gui/viewer_3D, e ele nao
        // checa mapping_mode. NAO chamamos ele aqui a cada mensagem -- isso reinicia
        // o filtro de particulas do localize_ackerman toda hora e brigaria com o
        // proprio fluxo continuo (publish_globalpos_on_mapping_mode(), acima) que ja
        // move x/y/yaw sozinho a cada scan. So' guardamos a pose pra usar quando o
        // operador pedir um snap explicito -- ver onInitialPoseTrigger().
        m.num_trailers = num_trailers_;
        for (int i = 0; i < MAX_NUM_TRAILERS; ++i)
            m.trailer_theta[i] = (i < num_trailers_) ? yaw : 0.0;

        last_world_x_ = m.pose.position.x;
        last_world_y_ = m.pose.position.y;
        last_world_yaw_ = yaw;
        last_slam_frame_id_ = msg->header.frame_id;   // usado por onCarmenInitializeMessage()
        have_world_pose_ = true;

        // Publica o globalpos inicial JA' NESTA PRIMEIRA MENSAGEM, sem esperar
        // por um scan do velodyne (publish_globalpos_on_mapping_mode() so' roda
        // quando um scan chega -- ver carmen_fused_odometry_publish_message() logo
        // abaixo). Mitiga (nao resolve) um bug conhecido do navigator_gui2: ate' a
        // PRIMEIRA carmen_localize_ackerman_globalpos chegar, GtkGui::globalpos
        // aponta pra memoria de pilha invalida (navigator_gui2_main.cpp:2285), e
        // qualquer carmen_navigator_ackerman_status_message que chegue nesse
        // intervalo pode crashar o navigator_gui. Isso so' encurta a janela --
        // nao elimina a causa raiz, que continua no navigator_gui2.
        if (!have_published_initial_globalpos_)
        {
            have_published_initial_globalpos_ = true;
            snapGlobalpos(last_world_x_, last_world_y_, last_world_yaw_, "fused_odometry_from_slam_startup");
            ROS_INFO("globalpos inicial publicado (snap) em x=%.3f y=%.3f yaw=%.4f -- "
                     "antes do 1o scan do velodyne, pra' reduzir a janela do bug de "
                     "startup do navigator_gui2.", last_world_x_, last_world_y_, last_world_yaw_);
        }

        m.gps_position_at_turn_on.x = 0.0;
        m.gps_position_at_turn_on.y = 0.0;
        m.gps_position_at_turn_on.z = 0.0;

        // O stamp do /lio_sam/mapping/odometry e' o INICIO da varredura, na mesma
        // base de tempo do CARMEN (o bridge converte sem deslocamento). A mensagem
        // do velodyne no CARMEN carimba o FIM, entao o dt que o
        // publish_globalpos_on_mapping_mode() calcula (~50 ms) e' exatamente a
        // duracao da varredura -- e a extrapolacao dele acerta o alvo.
        m.timestamp = stamp;
        m.host      = carmen_get_host();

        carmen_fused_odometry_publish_message(&m);

        if ((++published_ % 200) == 0)
            ROS_INFO("fused_odometry: %zu publicadas | x=%.3f y=%.3f yaw=%.4f v=%.2f phi=%.4f t=%.6f",
                     published_, m.pose.position.x, m.pose.position.y, yaw, v, phi, m.timestamp);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_slam_, sub_ack_, sub_initialpose_;
    ros::Publisher  pub_initialpose_;
    ros::Timer      ipc_poll_timer_;
    std::string     last_slam_frame_id_;

    std::string slam_odom_topic_, ackermann_topic_, world_origin_file_;
    double origin_x_{0.0}, origin_y_{0.0}, origin_z_{0.0}, origin_yaw_{0.0};
    double ackermann_max_dt_{0.2};
    double max_publish_rate_{50.0};
    double last_pub_stamp_{0.0};
    std::string ipc_host_, base_frame_id_, lidar_frame_id_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform T_base_lidar_;
    bool have_base_lidar_{false};
    tf2::Transform T_world_map_;

    bool   have_ack_{false};
    double last_v_{0.0}, last_phi_{0.0}, last_ack_time_{0.0};
    size_t published_{0};
    int    num_trailers_{0};
    std::string initialpose_topic_;
    double last_world_x_{0.0}, last_world_y_{0.0}, last_world_yaw_{0.0};
    bool   have_world_pose_{false};
    bool   have_published_initial_globalpos_{false};
};

// Trampolim do handler IPC (carmen_handler_t e' ponteiro-a-funcao livre, nao
// aceita ponteiro-a-membro) pro metodo real -- ver declaracao no topo do
// arquivo e FusedOdometryFromSlam::onCarmenInitializeMessage().
static void carmenInitializeMessageHandler(carmen_localize_ackerman_initialize_message *msg)
{
    if (g_fused_odom_node)
        g_fused_odom_node->onCarmenInitializeMessage(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fused_odometry_from_slam");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    FusedOdometryFromSlam node(nh, pnh);
    ros::spin();
    return 0;
}
