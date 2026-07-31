#include "utility.h"

#include <algorithm>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TwistStamped.h>
#include <memory>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

// ─── Helpers do modelo de bicicleta (compartilhados por Ackermann e Fusion) ──
//
// FIX: nada limitava phi (angulo de esterco) antes de entrar no modelo. Perto
// de 90 graus, tan(phi) e principalmente 1/cos(phi)^2 (usado no Jacobiano Fu
// abaixo) explodem -- e nenhum veiculo real esterca perto disso, entao uma
// leitura nessa faixa e' quase certeza de glitch de sensor (CAN sob vibracao,
// saturacao do encoder de esterco), tipicamente em curva fechada. Sem clamp,
// isso estoura o Sigma/cov6 daquele passo, o ISAM2 lanca
// IndeterminantLinearSystemException, o grafo e' resetado (resetParams() +
// resetOptimization()) e a publicacao de odometria fica parada ate a proxima
// correcao reconstruir o grafo -- e' essa janela de varios segundos sem odom
// real (odomReal=0) que aparecia so na hora da curva.
// Ajustar kMaxSteerAngle pro limite fisico real do veiculo (com folga) se
// souber o esterco maximo de verdade; 60 graus ja e' bem acima do que a
// maioria dos caminhoes/veiculos Ackermann atinge.
static constexpr double kMaxSteerAngle = 60.0 * M_PI / 180.0;

static inline double clampSteerAngle(double phi)
{
    if (phi >  kMaxSteerAngle) return  kMaxSteerAngle;
    if (phi < -kMaxSteerAngle) return -kMaxSteerAngle;
    return phi;
}

// Propaga apenas a media (x, y, yaw) de um passo do modelo de bicicleta
// (ponto medio no yaw). L = distancia entre eixos.
static void bicyclePropagateMean(double v, double phi, double dt, double L,
                                 double& x, double& y, double& yaw)
{
    phi = clampSteerAngle(phi);
    const double omega  = (L > 1e-6) ? (v * std::tan(phi) / L) : 0.0;
    const double dyaw   = omega * dt;
    const double yawMid = yaw + 0.5 * dyaw;
    x   += v * std::cos(yawMid) * dt;
    y   += v * std::sin(yawMid) * dt;
    yaw += dyaw;
}

// Integra um passo atualizando a media (x, y, yaw) e a covariancia 3x3
// (ordem: x, y, theta) via propagacao EKF do modelo de bicicleta.
// O escorregamento (slip) e random walk no tempo: variancia += (sigma^2)*dt.
// O slip x,y esta no frame do corpo e e rotacionado para o frame acumulado.
static void bicycleIntegrateStep(double v, double phi, double dt, double L,
                                 double velN, double steerN,
                                 double slipX, double slipY, double slipTheta,
                                 double& x, double& y, double& yaw,
                                 Eigen::Matrix3d& Sigma)
{
    phi = clampSteerAngle(phi);
    const double omega  = (L > 1e-6) ? (v * std::tan(phi) / L) : 0.0;
    const double dyaw   = omega * dt;
    const double yawMid = yaw + 0.5 * dyaw;
    const double c = std::cos(yawMid);
    const double s = std::sin(yawMid);

    // Jacobiano do estado Fx (d[x',y',th']/d[x,y,th])
    Eigen::Matrix3d Fx = Eigen::Matrix3d::Identity();
    Fx(0,2) = -v * s * dt;
    Fx(1,2) =  v * c * dt;

    // Jacobiano do controle Fu (d[x',y',th']/d[v,phi])
    Eigen::Matrix<double,3,2> Fu = Eigen::Matrix<double,3,2>::Zero();
    Fu(0,0) = c * dt;
    Fu(1,0) = s * dt;
    Fu(2,0) = (L > 1e-6) ? (std::tan(phi) / L) * dt : 0.0;
    const double cphi = std::cos(phi);
    Fu(2,1) = (L > 1e-6 && std::abs(cphi) > 1e-6) ? v * dt / (L * cphi * cphi) : 0.0;

    Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
    Q(0,0) = velN   * velN;
    Q(1,1) = steerN * steerN;

    // Escorregamento (random walk no tempo); slip x,y no frame do corpo.
    Eigen::Matrix2d Sxy = Eigen::Matrix2d::Zero();
    Sxy(0,0) = slipX * slipX * dt;
    Sxy(1,1) = slipY * slipY * dt;
    Eigen::Matrix2d R;
    R << c, -s, s, c;
    Eigen::Matrix3d Qslip = Eigen::Matrix3d::Zero();
    Qslip.block<2,2>(0,0) = R * Sxy * R.transpose();
    Qslip(2,2) = slipTheta * slipTheta * dt;

    Sigma = Fx * Sigma * Fx.transpose() + Fu * Q * Fu.transpose() + Qslip;

    x   += v * c * dt;
    y   += v * s * dt;
    yaw += dyaw;
}

// Constroi a covariancia 6x6 do BetweenFactor a partir da covariancia 3x3
// (x, y, theta). Ordem GTSAM: [roll, pitch, yaw, x, y, z]. roll/pitch/z recebem
// variancia grande (o LiDAR determina a atitude fora do plano).
static gtsam::Matrix bicycleCov6FromSigma(const Eigen::Matrix3d& Sigma)
{
    const double BIG = 1e6;
    // FIX: piso de variancia era 1e-9. Combinado com BIG=1e6 na mesma matriz
    // 6x6, isso da uma razao de 1e15 entre os menores/maiores autovalores --
    // bem no limite da precisao double (~1e-15/1e-16). Quando o Sigma vindo
    // da integracao Ackermann fica exatamente zero (nenhuma amostra integrada
    // nessa janela -- ver aviso novo em odometryHandler), o cov6 resultante e
    // tao mal-condicionado que o Cholesky/QR do ISAM2 perde posto e estoura
    // IndeterminantLinearSystemException, mesmo a matriz sendo teoricamente
    // PD. Elevar o piso pra 1e-4 mantem a mesma ordem de grandeza do resto do
    // grafo (correctionNoise ~0.05-0.1) e derruba a razao pra ~1e10, dentro
    // da faixa que o ISAM2 fatoriza sem problema.
    const double MIN_VAR = 1e-4;
    gtsam::Matrix cov6 = gtsam::Matrix::Zero(6,6);
    cov6(0,0) = BIG;                 // roll
    cov6(1,1) = BIG;                 // pitch
    cov6(5,5) = BIG;                 // z
    cov6(2,2) = std::max(Sigma(2,2), MIN_VAR);   // yaw
    cov6(3,3) = std::max(Sigma(0,0), MIN_VAR);   // x
    cov6(4,4) = std::max(Sigma(1,1), MIN_VAR);   // y
    cov6(2,3) = Sigma(2,0); cov6(3,2) = Sigma(0,2); // yaw-x
    cov6(2,4) = Sigma(2,1); cov6(4,2) = Sigma(1,2); // yaw-y
    cov6(3,4) = Sigma(0,1); cov6(4,3) = Sigma(1,0); // x-y
    return cov6;
}

static constexpr double kMaxIntegrationDt = 1.0;
static constexpr double kMinPreintDt = 1e-4;

static bool integrationDtOk(double dt)
{
    return dt > 0.0 && dt <= kMaxIntegrationDt;
}

template<typename OptimizerT>
static bool runOptimizerUpdate(OptimizerT &optimizer,
                               gtsam::NonlinearFactorGraph &graphFactors,
                               gtsam::Values &graphValues,
                               const char *tag)
{
    try {
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        return true;
    } catch (const gtsam::IndeterminantLinearSystemException &e) {
        ROS_WARN("%s: singular graph (%s), resetting preintegration", tag, e.what());
        return false;
    }
}

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subLocReset;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;
    bool haveLidar2Baselink_ = false;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    void ensureLidar2Baselink()
    {
        if (haveLidar2Baselink_ || lidarFrame == baselinkFrame)
            return;
        try
        {
            tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            haveLidar2Baselink_ = true;
            ROS_INFO("\033[1;32mTransformFusion: extrinseca %s->%s lida do TF.\033[0m",
                     lidarFrame.c_str(), baselinkFrame.c_str());
        }
        catch (tf::TransformException &ex)
        {
            // TF ainda nao publicou -- segue com identidade ate o pointcloud_node
            // enviar a TF estatica no primeiro scan.
        }
    }

    TransformFusion()
    {
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());
        // NOVO: essa classe funde lidarOdomAffine (pose global do mapping) com
        // deltas tirados da própria fila imuOdomQueue. Sem isso, num loc_reset
        // a fila fica com entradas de ANTES e DEPOIS do salto de posição
        // misturadas, e o delta front/back vira lixo -> teleporte visual na
        // odometria publicada, mesmo com o snap/ICP correto.
        subLocReset = nh.subscribe<std_msgs::Header>("lio_sam/mapping/loc_reset", 1, &TransformFusion::locResetHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
    }

    void locResetHandler(const std_msgs::Header::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        ROS_WARN("TransformFusion: pose resetada manualmente, limpando fila de odometria IMU pra evitar teleporte.");
        imuOdomQueue.clear(); // Aqui é só imuOdomQueue
        lidarOdomTime = -1;   // espera a próxima odometry antes de fundir de novo
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        ensureLidar2Baselink();

        // static tf
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }

        if (imuOdomQueue.empty())
            return;

        // compute latest odometry
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);
        // Computando RPY da pose
        double roll_globalpos, pitch_globalpos, yaw_globalpos;
        tf::Matrix3x3(odom_2_baselink.getRotation()).getRPY(roll_globalpos, pitch_globalpos, yaw_globalpos);
        // Printando pose x y yaw timestamp
        printf("pose x %f y %f yaw %f timestamp:  %f\n", odom_2_baselink.getOrigin().x(), odom_2_baselink.getOrigin().y(), yaw_globalpos, odom_2_baselink.stamp_.toSec());

        // publish IMU path
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Subscriber subLocReset;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    double lastLocResetTime = -1.0;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    // Pós-reset manual (loc_reset) ou pós-failureDetection: usa noise mais
    // frouxo no pose_factor por alguns keyframes. Dá tempo do scan2MapOptimization
    // convergir direito contra o mapa carregado antes de forçar o filtro a
    // "explicar" erro residual do ICP como velocidade. Sem isso: erro de poucos
    // cm com dt curto pós-reset -> velocidade absurda -> failureDetection (30 m/s)
    // -> reseta tudo de novo -> loop de warnings a cada ~0.5s.
    int resetGraceKeyframesLeft = 0;
    static const int RESET_GRACE_KEYFRAMES = 5;

    // NOVO: distingue "cold start" real (nó acabou de subir, não tem estimativa
    // nenhuma -> zera vel/bias, é seguro) de um loc_reset manual (o robô NÃO
    // parou nem mudou de bias fisicamente, só a ESTIMATIVA de posição estava
    // errada). Zerar vel_/bias_ num loc_reset joga fora uma estimativa boa e
    // ainda cria um degrau artificial de velocidade que o otimizador tenta
    // reconciliar contra a pose corrigida -> é isso que gera o "voo"/teleporte
    // em loop, independente de quão frouxo o pose_factor esteja.
    bool preserveStateOnNextInit = false;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration()
    {
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLocReset = nh.subscribe<std_msgs::Header>("lio_sam/mapping/loc_reset", 1, &IMUPreintegration::locResetHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void locResetHandler(const std_msgs::Header::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        ROS_WARN("IMU Preintegration: Iniciando reset, mantendo fila IMU para integrar a transicao...");
        
        // Salva o marco temporal que veio da thread do ICP
        lastLocResetTime = msg->stamp.toSec(); 

        // AS DUAS LINHAS ABAIXO FORAM REMOVIDAS:
        // imuQueOpt.clear();
        // imuQueImu.clear();
        // O GTSAM precisa das leituras que chegaram DURANTE o cálculo do ICP para 
        // saber como o robô se moveu do frame do clique até o tempo atual.

        resetParams();   
        
        // ZERAR A VELOCIDADE para não sair voando
        preserveStateOnNextInit = true;   // FIX: preserva vel/bias no loc_reset manual    
        resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;   
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (odomMsg->header.stamp.toSec() < lastLocResetTime)
        {
            return; 
        }

        double currentCorrectionTime = ROS_TIME(odomMsg);


        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (systemInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity e bias: só zera se for cold start de verdade.
            // Num loc_reset manual, prevVel_/prevBias_ ainda têm os últimos
            // valores estimados antes do reset e são reaproveitados (só a
            // POSE estava errada, não a velocidade nem o bias do IMU).
            if (!preserveStateOnNextInit)
            {
                prevVel_  = gtsam::Vector3(0, 0, 0);
                prevBias_ = gtsam::imuBias::ConstantBias();
            }
            preserveStateOnNextInit = false;
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            if (lastLocResetTime > 0) {
                ROS_INFO("\033[1;32m----> CONECTADO: Localizacao estabelecida, retomando odometria IMU!\033[0m");
            }
            return;
        }


        // reset graph for speed
        if (key == 100)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                if (!integrationDtOk(dt))
                {
                    lastImuT_opt = imuTime;
                    imuQueOpt.pop_front();
                    continue;
                }
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        if (imuIntegratorOpt_->deltaTij() < kMinPreintDt)
        {
            ROS_WARN_THROTTLE(1.0, "IMU preintegration deltaT=%.6f, skipping keyframe",
                              imuIntegratorOpt_->deltaTij());
            return;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        // NOVO: durante o grace period pós-reset, trata a pose do lidar como
        // "degenerate" mesmo que não seja — evita que o primeiro/segundo scan
        // pós-snap (ainda convergindo contra o mapa) seja lido como salto real
        // e vire velocidade estourada.
        bool inResetGrace = (resetGraceKeyframesLeft > 0);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, (degenerate || inResetGrace) ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        if (inResetGrace)
            --resetGraceKeyframesLeft;
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        if (!runOptimizerUpdate(optimizer, graphFactors, graphValues, "IMUPreintegration"))
        {
            graphFactors.resize(0);
            graphValues.clear();
            resetParams();
            resetOptimization();
            resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;
            return;
        }
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;   // NOVO: evita reentrar no mesmo loop
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
                if (!integrationDtOk(dt))
                {
                    lastImuQT = imuTime;
                    continue;
                }

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        if (!integrationDtOk(dt))
        {
            lastImuT_imu = imuTime;
            return;
        }
        lastImuT_imu = imuTime;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};


// ─── Predição por odometria Ackermann (acoplamento firme GTSAM) ──────────────
//
// Alternativa ao IMUPreintegration para robôs de cinemática Ackermann que usam
// velocidade (v) e ângulo de esterçamento (phi) para a predição do movimento.
// Em vez de um dead reckoning determinístico, monta um grafo GTSAM pose-only
// (ISAM2) que funde probabilisticamente:
//   - BetweenFactor: movimento predito pelo modelo de bicicleta, com covariância
//     derivada de ackermannVelNoise/ackermannSteerNoise + escorregamento
//     (slip lateral/diagonal) modelado como random walk no tempo.
//   - PriorFactor: correção de pose do scan-to-map (LiDAR).
// Publica o MESMO tópico/contrato do IMUPreintegration
// (odomTopic+"_incremental", child_frame_id="odom_imu"), de modo que
// TransformFusion, imageProjection e mapOptimization funcionem sem alteração.
class AckermannPreintegration : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subAckermann;
    ros::Subscriber subOdometry;
    ros::Subscriber subLocReset;
    ros::Publisher  pubImuOdometry;

    bool systemInitialized = false;
    bool doneFirstOpt = false;
    double lastAckT_opt  = -1;
    double lastAckT_odom = -1;
    double lastLocResetTime = -1.0;

    // Grafo pose-only
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;
    const double delta_t = 0;
    int key = 1;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;

    gtsam::Pose3 prevPose_;   // âncora corrigida (resultado da última otimização)

    // Filas de mensagens Ackermann (v, phi) para integração
    std::deque<geometry_msgs::TwistStamped> ackQueOpt;   // consumida na correção
    std::deque<geometry_msgs::TwistStamped> ackQueOdom;  // re-propagação de alta taxa

    // Pose propagada de alta taxa (publicada). x, y, yaw evoluem pelo modelo
    // Ackermann; z, roll, pitch acompanham a última correção do LiDAR.
    double odomX = 0, odomY = 0, odomZ = 0;
    double odomRoll = 0, odomPitch = 0, odomYaw = 0;

    // Grace period pós loc_reset / failure: afrouxa o prior de pose por alguns
    // keyframes para o scan-to-map convergir antes de "cobrar" a predição.
    int resetGraceKeyframesLeft = 0;
    static const int RESET_GRACE_KEYFRAMES = 5;

    // Extrínseca lidar_link -> base_link. NÃO é fixa no código: é lida do TF
    // publicado em runtime pelo pointcloud_node.cpp (que por sua vez monta
    // T_robot_lidar a partir do param_daemon / sensor_board_1 via IPC). Se o
    // lidar estiver montado invertido, essa rotação carrega essa inversão.
    // O grafo do Ackermann continua operando em convenção lidar_link (igual
    // lio_sam/mapping/odometry), mas o modelo de bicicleta (v = velocidade no
    // eixo x do VEÍCULO) precisa ser aplicado em base_link. Por isso: entra
    // lidarPose -> vira baselinkPose (lidar2Baselink), e na hora de publicar
    // volta pra lidar_link (baselink2Lidar), pro TransformFusion continuar
    // recebendo a odometria incremental na mesma convenção de sempre.
    //
    // Lookup é LAZY (não-bloqueante): TransformFusion, construída logo depois
    // no main(), já bloqueia até 3s esperando esse mesmo TF. Se a gente
    // bloquear de novo aqui no construtor, os delays somam e o nó demora pra
    // começar a publicar map->odom/odom->base_link -- é exatamente esse gap
    // de startup que gera o erro de extrapolação no RViz/TF buffer.
    tf::TransformListener tfListener;
    tf::StampedTransform lidar2BaselinkTF;
    gtsam::Pose3 lidar2Baselink = gtsam::Pose3::identity();
    gtsam::Pose3 baselink2Lidar = gtsam::Pose3::identity();
    bool haveLidar2Baselink = false;

    AckermannPreintegration()
    {
        subAckermann = nh.subscribe<geometry_msgs::TwistStamped>(ackermannTopic, 2000, &AckermannPreintegration::ackermannHandler, this, ros::TransportHints().tcpNoDelay());
        subOdometry  = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5, &AckermannPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLocReset  = nh.subscribe<std_msgs::Header>("lio_sam/mapping/loc_reset", 1, &AckermannPreintegration::locResetHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic+"_incremental", 2000);

        if (lidarFrame == baselinkFrame)
            haveLidar2Baselink = true; // mesmo frame, extrínseca é identidade mesmo

        priorPoseNoise   = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m,m,m
        correctionNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());     // confiança na correção LiDAR
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                     // degenerate / grace

        ROS_INFO("AckermannPreintegration (GTSAM): topic=%s, L=%.3f m, velNoise=%.3f, steerNoise=%.3f, slip[x,y,th]=[%.3f,%.3f,%.3f]",
                 ackermannTopic.c_str(), wheelbase,
                 ackermannVelNoise, ackermannSteerNoise,
                 ackermannSlipNoiseX, ackermannSlipNoiseY, ackermannSlipNoiseTheta);
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastAckT_opt = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void locResetHandler(const std_msgs::Header::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        // Guarda o marco do reset e reinicia o grafo. Não zera a pose (evita
        // teleporte): a próxima correção do LiDAR reancora o grafo na posição
        // correta. Grace period afrouxa o prior por alguns keyframes.
        lastLocResetTime = msg->stamp.toSec();
        lastAckT_odom = -1;
        resetParams();
        resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;
    }

    // Atualiza (x, y, yaw) pelo modelo de bicicleta (ponto médio no yaw).
    void propagatePlanar(double v, double phi, double dt, double& x, double& y, double& yaw)
    {
        bicyclePropagateMean(v, phi, dt, wheelbase, x, y, yaw);
    }

    // Integra um passo atualizando média (x, y, yaw) e a covariância 3x3.
    void integrateStep(double v, double phi, double dt,
                       double& x, double& y, double& yaw, Eigen::Matrix3d& Sigma)
    {
        bicycleIntegrateStep(v, phi, dt, wheelbase,
                             ackermannVelNoise, ackermannSteerNoise,
                             ackermannSlipNoiseX, ackermannSlipNoiseY, ackermannSlipNoiseTheta,
                             x, y, yaw, Sigma);
    }

    // Constrói o BetweenFactor 6x6 a partir da covariância 3x3 (x,y,theta).
    gtsam::Matrix cov6FromSigma(const Eigen::Matrix3d& Sigma)
    {
        return bicycleCov6FromSigma(Sigma);
    }

    // Tenta pegar a extrínseca lidar_link->base_link do TF SEM bloquear (só
    // consulta o que já está no buffer agora). Chamado a cada odometryHandler
    // até conseguir; depois disso vira no-op (haveLidar2Baselink=true). Assim
    // não trava o startup do nó (que também tem TransformFusion esperando o
    // mesmo TF logo em seguida no main()).
    void ensureLidar2Baselink()
    {
        if (haveLidar2Baselink)
            return;
        try
        {
            tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2BaselinkTF);

            tf::Quaternion q = lidar2BaselinkTF.getRotation();
            tf::Vector3    t = lidar2BaselinkTF.getOrigin();
            lidar2Baselink = gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                                           gtsam::Point3(t.x(), t.y(), t.z()));
            baselink2Lidar = lidar2Baselink.inverse();
            haveLidar2Baselink = true;

            ROS_INFO("\033[1;32mAckermannPreintegration: extrinseca %s->%s lida do TF (dinamica via IPC/param_daemon).\033[0m",
                      lidarFrame.c_str(), baselinkFrame.c_str());
        }
        catch (tf::TransformException &ex)
        {
            // Ainda não publicou -- normal nos primeiros ciclos. Segue com
            // identidade (mesmo comportamento de antes) até conseguir.
        }
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        ensureLidar2Baselink();

        // Evita ancorar o grafo (systemInitialized) com a extrínseca errada
        // (identidade) se o TF real do lidar->base_link ainda não chegou.
        // Só nos primeiros ciclos, geralmente resolve em 1-2 mensagens.
        if (!haveLidar2Baselink && lidarFrame != baselinkFrame)
            return;

        if (odomMsg->header.stamp.toSec() < lastLocResetTime)
            return;

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // FIX: esse early-return era silencioso -- se a correcao do lidar chega
        // e ainda nao tem NENHUMA amostra Ackermann nova na fila, o ciclo
        // inteiro e' descartado (grafo fica um ciclo atrasado) sem deixar
        // rastro nenhum no log. Com odometria a ~80Hz e lidar a ~20Hz isso
        // deveria ser raro (~4 amostras por correcao), entao se esse WARN
        // aparecer com frequencia e' sinal de fila/lock/backlog entre os dois
        // callbacks -- e um candidato direto pro "mapa puxando" que nao vinha
        // do glitch de phi (esse ja tem log e guarda proprios, ver acima).
        if (ackQueOpt.empty())
        {
            ROS_WARN_THROTTLE(1.0,
                "AckermannPreintegration: correcao do lidar chegou (t=%.3f) sem NENHUMA amostra "
                "Ackermann nova na fila -- ciclo de correcao pulado inteiro (nao so degradado)",
                currentCorrectionTime);
            return;
        }

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // Lidar (mapOptmization) -> base_link (frame que o modelo Ackermann
        // assume como "x = frente do veículo"). Usa a extrínseca dinâmica
        // lida do TF no construtor. Se o lidar estiver invertido, é essa
        // conversão que corrige a direção "frente/trás" do modelo Ackermann.
        gtsam::Pose3 baselinkPose = lidarPose.compose(lidar2Baselink);

        // 0. inicialização do sistema
        if (systemInitialized == false)
        {
            resetOptimization();

            // descarta mensagens Ackermann antigas
            while (!ackQueOpt.empty())
            {
                if (ROS_TIME(&ackQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastAckT_opt = ROS_TIME(&ackQueOpt.front());
                    ackQueOpt.pop_front();
                }
                else
                    break;
            }

            prevPose_ = baselinkPose;
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            graphValues.insert(X(0), prevPose_);
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            seedOdomFromPose(prevPose_);
            key = 1;
            systemInitialized = true;
            doneFirstOpt = true;
            if (lastLocResetTime > 0)
                ROS_INFO("\033[1;32m----> CONECTADO: Localizacao estabelecida, retomando odometria Ackermann!\033[0m");
            return;
        }

        // reset periódico do grafo para manter o ISAM2 enxuto
        if (key == 100)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            resetOptimization();
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            graphValues.insert(X(0), prevPose_);
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();
            key = 1;
        }

        // 1. integra o movimento Ackermann entre duas correções -> relPose + cov
        double rx = 0, ry = 0, ryaw = 0;
        Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
        while (!ackQueOpt.empty())
        {
            geometry_msgs::TwistStamped *thisAck = &ackQueOpt.front();
            double ackTime = ROS_TIME(thisAck);
            if (ackTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastAckT_opt < 0) ? (1.0 / 50.0) : (ackTime - lastAckT_opt);
                if (!integrationDtOk(dt))
                {
                    lastAckT_opt = ackTime;
                    ackQueOpt.pop_front();
                    continue;
                }
                integrateStep(thisAck->twist.linear.x, thisAck->twist.angular.z, dt, rx, ry, ryaw, Sigma);
                lastAckT_opt = ackTime;
                ackQueOpt.pop_front();
            }
            else
                break;
        }

        // DIAGNOSTICO: se nenhuma amostra Ackermann foi integrada nesta janela
        // (Sigma ficou exatamente zero), o BetweenFactor sai com confianca
        // quase absoluta (piso MIN_VAR) enquanto o mapOptmization pode estar
        // varias correcoes atrasado (ver timestamps de /ackermann/odom_raw vs
        // /lio_sam/mapping/odometry_incremental) -- essa e a assinatura do
        // backlog que estamos investigando, nao um bug isolado do modelo.
        if (Sigma.isZero(0))
            ROS_WARN_THROTTLE(1.0,
                "AckermannPreintegration: key=%d sem amostra Ackermann integrada nesta janela "
                "(currentCorrectionTime=%.3f, lastAckT_opt=%.3f) -- possivel backlog do mapOptmization",
                key, currentCorrectionTime, lastAckT_opt);

        gtsam::Pose3 relPose = gtsam::Pose3(gtsam::Rot3::Yaw(ryaw), gtsam::Point3(rx, ry, 0.0));
        gtsam::noiseModel::Gaussian::shared_ptr motionNoise = gtsam::noiseModel::Gaussian::Covariance(cov6FromSigma(Sigma));
        graphFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key - 1), X(key), relPose, motionNoise));

        // fator de pose (correção LiDAR, já em base_link)
        bool inResetGrace = (resetGraceKeyframesLeft > 0);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), baselinkPose, (degenerate || inResetGrace) ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        if (inResetGrace)
            --resetGraceKeyframesLeft;

        // valor inicial predito
        gtsam::Pose3 propPose = prevPose_.compose(relPose);
        graphValues.insert(X(key), propPose);

        // otimiza
        if (!runOptimizerUpdate(optimizer, graphFactors, graphValues, "AckermannPreintegration"))
        {
            graphFactors.resize(0);
            graphValues.clear();
            resetParams();
            resetOptimization();
            resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;
            return;
        }
        graphFactors.resize(0);
        graphValues.clear();

        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));

        // 2. re-propaga a odometria de alta taxa a partir da pose corrigida
        seedOdomFromPose(prevPose_);
        double lastAckQT = -1;
        while (!ackQueOdom.empty() && ROS_TIME(&ackQueOdom.front()) < currentCorrectionTime - delta_t)
        {
            lastAckQT = ROS_TIME(&ackQueOdom.front());
            ackQueOdom.pop_front();
        }
        if (!ackQueOdom.empty())
        {
            for (int i = 0; i < (int)ackQueOdom.size(); ++i)
            {
                geometry_msgs::TwistStamped *thisAck = &ackQueOdom[i];
                double ackTime = ROS_TIME(thisAck);
                double dt = (lastAckQT < 0) ? (1.0 / 50.0) : (ackTime - lastAckQT);
                if (!integrationDtOk(dt))
                {
                    lastAckQT = ackTime;
                    continue;
                }
                propagatePlanar(thisAck->twist.linear.x, thisAck->twist.angular.z, dt, odomX, odomY, odomYaw);
                lastAckQT = ackTime;
            }
        }
        lastAckT_odom = lastAckQT;

        ++key;
        doneFirstOpt = true;
    }

    void seedOdomFromPose(const gtsam::Pose3& pose)
    {
        odomX = pose.x();
        odomY = pose.y();
        odomZ = pose.z();
        gtsam::Vector3 rpy = pose.rotation().rpy();
        odomRoll  = rpy(0);
        odomPitch = rpy(1);
        odomYaw   = rpy(2);
    }

    void ackermannHandler(const geometry_msgs::TwistStamped::ConstPtr& ackMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        ackQueOpt.push_back(*ackMsg);
        ackQueOdom.push_back(*ackMsg);

        if (doneFirstOpt == false)
            return;

        const double v   = ackMsg->twist.linear.x;   // m/s
        const double phi = clampSteerAngle(ackMsg->twist.angular.z);  // rad
        const double t   = ROS_TIME(ackMsg);
        const double omega = (wheelbase > 1e-6) ? (v * std::tan(phi) / wheelbase) : 0.0;

        double dt = (lastAckT_odom < 0) ? (1.0 / 50.0) : (t - lastAckT_odom);
        if (!integrationDtOk(dt))
        {
            lastAckT_odom = t;
            return;
        }
        lastAckT_odom = t;

        propagatePlanar(v, phi, dt, odomX, odomY, odomYaw);

        // odomX/Y/Z/RPY estão em base_link (é o frame que o modelo Ackermann
        // assume). Antes de publicar, converte de volta pra lidar_link, pra
        // manter o mesmo contrato que o TransformFusion espera (mesma
        // convenção de lio_sam/mapping/odometry).
        gtsam::Pose3 baselinkPoseOut(gtsam::Rot3::RzRyRx(odomRoll, odomPitch, odomYaw),
                                      gtsam::Point3(odomX, odomY, odomZ));
        gtsam::Pose3 lidarPoseOut = baselinkPoseOut.compose(baselink2Lidar);

        // =================================================================
        // CORREÇÃO: CONVERSÃO DO TWIST (VELOCIDADE) PARA O FRAME DO LIDAR
        // =================================================================
        // Cria vetores de velocidade linear e angular puros no base_link
        gtsam::Vector3 linVelBase(v, 0.0, 0.0);
        gtsam::Vector3 angVelBase(0.0, 0.0, omega);

        // Rotaciona esses vetores fisicamente para o eixo de coordenadas do LiDAR
        // (A matriz lidar2Baselink.rotation() contém a conversão exata, lidando com os 180°)
        gtsam::Vector3 linVelLidar = lidar2Baselink.rotation() * linVelBase;
        gtsam::Vector3 angVelLidar = lidar2Baselink.rotation() * angVelBase;

        // Publica a odometria incremental
        nav_msgs::Odometry odometry;
        odometry.header.stamp    = ackMsg->header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id  = "odom_imu";

        odometry.pose.pose.position.x = lidarPoseOut.translation().x();
        odometry.pose.pose.position.y = lidarPoseOut.translation().y();
        odometry.pose.pose.position.z = lidarPoseOut.translation().z();
        odometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            lidarPoseOut.rotation().roll(), lidarPoseOut.rotation().pitch(), lidarPoseOut.rotation().yaw());

        // Preenche as velocidades agora ALINHADAS com o LiDAR
        odometry.twist.twist.linear.x  = linVelLidar.x();
        odometry.twist.twist.linear.y  = linVelLidar.y();
        odometry.twist.twist.linear.z  = linVelLidar.z();
        odometry.twist.twist.angular.x = angVelLidar.x();
        odometry.twist.twist.angular.y = angVelLidar.y();
        odometry.twist.twist.angular.z = angVelLidar.z();

        pubImuOdometry.publish(odometry);
    }
};


// ─── Fusão IMU + Ackermann no mesmo grafo GTSAM ─────────────────────────────
//
// Estende o IMUPreintegration: mantém todo o grafo inercial (X/V/B + ImuFactor +
// bias BetweenFactor) e ACRESCENTA, a cada keyframe, um BetweenFactor<Pose3>
// entre X(k-1) e X(k) com o movimento predito pelo modelo de bicicleta Ackermann
// e sua covariância (velNoise/steerNoise + slip). O otimizador funde as duas
// fontes ponderadas pelos respectivos ruídos. A predição de alta taxa continua
// vindo da IMU (imuHandler), pois é a fonte de maior frequência.
//
// Requer IMU. Para operar sem IMU, use odometrySource="ackermann".
class FusionPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subAckermann;
    ros::Subscriber subOdometry;
    ros::Subscriber subLocReset;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    // Fila de mensagens Ackermann (v, phi) para o BetweenFactor da correção.
    std::deque<geometry_msgs::TwistStamped> ackQueOpt;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    double lastAckT_opt = -1;
    double lastLocResetTime = -1.0;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    int resetGraceKeyframesLeft = 0;
    static const int RESET_GRACE_KEYFRAMES = 5;

    bool preserveStateOnNextInit = false;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    // FIX: extrínseca lidar_link -> base_link, lida do TF em runtime (igual
    // AckermannPreintegration). O modelo de bicicleta do Ackermann assume
    // v = velocidade no eixo x do VEÍCULO (base_link), não do lidar. Sem essa
    // extrínseca, o delta do Ackermann era tratado como se já estivesse no
    // frame do lidar e composto direto com imu2Lidar/lidar2Imu (que é só a
    // extrínseca IMU<->Lidar, sem nenhuma rotação). Se o base_link está
    // montado invertido 180° em relação ao lidar, isso faz o modo "fusion"
    // (IMU+Ackermann juntos) sair invertido -- mesmo IMU sozinho e Ackermann
    // sozinho (que já usa lidar2Baselink corretamente) funcionando bem.
    tf::TransformListener tfListener;
    tf::StampedTransform lidar2BaselinkTF;
    gtsam::Pose3 lidar2Baselink = gtsam::Pose3::identity();
    gtsam::Pose3 baselink2Lidar = gtsam::Pose3::identity();
    bool haveLidar2Baselink = false;

    void ensureLidar2Baselink()
    {
        if (haveLidar2Baselink)
            return;
        try
        {
            tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2BaselinkTF);

            tf::Quaternion q = lidar2BaselinkTF.getRotation();
            tf::Vector3    t = lidar2BaselinkTF.getOrigin();
            lidar2Baselink = gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                                           gtsam::Point3(t.x(), t.y(), t.z()));
            baselink2Lidar = lidar2Baselink.inverse();
            haveLidar2Baselink = true;

            ROS_INFO("\033[1;32mFusionPreintegration: extrinseca %s->%s lida do TF.\033[0m",
                      lidarFrame.c_str(), baselinkFrame.c_str());
        }
        catch (tf::TransformException &ex)
        {
            // TF ainda não publicou -- segue com identidade até conseguir.
        }
    }

    FusionPreintegration()
    {
        subImu       = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &FusionPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        subAckermann = nh.subscribe<geometry_msgs::TwistStamped>(ackermannTopic,    2000, &FusionPreintegration::ackermannHandler, this, ros::TransportHints().tcpNoDelay());
        subOdometry  = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &FusionPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLocReset  = nh.subscribe<std_msgs::Header>("lio_sam/mapping/loc_reset", 1, &FusionPreintegration::locResetHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

        if (lidarFrame == baselinkFrame)
            haveLidar2Baselink = true; // mesmo frame, extrínseca é identidade mesmo

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2);
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

        ROS_INFO("FusionPreintegration (IMU + Ackermann): imu=%s, ack=%s, L=%.3f m, velNoise=%.3f, steerNoise=%.3f, slip[x,y,th]=[%.3f,%.3f,%.3f]",
                 imuTopic.c_str(), ackermannTopic.c_str(), wheelbase,
                 ackermannVelNoise, ackermannSteerNoise,
                 ackermannSlipNoiseX, ackermannSlipNoiseY, ackermannSlipNoiseTheta);
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void locResetHandler(const std_msgs::Header::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        ROS_WARN("Fusion Preintegration: iniciando reset, mantendo filas para integrar a transicao...");
        lastLocResetTime = msg->stamp.toSec();
        resetParams();
        preserveStateOnNextInit = true;   // FIX: preserva vel/bias no loc_reset manual
        resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;
    }

    // Integra o movimento Ackermann acumulado na fila até currentCorrectionTime,
    // produzindo relPose (frame LiDAR) + covariância 3x3. Retorna false se não
    // houve nenhuma mensagem Ackermann para consumir no intervalo.
    bool integrateAckermann(double currentCorrectionTime, gtsam::Pose3& relPose, Eigen::Matrix3d& Sigma)
    {
        double rx = 0, ry = 0, ryaw = 0;
        Sigma = Eigen::Matrix3d::Zero();
        bool any = false;
        while (!ackQueOpt.empty())
        {
            geometry_msgs::TwistStamped *thisAck = &ackQueOpt.front();
            double ackTime = ROS_TIME(thisAck);
            if (ackTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastAckT_opt < 0) ? (1.0 / 50.0) : (ackTime - lastAckT_opt);
                if (!integrationDtOk(dt))
                {
                    lastAckT_opt = ackTime;
                    ackQueOpt.pop_front();
                    continue;
                }
                bicycleIntegrateStep(thisAck->twist.linear.x, thisAck->twist.angular.z, dt, wheelbase,
                                     ackermannVelNoise, ackermannSteerNoise,
                                     ackermannSlipNoiseX, ackermannSlipNoiseY, ackermannSlipNoiseTheta,
                                     rx, ry, ryaw, Sigma);
                lastAckT_opt = ackTime;
                ackQueOpt.pop_front();
                any = true;
            }
            else
                break;
        }
        relPose = gtsam::Pose3(gtsam::Rot3::Yaw(ryaw), gtsam::Point3(rx, ry, 0.0));
        return any;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        ensureLidar2Baselink();

        // Evita ancorar/fundir com a extrínseca errada (identidade) antes do
        // TF real lidar->base_link chegar, igual ao AckermannPreintegration.
        if (!haveLidar2Baselink && lidarFrame != baselinkFrame)
            return;

        if (odomMsg->header.stamp.toSec() < lastLocResetTime)
            return;

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // precisa de dados de IMU para integrar
        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // 0. inicialização do sistema
        if (systemInitialized == false)
        {
            resetOptimization();

            // descarta mensagens IMU antigas
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // descarta mensagens Ackermann antigas
            while (!ackQueOpt.empty())
            {
                if (ROS_TIME(&ackQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastAckT_opt = ROS_TIME(&ackQueOpt.front());
                    ackQueOpt.pop_front();
                }
                else
                    break;
            }

            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            if (!preserveStateOnNextInit)
            {
                prevVel_  = gtsam::Vector3(0, 0, 0);
                prevBias_ = gtsam::imuBias::ConstantBias();
            }
            preserveStateOnNextInit = false;
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;
            if (lastLocResetTime > 0)
                ROS_INFO("\033[1;32m----> CONECTADO: Localizacao estabelecida, retomando fusao IMU+Ackermann!\033[0m");
            return;
        }

        // reset periódico do grafo para manter o ISAM2 enxuto
        if (key == 100)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            resetOptimization();
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();
            key = 1;
        }

        // 1. integra a IMU e adiciona o ImuFactor
        while (!imuQueOpt.empty())
        {
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                if (!integrationDtOk(dt))
                {
                    lastImuT_opt = imuTime;
                    imuQueOpt.pop_front();
                    continue;
                }
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        if (imuIntegratorOpt_->deltaTij() < kMinPreintDt)
        {
            ROS_WARN_THROTTLE(1.0, "Fusion preintegration deltaT=%.6f, skipping keyframe",
                              imuIntegratorOpt_->deltaTij());
            return;
        }
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        // 1b. FUSÃO: BetweenFactor do modelo Ackermann entre X(key-1) e X(key).
        // FIX: integrateAckermann() devolve o delta do modelo de bicicleta, que
        // é naturalmente expresso em base_link (x = frente do VEÍCULO), e não
        // no frame do lidar como o nome/comentário antigo sugeria. Compor esse
        // delta direto com imu2Lidar/lidar2Imu (que é só a extrínseca IMU<->
        // Lidar) pulava a rotação lidar<->base_link. Enquanto essa rotação é
        // identidade (base_link == lidar_link em orientação) não dá pra notar;
        // com o base_link montado invertido 180° em relação ao lidar, o delta
        // do Ackermann entrava invertido na fusão -- por isso IMU sozinho e
        // Ackermann sozinho (AckermannPreintegration já usa lidar2Baselink)
        // funcionavam, e só o modo "fusion" saía invertido.
        gtsam::Pose3 relPoseBaselink;
        Eigen::Matrix3d ackSigma;
        if (integrateAckermann(currentCorrectionTime, relPoseBaselink, ackSigma))
        {
            gtsam::Pose3 relPoseLidar = lidar2Baselink.compose(relPoseBaselink).compose(baselink2Lidar);
            gtsam::Pose3 relPoseImu = imu2Lidar.compose(relPoseLidar).compose(lidar2Imu);
            gtsam::noiseModel::Gaussian::shared_ptr motionNoise = gtsam::noiseModel::Gaussian::Covariance(bicycleCov6FromSigma(ackSigma));
            graphFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key - 1), X(key), relPoseImu, motionNoise));
        }

        // fator de pose (correção LiDAR)
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        bool inResetGrace = (resetGraceKeyframesLeft > 0);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, (degenerate || inResetGrace) ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        if (inResetGrace)
            --resetGraceKeyframesLeft;

        // valores iniciais preditos (pela IMU)
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        // otimiza
        if (!runOptimizerUpdate(optimizer, graphFactors, graphValues, "FusionPreintegration"))
        {
            graphFactors.resize(0);
            graphValues.clear();
            resetParams();
            resetOptimization();
            resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;
            return;
        }
        graphFactors.resize(0);
        graphValues.clear();
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            resetGraceKeyframesLeft = RESET_GRACE_KEYFRAMES;
            return;
        }

        // 2. re-propaga a odometria IMU de alta taxa a partir do estado otimizado
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        if (!imuQueImu.empty())
        {
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
                if (!integrationDtOk(dt))
                {
                    lastImuQT = imuTime;
                    continue;
                }
                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset Fusion-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset Fusion-preintegration!");
            return true;
        }

        return false;
    }

    void ackermannHandler(const geometry_msgs::TwistStamped::ConstPtr& ackMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        ackQueOpt.push_back(*ackMsg);
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        if (!integrationDtOk(dt))
        {
            lastImuT_imu = imuTime;
            return;
        }
        lastImuT_imu = imuTime;

        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");

    ros::NodeHandle nh_private;
    std::string odometrySource;
    nh_private.param<std::string>("lio_sam/odometrySource", odometrySource, "imu");

    std::unique_ptr<IMUPreintegration>       imuP;
    std::unique_ptr<AckermannPreintegration> ackP;
    std::unique_ptr<FusionPreintegration>    fusP;

    if (odometrySource == "fusion")
    {
        fusP.reset(new FusionPreintegration());
        ROS_INFO("\033[1;32m----> Fusion (IMU + Ackermann) Preintegration Started.\033[0m");
    }
    else if (odometrySource == "ackermann")
    {
        ackP.reset(new AckermannPreintegration());
        ROS_INFO("\033[1;32m----> Ackermann (dead reckoning) Preintegration Started.\033[0m");
    }
    else
    {
        imuP.reset(new IMUPreintegration());
        ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    }

    TransformFusion TF;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}