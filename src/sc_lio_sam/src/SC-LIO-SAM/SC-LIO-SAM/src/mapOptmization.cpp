#include "utility.h"

#include "lio_sam/cloud_info.h"

#include <atomic>
#include <map>
#include <deque>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h>
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

#include "Scancontext.h"
#include "cuda_map_search.h"


using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose


void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

// giseop
enum class SCInputType { 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 

class mapOptimization : public ParamServer
{

public:

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubLocReset;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge; 

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;
    ros::Subscriber subInitialPose; // localization: RViz "2D Pose Estimate"

    // Localization mode state (non-static so the pose can be reset at runtime)
    bool locPoseInitialized = false;
    bool locResetRequested = false;
    // Mapa global de visualizacao (localizationMode): a nuvem e' fixa, entao e'
    // montada/voxelizada UMA vez e cacheada aqui; publishGlobalMap() so' atualiza
    // o header.stamp e republica. Ver o comentario la' sobre por que republicar e'
    // obrigatorio (TF do Fixed Frame do RViz expira pro stamp antigo).
    sensor_msgs::PointCloud2 locGlobalMapMsg;
    bool locGlobalMapEverPublished = false;
    // FIX: gate separada pro auto-relocalize via Scan Context (loopClosureThread,
    // roda a ~1Hz). ANTES essa checagem usava "!locPoseInitialized", mas essa
    // flag e' setada em updateInitialGuessLocalization() -- chamada a cada scan
    // do LiDAR (10-20Hz), no callback principal. Como o callback do LiDAR roda
    // muito mais rapido que a thread de loop closure, locPoseInitialized virava
    // true no PRIMEIRO scan (dezenas de ms apos o boot), antes da thread de 1Hz
    // conseguir rodar scManager.detectLoopClosureID() uma unica vez -- ou seja,
    // o auto-snap por Scan Context praticamente nunca disparava, e o robo
    // ficava preso na pose semente (initialPoseX/Y/Z do yaml) so andando por
    // odometria a partir dali.
    std::atomic<bool> locAutoRelocArmed{true};

    // Ancora GPS(raw)<->mapa. Calculada uma vez em modo mapeamento (ver
    // gpsHandler) e persistida em savePCDDirectory/gps_utm_anchor.txt; em
    // localizationMode e' lida desse arquivo no loadMap(). Sem ela, GPS e'
    // simplesmente ignorado (gpsAnchorValid fica false pra sempre).
    bool gpsAnchorValid = false;
    double gpsAnchorX = 0.0, gpsAnchorY = 0.0, gpsAnchorYaw = 0.0;

    // Semente da ancora: primeiro par (fix GPS bruto, pose SLAM) guardado. O rumo da
    // ancora sai da DIRECAO DE PERCURSO entre a semente e o par atual, nao do heading
    // instantaneo do GPS -- ver o comentario grande em gpsHandler().
    bool   gpsAnchorSeedValid = false;
    double gpsAnchorSeedRawX = 0.0, gpsAnchorSeedRawY = 0.0;
    double gpsAnchorSeedMapX = 0.0, gpsAnchorSeedMapY = 0.0;
    Eigen::Affine3f locLastImuPreTransformation;
    bool locLastImuPreTransAvailable = false;
    Eigen::Affine3f locLastImuTransformation;
    double timeLastLocReset = -1.0;

    // NOVO: pra poder reancorar o odom incremental quando a pose é resetada manualmente
    bool lastIncreOdomPubFlag = false;
    nav_msgs::Odometry laserOdomIncremental;
    Eigen::Affine3f increOdomAffine;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lio_sam::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses2D; // giseop 
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudRaw; // giseop
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS; // giseop
    pcl::PointCloud<PointType>::Ptr locIcpTargetMap;
    double laserCloudRawTime;

    // NOVO: snap de localização assíncrono, pra não travar o ros::spin() com o ICP
    std::mutex mtxLocSnap;
    std::atomic<bool> locSnapPending{false};
    // True assim que QUALQUER snap terminou aplicando uma pose ao robo -- por ICP
    // bem sucedido ou pelo chute bruto de fallback, vindo do GPS, do Scan Context ou
    // do clique no RViz. Diferente de locAutoRelocArmed, que significa apenas
    // "ninguem PEDIU um snap ainda": aqui a semantica e' "o robo ja' foi encaixado
    // alguma vez". E' isso que trava o auto-relocalize por GPS de ficar brigando com
    // o encaixe manual depois que o robo ja' esta' posicionado (ver gpsHandler).
    std::atomic<bool> locEverAnchored{false};
    // Conta quantos encaixes ja' foram aplicados. Cada snap novo e' um SALTO na
    // trajetoria; writeLocalizationPose() usa isso pra descartar alguns scans logo
    // depois e pra avisar quando o arquivo ganha uma descontinuidade no meio.
    std::atomic<int> locAnchorCount{0};
    // Trava de rotacao do modo ackermann (ver rotationLockActive/transformUpdate):
    // atitude congelada do primeiro scan, mantida ate' o mapa ficar pronto.
    bool  rotLockCaptured = false;
    float rotLocked[3] = {0.0f, 0.0f, 0.0f};   // roll, pitch, yaw
    // Pose odometrica do robo no instante em que o scan do snap foi capturado.
    // Protegida por mtxLocSnap, junto com locSnapGuess/locSnapScan.
    Eigen::Affine3f locSnapPoseAtRequest = Eigen::Affine3f::Identity();
    // Raio (m) em torno do chute usado pra recortar o alvo do ICP de encaixe. Precisa
    // ser bem maior que o setMaxCorrespondenceDistance (20 m) pra dar folga a chutes
    // imprecisos, e bem menor que o mapa todo pra o ICP terminar em tempo util.
    float locSnapCropRadius = 150.0f;
    // Voxel do alvo do ICP de encaixe. O cloudGlobal.pcd vem sem downsample (~18,7 M
    // pontos), o que torna o ICP inviavel; 0.4 m casa com mappingSurfLeafSize e e'
    // fino o bastante pro encaixe (o scan de origem ja' vem a 0.5 m do downSizeFilterSC).
    float locSnapTargetLeafSize = 0.4f;
    // Versao barateada do locIcpTargetMap, montada uma unica vez (lazy) no locSnapWorker.
    pcl::PointCloud<PointType>::Ptr locIcpTargetMapDS;
    // Contador de pedidos de snap. O locSnapWorker guarda o valor no inicio do ICP e
    // so' desarma locSnapPending se ele nao mudou -- assim um pedido novo que chegue
    // durante o ICP (tipicamente o clique do usuario no RViz) nao e' perdido.
    std::atomic<uint32_t> locSnapSeq{0};
    float locSnapGuess[6];
    pcl::PointCloud<PointType>::Ptr locSnapScan;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    // GPU map search state. cudaAvailable é setado uma vez no construtor.
    // cornerMapUploadedToGpu/surfMapUploadedToGpu dizem se o upload do mapa
    // local desta cena teve sucesso -- se não, cornerOptimization/
    // surfOptimization caem pro kdtree normal SEM precisar checar cudaAvailable
    // de novo (evita reupload falho silencioso virar busca contra mapa velho).
    bool cudaAvailable = false;
    bool cornerMapUploadedToGpu = false;
    bool surfMapUploadedToGpu = false;

    pcl::VoxelGrid<PointType> downSizeFilterSC; // giseop
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    // map<int, int> loopIndexContainer; // from new to old
    multimap<int, int> loopIndexContainer; // from new to old // giseop 

    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    // vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue; // Diagonal <- Gausssian <- Base
    vector<gtsam::SharedNoiseModel> loopNoiseQueue; // giseop for polymorhpisam (Diagonal <- Gausssian <- Base)

    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Affine3f incrementalOdometryAffineGuess; // chute puro (ackermann/imu), antes do ICP
    bool incrementalOdometryGuessFromOdom = false;  // true só se o Guess veio de odom real (ackermann), não do fallback de IMU

    // Histórico do gate anti-salto (ver transformUpdate()): últimas N correções
    // scan-match vs. odometria, usadas pra estimar um limiar robusto (mediana +
    // k*MAD) em vez de uma constante chutada.
    const float  locSnapFitnessScore    = 1.5f;

    // // loop detector 
    SCManager scManager;

    // data saver
    std::fstream pgSaveStream; // pg: pose-graph 
    std::fstream pgTimeSaveStream; // pg: pose-graph 
    std::vector<std::string> edges_str;
    std::vector<std::string> vertices_str;
    // std::fstream pgVertexSaveStream;
    // std::fstream pgEdgeSaveStream;

    std::string saveSCDDirectory;
    std::string saveNodePCDDirectory;

    // Fila de gravacao assincrona dos Scans/*.pcd (ver saveKeyFramesAndFactor e
    // pcdWriterThread). Mantem o I/O de ~2,5 MB por keyframe fora do callback do LiDAR.
    struct PendingPcd { std::string path; pcl::PointCloud<PointType>::Ptr cloud; };
    std::deque<PendingPcd> pcdWriteQueue;
    std::mutex mtxPcdQueue;
    static const size_t kPcdQueueMax = 64;   // ~160 MB de teto

    // Localization pose logging (graphslam poses_opt.dat format)
    std::ofstream posesOutStream;
    bool posesOutEnabled = false;
    tf::Transform T_posesOrigin = tf::Transform::getIdentity(); // deslocamento rigido opcional
    tf::Transform baselink2Lidar = tf::Transform::getIdentity(); // cached base_link -> lidar
    bool baselink2LidarCached = false;
    size_t posesWritten = 0;
    size_t posesSkippedNoCarmenStamp = 0;
    int    posesLastAnchorSeen = -1;   // pra detectar snap novo no meio da gravacao
    int    posesSettleLeft = 0;        // scans a descartar apos cada encaixe

    // Ponte de tempo CARMEN <-> ROS, publicada pelo pointcloud_node. O stamp ROS da
    // nuvem e' o INICIO da varredura; o CARMEN carimba o FIM. O graphslam_publish
    // casa timestamps com tolerancia de 1 us, entao o valor tem que vir de la',
    // nao ser reconstruido aqui.
    ros::Subscriber subCarmenScanTime;
    ros::Subscriber subAckermannStamps;
    std::mutex mtxCarmenStamps;
    std::map<int64_t, double> carmenStampByRosNs;   // chave: stamp ROS da nuvem, em ns
    std::deque<double> ackermannCarmenStamps;       // timestamps CARMEN da odometria
    std::deque<double> gpsCarmenStamps;             // timestamps CARMEN do gps_xyz
    static const size_t kCarmenStampMax = 4000;     // ~200 s a 20 Hz (~64 KB)
    static const size_t kAckermannStampMax = 800;
    static const size_t kGpsStampMax = 200;        // GPS e' lento (~5-10 Hz)

    // Interpolacao para os scans que o mapOptimization nao chegou a processar.
    // A ponte de tempo do bridge publica UM stamp por scan, inclusive os que a
    // fila do subCloud descartou -- entao da' pra emitir a pose deles sem exigir
    // que o no' acompanhe 20 Hz.
    // ─── Origem do mundo estimada pelo GPS ───────────────────────────────────
    // T_world_map: leva o frame do mapa (SC-LIO-SAM) pro frame do CARMEN/UTM.
    // Cada fix de GPS que casa no tempo com uma pose SLAM da' uma amostra:
    //     yaw_origem = yaw_utm - yaw_map          (media circular)
    //     t_origem   = p_utm  - Rz(yaw_origem) * p_map
    // Media circular no rumo e media simples na translacao. Usar o RUMO do GPS
    // (e nao ajustar rotacao pela nuvem de posicoes) faz isso funcionar mesmo
    // com o carro parado, quando nao ha' dispersao pra estimar rotacao.
    std::mutex mtxOrigin;
    size_t originSamples = 0;
    // Somas PONDERADAS por 1/covariancia: fix ruim entra, mas pesando pouco.
    double originWsumT = 0.0, originWsumY = 0.0;   // soma dos pesos (transl./rumo)
    double originSumSin = 0.0, originSumCos = 0.0;
    double originSumMapX = 0.0, originSumMapY = 0.0, originSumMapZ = 0.0;
    double originSumUtmX = 0.0, originSumUtmY = 0.0, originSumUtmZ = 0.0;
    // Origem corrente ja' resolvida (o que vai pro txt).
    double originX = 0.0, originY = 0.0, originZ = 0.0;
    double originRoll = 0.0, originPitch = 0.0, originYaw = 0.0;
    size_t originLastSaved = 0;
    std::atomic<bool> posesOriginReady{false};
    // Contadores de diagnostico: dizem QUAL portao esta' barrando as amostras.
    size_t originGpsMsgs = 0, originRejNotAnchored = 0, originRejTooOld = 0, originAccepted = 0;
    // Origem das medidas de rumo: bussola do GPS vs deslocamento entre fixes.
    size_t originHeadingFromCompass = 0, originHeadingFromMotion = 0;
    bool   haveLastOriginSample = false;
    double lastSampleMapX = 0.0, lastSampleMapY = 0.0;
    double lastSampleUtmX = 0.0, lastSampleUtmY = 0.0;

    // Fixes de GPS aguardando uma pose SLAM que os cubra no tempo.
    //
    // Nao da' pra exigir que o fix caia perto da ULTIMA pose concluida: o
    // mapOptimization processa a ~4-7 Hz enquanto o relogio simulado corre a
    // 20 Hz, entao a ultima pose ja' nasce 150-250 ms atrasada e quase todo fix
    // cairia fora da janela. Em vez disso os fixes ficam na fila e sao casados
    // com a pose INTERPOLADA entre os dois scans que os cercam -- mesma ideia da
    // interpolacao do poses_opt.dat.
    struct PendingGpsFix {
        double t = 0.0;
        double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
        double wT = 0.0, wY = 0.0;
    };
    std::deque<PendingGpsFix> pendingGpsFixes;
    static const size_t kPendingGpsMax = 400;

    // Duas ultimas poses SLAM CRUAS (frame do mapa, antes do T_posesOrigin).
    bool          haveLastRawPose = false;
    tf::Transform lastRawPose = tf::Transform::getIdentity();
    double        lastRawPoseStamp = 0.0;
    bool          havePrevRawPose = false;
    tf::Transform prevRawPose = tf::Transform::getIdentity();
    double        prevRawPoseStamp = 0.0;

    bool          posesHavePrev = false;
    int64_t       posesPrevRosNs = 0;
    double        posesPrevCarmenTs = 0.0;
    tf::Transform posesPrevT = tf::Transform::getIdentity();
    size_t        posesInterpolated = 0;

public:
    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        // FIX: latched (3o arg true). Em localizationMode o mapa so' e' publicado
        // UMA VEZ (ver publishGlobalMap()); sem latch, se o RViz demorar a
        // conectar o subscriber (comum, ele ainda ta subindo/carregando a
        // config), essa unica publicacao se perde pra sempre e o mapa nunca
        // mais aparece nessa sessao -- exatamente o "as vezes aparece, as
        // vezes nao" que fica dependendo de timing de boot do RViz.
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1, true);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_incremental", 1);
        pubLocReset                 = nh.advertise<std_msgs::Header>("lio_sam/mapping/loc_reset", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

        subCloud        = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", slamQueueSize, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS          = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop         = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subInitialPose  = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 8, &mapOptimization::initialPoseHandler, this, ros::TransportHints().tcpNoDelay());

        subInitialPose  = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 8, &mapOptimization::initialPoseHandler, this, ros::TransportHints().tcpNoDelay());
        pubLocReset     = nh.advertise<std_msgs::Header>("lio_sam/mapping/loc_reset", 1);   // NOVO: agora carrega o timestamp do frame corrigido (era Empty)

        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        const float kSCFilterSize = 0.5; // giseop
        downSizeFilterSC.setLeafSize(kSCFilterSize, kSCFilterSize, kSCFilterSize); // giseop

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        allocateMemory();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        saveSCDDirectory = savePCDDirectory + "SCDs/"; // SCD: scan context descriptor 
        saveNodePCDDirectory = savePCDDirectory + "Scans/";

        if (localizationMode)
        {
            // Localization mode: reuse an existing map. Do NOT erase/recreate the map
            // directory nor truncate any of its files. Just load the map and localize.
            loadMap();

            // Optional: log estimated poses in the graphslam poses_opt.dat format.
            if (!posesEnable)
            {
                ROS_INFO("Gravacao de poses DESLIGADA (lio_sam/posesEnable: false).");
            }
            else if (posesOutputFile.empty())
            {
                ROS_INFO("Gravacao de poses desligada: lio_sam/posesOutputFile esta' vazio.");
            }
            else
            {
                size_t slashPos = posesOutputFile.find_last_of('/');
                if (slashPos != std::string::npos && slashPos > 0)
                {
                    std::string parentDir = posesOutputFile.substr(0, slashPos);
                    int unused = system((std::string("mkdir -p '") + parentDir + "'").c_str());
                    (void)unused;
                }

                posesOutStream.open(posesOutputFile.c_str(), std::ofstream::out | std::ofstream::trunc);
                if (posesOutStream.is_open())
                {
                    posesOutStream << std::fixed;
                    posesOutEnabled = true;

                    tf::Quaternion q_origin;
                    q_origin.setRPY(0.0, 0.0, posesOriginYaw);
                    T_posesOrigin = tf::Transform(q_origin,
                        tf::Vector3(posesOriginX, posesOriginY, posesOriginZ));

                    // Fila grande de proposito: o ros::spin() e' single-thread, e
                    // enquanto o laserCloudInfoHandler processa um scan os stamps
                    // se acumulam aqui. Perder um stamp = perder a pose daquele scan.
                    subCarmenScanTime = nh.subscribe<sensor_msgs::TimeReference>(
                        posesCarmenStampTopic, 400, &mapOptimization::carmenScanTimeHandler, this,
                        ros::TransportHints().tcpNoDelay());
                    subAckermannStamps = nh.subscribe<geometry_msgs::TwistStamped>(
                        posesAckermannTopic, 200, &mapOptimization::ackermannStampHandler, this,
                        ros::TransportHints().tcpNoDelay());

                    ROS_INFO("Localization mode: gravando poses (formato graphslam) em '%s'.", posesOutputFile.c_str());
                    ROS_INFO("  Comeca a gravar so' depois que o robo encaixar no mapa "
                             "(+%d scans de acomodacao). Timestamp CARMEN vem de '%s'; "
                             "timestamp de odometria de '%s'.",
                             posesSettleScans, posesCarmenStampTopic.c_str(), posesAckermannTopic.c_str());
                    if (posesOriginMode == "gps")
                    {
                        // Reaproveita a origem que ficou salva junto do mapa (pode
                        // ter sido calculada durante o mapeamento). Se o txt nao
                        // existir, comeca do zero -- e o robo so' grava poses
                        // depois de o GPS fixar a origem de novo.
                        if (!loadWorldOrigin())
                            ROS_INFO("  Origem do mundo: MODO GPS, sem '%s' -- vai calcular do zero.",
                                     worldOriginFile().c_str());
                        if (posesOriginLock)
                            ROS_INFO("  Origem do mundo: TRAVADA (posesOriginLock: true) -- o GPS "
                                     "nao atualiza mais nada.");
                        else
                            ROS_INFO("  Origem do mundo: MODO GPS (%d amostras minimas, refino %s). "
                                     "Fix ruim entra pesando 1/covariancia, nao e' descartado.",
                                     posesOriginMinSamples, posesOriginRefine ? "LIGADO" : "desligado");
                    }
                    else if (posesOriginX != 0.0 || posesOriginY != 0.0 || posesOriginYaw != 0.0)
                        ROS_INFO("  Origem do mundo: MANUAL x=%.3f y=%.3f z=%.3f yaw=%.6f.",
                                 posesOriginX, posesOriginY, posesOriginZ, posesOriginYaw);
                    else
                        ROS_INFO("  Coordenadas puras do SC-LIO-SAM (sem deslocamento de origem).");
                }
                else
                {
                    ROS_ERROR("Localization mode: nao foi possivel abrir '%s' para gravar poses. Gravacao desativada.",
                              posesOutputFile.c_str());
                }
            }
        }
        else
        {
            // giseop
            // create directory and remove old files;
            // savePCDDirectory = std::getenv("HOME") + savePCDDirectory; // rather use global path
            int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
            unused = system((std::string("mkdir ") + savePCDDirectory).c_str());

            unused = system((std::string("exec rm -r ") + saveSCDDirectory).c_str());
            unused = system((std::string("mkdir -p ") + saveSCDDirectory).c_str());

            unused = system((std::string("exec rm -r ") + saveNodePCDDirectory).c_str());
            unused = system((std::string("mkdir -p ") + saveNodePCDDirectory).c_str());

            pgSaveStream = std::fstream(savePCDDirectory + "singlesession_posegraph.g2o", std::fstream::out);
            pgTimeSaveStream = std::fstream(savePCDDirectory + "times.txt", std::fstream::out); pgTimeSaveStream.precision(dbl::max_digits10);
            // pgVertexSaveStream = std::fstream(savePCDDirectory + "singlesession_vertex.g2o", std::fstream::out);
            // pgEdgeSaveStream = std::fstream(savePCDDirectory + "singlesession_edge.g2o", std::fstream::out);
        }

        // Resumo do gate anti-giro/anti-salto no boot. Sem isto nao da' pra saber, so'
        // olhando o log, se os parametros do params.yaml chegaram mesmo no no' -- e' o
        // primeiro lugar a conferir quando "mexi no peso e nao mudou nada".
        if (odometrySource == "ackermann" || odometrySource == "fusion")
        {
            ROS_INFO("Ackermann: gate anti-giro -- yaw peso=%.2f teto=%.1f deg | pos peso=%.2f teto=%.2f m | "
                     "rejeita acima de %.1f deg / %.2f m. (0 em teto/rejeicao = desligado, peso 1.0 = 100%% LiDAR)",
                     ackermannYawWeight, ackermannYawMaxCorrDeg,
                     ackermannPosWeight, ackermannPosMaxCorr,
                     ackermannRejectYawDeg, ackermannRejectPosM);
        }
    }

    // Gravacao final. Em localizationMode o caminho de salvar mapa nao roda
    // (saveMap() retorna cedo), entao sem isto o map_world_origin.txt ficaria
    // parado na ultima gravacao periodica -- ate' 50 amostras atras, justamente
    // as melhores, ja' que o refino so' melhora com o tempo.
    ~mapOptimization()
    {
        if (posesOriginReady.load())
        {
            saveWorldOrigin();
            ROS_INFO("Origem do mundo gravada no encerramento: '%s' (%zu amostras).",
                     worldOriginFile().c_str(), originSamples);
        }
        else if (posesOriginMode == "gps")
        {
            ROS_WARN("Encerrando sem origem do mundo valida: %zu amostras de %d "
                     "(fixes=%zu, sem_encaixe=%zu, fora_de_alcance=%zu). "
                     "'%s' NAO foi gravado.",
                     originSamples, posesOriginMinSamples, originGpsMsgs,
                     originRejNotAnchored, originRejTooOld, worldOriginFile().c_str());
        }

        if (posesOutStream.is_open())
        {
            posesOutStream.flush();
            posesOutStream.close();
            ROS_INFO("poses_opt: %zu poses gravadas (%zu interpoladas) em '%s'.",
                     posesWritten, posesInterpolated, posesOutputFile.c_str());
        }
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses2D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudRaw.reset(new pcl::PointCloud<PointType>()); // giseop
        laserCloudRawDS.reset(new pcl::PointCloud<PointType>()); // giseop
        locIcpTargetMap.reset(new pcl::PointCloud<PointType>()); // NOVO
        locIcpTargetMapDS.reset(new pcl::PointCloud<PointType>());
        locSnapScan.reset(new pcl::PointCloud<PointType>()); // NOVO

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        // A GPU e' OPT-IN (gpu:=true no roslaunch). Padrao CPU: no carro a GPU
        // fica ocupada com a IA, e mesmo sem CUDA nada aqui muda de resultado --
        // cudaAvailable=false simplesmente manda tudo pro kdtree/VoxelGrid.
        if (!useGpu)
        {
            cudaAvailable = false;
            ROS_INFO("\033[1;33m----> Busca de mapa na CPU (kdtree/VoxelGrid). "
                     "Passe gpu:=true no roslaunch pra usar CUDA.\033[0m");
        }
        else
        {
            cudaAvailable = cuda_map_search::init();
            if (cudaAvailable)
                ROS_INFO("\033[1;32m----> Busca de mapa na GPU (CUDA) habilitada.\033[0m");
            else
                ROS_WARN("gpu:=true pedido, mas a inicializacao do CUDA falhou "
                         "(sem toolkit no build, ou nenhuma GPU visivel) -- caindo pra CPU.");
        }

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP.setZero();
    }

    void loadMap()
    {
        // Load a previously saved map so it can be used only for localization.
        // The saved global feature clouds are used as a fixed map for scan-to-map matching.
        cout << "****************************************************" << endl;
        cout << "Localization mode: loading map from " << savePCDDirectory << endl;

        const std::string cornerFile = savePCDDirectory + "cloudCorner.pcd";
        const std::string surfFile   = savePCDDirectory + "cloudSurf.pcd";
        const std::string trajFile   = savePCDDirectory + "trajectory.pcd";
        const std::string transFile  = savePCDDirectory + "transformations.pcd";

        if (pcl::io::loadPCDFile<PointType>(cornerFile, *laserCloudCornerFromMapDS) == -1)
        {
            ROS_ERROR_STREAM("Localization mode: could not load corner map: " << cornerFile);
            ros::shutdown();
            return;
        }
        if (pcl::io::loadPCDFile<PointType>(surfFile, *laserCloudSurfFromMapDS) == -1)
        {
            ROS_ERROR_STREAM("Localization mode: could not load surf map: " << surfFile);
            ros::shutdown();
            return;
        }

        // Key poses are needed so the many "cloudKeyPoses3D empty" guards let the pipeline run.
        if (pcl::io::loadPCDFile<PointType>(trajFile, *cloudKeyPoses3D) == -1)
        {
            ROS_ERROR_STREAM("Localization mode: could not load trajectory: " << trajFile);
            ros::shutdown();
            return;
        }
        if (pcl::io::loadPCDFile<PointTypePose>(transFile, *cloudKeyPoses6D) == -1)
        {
            // Not strictly required for matching against the fixed global map; warn only.
            ROS_WARN_STREAM("Localization mode: could not load transformations (poses 6D): " << transFile);
        }

        // FIX: reconstroi o banco de Scan Context (em memoria) a partir dos .scd
        // salvos durante o mapeamento. Sem isso, scManager.polarcontexts_ fica
        // vazio em localizationMode (makeAndSaveScancontextAndKeys so e' chamado
        // no fluxo de mapeamento) e o primeiro detectLoopClosureID() do
        // loopClosureThread (auto-relocalize) da segfault (.back() de vetor
        // vazio em polarcontext_invkeys_mat_/polarcontexts_).
        {
            const std::string scdDir = savePCDDirectory + "SCDs/";
            int numLoadedSCD = 0;
            for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i)
            {
                const std::string scdFile = scdDir + padZeros((int)i) + ".scd";
                std::ifstream f(scdFile);
                if (!f.good())
                    continue;
                f.close();
                Eigen::MatrixXd sc = readSCD(scdFile);
                if (sc.size() == 0)
                    continue;
                scManager.loadScancontextAndKeys(sc);
                ++numLoadedSCD;
            }
            ROS_INFO_STREAM("Localization mode: " << numLoadedSCD << "/" << cloudKeyPoses6D->size()
                            << " descritores Scan Context carregados de " << scdDir
                            << " (necessario pro auto-relocalize).");
        }

        // FIX: carrega a ancora GPS<->mapa salva durante o mapeamento (ver
        // gpsHandler() e o fim de visualizeGlobalMapThread()). Sem esse
        // arquivo (mapa mapeado sem GPS, ou mapeado antes dessa mudanca),
        // gpsAnchorValid fica false e o GPS e' simplesmente ignorado pro
        // auto-relocalize -- cai pro Scan Context / clique manual.
        {
            const std::string anchorFile = savePCDDirectory + "gps_utm_anchor.txt";
            std::ifstream f(anchorFile);
            if (f.good() && (f >> gpsAnchorX >> gpsAnchorY >> gpsAnchorYaw))
            {
                gpsAnchorValid = true;
                ROS_INFO("Localization mode: ancora GPS<->mapa carregada de '%s' (x=%.3f y=%.3f yaw=%.3f rad).",
                         anchorFile.c_str(), gpsAnchorX, gpsAnchorY, gpsAnchorYaw);
            }
            else
            {
                ROS_WARN_STREAM("Localization mode: '" << anchorFile << "' nao encontrado/invalido -- "
                                "auto-relocalize por GPS desabilitado pra esse mapa (cai pro Scan Context/clique manual).");
            }
        }

        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        laserCloudSurfFromMapDSNum   = laserCloudSurfFromMapDS->size();

        // Build the map kdtrees once (the fixed map does not change during localization).
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        // NOVO: mapa denso pra ICP de re-localização. Se cloudGlobal.pcd não existir,
        // cai pro corner+surf mesmo (mais esparso, mas funciona).
        const std::string globalCloudFile = savePCDDirectory + "cloudGlobal.pcd";
        if (pcl::io::loadPCDFile<PointType>(globalCloudFile, *locIcpTargetMap) == -1)
        {
            ROS_WARN_STREAM("Localization mode: cloudGlobal.pcd nao encontrado (" << globalCloudFile
                            << "), usando corner+surf como alvo do ICP de encaixe.");
            *locIcpTargetMap += *laserCloudCornerFromMapDS;
            *locIcpTargetMap += *laserCloudSurfFromMapDS;
        }

        // Sobe o cloudGlobal.pcd puro, sem downsample defensivo -- esse voxel filter
        // só existia pra "proteger" o ICP contra um cloudGlobal gigante, mas custava
        // tempo de boot toda vez que o mapa era carregado, atrasando a subida do modo
        // de localização. Se o cloudGlobal.pcd for salvo grande demais, o certo é
        // resolver na origem (downsample no momento de salvar o mapa), não aqui.
        cout << "Localization mode: locIcpTargetMap carregado sem downsample ("
             << locIcpTargetMap->size() << " pontos)." << endl;

        // Publica o mapa AQUI, antes de qualquer coisa que dependa do robô.
        //
        // O waitForTransform logo abaixo BLOQUEIA o construtor -- e sob use_sim_time
        // aqueles "3 segundos" são tempo SIMULADO, ou seja, enquanto o playback não
        // publicar /clock esse wait não anda. As threads (inclusive a
        // visualizeGlobalMapThread) só sobem depois que o construtor retorna, então
        // o mapa ficava refém do playback pra aparecer. Publicando aqui, o mapa sobe
        // junto com o nó, independente de robô, TF, /clock ou playback -- e o thread
        // continua republicando com stamp fresco a cada 5 s.
        publishLocGlobalMap();

        // ==========================================================
        // Inicialização Automática com TF Dinâmico (Correção LiDAR invertido)
        // ==========================================================
        // 1. Cria a pose inicial baseada no YAML (representando o CARRO)
        tf::Pose initial_base_link_pose;
        initial_base_link_pose.setOrigin(tf::Vector3(initialPoseX, initialPoseY, initialPoseZ));
        tf::Quaternion q_init;
        q_init.setRPY(initialPoseRoll, initialPosePitch, initialPoseYaw);
        initial_base_link_pose.setRotation(q_init);

        // 2. Busca a transformação dinâmica base_link -> lidar_link
        //    (cacheada no membro baselink2Lidar para reuso na gravacao de poses)
        baselink2Lidar = tf::Transform::getIdentity();
        if (baselinkFrame != lidarFrame)
        {
            static tf::TransformListener tfListener;
            tf::StampedTransform transform;
            try {
                // Aguarda até 3 segundos para o ROS subir a árvore de TF no boot
                tfListener.waitForTransform(baselinkFrame, lidarFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(baselinkFrame, lidarFrame, ros::Time(0), transform);
                baselink2Lidar = transform;
                baselink2LidarCached = true;
                ROS_INFO("Localization auto-init: TF dinamico %s -> %s lido com sucesso.", baselinkFrame.c_str(), lidarFrame.c_str());
            } catch (tf::TransformException ex) {
                ROS_WARN("Localization auto-init: Falha ao ler TF %s -> %s. Usando identidade. (%s)", 
                         baselinkFrame.c_str(), lidarFrame.c_str(), ex.what());
            }
        }
        else
        {
            baselink2LidarCached = true;
        }

        // 3. Calcula a pose final do LiDAR aplicando a rotação do sensor
        tf::Pose initial_lidar_pose = initial_base_link_pose * baselink2Lidar;
        double r_auto, p_auto, y_auto;
        initial_lidar_pose.getBasis().getRPY(r_auto, p_auto, y_auto);

        // 4. Alimenta o otimizador com a pose real do LiDAR corrigida
        transformTobeMapped[0] = r_auto;
        transformTobeMapped[1] = p_auto;
        transformTobeMapped[2] = y_auto;
        transformTobeMapped[3] = initial_lidar_pose.getOrigin().x();
        transformTobeMapped[4] = initial_lidar_pose.getOrigin().y();
        transformTobeMapped[5] = initial_lidar_pose.getOrigin().z();
        // ==========================================================

        cout << "Localization mode: loaded " << laserCloudCornerFromMapDSNum << " corner and "
             << laserCloudSurfFromMapDSNum << " surf map points, "
             << cloudKeyPoses3D->size() << " key poses." << endl;
        cout << "Initial pose guess: x=" << initialPoseX << " y=" << initialPoseY << " z=" << initialPoseZ
             << " roll=" << initialPoseRoll << " pitch=" << initialPosePitch << " yaw=" << initialPoseYaw << endl;
        cout << "****************************************************" << endl;
    }

    void initialPoseHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
    {
        // NOVO: Lê a pose clicada (que representa o CARRO / base_link)
        // e transforma para a pose do LIDAR dinamicamente usando a TF do ROS.
        // Isso resolve o LiDAR invertido/dinâmico sem hardcode!
        if (!localizationMode)
            return;

        // FIX (sentido invertido): o RViz publica o "2D Pose Estimate" no FIXED FRAME
        // dele, que aqui e' "map_ground_link" (ver rviz_alberto.rviz) -- NAO em
        // odom/map. Antes essa pose era repassada crua, como se ja' estivesse em
        // odom. Mas map -> map_ground_link carrega yaw = 180 graus (o lidar da IARA
        // e' montado girado; map_ground_link e' derivado do inverso de T_robot_lidar,
        // ver publish_static_transforms no pointcloud_node) alem de um offset de
        // translacao. Resultado: apontar pra frente colocava o robo apontando pra
        // tras -- e, pior, o ICP recebia o scan girado meia-volta e nunca convergia
        // (fitness ~14-21 contra o limiar de 1.5), caindo sempre no chute bruto.
        //
        // Converte pelo header.frame_id da propria mensagem, entao funciona pra
        // qualquer Fixed Frame que o RViz estiver usando (map_ground_link, map, odom).
        geometry_msgs::Pose poseInOdom = poseMsg->pose.pose;
        const std::string srcFrame = poseMsg->header.frame_id;

        if (!srcFrame.empty() && srcFrame != odometryFrame)
        {
            static tf::TransformListener tfListenerInit;
            tf::StampedTransform T_odom_src;
            try {
                tfListenerInit.waitForTransform(odometryFrame, srcFrame, ros::Time(0), ros::Duration(1.0));
                tfListenerInit.lookupTransform(odometryFrame, srcFrame, ros::Time(0), T_odom_src);

                tf::Pose pIn;
                tf::poseMsgToTF(poseMsg->pose.pose, pIn);
                tf::poseTFToMsg(T_odom_src * pIn, poseInOdom);

                double srcYaw, outYaw, ignore1, ignore2, ignore3, ignore4;
                tf::Matrix3x3(pIn.getRotation()).getRPY(ignore1, ignore2, srcYaw);
                tf::Pose pOut; tf::poseMsgToTF(poseInOdom, pOut);
                tf::Matrix3x3(pOut.getRotation()).getRPY(ignore3, ignore4, outYaw);

                ROS_INFO("Localization mode: clique recebido em '%s' (yaw=%.1f graus), convertido pra '%s' "
                         "(yaw=%.1f graus).",
                         srcFrame.c_str(), srcYaw * 180.0 / M_PI, odometryFrame.c_str(), outYaw * 180.0 / M_PI);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("Localization mode: falha ao converter o clique de '%s' pra '%s' (%s). "
                          "Usando a pose crua -- o encaixe provavelmente vai sair errado/invertido.",
                          srcFrame.c_str(), odometryFrame.c_str(), ex.what());
            }
        }

        requestLocSnapFromBaseLinkPose(poseInOdom, "clique no RViz", /*manualRequest=*/true);
    }

    // Recebe uma pose do CARRO/base_link no frame do mapa (vinda do clique no
    // RViz ou de um fix de GPS ja ancorado -- ver gpsHandler) e dispara um
    // snap (ICP de re-localizacao em background, via locSnapWorker). Extraido
    // de initialPoseHandler pra ser reaproveitado pelas duas fontes.
    // manualRequest=true => veio do usuario (clique no RViz). Um pedido manual tem
    // PRIORIDADE e substitui um snap automatico (GPS/Scan Context) em andamento; um
    // pedido automatico continua sendo descartado se ja' houver snap rodando.
    void requestLocSnapFromBaseLinkPose(const geometry_msgs::Pose& baseLinkPoseInMap, const char* source,
                                        bool manualRequest = false)
    {
        // FIX: antes QUALQUER pedido era descartado se houvesse um snap pendente --
        // inclusive o clique do usuario no RViz. Com o auto-relocalize por GPS
        // religado, o proprio boot ja' dispara um snap, entao o usuario clicava e
        // era ignorado ("ja tem um snap em andamento"), sem forma de corrigir
        // manualmente um encaixe automatico ruim. Agora o clique vence: sobrescreve
        // o guess/scan e incrementa locSnapSeq, o que faz o locSnapWorker perceber
        // que o resultado do ICP em curso ficou obsoleto e reprocessar com o pedido
        // novo em vez de desarmar locSnapPending.
        if (locSnapPending.load() && !manualRequest)
        {
            ROS_WARN("Localization mode: ja tem um snap em andamento, ignorando pedido novo (%s).", source);
            return;
        }
        if (locSnapPending.load() && manualRequest)
        {
            ROS_INFO("Localization mode: pedido manual (%s) tem prioridade -- substituindo o snap em andamento.", source);
        }

        // 1. Obtém a pose recebida (frame do mapa -> base_link)
        tf::Pose clickedPose;
        tf::poseMsgToTF(baseLinkPoseInMap, clickedPose);

        // 2. Busca a transformação dinâmica base_link -> lidar_link
        tf::Transform baselink2Lidar = tf::Transform::getIdentity();
        if (baselinkFrame != lidarFrame)
        {
            static tf::TransformListener tfListener;
            tf::StampedTransform transform;
            try {
                // Tenta buscar a extrínseca dinâmica (carro -> lidar)
                tfListener.waitForTransform(baselinkFrame, lidarFrame, ros::Time(0), ros::Duration(1.0));
                tfListener.lookupTransform(baselinkFrame, lidarFrame, ros::Time(0), transform);
                baselink2Lidar = transform;
                ROS_INFO("Localization mode: TF dinamico %s -> %s lido com sucesso.", baselinkFrame.c_str(), lidarFrame.c_str());
            } catch (tf::TransformException ex) {
                ROS_WARN("Localization mode: Falha ao ler TF %s -> %s. Usando identidade. (%s)",
                         baselinkFrame.c_str(), lidarFrame.c_str(), ex.what());
            }
        }

        // 3. Calcula a pose final do LiDAR no mapa multiplicando as matrizes
        tf::Pose lidarPose = clickedPose * baselink2Lidar;

        double roll, pitch, yaw;
        lidarPose.getBasis().getRPY(roll, pitch, yaw);

        // 4. Chute bruto ajustado EXATAMENTE para onde o LiDAR está apontando
        float guess[6];
        guess[2] = yaw;
        guess[3] = lidarPose.getOrigin().x();
        guess[4] = lidarPose.getOrigin().y();

        // Acha a key pose mais próxima (só x,y) no mapa carregado, pra puxar
        // um z/roll/pitch que já são reais, em vez de reaproveitar valor velho
        float bestDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); ++i)
        {
            float dx = cloudKeyPoses3D->points[i].x - guess[3];
            float dy = cloudKeyPoses3D->points[i].y - guess[4];
            float d = dx * dx + dy * dy;
            if (d < bestDist) { bestDist = d; bestIdx = i; }
        }

        if (bestIdx >= 0)
        {
            guess[0] = cloudKeyPoses6D->points[bestIdx].roll;
            guess[1] = cloudKeyPoses6D->points[bestIdx].pitch;
            guess[5] = cloudKeyPoses6D->points[bestIdx].z;
        }
        else
        {
            // fallback se por algum motivo o mapa não tem poses carregadas
            guess[0] = transformTobeMapped[0];
            guess[1] = transformTobeMapped[1];
            guess[5] = transformTobeMapped[5];
        }

        // guarda o chute + uma cópia do scan atual, e devolve o controle pro
        // ros::spin() na hora — nada de ICP aqui dentro
        {
            std::lock_guard<std::mutex> lockSnap(mtxLocSnap);
            std::copy(std::begin(guess), std::end(guess), locSnapGuess);
            locSnapScan->clear();
            std::lock_guard<std::mutex> lockMain(mtx);
            *locSnapScan += *laserCloudRawDS;
            // Pose odometrica no instante em que o scan foi capturado. O locSnapWorker
            // usa isso pra descontar o quanto o robo andou durante o ICP antes de
            // aplicar o resultado (ver "FIX (defasagem)" la').
            locSnapPoseAtRequest = trans2Affine3f(transformTobeMapped);
        }
        // Incrementa ANTES de armar o pending: se um ICP ja' estiver rodando, ele vai
        // ver a sequencia mudada ao terminar e reprocessar com esse guess novo.
        locSnapSeq.fetch_add(1);
        locSnapPending = true;
        locAutoRelocArmed = false;
        ROS_INFO("Localization mode: pedido de snap recebido (%s), processando em background com TF dinamica...", source);
    }

    // Grava um PCD sem nunca deixar uma excecao escapar.
    //
    // FIX CRITICO: pcl::io::savePCDFileBinary lanca pcl::IOException quando a nuvem
    // esta' VAZIA ("Input point cloud has no data!"). Como isso acontecia dentro de
    // uma thread sem try/catch, virava std::terminate -> SIGABRT -> o no' morria. E
    // como o launch tem respawn="true" (module_loam.launch), o roslaunch subia o no'
    // de novo, cujo construtor em modo mapeamento roda "rm -r savePCDDirectory" --
    // APAGANDO o mapa que ja' estava pronto. Foi assim que o mapa se perdeu.
    // Uma nuvem vazia e' um scan ruim, nao motivo pra derrubar o mapeamento inteiro.
    template<typename PointT>
    static bool safeSavePCD(const std::string& path, const pcl::PointCloud<PointT>& cloud)
    {
        if (cloud.empty())
        {
            ROS_WARN_STREAM("Nuvem vazia, pulando gravacao de " << path
                            << " (nao e' erro fatal -- scan sem features).");
            return false;
        }
        try {
            pcl::io::savePCDFileBinary(path, cloud);
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Falha ao gravar " << path << ": " << e.what()
                             << " -- seguindo sem derrubar o no'.");
            return false;
        }
    }

    // Escreve os Scans/*.pcd enfileirados por saveKeyFramesAndFactor. Roda fora da
    // thread do ROS, entao o disco nunca segura o pipeline de odometria. Na saida,
    // drena o que restou pra nao perder keyframes do fim da sessao.
    void pcdWriterThread()
    {
        if (localizationMode || !savePCD)
            return;   // em localizacao nada e' gravado

        ros::WallRate rate(20);   // tempo de parede: nao depende do /clock do playback
        while (ros::ok())
        {
            PendingPcd item;
            bool has = false;
            {
                std::lock_guard<std::mutex> lock(mtxPcdQueue);
                if (!pcdWriteQueue.empty())
                {
                    item = pcdWriteQueue.front();
                    pcdWriteQueue.pop_front();
                    has = true;
                }
            }
            if (!has) { rate.sleep(); continue; }
            safeSavePCD(item.path, *item.cloud);
        }

        // Drena o resto no shutdown (best-effort).
        size_t drained = 0;
        while (true)
        {
            PendingPcd item;
            {
                std::lock_guard<std::mutex> lock(mtxPcdQueue);
                if (pcdWriteQueue.empty())
                    break;
                item = pcdWriteQueue.front();
                pcdWriteQueue.pop_front();
            }
            safeSavePCD(item.path, *item.cloud);
            ++drained;
        }
        if (drained)
            cout << "pcdWriterThread: " << drained << " Scans/*.pcd gravados no encerramento." << endl;
    }

    void locSnapWorker()
    {
        // NOVO: roda numa thread própria (ver main()). Faz o ICP pesado de
        // encaixe contra o locIcpTargetMap sem travar o resto do pipeline.
        // WallRate (tempo REAL), nao Rate: esta thread so' checa uma flag e roda o
        // ICP de encaixe -- nao tem nada a ver com o relogio do playback. Com
        // ros::Rate sob use_sim_time, pausar o playback congelava esta thread, entao
        // clicar no RViz com o playback pausado (justamente o momento em que da' pra
        // mirar com calma) nunca processava o snap.
        ros::WallRate rate(5); // só checa a flag, não fica em busy-loop
        while (ros::ok())
        {
            rate.sleep();
            if (!locSnapPending.load())
                continue;

            float guess[6];
            pcl::PointCloud<PointType>::Ptr scanForIcp(new pcl::PointCloud<PointType>());
            const uint32_t seqAtStart = locSnapSeq.load();
            Eigen::Affine3f poseAtRequest;
            {
                std::lock_guard<std::mutex> lockSnap(mtxLocSnap);
                std::copy(std::begin(locSnapGuess), std::end(locSnapGuess), guess);
                *scanForIcp += *locSnapScan;
                poseAtRequest = locSnapPoseAtRequest;
            }

            // FIX: se o pedido chegou antes de existir scan (tipico do auto-relocalize
            // por GPS, que dispara no primeiro fix -- as vezes centenas de ms antes do
            // primeiro scan do LiDAR), NAO aplica o chute bruto: apenas espera. Antes
            // esse caminho escrevia a pose crua do GPS em transformTobeMapped SEM ICP
            // NENHUM e ainda marcava locEverAnchored -- ou seja, o "pre-snap" que o
            // sistema fazia sozinho no boot, sem validacao contra o mapa. Agora o
            // pedido fica pendente e e' reprocessado assim que o primeiro scan chega,
            // ai sim com ICP de verdade.
            if (scanForIcp->points.empty())
            {
                std::lock_guard<std::mutex> lockSnap(mtxLocSnap);
                std::lock_guard<std::mutex> lockMain(mtx);
                if (!laserCloudRawDS->empty())
                {
                    *locSnapScan = *laserCloudRawDS;          // scan ja' disponivel: adota
                    locSnapPoseAtRequest = trans2Affine3f(transformTobeMapped);
                    ROS_INFO("Localization mode: scan ficou disponivel, retomando o snap pendente.");
                }
                else
                {
                    ROS_WARN_THROTTLE(2.0, "Localization mode: pedido de snap sem scan ainda -- "
                                           "aguardando o primeiro scan do LiDAR (pedido segue pendente).");
                }
                continue;   // mantem locSnapPending: reprocessa na proxima volta
            }

            if (locIcpTargetMap->points.empty())
            {
                ROS_WARN("Localization mode: sem mapa carregado, usando o chute bruto.");
                std::lock_guard<std::mutex> lock(mtx);
                transformTobeMapped[2] = guess[2];
                transformTobeMapped[3] = guess[3];
                transformTobeMapped[4] = guess[4];
                locResetRequested = true;
                lastIncreOdomPubFlag = false;
                // NOVO: carrega o timestamp do frame atual. O imuPreintegration usa
                // isso pra descartar mensagens de odometria atrasadas (publicadas
                // pela thread principal ENQUANTO o ICP do snap ainda rodava) que
                // cheguem depois do loc_reset com a pose antiga, de antes da correção.
                std_msgs::Header resetMsg;
                resetMsg.stamp = timeLaserInfoStamp;
                pubLocReset.publish(resetMsg);
                locEverAnchored = true;   // pose aplicada (chute bruto): GPS nao mexe mais
                ++locAnchorCount;
                if (locSnapSeq.load() == seqAtStart)
                    locSnapPending = false;
                continue;
            }

            // Leva o scan atual pra perto de onde o usuário clicou, antes do ICP
            PointTypePose guessPose = trans2PointTypePose(guess);
            Eigen::Affine3f guessAffine = pclPointToAffine3f(guessPose);
            pcl::PointCloud<PointType>::Ptr scanInMap(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*scanForIcp, *scanInMap, guessAffine.matrix());

            // FIX (velocidade): recorta o alvo do ICP a uma vizinhanca do chute em vez
            // de usar o locIcpTargetMap inteiro. Esse mapa tem ~18,7 MILHOES de pontos
            // (o downsample foi removido pra acelerar o boot), e so' montar a kd-tree
            // disso ja' custa segundos -- com ate' 100 iteracoes em cima, o snap
            // simplesmente nao terminava: o clique era aceito e o "ICP snap fitness"
            // nunca chegava a ser impresso. Como o chute (clique/GPS/SC) ja' coloca o
            // robo na regiao certa, pontos a mais de locSnapCropRadius nunca viram
            // correspondencia (setMaxCorrespondenceDistance e' 20 m) -- so' pesam.
            // Passo 1: baratear o mapa UMA VEZ (lazy, aqui na thread de background pra
            // nao pesar no boot) e cachear. Cortar sozinho nao bastou: com ~18,7 M
            // pontos, ate' um raio de 150 m ainda captura milhoes -- so' montar a
            // kd-tree disso ja' estoura o tempo, e o ICP nunca terminava. Voxelizar
            // e' o que muda a ordem de grandeza; o corte depois so' afina.
            // O locIcpTargetMap denso continua intacto pra quem precisar dele.
            if (locIcpTargetMapDS->empty())
            {
                const ros::WallTime dsT0 = ros::WallTime::now();
                pcl::VoxelGrid<PointType> dsFilter;
                dsFilter.setLeafSize(locSnapTargetLeafSize, locSnapTargetLeafSize, locSnapTargetLeafSize);
                dsFilter.setInputCloud(locIcpTargetMap);
                dsFilter.filter(*locIcpTargetMapDS);
                ROS_INFO("Localization mode: alvo do ICP reduzido de %zu para %zu pontos "
                         "(voxel %.2f m, %.2f s) -- feito uma vez so'.",
                         locIcpTargetMap->points.size(), locIcpTargetMapDS->points.size(),
                         locSnapTargetLeafSize, (ros::WallTime::now() - dsT0).toSec());
            }

            // Passo 2: recortar a vizinhanca do chute do mapa ja' barateado.
            pcl::PointCloud<PointType>::Ptr icpTarget(new pcl::PointCloud<PointType>());
            {
                const float cx = guess[3], cy = guess[4];
                const float r2 = locSnapCropRadius * locSnapCropRadius;
                icpTarget->points.reserve(locIcpTargetMapDS->points.size() / 8);
                for (const auto &p : locIcpTargetMapDS->points)
                {
                    const float dx = p.x - cx, dy = p.y - cy;
                    if (dx * dx + dy * dy <= r2)
                        icpTarget->points.push_back(p);
                }
                icpTarget->width = icpTarget->points.size();
                icpTarget->height = 1;
                icpTarget->is_dense = false;
            }

            if (icpTarget->points.size() < 1000)
            {
                ROS_WARN("Localization mode: so' %zu pontos do mapa num raio de %.0f m do chute "
                         "(x=%.1f y=%.1f) -- chute provavelmente fora do mapa. Usando o mapa inteiro.",
                         icpTarget->points.size(), locSnapCropRadius, guess[3], guess[4]);
                *icpTarget = *locIcpTargetMapDS;
            }

            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(20.0); // folga generosa pro clique impreciso
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);
            icp.setInputSource(scanInMap);
            icp.setInputTarget(icpTarget);

            pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
            const ros::WallTime icpT0 = ros::WallTime::now();
            icp.align(*aligned); // <- agora roda fora da thread do ROS, não trava mais o mapping
            const double icpSecs = (ros::WallTime::now() - icpT0).toSec();

            ROS_INFO("Localization mode: ICP snap fitness = %f (%zu pts alvo, %.2f s de parede)",
                     icp.getFitnessScore(), icpTarget->points.size(), icpSecs);

            if (!icp.hasConverged() || icp.getFitnessScore() > locSnapFitnessScore)
            {
                ROS_WARN_STREAM("Localization mode: ICP nao convergiu bem (score " << icp.getFitnessScore()
                                << " > " << locSnapFitnessScore << "). Aplicando so o chute bruto, sem refino do ICP.");

                std::lock_guard<std::mutex> lock(mtx);
                transformTobeMapped[2] = guess[2];
                transformTobeMapped[3] = guess[3];
                transformTobeMapped[4] = guess[4];
                transformTobeMapped[0] = guess[0];
                transformTobeMapped[1] = guess[1];
                transformTobeMapped[5] = guess[5];
                locResetRequested = true;
                lastIncreOdomPubFlag = false;

                std_msgs::Header resetMsg;
                resetMsg.stamp = timeLaserInfoStamp;
                pubLocReset.publish(resetMsg);

                locEverAnchored = true;   // pose aplicada (fallback do ICP): GPS nao mexe mais
                ++locAnchorCount;
                if (locSnapSeq.load() == seqAtStart)
                    locSnapPending = false;
                continue;
            }

            // Corrige o chute com a transformação encontrada pelo ICP
            Eigen::Affine3f refinedAffine = Eigen::Affine3f(icp.getFinalTransformation()) * guessAffine;
            float refined[6];

            {
                std::lock_guard<std::mutex> lock(mtx);

                // FIX (defasagem): compensa o quanto o robo ANDOU enquanto o ICP rodava.
                //
                // refinedAffine e' a pose correta do robo no instante em que o scan foi
                // capturado (o clique). Escrever isso direto em transformTobeMapped
                // teleporta o robo pra uma pose ja' VELHA -- e nesse trecho ele anda a
                // ~6 m/s, entao alguns segundos de ICP viram dezenas de metros de erro.
                // Era esse o efeito de "esta' rodando numa frequencia mais baixa que o
                // robo": o encaixe sempre chegava atrasado.
                //
                // Correcao: em vez de pose absoluta, aplica o DELTA. O quanto a odometria
                // andou desde o pedido e' T_drift = poseAtRequest^-1 * poseAgora, e a pose
                // certa agora e' refinedAffine * T_drift. A odometria e' confiavel nesse
                // intervalo curto -- o que estava errado era a ancoragem no mapa, que e'
                // justamente o que o ICP corrigiu.
                const Eigen::Affine3f poseNow = trans2Affine3f(transformTobeMapped);
                const Eigen::Affine3f driftDuringIcp = poseAtRequest.inverse() * poseNow;
                refinedAffine = refinedAffine * driftDuringIcp;

                const Eigen::Vector3f driftT = driftDuringIcp.translation();
                if (driftT.norm() > 0.5f)
                    ROS_INFO("Localization mode: robo andou %.2f m durante o ICP -- compensado no encaixe.",
                             driftT.norm());

                pcl::getTranslationAndEulerAngles(refinedAffine, refined[3], refined[4], refined[5],
                                                                 refined[0], refined[1], refined[2]);
                for (int i = 0; i < 6; ++i)
                    transformTobeMapped[i] = refined[i];
                locResetRequested = true;
                lastIncreOdomPubFlag = false;              // reancora o odom incremental na pose nova
                

                timeLastLocReset = timeLaserInfoCur;       // NOVO: Marca o instante do teleporte!

                // NOVO: timestamp do frame atual, pro imuPreintegration descartar
                std_msgs::Header resetMsg;
                resetMsg.stamp = timeLaserInfoStamp;
                pubLocReset.publish(resetMsg);    // avisa o IMU preintegration pra resetar o estado dele
            }

            locEverAnchored = true;   // encaixe por ICP concluido: GPS nao mexe mais
            ++locAnchorCount;
            ROS_INFO_STREAM("Localization mode: pose encaixada em x=" << transformTobeMapped[3]
                            << " y=" << transformTobeMapped[4] << " yaw=" << transformTobeMapped[2]);

            // FIX: WallDuration (tempo REAL), nao Duration (tempo SIMULADO).
            // Sob use_sim_time, ros::Duration(1.0).sleep() so' termina quando o
            // /clock avancar 1 s -- com o playback pausado, terminado ou lento isso
            // nunca acontece, e locSnapPending fica travado em true PRA SEMPRE.
            // Efeito: todo clique de "2D Pose Estimate" no RViz passava a ser
            // rejeitado com "ja tem um snap em andamento", sem jeito de recuperar.
            // Esse sleep so' existe pra dar um respiro entre snaps, entao tempo de
            // parede e' o certo aqui.
            ros::WallDuration(1.0).sleep();

            // So' desarma se ninguem pediu um snap NOVO enquanto o ICP rodava (ver
            // locSnapSeq em requestLocSnapFromBaseLinkPose). Se pediram, mantem
            // pendente pra reprocessar ja' com o guess novo na proxima volta.
            if (locSnapSeq.load() == seqAtStart)
                locSnapPending = false;
            else
                ROS_INFO("Localization mode: chegou pedido de snap novo durante o ICP -- reprocessando com o guess mais recente.");
        }
    }

    void writeVertex(const int _node_idx, const gtsam::Pose3& _initPose)
    {
        gtsam::Point3 t = _initPose.translation();
        gtsam::Rot3 R = _initPose.rotation();

        std::string curVertexInfo {
            "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
            + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
            + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
            + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

        // pgVertexSaveStream << curVertexInfo << std::endl;
        vertices_str.emplace_back(curVertexInfo);
    }
    
    void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose)
    {
        gtsam::Point3 t = _relPose.translation();
        gtsam::Rot3 R = _relPose.rotation();

        std::string curEdgeInfo {
            "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
            + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
            + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
            + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

        // pgEdgeSaveStream << curEdgeInfo << std::endl;
        edges_str.emplace_back(curEdgeInfo);
    }

    // void writeEdgeStr(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, const gtsam::SharedNoiseModel _noise)
    // {
    //     gtsam::Point3 t = _relPose.translation();
    //     gtsam::Rot3 R = _relPose.rotation();

    //     std::string curEdgeSaveStream;
    //     curEdgeSaveStream << "EDGE_SE3:QUAT " << _node_idx_pair.first << " " << _node_idx_pair.second << " "
    //         << t.x() << " "  << t.y() << " " << t.z()  << " " 
    //         << R.toQuaternion().x() << " " << R.toQuaternion().y() << " " << R.toQuaternion().z()  << " " << R.toQuaternion().w() << std::endl;

    //     edges_str.emplace_back(curEdgeSaveStream);
    // }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
        pcl::fromROSMsg(msgIn->cloud_deskewed,  *laserCloudRaw); // giseop
        laserCloudRawTime = cloudInfo.header.stamp.toSec(); // giseop save node time

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            if (localizationMode)
            {
                // Localization only: match the current scan against the fixed loaded map.
                // The map is not modified, so we skip extracting/adding key frames, the
                // pose graph optimization and the loop-closure corrections.
                updateInitialGuessLocalization();

                incrementalOdometryAffineGuess = trans2Affine3f(transformTobeMapped);
                incrementalOdometryGuessFromOdom = cloudInfo.odomAvailable;

                downsampleCurrentScan();

                scan2MapOptimization();

                publishOdometry();

                recordRawPoseForOrigin();
                writeLocalizationPose();

                publishFrames();
            }
            else
            {
                updateInitialGuess();

                incrementalOdometryAffineGuess = trans2Affine3f(transformTobeMapped);
                incrementalOdometryGuessFromOdom = cloudInfo.odomAvailable;

                extractSurroundingKeyFrames();

                downsampleCurrentScan();

                scan2MapOptimization();

                saveKeyFramesAndFactor();

                correctPoses();

                publishOdometry();

                // Em mapeamento a pose SLAM ja' e' auto-consistente desde o
                // primeiro keyframe, entao a origem do mundo tambem pode ser
                // estimada aqui (util pra calcular uma vez e reusar depois).
                recordRawPoseForOrigin();

                publishFrames();
            }
        }
    }

    // FIX: a ancora GPS<->mapa agora é responsabilidade do SC-LIO-SAM (antes
    // era do gps_localization_node, que precisava correlacionar com a
    // odometria local do LIO-SAM -- em localizationMode essa odometria
    // começa ERRADA (semente do yaml) até o robô encaixar no mapa, um
    // problema de ovo-e-galinha que corrompia a âncora). gps_localization_node
    // agora só funde antena+heading+latência e publica RAW, sem tentar
    // ancorar em frame nenhum (ver gps_localization_node.cpp). Essa classe:
    //   - MAPEAMENTO: âncora UMA VEZ contra a própria pose SLAM (sempre
    //     confiável aqui, sendo construída do zero) assim que tiver GPS bom
    //     o suficiente; grava em savePCDDirectory/gps_utm_anchor.txt junto
    //     com o resto do mapa (ver fim de visualizeGlobalMapThread()).
    //   - LOCALIZAÇÃO: lê a âncora do arquivo salvo (ver loadMap()) e usa
    //     pra converter cada fix RAW pro frame do mapa antes de disparar o
    //     auto-relocalize.
    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        // RELIGADO: o auto-relocalize por GPS em localizationMode tinha sido
        // desligado aqui (early return) so' pra isolar o problema do "mapa local
        // girando". Agora esta' de volta: assim que o primeiro fix de GPS chega, a
        // ancora gps_utm_anchor.txt (carregada no loadMap()) converte esse fix pro
        // frame do mapa e ja' dispara o encaixe -- em vez de esperar o Scan Context
        // ou o clique manual no RViz. Se o mapa nao tiver ancora salva,
        // gpsAnchorValid fica false e isso vira no-op (cai pro SC/clique).
        //
        // LIMITE: o GPS so' age ENQUANTO NINGUEM ENCAIXOU O ROBO. Assim que um snap
        // qualquer (GPS, Scan Context ou clique no RViz) aplica uma pose,
        // locEverAnchored trava esta funcao pro resto da sessao. Sem isso o GPS
        // continuava concorrendo com o encaixe manual: o usuario clicava pra
        // corrigir e o GPS puxava de volta pra ancora dele, que pode estar
        // imprecisa. Depois de encaixado, quem manda e' o scan-to-map.
        // Guarda o timestamp ANTES de qualquer early-return: o stamp deste topico
        // ja' e' o timestamp CARMEN do gps_xyz (o bridge converte direto, sem
        // deslocamento -- ipc_bridge_node.cpp:188), e ele alimenta a coluna gps_ts
        // do poses_opt.dat. Sem estar aqui em cima, o gate do locEverAnchored
        // abaixo mataria a coleta assim que o robo encaixasse.
        if (posesOutEnabled)
        {
            std::lock_guard<std::mutex> lock(mtxCarmenStamps);
            gpsCarmenStamps.push_back(gpsMsg->header.stamp.toSec());
            while (gpsCarmenStamps.size() > kGpsStampMax)
                gpsCarmenStamps.pop_front();
        }

        // Origem do mundo por GPS -- tambem antes do gate abaixo, e vale nos dois
        // modos (em mapeamento este handler nao tem o early-return, mas manter
        // aqui em cima deixa a ordem obvia).
        updateWorldOriginFromGps(gpsMsg);

        if (localizationMode && locEverAnchored.load())
            return;

        tf::Quaternion qRaw;
        tf::quaternionMsgToTF(gpsMsg->pose.pose.orientation, qRaw);
        double rawRoll, rawPitch, rawYaw;
        tf::Matrix3x3(qRaw).getRPY(rawRoll, rawPitch, rawYaw);
        const double rawX = gpsMsg->pose.pose.position.x;
        const double rawY = gpsMsg->pose.pose.position.y;

        // ===== FRAME DO FIX x FRAME DA POSE SLAM =====
        //
        // O fix vem do gps_localization_node ja' com o braco de alavanca da antena
        // aplicado: e' a pose do BASE_LINK (centro do eixo traseiro -- o mesmo ponto
        // que o modelo Ackermann usa como origem). Ja' transformTobeMapped e' a pose
        // do LIDAR. Misturar os dois insere um erro do tamanho do braco base_link->
        // lidar que GIRA JUNTO com o veiculo -- ou seja, um vies sistematico que muda
        // de direcao conforme o rumo, exatamente a cara de "deriva na diagonal".
        //
        // Na IARA isso passava batido: sensor_board_1_x = 0.695 m, menor que o ruido
        // do GPS. No Atego o board esta' a 3.80 m do eixo traseiro, entao o erro e' de
        // metros e nao da' pra ignorar.
        //
        // Correcao: TODA a conta de ancoragem daqui pra baixo roda em base_link,
        // usando currentBaseLinkPose() no lugar de transformTobeMapped[3]/[4]. A volta
        // pro frame do lidar acontece so' na hora de criar o GPSFactor
        // (addGPSFactor()), que e' quem restringe a pose do keyframe -- e essa e' do
        // lidar.
        if (!ensureBaselink2Lidar())
        {
            ROS_WARN_THROTTLE(2.0,
                "GPS: sem a extrinseca %s->%s ainda -- nao da' pra levar o fix (que e' do "
                "base_link) pro frame do lidar. Fix ignorado neste ciclo.",
                baselinkFrame.c_str(), lidarFrame.c_str());
            return;
        }

        if (!localizationMode && !gpsAnchorValid)
        {
            // Só tenta ancorar depois que a pose SLAM já existe (primeiro
            // keyframe feito) e o fix de GPS não é ruim demais.
            if (cloudKeyPoses3D->points.empty())
            {
                ROS_WARN_THROTTLE(2.0, "Mapping: GPS chegou mas ainda sem nenhum keyframe SLAM -- aguardando pro' ancorar.");
                return;
            }
            float noise_x = gpsMsg->pose.covariance[0];
            float noise_y = gpsMsg->pose.covariance[7];
            if (!gpsAnchorIgnoreCov && (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold))
            {
                // FIX: antes rejeitava calado -- se o fix nunca tiver
                // covariancia boa o suficiente (ex: sem RTK/DGPS), a ancora
                // NUNCA se calculava e nao saia nenhum aviso explicando o
                // porque. Agora avisa qual covariancia chegou vs o limiar.
                ROS_WARN_THROTTLE(2.0,
                    "Mapping: GPS com covariancia ruim demais pra ancorar (noise_x=%.3f noise_y=%.3f > "
                    "gpsCovThreshold=%.3f) -- sem fix melhor (RTK/DGPS), gps_utm_anchor.txt nunca vai ser gravado. "
                    "(lio_sam/gpsAnchorIgnoreCov: true ignora esse limiar so' pra ancora -- so' pra debug.)",
                    noise_x, noise_y, gpsCovThreshold);
                return;
            }
            if (gpsAnchorIgnoreCov && (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold))
            {
                ROS_WARN_THROTTLE(2.0,
                    "Mapping: gpsAnchorIgnoreCov=true -- ancorando com GPS de baixa qualidade "
                    "(noise_x=%.3f noise_y=%.3f, limiar seria %.3f). SO' PRA DEBUG -- a ancora sai "
                    "com a precisao (ruim) desse fix.", noise_x, noise_y, gpsCovThreshold);
            }
            if (std::abs(rawX) < 1e-6 && std::abs(rawY) < 1e-6)
            {
                ROS_WARN_THROTTLE(2.0, "Mapping: GPS com posicao (0,0) -- ainda nao inicializou de verdade, aguardando.");
                return; // GPS ainda não inicializou (0,0)
            }

            // T tal que: pose_mapa = T ⊕ pose_gps_raw. Aproximação já usada
            // no resto do arquivo (addGPSFactor original): trata a pose GPS
            // (carro) como equivalente a pose_mapa (lidar) sem corrigir o
            // braço de alavanca lidar<->base_link, que é pequeno perto do
            // ruído do GPS.
            // ===== RUMO DA ANCORA PELA DIRECAO DE PERCURSO =====
            //
            // O calculo antigo era instantaneo:
            //
            //     gpsAnchorYaw = transformTobeMapped[2] - rawYaw;
            //
            // e dependia do heading que o GPS reporta (rawYaw). O problema: a ancora e'
            // calculada no PRIMEIRO fix bom, que chega com o caminhao ainda PARADO --
            // e parado o GPS nao tem course-over-ground, entao manda theta = 0.0000
            // exato. Com transformTobeMapped[2] tambem 0 no arranque, saia
            // gpsAnchorYaw = 0: a ancora afirmava que o frame do mapa e o do GPS tem o
            // MESMO rumo.
            //
            // No log do Ype isso era falso por 171.5 graus. Na mesma janela de tempo o
            // GPS andou 64.11 m na direcao -119.1 graus e o SLAM andou 63.92 m na
            // direcao +52.4 graus (distancias praticamente iguais -- a odometria estava
            // otima; so' o RUMO relativo estava errado). Cada GPSFactor entrava puxando
            // o grafo pra um frame girado de ~180 graus, e quando o peso acumulado dos
            // fixes venceu o prior, o grafo rasgava: o mapa girava e roll/pitch iam
            // junto (de -0.02 pra +0.27 rad num scan).
            //
            // A correcao: nao usar o heading do GPS pra nada. Guarda o primeiro par
            // (fix bruto, pose SLAM) como semente e espera os DOIS frames acumularem
            // gpsAnchorMinBaseline metros de deslocamento. Ai' o rumo da ancora e' a
            // diferenca entre as duas DIRECOES DE PERCURSO, que e' observavel, robusta,
            // e nao depende do GPS reportar heading nenhum.
            // Pose do carro (base_link) no mapa -- MESMO ponto fisico que o fix
            // descreve. Ver o comentario de frame no topo do handler.
            const tf::Vector3 mapBase = currentBaseLinkPose().getOrigin();

            if (!gpsAnchorSeedValid)
            {
                gpsAnchorSeedValid = true;
                gpsAnchorSeedRawX = rawX;
                gpsAnchorSeedRawY = rawY;
                gpsAnchorSeedMapX = mapBase.x();
                gpsAnchorSeedMapY = mapBase.y();
                ROS_INFO("Mapping: semente da ancora GPS<->mapa guardada (gps=%.3f,%.3f mapa=%.3f,%.3f). "
                         "Aguardando %.1f m de percurso pra tirar o rumo do MOVIMENTO -- o heading "
                         "instantaneo do GPS nao serve, o caminhao esta' parado e ele manda theta=0.",
                         gpsAnchorSeedRawX, gpsAnchorSeedRawY, gpsAnchorSeedMapX, gpsAnchorSeedMapY,
                         gpsAnchorMinBaseline);
                return;
            }

            const double gdx = rawX - gpsAnchorSeedRawX;
            const double gdy = rawY - gpsAnchorSeedRawY;
            const double mdx = mapBase.x() - gpsAnchorSeedMapX;
            const double mdy = mapBase.y() - gpsAnchorSeedMapY;
            const double gBase = std::sqrt(gdx * gdx + gdy * gdy);
            const double mBase = std::sqrt(mdx * mdx + mdy * mdy);

            if (gBase < gpsAnchorMinBaseline || mBase < gpsAnchorMinBaseline)
            {
                ROS_INFO_THROTTLE(5.0,
                    "Mapping: acumulando percurso pra ancorar o GPS -- gps=%.1f m mapa=%.1f m "
                    "(precisa de %.1f m nos dois).", gBase, mBase, gpsAnchorMinBaseline);
                return;
            }

            // Se os dois frames discordam MUITO na distancia percorrida, um dos dois
            // esta' ruim (fix pulando, ou SLAM ja' derivado) e o rumo tirado dai' seria
            // lixo. Melhor nao ancorar do que ancorar torto.
            if (std::fabs(gBase - mBase) > 0.25 * std::max(gBase, mBase))
            {
                ROS_WARN_THROTTLE(5.0,
                    "Mapping: GPS andou %.1f m e o SLAM %.1f m no mesmo trecho -- discordancia "
                    "acima de 25%%, nao da' pra tirar rumo confiavel. Ancora adiada.", gBase, mBase);
                return;
            }

            gpsAnchorYaw = std::atan2(mdy, mdx) - std::atan2(gdy, gdx);
            while (gpsAnchorYaw >  M_PI) gpsAnchorYaw -= 2.0 * M_PI;
            while (gpsAnchorYaw <= -M_PI) gpsAnchorYaw += 2.0 * M_PI;

            const double c = cos(gpsAnchorYaw), s = sin(gpsAnchorYaw);
            gpsAnchorX = mapBase.x() - (c * rawX - s * rawY);
            gpsAnchorY = mapBase.y() - (s * rawX + c * rawY);
            gpsAnchorValid = true;
            ROS_INFO("\033[1;32mMapping: ancora GPS<->mapa calculada (x=%.3f y=%.3f yaw=%.3f rad = %.1f deg) "
                     "a partir de %.1f m de percurso (gps %.1f deg vs mapa %.1f deg). Sera salva em "
                     "'%sgps_utm_anchor.txt' quando o mapa for salvo.\033[0m",
                     gpsAnchorX, gpsAnchorY, gpsAnchorYaw, gpsAnchorYaw * 180.0 / M_PI,
                     mBase, std::atan2(gdy, gdx) * 180.0 / M_PI, std::atan2(mdy, mdx) * 180.0 / M_PI,
                     savePCDDirectory.c_str());
        }

        if (!gpsAnchorValid)
            return; // localizationMode sem gps_utm_anchor.txt carregado, ou mapeamento ainda sem 1a ancora

        // Converte o fix RAW pro frame do mapa usando a ancora.
        const double c = cos(gpsAnchorYaw), s = sin(gpsAnchorYaw);
        nav_msgs::Odometry mappedGps = *gpsMsg;
        mappedGps.pose.pose.position.x = gpsAnchorX + c * rawX - s * rawY;
        mappedGps.pose.pose.position.y = gpsAnchorY + s * rawX + c * rawY;

        if (!localizationMode)
        {
            // Frame do mapa, mas ainda descrevendo o BASE_LINK. Quem leva pro frame
            // do lidar (que e' o que o keyframe do grafo representa) e' o
            // addGPSFactor(), aplicando o braco base_link->lidar com a atitude do
            // proprio keyframe.
            gpsQueue.push_back(mappedGps);
            return;
        }

        // ==================== LOCALIZATION MODE ====================
        if (!locAutoRelocArmed.load() || locSnapPending.load())
            return;

        // Sanity check contra a extensao do mapa carregado ANTES de disparar
        // o snap -- mesmo com a ancora, um GPS com fix ruim/multipath pode
        // apontar pra longe demais.
        if (!cloudKeyPoses3D->points.empty())
        {
            float minX = std::numeric_limits<float>::max(), maxX = -std::numeric_limits<float>::max();
            float minY = std::numeric_limits<float>::max(), maxY = -std::numeric_limits<float>::max();
            for (const auto &p : cloudKeyPoses3D->points)
            {
                minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
                minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
            }
            const float margin = 100.0f; // m
            const double gx = mappedGps.pose.pose.position.x;
            const double gy = mappedGps.pose.pose.position.y;
            if (gx < minX - margin || gx > maxX + margin || gy < minY - margin || gy > maxY + margin)
            {
                ROS_WARN_THROTTLE(2.0,
                    "Localization mode: fix de GPS (x=%.3f y=%.3f, ja convertido pro frame do mapa) fora "
                    "da extensao do mapa carregado (%zu key poses, x:[%.1f,%.1f] y:[%.1f,%.1f] +%.0fm de "
                    "folga) -- ignorando auto-relocalize por GPS. Cai pro Scan Context/clique manual.",
                    gx, gy, cloudKeyPoses3D->points.size(), minX, maxX, minY, maxY, margin);
                return;
            }
        }

        geometry_msgs::Pose baseLinkPoseGuess;
        baseLinkPoseGuess.position = mappedGps.pose.pose.position;
        baseLinkPoseGuess.position.z = transformTobeMapped[5]; // GPS z e' ruim demais pro chute inicial
        tf::Quaternion qOut;
        qOut.setRPY(0.0, 0.0, gpsAnchorYaw + rawYaw);
        tf::quaternionTFToMsg(qOut, baseLinkPoseGuess.orientation);

        ROS_INFO("Localization mode: fix de GPS recebido, tentando auto-relocalize.");
        requestLocSnapFromBaseLinkPose(baseLinkPoseGuess, "GPS");
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }


    void visualizeGlobalMapThread()
    {
        // NOVO: em modo localização o mapa já está carregado em memória desde o
        // construtor (loadMap()), então a primeira tentativa de publicação roda
        // assim que a thread sobe -- não espera o primeiro rate.sleep() (5s) nem
        // depende de qualquer mensagem de odometria/lidar ter chegado. O mapa
        // "sobe junto com o sistema"; só falta ter subscriber (RViz) conectado.
        publishGlobalMap();

        // WallRate pelo mesmo motivo: republicar o mapa fixo nao pode depender do
        // /clock do playback (ver publishLocGlobalMap).
        ros::WallRate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }

        // In localization mode the map must not be overwritten on shutdown.
        if (savePCD == false || localizationMode)
            return;

        // save pose graph (runs when programe is closing)
        cout << "****************************************************" << endl; 
        cout << "Saving the posegraph ..." << endl; // giseop

        for(auto& _line: vertices_str)
            pgSaveStream << _line << std::endl;
        for(auto& _line: edges_str)
            pgSaveStream << _line << std::endl;

        pgSaveStream.close();
        // pgVertexSaveStream.close();
        // pgEdgeSaveStream.close();

        const std::string kitti_format_pg_filename {savePCDDirectory + "optimized_poses.txt"};
        saveOptimizedVerticesKITTIformat(isamCurrentEstimate, kitti_format_pg_filename);

        // save map 
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        if (cloudKeyPoses3D->empty())
        {
            ROS_WARN("Nenhum keyframe -- nada a salvar. (Se o no' acabou de reiniciar por respawn, "
                     "o mapa da sessao anterior pode ter sido apagado pelo construtor.)");
            return;
        }

        // save key frame transformations
        safeSavePCD(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        safeSavePCD(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
        // Origem do mundo estimada pelo GPS, junto com o resto do mapa -- assim a
        // localizacao depois reaproveita o que o mapeamento calculou.
        if (posesOriginReady.load())
            saveWorldOrigin();
        // extract global point cloud map
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        // OTIMIZACAO (sem perder um unico ponto -- resultado identico ao anterior):
        //
        //  1) Pre-aloca o tamanho exato. O "+=" antigo crescia a nuvem incrementalmente,
        //     e cada realocacao copia TUDO que ja' estava la'. Com ~90 keyframes de
        //     ~160k pontos isso significava recopiar gigabytes ao longo do loop.
        //  2) Transforma os keyframes EM PARALELO. Cada um escreve numa faixa disjunta
        //     do buffer (calculada pelos offsets abaixo), entao nao ha' concorrencia --
        //     e some a alocacao temporaria que transformPointCloud fazia por keyframe.
        //  3) Log a cada 20 keyframes em vez de todos. O "\r" + std::flush forcava um
        //     write() no terminal a CADA keyframe, o que serializa o loop inteiro.
        const int numKF = (int)cloudKeyPoses3D->size();
        std::vector<size_t> cornerOff(numKF + 1, 0), surfOff(numKF + 1, 0);
        for (int i = 0; i < numKF; ++i)
        {
            cornerOff[i + 1] = cornerOff[i] + cornerCloudKeyFrames[i]->size();
            surfOff[i + 1]   = surfOff[i]   + surfCloudKeyFrames[i]->size();
        }
        globalCornerCloud->resize(cornerOff[numKF]);
        globalSurfCloud->resize(surfOff[numKF]);
        cout << "Transformando " << numKF << " keyframes ("
             << cornerOff[numKF] << " corner + " << surfOff[numKF] << " surf pts)..." << endl;

        #pragma omp parallel for num_threads(numberOfCores) schedule(dynamic)
        for (int i = 0; i < numKF; ++i)
        {
            const Eigen::Affine3f t = pclPointToAffine3f(cloudKeyPoses6D->points[i]);

            const auto &cin = cornerCloudKeyFrames[i]->points;
            for (size_t k = 0; k < cin.size(); ++k)
            {
                PointType &o = globalCornerCloud->points[cornerOff[i] + k];
                o.x = t(0,0)*cin[k].x + t(0,1)*cin[k].y + t(0,2)*cin[k].z + t(0,3);
                o.y = t(1,0)*cin[k].x + t(1,1)*cin[k].y + t(1,2)*cin[k].z + t(1,3);
                o.z = t(2,0)*cin[k].x + t(2,1)*cin[k].y + t(2,2)*cin[k].z + t(2,3);
                o.intensity = cin[k].intensity;
            }

            const auto &sin_ = surfCloudKeyFrames[i]->points;
            for (size_t k = 0; k < sin_.size(); ++k)
            {
                PointType &o = globalSurfCloud->points[surfOff[i] + k];
                o.x = t(0,0)*sin_[k].x + t(0,1)*sin_[k].y + t(0,2)*sin_[k].z + t(0,3);
                o.y = t(1,0)*sin_[k].x + t(1,1)*sin_[k].y + t(1,2)*sin_[k].z + t(1,3);
                o.z = t(2,0)*sin_[k].x + t(2,1)*sin_[k].y + t(2,2)*sin_[k].z + t(2,3);
                o.intensity = sin_[k].intensity;
            }

            if ((i % 20) == 0)
                cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << numKF << " ...";
        }
        cout << "\rProcessing feature cloud " << numKF << " of " << numKF << " ... ok" << endl;
        // down-sample and save corner cloud
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        downSizeFilterCorner.filter(*globalCornerCloudDS);
        safeSavePCD(savePCDDirectory + "cloudCorner.pcd", *globalCornerCloudDS);
        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        safeSavePCD(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloudDS);

        // OTIMIZACAO: cloudGlobal.pcd agora sai das nuvens JA' DOWNSAMPLEADAS.
        // Antes somava globalCornerCloud + globalSurfCloud CRUS -- por isso o arquivo
        // saia com ~18,7 MILHOES de pontos (~300 MB): custava minutos pra escrever no
        // Ctrl+C, e depois obrigava o modo localizacao a voxelizar tudo em memoria
        // antes de cada ICP de encaixe (ver locIcpTargetMapDS no locSnapWorker).
        // Downsamplear aqui, na origem, resolve os dois de uma vez -- e o corner/surf
        // ja' sao salvos com esse mesmo downsample logo acima, entao nao se perde nada
        // que os outros arquivos ja' nao tivessem.
        *globalMapCloud += *globalCornerCloudDS;
        *globalMapCloud += *globalSurfCloudDS;
        safeSavePCD(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        cout << "cloudGlobal.pcd: " << globalMapCloud->size() << " pontos (downsampleado na origem)." << endl;

        // FIX: salva a ancora GPS<->mapa junto com o resto do mapa (ver
        // gpsHandler(), onde ela e' calculada uma vez contra a pose SLAM
        // assim que o primeiro GPS bom chega). Sem GPS durante o mapeamento,
        // gpsAnchorValid fica false e nao escreve nada -- localizationMode
        // so' vai avisar que o arquivo nao existe e seguir sem GPS.
        if (gpsAnchorValid)
        {
            const std::string anchorFile = savePCDDirectory + "gps_utm_anchor.txt";
            std::ofstream f(anchorFile);
            if (f.is_open())
            {
                // BUG CORRIGIDO: sem isto o ofstream usa a precisao padrao de 6
                // DIGITOS SIGNIFICATIVOS. Como o x e' UTM (~7,5 milhoes), gravava
                // "-7.4865e+06" e a leitura de volta dava -7486500 contra os
                // -7486392.96 reais -- 107 METROS de erro na ancora. Com isso o
                // auto-relocalize por GPS colocava o robo 100 m fora, o ICP nao
                // recuperava, e a ancora parecia "nao estar sendo usada".
                f << std::fixed << std::setprecision(9);
                f << gpsAnchorX << " " << gpsAnchorY << " " << gpsAnchorYaw << "\n";
                f.close();
                cout << "Ancora GPS<->mapa salva em " << anchorFile << endl;
            }
            else
            {
                ROS_WARN_STREAM("Nao consegui gravar a ancora GPS em " << anchorFile);
            }
        }

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed" << endl;
    }

    // Publica o mapa fixo do modo localização (carregado do disco).
    //
    // Chamado de DOIS lugares, de proposito:
    //   1. no fim do carregamento das nuvens em loadMap(), pra o mapa aparecer no
    //      RViz assim que o no' sobe -- sem depender de TF, /clock, playback ou
    //      qualquer mensagem do robo (ver o comentario sobre o waitForTransform
    //      logo abaixo, em loadMap(), que bloqueia o construtor por segundos);
    //   2. a cada ciclo do visualizeGlobalMapThread (5 s), pra republicar.
    //
    // Republicar e' obrigatorio, nao e' desperdicio: essa nuvem sai no frame
    // odometryFrame ("odom"), mas o Fixed Frame do RViz e' "map_ground_link" (ver
    // rviz_alberto.rviz). Pra desenhar, o RViz precisa da TF map_ground_link<-odom
    // NO TIMESTAMP DA MENSAGEM. Publicando so' uma vez no boot, ou a TF ainda nao
    // existia, ou existia e depois expirava do buffer de TF do RViz (historico
    // default ~10 s) -- nos dois casos o display morria com "No transform to fixed
    // frame" e, sem uma segunda mensagem, nunca se recuperava. E' exatamente por
    // isso que o map_local sempre funcionou: e' republicado a cada scan, com stamp
    // sempre fresco. Republicar tambem deixa o mapa imune a reinicio do RViz,
    // restart do playback e salto de /clock pra tras.
    void publishLocGlobalMap()
    {
        if (!localizationMode)
            return;

        // A nuvem e' FIXA, entao so' e' montada e voxelizada UMA vez e fica cacheada
        // em locGlobalMapMsg; as republicacoes so' trocam o header.stamp (custo ~0).
        if (locGlobalMapMsg.data.empty())
        {
            if (laserCloudCornerFromMapDS->empty() && laserCloudSurfFromMapDS->empty())
                return; // loadMap() ainda nao carregou as nuvens

            pcl::PointCloud<PointType>::Ptr loadedMap(new pcl::PointCloud<PointType>());
            *loadedMap += *laserCloudCornerFromMapDS;
            *loadedMap += *laserCloudSurfFromMapDS;

            // Filtro para o RViz respirar (NÃO afeta a precisão do ICP/Localização!)
            pcl::PointCloud<PointType>::Ptr loadedMapDS(new pcl::PointCloud<PointType>());
            pcl::VoxelGrid<PointType> downSizeFilterRViz;

            // Tamanho do voxel visual. Se o mapa for muito gigante, suba para 1.0 ou 2.0
            downSizeFilterRViz.setLeafSize(0.5, 0.5, 0.5);
            downSizeFilterRViz.setInputCloud(loadedMap);
            downSizeFilterRViz.filter(*loadedMapDS);

            // FIX: publica em mapFrame ("map"), NAO em odometryFrame ("odom").
            //
            // Publicar a mensagem ja' funcionava desde o boot, mas o RViz nao
            // conseguia DESENHAR ate' o robo aparecer: pra levar a nuvem ate' o Fixed
            // Frame ("map_ground_link") ele precisa da cadeia map_ground_link <- odom,
            // e quem publica map -> odom e' o imuPreintegration DENTRO do
            // imuOdometryHandler -- ou seja, so' quando chega odometria do robo. Sem
            // robo, o frame "odom" nem existe na arvore e a nuvem fica invisivel.
            //
            // "map", ao contrario, ja' esta' na arvore desde o boot pelo tf_static do
            // pointcloud_node (map -> map_base_link -> map_ground_link). E map -> odom
            // e' IDENTIDADE fixa (ver imuPreintegration.cpp: map_to_odom com RPY 0,0,0
            // e translacao 0,0,0), entao a nuvem sai numericamente igual -- so' passa a
            // ser desenhavel sem depender de nada do robo.
            pcl::toROSMsg(*loadedMapDS, locGlobalMapMsg);
            locGlobalMapMsg.header.frame_id = mapFrame;   // depois do toROSMsg: ele sobrescreve o header

            ROS_INFO("Mapa global montado pra visualizacao: %zu pontos (voxel 0.5 m), frame '%s' "
                     "(nao depende de odom/robo pra ser desenhado).",
                     loadedMapDS->size(), mapFrame.c_str());
        }

        // A 1a publicacao sai SEM gate de subscriber, de proposito: o publisher e'
        // latched (ver construtor), entao publicar no vazio e' o que carrega o latch
        // e garante que o RViz receba o mapa no instante em que assinar. Da 2a em
        // diante gateia por subscriber pra nao mandar dezenas de MB no vazio.
        if (!locGlobalMapEverPublished || pubLaserCloudSurround.getNumSubscribers() > 0)
        {
            locGlobalMapMsg.header.stamp = ros::Time::now();
            pubLaserCloudSurround.publish(locGlobalMapMsg);

            if (!locGlobalMapEverPublished)
                ROS_INFO("Mapa global publicado (latched) no boot -- independente de robo/playback/TF.");
            locGlobalMapEverPublished = true;
        }
    }

    void publishGlobalMap()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // In localization mode there are no per-keyframe feature clouds to rebuild the global
        // map from; just publish the fixed map that was loaded from disk.
        //
        // FIX: o gate "sem subscriber, nem tenta" (que existia aqui antes, pra
        // nao gastar CPU montando o mapa de visualizacao do modo MAPEAMENTO
        // toda hora) foi removido pra esse caminho -- em localizationMode isso
        // so' roda UMA VEZ (ver mapPublished abaixo), e com o publisher agora
        // latched (ver construtor), publicar mesmo sem subscriber ainda
        // conectado e' exatamente o que garante que o RViz recebe o mapa
        // quando conectar, nao importa se chegou atrasado.
        if (localizationMode)
        {
            publishLocGlobalMap();
            return;
        }

        // MODO MAPEAMENTO: isso roda em loop (a cada 5s) o resto da sessao,
        // entao aqui SIM vale poupar CPU se ninguem ta olhando -- diferente
        // do caminho de localizationMode acima, que so' roda uma vez.
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }


    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            
            if (localizationMode) 
            {
                // MODO LOCALIZAÇÃO: Usa o SC apenas para gerar um "chute" (Guess) para o locSnapWorker
                if (locAutoRelocArmed || locResetRequested) {
                    // FIX: detectLoopClosureID() compara contra polarcontexts_.back().
                    // Em localizationMode ninguem chama makeAndSaveScancontextAndKeys()
                    // pro scan AO VIVO (saveKeyFramesAndFactor(), que faz isso, so roda
                    // no modo mapeamento) -- sem empurrar o scan atual aqui, ".back()"
                    // seria so o ultimo keyframe do MAPA carregado, e a busca comparava
                    // o mapa contra ele mesmo, nunca contra o robo de verdade.
                    pcl::PointCloud<PointType>::Ptr liveScanForSC(new pcl::PointCloud<PointType>());
                    {
                        std::lock_guard<std::mutex> lockMain(mtx);
                        *liveScanForSC += *laserCloudRawDS;
                    }
                    if (liveScanForSC->empty())
                    {
                        // ainda nao tem scan processado (boot muito recente) -- tenta de novo no proximo tick
                        continue;
                    }
                    scManager.makeAndSaveScancontextAndKeys(*liveScanForSC);

                    auto detectResult = scManager.detectLoopClosureID();
                    int scMatchID = detectResult.first;

                    if (scMatchID != -1 && !locSnapPending.load()) {
                        ROS_INFO("Scan Context encontrou o mapa global! ID: %d", scMatchID);

                        std::lock_guard<std::mutex> lockSnap(mtxLocSnap);
                        // Puxa o X, Y, Z, Roll, Pitch e Yaw exatos daquele nó salvo no mapa
                        locSnapGuess[0] = cloudKeyPoses6D->points[scMatchID].roll;
                        locSnapGuess[1] = cloudKeyPoses6D->points[scMatchID].pitch;
                        locSnapGuess[2] = cloudKeyPoses6D->points[scMatchID].yaw;
                        locSnapGuess[3] = cloudKeyPoses6D->points[scMatchID].x;
                        locSnapGuess[4] = cloudKeyPoses6D->points[scMatchID].y;
                        locSnapGuess[5] = cloudKeyPoses6D->points[scMatchID].z;

                        locSnapScan->clear();
                        {
                            std::lock_guard<std::mutex> lockMain(mtx);
                            *locSnapScan += *laserCloudRawDS;
                        }
                        locSnapPending = true; // Dispara a thread de alinhamento ICP do LIO-SAM
                        locAutoRelocArmed = false; // achou candidato; so tenta de novo apos um reset (locResetRequested)
                    }
                }
            } 
            else 
            {
                // MODO MAPEAMENTO: Funcionamento normal que otimiza o mapa
                performRSLoopClosure();
                performSCLoopClosure(); 
                visualizeLoopClosure();
            }
        }
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performRSLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        copy_cloudKeyPoses2D->clear(); // giseop
        *copy_cloudKeyPoses2D = *cloudKeyPoses3D; // giseop 
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        std::cout << "RS loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        // 150 -> historyKeyframeSearchRadius (50 m). detectLoopClosureDistance so' aceita
        // candidatos DENTRO de historyKeyframeSearchRadius, entao o deslocamento a corrigir
        // nunca passa disso; 150 m era 3x o necessario e so' abria espaco pro ICP casar
        // pontos sem relacao e cair em minimo local ruim (loop rejeitado no fitness).
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
            std::cout << "ICP fitness test failed (" << icp.getFitnessScore() << " > " << historyKeyframeFitnessScore << "). Reject this RS loop." << std::endl;
            return;
        } else {
            std::cout << "ICP fitness test passed (" << icp.getFitnessScore() << " < " << historyKeyframeFitnessScore << "). Add this RS loop." << std::endl;
        }

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        // loopIndexContainer[loopKeyCur] = loopKeyPre;
        loopIndexContainer.insert(std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap
    } // performRSLoopClosure


    void performSCLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // find keys
        auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;;
        int loopKeyPre = detectResult.first;
        float yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
        if( loopKeyPre == -1 /* No loop found */)
            return;

        std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            // loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, loopKeyPre); // giseop 
            // loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);

            //int base_key = 0;
            //loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
            //loopFindNearKeyframesWithRespectTo(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key); // giseop 

            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);

            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius); // idem performRSLoopClosure: era 150 m
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);
        // giseop 
        // TODO icp align with initial 

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
            std::cout << "ICP fitness test failed (" << icp.getFitnessScore() << " > " << historyKeyframeFitnessScore << "). Reject this SC loop." << std::endl;
            return;
        } else {
            std::cout << "ICP fitness test passed (" << icp.getFitnessScore() << " < " << historyKeyframeFitnessScore << "). Add this SC loop." << std::endl;
        }

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        // // transform from world origin to wrong pose
        // Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // // transform from world origin to corrected pose
        // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        // pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        // gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

        // gtsam::Vector Vector6(6);
        // float noiseScore = icp.getFitnessScore();
        // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        // noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // giseop 
        //pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
        //gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        //gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

        // giseop, robust kernel for a SC loop
        float robustNoiseScore = 0.5; // constant is ok...
        gtsam::Vector robustNoiseVector6(6); 
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
        noiseModel::Base::shared_ptr robustConstraintNoise; 
        robustConstraintNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
        ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(robustConstraintNoise);
        mtx.unlock();

        // add loop constriant
        // loopIndexContainer[loopKeyCur] = loopKeyPre;
        loopIndexContainer.insert(std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap
    } // performSCLoopClosure


    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop; // unused 
        // kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        // kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
 
        for (int i = 0; i < (int)copy_cloudKeyPoses2D->size(); i++) // giseop
            copy_cloudKeyPoses2D->points[i].z = 1.1; // to relieve the z-axis drift, 1.1 is just foo val

        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D); // giseop
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses2D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0); // giseop
        
        // std::cout << "the number of RS-loop candidates  " << pointSearchIndLoop.size() << "." << std::endl; // giseop
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void loopFindNearKeyframesWithRespectTo(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int _wrt_key)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[_wrt_key]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[_wrt_key]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1; markerEdge.scale.y = 0.1; markerEdge.scale.z = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }



    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // FIX: lastImuTransformation so' vale como referencia se tiver sido gravada a
        // partir de uma atitude REAL. Sem isso o primeiro sweep, que costuma chegar
        // antes do IMU/odometria cobrirem o scan, gravava zeros como referencia; no
        // sweep seguinte a atitude verdadeira aparecia e a DIFERENCA inteira era
        // aplicada como incremento de rotacao. Era o giro de 137 graus (2.3923 rad) em
        // um unico frame: o keyframe 0 ficava em yaw=0 e os demais em yaw=2.39,
        // deformando o mapa logo no comeco.
        static bool lastImuTransValid = false;

        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            lastImuTransValid = (cloudInfo.imuAvailable == true);
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;

                // FIX (modo ackermann: "fica girando ate' fazer o primeiro mapa").
                //
                // Em odometrySource=="ackermann" a orientacao e' dead reckoning puro
                // (omega = v*tan(phi)/L, ver AckermannPreintegration). Qualquer VIES em
                // phi produz omega != 0 mesmo com o veiculo andando reto, entao a pose
                // descreve um circulo -- robo e nuvem girando solidarios. No comeco do
                // mapeamento o mapa ainda e' pobre demais pro scan-to-map corrigir isso,
                // e so' depois de acumular keyframes o giro para sozinho.
                //
                // Criterio de destrave: o MAPA GLOBAL existir (> 0 pontos). Enquanto o
                // mapa contra o qual o scan e' alinhado estiver vazio, o scan-to-map nao
                // tem como corrigir o yaw, entao o giro do dead reckoning passa direto --
                // e' exatamente ai' que o yaw fica congelado. Assim que o mapa tem
                // conteudo, o ganho de rotacao volta a ser aplicado normalmente (curvas
                // reais voltam a ser seguidas).
                //
                // ackermannYawLockKeyframes e' um reforco OPCIONAL: se > 0, segura o yaw
                // por mais tempo, ate' o mapa ter esse tanto de keyframes. Com 0 (padrao)
                // vale so' a condicao "mapa global > 0". Nao mexe em "imu"/"fusion", onde
                // o IMU ja' ancora a rotacao.
                const bool yawLocked = rotationLockActive();
                const float yawBeforePredict = transformTobeMapped[2];

                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                if (yawLocked)
                {
                    transformTobeMapped[2] = yawBeforePredict;
                    ROS_WARN_THROTTLE(2.0,
                        "Ackermann: yaw congelado -- mapa global com %d pts corner / %d surf, "
                        "%zu keyframes. Sem mapa o scan-to-map nao corrige o giro do dead "
                        "reckoning. Destrava sozinho quando o mapa global > 0"
                        "%s.",
                        laserCloudCornerFromMapDSNum, laserCloudSurfFromMapDSNum,
                        cloudKeyPoses3D->size(),
                        (ackermannYawLockKeyframes > 0)
                            ? " e o mapa atingir lio_sam/ackermannYawLockKeyframes keyframes" : "");
                }

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                lastImuTransValid = (cloudInfo.imuAvailable == true);
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);

            // FIX: primeira atitude REAL depois de um periodo sem dado (tipico do boot,
            // e tambem se o IMU/odometria falharem no meio). Aqui a referencia antiga e'
            // lixo -- so' re-sincroniza e sai, SEM aplicar o delta. Aplicar produziria um
            // salto de rotacao do tamanho do heading absoluto do sensor.
            if (!lastImuTransValid)
            {
                lastImuTransformation = transBack;
                lastImuTransValid = true;
                ROS_WARN("updateInitialGuess: primeira atitude valida (roll=%.3f pitch=%.3f yaw=%.3f rad) -- "
                         "re-sincronizando a referencia sem aplicar salto de rotacao.",
                         cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
                return;
            }

            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    void updateInitialGuessLocalization()
    {
        // Same idea as updateInitialGuess(), but for localization mode: the pose is seeded
        // from a known initial pose (parameter or RViz) instead of the map origin, and the
        // incremental trackers are member variables so the pose can be reset at runtime.

        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        // (re)initialization: on the very first frame or after a runtime pose reset,
        // just anchor on the current (seeded) pose and skip the incremental update.
        if (!locPoseInitialized || locResetRequested)
        {
            locPoseInitialized = true;
            locResetRequested = false;
            locLastImuPreTransAvailable = false;
            locLastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            // FIX: o cold start (boot do no) e' o MESMO tipo de janela arriscada que um
            // reset em runtime -- buffers do imageProjection ainda nao flusharam e o
            // AckermannPreintegration normalmente ainda nao tem amostra nenhuma (ver
            // WARN "sem amostra Ackermann integrada nesta janela" logo apos o boot).
            // Antes, timeLastLocReset so era armado no reset via RViz/snap worker (linha
            // ~630), entao nos primeiros segundos apos o boot o Escudo Anti-Teleporte
            // ficava desarmado e o scan2map corria sem essa protecao -- exatamente a
            // janela em que aparecem os saltos de varios metros / dezenas de graus no
            // log (limiares -1/-1, odomReal=0, caindo so no teto fixo de 1.2m/15deg,
            // repetido muitas vezes por segundo ate o Ackermann sincronizar).
            timeLastLocReset = timeLaserInfoCur;
            return;
        }

        // NOVO: Escudo Anti-Teleporte Fantasma!
        // Ignora os chutes do imageProjection (que ainda tem buffer velho) por 1.5s apos o clique.
        if (timeLastLocReset > 0 && (timeLaserInfoCur - timeLastLocReset) < 1.5)
        {
            return; 
        }

        // use imu pre-integration estimation for pose guess
        if (cloudInfo.odomAvailable == true)
        {
            printf("[GUESS_IN] t=%.3f x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f\n",
                   timeLaserInfoCur,
                   cloudInfo.initialGuessX, cloudInfo.initialGuessY, cloudInfo.initialGuessZ,
                   cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            fflush(stdout);

            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (locLastImuPreTransAvailable == false)
            {
                locLastImuPreTransformation = transBack;
                locLastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = locLastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                locLastImuPreTransformation = transBack;

                locLastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        //
        // FIX: este bloco aplica SO' ROTACAO. Em modo ackermann isso e' veneno:
        // o ackermannDeskewInfo() do imageProjection (linha 354) forca
        // cloudInfo.imuAvailable = true MESMO SEM IMU -- e' assim que ele habilita
        // o deskew --, e preenche imuRollInit/Pitch/Yaw a partir da atitude da
        // propria odometria Ackermann. Resultado: sempre que odomAvailable for
        // false, caia-se aqui e o chute ganha rotacao com TRANSLACAO ZERO, ciclo
        // apos ciclo. O veiculo anda, o chute nao -- exatamente o sintoma de "as
        // nuvens giram e nao encaixam, como se o peso da odometria fosse 0".
        //
        // Pior: o WARN de diagnostico abaixo ficava INALCANCAVEL em modo
        // ackermann (imuAvailable sempre true), entao a falha era silenciosa.
        //
        // Mesma protecao que ja existia em outros tres pontos do arquivo
        // (linhas ~3304, ~3320 e ~4419), onde esse efeito colateral ja tinha sido
        // percebido -- esta funcao e' que nunca foi blindada.
        if (cloudInfo.imuAvailable == true && odometrySource != "ackermann")
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = locLastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            locLastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            return;
        }

        // DIAGNOSTICO: sem odometria pra predizer. O chute inicial fica CONGELADO
        // neste ciclo (transformTobeMapped nao atualizado) e o scan-to-map roda a
        // partir do chute do ciclo anterior -- com o robo ja' em outro lugar.
        // Em modo ackermann, chegar aqui significa que o AckermannPreintegration
        // nao esta' cobrindo o scan (odomTopic sem mensagem no intervalo).
        ROS_WARN_THROTTLE(1.0,
            "updateInitialGuessLocalization: sem predicao neste ciclo -- chute inicial "
            "CONGELADO (odomAvailable=%d, imuAvailable=%d, odometrySource='%s'). "
            "Em ackermann isso quer dizer que o AckermannPreintegration nao cobriu o scan.",
            (int)cloudInfo.odomAvailable, (int)cloudInfo.imuAvailable, odometrySource.c_str());
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    // ---- GPU/CPU downsample helper ------------------------------------
    // Tenta a GPU primeiro; se cudaAvailable==false OU a chamada falhar em
    // runtime (driver, OOM, etc.), cai pro pcl::VoxelGrid original sem
    // travar o node.
    void gpuVoxelDownsample(pcl::PointCloud<PointType>::Ptr& cloudIn,
                             pcl::PointCloud<PointType>::Ptr& cloudOut,
                             pcl::VoxelGrid<PointType>& cpuFilter,
                             float leafSize)
    {
        cloudOut->clear();
        if (cudaAvailable && !cloudIn->empty())
        {
            std::vector<float> xyziIn(cloudIn->size() * 4);
            for (size_t i = 0; i < cloudIn->size(); ++i)
            {
                xyziIn[i*4+0] = cloudIn->points[i].x;
                xyziIn[i*4+1] = cloudIn->points[i].y;
                xyziIn[i*4+2] = cloudIn->points[i].z;
                xyziIn[i*4+3] = cloudIn->points[i].intensity;
            }
            std::vector<float> xyziOut;
            if (cuda_map_search::voxelDownsample(xyziIn, (int)cloudIn->size(), leafSize, xyziOut))
            {
                size_t n = xyziOut.size() / 4;
                cloudOut->resize(n);
                for (size_t i = 0; i < n; ++i)
                {
                    cloudOut->points[i].x = xyziOut[i*4+0];
                    cloudOut->points[i].y = xyziOut[i*4+1];
                    cloudOut->points[i].z = xyziOut[i*4+2];
                    cloudOut->points[i].intensity = xyziOut[i*4+3];
                }
                return;
            }
            ROS_WARN_THROTTLE(10.0, "GPU voxel downsample failed at runtime, using CPU for this scan");
        }
        cpuFilter.setInputCloud(cloudIn);
        cpuFilter.filter(*cloudOut);
    }

    // ---- GPU map upload helpers ----------------------------------------
    bool uploadCornerMapToGpu(pcl::PointCloud<PointType>::Ptr& mapCloud)
    {
        if (!cudaAvailable || mapCloud->size() < 5) return false;
        std::vector<float> xyz(mapCloud->size() * 3);
        for (size_t i = 0; i < mapCloud->size(); ++i)
        {
            xyz[i*3+0] = mapCloud->points[i].x;
            xyz[i*3+1] = mapCloud->points[i].y;
            xyz[i*3+2] = mapCloud->points[i].z;
        }
        return cuda_map_search::uploadCornerMap(xyz, (int)mapCloud->size());
    }

    bool uploadSurfMapToGpu(pcl::PointCloud<PointType>::Ptr& mapCloud)
    {
        if (!cudaAvailable || mapCloud->size() < 5) return false;
        std::vector<float> xyz(mapCloud->size() * 3);
        for (size_t i = 0; i < mapCloud->size(); ++i)
        {
            xyz[i*3+0] = mapCloud->points[i].x;
            xyz[i*3+1] = mapCloud->points[i].y;
            xyz[i*3+2] = mapCloud->points[i].z;
        }
        return cuda_map_search::uploadSurfMap(xyz, (int)mapCloud->size());
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding corner key frames (or map)
        gpuVoxelDownsample(laserCloudCornerFromMap, laserCloudCornerFromMapDS, downSizeFilterCorner, mappingCornerLeafSize);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        gpuVoxelDownsample(laserCloudSurfFromMap, laserCloudSurfFromMapDS, downSizeFilterSurf, mappingSurfLeafSize);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // Upload o mapa local pra GPU UMA VEZ por scan. As 30 iterações do LM
        // em scan2MapOptimization() vão todas consultar contra este mesmo
        // upload -- isso é o que transforma "30x upload+busca completos" em
        // "1x upload, 30x busca em lote barata".
        cornerMapUploadedToGpu = uploadCornerMapToGpu(laserCloudCornerFromMapDS);
        surfMapUploadedToGpu   = uploadSurfMapToGpu(laserCloudSurfFromMapDS);

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // giseop
        laserCloudRawDS->clear();
        downSizeFilterSC.setInputCloud(laserCloudRaw);
        downSizeFilterSC.filter(*laserCloudRawDS);        

        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        gpuVoxelDownsample(laserCloudCornerLast, laserCloudCornerLastDS, downSizeFilterCorner, mappingCornerLeafSize);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        gpuVoxelDownsample(laserCloudSurfLast, laserCloudSurfLastDS, downSizeFilterSurf, mappingSurfLeafSize);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();        

    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        // Busca em lote na GPU: transforma todos os pontos do scan atual pro
        // frame do mapa de uma vez, manda pra GPU, e traz de volta os 5
        // vizinhos de cada um -- substitui as N chamadas individuais de
        // kdtreeCornerFromMap->nearestKSearch() por 1 kernel launch.
        bool useGpu = cudaAvailable && cornerMapUploadedToGpu && laserCloudCornerLastDSNum > 0;
        std::vector<PointType> pointSelCache;
        std::vector<int> gpuIdx;
        std::vector<float> gpuSqDist;

        if (useGpu)
        {
            std::vector<float> queryXYZ(laserCloudCornerLastDSNum * 3);
            pointSelCache.resize(laserCloudCornerLastDSNum);
            for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
            {
                pointAssociateToMap(&laserCloudCornerLastDS->points[i], &pointSelCache[i]);
                queryXYZ[i*3+0] = pointSelCache[i].x;
                queryXYZ[i*3+1] = pointSelCache[i].y;
                queryXYZ[i*3+2] = pointSelCache[i].z;
            }
            useGpu = cuda_map_search::knnSearchCorner5(queryXYZ, laserCloudCornerLastDSNum, gpuIdx, gpuSqDist);
        }

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd(5);
            std::vector<float> pointSearchSqDis(5);

            pointOri = laserCloudCornerLastDS->points[i];

            if (useGpu)
            {
                pointSel = pointSelCache[i];
                for (int k = 0; k < 5; ++k)
                {
                    pointSearchInd[k]   = gpuIdx[i*5+k];
                    pointSearchSqDis[k] = gpuSqDist[i*5+k];
                }
            }
            else
            {
                pointAssociateToMap(&pointOri, &pointSel);
                kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            }

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        bool useGpu = cudaAvailable && surfMapUploadedToGpu && laserCloudSurfLastDSNum > 0;
        std::vector<PointType> pointSelCache;
        std::vector<int> gpuIdx;
        std::vector<float> gpuSqDist;

        if (useGpu)
        {
            std::vector<float> queryXYZ(laserCloudSurfLastDSNum * 3);
            pointSelCache.resize(laserCloudSurfLastDSNum);
            for (int i = 0; i < laserCloudSurfLastDSNum; ++i)
            {
                pointAssociateToMap(&laserCloudSurfLastDS->points[i], &pointSelCache[i]);
                queryXYZ[i*3+0] = pointSelCache[i].x;
                queryXYZ[i*3+1] = pointSelCache[i].y;
                queryXYZ[i*3+2] = pointSelCache[i].z;
            }
            useGpu = cuda_map_search::knnSearchSurf5(queryXYZ, laserCloudSurfLastDSNum, gpuIdx, gpuSqDist);
        }

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd(5);
            std::vector<float> pointSearchSqDis(5);

            pointOri = laserCloudSurfLastDS->points[i];

            if (useGpu)
            {
                pointSel = pointSelCache[i];
                for (int k = 0; k < 5; ++k)
                {
                    pointSearchInd[k]   = gpuIdx[i*5+k];
                    pointSearchSqDis[k] = gpuSqDist[i*5+k];
                }
            }
            else
            {
                pointAssociateToMap(&pointOri, &pointSel);
                kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            }

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            // In localization mode the map is fixed and its kdtrees were already built once
            // in loadMap(), so we avoid rebuilding them (potentially large) on every scan.
            if (!localizationMode)
            {
                kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
                kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            }

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    // Mediana + k*MAD do histórico. MAD é robusto a outlier (até ~50% de
    // contaminação), então dá pra alimentar com a amostra crua, sem filtrar
    // os saltos antes. Sem amostra suficiente ainda (arranque), devolve
    // "infinito" -> gate desligado nos primeiros segundos em vez de usar
    // um valor chutado.
    // True enquanto a rotacao do robo deve ficar TRAVADA (roll+pitch+yaw), enquanto o
    // mapa ainda nao esta' pronto. Ver rotLocked[] e o uso no fim de transformUpdate().
    bool rotationLockActive()
    {
        if (odometrySource != "ackermann")
            return false;   // "imu"/"fusion" tem o IMU ancorando a atitude

        const bool globalMapEmpty = (laserCloudCornerFromMapDSNum == 0 && laserCloudSurfFromMapDSNum == 0);
        const bool tooFewKeyframes = (ackermannYawLockKeyframes > 0)
                                  && ((int)cloudKeyPoses3D->size() < ackermannYawLockKeyframes);
        return globalMapEmpty || tooFewKeyframes;
    }

    void transformUpdate()
    {
        // Em modo Ackermann a atitude (roll/pitch) fornecida via cloud_info é
        // derivada do modelo 2D (roll=pitch=0) e NÃO de um IMU real. Puxar
        // roll/pitch para esses zeros descartaria a inclinação estimada pelo
        // LiDAR. Só aplicamos o slerp de atitude quando a fonte é o IMU
        // (modo "imu" ou "fusion").
        if (cloudInfo.imuAvailable == true && odometrySource != "ackermann")
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
            
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        // ===== PESO DA ODOMETRIA ACKERMANN NO YAW (anti-giro do semi-reboque) =====
        //
        // Com semi-reboque o LiDAR fica com a traseira TAPADA pelo proprio reboque:
        // sobra menos de meio anel util dos 360 graus. Com meia varredura o
        // scan-to-map fica MAL CONDICIONADO em yaw -- a nuvem casa quase igual de bem
        // girada --, e a pose sai girando, ate' virar 180 graus. O dead reckoning do
        // Ackermann (omega = v*tan(phi)/L) erra devagar mas NAO inverte, entao serve
        // de ancora pro yaw.
        //
        // Aqui o yaw final vira uma media ponderada entre a PREDICAO do Ackermann
        // (incrementalOdometryAffineGuess = pose logo depois de updateInitialGuess,
        // antes do scan2MapOptimization) e o que o scan-to-map produziu:
        //
        //     yaw = yaw_pred + w * wrap(yaw_scan2map - yaw_pred)
        //
        // Nao mexe em roll/pitch: esses o LiDAR observa bem mesmo com meia varredura
        // (o chao continua visivel), e em ackermann puro a predicao nem tem atitude.
        applyAckermannYawWeight();

        // ===== TRAVA DE ROTACAO ATE' O MAPA FICAR PRONTO (modo ackermann) =====
        //
        // Aplicada AQUI, no fim de transformUpdate, de proposito: e' o ultimo ponto em
        // que a pose do scan e' definida, DEPOIS do scan2MapOptimization. O giro que se
        // via no comeco do mapeamento nao vinha da predicao do Ackermann -- vinha do
        // proprio scan-to-map divergindo com mapa pobre, produzindo correcoes de 51,
        // 123, 137 graus num scan so' (e o gate anti-salto abaixo esta' aposentado,
        // transThresh/rotThresh fixos em 1000, entao nada barrava isso).
        //
        // Enquanto o mapa nao esta' pronto, congela roll+pitch+yaw na atitude do
        // primeiro scan: o robo fica IMOVEL em rotacao e so' translada. Assim o
        // primeiro pedaco do mapa e' construido com atitude estavel, em vez de ser
        // envenenado por correcoes que giram o mundo. Ao destravar, o scan-to-map ja'
        // tem mapa suficiente pra convergir e as curvas reais voltam a ser seguidas.
        if (rotationLockActive())
        {
            if (!rotLockCaptured)
            {
                rotLocked[0] = transformTobeMapped[0];
                rotLocked[1] = transformTobeMapped[1];
                rotLocked[2] = transformTobeMapped[2];
                rotLockCaptured = true;
                ROS_WARN("Ackermann: rotacao TRAVADA (roll=%.3f pitch=%.3f yaw=%.3f rad) ate' o mapa ficar pronto.",
                         rotLocked[0], rotLocked[1], rotLocked[2]);
            }
            transformTobeMapped[0] = rotLocked[0];
            transformTobeMapped[1] = rotLocked[1];
            transformTobeMapped[2] = rotLocked[2];

            ROS_WARN_THROTTLE(2.0, "Ackermann: rotacao travada -- mapa global com %d corner / %d surf, "
                                   "%zu keyframes. So' translacao ate' o mapa ficar pronto.",
                              laserCloudCornerFromMapDSNum, laserCloudSurfFromMapDSNum,
                              cloudKeyPoses3D->size());
        }
        else if (rotLockCaptured)
        {
            rotLockCaptured = false;
            ROS_INFO("\033[1;32mAckermann: rotacao DESTRAVADA -- mapa pronto (%d corner / %d surf, %zu keyframes). "
                     "Curvas voltam a ser seguidas.\033[0m",
                     laserCloudCornerFromMapDSNum, laserCloudSurfFromMapDSNum, cloudKeyPoses3D->size());
        }

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);

        // Log da correcao aplicada pelo scan-to-map neste frame. O gate anti-salto que
        // existia aqui (transThresh/rotThresh + blend por slerp + histograma MAD) foi
        // REMOVIDO: estava desativado por limiares fixos em 1000 e nao e' mais usado.
        Eigen::Affine3f residual = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
        float rx, ry, rz, rroll, rpitch, ryaw;
        pcl::getTranslationAndEulerAngles(residual, rx, ry, rz, rroll, rpitch, ryaw);
        float jumpDist = sqrt(rx * rx + ry * ry);

        Eigen::Quaternionf qFront(incrementalOdometryAffineFront.rotation());
        Eigen::Quaternionf qBack(incrementalOdometryAffineBack.rotation());
        float jumpAngle = qFront.angularDistance(qBack); // rad, geodesica (nao so' yaw)

        ROS_INFO_STREAM_THROTTLE(1.0, "scan2map correction: " << jumpDist << " m / "
            << jumpAngle * 180.0 / M_PI << " deg");
    }

    // Diferenca angular levada pra (-pi, pi]. Sem isto a comparacao entre o yaw
    // predito e o yaw do scan-to-map estoura em 2*pi toda vez que um dos dois cruza
    // +-pi, e o gate barraria uma correcao de ~0 grau achando que e' de 360.
    static float wrapToPi(float angle)
    {
        while (angle >  M_PI) angle -= 2.0f * M_PI;
        while (angle <= -M_PI) angle += 2.0f * M_PI;
        return angle;
    }

    // Ver o comentario grande em transformUpdate().
    //
    // Mistura a saida do scan-to-map com a predicao do Ackermann, com peso e teto
    // separados pra YAW e pra TRANSLACAO HORIZONTAL, e rejeita o scan inteiro quando
    // a correcao passa de absurda.
    //
    // Por que so' yaw e x/y, e nao os 6 graus de liberdade:
    //   - roll/pitch: com o reboque tapando a traseira o chao continua visivel, entao
    //     o LiDAR observa bem a inclinacao. Alem disso o modelo bicicleta e' 2D e nao
    //     tem atitude nenhuma pra oferecer -- travar contra ele seria travar em zero.
    //   - z: mesmo motivo. A predicao Ackermann carrega o z anterior, entao limitar o
    //     z contra ela congelaria a altura e o caminhao nao subiria rampa.
    // Sobra exatamente o que o modelo bicicleta sabe e o LiDAR meio-cego erra: o plano
    // horizontal e o rumo.
    void applyAckermannYawWeight()
    {
        // So' faz sentido quando existe predicao de odometria de verdade. Em
        // odometrySource == "imu" o IMU ja' ancora a rotacao e o chute nao vem do
        // modelo bicicleta.
        if (odometrySource != "ackermann" && odometrySource != "fusion")
            return;

        // incrementalOdometryGuessFromOdom == false quer dizer que o cloud_info deste
        // scan veio SEM odometria (cloudInfo.odomAvailable false: a fila do
        // /ackermann/odom_raw nao cobriu o scan dentro dos +-15 ms que o
        // odomDeskewInfo() exige). Nesse scan nao ha' predicao Ackermann pra ancorar
        // em nada e o scan-to-map corre solto -- que e' justamente quando o salto
        // aparece. Loga alto, porque se isto estiver saindo direto o gate esta'
        // praticamente desligado e nenhum ajuste de peso vai adiantar.
        if (!incrementalOdometryGuessFromOdom)
        {
            ROS_WARN_THROTTLE(2.0,
                "Ackermann: SEM predicao de odometria neste scan (cloudInfo.odomAvailable=false) -- "
                "gate anti-giro INATIVO, scan-to-map livre. Se este aviso se repete, o problema e' "
                "sincronismo do /ackermann/odom_raw com o scan, nao o peso.");
            return;
        }

        const bool yawWeightOn = (ackermannYawWeight < 1.0f);
        const bool yawCapOn    = (ackermannYawMaxCorrDeg > 0.0f);
        const bool posWeightOn = (ackermannPosWeight < 1.0f);
        const bool posCapOn    = (ackermannPosMaxCorr > 0.0f);
        const bool rejectOn    = (ackermannRejectYawDeg > 0.0f || ackermannRejectPosM > 0.0f);
        if (!yawWeightOn && !yawCapOn && !posWeightOn && !posCapOn && !rejectOn)
            return;   // tudo desligado: comportamento original, 100% LiDAR

        float gx, gy, gz, groll, gpitch, gyaw;
        pcl::getTranslationAndEulerAngles(incrementalOdometryAffineGuess, gx, gy, gz, groll, gpitch, gyaw);

        const float yawRaw = wrapToPi(transformTobeMapped[2] - gyaw);
        const float dxRaw  = transformTobeMapped[3] - gx;
        const float dyRaw  = transformTobeMapped[4] - gy;
        const float posRaw = std::sqrt(dxRaw * dxRaw + dyRaw * dyRaw);

        // ── Rejeicao: o scan-to-map produziu um teleporte. Descarta a saida INTEIRA e
        // fica so' com a predicao. Nao adianta podar um pouco uma pose que ja' e' lixo:
        // se o casamento errou tanto no plano, roll/pitch/z dele tambem nao valem nada.
        // A predicao carrega a atitude e a altura do scan anterior, entao o resultado e'
        // "segura tudo e anda pela odometria" -- exatamente o que se quer num scan ruim.
        if ((ackermannRejectYawDeg > 0.0f && std::fabs(yawRaw) > ackermannRejectYawDeg * M_PI / 180.0f) ||
            (ackermannRejectPosM   > 0.0f && posRaw > ackermannRejectPosM))
        {
            pcl::getTranslationAndEulerAngles(incrementalOdometryAffineGuess,
                                              transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            ROS_WARN_THROTTLE(1.0,
                "Ackermann: scan-to-map REJEITADO -- queria mover %.2f m / %.1f graus de uma vez "
                "(limites %.2f m / %.1f deg). Scan descartado, pose segue pela odometria. "
                "isDegenerate=%d, %d edge / %d surf no scan.",
                posRaw, yawRaw * 180.0 / M_PI, ackermannRejectPosM, ackermannRejectYawDeg,
                (int)isDegenerate, laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
            return;
        }

        // ── Yaw: PESO primeiro, TETO depois. Nessa ordem os dois knobs sao
        // independentes e legiveis: o peso diz com que velocidade o LiDAR puxa o erro
        // em regime normal, o teto e' o limite duro do que sai aplicado neste scan.
        // (Na ordem inversa eles se multiplicariam e o teto viraria peso*teto, que nao
        // e' o numero que se le' no params.yaml.)
        float yawCorr = yawRaw;
        if (yawWeightOn)
            yawCorr *= std::max(0.0f, ackermannYawWeight);
        if (yawCapOn)
        {
            const float cap = ackermannYawMaxCorrDeg * M_PI / 180.0f;
            if (yawCorr >  cap) yawCorr =  cap;
            if (yawCorr < -cap) yawCorr = -cap;
        }

        transformTobeMapped[2] = wrapToPi(gyaw + yawCorr);

        // ── Translacao horizontal: peso e teto agem sobre o VETOR (dx,dy), pra nao
        // torcer a direcao da correcao -- limitar x e y separados enviesaria o
        // deslocamento pro eixo de maior componente.
        float scale = posWeightOn ? std::max(0.0f, ackermannPosWeight) : 1.0f;
        if (posCapOn && posRaw * scale > ackermannPosMaxCorr)
            scale = ackermannPosMaxCorr / posRaw;

        transformTobeMapped[3] = gx + dxRaw * scale;
        transformTobeMapped[4] = gy + dyRaw * scale;

        // So' loga quando a correcao foi de fato podada -- em regime normal (fracao de
        // grau, centimetros) o log seria puro ruido.
        const float yawTrimmed = std::fabs(yawRaw - yawCorr);
        const float posTrimmed = posRaw * (1.0f - scale);
        if (yawTrimmed > 0.5f * M_PI / 180.0f || posTrimmed > 0.05f)
        {
            ROS_WARN_THROTTLE(1.0,
                "Ackermann: correcao do scan-to-map podada -- yaw %.2f -> %.2f graus, "
                "pos %.3f -> %.3f m (pesos %.2f/%.2f, tetos %.1f deg / %.2f m). "
                "isDegenerate=%d, %d edge / %d surf.",
                yawRaw * 180.0 / M_PI, yawCorr * 180.0 / M_PI,
                posRaw, posRaw * scale,
                ackermannYawWeight, ackermannPosWeight,
                ackermannYawMaxCorrDeg, ackermannPosMaxCorr,
                (int)isDegenerate, laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            // ANCORA DE GAUGE. O prior original era
            //     (1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8)
            // ou seja: yaw com sigma de ~180 graus e posicao livre. Com isso o grafo
            // ficava com o gauge SOLTO -- os BetweenFactor de odometria travam a FORMA
            // da trajetoria, mas ela inteira continuava livre pra girar e transladar
            // como corpo rigido.
            //
            // O GPSFactor e' so' posicao (gtsam::Point3), nao informa rumo. Com poucos
            // keyframes numa reta, dois fixes definem uma LINHA -- e um corpo rigido
            // encaixa numa linha de duas maneiras, pra frente ou pra tras. Era dai' que
            // vinha o flip de ~180 graus no instante em que o primeiro GPS entrava
            // (visto no log: pose GLOBAL virava 169.6 graus enquanto a INCREMENTAL,
            // que nao passa pelo grafo, seguia lisa).
            //
            // Ancorado, o frame do mapa fica preso na primeira pose e o GPS so'
            // consegue ENTORTAR a trajetoria. O valor do prior nao precisa ser o rumo
            // "verdadeiro": ele so' precisa ser FIXO, porque e' ele que DEFINE o frame
            // do mapa.
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-2, 1e-2, priorYawNoise,
                              priorPosNoise, priorPosNoise, priorPosNoise).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            ROS_INFO("Grafo: ancora de gauge no keyframe 0 -- yaw var=%.3g rad^2 (sigma=%.2f deg), "
                     "pos var=%.3g m^2 (sigma=%.3f m). Sem isto o GPS (so' posicao) pode girar o mapa inteiro.",
                     priorYawNoise, std::sqrt(priorYawNoise) * 180.0 / M_PI,
                     priorPosNoise, std::sqrt(priorPosNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));

            writeVertex(0, trans2gtsamPose(transformTobeMapped));

        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtsam::Pose3 relPose = poseFrom.between(poseTo);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), relPose, odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);

            writeVertex(cloudKeyPoses3D->size(), poseTo);
            writeEdge({cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size()}, relPose); // giseop
        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            // Baseline minimo antes do PRIMEIRO fix entrar no grafo. O original era
            // 5.0 m fixo -- curto demais: no log o caminhao cruzava 5 m em t=436.8 e o
            // primeiro salto (11.2 m) vinha em t=437.18, com a trajetoria ainda sendo
            // uma reta de 6 m. Rumo nao observavel + GPS so'-posicao = encaixe ambiguo.
            static bool gpsBaselineReached = false;
            const float baseline = pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back());
            if (baseline < gpsMinBaseline)
            {
                ROS_INFO_THROTTLE(5.0,
                    "GPS: segurando os fixes -- baseline da trajetoria %.1f m < gpsMinBaseline %.1f m. "
                    "Com trajetoria curta o rumo ainda nao e' observavel e o fix (so' posicao) pode "
                    "encaixar o mapa girado.",
                    baseline, gpsMinBaseline);
                return;
            }
            if (!gpsBaselineReached)
            {
                gpsBaselineReached = true;
                ROS_INFO("\033[1;32mGPS: baseline de %.1f m atingido (%zu keyframes) -- fixes liberados "
                         "pro grafo.\033[0m", baseline, cloudKeyPoses3D->size());
            }
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;

                // ===== BRACO DE ALAVANCA BASE_LINK -> LIDAR =====
                //
                // gps_* descreve o BASE_LINK (centro do eixo traseiro, origem do
                // modelo Ackermann). O GPSFactor abaixo restringe a TRANSLACAO DO
                // KEYFRAME, e o keyframe e' a pose do LIDAR. Sem converter, o grafo
                // recebe "o lidar esta' onde o eixo traseiro esta'" -- um erro do
                // tamanho do braco, girando junto com o rumo do veiculo.
                //
                // No Atego o lidar fica ~3.8 m a frente do eixo: em rumo norte o GPS
                // puxa o mapa 3.8 m pro sul, em rumo leste puxa 3.8 m pro oeste, e
                // assim por diante. Como cada fix entra com o rumo do momento, o
                // resultado nao e' um deslocamento fixo (que a ancora absorveria) e
                // sim uma deformacao que acompanha a trajetoria.
                //
                // A atitude usada e' a do proprio scan que esta' virando keyframe
                // (transformTobeMapped), que e' a melhor estimativa disponivel do
                // rumo no instante do fix.
                {
                    tf::Quaternion qMapLidar;
                    qMapLidar.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
                    const tf::Matrix3x3 R_map_lidar(qMapLidar);
                    // R_map_base = R_map_lidar * R_lidar_base
                    const tf::Matrix3x3 R_map_base = R_map_lidar * baselink2Lidar.getBasis().transpose();
                    const tf::Vector3 lever = R_map_base * baselink2Lidar.getOrigin();

                    gps_x += lever.x();
                    gps_y += lever.y();
                    gps_z += lever.z();

                    ROS_INFO_THROTTLE(10.0,
                        "GPS: braco base_link->lidar aplicado ao fix -- (%.3f, %.3f, %.3f) m no frame "
                        "do mapa (extrinseca local %.3f, %.3f, %.3f).",
                        lever.x(), lever.y(), lever.z(),
                        baselink2Lidar.getOrigin().x(), baselink2Lidar.getOrigin().y(),
                        baselink2Lidar.getOrigin().z());
                }

                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < gpsFactorSpacing)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i]; // original 
            auto noiseBetween = loopNoiseQueue[i]; // giseop for polymorhpism // shared_ptr<gtsam::noiseModel::Base>, typedef noiseModel::Base::shared_ptr gtsam::SharedNoiseModel
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));

            writeEdge({indexFrom, indexTo}, poseBetween); // giseop
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();

        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor(); // radius search loop factor (I changed the orignal func name addLoopFactor to addLoopFactor)

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

        if( sc_input_type == SCInputType::SINGLE_SCAN_FULL ) {
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudRawDS,  *thisRawCloudKeyFrame);
            scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        }  
        else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) { 
            scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame); 
        }
        else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) { 
            pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
            loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, historyKeyframeSearchNum);
            scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud); 
        }

        // save sc data
        const auto& curr_scd = scManager.getConstRefRecentSCD();
        std::string curr_scd_node_idx = padZeros(scManager.polarcontexts_.size() - 1);

        saveSCD(saveSCDDirectory + curr_scd_node_idx + ".scd", curr_scd);


        // save keyframe cloud as file giseop
        //
        // OTIMIZACAO: a gravacao do .pcd (~2,5 MB por keyframe) saiu do caminho
        // critico. Antes savePCDFileBinary rodava AQUI, sincrono, dentro do callback
        // do LiDAR (laserCloudInfoHandler -> saveKeyFramesAndFactor): cada keyframe
        // parava o pipeline pra escrever 2,5 MB em disco. Com o espacamento antigo de
        // 0,1 m e o carro a ~6 m/s isso dava ~150 MB/s de escrita sincrona travando a
        // odometria -- e nenhuma GPU resolveria, por ser I/O na thread errada.
        // Agora so' enfileira; quem escreve e' o pcdWriterThread (ver main()).
        bool saveRawCloud { true };
        pcl::PointCloud<PointType>::Ptr thisKeyFrameCloud(new pcl::PointCloud<PointType>());
        if(saveRawCloud) {
            *thisKeyFrameCloud += *laserCloudRaw;
        } else {
            *thisKeyFrameCloud += *thisCornerKeyFrame;
            *thisKeyFrameCloud += *thisSurfKeyFrame;
        }
        {
            std::lock_guard<std::mutex> lock(mtxPcdQueue);
            pcdWriteQueue.push_back({saveNodePCDDirectory + curr_scd_node_idx + ".pcd", thisKeyFrameCloud});
            // Guarda de memoria: se o disco nao acompanhar, a fila cresce sem limite e
            // vira OOM (cada item ~2,5 MB). Avisa e descarta o mais antigo.
            if (pcdWriteQueue.size() > kPcdQueueMax)
            {
                ROS_WARN_THROTTLE(5.0, "Gravacao de Scans/*.pcd nao acompanha o mapeamento "
                                       "(fila em %zu). Descartando os mais antigos -- o disco e' o gargalo.",
                                  pcdWriteQueue.size());
                pcdWriteQueue.pop_front();
            }
        }
        pgTimeSaveStream << laserCloudRawTime << std::endl;

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    // Ponte de tempo publicada pelo pointcloud_node: header.stamp e' o stamp ROS
    // da nuvem (inicio da varredura) e time_ref e' o timestamp CARMEN original
    // (fim da varredura) -- e' esse que o graphslam_publish casa com o log.
    void carmenScanTimeHandler(const sensor_msgs::TimeReference::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mtxCarmenStamps);
        carmenStampByRosNs[msg->header.stamp.toNSec()] = msg->time_ref.toSec();
        while (carmenStampByRosNs.size() > kCarmenStampMax)
            carmenStampByRosNs.erase(carmenStampByRosNs.begin());
    }

    // /ackermann/odom_raw sai do bridge com o timestamp CARMEN convertido direto
    // (sem deslocamento de varredura), entao o stamp ROS ja' E' o timestamp CARMEN
    // a menos de ~1 ns. E' o que a coluna odometry_ts precisa pro publish_globalpos
    // achar v e phi na fila do base_ackerman.
    void ackermannStampHandler(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mtxCarmenStamps);
        ackermannCarmenStamps.push_back(msg->header.stamp.toSec());
        while (ackermannCarmenStamps.size() > kAckermannStampMax)
            ackermannCarmenStamps.pop_front();
    }

    // Timestamp CARMEN do scan atual. Casamento exato pelo stamp ROS (a chave sai
    // do mesmo int64 nos dois lados, entao nao ha' erro de arredondamento).
    bool lookupCarmenStamp(const ros::Time &rosStamp, double &carmenTs)
    {
        std::lock_guard<std::mutex> lock(mtxCarmenStamps);
        auto it = carmenStampByRosNs.find(rosStamp.toNSec());
        if (it == carmenStampByRosNs.end())
            return false;
        carmenTs = it->second;
        return true;
    }

    // Scans que existiram entre dois processados (exclusivo nas duas pontas).
    // Sao os que a fila do subCloud/imageProjection descartou -- temos o stamp
    // deles, so' nao temos a pose.
    void collectStampsBetween(int64_t aNs, int64_t bNs,
                              std::vector<std::pair<int64_t, double>> &out)
    {
        std::lock_guard<std::mutex> lock(mtxCarmenStamps);
        for (auto it = carmenStampByRosNs.upper_bound(aNs);
             it != carmenStampByRosNs.end() && it->first < bNs; ++it)
            out.push_back(*it);
    }

    void writePoseLine(const tf::Transform &T, double carmenTs)
    {
        double roll, pitch, yaw;
        tf::Matrix3x3(T.getRotation()).getRPY(roll, pitch, yaw);
        const double odomTs = nearestAckermannStamp(carmenTs);
        const double gpsTs  = nearestGpsStamp(carmenTs);

        // Formato graphslam (10 campos, ver graphslam.cpp:1063):
        //   x y z roll pitch yaw vertex_ts lidar_ts odometry_ts gps_ts
        // Coordenadas com 6 casas; timestamps com 9, porque o casamento do
        // graphslam_publish e' de 1 us e 6 casas deixariam so' meio microssegundo
        // de folga. -1 e' o sentinela do proprio graphslam pra "sem aresta desse
        // tipo" (graphslam.cpp:660) -- aparece quando nao ha' mensagem proxima.
        posesOutStream.precision(6);
        posesOutStream << T.getOrigin().x() << " " << T.getOrigin().y() << " "
                       << T.getOrigin().z() << " "
                       << roll << " " << pitch << " " << yaw << " ";
        posesOutStream.precision(9);
        posesOutStream << carmenTs << " " << carmenTs << " " << odomTs << " " << gpsTs << "\n";
        ++posesWritten;
    }

    // Timestamp CARMEN mais proximo do scan dentro de uma janela (-1 se nao houver).
    // O valor devolvido e' sempre um timestamp REAL de mensagem do log -- e' isso
    // que o graphslam_publish precisa pra casar com 1 us de tolerancia.
    double nearestStamp(const std::deque<double> &q, double carmenTs, double window)
    {
        std::lock_guard<std::mutex> lock(mtxCarmenStamps);
        double best = -1.0, bestDiff = window;
        for (double t : q)
        {
            const double d = std::fabs(t - carmenTs);
            if (d < bestDiff) { bestDiff = d; best = t; }
        }
        return best;
    }

    // Odometria roda rapido; GPS e' lento (~5-10 Hz), por isso a janela maior.
    double nearestAckermannStamp(double carmenTs) { return nearestStamp(ackermannCarmenStamps, carmenTs, 0.2); }
    double nearestGpsStamp(double carmenTs)       { return nearestStamp(gpsCarmenStamps,       carmenTs, 0.5); }

    // Extrinseca base_link -> lidar, com lookup preguicoso (o loadMap() pode nao
    // ter conseguido no boot, antes de a TF existir).
    bool ensureBaselink2Lidar()
    {
        if (baselink2LidarCached)
            return true;

        if (baselinkFrame == lidarFrame)
        {
            baselink2Lidar = tf::Transform::getIdentity();
            baselink2LidarCached = true;
            return true;
        }

        static tf::TransformListener tfListener;
        tf::StampedTransform transform;
        try {
            tfListener.lookupTransform(baselinkFrame, lidarFrame, ros::Time(0), transform);
            baselink2Lidar = transform;
            baselink2LidarCached = true;
            return true;
        } catch (const tf::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0, "TF %s -> %s indisponivel: %s",
                              baselinkFrame.c_str(), lidarFrame.c_str(), ex.what());
            return false;
        }
    }

    // Pose do carro (base_link) no frame do mapa, a partir de transformTobeMapped
    // (roll, pitch, yaw, x, y, z -- que e' a pose do LIDAR).
    tf::Transform currentBaseLinkPose()
    {
        tf::Transform T_map_lidar;
        T_map_lidar.setOrigin(tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::Quaternion q;
        q.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        T_map_lidar.setRotation(q);
        return T_map_lidar * baselink2Lidar.inverse();
    }

    // Guarda a pose SLAM CRUA do scan atual e consome os fixes de GPS que ficaram
    // entre a pose anterior e esta. Chamado uma vez por scan, nos dois modos.
    void recordRawPoseForOrigin()
    {
        if (posesOriginMode != "gps") return;
        if (!ensureBaselink2Lidar()) return;

        const tf::Transform T = currentBaseLinkPose();
        const double stamp = timeLaserInfoStamp.toSec();

        std::vector<PendingGpsFix> ready;
        tf::Transform Ta, Tb;
        double ta = 0.0, tb = 0.0;
        bool canInterp = false;
        {
            std::lock_guard<std::mutex> lock(mtxOrigin);
            prevRawPose      = lastRawPose;
            prevRawPoseStamp = lastRawPoseStamp;
            havePrevRawPose  = haveLastRawPose;

            lastRawPose      = T;
            lastRawPoseStamp = stamp;
            haveLastRawPose  = true;

            if (!havePrevRawPose || stamp <= prevRawPoseStamp)
                return;

            ta = prevRawPoseStamp; tb = stamp;
            Ta = prevRawPose;      Tb = lastRawPose;
            canInterp = (tb - ta) <= posesOriginMaxDt;

            // Fixes mais velhos que a pose anterior nunca serao cobertos.
            while (!pendingGpsFixes.empty() && pendingGpsFixes.front().t < ta)
            {
                pendingGpsFixes.pop_front();
                ++originRejTooOld;
            }
            while (!pendingGpsFixes.empty() && pendingGpsFixes.front().t <= tb)
            {
                ready.push_back(pendingGpsFixes.front());
                pendingGpsFixes.pop_front();
            }
        }

        if (ready.empty()) return;

        // Buraco grande entre scans: interpolar linearmente ali seria chute.
        if (!canInterp)
        {
            ROS_WARN_THROTTLE(10.0,
                "origem: %zu fix(es) caíram num buraco de %.2f s entre scans (> posesOriginMaxDt=%.2f) "
                "-- descartados.", ready.size(), tb - ta, posesOriginMaxDt);
            originRejTooOld += ready.size();
            return;
        }

        for (const auto &fix : ready)
        {
            const double a = (fix.t - ta) / (tb - ta);
            const tf::Transform Ti(Ta.getRotation().slerp(Tb.getRotation(), a),
                                   Ta.getOrigin().lerp(Tb.getOrigin(), a));
            accumulateOriginSample(Ti, fix);
        }
    }

    // Acumula uma medida de rumo relativo (dyaw = rumo_utm - rumo_mapa) na media
    // circular ponderada.
    void addHeadingSample(double dyaw, double w, bool fromMotion)
    {
        originSumSin += w * std::sin(dyaw);
        originSumCos += w * std::cos(dyaw);
        originWsumY  += w;
        if (fromMotion) ++originHeadingFromMotion; else ++originHeadingFromCompass;
    }

    void accumulateOriginSample(const tf::Transform &T_map_base, const PendingGpsFix &fix)
    {
        double mapRoll, mapPitch, mapYaw;
        tf::Matrix3x3(T_map_base.getRotation()).getRPY(mapRoll, mapPitch, mapYaw);
        const double dyaw = fix.yaw - mapYaw;

        std::lock_guard<std::mutex> lock(mtxOrigin);
        ++originAccepted;
        // 1) Rumo da bussola do GPS (funciona parado, mas e' o menos preciso).
        addHeadingSample(dyaw, fix.wY, false);

        // 2) Rumo do MOVIMENTO. Entre dois fixes, o deslocamento em UTM e o
        //    deslocamento no frame do mapa apontam pra mesma direcao fisica --
        //    a diferenca dos dois angulos e' o dyaw, sem depender de bussola
        //    nenhuma. E' imune a marcha a re: se o carro anda de re, os DOIS
        //    deslocamentos invertem e a diferenca continua certa.
        //
        //    O peso vem da geometria: com linha de base d e ruido de posicao
        //    sigma, o erro angular vale ~sigma*sqrt(2)/d, entao a variancia do
        //    angulo e' 2*sigma^2/d^2 e o peso (inverso) e' d^2/(2*sigma^2) --
        //    cresce com o QUADRADO da distancia. Trecho longo domina sozinho,
        //    que e' exatamente o que se quer numa bussola por deslocamento.
        //
        //    O sigma aqui NAO e' a covariancia publicada (25 m^2): aquilo e' o
        //    erro ABSOLUTO. O erro de posicao do GPS e' fortemente correlacionado
        //    entre epocas proximas, entao a DIFERENCA entre dois fixes e' bem
        //    mais precisa que cada um deles -- por isso posesOriginRelPosNoise
        //    e' um parametro separado, bem menor.
        //    BUG CORRIGIDO (a linha de base nunca acumulava): o ponto de
        //    referencia (lastSample*) era reescrito a CADA fix, inclusive quando o
        //    teste de comprimento falhava. Com isso a linha de base era sempre um
        //    unico passo entre fixes -- a 20 Hz e ~5 m/s, uns 0.25 m. Com
        //    posesOriginMinBaseline de 1 m seria preciso o veiculo andar 1 m entre
        //    dois fixes (20 m/s, 72 km/h) pra a condicao fechar uma vez; na pratica
        //    nunca fechava. Sintoma no mapa salvo:
        //
        //        # amostras=4646 ... rumo_bussola=4626 rumo_deslocamento=0
        //
        //    ou seja, 100% do rumo da origem saia da bussola do GPS -- justamente a
        //    fonte que carrega o offset de montagem das antenas, e que este
        //    estimador existia pra dispensar.
        //
        //    Agora a semente SO' avanca quando ela ja' cumpriu seu papel (amostra
        //    emitida) ou quando o par e' descartado por inconsistencia -- neste
        //    caso um dos dois lados esta' ruim e insistir nele so' propaga o erro.
        //    Enquanto o veiculo nao andou o bastante, a semente FICA, e a linha de
        //    base cresce fix a fix ate' passar do limiar.
        bool advanceSeed = true;

        if (!haveLastOriginSample)
        {
            advanceSeed = true;   // primeira amostra: vira semente
        }
        else
        {
            const double dux = fix.x - lastSampleUtmX;
            const double duy = fix.y - lastSampleUtmY;
            const double dmx = T_map_base.getOrigin().x() - lastSampleMapX;
            const double dmy = T_map_base.getOrigin().y() - lastSampleMapY;
            const double du = std::hypot(dux, duy);
            const double dm = std::hypot(dmx, dmy);

            const bool longEnough = (du >= posesOriginMinBaseline &&
                                     dm >= posesOriginMinBaseline);

            if (!longEnough)
            {
                // Ainda nao andou o suficiente: segura a semente e continua somando.
                advanceSeed = false;
            }
            else
            {
                // Os dois deslocamentos tem que ter comprimento parecido -- se nao
                // tiverem, um dos dois esta' errado (salto de GPS, ou pose SLAM que
                // pulou) e a direcao nao vale nada. Nesse caso a semente avanca
                // assim mesmo, pra nao ficar presa num par ruim pra sempre.
                const bool consistent = (std::min(du, dm) / std::max(du, dm)) > 0.7;
                if (consistent)
                {
                    const double dyawMotion = std::atan2(duy, dux) - std::atan2(dmy, dmx);
                    const double w = (dm * dm) / (2.0 * std::max(1e-6, posesOriginRelPosNoise));
                    addHeadingSample(dyawMotion, w, true);
                }
                else
                {
                    ROS_WARN_THROTTLE(10.0,
                        "origem: trecho descartado pro rumo por deslocamento -- GPS andou %.2f m e o "
                        "SLAM %.2f m no mesmo intervalo (razao %.2f < 0.70).",
                        du, dm, std::min(du, dm) / std::max(du, dm));
                }
                advanceSeed = true;
            }
        }

        if (advanceSeed)
        {
            lastSampleMapX = T_map_base.getOrigin().x();
            lastSampleMapY = T_map_base.getOrigin().y();
            lastSampleUtmX = fix.x;
            lastSampleUtmY = fix.y;
            haveLastOriginSample = true;
        }

        originSumMapX += fix.wT * T_map_base.getOrigin().x();
        originSumMapY += fix.wT * T_map_base.getOrigin().y();
        originSumMapZ += fix.wT * T_map_base.getOrigin().z();
        originSumUtmX += fix.wT * fix.x;
        originSumUtmY += fix.wT * fix.y;
        originSumUtmZ += fix.wT * fix.z;
        originWsumT   += fix.wT;
        ++originSamples;

        if (static_cast<int>(originSamples) < posesOriginMinSamples) return;
        if (originWsumT <= 0.0 || originWsumY <= 0.0) return;

        const double yaw = std::atan2(originSumSin, originSumCos);
        const double mx = originSumMapX / originWsumT;
        const double my = originSumMapY / originWsumT;
        const double mz = originSumMapZ / originWsumT;
        const double ux = originSumUtmX / originWsumT;
        const double uy = originSumUtmY / originWsumT;
        const double uz = originSumUtmZ / originWsumT;
        const double c = std::cos(yaw), s = std::sin(yaw);

        originX = ux - (c * mx - s * my);
        originY = uy - (s * mx + c * my);
        originZ = uz - mz;
        originYaw = yaw;
        // roll/pitch: ainda NAO estimados. A origem e' planar (so' yaw). Os
        // campos existem no txt e sao sempre 0.0, reservados pra quando entrarem.
        originRoll = originPitch = 0.0;

        tf::Quaternion q_origin;
        q_origin.setRPY(originRoll, originPitch, originYaw);
        T_posesOrigin = tf::Transform(q_origin, tf::Vector3(originX, originY, originZ));

        const bool first = !posesOriginReady.exchange(true);
        if (first || (originSamples - originLastSaved) >= 50)
        {
            originLastSaved = originSamples;
            ROS_INFO("\033[1;32m[origem do mundo] %zu amostras -> x: %.6f  y: %.6f  z: %.6f  "
                     "yaw: %.6f (+-%.2f deg) | rumo: %zu bussola + %zu deslocamento\033[0m",
                     originSamples, originX, originY, originZ, originYaw,
                     originYawStdDeg(), originHeadingFromCompass, originHeadingFromMotion);
            saveWorldOrigin();
        }
    }

    // Uma amostra da origem do mundo, a partir de um fix de GPS que casa no
    // tempo com uma pose SLAM. Chamado do gpsHandler.
    // Relatorio periodico: com ele da' pra ver de fora qual portao esta' barrando,
    // em vez de adivinhar por que o txt nao apareceu.
    void reportOriginDiagnostics()
    {
        ROS_INFO_THROTTLE(5.0,
            "[origem] fixes=%zu | aceitos=%zu | barrados: sem_encaixe=%zu, fora_de_alcance=%zu "
            "| na fila=%zu | amostras=%zu/%d | pronta=%s",
            originGpsMsgs, originAccepted, originRejNotAnchored, originRejTooOld,
            pendingGpsFixes.size(), originSamples, posesOriginMinSamples,
            posesOriginReady.load() ? "SIM" : "nao");
    }

    void updateWorldOriginFromGps(const nav_msgs::Odometry::ConstPtr &gpsMsg)
    {
        if (posesOriginMode != "gps")
            return;

        // Origem travada: nunca atualiza pelo GPS. Vale o que veio do
        // map_world_origin.txt (ou dos posesOrigin* do yaml). E' o que usar
        // quando o fix esta' ruim e voce nao quer que ele estrague uma origem
        // boa ja' estabelecida.
        if (posesOriginLock)
            return;

        // Congela depois das primeiras amostras se o refino estiver desligado.
        if (posesOriginReady.load() && !posesOriginRefine)
            return;

        ++originGpsMsgs;
        reportOriginDiagnostics();

        // O robo precisa estar de fato posicionado: em localizacao, encaixado
        // no mapa; em mapeamento, com pelo menos um keyframe (ali a pose SLAM
        // ja' e' auto-consistente desde o inicio). Sem isso a origem sairia da
        // pose que o robo ACHA que tem -- antes do encaixe, o chute do yaml.
        if (localizationMode ? !locEverAnchored.load() : cloudKeyPoses3D->points.empty())
        {
            ++originRejNotAnchored;
            return;
        }

        // Pose do carro em UTM: ja' vem com o braco de alavanca da antena
        // aplicado pelo gps_localization_node, e a orientacao e' o rumo.
        tf::Quaternion q_utm;
        tf::quaternionMsgToTF(gpsMsg->pose.pose.orientation, q_utm);
        double utmRoll, utmPitch, utmYaw;
        tf::Matrix3x3(q_utm).getRPY(utmRoll, utmPitch, utmYaw);

        // Fix ruim NAO e' descartado -- entra pesando menos. Esse GPS anda com
        // covariancia de 25 m^2, entao filtrar por limiar deixaria o modo "gps"
        // sem nenhuma amostra; ponderar por 1/cov aproveita tudo e deixa os fixes
        // melhores dominarem a media naturalmente.
        const double covT = std::max(0.01, 0.5 * (gpsMsg->pose.covariance[0] +
                                                  gpsMsg->pose.covariance[7]));
        const double covY = std::max(1e-4, gpsMsg->pose.covariance[35]);

        PendingGpsFix fix;
        fix.t   = gpsMsg->header.stamp.toSec();
        fix.x   = gpsMsg->pose.pose.position.x;
        fix.y   = gpsMsg->pose.pose.position.y;
        fix.z   = gpsMsg->pose.pose.position.z;
        fix.yaw = utmYaw;
        fix.wT  = 1.0 / covT;
        fix.wY  = 1.0 / covY;

        // Enfileira: quem consome e' o recordRawPoseForOrigin(), quando existir
        // um par de scans que cerque este instante.
        std::lock_guard<std::mutex> lock(mtxOrigin);
        pendingGpsFixes.push_back(fix);
        while (pendingGpsFixes.size() > kPendingGpsMax)
        {
            pendingGpsFixes.pop_front();
            ++originRejTooOld;
        }
    }

    // Dispersao da media circular do rumo. R = |soma dos versores| / soma dos pesos:
    // R -> 1 significa que todas as medidas concordam. sigma ~ sqrt(-2 ln R) e' o
    // desvio equivalente da gaussiana -- na pratica, "a precisao da bussola".
    double originYawStdDeg() const
    {
        if (originWsumY <= 0.0) return -1.0;
        const double R = std::hypot(originSumSin, originSumCos) / originWsumY;
        if (R <= 0.0 || R >= 1.0) return 0.0;
        return std::sqrt(-2.0 * std::log(R)) * 180.0 / M_PI;
    }

    std::string worldOriginFile() const { return savePCDDirectory + "map_world_origin.txt"; }

    // Persiste a origem do mundo junto do mapa. Apagar esse arquivo faz o modo
    // "gps" recomecar do zero e o "manual" voltar aos posesOrigin* do yaml
    // (que, zerados, sao a propria origem da nuvem de pontos).
    void saveWorldOrigin()
    {
        std::ofstream f(worldOriginFile());
        if (!f.is_open())
        {
            ROS_WARN_THROTTLE(30.0, "Nao consegui gravar '%s'.", worldOriginFile().c_str());
            return;
        }
        f << std::fixed << std::setprecision(9);
        f << "# origem do mundo: frame CARMEN/UTM <- frame do mapa (SC-LIO-SAM)\n";
        f << "# x y z roll pitch yaw\n";
        f << "# roll e pitch ainda NAO sao estimados (origem planar) -- reservados\n";
        f << originX << " " << originY << " " << originZ << " "
          << originRoll << " " << originPitch << " " << originYaw << "\n";
        f << "# amostras=" << originSamples
          << " yaw_std_deg=" << originYawStdDeg()
          << " rumo_bussola=" << originHeadingFromCompass
          << " rumo_deslocamento=" << originHeadingFromMotion << "\n";
        // Pesos acumulados: quem le' usa isto pra SEMEAR os acumuladores e
        // continuar refinando de onde parou, em vez de recomecar do zero.
        f << "pesos " << originWsumT << " " << originWsumY << "\n";
        f.close();
    }

    bool loadWorldOrigin()
    {
        std::ifstream f(worldOriginFile());
        if (!f.is_open())
            return false;

        bool got = false;
        double wT = 0.0, wY = 0.0;
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);

            if (line.compare(0, 6, "pesos ") == 0)
            {
                std::string tag;
                ss >> tag >> wT >> wY;
                continue;
            }

            double x, y, z, roll, pitch, yaw;
            if (!(ss >> x >> y >> z >> roll >> pitch >> yaw)) continue;
            originX = x; originY = y; originZ = z;
            originRoll = roll; originPitch = pitch; originYaw = yaw;
            got = true;
        }
        if (!got) return false;

        tf::Quaternion q;
        q.setRPY(originRoll, originPitch, originYaw);
        T_posesOrigin = tf::Transform(q, tf::Vector3(originX, originY, originZ));
        posesOriginReady = true;

        // SEMEIA os acumuladores com a origem carregada, como se ela fosse um
        // conjunto de amostras de peso wT/wY. Sem isso o refino recomecaria do
        // zero e as primeiras posesOriginMinSamples amostras SUBSTITUIRIAM a
        // origem salva -- com fix ruim, jogar fora uma origem boa.
        //
        // A semente usa media do mapa = 0 e media UTM = t: com o mesmo yaw isso
        // reproduz exatamente t = mean_utm - Rz(yaw)*mean_map, entao enquanto
        // nao chegar amostra nova a origem fica identica a' que estava no txt.
        if (wT > 0.0 && wY > 0.0)
        {
            originWsumT   = wT;
            originSumMapX = originSumMapY = originSumMapZ = 0.0;
            originSumUtmX = wT * originX;
            originSumUtmY = wT * originY;
            originSumUtmZ = wT * originZ;
            originWsumY   = wY;
            originSumSin  = wY * std::sin(originYaw);
            originSumCos  = wY * std::cos(originYaw);
            originSamples = static_cast<size_t>(posesOriginMinSamples);
        }

        ROS_INFO("Origem do mundo carregada de '%s': x=%.6f y=%.6f z=%.6f yaw=%.6f "
                 "(pesos %.2f/%.2f -- %s).",
                 worldOriginFile().c_str(), originX, originY, originZ, originYaw, wT, wY,
                 (wT > 0.0 && wY > 0.0) ? "refino continua de onde parou"
                                        : "sem pesos no txt: amostras novas vao SUBSTITUIR");
        return true;
    }

    void writeLocalizationPose()
    {
        // Só grava em modo de localização, com arquivo aberto.
        if (!posesOutEnabled)
            return;

        // Só depois que o robô encaixou no mapa. Antes disso a pose vem do chute
        // inicial / GPS / snap, e cada correção entra no arquivo como um salto.
        //
        // Com posesRequireAnchor false (default do modo "manual") este portao sai do
        // caminho: a pose inicial e' a que o operador deu em initialPose*, nao vem de
        // fix nenhum, e nao ha' correcao pendente para esperar. Sem isso, veiculo sem
        // GPS so' comecava a gravar depois de um clique de "2D Pose Estimate" no RViz --
        // e, se ninguem clicasse, o arquivo ficava vazio sem nunca dizer o porque alem
        // desta linha de log.
        //
        // O tratamento de descontinuidade adiante NAO depende deste portao: se um snap
        // acontecer no meio da gravacao, o locAnchorCount muda, o aviso sai e os
        // posesSettleScans seguintes sao descartados do mesmo jeito.
        if (posesRequireAnchor && !locEverAnchored.load())
        {
            ROS_INFO_THROTTLE(10.0,
                "poses_opt: aguardando o robo encaixar no mapa para comecar a gravar. "
                "(posesRequireAnchor=true; em veiculo sem GPS use posesOriginMode 'manual' "
                "ou ponha posesRequireAnchor: false)");
            return;
        }

        // Cada snap novo (clique no RViz, GPS, Scan Context) é um salto. Descarta
        // alguns scans depois de cada um, e avisa se isso acontecer com o arquivo
        // já em andamento -- ali o arquivo ganha uma descontinuidade.
        const int anchorNow = locAnchorCount.load();
        if (anchorNow != posesLastAnchorSeen)
        {
            if (posesLastAnchorSeen >= 0 && posesWritten > 0)
                ROS_WARN("poses_opt: novo encaixe aplicado depois de %zu poses gravadas -- "
                         "o arquivo tera' uma descontinuidade aqui. Descartando %d scans.",
                         posesWritten, posesSettleScans);
            else
                ROS_INFO("poses_opt: robo encaixado, comecando a gravar em %d scans.",
                         posesSettleScans);
            posesLastAnchorSeen = anchorNow;
            posesSettleLeft = posesSettleScans;
            posesHavePrev = false;   // nao interpolar por cima do salto do snap
        }
        if (posesSettleLeft > 0)
        {
            --posesSettleLeft;
            return;
        }

        // Filtro temporal opcional (0 = desligado).
        const double stamp = timeLaserInfoStamp.toSec();
        if (stamp < posesStartTime)
            return;

        // Timestamp CARMEN deste scan. Sem ele a linha é inútil: o graphslam_publish
        // não casaria com o log e simplesmente não publicaria essa pose.
        double carmenTs = 0.0;
        if (!lookupCarmenStamp(timeLaserInfoStamp, carmenTs))
        {
            ++posesSkippedNoCarmenStamp;
            ROS_WARN_THROTTLE(5.0,
                "poses_opt: sem timestamp CARMEN para o scan t=%.6f (%zu descartados). "
                "O pointcloud_node esta' publicando '%s'?",
                stamp, posesSkippedNoCarmenStamp, posesCarmenStampTopic.c_str());
            return;
        }

        if (!ensureBaselink2Lidar())
            return;

        // Em modo "gps" a origem so' existe depois de amostras suficientes; gravar
        // antes disso produziria linhas com a origem errada no comeco do arquivo.
        if (posesOriginMode == "gps" && !posesOriginReady.load())
        {
            ROS_INFO_THROTTLE(5.0,
                "poses_opt: aguardando o GPS fixar a origem do mundo (%zu/%d amostras).",
                originSamples, posesOriginMinSamples);
            return;
        }

        const tf::Transform T_odom_base = currentBaseLinkPose();

        // Coordenadas do proprio SC-LIO-SAM, com deslocamento rigido opcional.
        // Identidade (padrao) => grava exatamente o que o SC-LIO-SAM estima.
        const tf::Transform T_out = T_posesOrigin * T_odom_base;

        // Preenche os scans que o no' nao processou entre a pose anterior e esta.
        // O bridge publicou o stamp deles, entao sabemos exatamente QUANDO cada um
        // aconteceu -- falta so' a pose, que sai por interpolacao (lerp na posicao,
        // slerp na atitude) entre duas poses de scan-to-map vizinhas.
        //
        // O que estraga a interpolacao nao e' o buraco ser longo, e' a CURVA: em
        // reta, interpolar 2 s e' exato; numa curva fechada 0,3 s ja' erra. A flecha
        // da corda sobre um arco de comprimento s com variacao de rumo dyaw vale
        // ~s*dyaw/8 -- e temos s e dyaw das duas poses vizinhas. Entao o criterio e'
        // o erro estimado, com um teto de tempo so' por sanidade.
        const int64_t curRosNs = timeLaserInfoStamp.toNSec();
        if (posesHavePrev && posesInterpolateMaxGap > 0.0)
        {
            const double gap = carmenTs - posesPrevCarmenTs;

            const double chord = posesPrevT.getOrigin().distance(T_out.getOrigin());
            double r0, p0, y0, r1, p1, y1;
            tf::Matrix3x3(posesPrevT.getRotation()).getRPY(r0, p0, y0);
            tf::Matrix3x3(T_out.getRotation()).getRPY(r1, p1, y1);
            double dyaw = std::fabs(std::remainder(y1 - y0, 2.0 * M_PI));
            const double sagitta = chord * dyaw / 8.0;

            if (gap > 0.0 && gap <= posesInterpolateMaxGap && sagitta <= posesInterpolateMaxError)
            {
                std::vector<std::pair<int64_t, double>> missing;
                collectStampsBetween(posesPrevRosNs, curRosNs, missing);
                for (const auto &m : missing)
                {
                    const double a = (m.second - posesPrevCarmenTs) / gap;
                    if (a <= 0.0 || a >= 1.0) continue;
                    tf::Transform Ti(
                        posesPrevT.getRotation().slerp(T_out.getRotation(), a),
                        posesPrevT.getOrigin().lerp(T_out.getOrigin(), a));
                    writePoseLine(Ti, m.second);
                    ++posesInterpolated;
                }
            }
        }

        writePoseLine(T_out, carmenTs);
        posesOutStream.flush();

        posesHavePrev    = true;
        posesPrevRosNs   = curRosNs;
        posesPrevCarmenTs = carmenTs;
        posesPrevT       = T_out;

        if ((posesWritten % 500) == 0)
            ROS_INFO("poses_opt: %zu poses gravadas (%zu interpoladas, %.0f%%). "
                     "Ultima t_carmen=%.6f, dt ROS->CARMEN=%.4f s.",
                     posesWritten, posesInterpolated,
                     100.0 * posesInterpolated / std::max<size_t>(posesWritten, 1),
                     carmenTs, carmenTs - stamp);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        printf("[PUBLISH_GLOBAL] t=%.3f x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f\n",
               timeLaserInfoStamp.toSec(),
               transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
               transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        fflush(stdout);

        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // TF odom->lidarFrame removido: esse broadcast competia com a cadeia
        // já correta e de alta frequência odom->base_link (TransformFusion,
        // em imuPreintegration.cpp) + base_link->lidarFrame (estático, em
        // pointcloud_node.cpp). Como esse aqui só atualizava na cadência
        // (mais lenta) do scan-to-map, sempre que o mapping atrasava o frame
        // do lidar "travava" na última pose e parecia desconectar do robô.
        // A pose continua disponível via pubLaserOdometryGlobal acima, pra
        // quem precisar dela diretamente (não como TF).

        // Publish odometry for ROS (incremental)
        //bool lastIncreOdomPubFlag = false;
        //nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        //Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)   // sem "static bool" aqui
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            // Ver comentário em transformUpdate(): não puxar roll/pitch para a
            // atitude "IMU" quando ela é, na verdade, o zero do modelo Ackermann.
            if (cloudInfo.imuAvailable == true && odometrySource != "ackermann")
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }

        printf("[PUBLISH_INCREMENTAL] t=%.3f x=%.4f y=%.4f z=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f\n",
               laserOdomIncremental.header.stamp.toSec(),
               laserOdomIncremental.pose.pose.position.x,
               laserOdomIncremental.pose.pose.position.y,
               laserOdomIncremental.pose.pose.position.z,
               laserOdomIncremental.pose.pose.orientation.x,
               laserOdomIncremental.pose.pose.orientation.y,
               laserOdomIncremental.pose.pose.orientation.z,
               laserOdomIncremental.pose.pose.orientation.w);
        fflush(stdout);

        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
    std::thread locSnapThread(&mapOptimization::locSnapWorker, &MO); // NOVO
    std::thread pcdWriteThread(&mapOptimization::pcdWriterThread, &MO); // grava Scans/*.pcd fora do caminho critico

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();
    locSnapThread.join(); // NOVO
    pcdWriteThread.join();

    return 0;
}