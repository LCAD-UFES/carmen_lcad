#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <sstream>

using namespace std;

typedef std::numeric_limits< double > dbl;

typedef pcl::PointXYZI PointType;

static Eigen::Matrix3d rpyToRot(double roll, double pitch, double yaw)
{
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::Matrix3x3 m(q);
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R(i, j) = m[i][j];
    return R;
}

class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;
    string ackermannTopic;

    // Prediction odometry source: "imu", "ackermann" or "fusion"
    //   imu       = IMU preintegration only
    //   ackermann = Ackermann dead reckoning only (no IMU)
    //   fusion    = IMU + Ackermann in the same factor graph
    string odometrySource;
    // Ackermann kinematics: distance between front and rear axles (m)
    float  wheelbase;
    // Ackermann measurement noise (control): stddev of v [m/s] and phi [rad]
    float  ackermannVelNoise;
    float  ackermannSteerNoise;
    // Ackermann slip (lateral/diagonal) process noise, random walk in time:
    // stddev per sqrt(second) for x [m/sqrt(s)], y [m/sqrt(s)] and theta [rad/sqrt(s)]
    float  ackermannSlipNoiseX;
    float  ackermannSlipNoiseY;
    float  ackermannSlipNoiseTheta;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;
    // DEBUG: ignora gpsCovThreshold só na hora de calcular a âncora
    // GPS<->mapa (mapOptmization::gpsHandler). Continua respeitando
    // gpsCovThreshold normalmente pro addGPSFactor() do resto do mapeamento
    // -- isso só destrava o cálculo/gravação do gps_utm_anchor.txt com fix
    // de GPS ruim (sem RTK/DGPS), pra poder testar o fluxo sem precisar de
    // um fix bom em campo. NÃO usar em mapa de produção: a âncora sai com a
    // precisão do fix que estiver disponível.
    bool gpsAnchorIgnoreCov;

    // ===== Âncora de gauge do grafo (keyframe 0) =====
    //
    // O prior original do LIO-SAM no keyframe 0 é (1e-2, 1e-2, PI², 1e8, 1e8, 1e8):
    // yaw com sigma de ~180° e posição praticamente livre. Isso deixa o grafo com o
    // gauge SOLTO -- os BetweenFactor de odometria travam a FORMA da trajetória, mas
    // ela inteira continua livre pra girar e transladar como corpo rígido.
    //
    // O GPSFactor é só posição (gtsam::Point3), não informa rumo. Com poucos keyframes
    // numa reta, dois fixes de GPS definem uma linha -- e um corpo rígido encaixa numa
    // linha de DUAS maneiras, pra frente ou pra trás. Era daí que vinha o flip de ~180°
    // assim que o primeiro GPS entrava no grafo.
    //
    // Apertando o prior, o frame do mapa fica ancorado na primeira pose e o GPS só
    // consegue ENTORTAR a trajetória, nunca girá-la inteira. O valor não precisa ser o
    // rumo "verdadeiro": ele só precisa ser FIXO, porque é ele que define o frame do
    // mapa. Variâncias em rad² e m².
    float priorYawNoise;
    float priorPosNoise;

    // Distância em linha reta entre o primeiro e o último keyframe exigida antes do
    // PRIMEIRO GPSFactor entrar no grafo. Com baseline curto o rumo ainda não é
    // observável e o fix chega antes de a trajetória ter forma suficiente pra ser
    // encaixada de um jeito só. O original era 5.0 m, curto demais.
    float gpsMinBaseline;

    // Espaçamento entre GPSFactor sucessivos [m].
    float gpsFactorSpacing;

    // Percurso mínimo (nos DOIS frames) antes de calcular a âncora GPS<->mapa.
    // O rumo da âncora sai da diferença entre as duas DIREÇÕES DE PERCURSO, não do
    // heading que o GPS reporta: a âncora é calculada no primeiro fix bom, que chega
    // com o veículo ainda parado, e parado o GPS manda theta = 0.0000 exato. Era daí
    // que saía uma âncora com rumo 0 quando o certo eram ~171°, e cada GPSFactor
    // entrava puxando o grafo pra um frame girado de meia volta.
    float gpsAnchorMinBaseline;

    // Só para odometrySource == "ackermann". Enquanto o mapa tiver MENOS que este
    // número de keyframes, o yaw predito pelo modelo bicicleta é congelado (a
    // translação continua normal). Motivo: em ackermann puro a orientação é dead
    // reckoning (omega = v*tan(phi)/L); qualquer viés em phi faz o veículo descrever
    // um círculo mesmo andando reto, e no começo do mapeamento não há mapa suficiente
    // pro scan-to-map corrigir isso -- é o "fica girando até fazer o primeiro mapa".
    // 0 desliga o travamento.
    // ATENÇÃO: enquanto travado o robô NÃO acompanha curva real, então comece o
    // mapeamento andando reto.
    int ackermannYawLockKeyframes;

    // ===== Peso da odometria Ackermann no yaw (anti-giro do semi-reboque) =====
    //
    // Com semi-reboque o LiDAR fica com a traseira TAPADA: sobra menos de meio anel
    // útil dos 360°. Meia varredura deixa o scan-to-map mal condicionado em yaw --
    // a nuvem casa quase igual de bem girada --, e a pose sai girando até inverter.
    // O dead reckoning do Ackermann (omega = v*tan(phi)/L) erra devagar mas nunca
    // inverte, então dá pra usar como âncora do yaw.
    //
    // ackermannYawWeight: fração da correção de yaw do scan-to-map que é aceita a
    // cada scan, medida contra a predição do Ackermann.
    //   1.0 = comportamento original (100% LiDAR, sem âncora)
    //   0.3 = confia mais no Ackermann; o LiDAR corrige 30% do erro por scan
    //   0.0 = yaw puro de dead reckoning (o LiDAR não mexe no yaw)
    // Só vale quando odometrySource é "ackermann" ou "fusion" E o chute veio da
    // odometria de verdade (não do fallback de IMU).
    float ackermannYawWeight;

    // Teto ADICIONAL, em graus por scan, para a correção de yaw do scan-to-map.
    // Independe do peso: nenhuma correção maior que isto passa. É o que barra o
    // salto grande de uma vez só. 0 desliga o teto.
    float ackermannYawMaxCorrDeg;

    // Mesma ideia do par acima, para a TRANSLAÇÃO HORIZONTAL (x,y). É o que segura o
    // "salto": com meia varredura o scan-to-map escorrega ao longo de corredor/muro e
    // joga a pose metros à frente ou atrás num scan só.
    //   ackermannPosWeight  : fração da correção de x/y aceita por scan (1.0 = original)
    //   ackermannPosMaxCorr : teto da correção de x/y por scan [m]; 0 = sem teto
    // Não age em z: a predição Ackermann é 2D e carrega o z anterior, então limitar o z
    // contra ela congelaria a altura e o caminhão não subiria rampa.
    float ackermannPosWeight;
    float ackermannPosMaxCorr;

    // Rejeição total: se a correção que o scan-to-map quer aplicar passar de um destes,
    // a saída do scan é DESCARTADA inteira e a pose segue só pela odometria naquele
    // scan. É a rede de segurança contra o teleporte/inversão de uma vez, quando o
    // casamento caiu num mínimo completamente errado. 0 desliga.
    float ackermannRejectYawDeg;
    float ackermannRejectPosM;

    // Tamanho das filas dos subscribers da cadeia SLAM.
    // 1 (padrao) = sempre processa o scan MAIS NOVO e descarta o resto -- correto
    // no carro, onde pose atrasada e' pior que pose faltando.
    // Em PLAYBACK/geracao de mapa e' o contrario: latencia nao importa e cada scan
    // perdido e' um buraco. Suba pra 50-100 e o no' absorve os transientes (buraco,
    // lombada) atrasando um pouco em vez de perder pose.
    int    slamQueueSize;

    // Busca de mapa na GPU (CUDA). Padrao FALSE -- no carro a GPU ja' esta'
    // ocupada com a IA, e localizar contra mapa fixo cabe na CPU. Ligue com
    // gpu:=true no roslaunch quando a GPU estiver livre.
    bool useGpu;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Localization mode (reuse a previously saved map instead of building/erasing it)
    bool localizationMode;
    float initialPoseX;
    float initialPoseY;
    float initialPoseZ;
    float initialPoseRoll;
    float initialPosePitch;
    float initialPoseYaw;

    // Localization pose logging (graphslam poses_opt.dat format). Only in localizationMode.
    // As coordenadas gravadas sao as do proprio SC-LIO-SAM. posesOrigin* aplica um
    // deslocamento rigido opcional (identidade = coordenadas puras do SC-LIO-SAM;
    // preenchido com a ancora UTM = arquivo sobreposto ao mapa do CARMEN).
    // Double porque UTM precisa de ~13 digitos significativos; float truncaria.
    bool   posesEnable;            // liga/desliga a gravacao do poses_opt.dat
    std::string posesOutputFile;
    double posesStartTime;
    double posesOriginX;
    double posesOriginY;
    double posesOriginZ;
    double posesOriginYaw;
    // "manual" = usa posesOrigin* fixos (funciona como GPS falso: voce escolhe
    //            onde fica a origem do mapa, ex.: 500000, 0).
    // "gps"    = calcula a origem a partir do GPS real do carro + rumo.
    std::string posesOriginMode;
    bool   posesOriginLock;        // trava a origem: nunca atualiza pelo GPS
    bool   posesOriginRefine;      // continua refinando enquanto encaixado
    int    posesOriginMinSamples;  // amostras antes de considerar valida
    double posesOriginMaxDt;       // casamento GPS <-> pose SLAM (s)
    double posesOriginMinBaseline; // deslocamento minimo (m) pra tirar rumo do movimento
    double posesOriginRelPosNoise; // variancia do erro RELATIVO do GPS (m^2)
    int    posesSettleScans;
    double posesInterpolateMaxGap;
    double posesInterpolateMaxError;
    std::string posesCarmenStampTopic;
    std::string posesAckermannTopic;

    // LiDAR sensor configuration (Velodyne point format: x,y,z,intensity,ring,time)
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");
        nh.param<std::string>("lio_sam/ackermannTopic", ackermannTopic, "/ackermann/odom_raw");

        nh.param<std::string>("lio_sam/odometrySource", odometrySource, "imu");
        nh.param<float>("lio_sam/wheelbase", wheelbase, 2.625);
        nh.param<float>("lio_sam/ackermannVelNoise", ackermannVelNoise, 0.1);
        nh.param<float>("lio_sam/ackermannSteerNoise", ackermannSteerNoise, 0.02);
        nh.param<float>("lio_sam/ackermannSlipNoiseX", ackermannSlipNoiseX, 0.02);
        nh.param<float>("lio_sam/ackermannSlipNoiseY", ackermannSlipNoiseY, 0.05);
        nh.param<float>("lio_sam/ackermannSlipNoiseTheta", ackermannSlipNoiseTheta, 0.01);

        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<bool>("lio_sam/gpsAnchorIgnoreCov", gpsAnchorIgnoreCov, false);
        nh.param<float>("lio_sam/priorYawNoise", priorYawNoise, 1e-2);
        nh.param<float>("lio_sam/priorPosNoise", priorPosNoise, 1e-4);
        nh.param<float>("lio_sam/gpsMinBaseline", gpsMinBaseline, 30.0);
        nh.param<float>("lio_sam/gpsFactorSpacing", gpsFactorSpacing, 5.0);
        nh.param<float>("lio_sam/gpsAnchorMinBaseline", gpsAnchorMinBaseline, 15.0);
        nh.param<int>("lio_sam/ackermannYawLockKeyframes", ackermannYawLockKeyframes, 10);
        nh.param<float>("lio_sam/ackermannYawWeight", ackermannYawWeight, 1.0);
        nh.param<float>("lio_sam/ackermannYawMaxCorrDeg", ackermannYawMaxCorrDeg, 0.0);
        nh.param<float>("lio_sam/ackermannPosWeight", ackermannPosWeight, 1.0);
        nh.param<float>("lio_sam/ackermannPosMaxCorr", ackermannPosMaxCorr, 0.0);
        nh.param<float>("lio_sam/ackermannRejectYawDeg", ackermannRejectYawDeg, 0.0);
        nh.param<float>("lio_sam/ackermannRejectPosM", ackermannRejectPosM, 0.0);
        nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<int>("lio_sam/slamQueueSize", slamQueueSize, 1);
        nh.param<bool>("lio_sam/useGpu", useGpu, false);
        nh.param<bool>("lio_sam/savePCD", savePCD, false);
        nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<bool>("lio_sam/localizationMode", localizationMode, false);
        nh.param<float>("lio_sam/initialPoseX", initialPoseX, 0.0);
        nh.param<float>("lio_sam/initialPoseY", initialPoseY, 0.0);
        nh.param<float>("lio_sam/initialPoseZ", initialPoseZ, 0.0);
        nh.param<float>("lio_sam/initialPoseRoll", initialPoseRoll, 0.0);
        nh.param<float>("lio_sam/initialPosePitch", initialPosePitch, 0.0);
        nh.param<float>("lio_sam/initialPoseYaw", initialPoseYaw, 0.0);

        nh.param<bool>("lio_sam/posesEnable", posesEnable, true);
        nh.param<std::string>("lio_sam/posesOutputFile", posesOutputFile, "");
        nh.param<double>("lio_sam/posesStartTime",  posesStartTime,  0.0);
        nh.param<double>("lio_sam/posesOriginX",   posesOriginX,   0.0);
        nh.param<double>("lio_sam/posesOriginY",   posesOriginY,   0.0);
        nh.param<double>("lio_sam/posesOriginZ",   posesOriginZ,   0.0);
        nh.param<double>("lio_sam/posesOriginYaw", posesOriginYaw, 0.0);
        nh.param<std::string>("lio_sam/posesOriginMode", posesOriginMode, "manual");
        nh.param<bool>("lio_sam/posesOriginLock", posesOriginLock, false);
        nh.param<bool>("lio_sam/posesOriginRefine", posesOriginRefine, true);
        nh.param<int>("lio_sam/posesOriginMinSamples", posesOriginMinSamples, 20);
        nh.param<double>("lio_sam/posesOriginMaxDt", posesOriginMaxDt, 0.50);
        nh.param<double>("lio_sam/posesOriginMinBaseline", posesOriginMinBaseline, 1.0);
        nh.param<double>("lio_sam/posesOriginRelPosNoise", posesOriginRelPosNoise, 0.25);
        nh.param<int>("lio_sam/posesSettleScans",  posesSettleScans, 20);
        nh.param<double>("lio_sam/posesInterpolateMaxGap", posesInterpolateMaxGap, 5.0);
        nh.param<double>("lio_sam/posesInterpolateMaxError", posesInterpolateMaxError, 0.10);
        nh.param<std::string>("lio_sam/posesCarmenStampTopic", posesCarmenStampTopic, "/carmen/scan_time_reference");
        nh.param<std::string>("lio_sam/posesAckermannTopic",  posesAckermannTopic,  "/ackermann/odom_raw");

        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
        nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
        if (nh.hasParam("lio_sam/extrinsicRot")) {
            ROS_FATAL("Parameter 'lio_sam/extrinsicRot' is deprecated. "
                      "Use 'lio_sam/extrinsicRPY' as [roll, pitch, yaw] in radians.");
            ros::shutdown();
        }

        vector<double> extRPYV;
        nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>{0.0, 0.0, 0.0});

        if (extRPYV.size() == 9) {
            ROS_FATAL("Parameter 'lio_sam/extrinsicRPY' must be [roll, pitch, yaw] (3 values), "
                      "not a 3x3 rotation matrix (9 values).");
            ros::shutdown();
        }
        if (extRPYV.size() != 3) {
            ROS_FATAL("Parameter 'lio_sam/extrinsicRPY' must have exactly 3 values [roll, pitch, yaw].");
            ros::shutdown();
        }

        nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
        extRot = rpyToRot(extRPYV[0], extRPYV[1], extRPYV[2]);
        extRPY = extRot;
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};


sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

// Le de volta um .scd salvo por saveSCD() (mesmo formato: um valor double por
// celula, delimitado por espaco, uma linha por linha da matriz).
Eigen::MatrixXd readSCD(std::string fileToOpen)
{
    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileToOpen);
    std::string matrixRowString;
    std::string matrixEntry;

    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString);
        while (getline(matrixRowStringStream, matrixEntry, ' '))
            matrixEntries.push_back(stod(matrixEntry));
        matrixRowNumber++;
    }
    if (matrixRowNumber == 0 || matrixEntries.empty())
        return Eigen::MatrixXd();
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}
