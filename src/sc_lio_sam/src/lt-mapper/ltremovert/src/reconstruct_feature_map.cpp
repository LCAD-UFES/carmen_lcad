// reconstruct_feature_map.cpp
//
// Reconstroi cloudCorner.pcd / cloudSurf.pcd / cloudGlobal.pcd / trajectory.pcd / transformations.pcd
// a partir dos scans locais + poses usados pelo lt-mapper (Removerter), pra alimentar o
// loadMap() do SC-LIO-SAM em modo localizacao (mapOptmization.cpp).
//
// Por que isso existe:
//   - loadMap() exige EXATAMENTE esses 5 arquivos, no formato que o proprio SC-LIO-SAM
//     salva no fim do mapping (saveMapService).
//   - O merge multissessao do lt-mapper (Removerter) nao gera esses arquivos: ele so
//     produz nuvens densas (updated_map.pcd, map_static/, etc), sem separacao corner/surf.
//   - Os scans salvos pelo SC-LIO-SAM (Scans/*.pcd, lidos por Session::loadKeyframes) sao
//     XYZI puro, ja sem o campo "ring" -> nao da pra refazer a extracao de curvatura por
//     ring igual ao featureExtraction.cpp original. Em vez disso, este node classifica
//     corner/surf via analise geometrica local (PCA / autovalores da covizinhanca), que
//     funciona em nuvem nao-organizada.
//
// Reaproveita os MESMOS parametros de YAML que o Removerter ja usa (namespace "removert/"),
// entao nao precisa de config nova alem do bloco "map_reconstruct/" abaixo.
//
// --------------------------------------------------------------------------------------
// Parametros novos (adicione no mesmo yaml do removert, ou em outro carregado junto):
//
//   map_reconstruct/output_dir:            "/dados/sc_lio_sam_output/mapa_localizacao/"
//   map_reconstruct/corner_leaf_size:       0.2
//   map_reconstruct/surf_leaf_size:         0.4
//   map_reconstruct/global_leaf_size:       0.3
//   map_reconstruct/scan_predownsample_leaf: 0.1
//   map_reconstruct/feature_knn:            18
//   map_reconstruct/linearity_thresh:       0.6      # (l1-l2)/l1 >  isso  -> corner
//   map_reconstruct/planarity_thresh:       0.6      # (l2-l3)/l1 >  isso  -> surf
//   map_reconstruct/accumulate_flush_every: 50       # downsample parcial a cada N scans (memoria)
//   map_reconstruct/use_precomputed_global_map: true
//   map_reconstruct/precomputed_global_map_path: "/dados/sc_lio_sam_output/mapa_final_limpo/updated_map.pcd"
//
// Reaproveitados do Removerter (removert/...):
//   central_sess_scan_dir, central_sess_pose_path,
//   query_sess_scan_dir,   query_sess_pose_path,
//   ExtrinsicLiDARtoPoseBase
//
// --------------------------------------------------------------------------------------
// Build: adiciona no CMakeLists.txt do pacote removert (ou do sc-lio-sam):
//
//   find_package(OpenMP REQUIRED)
//   add_executable(reconstruct_feature_map src/reconstruct_feature_map.cpp)
//   target_link_libraries(reconstruct_feature_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
//
// Run:
//   rosrun <seu_pacote> reconstruct_feature_map _config:=/caminho/do/removert_config.yaml
//   (ou via roslaunch carregando o yaml com rosparam antes)
// --------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

// ---- mesmos tipos usados no removert e no mapOptmization -------------------------------
typedef pcl::PointXYZI PointType;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                   (float, x, x)(float, y, y)
                                   (float, z, z)(float, intensity, intensity)
                                   (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT PointTypePose;

// ---- struct simples pra carregar uma sessao (scan paths + poses) -----------------------
struct SessionData
{
    std::vector<std::string> scan_paths;
    std::vector<Eigen::Matrix4d> poses;
};

std::vector<double> splitPoseLine(const std::string &line, char delim)
{
    std::vector<double> parsed;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, delim))
    {
        if (!tok.empty())
            parsed.push_back(std::stod(tok));
    }
    return parsed;
}

SessionData loadSession(const std::string &scan_dir, const std::string &pose_path)
{
    SessionData sess;

    for (auto &entry : fs::directory_iterator(scan_dir))
        sess.scan_paths.push_back(entry.path().string());
    std::sort(sess.scan_paths.begin(), sess.scan_paths.end());

    std::ifstream pose_file(pose_path);
    std::string line;
    while (std::getline(pose_file, line))
    {
        auto vals = splitPoseLine(line, ' ');
        if (vals.size() == 12)
            vals.insert(vals.end(), {0.0, 0.0, 0.0, 1.0});
        if (vals.size() != 16)
        {
            ROS_WARN_STREAM("Linha de pose invalida (esperava 12 ou 16 valores, veio " << vals.size() << "), pulando.");
            continue;
        }
        Eigen::Matrix4d pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vals.data(), 4, 4);
        sess.poses.push_back(pose);
    }

    if (sess.scan_paths.size() != sess.poses.size())
    {
        ROS_ERROR_STREAM("Sessao " << scan_dir << ": " << sess.scan_paths.size()
                          << " scans mas " << sess.poses.size() << " poses. Abortando essa sessao.");
        sess.scan_paths.clear();
        sess.poses.clear();
    }
    return sess;
}

// ---- classificacao geometrica local (substitui a curvatura por ring do LOAM) -----------
// Pra cada ponto: pega k vizinhos, calcula autovalores da covariancia (l1>=l2>=l3>=0).
// linearidade = (l1-l2)/l1  -> alto = ponto tipo "aresta" (edge/corner)
// planaridade = (l2-l3)/l1  -> alto = ponto tipo "superficie" (surf)
void classifyLocalFeatures(const pcl::PointCloud<PointType>::Ptr &scan,
                            int knn, float linearity_thresh, float planarity_thresh,
                            pcl::PointCloud<PointType>::Ptr &edge_out,
                            pcl::PointCloud<PointType>::Ptr &surf_out)
{
    if (scan->size() < (size_t)(knn + 1))
        return;

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(scan);

    std::vector<uint8_t> label(scan->size(), 0); // 0=nada, 1=edge, 2=surf

    const int n = (int)scan->size();
#pragma omp parallel for num_threads(4)
    for (int i = 0; i < n; ++i)
    {
        std::vector<int> idx(knn);
        std::vector<float> dist(knn);
        if (kdtree.nearestKSearch(scan->points[i], knn, idx, dist) < knn)
            continue;

        Eigen::Vector3f centroid(0, 0, 0);
        for (int id : idx)
            centroid += scan->points[id].getVector3fMap();
        centroid /= float(knn);

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int id : idx)
        {
            Eigen::Vector3f d = scan->points[id].getVector3fMap() - centroid;
            cov += d * d.transpose();
        }
        cov /= float(knn);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        Eigen::Vector3f eigvals = solver.eigenvalues(); // ascendente: l3 <= l2 <= l1
        float l3 = std::max(eigvals(0), 1e-9f);
        float l2 = eigvals(1);
        float l1 = std::max(eigvals(2), 1e-9f);

        float linearity = (l1 - l2) / l1;
        float planarity = (l2 - l3) / l1;

        if (linearity > linearity_thresh)
            label[i] = 1;
        else if (planarity > planarity_thresh)
            label[i] = 2;
    }

    for (int i = 0; i < n; ++i)
    {
        if (label[i] == 1)
            edge_out->push_back(scan->points[i]);
        else if (label[i] == 2)
            surf_out->push_back(scan->points[i]);
    }
}

void voxelDownsample(pcl::PointCloud<PointType>::Ptr &cloud, float leaf)
{
    if (cloud->empty())
        return;
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setInputCloud(cloud);
    pcl::PointCloud<PointType>::Ptr out(new pcl::PointCloud<PointType>());
    vg.filter(*out);
    cloud = out;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reconstruct_feature_map");
    ros::NodeHandle nh;

    // --- parametros reaproveitados do removert ---
    std::string central_scan_dir, central_pose_path, query_scan_dir, query_pose_path;
    std::vector<double> extrinsic_vec;
    nh.param<std::string>("removert/central_sess_scan_dir", central_scan_dir, "");
    nh.param<std::string>("removert/central_sess_pose_path", central_pose_path, "");
    nh.param<std::string>("removert/query_sess_scan_dir", query_scan_dir, "");
    nh.param<std::string>("removert/query_sess_pose_path", query_pose_path, "");
    nh.param<std::vector<double>>("removert/ExtrinsicLiDARtoPoseBase", extrinsic_vec, std::vector<double>());

    if (extrinsic_vec.size() != 16)
    {
        ROS_ERROR("removert/ExtrinsicLiDARtoPoseBase precisa ter 16 valores (4x4 row-major). Abortando.");
        return 1;
    }
    Eigen::Matrix4d lidar2base_d = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extrinsic_vec.data(), 4, 4);
    Eigen::Matrix4f lidar2base = lidar2base_d.cast<float>();

    // --- parametros novos deste node ---
    std::string output_dir, precomputed_global_map_path;
    float corner_leaf, surf_leaf, global_leaf, predownsample_leaf;
    int knn, flush_every;
    float linearity_thresh, planarity_thresh;
    bool use_precomputed_global;

    nh.param<std::string>("map_reconstruct/output_dir", output_dir, "/tmp/loc_map/");
    nh.param<float>("map_reconstruct/corner_leaf_size", corner_leaf, 0.2);
    nh.param<float>("map_reconstruct/surf_leaf_size", surf_leaf, 0.4);
    nh.param<float>("map_reconstruct/global_leaf_size", global_leaf, 0.3);
    nh.param<float>("map_reconstruct/scan_predownsample_leaf", predownsample_leaf, 0.1);
    nh.param<int>("map_reconstruct/feature_knn", knn, 18);
    nh.param<float>("map_reconstruct/linearity_thresh", linearity_thresh, 0.6);
    nh.param<float>("map_reconstruct/planarity_thresh", planarity_thresh, 0.6);
    nh.param<int>("map_reconstruct/accumulate_flush_every", flush_every, 50);
    nh.param<bool>("map_reconstruct/use_precomputed_global_map", use_precomputed_global, true);
    nh.param<std::string>("map_reconstruct/precomputed_global_map_path", precomputed_global_map_path, "");

    if (output_dir.back() != '/')
        output_dir += "/";
    fs::create_directories(output_dir);

    // --- carrega as duas sessoes ---
    std::vector<SessionData> sessions;
    if (!central_scan_dir.empty())
        sessions.push_back(loadSession(central_scan_dir, central_pose_path));
    if (!query_scan_dir.empty())
        sessions.push_back(loadSession(query_scan_dir, query_pose_path));

    if (sessions.empty())
    {
        ROS_ERROR("Nenhuma sessao carregada (confira removert/central_sess_scan_dir e query_sess_scan_dir).");
        return 1;
    }

    pcl::PointCloud<PointType>::Ptr globalCorner(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurf(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalDense(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr trajectory(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointTypePose>::Ptr transformations(new pcl::PointCloud<PointTypePose>());

    int global_idx = 0;
    int scans_since_flush = 0;

    for (auto &sess : sessions)
    {
        for (size_t i = 0; i < sess.scan_paths.size(); ++i)
        {
            pcl::PointCloud<PointType>::Ptr scan_local(new pcl::PointCloud<PointType>());
            if (pcl::io::loadPCDFile<PointType>(sess.scan_paths[i], *scan_local) == -1)
            {
                ROS_WARN_STREAM("Falha ao carregar " << sess.scan_paths[i] << ", pulando.");
                continue;
            }

            // pre-downsample pra manter o KNN rapido
            voxelDownsample(scan_local, predownsample_leaf);

            // classifica edge/surf no frame local (invariante a transformacao rigida)
            pcl::PointCloud<PointType>::Ptr edge_local(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr surf_local(new pcl::PointCloud<PointType>());
            classifyLocalFeatures(scan_local, knn, linearity_thresh, planarity_thresh, edge_local, surf_local);

            // transforma pro frame global: lidar2base primeiro, depois pose do scan (mesma ordem do utility.cpp local2global)
            Eigen::Matrix4f pose_f = sess.poses[i].cast<float>();

            pcl::PointCloud<PointType>::Ptr edge_global(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr surf_global(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr dense_global(new pcl::PointCloud<PointType>());

            pcl::transformPointCloud(*edge_local, *edge_global, lidar2base);
            pcl::transformPointCloud(*edge_global, *edge_global, pose_f);

            pcl::transformPointCloud(*surf_local, *surf_global, lidar2base);
            pcl::transformPointCloud(*surf_global, *surf_global, pose_f);

            if (!use_precomputed_global)
            {
                pcl::transformPointCloud(*scan_local, *dense_global, lidar2base);
                pcl::transformPointCloud(*dense_global, *dense_global, pose_f);
                *globalDense += *dense_global;
            }

            *globalCorner += *edge_global;
            *globalSurf += *surf_global;

            // pose da trajetoria / transformations
            PointType p3d;
            p3d.x = pose_f(0, 3);
            p3d.y = pose_f(1, 3);
            p3d.z = pose_f(2, 3);
            p3d.intensity = float(global_idx);
            trajectory->push_back(p3d);

            Eigen::Affine3f affine(pose_f);
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(affine, x, y, z, roll, pitch, yaw);

            PointTypePose p6d;
            p6d.x = x; p6d.y = y; p6d.z = z;
            p6d.intensity = float(global_idx);
            p6d.roll = roll; p6d.pitch = pitch; p6d.yaw = yaw;
            p6d.time = double(global_idx);
            transformations->push_back(p6d);

            global_idx++;
            scans_since_flush++;

            if (scans_since_flush >= flush_every)
            {
                voxelDownsample(globalCorner, corner_leaf);
                voxelDownsample(globalSurf, surf_leaf);
                if (!use_precomputed_global)
                    voxelDownsample(globalDense, global_leaf);
                scans_since_flush = 0;
            }

            if (global_idx % 20 == 0)
                ROS_INFO_STREAM("Processados " << global_idx << " scans... corner=" << globalCorner->size()
                                 << " surf=" << globalSurf->size());
        }
    }

    // downsample final
    voxelDownsample(globalCorner, corner_leaf);
    voxelDownsample(globalSurf, surf_leaf);

    if (use_precomputed_global)
    {
        if (precomputed_global_map_path.empty() ||
            pcl::io::loadPCDFile<PointType>(precomputed_global_map_path, *globalDense) == -1)
        {
            ROS_WARN("use_precomputed_global_map=true mas nao consegui carregar precomputed_global_map_path; "
                     "cloudGlobal.pcd sera reconstruido a partir dos scans mesmo.");
            use_precomputed_global = false;
        }
    }
    if (!use_precomputed_global)
        voxelDownsample(globalDense, global_leaf);

    // --- salva no formato que o loadMap() do SC-LIO-SAM espera ---
    pcl::io::savePCDFileBinary(output_dir + "cloudCorner.pcd", *globalCorner);
    pcl::io::savePCDFileBinary(output_dir + "cloudSurf.pcd", *globalSurf);
    pcl::io::savePCDFileBinary(output_dir + "cloudGlobal.pcd", *globalDense);
    pcl::io::savePCDFileBinary(output_dir + "trajectory.pcd", *trajectory);
    pcl::io::savePCDFileBinary(output_dir + "transformations.pcd", *transformations);

    ROS_INFO_STREAM("****************************************************");
    ROS_INFO_STREAM("Mapa de localizacao reconstruido em " << output_dir);
    ROS_INFO_STREAM("  cloudCorner: " << globalCorner->size() << " pts");
    ROS_INFO_STREAM("  cloudSurf:   " << globalSurf->size() << " pts");
    ROS_INFO_STREAM("  cloudGlobal: " << globalDense->size() << " pts");
    ROS_INFO_STREAM("  poses:       " << trajectory->size());
    ROS_INFO_STREAM("****************************************************");

    return 0;
}