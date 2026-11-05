// Reconstruct SC-LIO-SAM localization clouds from aligned scans and poses.
#include "removert/runtime.hpp"
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
            RCLCPP_WARN_STREAM(lt_mapper::node->get_logger(), "Linha de pose invalida (esperava 12 ou 16 valores, veio " << vals.size() << "), pulando.");
            continue;
        }
        Eigen::Matrix4d pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vals.data(), 4, 4);
        sess.poses.push_back(pose);
    }

    if (sess.scan_paths.size() != sess.poses.size())
    {
        RCLCPP_ERROR_STREAM(lt_mapper::node->get_logger(), "Sessao " << scan_dir << ": " << sess.scan_paths.size()
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

int
run_reconstruction(int argc, char **argv)
{

    

    // --- parametros reaproveitados do removert ---
    std::string central_scan_dir, central_pose_path, query_scan_dir, query_pose_path;
    std::vector<double> extrinsic_vec;
    lt_mapper::parameter("removert.central_sess_scan_dir", central_scan_dir, "");
    lt_mapper::parameter("removert.central_sess_pose_path", central_pose_path, "");
    lt_mapper::parameter("removert.query_sess_scan_dir", query_scan_dir, "");
    lt_mapper::parameter("removert.query_sess_pose_path", query_pose_path, "");
    lt_mapper::parameter("removert.ExtrinsicLiDARtoPoseBase", extrinsic_vec, std::vector<double>());

    if (extrinsic_vec.size() != 16)
    {
        RCLCPP_ERROR(lt_mapper::node->get_logger(), "removert.ExtrinsicLiDARtoPoseBase precisa ter 16 valores (4x4 row-major). Abortando.");
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

    lt_mapper::parameter("map_reconstruct.output_dir", output_dir, "/tmp/loc_map/");
    lt_mapper::parameter("map_reconstruct.corner_leaf_size", corner_leaf, 0.2);
    lt_mapper::parameter("map_reconstruct.surf_leaf_size", surf_leaf, 0.4);
    lt_mapper::parameter("map_reconstruct.global_leaf_size", global_leaf, 0.3);
    lt_mapper::parameter("map_reconstruct.scan_predownsample_leaf", predownsample_leaf, 0.1);
    lt_mapper::parameter("map_reconstruct.feature_knn", knn, 18);
    lt_mapper::parameter("map_reconstruct.linearity_thresh", linearity_thresh, 0.6);
    lt_mapper::parameter("map_reconstruct.planarity_thresh", planarity_thresh, 0.6);
    lt_mapper::parameter("map_reconstruct.accumulate_flush_every", flush_every, 50);
    lt_mapper::parameter("map_reconstruct.use_precomputed_global_map", use_precomputed_global, true);
    lt_mapper::parameter("map_reconstruct.precomputed_global_map_path", precomputed_global_map_path, "");

    output_dir = lt_mapper::output_directory(output_dir,
        {central_scan_dir, query_scan_dir, central_pose_path, query_pose_path}).string();
    if (corner_leaf <= 0 || surf_leaf <= 0 || global_leaf <= 0 || predownsample_leaf <= 0 ||
        knn < 3 || flush_every < 1)
        throw std::runtime_error("Invalid reconstruction voxel, neighborhood or flush parameters");
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
        RCLCPP_ERROR(lt_mapper::node->get_logger(), "Nenhuma sessao carregada (confira removert/central_sess_scan_dir e query_sess_scan_dir).");
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
                RCLCPP_WARN_STREAM(lt_mapper::node->get_logger(), "Falha ao carregar " << sess.scan_paths[i] << ", pulando.");
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
                RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "Processados " << global_idx << " scans... corner=" << globalCorner->size()
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
            RCLCPP_WARN(lt_mapper::node->get_logger(), "use_precomputed_global_map=true mas nao consegui carregar precomputed_global_map_path; "
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

    RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "****************************************************");
    RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "Mapa de localizacao reconstruido em " << output_dir);
    RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "  cloudCorner: " << globalCorner->size() << " pts");
    RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "  cloudSurf:   " << globalSurf->size() << " pts");
    RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "  cloudGlobal: " << globalDense->size() << " pts");
    RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "  poses:       " << trajectory->size());
    RCLCPP_INFO_STREAM(lt_mapper::node->get_logger(), "****************************************************");

    return 0;
}
int
main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    lt_mapper::node = std::make_shared<rclcpp::Node>("reconstruct_feature_map");
    int result = 1;
    try
    {
        result = run_reconstruction(argc, argv);
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(lt_mapper::node->get_logger(), "%s", error.what());
    }
    lt_mapper::node.reset();
    rclcpp::shutdown();
    return result;
}
