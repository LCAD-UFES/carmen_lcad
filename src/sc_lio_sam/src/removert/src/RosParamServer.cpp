#include "removert/RosParamServer.h"


RosParamServer::RosParamServer()

{
    lt_mapper::parameter("removert.isScanFileKITTIFormat", isScanFileKITTIFormat_, true);

    lt_mapper::parameter("removert.rimg_color_min", rimg_color_min_, 0.0);
    lt_mapper::parameter("removert.rimg_color_max", rimg_color_max_, 10.0);
    kRangeColorAxis = std::pair<float, float> {rimg_color_min_, rimg_color_max_}; // meter
    kRangeColorAxisForDiff = std::pair<float, float>{0.0, 0.5}; // meter 

    // fov 
    lt_mapper::parameter("removert.sequence_vfov", kVFOV, 50.0);
    lt_mapper::parameter("removert.sequence_hfov", kHFOV, 360.0);
    kFOV = std::pair<float, float>(kVFOV, kHFOV);

    // resolution 
    lt_mapper::parameter("removert.remove_resolution_list", remove_resolution_list_, std::vector<float>());
    lt_mapper::parameter("removert.revert_resolution_list", revert_resolution_list_, std::vector<float>());

    // knn 
    lt_mapper::parameter("removert.num_nn_points_within", kNumKnnPointsToCompare, 3);                 // using higher, more strict static
    lt_mapper::parameter("removert.dist_nn_points_within", kScanKnnAndMapKnnAvgDiffThreshold, 0.1); // using smaller, more strict static

    // sequcne system info 
    lt_mapper::parameter("removert.ExtrinsicLiDARtoPoseBase", kVecExtrinsicLiDARtoPoseBase, std::vector<double>());
    if (kVecExtrinsicLiDARtoPoseBase.size() != 16 ||
        !std::all_of(kVecExtrinsicLiDARtoPoseBase.begin(), kVecExtrinsicLiDARtoPoseBase.end(),
                     [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("removert.ExtrinsicLiDARtoPoseBase requires 16 finite values");
    kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);
    kSE3MatExtrinsicPoseBasetoLiDAR = kSE3MatExtrinsicLiDARtoPoseBase.inverse();

    // config: point cloud pre-processing
    lt_mapper::parameter("removert.downsample_voxel_size", kDownsampleVoxelSize, 0.05);

    // sessions' paths 
    lt_mapper::parameter("removert.central_sess_scan_dir", central_sess_scan_dir_, "/use/your/directory/having/*.bin");
    lt_mapper::parameter("removert.central_sess_pose_path", central_sess_pose_path_, "/use/your/path/having/pose.txt");
    lt_mapper::parameter("removert.query_sess_scan_dir", query_sess_scan_dir_, "/use/your/directory/having/*.bin");
    lt_mapper::parameter("removert.query_sess_pose_path", query_sess_pose_path_, "/use/your/path/having/pose.txt");


    // target scan index range (used in Removert.cpp)
    lt_mapper::parameter("removert.start_idx", start_idx_, 1);
    lt_mapper::parameter("removert.end_idx", end_idx_, 100);

    lt_mapper::parameter("removert.use_keyframe_gap", use_keyframe_gap_, true);
    lt_mapper::parameter("removert.use_keyframe_meter", use_keyframe_meter_, false);
    lt_mapper::parameter("removert.keyframe_gap", keyframe_gap_, 10);
    lt_mapper::parameter("removert.keyframe_meter", keyframe_gap_meter_, 2.0);

    lt_mapper::parameter("removert.repeat_removert_iter", repeat_removert_iter_, 1);

    // faster
    lt_mapper::parameter("removert.num_omp_cores", kNumOmpCores, 4);

    // save info
    lt_mapper::parameter("removert.saveMapPCD", kFlagSaveMapPointcloud, false);
    lt_mapper::parameter("removert.saveCleanScansPCD", kFlagSaveCleanScans, false);
    lt_mapper::parameter("removert.save_pcd_directory", save_pcd_directory_, "");


    if (keyframe_gap_ <= 0 || kNumOmpCores <= 0 || start_idx_ < 0 || end_idx_ < start_idx_ ||
        kDownsampleVoxelSize <= 0 || kVFOV <= 0 || kHFOV <= 0 || remove_resolution_list_.empty() ||
        revert_resolution_list_.empty())
        throw std::runtime_error("Invalid Removert sampling, resolution or geometry parameters");
    for (const auto &path : {central_sess_scan_dir_, query_sess_scan_dir_})
        if (!fs::is_directory(path))
            throw std::runtime_error("Missing scan directory: " + path);
    for (const auto &path : {central_sess_pose_path_, query_sess_pose_path_})
        if (!fs::is_regular_file(path))
            throw std::runtime_error("Missing pose file: " + path);
    save_pcd_directory_ = lt_mapper::output_directory(save_pcd_directory_,
        {central_sess_scan_dir_, query_sess_scan_dir_, central_sess_pose_path_, query_sess_pose_path_}).string();
} // ctor RosParamServer

