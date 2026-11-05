#include "ltslam/RosParamServer.h"


RosParamServer::RosParamServer()

{
    lt_mapper::parameter("ltslam.sessions_dir", sessions_dir_, "/");
    lt_mapper::parameter("ltslam.central_sess_name", central_sess_name_, "01");
    lt_mapper::parameter("ltslam.query_sess_name", query_sess_name_, "02");

    lt_mapper::parameter("ltslam.save_directory", save_directory_, ""); // it means /.../home/LTslam/
    
    if (!fs::is_directory(fs::path(sessions_dir_) / central_sess_name_) ||
        !fs::is_directory(fs::path(sessions_dir_) / query_sess_name_) ||
        central_sess_name_ == query_sess_name_)
        throw std::runtime_error("Two distinct existing input sessions are required");
    auto output = lt_mapper::output_directory(save_directory_,
        {(fs::path(sessions_dir_) / central_sess_name_).string(),
         (fs::path(sessions_dir_) / query_sess_name_).string()});
    fs::remove_all(output);
    fs::create_directories(output);
    save_directory_ = output.string() + "/";

    lt_mapper::parameter("ltslam.is_display_debug_msgs", is_display_debug_msgs_, false);

    lt_mapper::parameter("ltslam.numberOfCores", numberOfCores, 4);

    lt_mapper::parameter("ltslam.kNumSCLoopsUpperBound", kNumSCLoopsUpperBound, 10);
    lt_mapper::parameter("ltslam.kNumRSLoopsUpperBound", kNumRSLoopsUpperBound, 10);

    lt_mapper::parameter("ltslam.loopFitnessScoreThreshold", loopFitnessScoreThreshold, 0.5);

    // chute inicial do anchor da sessão query (x, y, z em metros; yaw em graus)
    // dica: se a sessão query começa aproximadamente onde a central termina (percurso
    // de volta), um bom chute é a pose do ultimo no da sessão central + ~180 graus de yaw.
    lt_mapper::parameter("ltslam.query_anchor_init_x",       query_anchor_init_x_,       0.0);
    lt_mapper::parameter("ltslam.query_anchor_init_y",       query_anchor_init_y_,       0.0);
    lt_mapper::parameter("ltslam.query_anchor_init_z",       query_anchor_init_z_,       0.0);
    lt_mapper::parameter("ltslam.query_anchor_init_yaw_deg", query_anchor_init_yaw_deg_, 0.0);

    lt_mapper::parameter("ltslam.scDistThres",     sc_dist_thres_,     0.3);
    lt_mapper::parameter("ltslam.scNumCandidates",    sc_num_candidates_, 3);

    lt_mapper::parameter("ltslam.rsSearchRadius",  rs_search_radius_,  10.0);

    usleep(100);
} // ctor RosParamServer