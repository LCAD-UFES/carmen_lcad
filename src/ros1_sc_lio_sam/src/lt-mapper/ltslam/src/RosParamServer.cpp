#include "ltslam/RosParamServer.h"


RosParamServer::RosParamServer()
: nh(nh_super)
{
    nh.param<std::string>("ltslam/sessions_dir", sessions_dir_, "/");
    nh.param<std::string>("ltslam/central_sess_name", central_sess_name_, "01");
    nh.param<std::string>("ltslam/query_sess_name", query_sess_name_, "02");

    nh.param<std::string>("ltslam/save_directory", save_directory_, "/LTslam/"); // it means /.../home/LTslam/
    
    int unused = system((std::string("exec rm -r ") + save_directory_).c_str());
    unused = system((std::string("mkdir -p ") + save_directory_).c_str());

    nh.param<bool>("ltslam/is_display_debug_msgs", is_display_debug_msgs_, false);

    nh.param<int>("ltslam/numberOfCores", numberOfCores, 4);

    nh.param<int>("ltslam/kNumSCLoopsUpperBound", kNumSCLoopsUpperBound, 10);
    nh.param<int>("ltslam/kNumRSLoopsUpperBound", kNumRSLoopsUpperBound, 10);

    nh.param<float>("ltslam/loopFitnessScoreThreshold", loopFitnessScoreThreshold, 0.5);

    // chute inicial do anchor da sessão query (x, y, z em metros; yaw em graus)
    // dica: se a sessão query começa aproximadamente onde a central termina (percurso
    // de volta), um bom chute é a pose do ultimo no da sessão central + ~180 graus de yaw.
    nh.param<double>("ltslam/query_anchor_init_x",       query_anchor_init_x_,       0.0);
    nh.param<double>("ltslam/query_anchor_init_y",       query_anchor_init_y_,       0.0);
    nh.param<double>("ltslam/query_anchor_init_z",       query_anchor_init_z_,       0.0);
    nh.param<double>("ltslam/query_anchor_init_yaw_deg", query_anchor_init_yaw_deg_, 0.0);

    nh.param<double>("ltslam/scDistThres",     sc_dist_thres_,     0.3);
    nh.param<int>("ltslam/scNumCandidates",    sc_num_candidates_, 3);

    nh.param<double>("ltslam/rsSearchRadius",  rs_search_radius_,  10.0);

    usleep(100);
} // ctor RosParamServer