#pragma once

#include "ltslam/utility.h"

class RosNodeHandle
{
public:
    ros::NodeHandle nh_super;
}; // class: RosNodeHandle


class RosParamServer: public RosNodeHandle
{
public:
    ros::NodeHandle & nh;

    std::string sessions_dir_;
    std::string central_sess_name_;
    std::string query_sess_name_;

    std::string save_directory_;
    
    bool is_display_debug_msgs_;

    int kNumSCLoopsUpperBound;
    int kNumRSLoopsUpperBound;
    int numberOfCores;

    float loopFitnessScoreThreshold;

    // ---- chute inicial grosseiro do anchor da sessão "query" (não-base) ----
    // Usado quando não existe NENHUM loop inter-sessão (SC) para "puxar" o anchor
    // para longe da origem. Sem isso, o anchor fica preso perto de poseOrigin (0,0,0),
    // que coincide com a origem local da sessão central -> os dois mapas "empilham"
    // na origem em vez de ficarem posicionados corretamente (ver initTrajectoryByAnchoring).
    double query_anchor_init_x_;
    double query_anchor_init_y_;
    double query_anchor_init_z_;
    double query_anchor_init_yaw_deg_;

    // ---- tuning do Scan Context (ajuda em loops de longa distância / sentido contrário) ----
    double sc_dist_thres_;
    int    sc_num_candidates_;

    // ---- raio (m) da busca RS (radius search) por candidatos entre sessões ----
    double rs_search_radius_;

public:
    RosParamServer();

}; // class: RosParamServer