#ifndef GLOBAL_PLANNING_MESSAGES_H_
#define GLOBAL_PLANNING_MESSAGES_H_

#include <carmen/carmen.h>
#include "global.h"

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************** */
/*                        ROUTE PLANNER MESSAGES                    */
/****************************************************************** */

// Atencao!!! Ao adicionar um novo tipo de anotacao, incrementar o NUM_RDDF_ANNOTATION_TYPES
// (note que comecca de zero; assim, NUM_RDDF_ANNOTATION_TYPES eh igual ao ultimo codigo mais 1)

#define RDDF_ANNOTATION_TYPE_NONE                                              0
#define RDDF_ANNOTATION_TYPE_END_POINT_AREA                                    1
#define RDDF_ANNOTATION_TYPE_HUMAN_INTERVENTION                                2
#define RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK                                  3
#define RDDF_ANNOTATION_TYPE_STOP                                              4
#define RDDF_ANNOTATION_TYPE_BARRIER                                           5
#define RDDF_ANNOTATION_TYPE_BUMP                                              6
#define RDDF_ANNOTATION_TYPE_SPEED_LIMIT                                       7
#define RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT                                     8
#define RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN                                      9
#define RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP                               10
#define RDDF_ANNOTATION_TYPE_DYNAMIC                                          11
#define RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP                            12
#define RDDF_ANNOTATION_TYPE_PLACE_OF_INTEREST                                13
#define RDDF_ANNOTATION_TYPE_YIELD                                            14
#define RDDF_ANNOTATION_TYPE_PREDEFINED_ROUTE                                 15
#define RDDF_ANNOTATION_TYPE_AMV                                              16
#define RDDF_ANNOTATION_TYPE_NARROW_LANE                                      17
#define RDDF_ANNOTATION_TYPE_RETARDER_BRAKE                                   18
#define RDDF_ANNOTATION_TYPE_TURN_LEFT_INDICATOR                              19
#define RDDF_ANNOTATION_TYPE_TURN_RIGHT_INDICATOR                             20
#define RDDF_ANNOTATION_TYPE_GEAR                                             21
#define RDDF_ANNOTATION_TYPE_QUEUE                                            22
#define RDDF_ANNOTATION_TYPE_SINGLE_TRAFFIC_AREA                              23
#define RDDF_ANNOTATION_TYPE_MAP_SWITCH                                       24
#define RDDF_ANNOTATION_TYPE_ROAD_STEEPNESS                                   25
#define RDDF_ANNOTATION_TYPE_SPRINKLING_AREA                                  26
#define RDDF_ANNOTATION_TYPE_GPS_CORRECTION                                   27
#define RDDF_ANNOTATION_TYPE_REDUCE_SENSITIVITY                               28
#define RDDF_ANNOTATION_TYPE_GRAPH_CORRECTION                                 29
#define RDDF_ANNOTATION_TYPE_FRENET_CHANGE_NUM_PATHS                          30
#define RDDF_ANNOTATION_TYPE_DISABLE_CHECK_LOCALIZATION                       31
#define RDDF_ANNOTATION_TYPE_CHANGE_LIDAR_RANGE_MAX                           32
#define RDDF_ANNOTATION_TYPE_MOVING_OBJECTS                                   33
#define RDDF_ANNOTATION_TYPE_SET_COLLISION_GEOMETRY_MODEL                     34
#define RDDF_ANNOTATION_TYPE_LOCALIZE_ACKERMAN_REINITIALIZE                   35
#define RDDF_ANNOTATION_TYPE_CHANGE_LOCALIZE_THRESHOLD                        36
#define RDDF_ANNOTATION_TYPE_LOCALIZE_ACKERMAN_LIDAR_USED_IN_GLOBALPOS        37
#define RDDF_ANNOTATION_TYPE_MOVE_AWAY_FROM_OBSTACLE_DISTANCE                 38
#define RDDF_ANNOTATION_TYPE_CHANGE_BEHAVIOR_SELECTOR_SAFE_DISTANCE           39
#define RDDF_ANNOTATION_TYPE_CHANGE_LOCALIZE_LEVEL_WEIGHT                     40
#define RDDF_ANNOTATION_TYPE_FRENET_CHANGE_SAFE_DISTANCE                      41
#define RDDF_ANNOTATION_TYPE_OBSTACLE_AVOIDER_CHANGE_SAFE_DISTANCE            42
#define RDDF_ANNOTATION_TYPE_RAILROAD_CROSSING                                43
#define RDDF_ANNOTATION_TYPE_MIN_MAX_FORCE_OBSTACLE_HEIGHT                    44
#define RDDF_ANNOTATION_TYPE_ENABLE_FORCE_OBSTACLE_HEIGHT                     45
#define RDDF_ANNOTATION_TYPE_DISABLE_FORCE_OBSTACLE_HEIGHT                    46
#define RDDF_ANNOTATION_TYPE_CHANGE_LOCALIZE_LEVEL_SAFE_HEIGHT                47
#define RDDF_ANNOTATION_TYPE_CHANGE_LOCALIZE_LEVEL_UNSAFE_HEIGHT              48
#define RDDF_ANNOTATION_TYPE_LOCALIZE_RESET_PARAMETERS                        49
#define RDDF_ANNOTATION_TYPE_BLOCK_PATH_ON_MAP                                50
#define RDDF_ANNOTATION_TYPE_CHANGE_OBSTACLE_AVOIDER_POTENTIAL_COLLISION_EXTENSION 51
#define RDDF_ANNOTATION_TYPE_TURN_EMERGENCY_LIGHTS                            52

// Atencao!!! Ao adicionar um novo tipo de CODE de anotacao, incrementar o NUM_RDDF_ANNOTATION_CODES
// (note que comecca de zero; assim, NUM_RDDF_ANNOTATION_CODES eh igual ao ultimo codigo mais 1)
#define RDDF_ANNOTATION_CODE_NONE                                              0

#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_0                                            1
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_5                                            2
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_10                                           3
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_15                                           4
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_20                                           5
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_30                                           6
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_40                                           7
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_60                                           8
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_80                                           9
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_100                                         10
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_110                                         11

#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED                                       12
#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN                                     13
#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW                                    14
#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF                                       15
#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT                                 16
#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT                                  17

#define RDDF_ANNOTATION_CODE_DYNAMIC_STOP                                            18

#define RDDF_ANNOTATION_CODE_PEDESTRIAN_TRACK_BUSY                                   19

#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_GO_STRAIGHT                                20
#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF                                        21

#define RDDF_ANNOTATION_CODE_PREDEFINED_ROUTE_CYCLIC                                 22

#define RDDF_ANNOTATION_CODE_BARRIER_25_25                                           23
#define RDDF_ANNOTATION_CODE_BARRIER_25_20                                           24
#define RDDF_ANNOTATION_CODE_BARRIER_25_15                                           25
#define RDDF_ANNOTATION_CODE_BARRIER_25_10                                           26
#define RDDF_ANNOTATION_CODE_BARRIER_20_25                                           27
#define RDDF_ANNOTATION_CODE_BARRIER_20_20                                           28
#define RDDF_ANNOTATION_CODE_BARRIER_20_15                                           29
#define RDDF_ANNOTATION_CODE_BARRIER_20_10                                           30
#define RDDF_ANNOTATION_CODE_BARRIER_15_25                                           31
#define RDDF_ANNOTATION_CODE_BARRIER_15_20                                           32
#define RDDF_ANNOTATION_CODE_BARRIER_15_15                                           33
#define RDDF_ANNOTATION_CODE_BARRIER_15_10                                           34
#define RDDF_ANNOTATION_CODE_BARRIER_10_25                                           35
#define RDDF_ANNOTATION_CODE_BARRIER_10_20                                           36
#define RDDF_ANNOTATION_CODE_BARRIER_10_15                                           37
#define RDDF_ANNOTATION_CODE_BARRIER_10_10                                           38

#define RDDF_ANNOTATION_CODE_NARROW_LANE_BEGIN                                       39
#define RDDF_ANNOTATION_CODE_NARROW_LANE_END                                         40

#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_ON                                       41
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_OFF                                      42

#define RDDF_ANNOTATION_CODE_TURN_LEFT_INDICATOR_ON                                  43
#define RDDF_ANNOTATION_CODE_TURN_LEFT_INDICATOR_OFF                                 44

#define RDDF_ANNOTATION_CODE_TURN_RIGHT_INDICATOR_ON                                 45
#define RDDF_ANNOTATION_CODE_TURN_RIGHT_INDICATOR_OFF                                46

#define RDDF_ANNOTATION_CODE_GEAR_1                                                  47
#define RDDF_ANNOTATION_CODE_GEAR_2                                                  48
#define RDDF_ANNOTATION_CODE_GEAR_3                                                  49
#define RDDF_ANNOTATION_CODE_GEAR_4                                                  50
#define RDDF_ANNOTATION_CODE_GEAR_5                                                  51
#define RDDF_ANNOTATION_CODE_GEAR_6                                                  52
#define RDDF_ANNOTATION_CODE_GEAR_7                                                  53
#define RDDF_ANNOTATION_CODE_GEAR_8                                                  54
#define RDDF_ANNOTATION_CODE_GEAR_9                                                  55
#define RDDF_ANNOTATION_CODE_GEAR_10                                                 56
#define RDDF_ANNOTATION_CODE_GEAR_11                                                 57
#define RDDF_ANNOTATION_CODE_GEAR_12                                                 58
#define RDDF_ANNOTATION_CODE_GEAR_13                                                 59
#define RDDF_ANNOTATION_CODE_GEAR_14                                                 60
#define RDDF_ANNOTATION_CODE_GEAR_15                                                 61
#define RDDF_ANNOTATION_CODE_GEAR_16                                                 62
#define RDDF_ANNOTATION_CODE_GEAR_17                                                 63
#define RDDF_ANNOTATION_CODE_GEAR_18                                                 64
#define RDDF_ANNOTATION_CODE_GEAR_19                                                 65
#define RDDF_ANNOTATION_CODE_GEAR_20                                                 66
#define RDDF_ANNOTATION_CODE_GEAR_21                                                 67
#define RDDF_ANNOTATION_CODE_GEAR_22                                                 68
#define RDDF_ANNOTATION_CODE_GEAR_23                                                 69
#define RDDF_ANNOTATION_CODE_GEAR_24                                                 70
#define RDDF_ANNOTATION_CODE_GEAR_25                                                 71
#define RDDF_ANNOTATION_CODE_GEAR_26                                                 72
#define RDDF_ANNOTATION_CODE_GEAR_27                                                 73
#define RDDF_ANNOTATION_CODE_GEAR_28                                                 74
#define RDDF_ANNOTATION_CODE_GEAR_29                                                 75
#define RDDF_ANNOTATION_CODE_GEAR_30                                                 76
#define RDDF_ANNOTATION_CODE_GEAR_31                                                 77
#define RDDF_ANNOTATION_CODE_GEAR_32                                                 78
#define RDDF_ANNOTATION_CODE_GEAR_RW_1                                               79
#define RDDF_ANNOTATION_CODE_GEAR_RW_2                                               80
#define RDDF_ANNOTATION_CODE_GEAR_NEUTRAL                                            81

#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_25                                          82

#define RDDF_ANNOTATION_CODE_QUEUE_BUSY                                              83

#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_1                                  84
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_2                                  85
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_3                                  86
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_4                                  87
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_5                                  88
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_6                                  89
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_7                                  90
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_8                                  91
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_9                                  92
#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_10                                 93

#define RDDF_ANNOTATION_CODE_MAP_SWITCH_LEVEL0                                       94
#define RDDF_ANNOTATION_CODE_MAP_SWITCH_LEVEL1                                       95

#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_NEGATIVE_LEVEL5                          96
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_NEGATIVE_LEVEL4                          97
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_NEGATIVE_LEVEL3                          98
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_NEGATIVE_LEVEL2                          99
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_NEGATIVE_LEVEL1                         100
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_PLANE                                   101
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_POSITIVE_LEVEL1                         102
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_POSITIVE_LEVEL2                         103
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_POSITIVE_LEVEL3                         104
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_POSITIVE_LEVEL4                         105
#define RDDF_ANNOTATION_CODE_ROAD_STEEPNESS_POSITIVE_LEVEL5                         106
#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_CUSTOM                                     107
#define RDDF_ANNOTATION_CODE_GPS_CORRECTION_BEGIN                                   110
#define RDDF_ANNOTATION_CODE_GPS_CORRECTION_END                                     111

#define RDDF_ANNOTATION_CODE_JET_SPRINKLING_AREA_INIT                               108
#define RDDF_ANNOTATION_CODE_JET_SPRINKLING_AREA_END                                109
#define RDDF_ANNOTATION_CODE_SHOWER_SPRINKLING_AREA_INIT                            112
#define RDDF_ANNOTATION_CODE_SHOWER_SPRINKLING_AREA_END                             113
#define RDDF_ANNOTATION_CODE_LEFT_SIDE_SPRINKLING_AREA_INIT                         114
#define RDDF_ANNOTATION_CODE_LEFT_SIDE_SPRINKLING_AREA_END                          115
#define RDDF_ANNOTATION_CODE_RIGHT_SIDE_SPRINKLING_AREA_INIT                        116
#define RDDF_ANNOTATION_CODE_RIGHT_SIDE_SPRINKLING_AREA_END                         117
#define RDDF_ANNOTATION_CODE_BACK_SPRINKLING_AREA_INIT                              118
#define RDDF_ANNOTATION_CODE_BACK_SPRINKLING_AREA_END                               119
#define RDDF_ANNOTATION_CODE_LOWER_SPRINKLING_AREA_INIT                             120
#define RDDF_ANNOTATION_CODE_LOWER_SPRINKLING_AREA_END                              121
#define RDDF_ANNOTATION_CODE_OUTLET_SPRINKLING_AREA_INIT                            122
#define RDDF_ANNOTATION_CODE_OUTLET_SPRINKLING_AREA_END                             123
#define RDDF_ANNOTATION_CODE_LID_SPRINKLING_AREA_INIT                               124
#define RDDF_ANNOTATION_CODE_LID_SPRINKLING_AREA_END                                125

#define RDDF_ANNOTATION_CODE_BEGIN_REDUCE_SENSITIVITY                               126
#define RDDF_ANNOTATION_CODE_END_REDUCE_SENSITIVITY                                 127

#define RDDF_ANNOTATION_CODE_FRENET_CHANGE_NUM_PATHS                                128

#define RDDF_ANNOTATION_CODE_DISABLE_CHECK_LOCALIZATION_BEGIN                       129
#define RDDF_ANNOTATION_CODE_DISABLE_CHECK_LOCALIZATION_END                         130

#define RDDF_ANNOTATION_CODE_YIELD_BUSY                                             131

#define RDDF_ANNOTATION_CODE_BEGIN_CHANGE_RANGE_MAX_FACTOR                          132
#define RDDF_ANNOTATION_CODE_END_CHANGE_RANGE_MAX_FACTOR                            133
#define RDDF_ANNOTATION_CODE_MOVING_OBJECTS_DISABLE                                 134
#define RDDF_ANNOTATION_CODE_MOVING_OBJECTS_ENABLE                                  135

#define RDDF_ANNOTATION_CODE_GRAPH_CORRECTION_BEGIN                                 136
#define RDDF_ANNOTATION_CODE_GRAPH_CORRECTION_END                                   137

#define RDDF_ANNOTATION_CODE_CHANGE_LOCALIZE_THRESHOLD_COSSINE_DIST                 138
#define RDDF_ANNOTATION_CODE_CHANGE_LOCALIZE_THRESHOLD_INSTANTANT_POINTS            139
#define RDDF_ANNOTATION_CODE_CHANGE_LOCALIZE_THRESHOLD_N_FEATURES                   140

#define RDDF_ANNOTATION_CODE_YIELD_ALL_OBJECTS                                      141
#define RDDF_ANNOTATION_CODE_STOP_BUSY                                              142

#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_COSSINE_DIST_THRESHOLD       143
#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_INSTANTANT_POINTS_THRESHOLD  144
#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_N_FEATURES_THRESHOLD         145
#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_ALL_THRESHOLD                146
#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_SAFE_HEIGHT                  147
#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_UNSAFE_HEIGHT                148
#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_ALL_HEIGHT                   149
#define RDDF_ANNOTATION_CODE_LOCALIZE_RESET_PARAMETERS_MAP_WEIGHT                   150
#define RDDF_ANNOTATION_CODE_MAPPER_BLOCK_PATH_ON_MAP_RED_CELL                      151
#define RDDF_ANNOTATION_CODE_MAPPER_BLOCK_PATH_ON_MAP_GREEN_CELL                    152
#define RDDF_ANNOTATION_CODE_MAPPER_BLOCK_PATH_ON_MAP_OCCUPIED_CELL                 153
#define RDDF_ANNOTATION_CODE_MAPPER_BLOCK_PATH_ON_MAP_FREE_CELL                     154

#define RDDF_ANNOTATION_CODE_RAILROAD_CROSSING_BUSY                                 155

#define RDDF_ANNOTATION_CODE_TURN_EMERGENCY_LIGHTS_ON                               156
#define RDDF_ANNOTATION_CODE_TURN_EMERGENCY_LIGHTS_OFF                              157

#define MIN_DISTANCE_TO_CONSIDER_CROSSWALK                                          1.0




#define ROUTE_PLANNER_GET_LANE_LEFT_WIDTH(traffic_restrictions, new_traffic_restrictions) (new_traffic_restrictions == 1) ? (((double) (traffic_restrictions & 0x3ff)) * 0.1) : (((double) (traffic_restrictions & 0x3f)) * 0.1)
#define ROUTE_PLANNER_GET_LANE_RIGHT_WIDTH(traffic_restrictions, new_traffic_restrictions) (new_traffic_restrictions == 1) ? (((double) ((traffic_restrictions & (0x3ff << 10)) >> 10)) * 0.1) : (((double) ((traffic_restrictions & (0x3f << 6)) >> 6)) * 0.1)
#define ROUTE_PLANNER_SET_LANE_LEFT_WIDTH(traffic_restrictions, lane_width) ((traffic_restrictions & ~0x3ff) | ((int) (lane_width * 10.0) & 0x3ff))
#define ROUTE_PLANNER_SET_LANE_RIGHT_WIDTH(traffic_restrictions, lane_width) ((traffic_restrictions & ~(0x3ff << 10)) | (((int) (lane_width * 10.0) & 0x3ff) << 10))



#define print_offroad_planner_request(x) ( \
    (x == NO_REQUEST)? "NO_REQUEST": \
    (x == PLAN_FROM_POSE_TO_LANE)? "PLAN_FROM_POSE_TO_LANE": \
    (x == PLAN_FROM_LANE_TO_FINAL_POSE)? "PLAN_FROM_LANE_TO_FINAL_POSE": \
    (x == PLAN_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT)? "PLAN_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT": \
    (x == PLAN_FROM_CURRENT_POSE_TO_ENGAGE_POSE)? "PLAN_FROM_CURRENT_POSE_TO_ENGAGE_POSE": \
    (x == PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE)? "PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE": "" )



#define print_route_planner_state(x) ( \
    (x == IDLE)? "IDLE": \
    (x == IN_RDDF_MODE)? "IN_RDDF_MODE": \
    (x == COULD_NOT_COMPUTE_THE_ROUTE)? "COULD_NOT_COMPUTE_THE_ROUTE": \
    (x == PUBLISHING_ROUTE)? "PUBLISHING_ROUTE": \
    (x == COMPUTING_ROUTE)? "COMPUTING_ROUTE": \
    (x == RECOMPUTING_ROUTE)? "RECOMPUTING_ROUTE": \
    (x == ROUTE_RECOMPUTED)? "ROUTE_RECOMPUTED": \
    (x == PLANNING_FROM_CURRENT_POSE_TO_FINAL_POSE)? "PLANNING_FROM_CURRENT_POSE_TO_FINAL_POSE": \
    (x == PLANNING_FROM_POSE_TO_LANE)? "PLANNING_FROM_POSE_TO_LANE": \
    (x == PLANNING_FROM_LANE_TO_FINAL_POSE)? "PLANNING_FROM_LANE_TO_FINAL_POSE": \
    (x == EXECUTING_OFFROAD_PLAN)? "EXECUTING_OFFROAD_PLAN": \
    (x == END_OF_PATH_REACHED_IN_OFFROAD_PLAN)? "END_OF_PATH_REACHED_IN_OFFROAD_PLAN": \
    (x == END_OF_PATH_REACHED_IN_ROUTE)? "END_OF_PATH_REACHED_IN_ROUTE": \
    (x == PLANNING_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT)? "PLANNING_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT": \
    (x == IN_RECTLINEAR_ROUTE_SEGMENT)? "IN_RECTLINEAR_ROUTE_SEGMENT": \
    (x == OFFROAD_PLANNER_ERROR)? "OFFROAD_PLANNER_ERROR": "")


/*************************************/
/***********Route Messages************/

/*
typedef enum
{
    NO_REQUEST,
    PLAN_FROM_POSE_TO_LANE,
    PLAN_FROM_LANE_TO_FINAL_POSE,
    PLAN_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT,
    PLAN_FROM_CURRENT_POSE_TO_RECTLINEAR_ROUTE_SEGMENT,
    PLAN_FROM_CURRENT_POSE_TO_ENGAGE_POSE,
    PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE
} offroad_planner_request_t;



typedef enum ROUTE_PLANNER_STATE
{
    IDLE,
    IN_RDDF_MODE,
    COULD_NOT_COMPUTE_THE_ROUTE,
    PUBLISHING_ROUTE,
    COMPUTING_ROUTE,
    RECOMPUTING_ROUTE,
    ROUTE_RECOMPUTED,
    PLANNING_FROM_CURRENT_POSE_TO_FINAL_POSE,
    PLANNING_FROM_CURRENT_POSE_TO_ENGAGE_POSE,
    PLANNING_FROM_POSE_TO_LANE,
    PLANNING_FROM_LANE_TO_FINAL_POSE,
    EXECUTING_OFFROAD_PLAN,
    END_OF_PATH_REACHED_IN_OFFROAD_PLAN,
    END_OF_PATH_REACHED_IN_ROUTE,
    PLANNING_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT,
    IN_RECTLINEAR_ROUTE_SEGMENT,
    OFFROAD_PLANNER_ERROR
} carmen_route_planner_state_t;



typedef struct
{
    carmen_robot_and_trailers_traj_point_t pose;
    int node_id;
    int lane_id;
} carmen_route_planner_route_t;

typedef struct
{
    int node_id;
    int index_of_node_in_current_lane;
    int target_node_index_in_nearby_lane;
    int target_lane_id;
} carmen_route_planner_junction_t;

typedef struct
{
    int number_of_poses;
    int number_of_poses_back;
    carmen_robot_and_trailers_traj_point_t *poses;
    carmen_robot_and_trailers_traj_point_t *poses_back;
    int *annotations;
    int *annotations_codes;

    int number_of_nearby_lanes;
    int *nearby_lanes_indexes;                               // O ponto em nearby_lanes onde comecca cada lane.
    int *nearby_lanes_sizes;                                 // O tamanho de cada lane.
    int *nearby_lanes_ids;                                   // Cada id eh um codigo que identifica uma lane unicamente.
    int nearby_lanes_size;                                   // Igual ao numero de poses de todas as lanes somado.
    carmen_robot_and_trailers_traj_point_t *nearby_lanes;     // Todas as lanes (number_of_nearby_lanes), uma apos a outra. A primeira lane eh sempre a rota e sempre deve ter id = 0, jah que ela eh uma composicao de lanes do grafo
    int *traffic_restrictions;                               // LANE_LEFT_WIDTH | LANE_RIGHT_WIDTH | LEFT_MARKING | RIGHT_MARKING | LEVEL | YIELD | BIFURCATION
                                                             //     6 bits      |      6 bits      |  3 bits enum |  3 bits enum  | 2 bits| 1 bit |   1 bit

    int *nearby_lanes_merges_indexes;                        // Size == number_of_nearby_lanes. O ponto em nearby_lanes_merges onde começam os merges de cada lane.
    int *nearby_lanes_merges_sizes;                          // Size == number_of_nearby_lanes. O número de merges de cada lane.
    int nearby_lanes_merges_size;                            // Igual ao numero de merges de todas as lanes somado.
    carmen_route_planner_junction_t *nearby_lanes_merges;             // Size == nearby_lanes_merges_size. Todos os merges, um atras do outro.

    int *nearby_lanes_forks_indexes;                         // Size == number_of_nearby_lanes. O ponto em nearby_lanes_forks onde começam os forks de cada lane.
    int *nearby_lanes_forks_sizes;                           // Size == number_of_nearby_lanes. O número de forks de cada lane.
    int nearby_lanes_forks_size;                             // Igual ao numero de forks de todas as lanes somado.
    carmen_route_planner_junction_t *nearby_lanes_forks;      // Size == nearby_lanes_forks_size. Todos os forks, um atras do outro.

    int *nearby_lanes_crossroads_indexes;                    // Size == number_of_nearby_lanes. O ponto em nearby_crossroads onde começam os cruzamentos de cada lane.
    int *nearby_lanes_crossroads_sizes;                      // Size == number_of_nearby_lanes. O número de cruzamentos de cada lane.
    int nearby_lanes_crossroads_size;                        // Igual ao numero de cruzamentos de todas as lanes somado.
    carmen_route_planner_junction_t *nearby_lanes_crossroads; // Size == nearby_lanes_crossroads_size. Todos os cruzamenttos, um atras do outro.

    int *nearby_lanes_node_ids;                              // Size == nearby_lanes_size. Ids dos nós (poses) de todas as lanes.

    int route_size;
    carmen_route_planner_route_t *route;                      // Size == route_size. Vetor com as poses, o id das poses e o id das lanes
    



    //  Uma network com tres lanes com tamanhos 5, 3 e 6 poses teria:
    //  number_of_nearby_lanes = 3
    //    nearby_lanes_indexes -> 0, 5, 8
    //    nearby_lanes_sizes -> 5, 3, 6
    //    nearby_lanes_size = 5+3+6 = 14
    //    nearby_lanes (p_lane_pose) -> p_0_0, p_0_1, p_0_2, p_0_3, p_0_4, p_1_0, p_1_1, p_1_2, p_2_0, p_2_1, p_2_2, p_2_3, p_2_4, p_2_5

    offroad_planner_request_t offroad_planner_request;
    carmen_route_planner_state_t route_planner_state;

    int new_traffic_restrictions;

    double remaining_distance;
    double remaining_time;

    int *hash_to_nearby_lane; // um array com tamanho number_of_poses, na qual em cada posição contém o index associado de poses até nearby_lanes. Ou seja, para pegar o traffic_restrictions da poses[0], faz: traffic_restrictions[hash_to_nearby_lanes[0]]

    double timestamp;
    char *host;
} carmen_route_planner_road_network_message;

*/


#define        CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME        "carmen_route_planner_road_network_message"



#define        CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_NAME        "carmen_route_planner_predefined_route_message"
#define        CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_FMT         "{string, int, double, string}"




#define        CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME        "carmen_route_planner_destination_message"
#define        CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_FMT         "{string, {double, double, double}, double, string}"




#define        CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME    "carmen_route_planner_pallet_and_destination_message"
#define        CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_FMT     "{string, {double, double, double}, string, {double, double, double}, double, string}"




#define        CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME        "carmen_route_planner_route_status_change"
#define        CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_FMT         "{int, int, double, string}"


typedef struct
{
    int number_of_locked_node_ids;
    int *locked_graph_node_ids;
    int status;            /* lock:1 , unlock:0 */
    char *graph_id;
    double timestamp;
    char *host;
} carmen_route_planner_locked_roads_message;

#define        CARMEN_ROUTE_PLANNER_LOCKED_ROADS_NAME        "carmen_route_planner_locked_roads_message"
#define        CARMEN_ROUTE_PLANNER_LOCKED_ROADS_FMT         "{int, <int:1>, int, string, double, string}"

typedef struct
{
    int number_of_nodes;
    int *graph_node_ids;
    double *weights;			
    double timestamp;
    char *host;
} carmen_route_planner_change_weight_node_message;

#define		CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_NAME		"carmen_route_planner_change_weight_node_message"
#define		CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_FMT		"{int, <int:1>, <double:1>, double, string}"





#define        CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME                "carmen_route_planner_route_reload"
#define        CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_FMT                 "{string, double, string}"


#ifndef EDGE_TYPE_
#define EDGE_TYPE_
// These structs are defined in a C++ library (road_network_generator_utils.h) but this is a C library



typedef struct
{
    int id;
    carmen_position_t start;
    carmen_position_t end;
    edge_t edge;
    int status;        /* enabled:1 , disabled:0 */
} route_t;

#endif




#define        CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME        "carmen_route_planner_route_list_request"
#define        CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_FMT         "{{double, double}, double, int, <{int, {double, double}, {double, double}, {int, int, int, int, double}, int}:3>, double, string}"

#define        CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME       "carmen_route_planner_route_list_response"
#define        CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_FMT        CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_FMT





#define        CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_NAME        "carmen_route_planner_node_ids_ahead_request"
#define        CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_FMT         "{{double, double, double}, double, int, <int:3>, double, string}"

#define        CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_NAME       "carmen_route_planner_node_ids_ahead_response"
#define        CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_FMT        CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_FMT




/*************************************/
/********Annotations Messages*********/

#ifndef __CARMEN_RDDF_MESSAGES_H__
#define __CARMEN_RDDF_MESSAGES_H__

typedef struct
{
    int number_of_poses;
    carmen_robot_and_trailers_traj_point_t *poses;
    double timestamp;
    char *host;
} carmen_rddf_waypoints_around_end_point_message;

#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME "carmen_rddf_waypoints_around_end_point_message"
#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT  "{int, <{double,double,double,int, [double:5],double,double}:1>, double, string}"


typedef struct
{
    int traffic_sign_state;
    double traffic_sign_data;
    double timestamp;
    char *host;
} carmen_rddf_traffic_sign_message;

#define CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME "carmen_rddf_traffic_sign_message"
#define CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_FMT  "{int,double,double,string}"


typedef struct
{
    crud_t action;
    carmen_annotation_t old_annotation;
    carmen_annotation_t new_annotation;
    double timestamp;
    char *host;
} carmen_rddf_update_annotation_message;

#define CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_NAME "carmen_rddf_update_annotation_message"
#define CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_FMT  "{int,{{double,double,double},double,string,int,int,int},{{double,double,double},double,string,int,int,int},double,string}"


// TODO: update this message to use carmen_annotation_t.
typedef struct
{
    carmen_vector_3D_t annotation_point;
    double annotation_orientation;
    char *annotation_description;
    int annotation_type;
    int annotation_code;
    int annotation_id;
    double timestamp;
    char *host;
} carmen_rddf_add_annotation_message;

#define CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME "carmen_rddf_add_annotation_message"
#define CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_FMT  "{{double,double,double},double,string,int,int,int,double,string}"

typedef carmen_rddf_add_annotation_message carmen_rddf_dynamic_annotation_message;

#define CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME "carmen_rddf_dynamic_annotation_message"
#define CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT  "{{double,double,double},double,string,int,int,int,double,string}"

    typedef struct
    {
        int num_annotations;
        carmen_annotation_t *annotations;
        double timestamp;
        char *host;
    } carmen_rddf_annotation_message;

    #define CARMEN_RDDF_ANNOTATION_MESSAGE_NAME "carmen_rddf_annotation_message"
    #define CARMEN_RDDF_ANNOTATION_MESSAGE_FMT  "{int, <{{double,double,double},double,string,int,int,int,int,<double:7>,<double:7>}:1>,double,string}"

	
	#define CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME "carmen_route_planner_initial_annotation_message"
	#define CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_FMT "{int, <{{double,double,double},double,string,int,int,int,int,<double:7>,<double:7>}:1>,double,string}"


    typedef struct
    {
        int number_of_poses;
        int number_of_poses_back;
        carmen_robot_and_trailers_traj_point_t *poses;
        carmen_robot_and_trailers_traj_point_t *poses_back;
        int *annotations;
        int *annotations_codes;
        double timestamp;
        char *host;
    } carmen_rddf_road_profile_message;

    #define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME "carmen_rddf_road_profile_message"
    #define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT  "{int, int, <{double, double, double, int, [double:5], double, double}:1>, <{double, double, double, int, [double:5], double, double}:2>, <int:1>, <int:1>, double, string}"


    typedef struct
    {
        int half_meters_to_final_goal;    // number of waypoints to consider near end_point (point)
        carmen_robot_and_trailers_pose_t point;
        double timestamp;
        char *host;
    } carmen_rddf_end_point_message;

    #define CARMEN_RDDF_END_POINT_MESSAGE_NAME "carmen_rddf_end_point_message"
    #define CARMEN_RDDF_END_POINT_MESSAGE_FMT  "{int, {double, double, double, int, [double:5]}, double, string}"


	typedef struct
	{
		int release; // 1 for release 0 for lock
        carmen_point_t point;
		double timestamp;
        char *host;
	} carmen_release_annotation_message;
	#define CARMEN_RELEASE_ANNOTATION_MESSAGE_NAME "carmen_release_annotation_message"
	#define CARMEN_RELEASE_ANNOTATION_MESSAGE_FMT  "{int, {double, double, double}, double, string}"

	typedef struct
	{
		int activate; // 1 para ativar;
		double timestamp;
        char *host;
	} carmen_offroad_activate_message;
	#define CARMEN_OFFROAD_ACTIVATE_MESSAGE_NAME "carmen_offroad_activate_message"
	#define CARMEN_OFFROAD_ACTIVATE_MESSAGE_FMT  "{int, double, string}"

#endif

//************************* Obstacle Avoider **************************/

typedef struct
{
    carmen_point_t left_near_obstacle;
    carmen_point_t right_near_obstacle;
    double timestamp;
    char *host;
} carmen_robot_ackerman_road_velocity_control_message;

#define      CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_NAME         "carmen_robot_ackerman_road_velocity_control"
#define      CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_FMT          "{{double, double, double}, {double,double,double}, double, string}"


typedef struct
{
    int robot_will_hit_obstacle;
    double timestamp;
    char *host;
} carmen_obstacle_avoider_robot_will_hit_obstacle_message;

#define      CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME         "carmen_obstacle_avoider_robot_hit_obstacle"
#define      CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_FMT          "{int, double, string}"


//************************* mpp/Model Predictive **************************/

typedef struct
{
    carmen_robot_and_trailers_traj_point_t *plan;
    int       plan_length;
    double timestamp;
    char  *host;
} carmen_model_predictive_planner_motion_plan_message;

#define CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME "carmen_model_predictive_planner_motion_plan"
#define CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_FMT  "{<{double, double, double, int, [double:5], double, double}:2>,int,double,string}"

/****************************************************************** */
/*                    AND ROUTE PLANNER MESSAGES                    */
/****************************************************************** */



#ifdef __cplusplus
}
#endif

#endif /* GLOBAL_PLANNING_MESSAGES_H_ */
