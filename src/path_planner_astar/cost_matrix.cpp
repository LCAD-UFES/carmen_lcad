#include "path_planner_astar.h"
#define CLOSEST_CIRCLE_MIN_DIST 1.5
#define MIN_THETA_DIFF    0.0872665 // 5 0.0436332 // 2.5 // 0.261799 // 15 degrees
#define MIN_STEERING_DIFF 0.0872665
#define MIN_POS_DISTANCE  0.2 // the carmen grid map resolution

#define OPENCV 1
#define THETA_SIZE 1
#define ASTAR_GRID_RESOLUTION 1.0

carmen_point_t *final_goal = NULL;
carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
carmen_robot_ackerman_config_t robot_config;
carmen_obstacle_distance_mapper_map_message distance_map;
state_node_p ***astar_map;

using namespace std;
using namespace cv;
using namespace boost::heap;

Mat map_image;
