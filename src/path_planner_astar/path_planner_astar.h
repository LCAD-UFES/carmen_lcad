#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <algorithm>
#include <car_model.h>

#include <boost/heap/fibonacci_heap.hpp>

#include <queue>
#include <list>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DELTA_T 0.01                      // Size of step for the ackerman Euler method

void
compute_astar_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config,
		carmen_obstacle_distance_mapper_map_message *distance_map);

typedef struct state_node
{
	carmen_ackerman_traj_point_t state;
	double f;                              // Total distance g + h
	double g;                                // Distance from start to current state
	double h;                                // Distance from current state to goal
	double angular_distance_to_goal;
	int is_open;
	int is_closed;
	int is_obstacle;
	int was_visited;
	state_node *parent;
} state_node, *state_node_p;

typedef struct discrete_pos_node
{
	int x;
	int y;
	int theta;
} discrete_pos_node;


class StateNodePtrComparator {
public:
	bool operator() (state_node *a, state_node *b) const
	{
		return (a->f > b->f);
	}
};
