#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


typedef struct circle_node
{
	double x;
	double y;
	double radius;
	double g;                             // Distance from start to current state
	double h;                             // Distance from current state to goal state
	circle_node *parent;
} circle_node;


std::vector<circle_node>
circle_exploration(carmen_point_t *robot_pose, carmen_point_t *goal, carmen_obstacle_distance_mapper_map_message *distance_map, cv::Mat *map_image, int draw);


circle_node
closest_circle_new(carmen_ackerman_traj_point_t state, std::vector<circle_node> &circle_path);
