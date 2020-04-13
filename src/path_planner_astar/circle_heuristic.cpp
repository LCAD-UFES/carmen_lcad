#include "circle_heuristic.h"

#include <queue>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/collision_detection.h>

using namespace std;
using namespace cv;

#define MIN_OVERLAP_FACTOR 0.5	          // if two circles overlaps more than this factor then they are considered connected
#define MAX_OVERLAP_FACTOR 0.1	          // if two circles overlaps more than this factor then they are considered the same
#define RGOAL 3.5				          // ???
#define DELTA_T 0.01                      // Size of step for the ackerman Euler method
#define ALFA 1                	          // Weight of nearest circle radius for step_size
#define BETA 1               	          // Weight of nearest circle path distance to goal for step_size
#define MAX_STEP_SIZE 2.0		          // max step size in seconds
#define KMIN 0.0125 			          // Step rate multiplier
//#define MIN_THETA_DIFF 0.24		          // 15 degree from stehs_planner.h


class CircleNodeComparator {
public:
	bool operator() (circle_node *a, circle_node *b)
	{
		float w1 = 1.0;
		float w2 = 5.0;
		return ((w1 * a->h - w2 * a->radius) > (w1 * b->h - w2 * b->radius)); // Uses radius as a choice criterion (the bigger the better)
		//return (a->h > b->h);             // The default c++ stl is a max heap, so wee need to invert here
	}
};


double
get_obstacle_distance(double x, double y, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	if( NULL == distance_map)
		exit(1);

    carmen_point_t p;
    p.x = x;
    p.y = y;

    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, distance_map));
}


circle_node*
create_circle_node(double cx, double cy, double radius, double g, double h, circle_node *parent)
{
	circle_node *new_circle = (circle_node*) malloc(sizeof(circle_node));
	new_circle->x = cx;
	new_circle->y = cy;
	new_circle->radius = radius;
	new_circle->g = g;      // (distance) cost from start to current state
	new_circle->h = h;       // the (distance) cost from current state to goal
	new_circle->parent = parent;

	return new_circle;
}


bool
circle_overlap(circle_node *current, circle_node *circle, double overlap_factor)
{
	double distance;
	double greater, smaller;

	if (current->radius < circle->radius)
	{
		greater = current->radius;
		smaller = circle->radius;
	}
	else
	{
		greater = circle->radius;
		smaller = current->radius;
	}

	distance = sqrt(pow(current->x - circle->x, 2) + pow(current->y - circle->y, 2));

	return ((distance - greater) < (smaller * overlap_factor));
}


bool
circle_node_exist(circle_node *current, std::vector<circle_node*> &closed_set)
{
    std::vector<circle_node*>::iterator it = closed_set.begin();
    std::vector<circle_node*>::iterator end = closed_set.end();

    while (it != end)
    {
        //if (current->parent != (*it) && current->parent != (*it)->parent && current->circle.Overlap((*it)->circle, MAX_OVERLAP_FACTOR))
    	if (current->parent != (*it) && current->parent != (*it)->parent && circle_overlap(current, *it, MAX_OVERLAP_FACTOR))
            return true;

        it++;
    }

    return false;
}


circle_node
closest_circle_new(carmen_ackerman_traj_point_t state, std::vector<circle_node> &circle_path)
{
	double min_dist = DBL_MAX;
	double dist;
	int indice = 0;

	for (unsigned int i = 0; i < circle_path.size(); i++)
	{
	  dist = DIST2D(state, circle_path[i]);
	  if (dist < min_dist)
	  {
	    min_dist = dist;
	    indice = i;
	  }
	}

	if (indice == 0)
	  indice = 1;
	return (circle_path[indice - 1]);
}


std::vector<circle_node>
build_circle_path_new(circle_node *node, double initial_cost)
{
	std::vector<circle_node> circle_path;
	int i = 0;
	double cost = initial_cost;
    while (node != NULL)
    {
		node->h = cost;
		if (node->parent != NULL)
			cost += DIST2D(*node, *(node->parent));
			circle_path.push_back(*node);
			node = node->parent;
			i++;
    }
    return(circle_path);
}

double
distance(double ax, double ay, double bx, double by)
{
    double dx = ax - bx;
    double dy = ay - by;
    return sqrt(dx * dx + dy * dy);
}


void
draw_circle_on_map_img(double x, double y, double r, carmen_map_config_t config, cv::Mat *map_image)
{
	int img_x = round((double) (x - config.x_origin) / config.resolution);
	int img_y = round((double) (y - config.y_origin) / config.resolution);
	int img_r = round(r / config.resolution);
	circle(*map_image, Point(img_x, config.y_size - 1 - img_y), img_r, 0, 1, 8);
	imshow("Obstacle Map", *map_image);
	waitKey(1);
}


void
draw_circle_path(std::vector<circle_node> circle_path, carmen_map_config_t config, cv::Mat *map_image)
{
	for (unsigned int i = 0; i < circle_path.size(); i++)
	{
		draw_circle_on_map_img(circle_path[i].x, circle_path[i].y, circle_path[i].radius, config, map_image);
	}

}

void
expand_circle(circle_node *current, priority_queue<circle_node*, std::vector<circle_node*>, CircleNodeComparator> &open_set, carmen_point_t goal, double robot_width,
		carmen_obstacle_distance_mapper_map_message *distance_map)
{
	// Considering a 2m radius parent circle that involves the car we want 32 children circles
    unsigned int children_amount = /*16;*/(unsigned int) (16.0 * current->radius);

    double displacement_angle = 2.0 * M_PI / (double) children_amount;
    double cos_displacement_angle = cos(displacement_angle);
    double sin_displacement_angle = sin(displacement_angle);
    double t;

    double px = current->x;
    double py = current->y;
    double pr = current->radius;
    double pg = current->g;

    double x = pr;
    double y = 0.0;

    for (unsigned int i = 0; i < children_amount; i++)
    {
        double nx = x + px;
        double ny = y + py;

        double nearst_obstacle_distance = get_obstacle_distance(nx, ny, distance_map);

        if (nearst_obstacle_distance > robot_width * 0.5)		// TODO this should be a defined parameter
        {
            // TODO verificar se é possível remover filhos com overlap
            open_set.push(create_circle_node(nx, ny, nearst_obstacle_distance, pg + pr, distance(nx, ny, goal.x, goal.y), current));

            //draw_circle_on_map_img(nx, ny, nearst_obstacle_distance, distance_map->config);
            //draw_point_on_map_img(nx, ny, distance_map->config);
        }

        // Apply the rotation matrix
        t = x;
        x = cos_displacement_angle * x - sin_displacement_angle * y;
        y = sin_displacement_angle * t + cos_displacement_angle * y;
    }
}


std::vector<circle_node>
space_exploration(circle_node *start_node, circle_node *goal_node, carmen_point_t goal, double robot_width, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	std::vector<circle_node> temp_circle_path;
    priority_queue<circle_node*, std::vector<circle_node*>, CircleNodeComparator> open_set;
    std::vector<circle_node*> closed_set;
    open_set.push(start_node); // Added

    while (!open_set.empty())
    {
    	circle_node *current = open_set.top();                   // Get the circle witch is the closest to the goal node
        open_set.pop();

        if (goal_node->g < (current->g + current->h))
        {
	    double initial_cost = DIST2D(*goal_node, *(goal_node->parent));
            temp_circle_path = build_circle_path_new(goal_node->parent, initial_cost);

            while(!open_set.empty())
            {
            	circle_node *tmp = open_set.top();
                open_set.pop();
                delete tmp;
            }
            break;
        }
        else if (!circle_node_exist(current, closed_set))
        {
        	expand_circle(current, open_set, goal, robot_width, distance_map);

            if (circle_overlap(current, goal_node, MIN_OVERLAP_FACTOR))
            {
                if ((current->g + current->h) < goal_node->g)
                {
                    goal_node->g = (current->g + current->h);
                    goal_node->parent = current;
                }
            }
            closed_set.push_back(current);
        }
        else
            delete current;
    }
    while(!closed_set.empty())                // Only to clean the closed_set
    {
    	circle_node *tmp = closed_set.back();
        closed_set.pop_back();
        delete tmp;
    }
    return (temp_circle_path);
}


std::vector<circle_node>
circle_exploration(carmen_point_t *robot_pose, carmen_point_t *goal, carmen_obstacle_distance_mapper_map_message *distance_map, cv::Mat *map_image, int draw)
{
	circle_node *start_circle, *goal_circle;
	std::vector<circle_node> circle_path;

	start_circle = create_circle_node(robot_pose->x, robot_pose->y, get_obstacle_distance(robot_pose->x, robot_pose->y, distance_map), 0.0,
			DIST2D_P(robot_pose, goal), NULL);

	goal_circle = create_circle_node(goal->x, goal->y, get_obstacle_distance(goal->x, goal->y, distance_map), DBL_MAX, 0.0, NULL);

	circle_path = space_exploration(start_circle, goal_circle, *goal, 4.0, distance_map);     // TODO Read car width from carmen.ini investigate how to avoid passing the goal as parameter

	circle_path.insert(circle_path.begin(), *goal_circle);

	if (draw)
		draw_circle_path(circle_path, distance_map->config, map_image);

	// TODO Checar se precisa atualizar os custos g e h do circle path

	return (circle_path);
}
