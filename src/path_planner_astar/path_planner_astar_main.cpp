#include "path_planner_astar.h"
#define CLOSEST_CIRCLE_MIN_DIST 1.5
#define MIN_THETA_DIFF    0.0872665 // 5 0.0436332 // 2.5 // 0.261799 // 15 degrees
#define MIN_STEERING_DIFF 0.0872665
#define MIN_POS_DISTANCE  0.2 // the carmen grid map resolution

#define OPENCV 1
#define THETA_SIZE 1
#define ASTAR_GRID_RESOLUTION 1.0

#define CIRCLE_HEURISTIC 0

carmen_point_t *final_goal = NULL;
carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
carmen_robot_ackerman_config_t robot_config;
carmen_obstacle_distance_mapper_map_message distance_map;
state_node_p ***astar_map;

using namespace std;
using namespace cv;
using namespace boost::heap;

Mat map_image;

// For circle heuristic

#define MIN_OVERLAP_FACTOR 0.5	          // if two circles overlaps more than this factor then they are considered connected
#define MAX_OVERLAP_FACTOR 0.1	          // if two circles overlaps more than this factor then they are considered the same
#define RGOAL 3.5				          // ???
#define DELTA_T 0.01                      // Size of step for the ackerman Euler method
#define ALFA 1                	          // Weight of nearest circle radius for step_size
#define BETA 1               	          // Weight of nearest circle path distance to goal for step_size
#define MAX_STEP_SIZE 2.0		          // max step size in seconds
#define KMIN 0.0125 			          // Step rate multiplier
//#define MIN_THETA_DIFF 0.24		          // 15 degree from stehs_planner.h

typedef struct circle_node
{
	double x;
	double y;
	double radius;
	double g;                             // Distance from start to current state
	double h;                             // Distance from current state to goal state
	circle_node *parent;
} circle_node;

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

/////////////////////////////////////////////////////////////////////////////

void
display_map(carmen_obstacle_distance_mapper_map_message *distance_map)
{
	if (distance_map->complete_x_offset == NULL)
		return;

	unsigned int width = distance_map->config.x_size;
	unsigned int height = distance_map->config.y_size;
	unsigned int size = width * height;
	unsigned char map[3*size];

	for (unsigned int i = 0; i < size; ++i)
	{
		unsigned int row = (height - 1) - i % height;
		unsigned int col = i / height;
		unsigned int index = row * width + col;

		if (0.0 == distance_map->complete_x_offset[i] && 0.0 == distance_map->complete_y_offset[i])
		{
			map[index*3] = 0;
			map[index*3+1] = 0;
			map[index*3+2] = 0;
		}
		else
		{
			map[index*3] = 255;
			map[index*3+1] = 255;
			map[index*3+2] = 255;
		}
	}
    Mat img(width, height, CV_8UC3, map);
    map_image = img.clone();
    imshow("Obstacle Map", map_image);
    waitKey(1);
}


void
draw_circle_on_map_img(double x, double y, double r, carmen_map_config_t config)
{
	int img_x = round((double) (x - config.x_origin) / config.resolution);
	int img_y = round((double) (y - config.y_origin) / config.resolution);
	int img_r = round(r / config.resolution);
	circle(map_image, Point(img_x, config.y_size - 1 - img_y), img_r, 0, 1, 8);
	imshow("Obstacle Map", map_image);
	waitKey(1);
	//usleep(500000);
}


void
draw_point_on_map_img(double x, double y, carmen_map_config_t config, int r, int g, int b, int size=1)
{
	int img_x = (double) (x - config.x_origin) / config.resolution;
	int img_y = (double) (y - config.y_origin) / config.resolution;
	circle(map_image, Point(img_x, config.y_size - 1 - img_y), size, Scalar(b, g, r), -1, 8);
	imshow("Obstacle Map", map_image);
	waitKey(1);
	//usleep(10000);
}


void
draw_state_path(vector<state_node> state_path, carmen_map_config_t config)
{
	for (unsigned int i = 0; i < state_path.size(); i++)
	{
		draw_point_on_map_img(state_path[i].state.x, state_path[i].state.y, config, 0, 0, 0);
	}
	//usleep(500000);
}


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
}


double
carmen_compute_abs_steering_angular_distance(double phi_1, double phi_2)
{
	return (abs(phi_1 - phi_2));
}


state_node*
create_state_node(double x, double y, double theta, double v, double phi, double g, double h, state_node *parent)
{
	state_node *new_state = (state_node*) malloc(sizeof(state_node));
	new_state->state.x = x;
	new_state->state.y = y;
	new_state->state.theta = theta;
	new_state->state.v = v;
	new_state->state.phi = phi;
	new_state->f = g+h;
	new_state->g = g;
	new_state->h = h;
	new_state->parent = parent;

	return (new_state);
}


std::vector<state_node>
build_state_path(state_node *node)
{
	std::vector<state_node> state_path;
	int i = 0;

    while (node != NULL)
    {
        state_path.push_back(*node);
        node = node->parent;
        i++;
    }

    return(state_path);
}


static void
get_pos_message(carmen_point_t robot_pose)
{
	if (final_goal != NULL)
	{
		printf("Pos_message_handler: %lf %lf %lf\n", robot_pose.x, robot_pose.y, robot_pose.theta);
		compute_astar_path(&robot_pose, final_goal, robot_config , &distance_map);
		final_goal = NULL;
	}
}


double
obstacle_distance(double x, double y, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	if( NULL == distance_map)
		exit(1);

    carmen_point_t p;
    p.x = x;
    p.y = y;

    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, distance_map));
}


void
alloc_astar_map(carmen_obstacle_distance_mapper_map_message *distance_map)
{
	int i, j, z;
	int theta_size = THETA_SIZE;
	int x_size = round(distance_map->config.x_size / ASTAR_GRID_RESOLUTION + 1);
	int y_size = round(distance_map->config.y_size / ASTAR_GRID_RESOLUTION + 1);
	printf("sizemap = %d %d \n", x_size, y_size);
	astar_map = (state_node_p ***)calloc(x_size, sizeof(state_node_p**));
	carmen_test_alloc(astar_map);

	for (i = 0; i < x_size; i++)
	{
		astar_map[i] = (state_node_p **)calloc(y_size, sizeof(state_node_p*));
		carmen_test_alloc(astar_map[i]);

		for (j = 0; j < y_size; j++)
		{
			astar_map[i][j] = (state_node_p*)calloc(theta_size, sizeof(state_node_p));
			carmen_test_alloc(astar_map[i][j]);
		}
	}
}

void
clear_astar_map(carmen_obstacle_distance_mapper_map_message *distance_map)
{
	int i, j, k, z;
	int x_size = round(distance_map->config.x_size / ASTAR_GRID_RESOLUTION + 1);
	int y_size = round(distance_map->config.y_size / ASTAR_GRID_RESOLUTION + 1);

	for (i = 0; i < x_size; i++)
		for (j = 0; j < y_size; j++)
			for (k = 0; k < THETA_SIZE; k++)
				astar_map[i][j][k] = NULL;
}


int
get_astar_map_theta(double theta)
{
	theta = theta < 0 ? (2 * M_PI + theta) : theta;
	int resolution = (int) round(360/THETA_SIZE);

	return  (int)round((carmen_radians_to_degrees(theta) / resolution)) % (int)round(360 / resolution);
}


int
get_astar_map_x(double x, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	return round((double) (x - distance_map->config.x_origin) / distance_map->config.resolution);
}


int
get_astar_map_y(double y, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	return round((double) (y - distance_map->config.y_origin) / distance_map->config.resolution);
}


bool
my_f_ordenation (state_node *a, state_node *b)
{
	return (a->f > b->f);
}


std::vector<carmen_ackerman_traj_point_t>
build_rddf_poses(std::vector<state_node> &path, state_node *current_state, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	path = build_state_path(current_state->parent);
	std::reverse(path.begin(), path.end());
	std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path;

	for (int i = 0; i < path.size(); i++)
	{
		temp_rddf_poses_from_path.push_back(path[i].state);
		if(OPENCV)
			draw_point_on_map_img(path[i].state.x, path[i].state.y, distance_map->config, 255, 0, 0);

	}

	return temp_rddf_poses_from_path;
}


void
astar_publish_rddf_message(state_node *current_state,  carmen_obstacle_distance_mapper_map_message *distance_map)
{
	std::vector<state_node> path;
	std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path = build_rddf_poses(path, current_state, distance_map);
	carmen_ackerman_traj_point_t *carmen_rddf_poses_from_path = &temp_rddf_poses_from_path[0];
	carmen_ackerman_traj_point_t last_pose = {.x = 0.0, .y = 0.0, .theta = 0.0, .v = 9.0, .phi=0.2};
	int annotations[2] = {1, 2};
	int annotation_codes[2] = {1, 2};
/*
	for (int i = 0; i < path.size(); i++)
	{
		printf("Poses do rddf: %f %f %d\n", carmen_rddf_poses_from_path[i].x, carmen_rddf_poses_from_path[i].y, i);
	//	printf("Poses do path: %f %f %d\n", path[i].state.x, path[i].state.y, i);
	}
*/
	carmen_rddf_publish_road_profile_message(carmen_rddf_poses_from_path, &last_pose, path.size(), 1, annotations, annotation_codes);
	printf("Poses enviadas!\n");
	temp_rddf_poses_from_path.clear();
}


discrete_pos_node*
get_current_pos(state_node* current_state, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	discrete_pos_node *current_pos = (discrete_pos_node*) malloc(sizeof(discrete_pos_node));
	current_pos->x = get_astar_map_x(current_state->state.x, distance_map);
	current_pos->y = get_astar_map_y(current_state->state.y, distance_map);
	current_pos->theta = get_astar_map_theta(current_state->state.theta);

	return current_pos;
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
circle_node_exist(circle_node *current, vector<circle_node*> &closed_set)
{
    vector<circle_node*>::iterator it = closed_set.begin();
    vector<circle_node*>::iterator end = closed_set.end();

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
closest_circle_new(state_node *current_state, vector<circle_node> &circle_path)
{
	double min_dist = DBL_MAX;
	double dist;
	int indice = 0;

	for (unsigned int i = 0; i < circle_path.size(); i++)
	{
	  dist = DIST2D(current_state->state, circle_path[i]);
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


vector<circle_node>
build_circle_path_new(circle_node *node, double initial_cost)
{
	vector<circle_node> circle_path;
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
draw_circle_path(vector<circle_node> circle_path, carmen_map_config_t config)
{
	for (unsigned int i = 0; i < circle_path.size(); i++)
	{
		draw_circle_on_map_img(circle_path[i].x, circle_path[i].y, circle_path[i].radius, config);
	}

}

void
expand_circle(circle_node *current, priority_queue<circle_node*, vector<circle_node*>, CircleNodeComparator> &open_set, carmen_point_t goal, double robot_width,
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

        double nearst_obstacle_distance = obstacle_distance(nx, ny, distance_map);

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


vector<circle_node>
space_exploration(circle_node *start_node, circle_node *goal_node, carmen_point_t goal, double robot_width, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	vector<circle_node> temp_circle_path;
    priority_queue<circle_node*, vector<circle_node*>, CircleNodeComparator> open_set;
    vector<circle_node*> closed_set;
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


vector<circle_node>
circle_exploration(carmen_point_t *robot_pose, carmen_point_t *goal, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	circle_node *start_circle, *goal_circle;
	vector<circle_node> circle_path;

	start_circle = create_circle_node(robot_pose->x, robot_pose->y, obstacle_distance(robot_pose->x, robot_pose->y, distance_map), 0.0,
			DIST2D_P(robot_pose, goal), NULL);

	goal_circle = create_circle_node(goal->x, goal->y, obstacle_distance(goal->x, goal->y, distance_map), DBL_MAX, 0.0, NULL);

	circle_path = space_exploration(start_circle, goal_circle, *goal, 4.0, distance_map);     // TODO Read car width from carmen.ini investigate how to avoid passing the goal as parameter

	circle_path.insert(circle_path.begin(), *goal_circle);

	if (OPENCV)
		draw_circle_path(circle_path, distance_map->config);

	// TODO Checar se precisa atualizar os custos g e h do circle path

	return (circle_path);
}


void
expand_state(state_node *current_state, state_node *goal_state, fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> &heap_open_list,
		carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map, vector<circle_node> circle_path)
{
	#define NHOLONOMIC 1
	#define NUM_STEERING_ANGLES 3
    double target_phi, distance_traveled = 0.0;
    distance_traveled = 2.0;
    double steering_acceleration[NUM_STEERING_ANGLES] = {-0.25, 0.0, 0.25}; //TODO ler velocidade angular do volante do carmen.ini
    double target_v[2]   = {2.0, -2.0};
    double add_x[3] = {-1.0, 0.0, 1.0};
    double add_y[3] = {-1.0, 0.0, 1.0};
    int size_for;

    if (NHOLONOMIC)
    	size_for = sizeof(target_v)/sizeof(target_v[0]);
    else
    	size_for = 3;

    //   for neighbors of current:
    for (int i = 0; i < size_for; ++i)
    {
        for (int j = 0; j < NUM_STEERING_ANGLES; ++j)
        {
        	state_node_p new_state = (state_node_p) malloc(sizeof(state_node));
        	target_phi = carmen_clamp(-robot_config.max_phi, (current_state->state.phi + steering_acceleration[j]), robot_config.max_phi);
        	//target_phi = steering_acceleration[j];

        	// Utilizando expansão não-holonomica
        	if(NHOLONOMIC)
        	{
        		new_state->state = carmen_libcarmodel_recalc_pos_ackerman(current_state->state, target_v[i], target_phi, 1.0 , &distance_traveled, DELTA_T, robot_config);
        	}
        	else
        	{
        		// Utilizando expansão holonomica
				new_state->state.x = current_state->state.x + add_x[i];
				new_state->state.y = current_state->state.y + add_y[j];
				new_state->state.theta = current_state->state.theta;
        	}

        	new_state->parent = current_state;
        	//    cost = g(current) + movementcost(current, neighbor)
        	new_state->g = current_state->g + DIST2D(current_state->state, new_state->state);

        	if (circle_path.empty()) //not using circle heuristic
        		new_state->h = DIST2D(new_state->state, goal_state->state);
        	else //using circle heuristic
        	{
				circle_node current_circle = closest_circle_new(new_state, circle_path);
				new_state->h = DIST2D(new_state->state, current_circle) + current_circle.h;
			}


        	new_state->f = new_state->g + new_state->h;

        	if(new_state->state.v < 0)
        	{
        		new_state->f *= 1.1;
        	}

        	if(new_state->state.v != current_state->state.v)
        	{
        		new_state->f += 1;
        	}

        	new_state->is_open = 1;
        	discrete_pos_node *current_pos = get_current_pos(new_state, distance_map);

        	if (NHOLONOMIC)
        		current_pos->theta = get_astar_map_theta(new_state->state.theta);
        	else
        		current_pos->theta = 0;

        	// delete the node if near an obstacle
        	if (obstacle_distance(new_state->state.x, new_state->state.y, distance_map) < 2.0)
        	{
        		free (new_state);
        	}
        	else
        	{
        		// if neighbor in OPEN and cost less than g(neighbor):
        		if(astar_map[current_pos->x][current_pos->y][current_pos->theta] != NULL && (astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open == 1 &&
        				astar_map[current_pos->x][current_pos->y][current_pos->theta]->g > new_state->g))
				{
        			// remove neighbor from OPEN, because new path is better
        			astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open = 0;
				}

        		// if neighbor in CLOSED and cost less than g(neighbor): ⁽²⁾
        		if(astar_map[current_pos->x][current_pos->y][current_pos->theta] != NULL && (astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_closed == 1 &&
        				astar_map[current_pos->x][current_pos->y][current_pos->theta]->g > new_state->g))
        		{
        			// remove neighbor from CLOSED
        			astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_closed = 0;
        		}

        		// if neighbor not in OPEN and neighbor not in CLOSED: -- Também precisa verificar se ele é NULL, que significa que o neighbor não existe ainda
        		if (astar_map[current_pos->x][current_pos->y][current_pos->theta] == NULL || (astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_closed == 0 &&
        				astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open == 0))
        		{
        			// set g(neighbor) to cost -- Isso já foi feito anteriormente
        			// add neighbor to OPEN
        			heap_open_list.push(new_state);
        			astar_map[current_pos->x][current_pos->y][current_pos->theta] = new_state;
        			// set priority queue rank to g(neighbor) + h(neighbor) -- Já é feito pela heap
        			// set neighbor's parent to current -- Já foi feito anteriormente
        		}
        		if(0)
				{
					if(j == 0)
						draw_point_on_map_img(new_state->state.x, new_state->state.y, distance_map->config, 0, 0, 255);
					else if(j == 2)
						draw_point_on_map_img(new_state->state.x, new_state->state.y, distance_map->config, 255, 0, 0);
					else
						draw_point_on_map_img(new_state->state.x, new_state->state.y, distance_map->config, 0, 255, 0);
//	        		usleep(100000);
				}
        	}
        }
    }

}


void
compute_astar_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config,
		carmen_obstacle_distance_mapper_map_message *distance_map)
{

	// Following the code from: http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html

	state_node *start_state, *goal_state;
	start_state = create_state_node(robot_pose->x, robot_pose->y, robot_pose->theta, 0.1, 0.0, 0.0, DIST2D_P(robot_pose, goal_pose), NULL);
	goal_state = create_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, 0.0, 0.0, DBL_MAX, DBL_MAX, NULL);
	fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> heap_open_list;
	alloc_astar_map(distance_map);

	if(OPENCV)
	{
		display_map(distance_map);
		draw_point_on_map_img(start_state->state.x, start_state->state.y, distance_map->config, 128, 128, 128);
		draw_point_on_map_img(goal_state->state.x, goal_state->state.y, distance_map->config, 128, 128, 128);
	}

	start_state->is_open = 1;
	// OPEN = priority queue containing START
	heap_open_list.push(start_state);
	discrete_pos_node *current_pos = get_current_pos(start_state, distance_map);
	astar_map[current_pos->x][current_pos->y][current_pos->theta] = start_state;

	//CLOSED = empty set -- No meu caso, closed é o parâmetro is_closed do nó

	vector<circle_node> circle_path;
	if (CIRCLE_HEURISTIC)
		circle_path = circle_exploration(robot_pose, goal_pose, distance_map);

	// while OPEN is not empty -- Adaptado
	while (!heap_open_list.empty())
	{
		//  current = remove lowest rank item from OPEN
		state_node *current_state = heap_open_list.top();
		heap_open_list.pop();
		discrete_pos_node *current_pos = get_current_pos(current_state, distance_map);
		// add current to CLOSED -- O current precisa ser adicionado ao CLOSED após a expansão


		//if lowest rank in OPEN is the GOAL: -- Adaptado
		if (DIST2D(current_state->state, goal_state->state) < 1.5 )
		{
			astar_publish_rddf_message(current_state, distance_map);
			heap_open_list.clear();
			break;
		}
		else
		{
			// Expansion
			expand_state(current_state, goal_state, heap_open_list, robot_config, distance_map, circle_path);
		}

		if(OPENCV)
			draw_point_on_map_img(current_state->state.x, current_state->state.y, distance_map->config, 0, 0, 128);
	}
	
	clear_astar_map(distance_map);
	printf("Terminou compute_astar_path !\n");
//	exit(1);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	current_globalpos_msg = msg;
	get_pos_message(msg->globalpos/*, msg->v, msg->timestamp*/);
}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	final_goal = &(rddf_end_point_message->point);
	printf("carmen_rddf_play_end_point: %lf %lf %lf\n", rddf_end_point_message->point.x, rddf_end_point_message->point.y, rddf_end_point_message->point.theta);
//	printf ("Recebeu Goal!!!!!  %d\n", rddf_end_point_message->number_of_poses);
}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;

	if (compact_distance_map == NULL)
	{
		carmen_obstacle_distance_mapper_create_new_map(&distance_map, message->config, message->host, message->timestamp);
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(&distance_map, compact_distance_map, DISTANCE_MAP_HUGE_DISTANCE);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribes                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_subscribe_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) carmen_rddf_play_end_point_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


///////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        printf("path_planner_astar_main: Disconnected.\n");
        exit(0);
    }
}


static void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
			{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &robot_config.length,							 			1, NULL},
			{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &robot_config.width,								 			1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels,			 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
			{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &robot_config.max_v,									 		1, NULL},
			{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &robot_config.max_phi,								 		1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate,					1, NULL},
			{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient,							1, NULL}
		};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	carmen_rddf_play_get_parameters(argc, argv);

	carmen_rddf_play_subscribe_messages();

	signal(SIGINT, shutdown_module);

	printf("Até aqui está funcionando!\n");

	carmen_ipc_dispatch();

	return (0);
}
