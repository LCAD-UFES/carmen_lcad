#include "stehs_planner.h"

using namespace std;
using namespace cv;


double
distance(double ax, double ay, double bx, double by)
{
    double dx = ax - bx;
    double dy = ay - by;
    return sqrt(dx * dx + dy * dy);
}


double
obstacle_distance(double x, double y, carmen_obstacle_distance_mapper_map_message *distance_map)
{
    carmen_point_t p;

    p.x = x;
    p.y = y;

    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, distance_map));
}


circle_node*
create_circle_node(double cx, double cy, double radius, double g, double f, circle_node *p)
{
	circle_node *new_circle = (circle_node*) malloc(sizeof(circle_node));
	new_circle->x = cx;
	new_circle->y = cy;
	new_circle->radius = radius;
	new_circle->g = g;      // (distance) cost from start to current state
	new_circle->f = f;      // total (distance) cost, g + h
	//new_circle->h = ;       // the (distance) cost from current state to goal
	new_circle->parent = p;

	return new_circle;
}

/*
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

	distance = sqrt(std::pow(current->x - circle->x, 2) + pow(current->y - circle->y, 2));

	return (distance - greater) < smaller * overlap_factor;
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


void
display_map(carmen_obstacle_distance_mapper_map_message *distance_map)
{
	unsigned int width = distance_map->config.x_size;
	unsigned int height = distance_map->config.y_size;
	unsigned int size = width * height;
	unsigned char *map = new unsigned char[size];

	for (unsigned int i = 0; i < size; ++i)
	{
		unsigned int row = (height - 1) - i % height;

		unsigned int col = i / height;

		unsigned int index = row * width + col;

		if (0.0 == distance_map->complete_x_offset[i] && 0.0 == distance_map->complete_y_offset[i])
		{
			map[index] = 0;
		} else
		{
			map[index] = 255;
		}
	}

    Mat img(width, height, CV_8UC1, map);

    imshow("Obstacle Map", img);
    waitKey(1);
}


void
expand(circle_node *current, priority_queue<circle_node*, vector<circle_node*>, CircleNodeComparator> &open_set, carmen_point_t goal, double robot_width,
		carmen_obstacle_distance_mapper_map_message *distance_map)
{
	// Considering a 2m radius parent circle that involves the car we want 32 children circles
    unsigned int children_amount = (unsigned int) (16.0 * current->radius);

    double theta = 2.0 * M_PI / (double) children_amount;
    double c = cos(theta);
    double s = sin(theta);
    double t;

    double px = current->x;
    double py = current->y;
    double pg = current->g;
    double pr = current->radius;

    double x = pr;
    double y = 0.0;

    for (unsigned int i = 0; i < children_amount; i++)
    {
        double nx = x + px;
        double ny = y + py;

        double nearst_obstacle_distance = obstacle_distance(nx, ny, distance_map);

        if (nearst_obstacle_distance > robot_width * 0.5)
        {
            // TODO verificar se é possível remover filhos com overlap
            open_set.push(create_circle_node(nx, ny, nearst_obstacle_distance, pg + pr, pg + pr + distance(nx, ny, goal.x, goal.y), current));
        }

        // Apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
}


list<circle_node*>
build_circle_path(circle_node *node)
{
    list<circle_node*> temp_circle_path;

    while (node != NULL)
    {
        temp_circle_path.push_front(node);

        node = node->parent;
    }

    return(temp_circle_path);
}


list<circle_node>
space_exploration(circle_node *start_node, circle_node *goal_node, carmen_point_t goal, double robot_width, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	printf("SpaceExploration\n");

    list<circle_node> temp_circle_path;

    start_node->g = 0.0;
    start_node->f = distance(start_node->x, start_node->y, goal_node->x, goal_node->y);

    goal_node->g = goal_node->f = DBL_MAX;

    priority_queue<circle_node*, vector<circle_node*>, CircleNodeComparator> open_set;

    vector<circle_node*> closed_set;

//    if (start_node->circle.Overlap(goal_node->circle, MIN_OVERLAP_FACTOR))
//    {
//    	temp_circle_path.push_back(*start_node);
//    	temp_circle_path.push_back(*goal_node);
//
//    	return (temp_circle_path);
//    }

    expand(start_node, open_set, goal, robot_width, distance_map);

    while (!open_set.empty())
    {
    	circle_node *current = open_set.top();                   // Get the circle witch is the closest to the goal node
        open_set.pop();

        if (goal_node->f < current->f)
        {
            temp_circle_path = build_circle_path(goal_node->parent);

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
        	expand(start_node, open_set, goal, robot_width, distance_map);

//            if (current->circle.Overlap(goal_node->circle, MIN_OVERLAP_FACTOR))
//            {
//
//                if (current->f < goal_node->g)
//                {
//                    goal_node->g = current->f;
//                    goal_node->parent = current;
//                }
//            }
            closed_set.push_back(current);
        }
        else
        {
            delete current;
        }
    }

    while(!closed_set.empty())                // Only to clean the closed_set
    {
    	circle_node *tmp = closed_set.back();

        closed_set.pop_back();

        delete tmp;
    }

    return (temp_circle_path);
}


//void
//goal_space_exploration()
//{
//    CircleNodePtr start_circle =  new CircleNode(start.x, start.y, ObstacleDistance(start), 0.0, 0.0, nullptr);
//
//    CircleNodePtr goal_circle = new CircleNode(goal.x, goal.y, ObstacleDistance(goal), DBL_MAX, DBL_MAX, nullptr);
//
//    circle_path = SpaceExploration(start_circle, goal_circle);
//
//    UpdateCircleGoalDistance();
//}

*/
void
compute_rddf_using_stehs_planner(carmen_point_t *robot_pose, carmen_point_t *goal, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	//printf ("Entrou\n");
	robot_pose = goal;

	//display_map(distance_map);
}


