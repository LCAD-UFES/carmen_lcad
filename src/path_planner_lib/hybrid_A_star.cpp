#include "path_planner.h"


using namespace std;
using namespace cv;


#define CLOSEST_CIRCLE_MIN_DIST 1.5


Mat map_image;


void
display_map(carmen_obstacle_distance_mapper_map_message *distance_map)
{
	if (distance_map->complete_x_offset == NULL)
		return;

	unsigned int width = distance_map->config.x_size;
	unsigned int height = distance_map->config.y_size;
	unsigned int size = width * height;
	unsigned char map[size];

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

	//printf("New Circle %d %d %d rows %d cols %d\n", img_x, img_y, (int) r, map_image.rows, map_image.cols);

	circle(map_image, Point(img_x, config.y_size - 1 - img_y), img_r, 0, 1, 8);
	imshow("Obstacle Map", map_image);
	waitKey(1);
	//usleep(500000);
}


void
draw_point_on_map_img(double x, double y, carmen_map_config_t config)
{
	int img_x = (double) (x - config.x_origin) / config.resolution;
	int img_y = (double) (y - config.y_origin) / config.resolution;

	circle(map_image, Point(img_x, config.y_size - 1 - img_y), 1, 128, -1, 8);
	imshow("Obstacle Map", map_image);
	waitKey(1);
	//usleep(10000);
}


void
draw_circle_path(vector<circle_node> circle_path, carmen_map_config_t config)
{
	for (unsigned int i = 0; i < circle_path.size(); i++)
	{
		draw_circle_on_map_img(circle_path[i].x, circle_path[i].y, circle_path[i].radius, config);
	}
	usleep(500000);
}


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
create_circle_node(double cx, double cy, double radius, double g, double h, circle_node *parent)
{
	circle_node *new_circle = (circle_node*) malloc(sizeof(circle_node));
	new_circle->x = cx;
	new_circle->y = cy;
	new_circle->radius = radius;
	//new_circle->f = f;      // total (distance) cost, g + h
	new_circle->g = g;      // (distance) cost from start to current state
	new_circle->h = h;       // the (distance) cost from current state to goal
	new_circle->parent = parent;

	return new_circle;
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
	new_state->g = g;
	new_state->h = h;
	new_state->parent = parent;

	return (new_state);
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

            draw_circle_on_map_img(nx, ny, nearst_obstacle_distance, distance_map->config);
            //draw_point_on_map_img(nx, ny, distance_map->config);
        }

        // Apply the rotation matrix
        t = x;
        x = cos_displacement_angle * x - sin_displacement_angle * y;
        y = sin_displacement_angle * t + cos_displacement_angle * y;
    }
}


vector<circle_node>
build_circle_path(circle_node *node)
{
	vector<circle_node> circle_path;
	int i = 0;
    while (node != NULL)
    {
        circle_path.push_back(*node);

        node = node->parent;

        i++;
    }

    //printf("--- %u %d\n", (int) circle_path.size(), i);

    return(circle_path);
}


vector<circle_node>
space_exploration(circle_node *start_node, circle_node *goal_node, carmen_point_t goal, double robot_width, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	//printf("Space Exploration\n");

	vector<circle_node> temp_circle_path;

    priority_queue<circle_node*, vector<circle_node*>, CircleNodeComparator> open_set;

    vector<circle_node*> closed_set;


    open_set.push(start_node); // Added

//    expand_circle(start_node, open_set, goal, robot_width, distance_map);

    while (!open_set.empty())
    {
    	circle_node *current = open_set.top();                   // Get the circle witch is the closest to the goal node
        open_set.pop();

        if (goal_node->g < (current->g + current->h))
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
        	expand_circle(current, open_set, goal, robot_width, distance_map);

            if (circle_overlap(current, goal_node, MIN_OVERLAP_FACTOR))
            {
            	//printf ("Goal Overlap\n");
                if ((current->g + current->h) < goal_node->g)
                {
                    goal_node->g = (current->g + current->h);
                    goal_node->parent = current;
                }
            }
            closed_set.push_back(current);
        }
        else
        {
        	//printf("3\n");
            delete current;
        }
    }

    while(!closed_set.empty())                // Only to clean the closed_set
    {
    	circle_node *tmp = closed_set.back();

        closed_set.pop_back();

        delete tmp;
    }

    //printf("Finished!!!\n");

    return (temp_circle_path);
}


vector<circle_node>
space_exploration_old(circle_node *start_circle, circle_node *goal_circle, carmen_point_t goal, double robot_width, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	// printf("Space Exploration Old\n");

	vector<circle_node> circle_path;

    start_circle->g = 0.0;
    //start_circle->f = distance(start_circle->x, start_circle->y, goal_circle->x, goal_circle->y);

    priority_queue<circle_node*, vector<circle_node*>, CircleNodeComparator> open_set;

    vector<circle_node*> closed_set;

    open_set.push(start_circle);

    expand_circle(start_circle, open_set, goal, robot_width, distance_map);

    while (!open_set.empty())
    {
    	circle_node *current = open_set.top();                   // Get the circle witch is the closest to the goal node
        open_set.pop();

        if (circle_overlap(current, goal_circle, MIN_OVERLAP_FACTOR))
        {
        	goal_circle->parent = current;

            circle_path = build_circle_path(goal_circle);

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
        	//printf("Expand\n");
        	expand_circle(current, open_set, goal, robot_width, distance_map);
            closed_set.push_back(current);
        }
    }

    while(!closed_set.empty())                // Only to clean the closed_set
    {
    	circle_node *tmp = closed_set.back();

        closed_set.pop_back();

        delete tmp;
    }

    return (circle_path);
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


circle_node
closest_circle(state_node *current_state, vector<circle_node> &circle_path)
{
	double dist = DIST2D(current_state->state, circle_path.back());

	if (dist < CLOSEST_CIRCLE_MIN_DIST)
	{
		circle_path.pop_back();
		//printf ("DIST %lf  *************************************\n", dist);
	}

	return (circle_path.back());
}


#define MIN_THETA_DIFF    0.0872665 // 5 0.0436332 // 2.5 // 0.261799 // 15 degrees
#define MIN_STEERING_DIFF 0.0872665
#define MIN_POS_DISTANCE  0.3 // the carmen grid map resolution

bool
state_node_exist(state_node *new_state, vector<state_node*> &closed_set)
{
	double distance = 0.0;
	double orientation_diff = 0.0;
	double steering_diff = 0.0;

	//printf ("%d\n", closed_set.size());

    for (unsigned int i = 0; i < closed_set.size(); i++)
    {
    	distance = DIST2D(new_state->state, closed_set[i]->state);
//
//    	if (distance < 0.5)
//    		printf ("State Exist! %lf %lf %lf %lf %lf %lf\n", distance, angular_distance);

    	orientation_diff = carmen_compute_abs_angular_distance(new_state->state.theta, closed_set[i]->state.theta);
    	steering_diff = carmen_compute_abs_steering_angular_distance(new_state->state.phi, closed_set[i]->state.phi);

    	if (distance < MIN_POS_DISTANCE && orientation_diff < MIN_THETA_DIFF && steering_diff < MIN_POS_DISTANCE && (new_state->state.v == closed_set[i]->state.v))
    	{
    		//printf ("State Exist! %lf %lf\n", distance, angular_distance);
    		return true;
    	}
    }

    return false;
}



#define DIRECTION_OF_MOVEMENT_CHANGE_PENALTY 5
#define DRIVING_BACKWARD_PENALTY 5


void
expand_state(state_node *current_state, state_node *goal_state, vector<state_node*> &closed_set, circle_node current_circle, priority_queue<state_node*, vector<state_node*>, StateNodePtrComparator> &open_set,
		carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map)
{
    double target_phi, distance_traveled = 0.0;
	#define NUM_STEERING_ANGLES 3
    double steering_acceleration[NUM_STEERING_ANGLES] = {-0.25, 0.0, 0.25}; //TODO ler velocidade angular do volante do carmen.ini
    double target_v[3]   = {2.0, 0.0, -2.0};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < NUM_STEERING_ANGLES; ++j)
        {
        	state_node *new_state = (state_node*) malloc(sizeof(state_node));

        	target_phi = carmen_clamp(-robot_config.max_phi, (current_state->state.phi + steering_acceleration[j]), robot_config.max_phi);

        	new_state->state = carmen_libcarmodel_recalc_pos_ackerman(current_state->state, target_v[i], target_phi,
        			0.25, &distance_traveled, DELTA_T, robot_config);

        	new_state->parent = current_state;
        	new_state->g = current_state->g + DIST2D(current_state->state, new_state->state);
        	new_state->h = DIST2D(new_state->state, current_circle) + current_circle.h;
        	//new_state->h = max(DIST2D(new_state->state, current_circle) + current_circle.h, carmen_compute_abs_angular_distance(new_state->state.theta, goal_state->state.theta));

        	draw_point_on_map_img(new_state->state.x, new_state->state.y, distance_map->config);

//        	if (new_state->state.v != current_state->state.v)
//        		new_state->h += DIRECTION_OF_MOVEMENT_CHANGE_PENALTY;

        	//double max_curvature = carmen_get_curvature_from_phi(robot_config.max_phi, new_state->state.v, robot_config.understeer_coeficient, robot_config.distance_between_front_and_rear_axles); // TODO is it necessary divide theta dif bay max k (stehs 2015-1)

//        	if (!Exist(next_state, closed_set, 1.0 /*k*/) && !Collision(next_state))
        	if (trajectory_pose_hit_obstacle(new_state->state, 0.5, distance_map, &robot_config) || state_node_exist(new_state, closed_set))   // TODO ler a margem de segurança do carmen.ini
        	{
        		free (new_state);
        	}
        	else
        	{
        		open_set.push(new_state);
//            	printf ("Ang Dist %lf\n", (/*carmen_radians_to_degrees*/(carmen_compute_abs_angular_distance(new_state->state.theta, goal_state->state.theta))) /*/ max_curvature*/);
        		//printf("State %lf %lf %lf %lf %lf %lf %lf\n", new_state->state.x, new_state->state.y, new_state->state.theta, new_state->state.v, new_state->state.phi, new_state->g, new_state->h);
        	}
        }
    }
    //printf("Size %d\n", (int) open_set.size());
}


vector<state_node>
heuristic_search(state_node *start_state, state_node *goal_state, vector<circle_node> circle_path,
		carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map)
{
    priority_queue<state_node*, vector<state_node*>, StateNodePtrComparator> open_set;

    vector<state_node*> closed_set;
    vector<state_node> state_path;

    open_set.push(start_state);

    while (!open_set.empty())
    {
    	state_node *current_state = open_set.top();
        open_set.pop();
        printf("--- State %lf %lf %lf %lf %lf %lf %lf\n", current_state->state.x, current_state->state.y, current_state->state.theta, current_state->state.v, current_state->state.phi, current_state->g, current_state->h);

        if (DIST2D(current_state->state, goal_state->state) < 0.5)
//        if ((goal_state->g + goal_state->h) < (current_state->g + current_state->h))
        {
        	while(!open_set.empty())
        	{
        		state_node *tmp = open_set.top();

        		open_set.pop();

        		delete tmp;
        	}
        	break;
        }
        //circle_node c = closest_circle(current_state, circle_path);
        //draw_circle_on_map_img(c.x, c.y, c.radius, distance_map->config);

        if (!state_node_exist(current_state, closed_set))
        	expand_state(current_state, goal_state, closed_set, closest_circle(current_state, circle_path), open_set, robot_config, distance_map);

//        if (current->h < RGOAL)      // FIXME Essa verificação nao devia ser feita usando o melhor candidato da ultima chamada a Expand???
//        {
//            GoalExpand(current, goal_state, open_set);
//        }

        closed_set.push_back(current_state);

//        if (open_set.empty())
//        {
//            k *= 0.5;
//
//            if (k > KMIN)
//            {
//                SetSwap(open_set, closed_set);
//            }
//        }
        //printf("FOI\n");
    }

    while(!closed_set.empty())
    {
    	state_node *tmp = closed_set.back();

        closed_set.pop_back();

        delete tmp;
    }

    //printf("Saiu HS\n");
    return (state_path);
}


vector<circle_node>
circle_exploration(carmen_point_t *robot_pose, carmen_point_t *goal, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	circle_node *start_circle, *goal_circle;
	vector<circle_node> circle_path;

	//printf("Start %lf %lf\n", robot_pose->x, goal->x);

	start_circle = create_circle_node(robot_pose->x, robot_pose->y, obstacle_distance(robot_pose->x, robot_pose->y, distance_map), 0.0,
			distance(robot_pose->x, robot_pose->y, goal->x, goal->y), NULL);
	//draw_circle_on_map_img(robot_pose->x, robot_pose->y, obstacle_distance(robot_pose->x, robot_pose->y, distance_map), distance_map->config);

	//printf("Goal\n");
	goal_circle = create_circle_node(goal->x, goal->y, obstacle_distance(goal->x, goal->y, distance_map), DBL_MAX, 0.0, NULL);
	//draw_circle_on_map_img(goal->x, goal->y, obstacle_distance(goal->x, goal->y, distance_map), distance_map->config);

	//printf("Space Exploration\n");
	circle_path = space_exploration(start_circle, goal_circle, *goal, 4.0, distance_map);     // TODO Read car width from carmen.ini investigate how to avoid passing the goal as parameter

	//circle_path.push_back(*goal_circle);
	circle_path.insert(circle_path.begin(), *goal_circle);

	//for(unsigned int i = 0; i < circle_path.size(); i++) printf("H %lf\n", circle_path[i].h);

	draw_circle_path(circle_path, distance_map->config);

	// TODO Checar se precisa atualizar os custos g e h do circle path

	//circle_path =	revert_order_and_insert_start_and_goal(circle_path, start_circle, goal_circle);

	return (circle_path);
}


void
compute_hybrid_A_star_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	state_node *start_state, *goal_state;

	start_state = create_state_node(robot_pose->x, robot_pose->y, robot_pose->theta, 0.0, 0.0, 0.0, DIST2D_P(robot_pose, goal_pose), NULL);
	goal_state = create_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, 0.0, 0.0, DBL_MAX, DBL_MAX, NULL);

	vector<circle_node> circle_path = circle_exploration(robot_pose, goal_pose, distance_map);
//
	// vector<state_node> path = heuristic_search(start_state, goal_state, circle_path, robot_config, distance_map);

	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	double distance_traveled = 0.0;
	carmen_ackerman_traj_point_t rs_points[5];

	draw_point_on_map_img(start_state->state.x, start_state->state.y, distance_map->config);
	draw_point_on_map_img(goal_state->state.x, goal_state->state.y, distance_map->config);

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);

	reed_shepp(start_state->state, goal_state->state, &rs_numero, &tr, &ur, &vr);
	rs_pathl = constRS(rs_numero, tr, ur, vr, start_state->state, rs_points);

	printf("Poses in Reed Sheep path: %d\n", rs_pathl);
	for (int i = rs_pathl - 1; i > 0 /*rs_pathl*/; i--)
	{
		carmen_ackerman_traj_point_t point = rs_points[i];
		while (DIST2D(point, rs_points[i+1]) > 2.0)
//		for (int i = 0; i < 10; i++)
		{
			point = carmen_libcarmodel_recalc_pos_ackerman(point, 2.0, rs_points[i].phi,
					0.25, &distance_traveled, DELTA_T, robot_config);

			draw_point_on_map_img(point.x, point.y, distance_map->config);
			//draw_point_on_map_img(rs_points[i].x, rs_points[i].y, distance_map->config);
			usleep(500000);
			printf("%lf %lf %lf %lf %lf\n", rs_points[i].x, rs_points[i].y, rs_points[i].theta, rs_points[i].v, rs_points[i].phi);
		}
	}
}


void
compute_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config,
		carmen_obstacle_distance_mapper_map_message *distance_map)
{
	display_map(distance_map);

	compute_hybrid_A_star_path(robot_pose, goal_pose, robot_config, distance_map);
}

