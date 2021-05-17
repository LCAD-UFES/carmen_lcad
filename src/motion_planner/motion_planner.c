#include <carmen/obstacle_avoider_interface.h>
#include "motion_planner.h"

//static void teste_stop(double space_interval, double vel);
static carmen_robot_and_trailer_traj_point_t trajectory_vector_of_points[MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE];

int current_motion_command_vetor_index = 0;
carmen_robot_and_trailer_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS][NUM_MOTION_COMMANDS_PER_VECTOR];
int nun_motion_commands[NUM_MOTION_COMMANDS_VECTORS];

carmen_robot_and_trailer_traj_point_t g_robot_position;
carmen_motion_planner_path_message path;
int autonomous_status = 0;

carmen_behavior_selector_algorithm_t current_algorithm = CARMEN_BEHAVIOR_SELECTOR_GRADIENT;
static carmen_behavior_selector_task_t current_task = BEHAVIOR_SELECTOR_PARK;
double min_delta_d = INTMAX_MAX;
int replan = 1;

static carmen_robot_ackerman_config_t carmen_robot_ackerman_config;

int current_map = 0;
carmen_map_p map_vector[NUM_MAPS];
int necessary_maps_available = 0;

carmen_robot_and_trailer_traj_point_t *g_current_trajectory = NULL;
int g_current_trajectory_length;

static double phi_gain = 1.0;



carmen_robot_and_trailer_traj_point_t
predict_new_robot_position(carmen_robot_and_trailer_traj_point_t current_robot_position, double v, double phi, double time, carmen_robot_ackerman_config_t *car_config)
{
	carmen_robot_and_trailer_traj_point_t new_robot_position;
	int i;
	double delta_time;

	new_robot_position = current_robot_position;
	delta_time = time / 10.0;
	for (i = 0; i < 10; i++)
	{
		new_robot_position.theta = new_robot_position.theta + (v * tan(phi) / car_config->distance_between_front_and_rear_axles) * delta_time;
		new_robot_position.x = new_robot_position.x + v * cos(new_robot_position.theta) * delta_time;
		new_robot_position.y = new_robot_position.y + v * sin(new_robot_position.theta) * delta_time;
	}

	new_robot_position.v = v;
	new_robot_position.phi = phi;

	return (new_robot_position);
}


int
build_predicted_trajectory(carmen_robot_and_trailer_motion_command_t *motion_commands_vector, int num_motion_commands,
		carmen_robot_and_trailer_traj_point_t initial_pose)
{
	int i, trajectory_vector_of_points_size = 0;
	double t;
	carmen_robot_and_trailer_traj_point_t pose;

	pose = initial_pose;

	for (i = 0; i < num_motion_commands; i++)
	{
		for (t = 0.0; t < motion_commands_vector[i].time; t += (motion_commands_vector[i].time / 5.0))
		{
			pose = predict_new_robot_position(pose, motion_commands_vector[i].v, motion_commands_vector[i].phi, (motion_commands_vector[i].time / 5.0), &carmen_robot_ackerman_config);
			trajectory_vector_of_points[trajectory_vector_of_points_size] = pose;
			trajectory_vector_of_points_size++;
			if (trajectory_vector_of_points_size >= (MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE - 2))
				break;
		}
	}
	return (trajectory_vector_of_points_size);
}


carmen_navigator_ackerman_plan_message 
build_navigator_ackerman_plan_message(carmen_robot_and_trailer_motion_command_t *motion_commands_vector, int num_motion_commands)
{
	int i, trajectory_vector_of_points_size;
	carmen_navigator_ackerman_plan_message predicted_trajectory_message;

	trajectory_vector_of_points_size = build_predicted_trajectory(motion_commands_vector, num_motion_commands, g_robot_position);

	predicted_trajectory_message.path_length = trajectory_vector_of_points_size;
	predicted_trajectory_message.path = (carmen_robot_and_trailer_traj_point_t *) malloc(sizeof(carmen_robot_and_trailer_traj_point_t) * (trajectory_vector_of_points_size));

	for (i = 0; i < trajectory_vector_of_points_size; i++)
	{
		predicted_trajectory_message.path[i].x 		= trajectory_vector_of_points[i].x;
		predicted_trajectory_message.path[i].y 		= trajectory_vector_of_points[i].y;
		predicted_trajectory_message.path[i].theta 	= trajectory_vector_of_points[i].theta;
		predicted_trajectory_message.path[i].v 		= trajectory_vector_of_points[i].v;
		predicted_trajectory_message.path[i].phi 	= trajectory_vector_of_points[i].phi;
	}

	predicted_trajectory_message.timestamp = carmen_get_time();
	predicted_trajectory_message.host = carmen_get_host();

	return predicted_trajectory_message;
}


carmen_robot_and_trailer_traj_point_t
carmen_conventional_astar_ackerman_kinematic_3(carmen_robot_and_trailer_traj_point_t point, double lenght, double phi, double v)
{

	double	radcurv = lenght / tan(fabs(phi));

	if(phi == 0)
	{

		point.x += v * cos(point.theta);
		point.y += v * sin(point.theta);
		point.theta = carmen_normalize_theta(point.theta);
		point.phi = phi;
		point.v = v;
	}
	else
	{
		double temp_v = fabs(v) / radcurv;
		int direction_signal = phi >= 0 ? -1 : 1;

		double center_x = point.x + radcurv * sin(point.theta) * direction_signal;
		double center_y = point.y - radcurv * cos(point.theta) * direction_signal;
		double va1 = carmen_normalize_theta(point.theta + 1.5707963268 * direction_signal);
		double va2;

		if (v >= 0)
		{
			va2 = va1 - temp_v * direction_signal;
		}
		else
		{
			va2 = va1 + temp_v * direction_signal;
		}

		point.x = center_x + radcurv * cos(va2);
		point.y = center_y + radcurv * sin(va2);
		point.theta = point.theta - v / radcurv * direction_signal;

		point.theta = carmen_normalize_theta(point.theta);
		point.v = v;
		point.phi = phi;

	}

	return point;
}


static void
delete_path_point(int index)
{
	int num_to_move;

	if (index >= path.path_size || index < 0)
	{
		return;
	}

	num_to_move = path.path_size - index - 1;
	memmove(path.path+index, path.path+index+1, num_to_move*sizeof(carmen_traj_point_t));

	path.path_size--;
}


static int
next_waypoint_astar(carmen_robot_and_trailer_traj_point_t *waypoint)
{
	if (path.path == NULL)
		return -1;
	//	int i;
	//	carmen_ackerman_traj_point_t pose =  *waypoint;
	//	double min_pose_dist = carmen_distance_ackerman_traj(waypoint, &path.path[0]);
	//	double pose_dist = 0;
	//	double temp;
	//	for (i = 1; i <= 30; i++)
	//	{
	//		pose_dist = carmen_distance_ackerman_traj(&pose, &path.path[0]);
	//		if (pose_dist < 0.1 ||  pose_dist > min_pose_dist)
	//		{
	//			path.path[0].v = (path.path[0].v / fabs(path.path[0].v)) * (i / 20.0);
	//			break;
	//		}
	//		printf("pose_dist %f x %f y %f\n",pose_dist, pose.x, pose.y);
	//		min_pose_dist = pose_dist;
	//		pose = predict_new_robot_position(*waypoint, 1, path.path[0].phi, (i / 20.0), &carmen_robot_ackerman_config);
	//	}
	//	printf("v %f d_p %f i %d %f\n",path.path[0].v, carmen_distance_ackerman_traj(waypoint, &path.path[0]), i, temp);

	double delta_theta = fabs(carmen_normalize_theta(waypoint->theta - path.path[0].theta));

	replan = 0;
	if (carmen_distance_ackerman_traj(waypoint, &path.path[0]) <= 0.3 && delta_theta < carmen_degrees_to_radians(1.5))
	{
		delete_path_point(0);
		min_delta_d = INTMAX_MAX;
		if (path.path_size <= 1)
		{
			replan = 1;
			return 1;

		}
	}

	if (min_delta_d != -1 && carmen_distance_ackerman_traj(waypoint, &path.path[0]) - min_delta_d > 0.5)
	{
		replan = 1;
		return -1;
	}
	min_delta_d = carmen_fmin(carmen_distance_ackerman_traj(waypoint, &path.path[0]), min_delta_d);
	return 0;
}


static int
next_waypoint(carmen_robot_and_trailer_traj_point_t *waypoint, int* waypoint_index)
{
	carmen_robot_and_trailer_traj_point_t *point;
	int next_point;
	double delta_dist;

	if (path.path_size <= 1)
		return -1;

	next_point = 0;
	do
	{
		next_point++;

		point = next_point < path.path_size ? (path.path + next_point) : NULL;

		if (path.path_size - next_point < 2)
			break;
		delta_dist = carmen_distance_ackerman_traj(waypoint, point);
	} while (delta_dist < 0.3/*nav_conf->waypoint_tolerance*/);

	delta_dist = carmen_distance_ackerman_traj(waypoint, point);

	if (delta_dist < 0.4/*nav_conf->goal_size*/ && path.path_size - next_point == 1)
		return 1;

	*waypoint = *point;
	*waypoint_index = next_point;

	if (next_point == 2)
		delete_path_point(0);

	return 0;
}


static carmen_robot_and_trailer_traj_point_t
get_center_of_the_car_front_axel(carmen_robot_and_trailer_traj_point_t current_robot_position)
{
	carmen_robot_and_trailer_traj_point_t center_of_the_car_front_axel;

	center_of_the_car_front_axel.x = current_robot_position.x + carmen_robot_ackerman_config.distance_between_front_and_rear_axles * cos(current_robot_position.theta);
	center_of_the_car_front_axel.y = current_robot_position.y + carmen_robot_ackerman_config.distance_between_front_and_rear_axles * sin(current_robot_position.theta);

	return (center_of_the_car_front_axel);
}


static double
dist2(carmen_robot_and_trailer_traj_point_t v, carmen_robot_and_trailer_traj_point_t w)
{ 
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}


static carmen_robot_and_trailer_traj_point_t
get_the_point_nearest_to_the_trajectory(int *point_in_trajectory_is,
		carmen_robot_and_trailer_traj_point_t current_robot_position,
		carmen_robot_and_trailer_traj_point_t waypoint,
		carmen_robot_and_trailer_traj_point_t center_of_the_car_front_axel)
{

#define	POINT_WITHIN_SEGMENT		0
#define	SEGMENT_TOO_SHORT		1
#define	POINT_BEFORE_SEGMENT	2
#define	POINT_AFTER_SEGMENT			3

	// Return minimum distance between line segment vw and point p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	carmen_robot_and_trailer_traj_point_t v, w, p;
	double l2, t;

	p.x = center_of_the_car_front_axel.x;
	p.y = center_of_the_car_front_axel.y;
	p.v = 0;
	p.phi = 0;
	p.theta = 0;

	v.x = current_robot_position.x;
	v.y = current_robot_position.y;
	v.v = 0;
	v.phi = 0;
	v.theta = 0;

	w.x = waypoint.x;
	w.y = waypoint.y;
	w.v = 0;
	w.phi = 0;
	w.theta = 0;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.
	if (l2 < 0.1)	  // v ~== w case // @@@ Alberto: Checar isso
	{
		*point_in_trajectory_is = SEGMENT_TOO_SHORT;
		return (v);
	}

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	if (t < 0.0) 	// p beyond the v end of the segment
	{
		*point_in_trajectory_is = POINT_BEFORE_SEGMENT;
		return (v);
	}
	if (t > 1.0)	// p beyond the w end of the segment
	{
		*point_in_trajectory_is = POINT_AFTER_SEGMENT;
		return (w);
	}

	// Projection falls on the segment
	p.x = v.x + t * (w.x - v.x);
	p.y = v.y + t * (w.y - v.y);
	*point_in_trajectory_is = POINT_WITHIN_SEGMENT;

	return (p);
}


static double
get_acceleration(double v, double target_v, carmen_robot_ackerman_config_t *ackerman_config)
{
	double acceleration;

	if (fabs(target_v) > fabs(v))
	{
		acceleration = target_v >= 0.0 ? ackerman_config->maximum_acceleration_forward : ackerman_config->maximum_acceleration_reverse;
	}
	else
	{
		acceleration = target_v >= 0.0 ? ackerman_config->maximum_deceleration_forward : ackerman_config->maximum_deceleration_reverse;
	}

	return acceleration;
}


static double
get_motion_command_v(double v, double desired_v, double time)
{
	double max_acceleration;
	double new_v, acceleration;	// @@@ nao estou considerando frenagens...

	if (time == 0)
		return v;

	max_acceleration = get_acceleration(v, desired_v, &carmen_robot_ackerman_config);
	acceleration = (desired_v - v) / time; // @@@ estou supondo que a velocidade esta sempre abaixo da maxima...
	if (fabs(acceleration) > max_acceleration)	// @@@ tem que tratar o sinal de acceleration (quande de uma re)
		acceleration = max_acceleration * (acceleration / fabs(acceleration));

	new_v = v + acceleration * time;

	return (new_v);
}


static int
phi_signal_negative(carmen_robot_and_trailer_traj_point_t origin, carmen_robot_and_trailer_traj_point_t source_v1,
		carmen_robot_and_trailer_traj_point_t source_v2)
{
	carmen_robot_and_trailer_traj_point_t v1, v2;
	double signal;

	v1.x = source_v1.x - origin.x;
	v1.y = source_v1.y - origin.y;

	v2.x = source_v2.x - origin.x;
	v2.y = source_v2.y - origin.y;

	signal = carmen_normalize_theta(atan2(v2.y, v2.x) - atan2(v1.y, v1.x));
	if (signal < 0.0)
		return (1);
	else
		return (0);
}


static double
get_motion_command_phi(double phi, double time, carmen_robot_and_trailer_traj_point_t current_robot_position,
		carmen_robot_and_trailer_traj_point_t center_of_the_car_front_axel, carmen_robot_and_trailer_traj_point_t point_in_trajectory)
{
	double desired_phi, new_phi, v, efa;
	double max_phi;
	double maximum_steering_command_rate_in_degrees;
	double phi_acceleration;

	v = current_robot_position.v;

	// See "Stanley: The Robot that Won the DARPA Grand Challenge", pp. 684,
	// and "Automatic Steering Methods for Autonomous Automobile Path Tracking" CMU-RI-TR-09-08, pp. 15

//	phi_gain = 1.0; // ERA 1.5 // @@@ Alberto: Tinha que ler isso do carmen ini
	efa = carmen_distance_ackerman_traj(&center_of_the_car_front_axel, &point_in_trajectory);
	desired_phi = atan2(phi_gain * efa, v); // @@@ Alberto: Nao trata v negativo. O teste de sinal abaixo nao deveria ser antes desta conta?
	if (phi_signal_negative(current_robot_position, center_of_the_car_front_axel, point_in_trajectory))
		desired_phi = -desired_phi;

	max_phi = carmen_robot_ackerman_config.max_phi;

	if (fabs(desired_phi) > max_phi)
		desired_phi = (desired_phi / fabs(desired_phi)) * max_phi;

	maximum_steering_command_rate_in_degrees = carmen_get_phi_from_curvature(carmen_robot_ackerman_config.maximum_steering_command_rate,
			current_robot_position.v,
			carmen_robot_ackerman_config.understeer_coeficient,
			carmen_robot_ackerman_config.distance_between_front_and_rear_axles);

	phi_acceleration = (desired_phi - phi) / time; 	// @@@ estou supondo que phi esta sempre abaixo do maximo...
	if (fabs(phi_acceleration) > maximum_steering_command_rate_in_degrees)	// @@@ tem que tratar o sinal de acceleration (quande de uma re)
		phi_acceleration = maximum_steering_command_rate_in_degrees * (phi_acceleration / fabs(phi_acceleration));

	new_phi = phi + phi_acceleration * time;


	return (new_phi);
}


static double
get_motion_command_time(carmen_robot_and_trailer_traj_point_t current_robot_position __attribute__ ((unused)))
{

	return (0.05);

	/*	double v;


	v = fabs(current_robot_position.v);

	if (v == 0.0)
		return (0.05); // @@@ minimum time to update robot planned trajectory: nao deveria estar no carmen.ini? Tem um modo certo de calcular?
	if ((1.0 / v) > 0.05)	// time to travel 1 meter > 0.02s
		return (0.05); // minimum time to update robot planned trajectory

	return (0.5 / v); // time to travel 0.5 meter
	 */
}


void
print_motion_command(int i, carmen_robot_and_trailer_motion_command_t motion_command)
{
	printf("i = %d, v = %lf, phi = %lf, t = %lf\n", i, motion_command.v, motion_command.phi, motion_command.time);
}


void
print_motion_command_vector(carmen_robot_and_trailer_motion_command_t *motion_commands_vector, int num_motion_commands)
{
	int i;

	for (i = 0; i < num_motion_commands; i++)
		print_motion_command(i, motion_commands_vector[i]);
	printf("\n");
}


int
get_nearest_trajectory_point_index(carmen_robot_and_trailer_traj_point_t *trajectory, int trajectory_length,
		carmen_robot_and_trailer_traj_point_t *front_pose)
{
	int closest_index, i;
	double min_distance, distance;

	min_distance = DBL_MAX;
	closest_index = trajectory_length - 1;

	for (i = 0; i < trajectory_length; i++)
	{
		distance = carmen_distance_ackerman_traj(&trajectory[i], front_pose);

		if (distance < min_distance)
		{
			min_distance = distance;
			closest_index = i;
		}
	}

	return closest_index;
}

int
motion_planner_compute_trajectory(carmen_robot_and_trailer_motion_command_t *next_motion_commands_vector,
		carmen_robot_and_trailer_traj_point_t *trajectory, int trajectory_length, carmen_robot_and_trailer_traj_point_t current_robot_position,
		int num_motion_commands, double time_budget, double max_v)
{
	int trajectory_index, motion_command_index;
	carmen_robot_and_trailer_traj_point_t center_of_the_car_front_axel, point_in_trajectory;
	int nearest_point_index;

	trajectory_index = motion_command_index = 0;

	center_of_the_car_front_axel = get_center_of_the_car_front_axel(current_robot_position);

	while ((time_budget > 0.0) && (motion_command_index < num_motion_commands) && (trajectory_index < trajectory_length))
	{
		nearest_point_index = get_nearest_trajectory_point_index(trajectory, trajectory_length, &center_of_the_car_front_axel);

		if (nearest_point_index == (trajectory_length - 1))
		{
			next_motion_commands_vector[motion_command_index].v = 0.0;
			next_motion_commands_vector[motion_command_index].phi = current_robot_position.phi;
			next_motion_commands_vector[motion_command_index].time = get_motion_command_time(current_robot_position);

			motion_command_index++;

			return motion_command_index;
		}

		point_in_trajectory = trajectory[nearest_point_index];

		next_motion_commands_vector[motion_command_index].v = get_motion_command_v(current_robot_position.v, max_v, next_motion_commands_vector[motion_command_index].time);
		next_motion_commands_vector[motion_command_index].phi =  get_motion_command_phi(current_robot_position.phi, next_motion_commands_vector[motion_command_index].time, current_robot_position, center_of_the_car_front_axel, point_in_trajectory);
		next_motion_commands_vector[motion_command_index].time = get_motion_command_time(current_robot_position);

		current_robot_position = predict_new_robot_position(current_robot_position, next_motion_commands_vector[motion_command_index].v, next_motion_commands_vector[motion_command_index].phi, next_motion_commands_vector[motion_command_index].time, &carmen_robot_ackerman_config);
		center_of_the_car_front_axel = get_center_of_the_car_front_axel(current_robot_position);

		time_budget -= next_motion_commands_vector[motion_command_index].time;
		motion_command_index++;
	}

	return motion_command_index;
}


int
motion_planner_compute_trajectory_old(carmen_robot_and_trailer_motion_command_t *next_motion_commands_vector,
		carmen_robot_and_trailer_traj_point_t *trajectory, int trajectory_length, carmen_robot_and_trailer_traj_point_t current_robot_position,
		int num_motion_commands, double time_budget, double max_v)
{
	int trajectory_index, motion_command_index;
	carmen_robot_and_trailer_traj_point_t center_of_the_car_front_axel, point_in_trajectory;
	int point_in_trajectory_is;

	trajectory_index = motion_command_index = 0;
	while ((trajectory_index < trajectory_length) && (motion_command_index < num_motion_commands) && (time_budget > 0.0))
	{
		center_of_the_car_front_axel = get_center_of_the_car_front_axel(current_robot_position);
		while ((time_budget > 0.0) && (motion_command_index < num_motion_commands) && (trajectory_index < trajectory_length))
		{
			point_in_trajectory = get_the_point_nearest_to_the_trajectory(&point_in_trajectory_is, current_robot_position, trajectory[trajectory_index], center_of_the_car_front_axel);

			if (point_in_trajectory_is == POINT_WITHIN_SEGMENT)
			{
				next_motion_commands_vector[motion_command_index].time = get_motion_command_time(current_robot_position);
				next_motion_commands_vector[motion_command_index].phi = current_robot_position.phi = get_motion_command_phi(current_robot_position.phi, next_motion_commands_vector[motion_command_index].time, current_robot_position, center_of_the_car_front_axel, point_in_trajectory);
				next_motion_commands_vector[motion_command_index].v = current_robot_position.v = get_motion_command_v(current_robot_position.v, max_v, next_motion_commands_vector[motion_command_index].time);
				current_robot_position = predict_new_robot_position(current_robot_position, next_motion_commands_vector[motion_command_index].v, next_motion_commands_vector[motion_command_index].phi, next_motion_commands_vector[motion_command_index].time, &carmen_robot_ackerman_config);
				time_budget -= next_motion_commands_vector[motion_command_index].time;
				motion_command_index++;
			}

			if (point_in_trajectory_is == POINT_AFTER_SEGMENT)
			{	// No movimento simulado em predict_new_robot_position(), ou no proprio plano, o eixo dianteiro do carro ultrapassou o waypoint trajectory[trajectory_index]
				trajectory_index++;
				break;
			}

			if (point_in_trajectory_is == SEGMENT_TOO_SHORT)
			{	// No movimento simulado em predict_new_robot_position(), ou no proprio plano, o carro (eixo traseiro) alcanccou exatamente o waypoint trajectory[trajectory_index]
				trajectory_index++;
				break;
			}

			if (point_in_trajectory_is == POINT_BEFORE_SEGMENT)
			{	// No movimento simulado em predict_new_robot_position(), ou no proprio plano, o carro (eixo traseiro) ultrapassou o waypoint trajectory[trajectory_index]
				trajectory_index++;
				break;
			}
		}
	}

	return (motion_command_index);
}


carmen_point_t
to_carmen_point_t (carmen_robot_and_trailer_traj_point_t *p)
{
	carmen_point_t point;

	point.x = p->x;
	point.y = p->y;
	point.theta = p->theta;

	return (point);
}


carmen_point_t 
to_map_pose(carmen_point_t world_pose, carmen_map_config_t *map_config) 
{
	carmen_point_t p;

	p.theta = world_pose.theta;
	p.x = (world_pose.x - map_config->x_origin) / map_config->resolution;
	p.y = (world_pose.y - map_config->y_origin) / map_config->resolution;

	return p;
}


int
colision_detection_is_valid_position(int x, int y, carmen_map_t *map)
{
	return x >= 0 && x < map->config.x_size && y >= 0 && y < map->config.y_size;
}


int 
pose_hit_obstacle(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config)
{
	int vertical_size, horizontal_size;
	carmen_point_t vertical_pose, horizontal_pose[2];
	double delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y, value;
	int v, h, i;

	if (map == NULL)
		return 1;

	vertical_size = ceil(car_config->length / map->config.resolution);
	horizontal_size = ceil((car_config->width / 2.0) / map->config.resolution);

	delta_vertical_x = cos(pose.theta);
	delta_vertical_y = sin(pose.theta);

	delta_horizontal_x = cos(M_PI/2 - pose.theta);
	delta_horizontal_y = sin(M_PI/2 - pose.theta);

	vertical_pose = to_map_pose(pose, &map->config);

	vertical_pose.x -= car_config->distance_between_rear_car_and_rear_wheels / map->config.resolution * cos(vertical_pose.theta);
	vertical_pose.y -= car_config->distance_between_rear_car_and_rear_wheels / map->config.resolution * sin(vertical_pose.theta);

	for (v = 0; v <= vertical_size; v++)
	{
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;

		for (h = 0; h <= horizontal_size; h++)
		{
			for (i = 0; i < 2; i++) 
			{

				if (!colision_detection_is_valid_position(horizontal_pose[i].x, horizontal_pose[i].y, map))
					return 1;

				value = map->complete_map[(int)horizontal_pose[i].x * map->config.y_size + (int)horizontal_pose[i].y];

				if (value > 0.5)
					return 1;
			}

			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;

			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}

		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}

	return 0;
}


static void 
stop_robot()
{
	int i;
	int j;
	replan = 1;

	for (i = 0; i < NUM_MOTION_COMMANDS_VECTORS; i++)
	{
		for (j = 0; j < NUM_MOTION_COMMANDS_PER_VECTOR; j++)
		{
			motion_commands_vector[i][j].v = 0.0;
			motion_commands_vector[i][j].time = 0.05;
			motion_commands_vector[i][j].phi = g_robot_position.phi;
		}
		nun_motion_commands[i] = NUM_MOTION_COMMANDS_PER_VECTOR;
	}

	current_motion_command_vetor_index = 0;
	carmen_obstacle_avoider_publish_base_ackerman_motion_command(motion_commands_vector[current_motion_command_vetor_index],
			nun_motion_commands[current_motion_command_vetor_index], carmen_get_time());
	IPC_listen(50);
	publish_obstacle_avoider_path(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);
	IPC_listen(50);

}


void
motion_planning_obstacle_avoiding_handler()
{
	carmen_robot_and_trailer_motion_command_t *next_motion_commands_vector;
	int next_motion_command_vector_index, motion_commands_trajectory_size;
	double time_budget;
	carmen_robot_and_trailer_traj_point_t current_robot_position;
	int hit_obstacle, i;
	int trajectory_lenght;
	double current_max_v;
	carmen_robot_and_trailer_traj_point_t *trajectory;
	int trajectory_length;
	carmen_robot_and_trailer_traj_point_t robot_position;
	static int already_planning = 0;

	if (current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT || current_task == BEHAVIOR_SELECTOR_PARK)
		return;

	if (already_planning)
		return;
	already_planning = 1;

	if (g_current_trajectory == NULL)
	{
		already_planning = 0;
		return;
	}

	trajectory = g_current_trajectory;
	trajectory_length = g_current_trajectory_length;
	robot_position = g_robot_position;

	if (trajectory_length <= 0) // invalid trajectory // @@@ Alberto: isso acontece? Se acontence, por que acontece?
	{
		printf("trajectory_length <= 0 in motion_planning_obstacle_avoiding_handler()\n");
		already_planning = 0;
		return;
	}

	next_motion_command_vector_index = (current_motion_command_vetor_index + 1) % NUM_MOTION_COMMANDS_VECTORS;
	next_motion_commands_vector = motion_commands_vector[next_motion_command_vector_index];

	current_robot_position.x = robot_position.x;
	current_robot_position.y = robot_position.y;
	current_robot_position.theta = robot_position.theta;
	current_robot_position.v = robot_position.v;
	current_robot_position.phi = robot_position.phi;

	time_budget = 20.0; // @@@ tem que ver como melhor determinar este valor
	current_max_v = carmen_robot_ackerman_config.max_v;
	do
	{
		motion_commands_trajectory_size = motion_planner_compute_trajectory(next_motion_commands_vector, trajectory, trajectory_length, current_robot_position, 
				NUM_MOTION_COMMANDS_PER_VECTOR, time_budget, current_max_v);

		hit_obstacle = 0;
		trajectory_lenght = build_predicted_trajectory(next_motion_commands_vector, motion_commands_trajectory_size, current_robot_position);

		for (i = 0; i < trajectory_lenght; i++)
		{
			if (pose_hit_obstacle(to_carmen_point_t(&(trajectory_vector_of_points[i])), map_vector[current_map], &carmen_robot_ackerman_config))
			{
				current_max_v *= 0.7;
				hit_obstacle = 1;
				break;
			}
		}
	} while (hit_obstacle && fabs((current_max_v /*- (current_robot_position.v / 10.0)*/)) > 0.05); // @@@ Alberto: Isso nao trata velocidades negativas...

	if ((motion_commands_trajectory_size == 0) || (autonomous_status == 0) || (hit_obstacle == 1))
	{
		stop_robot();
		already_planning = 0;
		//		printf("hit = %d\n", count++);
		return;
	}

	nun_motion_commands[next_motion_command_vector_index] = motion_commands_trajectory_size;

	current_motion_command_vetor_index = next_motion_command_vector_index;

	carmen_obstacle_avoider_publish_base_ackerman_motion_command(motion_commands_vector[current_motion_command_vetor_index],
			nun_motion_commands[current_motion_command_vetor_index], carmen_get_time());
	IPC_listen(50);
	publish_obstacle_avoider_path(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);
	IPC_listen(50);
	publish_motion_planner_path(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);
	IPC_listen(50);

	already_planning = 0;
}


static void
send_trajectory_to_robot(carmen_robot_and_trailer_traj_point_t *trajectory, int trajectory_length)
{
	int i;

	if (g_current_trajectory != NULL)
		free(g_current_trajectory);

	g_current_trajectory = (carmen_robot_and_trailer_traj_point_t *) malloc(trajectory_length * sizeof(carmen_robot_and_trailer_traj_point_t));
	g_current_trajectory_length = trajectory_length;

	for (i = 0; i < g_current_trajectory_length; i++)
		g_current_trajectory[i] = trajectory[i];
}


void
clear_current_trajectory()
{
	if (g_current_trajectory != NULL)
		free(g_current_trajectory);

	g_current_trajectory = NULL;
	g_current_trajectory_length = 0;
}


static void
generate_next_motion_command(void)
{
	carmen_robot_and_trailer_traj_point_t waypoint;
	int waypoint_index = 0;
	int waypoint_status = 0;

	if (!autonomous_status)
		return;

	waypoint = g_robot_position;
	if (current_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
		waypoint_status = next_waypoint(&waypoint, &waypoint_index);

	if (current_task == BEHAVIOR_SELECTOR_PARK)
		waypoint_status = next_waypoint_astar(&waypoint);

	if (waypoint_status > 0) /* Goal reached? */
	{
		//	clear_current_trajectory();
		stop_robot();
		return;
	}

	if (waypoint_status < 0) /* There is no path to the goal? */
	{
		//	clear_current_trajectory();
		stop_robot();
		return;
	}

	if (current_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
		send_trajectory_to_robot(path.path + waypoint_index, path.path_size - waypoint_index);

	if (current_task == BEHAVIOR_SELECTOR_PARK)
		publish_astar_path(path.path + waypoint_index, path.path_size - waypoint_index, g_robot_position);
	//teste_stop(0.1, 2.0);

}


void
motion_planner_set_robot_pose(carmen_point_t pose, double v, double phi)
{
	g_robot_position.x = pose.x;
	g_robot_position.y = pose.y;
	g_robot_position.theta = pose.theta;

	g_robot_position.v = v;
	g_robot_position.phi = phi;

	generate_next_motion_command();

	motion_planning_obstacle_avoiding_handler();
	publish_status();
	publish_plan();
}


void
motion_planner_set_path(carmen_motion_planner_path_message *msg)
{
	if (replan == 1)
	{
		if (path.path)
			free(path.path);

		path = *msg;
		path.path = (carmen_robot_and_trailer_traj_point_t *) malloc(sizeof(carmen_robot_and_trailer_traj_point_t) * path.path_size);
		memcpy(path.path, msg->path, sizeof(carmen_robot_and_trailer_traj_point_t) * path.path_size);
		min_delta_d = INTMAX_MAX;
		publish_plan();
	}
}


void
motion_planner_go()
{
	autonomous_status = 1;
	replan = 1;
}


void
motion_planner_stop()
{
	replan = 1;
	autonomous_status = 0;
}


void
motion_planner_set_algorithm(carmen_behavior_selector_algorithm_t new_algorithm, carmen_behavior_selector_task_t new_task)
{
	current_algorithm = new_algorithm;
	current_task = new_task;
}


void 
copy_grid_mapping_to_map_vector(carmen_mapper_map_message *grid_map, int position)
{
	int i;

	//printf("position = %d\n", position);
	if (map_vector[position] == NULL)
	{
		map_vector[position] = (carmen_map_t *) malloc (sizeof(carmen_map_t));
		carmen_test_alloc(map_vector[position]);
		map_vector[position]->complete_map = (double *) malloc(sizeof(double) * grid_map->size);
		carmen_test_alloc(map_vector[position]->complete_map);
		map_vector[position]->map = (double **)calloc(grid_map->config.x_size, sizeof(double *));
		carmen_test_alloc(map_vector[position]->map);
	}
	else if ((grid_map->size != (map_vector[position]->config.x_size * map_vector[position]->config.y_size)) || // o novo mapa pode ser de tamanho diferente...
			(grid_map->config.x_size != map_vector[position]->config.x_size))
	{
		free(map_vector[position]->map);
		free(map_vector[position]->complete_map);

		map_vector[position]->complete_map = (double *) malloc(sizeof(double) * grid_map->size);
		carmen_test_alloc(map_vector[position]->complete_map);
		map_vector[position]->map = (double **)calloc(grid_map->config.x_size, sizeof(double *));
		carmen_test_alloc(map_vector[position]->map);
	}

	map_vector[position]->config = grid_map->config;

	memcpy(map_vector[position]->complete_map, grid_map->complete_map, sizeof(double) * grid_map->size);

	for (i = 0; i < map_vector[position]->config.x_size; i++)
	{
		map_vector[position]->map[i] = map_vector[position]->complete_map + i * map_vector[position]->config.y_size;
	}
}

//
//void
//teste_stop(double space_interval, double vel)
//{
//
//	//todo time = space_interval / v ........ implementar isso depois
//	carmen_ackerman_motion_command_p next_motion_commands_vector;
//	int next_motion_command_vector_index;
//	next_motion_command_vector_index = (current_motion_command_vetor_index + 1) % NUM_MOTION_COMMANDS_VECTORS;
//	next_motion_commands_vector = motion_commands_vector[next_motion_command_vector_index];
//	double v  = vel;
//	int i;
//	int motion_command_index = 0;
//
//	int num_of_inicial_commads = 10;
//
//	for (i = 0; i < num_of_inicial_commads; i++)
//	{
//		next_motion_commands_vector[motion_command_index].phi = 0;
//		next_motion_commands_vector[motion_command_index].v = v;
//		next_motion_commands_vector[motion_command_index].time = space_interval;
//		motion_command_index++;
//	}
//
//	double velocity_fuction;
//
//	for (i = motion_command_index; i < NUM_MOTION_COMMANDS_PER_VECTOR; i++)
//	{
//		velocity_fuction =  v * 0.8;
//		//		velocity_fuction = v - vel * 0.01;
//
//		v = get_motion_command_v(v, velocity_fuction, space_interval);
//
//		printf("velocity_fuction %f v %f time %f i %d\n",velocity_fuction,v, space_interval,i);
//		if (v < 0.01)
//		{
//			next_motion_commands_vector[motion_command_index].phi = 0;
//			next_motion_commands_vector[motion_command_index].v = 0;
//			next_motion_commands_vector[motion_command_index].time = space_interval;
//			break;
//		}
//
//		next_motion_commands_vector[motion_command_index].phi = 0;
//		next_motion_commands_vector[motion_command_index].v = v;
//		next_motion_commands_vector[motion_command_index].time = space_interval;
//
//		motion_command_index++;
//	}
//
//	nun_motion_commands[next_motion_command_vector_index] = motion_command_index;
//	current_motion_command_vetor_index = next_motion_command_vector_index;
//	printf("fimm\n");
//	carmen_robot_ackerman_send_base_ackerman_motion_command(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);
//	publish_obstacle_avoider_path(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);
//	publish_motion_planner_path(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);
//	usleep(5000000);
//}



void
publish_astar_path(carmen_robot_and_trailer_traj_point_t *path, int path_size, carmen_robot_and_trailer_traj_point_t robot_position)
{
	carmen_robot_and_trailer_motion_command_t *next_motion_commands_vector;
	int next_motion_command_vector_index;
	if (path_size <= 0)
	{
		printf("trajectory_length <= 0 in send_trajectory_to_robot()\n");
		return;
	}

	next_motion_command_vector_index = (current_motion_command_vetor_index + 1) % NUM_MOTION_COMMANDS_VECTORS;
	next_motion_commands_vector = motion_commands_vector[next_motion_command_vector_index];

	int i, j;
	double velocity_signal;
	int motion_command_index = 0;

	double same_phi_count = 0;
	double same_phi_count_2 = 0;

	//double same_phi_index = 0;

	for (i = 0; i < path_size; i++)
	{
		if (path[0].phi == path[i].phi)
		{
			carmen_robot_and_trailer_traj_point_t temp_point;
			temp_point = robot_position;

			if(i == 0)
				for(j = 0; j < 1000; j++)
				{
					temp_point = carmen_conventional_astar_ackerman_kinematic_3(temp_point, 2.16, path[0].phi, 0.01 * (path[i].v / fabs(path[i].v)));
					same_phi_count_2 += 0.01;
					if (fabs(temp_point.x - path[i].x) < 0.1 &&  fabs(temp_point.y - path[i].y) < 0.1 )
					{
						break;
					}

					if(j == 999)
					{
						printf("j = 999\n");
						same_phi_count_2 = 0;

					}
				}
			else
				same_phi_count_2 += fabs(path[i].v);

			same_phi_count += fabs(path[i].v);
		}
		else
		{
			//same_phi_index = i + 1;
			break;
		}
	}

//	printf("same_phi_count %.2f\n",same_phi_count);
//	printf("same_phi_count_2 %.2f\n",same_phi_count_2);


	same_phi_count_2 = same_phi_count_2 / 4;
	if (same_phi_count_2 >= 1.5)
		same_phi_count_2 = 1.5;
	if (same_phi_count_2 < 0.5)
		same_phi_count_2 = 0.5;

	if(fabs(path[0].phi - robot_position.phi) > carmen_degrees_to_radians(1.5))//todo AQUI tem que virar parametro
	{
		next_motion_commands_vector[motion_command_index].v = 0;
		next_motion_commands_vector[motion_command_index].phi = path[0].phi;
//		double signal_phi = (path[0].phi - robot_position.phi) / fabs(path[0].phi - robot_position.phi);
		//next_motion_commands_vector[motion_command_index].phi = robot_position.phi + carmen_degrees_to_radians(1.5 * signal_phi);
		next_motion_commands_vector[motion_command_index].time = fabs(path[0].v);
		motion_command_index++;
	}
	else
	{
		for (i = 0; i < 1; i++)//todo colocado soh 1 //acertar isso
		{
			if (path[i].v != 0)
				velocity_signal = (fabs(path[i].v) / path[i].v);
			else
			{
				printf("nao era pra entrar aqui.. algo errado?\n");
				velocity_signal = 0;

			}
			next_motion_commands_vector[motion_command_index].phi = path[i].phi;
			next_motion_commands_vector[motion_command_index].v = velocity_signal * same_phi_count_2;
			next_motion_commands_vector[motion_command_index].time = fabs(path[i].v) / same_phi_count_2;
			motion_command_index++;
		}
	}
	nun_motion_commands[next_motion_command_vector_index] = motion_command_index;
	current_motion_command_vetor_index = next_motion_command_vector_index;

	carmen_obstacle_avoider_publish_base_ackerman_motion_command(motion_commands_vector[current_motion_command_vetor_index],
			nun_motion_commands[current_motion_command_vetor_index], carmen_get_time());
	publish_obstacle_avoider_path(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);
	publish_motion_planner_path(motion_commands_vector[current_motion_command_vetor_index], nun_motion_commands[current_motion_command_vetor_index]);

}


int 
motion_planner_read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.max_v, 1, NULL},
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.max_phi, 1, NULL},
			{"robot", "min_approach_dist", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.approach_dist, 1, NULL},
			{"robot", "min_side_dist", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.side_dist, 1, NULL},
			{"robot", "length", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.length, 0, NULL},
			{"robot", "width", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.width, 0, NULL},
			{"robot", "reaction_time", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.reaction_time, 0, NULL},
			{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_rear_wheels, 1,NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_front_and_rear_axles, 1, NULL},
			{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{"robot", "allow_rear_motion", CARMEN_PARAM_ONOFF, &carmen_robot_ackerman_config.allow_rear_motion, 1, NULL},
			{"robot", "interpolate_odometry", CARMEN_PARAM_ONOFF, &carmen_robot_ackerman_config.interpolate_odometry, 1, NULL},
			{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &(carmen_robot_ackerman_config.maximum_acceleration_forward), 0, NULL},
			{"robot", "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &(carmen_robot_ackerman_config.maximum_deceleration_forward), 0, NULL},
			{"robot", "maximum_acceleration_reverse", CARMEN_PARAM_DOUBLE, &(carmen_robot_ackerman_config.maximum_acceleration_reverse), 0, NULL},
			{"robot", "maximum_deceleration_reverse", CARMEN_PARAM_DOUBLE, &(carmen_robot_ackerman_config.maximum_deceleration_reverse), 0, NULL},

			{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.understeer_coeficient, 1, NULL},
			{"robot", "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.maximum_steering_command_rate, 1, NULL},
			{"motion_planner", "phi_gain", CARMEN_PARAM_DOUBLE, &phi_gain, 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}
