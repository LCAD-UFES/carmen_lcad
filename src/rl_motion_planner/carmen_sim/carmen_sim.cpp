
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/simulator_ackerman_simulation.h>
#include <carmen/collision_detection.h>
#include "carmen_sim.h"

#include <opencv/cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;


/*
 *********************************
 * PUBLIC METHODS
 *********************************
*/

CarmenSim::CarmenSim(bool fix_initial_position, bool use_truepos,
		bool allow_negative_commands,
		bool enjoy_mode,
		bool use_latency,
		const char *rddf_name,
		const char *map_dir,
		double min_dist_to_initial_goal,
		double max_dist_to_initial_goal)
{
	_fix_initial_position = fix_initial_position;
	_use_truepos = use_truepos;
    _rddf_name = rddf_name;
    _allow_negative_commands = allow_negative_commands;
    _enjoy_mode = enjoy_mode;
    _use_latency = use_latency;

	_min_pose_skip_to_initial_goal = (int) (min_dist_to_initial_goal / 0.5);
	_max_pose_skip_to_initial_goal = (int) (max_dist_to_initial_goal / 0.5);

    string carmen_home = getenv("CARMEN_HOME");
    string rddf_path = carmen_home + "/data/rndf/" + _rddf_name;

    memset(&_simulator_config, 0, sizeof(_simulator_config));
    memset(&_obstacle_distance_map, 0, sizeof(_obstacle_distance_map));

    _load_rddf(rddf_path);
    _load_params();

    _map_dir = carmen_home + "/data/" + map_dir;
	_map_resolution = 0.2;
	_viewer_subsampling = 2;
	_map_pixel_by_meter = 1. / (_map_resolution * _viewer_subsampling);

	carmen_grid_mapping_init_parameters(_map_resolution, 210);

    _car_length = (_robot_ackerman_config.distance_between_front_and_rear_axles +
    		_robot_ackerman_config.distance_between_rear_car_and_rear_wheels +
			_robot_ackerman_config.distance_between_front_car_and_front_wheels);

	_car_width = _robot_ackerman_config.distance_between_rear_wheels;

	_front_laser.range = (double *) calloc (_simulator_config.front_laser_config.num_lasers, sizeof(double));
	carmen_test_alloc(_front_laser.range);
	_front_laser.num_remissions = 0;
	_front_laser.remission = 0;

	_rear_laser.range = (double *) calloc (_simulator_config.rear_laser_config.num_lasers, sizeof(double));
	carmen_test_alloc(_rear_laser.range);
	_rear_laser.num_remissions = 0;
	_rear_laser.remission = 0;
}


void
CarmenSim::set_seed(int seed)
{
	srand(seed);
}


void
CarmenSim::reset()
{
	int direction, M, m;

	M = _max_pose_skip_to_initial_goal;
	m = _min_pose_skip_to_initial_goal;

	if (_allow_negative_commands)
		direction = (rand() % 2) * 2 - 1;  // -1 or 1
	else
		direction = 1;

	int pose_id;

	if (_fix_initial_position) pose_id = 500; //M;
	else pose_id = M + rand() % (_rddf.size() - M);

	int shift = m + (rand() % (M - m));  // shift is a random integer between m and M

	if (_enjoy_mode)
	{
		shift = 10;
		direction = 1;
	}

	int goal_id = pose_id + shift * direction;

	carmen_ackerman_motion_command_t pose = _rddf[pose_id];
	_goal = _rddf[goal_id];

	// reset simulator
	_simulator_config.true_pose.x = pose.x;
	_simulator_config.true_pose.y = pose.y;
	_simulator_config.true_pose.theta = pose.theta;
	_simulator_config.odom_pose.theta = pose.theta;
	_simulator_config.v = 0.;
	_simulator_config.phi = 0.;
	_simulator_config.target_v = 0.;
	_simulator_config.target_phi = 0.;
	_simulator_config.current_motion_command_vector = NULL;
	_simulator_config.nun_motion_commands = 0;
	_simulator_config.current_motion_command_vector_index = 0;
	_simulator_config.initialize_neural_networks = 1;

	_update_map();

	carmen_simulator_ackerman_calc_laser_msg(&_front_laser, &_simulator_config, 0);
	carmen_simulator_ackerman_calc_laser_msg(&_rear_laser, &_simulator_config, 1);

	//_my_counter = 0;
}


// Copied from carmen. TODO: Find where this is defined, make it public, and import.
int
apply_system_latencies(carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands)
{
	int i, j;

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;

		for (double lat = 0.0; lat < 0.2; j++)
		{
			if (j >= nun_motion_commands)
				break;

			lat += current_motion_command_vector[j].time;
		}

		if (j >= nun_motion_commands)
			break;

		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
	}

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;

		for (double lat = 0.0; lat < 0.6; j++)
		{
			if (j >= nun_motion_commands)
				break;

			lat += current_motion_command_vector[j].time;
		}

		if (j >= nun_motion_commands)
			break;

		current_motion_command_vector[i].v = current_motion_command_vector[j].v;
	}

	return (i);
}


// TODO: use some implementation from carmen.
void
ackerman_motion_model(double *x, double *y, double *th, double v, double phi, double dt,
		double distance_between_front_and_rear_axles)
{
	(*x) +=  v * dt * cos(*th);
	(*y) +=  v * dt * sin(*th);
	(*th) += v * dt * tan(phi) / distance_between_front_and_rear_axles;
	(*th) = carmen_normalize_theta(*th);
}


void
CarmenSim::step(double v, double phi, double dt)
{
	static const int N_COMMANDS = 1;
	static carmen_ackerman_motion_command_t *cmds = NULL;

	if (cmds == NULL)
		cmds = (carmen_ackerman_motion_command_t *) calloc (N_COMMANDS, sizeof(carmen_ackerman_motion_command_t));

	for (int i = 0; i < N_COMMANDS; i++)
	{
		cmds[i].x = 0.0;
		cmds[i].y = 0.0;
		cmds[i].theta = 0.0;
		cmds[i].v = v;
		cmds[i].phi = phi;
		cmds[i].time = dt;
	}

	_simulator_config.current_motion_command_vector = cmds;
	_simulator_config.nun_motion_commands = N_COMMANDS;
	_simulator_config.time_of_last_command = carmen_get_time();
	_simulator_config.delta_t = dt;

	if (_use_latency)
	{
		carmen_simulator_ackerman_recalc_pos(&_simulator_config, _use_velocity_nn, _use_phi_nn);
	}
	else
	{
		_simulator_config.v = v;
		_simulator_config.phi = phi;

		ackerman_motion_model(&(_simulator_config.true_pose.x),
			&(_simulator_config.true_pose.y),
			&(_simulator_config.true_pose.theta),
			_simulator_config.v,
			_simulator_config.phi,
			_simulator_config.delta_t,
			_simulator_config.distance_between_front_and_rear_axles);
	}

    _update_map();

    if (_enjoy_mode)
    	_goal = _rddf[_find_nearest_goal(_simulator_config.true_pose.x, _simulator_config.true_pose.y) + 10];

	carmen_simulator_ackerman_calc_laser_msg(&_front_laser, &_simulator_config, 0);
	carmen_simulator_ackerman_calc_laser_msg(&_rear_laser, &_simulator_config, 1);
}


void
draw_rectangle(Mat &img,
		double x, double y, double theta,
		double height, double width, Scalar color,
		double x_origin, double y_origin, double pixels_by_meter)
{
	vector<Point2f> vertices;
	vector<Point> vertices_px;

	vertices.push_back(Point2f(-width / 2., -height / 2.));
	vertices.push_back(Point2f(-width / 2., height / 2.));
	vertices.push_back(Point2f(width / 2., height / 2.));
	vertices.push_back(Point2f(width / 2., 0.));
	vertices.push_back(Point2f(0., 0.));
	vertices.push_back(Point2f(width / 2., 0));
	vertices.push_back(Point2f(width / 2., -height / 2.));

	double v_radius, v_angle;

	// transform vertices
	for (int i = 0; i < vertices.size(); i++)
	{
		v_radius = sqrt(pow(vertices[i].x, 2.) + pow(vertices[i].y, 2.));
		v_angle = atan2(vertices[i].y, vertices[i].x);

	    vertices[i].x = v_radius * cos(v_angle + theta) + x;
	    vertices[i].y = v_radius * sin(v_angle + theta) + y;

	    Point p;
	    p.x = (int) ((vertices[i].x - x_origin) * pixels_by_meter);
	    p.y = (int) ((vertices[i].y - y_origin) * pixels_by_meter);
	    p.y = img.rows - p.y;

	    vertices_px.push_back(p);
	}

	for (int i = 0; i < vertices_px.size(); i++)
	{
		if (i == vertices_px.size() - 1)
			line(img, vertices_px[i], vertices_px[0], color, 1);
		else
			line(img, vertices_px[i], vertices_px[i + 1], color, 1);
	}
}


void
CarmenSim::view()
{
	int x_size = _simulator_config.map.config.x_size;
	int y_size = _simulator_config.map.config.y_size;
	int x_origin = _simulator_config.map.config.x_origin;
	int y_origin = _simulator_config.map.config.y_origin;
	double **map = _simulator_config.map.map;

	Mat viewer = Mat(Size(x_size / _viewer_subsampling, y_size / _viewer_subsampling), CV_8UC3);

	for (int i = 0, pi = 0; i < y_size; i += _viewer_subsampling, pi++)
	{
		for (int j = 0, pj = 0; j < x_size; j += _viewer_subsampling, pj++)
		{
			if (map[i][j] < 0.)
			{
				viewer.data[3 * (pi * viewer.cols + pj) + 0] = (uchar) 255;
				viewer.data[3 * (pi * viewer.cols + pj) + 1] = (uchar) 0;
				viewer.data[3 * (pi * viewer.cols + pj) + 2] = (uchar) 0;
			}
			else
			{
				double free_prob = 1. - map[i][j];
				viewer.data[3 * (pi * viewer.cols + pj) + 0] = (uchar) (255. * free_prob);
				viewer.data[3 * (pi * viewer.cols + pj) + 1] = (uchar) (255. * free_prob);
				viewer.data[3 * (pi * viewer.cols + pj) + 2] = (uchar) (255. * free_prob);
			}
		}
	}

	cv::transpose(viewer, viewer);
	flip(viewer, viewer, 0);

    double center_to_rear_axis = _car_length / 2. - _robot_ackerman_config.distance_between_rear_car_and_rear_wheels;
    double shift_x = center_to_rear_axis * cos(_simulator_config.true_pose.theta);
    double shift_y = center_to_rear_axis * sin(_simulator_config.true_pose.theta);

    Point p;
    p.x = (int) ((_simulator_config.true_pose.x - _simulator_config.map.config.x_origin) * _map_pixel_by_meter);
    p.y = viewer.rows - (int) ((_simulator_config.true_pose.y - _simulator_config.map.config.y_origin) * _map_pixel_by_meter);
    circle(viewer, p, 3, Scalar(0, 0, 255), -1);

	draw_rectangle(viewer,
			_simulator_config.true_pose.x + shift_x,
			_simulator_config.true_pose.y + shift_y,
			_simulator_config.true_pose.theta,
			_car_width, _car_length, Scalar(0, 0, 255),
			x_origin, y_origin,
			_map_pixel_by_meter);

    p.x = (int) ((_goal.x - _simulator_config.map.config.x_origin) * _map_pixel_by_meter);
    p.y = viewer.rows - (int) ((_goal.y - _simulator_config.map.config.y_origin) * _map_pixel_by_meter);
    circle(viewer, p, 3, Scalar(0, 255, 0), -1);
    circle(viewer, p, _map_pixel_by_meter, Scalar(0, 255, 0), 1);

    shift_x = center_to_rear_axis * cos(_goal.theta);
    shift_y = center_to_rear_axis * sin(_goal.theta);

    draw_rectangle(viewer, _goal.x + shift_x, _goal.y + shift_y, _goal.theta,
			_car_width, _car_length, Scalar(0, 128, 0),
			x_origin, y_origin,
			_map_pixel_by_meter);

	imshow("viewer", viewer);
	waitKey(1);
}


vector<double>
CarmenSim::laser()
{
	int i;
	std::vector<double> readings;

	for (i = 0; i < _front_laser.num_readings; i++)
		readings.push_back(_front_laser.range[i]);

	for (i = 0; i < _rear_laser.num_readings; i++)
		readings.push_back(_rear_laser.range[i]);

	return readings;
}


vector<double>
CarmenSim::pose()
{
	vector<double> data;

	data.push_back(_simulator_config.true_pose.x);
	data.push_back(_simulator_config.true_pose.y);
	data.push_back(_simulator_config.true_pose.theta);
	data.push_back(_simulator_config.v);
	data.push_back(_simulator_config.phi);

	return data;
}


vector<double>
CarmenSim::goal()
{
	vector<double> data;

	data.push_back(_goal.x);
	data.push_back(_goal.y);
	data.push_back(_goal.theta);
	data.push_back(_goal.v);
	data.push_back(_goal.phi);

	return data;
}


bool
CarmenSim::hit_obstacle()
{
	carmen_ackerman_traj_point_t tpose;

	tpose.x = _simulator_config.true_pose.x;
	tpose.y = _simulator_config.true_pose.y;
	tpose.theta = _simulator_config.true_pose.theta;
	tpose.v = _simulator_config.v;
	tpose.phi = _simulator_config.phi;

	carmen_obstacle_distance_mapper_map_message mess;
	memset(&mess, 0, sizeof(mess));

	mess.config = _obstacle_distance_map.config;
	mess.complete_x_offset = _obstacle_distance_map.complete_x_offset;
	mess.complete_y_offset = _obstacle_distance_map.complete_y_offset;
	mess.size = mess.config.x_size * mess.config.y_size;

	int hit = trajectory_pose_hit_obstacle(tpose,
		_robot_ackerman_config.obstacle_avoider_obstacles_safe_distance,
		&mess, &_robot_ackerman_config);

	return (hit == 1);
}


/*
 *********************************
 * PRIVATE METHODS
 *********************************
*/

void
CarmenSim::_load_rddf(string path)
{
	FILE *fptr = fopen(path.c_str(), "r");

	if (fptr == NULL)
		exit(printf("Could not open file '%s'", path.c_str()));

	_rddf.clear();

	while (!feof(fptr))
	{
		carmen_ackerman_motion_command_t p;

		fscanf(fptr, "%lf %lf %lf %lf %lf %lf",
				&p.x, &p.y, &p.theta, &p.v, &p.phi, &p.time);

		_rddf.push_back(p);
	}

	fclose(fptr);
}


void
CarmenSim::_update_map()
{
	carmen_point_t p;

	p.x = _simulator_config.true_pose.x;
	p.y = _simulator_config.true_pose.y;
	p.theta = _simulator_config.true_pose.theta;

	// check if there is map that contains the current pose.
	double x_or, y_or;
	carmen_grid_mapping_get_map_origin(&p, &x_or, &y_or);

	if (x_or == _simulator_config.map.config.x_origin && y_or == _simulator_config.map.config.y_origin)
		return;

	static char map_path[1024];
	sprintf(map_path, "%s/m%d_%d.map", _map_dir.c_str(), (int) x_or, (int) y_or);
	FILE *fptr = fopen(map_path, "r");

	if (fptr == NULL)
	{
		_robot_is_on_map = false;
		printf("Warning: There is not a map that contains the pose (%lf, %lf)\n", p.x, p.y);
		return;
	}
	else
		fclose(fptr);

	carmen_grid_mapping_get_block_map_by_origin((char *) _map_dir.c_str(), 'm', p,
												&_simulator_config.map);

	// recompute obstacle distance map
	if (_obstacle_distance_map.complete_distance == NULL)
		carmen_prob_models_initialize_distance_map(&_obstacle_distance_map,
													_simulator_config.map.config);

	carmen_prob_models_create_distance_map(&_obstacle_distance_map, &_simulator_config.map, 0.5);
}


int
CarmenSim::_find_nearest_goal(double x, double y)
{
	int p = -1;
	double dist, min_dist = DBL_MAX;

	for (int i = 0; i < _rddf.size(); i++)
	{
		dist = pow(x - _rddf[i].x, 2) + pow(y - _rddf[i].y, 2);

		if (dist < min_dist)
		{
			min_dist = dist;
			p = i;
		}
	}

	return p;
}


void
fill_laser_config_data(carmen_simulator_ackerman_laser_config_t *lasercfg)
{
	lasercfg->num_lasers = 1 + carmen_round(lasercfg->fov / lasercfg->angular_resolution);
	lasercfg->start_angle = -0.5 * lasercfg->fov;

	/* give a warning if it is not a standard configuration */

	if (fabs(lasercfg->fov - M_PI) > 1e-6 &&
			fabs(lasercfg->fov - 100.0/180.0 * M_PI) > 1e-6 &&
			fabs(lasercfg->fov -  90.0/180.0 * M_PI) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (fov=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->fov));

	if (fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(1.0)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.5)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.25)) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (res=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->angular_resolution));
}


void
CarmenSim::_load_params()
{
	// TODO: read properly from param's file.
	_robot_ackerman_config.obstacle_avoider_obstacles_safe_distance = 1.1;
	_robot_ackerman_config.max_v = 5.5;
	_robot_ackerman_config.max_phi = 0.5337;
	_robot_ackerman_config.approach_dist = 0.20;
	_robot_ackerman_config.side_dist = 0.20;
	_robot_ackerman_config.length = 4.425;
	_robot_ackerman_config.width = 1.806;
	_robot_ackerman_config.maximum_acceleration_forward = 0.6;
	_robot_ackerman_config.maximum_deceleration_forward = 3.3;
	_robot_ackerman_config.reaction_time = 0.1;
	_robot_ackerman_config.distance_between_rear_wheels = 1.535;
	_robot_ackerman_config.distance_between_front_and_rear_axles = 2.625;
	_robot_ackerman_config.distance_between_front_car_and_front_wheels = 0.85;
	_robot_ackerman_config.distance_between_rear_car_and_rear_wheels = 0.96;
	_robot_ackerman_config.maximum_steering_command_rate = 0.335;
	_robot_ackerman_config.understeer_coeficient = 0.0015;
	_robot_ackerman_config.allow_rear_motion = true;
	_robot_ackerman_config.interpolate_odometry = false;
	_simulator_config.real_time = 0.025;
	_simulator_config.sync_mode = false;
	_simulator_config.motion_timeout = 10000.0;
	_simulator_config.use_front_laser = true;
	_simulator_config.front_laser_config.id = 1;
	_simulator_config.use_rear_laser = true;
	_simulator_config.rear_laser_config.id = 9;
	_simulator_config.width = 1.806;
	_simulator_config.length = 4.425;
	_simulator_config.distance_between_rear_wheels = 1.535;
	_simulator_config.distance_between_front_and_rear_axles = 2.625;
	_simulator_config.max_v = 5.5;
	_simulator_config.max_phi = 0.5337;
	_simulator_config.maximum_steering_command_rate = 0.335;
	_simulator_config.understeer_coeficient = 0.0015;
	_simulator_config.understeer_coeficient2 = 0.0015;
	_simulator_config.maximum_speed_forward = 46.0;
	_simulator_config.maximum_speed_reverse = 20.0;
	_simulator_config.maximum_acceleration_forward = 0.6;
	_simulator_config.maximum_deceleration_forward = 3.3;
	_simulator_config.maximum_acceleration_reverse = 1.2;
	_simulator_config.maximum_deceleration_reverse = 2.7;
	_simulator_config.distance_between_rear_car_and_rear_wheels = 0.96;
	_simulator_config.use_mpc = false;
	_simulator_config.use_rlpid = false;
	_simulator_config.front_laser_config.max_range = 50;
	_simulator_config.front_laser_config.offset = 0.0;
	_simulator_config.front_laser_config.side_offset = 0.0;
	_simulator_config.front_laser_config.angular_offset = 0.0;
	_simulator_config.front_laser_config.fov = 180;
	_simulator_config.front_laser_config.angular_resolution = 0.5;
	_simulator_config.front_laser_config.prob_of_random_max = .0001;
	_simulator_config.front_laser_config.prob_of_random_reading = .0001;
	_simulator_config.front_laser_config.variance = .001;
	_simulator_config.rear_laser_config.max_range = 50;
	_simulator_config.rear_laser_config.offset = 0.0;
	_simulator_config.rear_laser_config.side_offset = 0.0;
	_simulator_config.rear_laser_config.angular_offset = 3.1415923;
	_simulator_config.rear_laser_config.fov = 180;
	_simulator_config.rear_laser_config.angular_resolution = 0.5;
	_simulator_config.rear_laser_config.prob_of_random_max = .0001;
	_simulator_config.rear_laser_config.prob_of_random_reading = .0001;
	_simulator_config.rear_laser_config.variance = .001;

	_simulator_config.front_laser_config.fov = carmen_degrees_to_radians(_simulator_config.front_laser_config.fov);
	_simulator_config.front_laser_config.angular_resolution = carmen_degrees_to_radians(_simulator_config.front_laser_config.angular_resolution);
	_simulator_config.rear_laser_config.fov = carmen_degrees_to_radians(_simulator_config.rear_laser_config.fov);
	_simulator_config.rear_laser_config.angular_resolution = carmen_degrees_to_radians(_simulator_config.rear_laser_config.angular_resolution);

	fill_laser_config_data(&_simulator_config.front_laser_config);
	fill_laser_config_data(&_simulator_config.rear_laser_config);

	_simulator_config.robot_config = _robot_ackerman_config;

	_use_velocity_nn = false;
	_use_phi_nn = true;
}


int
pose_is_outside_map(carmen_map_t &map, double x, double y)
{
	bool map_is_empty = map.config.x_origin == 0 || map.config.y_origin == 0 || map.config.x_size == 0 || map.config.y_size == 0;
	bool x_is_out = x < map.config.x_origin || x > (map.config.x_origin + map.config.x_size * map.config.resolution);
	bool y_is_out = y < map.config.y_origin || y > (map.config.y_origin + map.config.y_size * map.config.resolution);

	return (map_is_empty || x_is_out || y_is_out);
}

