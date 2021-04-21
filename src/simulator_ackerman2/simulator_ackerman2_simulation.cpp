/*******************************************
 * library with the functions for the guts *
 * of the simulator                        *
 *******************************************/

#include "simulator_ackerman2_simulation.h"

#include <carmen/collision_detection.h>
#include <carmen/ford_escape_hybrid.h>
#include <car_model.h>
#include <control.h>
#include <pthread.h>
#include "objects_ackerman.h"

double
get_acceleration(double v, double target_v, carmen_simulator_ackerman_config_t *simulator_config)
{
	double acceleration;

	if (fabs(target_v) > fabs(v))
		acceleration = target_v >= 0.0 ? simulator_config->maximum_acceleration_forward : simulator_config->maximum_acceleration_reverse;
	else
		acceleration = target_v >= 0.0 ? simulator_config->maximum_deceleration_forward : simulator_config->maximum_deceleration_reverse;

	return (acceleration);
}

double
compute_new_velocity(carmen_simulator_ackerman_config_t *simulator_config)
{
	double minimum_time_to_reach_desired_velocity;
	double acceleration;
	int signal_target_v, signal_v, command_signal;
	double target_v, time, time_rest;

	signal_target_v = simulator_config->target_v >= 0 ? 1 : -1;
	signal_v = simulator_config->v >= 0 ? 1 : -1;

	target_v = simulator_config->target_v;

	if (signal_target_v != signal_v)
		target_v = 0;

	acceleration = get_acceleration(simulator_config->v, target_v, simulator_config);

	minimum_time_to_reach_desired_velocity = fabs((target_v - simulator_config->v) / acceleration); // s = v / a, i.e., s = (m/s) / (m/s^2)

	command_signal = simulator_config->v < target_v ? 1 : -1;

	time = fmin(simulator_config->delta_t, minimum_time_to_reach_desired_velocity);

	simulator_config->v += command_signal * acceleration * time;

	if (signal_target_v != signal_v)
	{
		time_rest = simulator_config->delta_t - time;
		target_v = simulator_config->target_v;

		acceleration = get_acceleration(simulator_config->v, target_v, simulator_config);

		minimum_time_to_reach_desired_velocity = fabs((target_v - simulator_config->v) / acceleration); // s = v / a, i.e., s = (m/s) / (m/s^2)

		command_signal = simulator_config->v < target_v ? 1 : -1;

		time = fmin(time_rest, minimum_time_to_reach_desired_velocity);

		simulator_config->v += command_signal * acceleration * time;
	}

	simulator_config->v = carmen_clamp(-simulator_config->maximum_speed_reverse, simulator_config->v, simulator_config->maximum_speed_forward);
	
	return (simulator_config->v);
}


double
compute_curvature(double phi, carmen_simulator_ackerman_config_t *simulator_config)
{
	double curvature;

	curvature = carmen_get_curvature_from_phi(phi, simulator_config->v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);

	return (curvature);
}


double
compute_curvature2(double phi, carmen_simulator_ackerman_config_t *simulator_config)
{
	double curvature;

	curvature = carmen_get_curvature_from_phi(phi, simulator_config->v, simulator_config->understeer_coeficient2, simulator_config->distance_between_front_and_rear_axles);

	return (curvature);
}


double
get_phi_from_curvature(double curvature, carmen_simulator_ackerman_config_t *simulator_config)
{
	double phi;

	phi = carmen_get_phi_from_curvature(curvature, simulator_config->v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);

	return (phi);
}


double
get_phi_from_curvature2(double curvature, carmen_simulator_ackerman_config_t *simulator_config)
{
	double phi;

	phi = carmen_get_phi_from_curvature(curvature, simulator_config->v, simulator_config->understeer_coeficient2, simulator_config->distance_between_front_and_rear_axles);

	return (phi);
}


double
compute_new_phi(carmen_simulator_ackerman_config_t *simulator_config)
{
	double current_curvature;
	double desired_curvature;
	double delta_curvature;
	double minimum_time_to_reach_desired_curvature;
	double phi;

	current_curvature = compute_curvature(simulator_config->phi, simulator_config);
	desired_curvature = compute_curvature(simulator_config->target_phi, simulator_config);
	delta_curvature = desired_curvature - current_curvature;
	minimum_time_to_reach_desired_curvature = fabs(delta_curvature / simulator_config->maximum_steering_command_rate);

	if (minimum_time_to_reach_desired_curvature > simulator_config->delta_t) 
	{
		if (delta_curvature < 0.0) 
			current_curvature -= simulator_config->maximum_steering_command_rate * simulator_config->delta_t;
		else
			current_curvature += simulator_config->maximum_steering_command_rate * simulator_config->delta_t;
	} 
	else
		current_curvature = desired_curvature;

	double max_c = carmen_get_curvature_from_phi(simulator_config->max_phi, 0.0, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);
	if (fabs(current_curvature) > max_c)
	{
		current_curvature = current_curvature / fabs(current_curvature);
		current_curvature *= max_c;
	}

	phi = get_phi_from_curvature(current_curvature, simulator_config);

	//printf("desired_phi = %lf, phi = %lf\n", simulator_config->target_phi, simulator_config->phi);

	return (phi);
}


int
hit_something_in_the_map(carmen_simulator_ackerman_config_t *simulator_config, carmen_point_t new_true)
{
	//	Verificando colisão completa
	//	carmen_robot_ackerman_config_t robot_config;
	//
	//	robot_config.distance_between_rear_car_and_rear_wheels = simulator_config->distance_between_rear_car_and_rear_wheels;
	//	robot_config.width = simulator_config->width;
	//	robot_config.length = simulator_config->length;
	//
	//	return pose_hit_obstacle(new_true, &simulator_config->map, &robot_config);

	carmen_map_p map;
	int map_x, map_y;

	map = &(simulator_config->map);
	map_x = (new_true.x - map->config.x_origin) / map->config.resolution;
	map_y = (new_true.y - map->config.y_origin) / map->config.resolution;

	if (map_x < 0 || map_x >= map->config.x_size ||
			map_y < 0 || map_y >= map->config.y_size ||
			map->map[map_x][map_y] > .15 ||					// @@@ Tem que tratar o robo retangular...
			carmen_simulator_object_too_close(new_true.x, new_true.y, -1)) 	// @@@ Tem que tratar o robo retangular...
		return (1);

	return (0);
}


void
update_target_v_and_target_phi(carmen_simulator_ackerman_config_t *simulator_config)
{
	double time_elapsed_since_last_motion_command, motion_command_time_consumed;
	int i = 0, motion_command_completely_consumed;

	if (simulator_config->current_motion_command_vector == NULL)
	{
		simulator_config->target_v = 0.0;
		simulator_config->target_phi = 0.0;
		return;
	}

	time_elapsed_since_last_motion_command = carmen_get_time() - simulator_config->time_of_last_command;

	motion_command_completely_consumed = 1;
	motion_command_time_consumed = 0.0;
	do
	{
		motion_command_time_consumed += simulator_config->current_motion_command_vector[i].time;
		if (motion_command_time_consumed > time_elapsed_since_last_motion_command)
		{
			motion_command_completely_consumed = 0;
			break;
		}
		i++;
	} while (i < simulator_config->nun_motion_commands);

	if (motion_command_completely_consumed)
	{
		simulator_config->target_v = 0.0;
	}
	else
	{
		simulator_config->target_v = simulator_config->current_motion_command_vector[i].v;
		simulator_config->target_phi = simulator_config->current_motion_command_vector[i].phi;
	}
	simulator_config->current_motion_command_vector_index = i;
}


double
compute_new_velocity_with_ann(carmen_simulator_ackerman_config_t *simulator_config)
{
	static double throttle_command = 0.0;
	static double brakes_command = 100.0 / 100.0;
	static int gear_command = 0;
	static fann_type velocity_ann_input[NUM_VELOCITY_ANN_INPUTS];
	//fann_type *velocity_ann_output;
	static struct fann *velocity_ann = NULL;

	if (velocity_ann == NULL)
	{
		velocity_ann = fann_create_from_file("velocity_ann.net");
		if (velocity_ann == NULL)
		{
			printf("Error: Could not create velocity_ann\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_velocity_ann_input(velocity_ann_input);
	}
	
	if (simulator_config->initialize_neural_networks)
		carmen_libcarneuralmodel_init_velocity_ann_input(velocity_ann_input);

	if (simulator_config->use_mpc)
	{
		carmen_libmpc_compute_velocity_effort(&throttle_command, &brakes_command, &gear_command,
				simulator_config->current_motion_command_vector, simulator_config->nun_motion_commands,
				simulator_config->v, simulator_config->time_of_last_command, &simulator_config->robot_config);
	}
	else
	{
		carmen_libpid_velocity_PID_controler(&throttle_command, &brakes_command, &gear_command,
				simulator_config->target_v, simulator_config->v, simulator_config->delta_t, 0);
	}

	if (gear_command == 129) // marcha reh
		simulator_config->v = -carmen_libcarneuralmodel_compute_new_velocity_from_efforts(velocity_ann_input, velocity_ann, throttle_command, brakes_command, -simulator_config->v);
	else
		simulator_config->v = carmen_libcarneuralmodel_compute_new_velocity_from_efforts(velocity_ann_input, velocity_ann, throttle_command, brakes_command, simulator_config->v);

	//simulator_config->v = simulator_config->v + simulator_config->v * carmen_gaussian_random(0.0, 0.007); // add some noise

	return (simulator_config->v);
}


double
distance_point_to_line(double point_x, double point_y, double line_a_x, double line_a_y, double line_b_x, double line_b_y)
{
	double a = line_a_y - line_b_y;                                                     // yA-yB=a,  xB-xA=b,  xAyB-xB*yA=c
	double b = line_b_x - line_a_x;
	double c = (line_a_x * line_b_y) - (line_b_x * line_a_y);

	if ( a == 0.0 || b == 0.0)
		return (5.0);   // TODO quando os pontos sao iguais, oq fazer?

	double dist = fabs((a *  point_x) + (b * point_y) + c) / sqrt((a * a) + (b * b));   // dist = |ax0 + by0 + c| / √(a^2 + b^2)

	return (dist);
}


unsigned int
find_two_closest_point_in_path(carmen_simulator_ackerman_config_t *simulator_config) // Always returns the previous of the two points
{                                                                                    // between the global_pos
	double dist = 0.0, dist_1 = 9999.9, dist_2 = 9999.9;
	unsigned int i = 0;
	unsigned int number_of_poses = abs(simulator_config->path_goals_and_annotations->number_of_poses);

	for (; i < number_of_poses; i++)
	{
		dist = carmen_distance_ackerman_traj(&simulator_config->path_goals_and_annotations->poses[i], &simulator_config->path_goals_and_annotations->poses[i + 1]);
		dist_2 = dist_1;
		dist_1 = dist;

		if (dist > dist_1)
			break;
	}

	if (dist > dist_2)		// TODO check if it is correct
		return (i - 2);
	else
		return (i - 1);
}


void
compute_lateral_and_heading_errors(carmen_simulator_ackerman_config_t *simulator_config, double &lateral_error, double &heading_error)
{
	// TODO Check if it is simply get the first of poses back and first of poses
	unsigned int closest_rddf_index = find_two_closest_point_in_path(simulator_config);      // Compute the two RDDF points between the actual car pose

	// The lateral error is the distance from the car pose (global_pos) to the line composed of the two closest points in the RDDF
	lateral_error = distance_point_to_line(simulator_config->global_pos.globalpos.x, simulator_config->global_pos.globalpos.y,
			simulator_config->path_goals_and_annotations->poses[closest_rddf_index].x, simulator_config->path_goals_and_annotations->poses[closest_rddf_index].y,
			simulator_config->path_goals_and_annotations->poses[closest_rddf_index + 1].x, simulator_config->path_goals_and_annotations->poses[closest_rddf_index + 1].y);


	heading_error = atan2(simulator_config->path_goals_and_annotations->poses[closest_rddf_index].y - simulator_config->path_goals_and_annotations->poses[closest_rddf_index + 1].y,
			simulator_config->path_goals_and_annotations->poses[closest_rddf_index].x - simulator_config->path_goals_and_annotations->poses[closest_rddf_index + 1].x);
}




#define g 9.80665		// Gravity constant

double                                                                                                           // TODO missing Ux e Rs
compute_feed_forward_phi(carmen_simulator_ackerman_config_t *simulator_config, double Ux, double Rs, double Kug) // TODO Ux, Rs, Kug sao mesmo parametros?
{
	double L = simulator_config->distance_between_front_and_rear_axles;
	return ((L + (Kug * Ux * Ux) / g) / Rs);  // TODO Check if g is really the gravity
}


#define Kp 10			// Kritayakiranas constant
#define Xla 20          // Position of the point ahead the car to compute the lookahead_error (Kritayakirana used 20)

double
compute_control_phi(carmen_simulator_ackerman_config_t *simulator_config)  // TODO missing Cf
{
	double Cf = 1.0; // TODO Como encontrar Cf??? (Frontal Cornering Stifiness)
	double e_lateral_error = 0.0, delta_psi_heading_error = 0.0;

	compute_lateral_and_heading_errors(simulator_config, e_lateral_error, delta_psi_heading_error);

	double lookahead_error = e_lateral_error + Xla * sin(delta_psi_heading_error);

	return (-2 * (Kp/Cf) * lookahead_error);
}


double
compute_damping_phi() // TODO missing r_yaw_rate, beta_vehicle_sideslip, K_road_curvature
{
	double r_yaw_rate = 0.0, Ux_desired_speed = 0.0, K_road_curvature = 0.0, delta_psi_heading_error = 0.0, beta_vehicle_sideslip = 0.0;
	double K_delta_psi_derivative = 0.0;
	double delta_psi_derivative = r_yaw_rate - (Ux_desired_speed * K_road_curvature) * (cos(delta_psi_heading_error) - tan(beta_vehicle_sideslip) * sin(delta_psi_heading_error));

	return (- K_delta_psi_derivative * delta_psi_derivative);
}


double
compute_desired_phi_from_dynamic_model(carmen_simulator_ackerman_config_t *simulator_config)
{
	return (compute_feed_forward_phi(simulator_config, 1, 1, 1) + compute_control_phi(simulator_config) + compute_damping_phi());
}


double
compute_new_phi_with_ann(carmen_simulator_ackerman_config_t *simulator_config)
{
	static double steering_effort = 0.0;
	double atan_current_curvature;
	static fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	static struct fann *steering_ann = NULL;

	if (steering_ann == NULL)
	{
		steering_ann = fann_create_from_file("steering_ann.net");
		if (steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann compute_new_phi_with_ann()\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_steering_ann_input(steering_ann_input);
	}

	if (simulator_config->initialize_neural_networks)
		carmen_libcarneuralmodel_init_steering_ann_input(steering_ann_input);

	atan_current_curvature = carmen_get_curvature_from_phi(simulator_config->phi, simulator_config->v, simulator_config->understeer_coeficient2,
															simulator_config->distance_between_front_and_rear_axles);

	// Dynamic model
//	double dynamic_desired_phi = compute_desired_phi_from_dynamic_model(simulator_config);
//	double atan_desired_curvature = carmen_get_curvature_from_phi(dynamic_desired_phi, simulator_config->v, 1.0, simulator_config->distance_between_front_and_rear_axles);

	// Old model
	double atan_desired_curvature = carmen_get_curvature_from_phi(simulator_config->target_phi, simulator_config->v, simulator_config->understeer_coeficient2,
															simulator_config->distance_between_front_and_rear_axles);

	steering_effort = carmen_libpid_steering_PID_controler_FUZZY(atan_desired_curvature, atan_current_curvature, simulator_config->delta_t, 0, simulator_config->v);

	//pid_plot_phi(simulator_config->phi, simulator_config->target_phi, 0.55, "phi");

	double phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(steering_effort, atan_current_curvature,
			steering_ann_input, steering_ann, simulator_config->v,
			simulator_config->understeer_coeficient2, simulator_config->distance_between_front_and_rear_axles,
			simulator_config->max_phi);

	return (phi);
}


carmen_robot_ackerman_config_t
get_robot_config(carmen_simulator_ackerman_config_t *simulator_config)
{
	carmen_robot_ackerman_config_t robot_config;

	robot_config.distance_between_front_and_rear_axles = simulator_config->distance_between_front_and_rear_axles;
	robot_config.understeer_coeficient = simulator_config->understeer_coeficient;
	robot_config.max_phi = simulator_config->max_phi;
	robot_config.maximum_steering_command_rate = simulator_config->maximum_steering_command_rate;

	return (robot_config);
}


void
carmen_simulator_ackerman2_recalc_pos(carmen_simulator_ackerman_config_t *simulator_config)
{
	carmen_point_t new_odom;
	carmen_point_t new_true;
	double v, phi;

	update_target_v_and_target_phi(simulator_config);

	//Velocity must be calculated before phi
	//v   = compute_new_velocity(simulator_config);
	//phi = compute_new_phi(simulator_config);// + carmen_gaussian_random(0.0, carmen_degrees_to_radians(0.1));

	v   = compute_new_velocity_with_ann(simulator_config);
	phi = compute_new_phi_with_ann(simulator_config);// + carmen_gaussian_random(0.0, carmen_degrees_to_radians(0.05));

	phi = carmen_clamp(-simulator_config->max_phi, phi, simulator_config->max_phi);
	simulator_config->phi = phi;
	simulator_config->initialize_neural_networks = 0;

	new_odom = simulator_config->odom_pose;
	new_true = simulator_config->true_pose;

	new_odom.x +=  v * simulator_config->delta_t * cos(new_true.theta);
	new_odom.y +=  v * simulator_config->delta_t * sin(new_true.theta);
	new_odom.theta += v * simulator_config->delta_t * tan(phi) / simulator_config->distance_between_front_and_rear_axles;
	new_odom.theta = carmen_normalize_theta(new_odom.theta);

	new_true.x +=  v * simulator_config->delta_t * cos(new_true.theta);
	new_true.y +=  v * simulator_config->delta_t * sin(new_true.theta);
	new_true.theta += v * simulator_config->delta_t * tan(phi) / simulator_config->distance_between_front_and_rear_axles;
	new_true.theta = carmen_normalize_theta(new_true.theta);

	if (hit_something_in_the_map(simulator_config, new_true))
		return; // Do not update pose

	simulator_config->odom_pose = new_odom;
	simulator_config->true_pose = new_true;
}


/* adds error to a laser scan */
static void 
add_laser_error(carmen_laser_laser_message * laser, 
		carmen_simulator_ackerman_laser_config_t *laser_config)
{
	int i;

	for (i = 0; i < laser_config->num_lasers; i ++)
	{
		if (laser->range[i] > laser_config->max_range)
			laser->range[i] = laser_config->max_range;
		else if (carmen_uniform_random(0, 1.0) < laser_config->prob_of_random_max)
			laser->range[i] = laser_config->max_range;
		else if (carmen_uniform_random(0, 1.0) < laser_config->prob_of_random_reading)
			laser->range[i] = carmen_uniform_random(0, laser_config->num_lasers);
		else 
			laser->range[i] += carmen_gaussian_random(0.0, laser_config->variance);
	}
}


/*calculates a laser message based upon the current position*/
void
carmen_simulator_ackerman2_calc_laser_msg(carmen_laser_laser_message *laser,
		carmen_simulator_ackerman_config_p simulator_config,
		int is_rear)
{
	carmen_traj_point_t point;
	carmen_simulator_ackerman_laser_config_t *laser_config = NULL;


	if (is_rear) 
	{
		laser_config = &(simulator_config->rear_laser_config);
	} 
	else  
	{
		laser_config = &(simulator_config->front_laser_config);
	}

	laser->id = laser_config->id; 

	point.x = simulator_config->true_pose.x + 
			laser_config->offset * cos(simulator_config->true_pose.theta) -
			laser_config->side_offset *  sin(simulator_config->true_pose.theta);

	point.y = simulator_config->true_pose.y + 
			laser_config->offset * sin(simulator_config->true_pose.theta)	+
			laser_config->side_offset * cos(simulator_config->true_pose.theta);

	point.theta = carmen_normalize_theta(simulator_config->true_pose.theta + laser_config->angular_offset);

	point.t_vel = simulator_config->v;
	point.r_vel = simulator_config->phi;

	laser->num_readings = laser_config->num_lasers;

	laser->config.maximum_range       = laser_config->max_range;
	laser->config.fov                 = laser_config->fov;
	laser->config.start_angle         = laser_config->start_angle;
	laser->config.angular_resolution  = laser_config->angular_resolution;
	laser->config.laser_type          = SIMULATED_LASER;
	laser->config.accuracy            = laser_config->variance; 

	//this was placed here because compiling with the old motion model
	//did't work, check this if this breaks something
	laser->config.remission_mode      = REMISSION_NONE;

	{
		carmen_map_t _map;
		carmen_traj_point_t _point;

		_map = simulator_config->map;
		_map.config.x_origin = _map.config.y_origin =  0;
		_point.x = point.x - simulator_config->map.config.x_origin;
		_point.y = point.y - simulator_config->map.config.y_origin;
		_point.theta = point.theta;

		carmen_geometry_generate_laser_data(laser->range, &_point, laser->config.start_angle,
				laser->config.start_angle+laser->config.fov,
				laser_config->num_lasers,
				&_map);
	}

	carmen_simulator_ackerman_add_objects_to_laser(laser, simulator_config, is_rear);

	add_laser_error(laser, laser_config);
}


double
SpeedControlLogic(carmen_ackerman_path_point_t pose, carmen_simulator_ackerman_config_t *simulator_config)
{
	double v_scl = simulator_config->max_v;
	//todo transformar esses valores em parametros no ini

	double a_scl = 0.1681;
	double b_scl = -0.0049;
	double safety_factor = 1;

	double kt = carmen_get_curvature_from_phi(pose.phi, pose.v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);

	double v_cmd = fabs(simulator_config->target_v);

	double v_cmd_max = carmen_fmax(v_scl, (kt + simulator_config->delta_t  - a_scl) / b_scl);

	double k_max_scl = carmen_fmin(carmen_get_curvature_from_phi(simulator_config->max_phi, pose.v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles),
			a_scl + b_scl * v_cmd);

	double kt_dt = carmen_get_curvature_from_phi(simulator_config->target_phi, pose.v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);
	if (fabs(kt_dt) >= k_max_scl)
		v_cmd = fabs(safety_factor * v_cmd_max);

	if(pose.v != 0)
		return v_cmd * (pose.v / fabs(pose.v));
	else
		return 0;
}


carmen_ackerman_path_point_t
DynamicsResponse(carmen_ackerman_path_point_t pose, carmen_simulator_ackerman_config_t *simulator_config)
{
	double kt = carmen_get_curvature_from_phi(pose.phi, pose.v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);
	double kt_dt = carmen_get_curvature_from_phi(simulator_config->target_phi, pose.v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);
	double dk_dt_cmd = carmen_clamp(0 , kt_dt - kt, simulator_config->maximum_steering_command_rate); //todo descobrir qual é o comando minimo de curvatura, se existir
	double dv_dt_cmd;

	simulator_config->target_v = SpeedControlLogic(pose, simulator_config);

	double c = carmen_get_curvature_from_phi(simulator_config->max_phi, pose.v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);
	pose.phi = get_phi_from_curvature(carmen_clamp(-c, kt + dk_dt_cmd * simulator_config->delta_t, c), simulator_config);

	if (pose.v >= 0)
		dv_dt_cmd = carmen_clamp(-simulator_config->maximum_deceleration_forward, simulator_config->target_v - pose.v, simulator_config->maximum_acceleration_forward);
	else
		dv_dt_cmd = carmen_clamp(-simulator_config->maximum_deceleration_reverse, simulator_config->target_v - pose.v, simulator_config->maximum_acceleration_reverse);

	pose.v = pose.v + dv_dt_cmd * simulator_config->delta_t;
	return pose;
}


carmen_ackerman_path_point_t
motionModel(carmen_ackerman_path_point_t pose, carmen_simulator_ackerman_config_t *simulator_config)
{
	pose.x += pose.v * simulator_config->delta_t * cos(pose.theta);
	pose.y +=  pose.v * simulator_config->delta_t * sin(pose.theta);
	pose.theta += pose.v * simulator_config->delta_t * carmen_get_curvature_from_phi(pose.phi, pose.v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);
	pose.theta = carmen_normalize_theta(pose.theta);
	//todo como tratar o delay?

	pose = DynamicsResponse(pose, simulator_config);
	return pose;
}

