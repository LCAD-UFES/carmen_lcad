#include "car_model.h"


void
carmen_libcarneuralmodel_init_steering_ann_input(fann_type *input)
{
	int i;

	// steering = 0, current_curvature = 0
	for (i = 0; i < (NUM_STEERING_ANN_INPUTS - 1); i += 2)
	{
		input[i] = 0.0;
		input[i + 1] = 0.0;
	}
}


void
carmen_libcarneuralmodel_build_steering_ann_input(fann_type *input, double s, double cc)
{
	int i;

	for (i = 0; i < (NUM_STEERING_ANN_INPUTS - 2); i++)
		input[i] = input[i + 2];

	input[NUM_STEERING_ANN_INPUTS - 2] = s / 100.0;
	input[NUM_STEERING_ANN_INPUTS - 1] = cc;
}


/*struct fann *
carmen_libcarneuralmodel_init_steering_ann (fann_type *steering_ann_input)
{
	struct fann *steering_ann;

	if (steering_ann == NULL)
	{
		steering_ann = fann_create_from_file("steering_ann.net");
		if (steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_steering_ann_input(steering_ann_input);
	}

	return steering_ann;
}*/


// aoc is the arch tangent of curvature
double
carmen_libcarneuralmodel_compute_new_aoc_from_effort(double steering_effort, double atan_current_curvature, fann_type *steering_ann_input, struct fann *steering_ann)
{
	fann_type *steering_ann_output;

	carmen_libcarneuralmodel_build_steering_ann_input(steering_ann_input, carmen_clamp(-100, steering_effort, 100), atan_current_curvature);

	steering_ann_output = fann_run(steering_ann, steering_ann_input);

	return (steering_ann_output[0]);
}


double
carmen_libcarneuralmodel_compute_new_phi_from_effort_new(double steering_effort, double atan_current_curvature, fann_type *steering_ann_input, struct fann *steering_ann,
														double v, double understeer_coeficient, double distance_between_front_and_rear_axles, double max_phi)
{
	double aoc = carmen_libcarneuralmodel_compute_new_aoc_from_effort(steering_effort, atan_current_curvature, steering_ann_input, steering_ann);

	double phi = carmen_get_phi_from_curvature(tan(aoc), v, understeer_coeficient, distance_between_front_and_rear_axles);
	phi = carmen_clamp(-max_phi, phi, max_phi);

	return (phi);
}


double
carmen_libcarneuralmodel_compute_new_phi_from_effort(double steering_effort, double atan_current_curvature, fann_type *steering_ann_input, struct fann *steering_ann,
														double v, double understeer_coeficient, double distance_between_front_and_rear_axles,
														double max_phi)
{
	fann_type *steering_ann_output;

	carmen_libcarneuralmodel_build_steering_ann_input(steering_ann_input, carmen_clamp(-100, steering_effort, 100), atan_current_curvature);

	steering_ann_output = fann_run(steering_ann, steering_ann_input);

	double phi = carmen_get_phi_from_curvature(tan(steering_ann_output[0]), v, understeer_coeficient, distance_between_front_and_rear_axles);
	phi = carmen_clamp(-max_phi, phi, max_phi);

	return (phi);
}


void
carmen_libcarneuralmodel_init_velocity_ann_input(fann_type *input)
{
	int i;

	// acelerador = 0, freio = 100.0, v = 0.0
	for (i = 0; i < (NUM_VELOCITY_ANN_INPUTS - 1); i += 3)
	{
		input[i] = 0.0;
		input[i + 1] = 100.0 / 100.0;
		input[i + 2] = 0.0;
	}
}


void
carmen_libcarneuralmodel_build_velocity_ann_input(fann_type *input, double t, double b, double cv)
{
	int i;

	for (i = 0; i < (NUM_VELOCITY_ANN_INPUTS - 3); i++)
		input[i] = input[i + 3];

	input[NUM_VELOCITY_ANN_INPUTS - 3] = t / 100.0;
	input[NUM_VELOCITY_ANN_INPUTS - 2] = b / 100.0;
	input[NUM_VELOCITY_ANN_INPUTS - 1] = cv / 5.0;
}


double
carmen_libcarneuralmodel_compute_new_velocity_from_efforts(fann_type *velocity_ann_input, struct fann *velocity_ann __attribute__ ((unused)), double throttle_effort, double brake_effort, double current_velocity)
{
//	fann_type *velocity_ann_output;

	carmen_libcarneuralmodel_build_velocity_ann_input(velocity_ann_input, carmen_clamp(-100, throttle_effort, 100), carmen_clamp(-100, brake_effort, 100), current_velocity);

//	velocity_ann_output = fann_run(velocity_ann, velocity_ann_input);
//
//	return (velocity_ann_output[0]);
	static double v = 0.0;
	v += 0.15 * (velocity_ann_input[0] - (velocity_ann_input[1] - 0.17));
	v = (v < 0.0)? 0.0: v;
	return (v); // Velocidade com atraso.
}


double
carmen_libcarneuralmodel_compute_new_phi_with_ann(double v, double current_phi, double desired_phi, double time,
	double understeer_coeficient, double distance_between_front_and_rear_axles, double max_phi)
{
	static double steering_effort = 0.0;
	double atan_current_curvature;
	double atan_desired_curvature;
	static fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	static struct fann *steering_ann = NULL;

	if (steering_ann == NULL)
	{
		steering_ann = fann_create_from_file("steering_ann.net");
		if (steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_steering_ann_input(steering_ann_input);
	}

	atan_current_curvature = carmen_get_curvature_from_phi(current_phi, v, understeer_coeficient, distance_between_front_and_rear_axles);

	atan_desired_curvature = carmen_get_curvature_from_phi(desired_phi, v, understeer_coeficient, distance_between_front_and_rear_axles);

	//PID
	steering_effort = carmen_libpid_steering_PID_controler(atan_desired_curvature, atan_current_curvature, time, 0, 0);

	//RL_PID
	//steering_effort = carmen_librlpid_compute_new_phi_with_ann (current_phi, desired_phi, /*next_desired_phi*/desired_phi, steering_ann_input,
	//		steering_ann, v, understeer_coeficient, distance_between_front_and_rear_axles);

	double new_phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(steering_effort, atan_current_curvature, steering_ann_input,
							steering_ann, v, understeer_coeficient, distance_between_front_and_rear_axles, max_phi);

	return (new_phi);
}


double
carmen_libcarneuralmodel_compute_new_velocity_with_ann(double desired_v, double v, double time)
{
	static double throttle_command = 0.0;
	static double brakes_command = 100.0 / 100.0;
	static int gear_command = 0;
	static fann_type velocity_ann_input[NUM_VELOCITY_ANN_INPUTS];
	fann_type *velocity_ann_output;
	static struct fann *velocity_ann = NULL;
	double next_velocity;

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

	carmen_libpid_velocity_PID_controler(&throttle_command, &brakes_command, &gear_command, desired_v, v, time, 0);

	if (gear_command == 129) // marcha reh
	{
		carmen_libcarneuralmodel_build_velocity_ann_input(velocity_ann_input, throttle_command, brakes_command, -v);
		velocity_ann_output = fann_run(velocity_ann, velocity_ann_input);

		next_velocity = velocity_ann_output[0];
	}
	else
	{
		carmen_libcarneuralmodel_build_velocity_ann_input(velocity_ann_input, throttle_command, brakes_command, v);
		velocity_ann_output = fann_run(velocity_ann, velocity_ann_input);

		next_velocity = velocity_ann_output[0];
	}
	next_velocity = next_velocity + next_velocity * carmen_gaussian_random(0.0, 0.007); // add some noise

	return (next_velocity);
}


void
update_target_v_and_target_phi(carmen_robot_and_trailer_motion_command_t *current_motion_command_vector, int nun_motion_commands, double &target_v,
								double &target_phi, double time_of_last_command, int &current_motion_command_vector_index)
{
	double time_elapsed_since_last_motion_command, motion_command_time_consumed;
	int i, motion_command_completely_consumed;

	if (current_motion_command_vector == NULL)
	{
		target_v = 0.0;
		target_phi = 0.0;
		return;
	}

	time_elapsed_since_last_motion_command = carmen_get_time() - time_of_last_command;

	i = 0;
	motion_command_completely_consumed = 1;
	motion_command_time_consumed = 0.0;
	do
	{
		motion_command_time_consumed += current_motion_command_vector[i].time;
		if (motion_command_time_consumed > time_elapsed_since_last_motion_command)
		{
			motion_command_completely_consumed = 0;
			break;
		}
		i++;
	} while (i < nun_motion_commands);

	if (motion_command_completely_consumed)
	{
		target_v = 0.0;
		//printf("motion command completely consumed; i = %d\n", i);
	}
	else
	{
		target_v = current_motion_command_vector[i].v;
		target_phi = current_motion_command_vector[i].phi;
		//printf("i = %d\n", i);
	}
	current_motion_command_vector_index = i;
}



void
carmen_libcarneuralmodel_compute_new_pos_with_ann(carmen_robot_and_trailer_motion_command_t *current_motion_command_vector, int nun_motion_commands, double &target_v,
													double &current_v, double &target_phi, double &current_phi, double time_of_last_command,
													int &current_motion_command_vector_index, double delta_t, double understeer_coeficient,
													double distance_between_front_and_rear_axles, double max_phi, carmen_point_t &odom_pose, carmen_point_t &true_pose)
{
	carmen_point_t new_odom;
	carmen_point_t new_true;
	double v, phi;

	update_target_v_and_target_phi(current_motion_command_vector, nun_motion_commands, target_v,
									target_phi, time_of_last_command, current_motion_command_vector_index);

	v   = carmen_libcarneuralmodel_compute_new_velocity_with_ann(target_v, current_v, delta_t);
	phi = carmen_libcarneuralmodel_compute_new_phi_with_ann(current_v, current_phi, target_phi, delta_t,
				understeer_coeficient, distance_between_front_and_rear_axles, max_phi);

	phi = carmen_clamp(-max_phi, phi, max_phi);

	new_odom = odom_pose;
	new_true = true_pose;

	new_odom.x +=  v *  delta_t * cos(new_true.theta);
	new_odom.y +=  v * delta_t * sin(new_true.theta);
	new_odom.theta += v * delta_t * tan(phi) / distance_between_front_and_rear_axles;
	new_odom.theta = carmen_normalize_theta(new_odom.theta);

	new_true.x +=  v * delta_t * cos(new_true.theta);
	new_true.y +=  v * delta_t * sin(new_true.theta);
	new_true.theta += v * delta_t * tan(phi) / distance_between_front_and_rear_axles;
	new_true.theta = carmen_normalize_theta(new_true.theta);

	//if(hit_something_in_the_map(simulator_config, new_true))
	//	return; // Do not update pose

	odom_pose.x = new_odom.x;
	odom_pose.y = new_odom.y;
	odom_pose.theta = new_odom.theta;
	true_pose.x = new_true.x;
	true_pose.y = new_true.y;
	true_pose.theta = new_true.theta;

	current_v = v;
	current_phi = phi;
}


//void
//carmen_libcarneuralmodel_recalc_pos(carmen_simulator_ackerman_config_t *simulator_config)
//{
//	carmen_libcarneuralmodel_compute_new_pos_with_ann(simulator_config->current_motion_command_vector, simulator_config->nun_motion_commands, simulator_config->target_v,
//			simulator_config->v, simulator_config->target_phi, simulator_config->phi, simulator_config->time_of_last_command,
//			simulator_config->current_motion_command_vector_index, simulator_config->delta_t, simulator_config->understeer_coeficient,
//			simulator_config->distance_between_front_and_rear_axles, simulator_config->max_phi, simulator_config->odom_pose, simulator_config->true_pose);
//}


