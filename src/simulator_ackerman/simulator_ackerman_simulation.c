/*******************************************
 * library with the functions for the guts *
 * of the simulator                        *
 *******************************************/

#include <carmen/carmen.h>
#include <carmen/collision_detection.h>
#include <carmen/ford_escape_hybrid.h>
#include <car_model.h>
#include <control.h>
#include <pthread.h>
#include "simulator_ackerman.h"
#include "simulator_ackerman_simulation.h"
#include "objects_ackerman.h"


#ifdef __USE_RL_CONTROL

double rl_control_throttle;
double rl_control_brake;
double rl_control_steering;

void
set_rl_control(double steering, double throttle, double brake)
{
	rl_control_steering = steering;
	rl_control_throttle = throttle;
	rl_control_brake = brake;
}

#endif


static double
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
	int i, motion_command_completely_consumed;

	if (simulator_config->current_motion_command_vector == NULL)
	{
		simulator_config->target_v = 0.0;
		simulator_config->target_phi = 0.0;
		return;
	}

	time_elapsed_since_last_motion_command = carmen_get_time() - simulator_config->time_of_last_command;

	i = 0;
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
		//printf("motion command completely consumed; i = %d\n", i);
	}
	else
	{
		simulator_config->target_v = simulator_config->current_motion_command_vector[i].v;
		simulator_config->target_phi = simulator_config->current_motion_command_vector[i].phi;
		//printf("i = %d\n", i);
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
		char path[256];
		sprintf(path, "%s/bin/velocity_ann.net", getenv("CARMEN_HOME"));

		velocity_ann = fann_create_from_file(path); //"velocity_ann.net");
		if (velocity_ann == NULL)
		{
			printf("Error: Could not create velocity_ann\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_velocity_ann_input(velocity_ann_input);
	}
	
	if (simulator_config->initialize_neural_networks)
		carmen_libcarneuralmodel_init_velocity_ann_input(velocity_ann_input);

#ifdef __USE_RL_CONTROL

	throttle_command = rl_control_throttle;
	brakes_command = rl_control_brake;

#else

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

#endif

	if (gear_command == 129) // marcha reh
		simulator_config->v = -carmen_libcarneuralmodel_compute_new_velocity_from_efforts(velocity_ann_input, velocity_ann, throttle_command, brakes_command, -simulator_config->v);
	else
		simulator_config->v = carmen_libcarneuralmodel_compute_new_velocity_from_efforts(velocity_ann_input, velocity_ann, throttle_command, brakes_command, simulator_config->v);

	//simulator_config->v = simulator_config->v + simulator_config->v * carmen_gaussian_random(0.0, 0.007); // add some noise

#ifdef __USE_RL_CONTROL
	if (brakes_command > 90.0)
		simulator_config->v = 0.0;
#endif

	return (simulator_config->v);
}


double
run_qlearning_network(struct fann *qlearning_ann,
        double atan_desired_curvature, double atan_current_curvature, double steering_command, double b, carmen_simulator_ackerman_config_t *simulator_config)
{
        fann_type  ann_input[4], *ann_output;
    	static double max_atan_curvature;
    	static int first_time = 1;

    	if (first_time)
    	{
    		max_atan_curvature = atan(compute_curvature(carmen_degrees_to_radians(30.0), simulator_config));
    		first_time = 0;
    	}

        ann_input[0] = atan_desired_curvature / max_atan_curvature;
        ann_input[1] = atan_current_curvature / max_atan_curvature;
        ann_input[2] = b / 6.0;
        ann_input[3] = steering_command / 100.0;
        ann_output   = fann_run(qlearning_ann, ann_input);
        return ann_output[0];
}


double
min_b_Q(int *b, struct fann *qlearning_ann, state s, action a, carmen_simulator_ackerman_config_t *simulator_config)
{
	int i;
	double ann_answer, min_ann_answer = 9999.9;

	*b = 0;
	for (i = 0; i <= 6; i++)
	{
		ann_answer = run_qlearning_network(qlearning_ann, s.atan_desired_curvature, s.atan_current_curvature, a.steering_command, (double) i, simulator_config);
		//fprintf(stdout, "%.5lf ", ann_answer);
		// Exploration
		if ((fabs(min_ann_answer - ann_answer) < 1e-3) && carmen_int_random(2))
		{
			*b = i;
			min_ann_answer = ann_answer;
		}
		//END: Exploration
		else if (ann_answer < min_ann_answer)
		{
			*b = i;
			min_ann_answer = ann_answer;
		}
	}
	//fprintf(stdout, "\n");

	return min_ann_answer;
}


void
qlearning_integrator(double *steering_command, int b)
{
	int i; double steering = 0.0;
	exhaustive_value_to_steering_mapping mapping[] =
	{
		  //{0, -50.000}, {1, -25.000}, {2, -12.500}, {3, -06.250}, {4, -03.125},
		  //{5,  00.000},
		  //{6,  03.125}, {7,  06.250}, {8,  12.500}, {9,  25.000}, {10,  50.000}
			{0, -50.0}, {1, -20.0}, {2, -06.0},
			{3, 0},
			{4,  06.0}, {5,  20.0}, {6,  50.0}
	};

	for (i = 0; i <= 6; i ++)
	{
		if (b == mapping[i].exhaustive_value)
		{
			steering = mapping[i].steering_output;
			break;
		}
	}

	*steering_command += steering;
	*steering_command  = carmen_clamp(-100.0, *steering_command, 100.0);
}


void
initialize_ann_data(struct fann_train_data **ann_data, int num_data, int num_input, int num_output)
{
	int i;
	*ann_data = (struct fann_train_data *) malloc(sizeof(struct fann_train_data));
	fann_init_error_data((struct fann_error *) *ann_data);
	(*ann_data)->num_data   = num_data;
	(*ann_data)->num_input  = num_input;
	(*ann_data)->num_output = num_output;
	(*ann_data)->input      = (fann_type **) calloc(num_data, sizeof(fann_type *));
	(*ann_data)->output     = (fann_type **) calloc(num_data, sizeof(fann_type *));
	for (i = 0; i < num_data; i ++)
	{
		(*ann_data)->input[i]  = (fann_type*) calloc(num_input, sizeof(fann_type));
		(*ann_data)->output[i] = (fann_type*) calloc(num_output, sizeof(fann_type));
	}
}


double

c(state s, action a __attribute__ ((unused)), state sl __attribute__ ((unused)))
{
#define	DELTA 0.1

	double s_error;

	s_error  = fabs(s.atan_desired_curvature - s.atan_current_curvature);

	if (s_error < DELTA)
		return 0.0;
	else
		return 1.0;
}

void
NFQ_train(struct fann *qlearning_ann, int num_data, state *s, action *a, carmen_simulator_ackerman_config_t *simulator_config)
{
	int k = 0; int b; int N = num_data - 1;
	double gamma = 0.3, target;
	static struct fann_train_data *ann_data;
	static double max_atan_curvature;

	if (ann_data == NULL)
	{
		initialize_ann_data(&ann_data, N*3, 4, 1);
		max_atan_curvature = atan(compute_curvature(carmen_degrees_to_radians(30.0), simulator_config));
	}

	do
	{
		target = c(s[k], a[k], s[k+1]) + gamma * min_b_Q(&b, qlearning_ann, s[k+1], a[k+1], simulator_config);

		ann_data->input[k][0] = s[k].atan_desired_curvature / max_atan_curvature;
		ann_data->input[k][1] = s[k].atan_current_curvature / max_atan_curvature;
		ann_data->input[k][2] = (double) s[k].b / 6.0;
		ann_data->input[k][3] = a[k].steering_command / 100.0;
		ann_data->output[k][0] = target;
		k++;
	} while(k < N);

	fann_train_on_data(qlearning_ann, ann_data, 200, 0, 0.001);
}


void
store_transition_experience_for_latter_training(state *s, action *a, int pattern_number,
		double steering_command, double atan_desired_curvature, double atan_current_curvature, double b)
{
	s[pattern_number].atan_current_curvature = atan_current_curvature;
	s[pattern_number].atan_desired_curvature = atan_desired_curvature;
	s[pattern_number].b                      = b;
	a[pattern_number].steering_command       = steering_command;
}

// Neural Fitted Q Iteration - First Experiences with a data efficient neural reinforcement learning method :
// <http://ml.informatik.uni-freiburg.de/_media/publications/rieecml05.pdf>
// Fig. 1.
void
compute_steering_with_qlearning(double *steering_command, double atan_desired_curvature, double atan_current_curvature, double delta_t, carmen_simulator_ackerman_config_t *simulator_config)
{
	int b;
	unsigned int num_data = 50;
	static state  *s;
	static action *a;
	state  s_temp;
	action a_temp;
	static unsigned int pattern_number = 0;
	static struct fann *qlearning_ann = NULL;
	double min_b_ann __attribute__ ((unused));

	if (delta_t == 0 || atan_desired_curvature == 0.00)
		return;

	if (qlearning_ann == NULL)
	{
		char path[256];
		sprintf(path, "%s/bin/qlearning_ann.net", getenv("CARMEN_HOME"));

		qlearning_ann = fann_create_from_file(path); // "qlearning_ann.net");
		if (qlearning_ann == NULL)
			{printf("Error: Could not create qlearning_ann\n");exit(1);}
		s = (state *) calloc(num_data, sizeof(state));
		a = (action *) calloc(num_data, sizeof(action));
	}

	if (pattern_number == 0) fprintf(stdout, "\n");

	s_temp.atan_desired_curvature = atan_desired_curvature;
	s_temp.atan_current_curvature = atan_current_curvature;
	a_temp.steering_command = *steering_command;
	min_b_ann = min_b_Q(&b, qlearning_ann, s_temp, a_temp, simulator_config);

	qlearning_integrator(steering_command, b);

	fprintf(stdout, "b: %d min_b_Q: %.5lf s: %.5lf ", b, min_b_ann, *steering_command);

	store_transition_experience_for_latter_training(s, a, pattern_number,
		*steering_command, atan_desired_curvature, atan_current_curvature, b);

	if ((pattern_number + 1) == num_data)
		NFQ_train(qlearning_ann, num_data, s, a, simulator_config);

	pattern_number = (pattern_number + 1) % num_data;
}

/*
double
compute_new_phi_with_ann_old(carmen_simulator_ackerman_config_t *simulator_config)
{
	static double steering_command = 0.0;
	double atan_current_curvature;
	double atan_desired_curvature;
	static fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	fann_type *steering_ann_output;
	static struct fann *steering_ann = NULL;

	if (steering_ann == NULL)
	{
		char path[256];
		sprintf(path, "%s/bin/steering_ann.net", getenv("CARMEN_HOME"));

		steering_ann = fann_create_from_file(path); //"steering_ann.net");
		if (steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann\n");
			exit(1);
		}
		init_steering_ann_input(steering_ann_input);
	}

	atan_current_curvature = atan(compute_curvature2(simulator_config->phi, simulator_config));
	atan_desired_curvature = atan(compute_curvature2(simulator_config->target_phi, simulator_config));

	carmen_ford_escape_hybrid_steering_PID_controler(&steering_command,
		atan_desired_curvature, atan_current_curvature, simulator_config->delta_t, 0);

	build_steering_ann_input(steering_ann_input, steering_command, atan_current_curvature);
	steering_ann_output = fann_run(steering_ann, steering_ann_input);

	// Alberto: O ganho de 1.05 abaixo foi necessario pois a rede nao estava gerando curvaturas mais extremas
	// que nao aparecem no treino mas apenas rodando livremente na simulacao
	simulator_config->phi = 1.05 * get_phi_from_curvature2(tan(steering_ann_output[0]), simulator_config);

	return (simulator_config->phi);
}*/


double
compute_new_phi_with_ann(carmen_simulator_ackerman_config_t *simulator_config)
{
	static double steering_effort = 0.0;
	double atan_current_curvature;
	static fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	static struct fann *steering_ann = NULL;

	if (steering_ann == NULL)
	{
		char path[256];
		sprintf(path, "%s/bin/steering_ann.net", getenv("CARMEN_HOME"));

		steering_ann = fann_create_from_file(path); //"steering_ann.net");
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

	double atan_desired_curvature = carmen_get_curvature_from_phi(simulator_config->target_phi, simulator_config->v, simulator_config->understeer_coeficient2,
															simulator_config->distance_between_front_and_rear_axles);


#ifdef __USE_RL_CONTROL

	steering_effot = rl_control_steering;

#else

	if (simulator_config->use_mpc)
	{
		steering_effort = carmen_libmpc_get_optimized_steering_effort_using_MPC(atan_current_curvature,
							simulator_config->current_motion_command_vector, simulator_config->nun_motion_commands,
							simulator_config->v, simulator_config->phi, simulator_config->time_of_last_command, &simulator_config->robot_config,
							simulator_config->initialize_neural_networks);
//		//POSITION CONTROL
//		steering_effort = carmen_libmpc_get_optimized_steering_effort_using_MPC_position_control(atan_current_curvature,
//							simulator_config->current_motion_command_vector, simulator_config->nun_motion_commands,
//							simulator_config->v, simulator_config->phi, simulator_config->time_of_last_command, &simulator_config->robot_config,
//							simulator_config->global_pos, simulator_config->initialize_neural_networks);
	}
	else
	{
		if (simulator_config->use_rlpid)
		{	//RL_PID
			if (simulator_config->current_motion_command_vector != NULL)//&& simulator_config->current_motion_command_vector[2].v > 0.0)
			{
				steering_effort = carmen_librlpid_compute_effort(atan_current_curvature, atan_desired_curvature, simulator_config->delta_t);
				//steering_effort = carmen_librlpid_compute_effort_new(atan_current_curvature, atan_desired_curvature, simulator_config->delta_t);
			}
		}
		else
		{   // PID
			//steering_effort = carmen_libpid_steering_PID_controler(atan_desired_curvature, atan_current_curvature, simulator_config->delta_t, 0);

			steering_effort = carmen_libpid_steering_PID_controler_FUZZY(atan_desired_curvature, atan_current_curvature, simulator_config->delta_t, 0, simulator_config->v);
		}
		#ifdef PLOT
				pid_plot_phi(simulator_config->phi, simulator_config->target_phi, 0.55, "phi");
		#endif
	}

#endif

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
carmen_simulator_ackerman_recalc_pos(carmen_simulator_ackerman_config_t *simulator_config,
		int use_velocity_nn, int use_phi_nn, int connected_to_iron_bird, double iron_bird_v, double iron_bird_phi)
{
	carmen_point_t new_odom;
	carmen_point_t new_true;
	double v, phi;

	update_target_v_and_target_phi(simulator_config);

	if (connected_to_iron_bird)
	{
		v = iron_bird_v;
		phi = iron_bird_phi;
	}
	else
	{
		// Velocity must be calculated before phi
		if (use_velocity_nn)
			v = compute_new_velocity_with_ann(simulator_config);
		else
			v = compute_new_velocity(simulator_config);

		if (use_phi_nn)
			phi = compute_new_phi_with_ann(simulator_config); // + carmen_gaussian_random(0.0, carmen_degrees_to_radians(0.05));
		else
			phi = compute_new_phi(simulator_config); // + carmen_gaussian_random(0.0, carmen_degrees_to_radians(0.1));
	}
#ifdef PLOT_VELOCITY
	pid_plot_velocity(simulator_config->v, simulator_config->target_v, 15.0, "vel");
#endif

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
carmen_simulator_ackerman_calc_laser_msg(carmen_laser_laser_message *laser, 
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

