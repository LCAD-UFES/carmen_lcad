//#include <carmen/base_ackerman_messages.h>
//#include <carmen/base_ackerman_interface.h>
#include <carmen/carmen.h>
#include <carmen/joyctrl.h>
//usar a mensagem do obstacle avoider
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/simulator_ackerman.h>
//#include <carmen/base_interface.h>
#include <carmen/behavior_selector_interface.h>

static double max_max_tv = 1.8, max_max_rv = 0.2;

static carmen_joystick_type joystick;
static int joystick_activated=0;
carmen_robot_ackerman_config_t robot_config;

carmen_behavior_selector_state_message behavior_selector_state_message;


void
publish_behavior_selector_current_state(carmen_behavior_selector_state_message *msg)
{
	IPC_RETURN_TYPE err;
	// msg->task = 0;
	// msg->algorithm = 0;
	// msg->route_planner_state = 0;
	// msg->offroad_planner_request = 0;
	// msg->timestamp = carmen_get_time();
	// msg->host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}

static double
get_acceleration(double v, double target_v, carmen_robot_ackerman_config_t *simulator_config)
{
	double acceleration;

	if (fabs(target_v) > fabs(v))
		acceleration = target_v >= 0.0 ? simulator_config->maximum_acceleration_forward : simulator_config->maximum_acceleration_reverse;
	else
		acceleration = target_v >= 0.0 ? simulator_config->maximum_deceleration_forward : simulator_config->maximum_deceleration_reverse;

	return (acceleration);
}

double
compute_new_velocity(carmen_robot_ackerman_config_t *simulator_config, double last_command_tv, double target_command_tv, double delta_t)
{
	double minimum_time_to_reach_desired_velocity;
	double acceleration;
	int signal_target_v, signal_v, command_signal;
	double target_v, time, time_rest;

	signal_target_v = target_command_tv >= 0 ? 1 : -1;
	signal_v = last_command_tv >= 0 ? 1 : -1;

	target_v = target_command_tv;

	if (signal_target_v != signal_v)
		target_v = 0;

	acceleration = get_acceleration(last_command_tv, target_v, simulator_config);

	minimum_time_to_reach_desired_velocity = fabs((target_v - last_command_tv) / acceleration); // s = v / a, i.e., s = (m/s) / (m/s^2)

	command_signal = last_command_tv < target_v ? 1 : -1;

	time = fmin(delta_t, minimum_time_to_reach_desired_velocity);

	last_command_tv += command_signal * acceleration * time;

	if (signal_target_v != signal_v)
	{
		time_rest = delta_t - time;
		target_v = target_command_tv;

		acceleration = get_acceleration(last_command_tv, target_v, simulator_config);

		minimum_time_to_reach_desired_velocity = fabs((target_v - last_command_tv) / acceleration); // s = v / a, i.e., s = (m/s) / (m/s^2)

		command_signal = last_command_tv < target_v ? 1 : -1;

		time = fmin(time_rest, minimum_time_to_reach_desired_velocity);

		last_command_tv += command_signal * acceleration * time;
	}

	last_command_tv = carmen_clamp(-simulator_config->max_v, last_command_tv, simulator_config->max_v);

	return (last_command_tv);
}

double
compute_new_phi(carmen_robot_ackerman_config_t *simulator_config, double command_tv, double last_phi, double target_phi, double delta_t)
{
	double current_curvature;
	double desired_curvature;
	double delta_curvature;
	double minimum_time_to_reach_desired_curvature;
	double phi;

	current_curvature = carmen_get_curvature_from_phi(last_phi, command_tv, simulator_config->understeer_coeficient,
			simulator_config->distance_between_front_and_rear_axles);
	desired_curvature = carmen_get_curvature_from_phi(target_phi, command_tv, simulator_config->understeer_coeficient,
			simulator_config->distance_between_front_and_rear_axles);
	delta_curvature = desired_curvature - current_curvature;
	minimum_time_to_reach_desired_curvature = fabs(delta_curvature / simulator_config->maximum_steering_command_rate);

	if (minimum_time_to_reach_desired_curvature > delta_t)
	{
		if (delta_curvature < 0.0)
			current_curvature -= simulator_config->maximum_steering_command_rate * delta_t;
		else
			current_curvature += simulator_config->maximum_steering_command_rate * delta_t;
	}
	else
		current_curvature = desired_curvature;

	double max_c = carmen_get_curvature_from_phi(simulator_config->max_phi, 0.0, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);
	if (fabs(current_curvature) > max_c)
	{
		current_curvature = current_curvature / fabs(current_curvature);
		current_curvature *= max_c;
	}

	phi = carmen_get_phi_from_curvature(current_curvature, simulator_config->max_v, simulator_config->understeer_coeficient, simulator_config->distance_between_front_and_rear_axles);

	//printf("desired_phi = %lf, phi = %lf\n", simulator_config->target_phi, simulator_config->phi);

	return (phi);
}

void define_messages()
{
/*
	IPC_RETURN_TYPE err;
		err = IPC_defineMsg(CARMEN_BASE_VELOCITY_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BASE_VELOCITY_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BASE_VELOCITY_NAME);

*/
	 
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
	
	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, IPC_VARIABLE_LENGTH, CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}

void send_base_velocity_command(double tv, double rv)
{
	static carmen_base_ackerman_velocity_message v;
	int num_commands = 1;
	carmen_robot_and_trailer_motion_command_t *msg =
				(carmen_robot_and_trailer_motion_command_t *) (malloc(num_commands * sizeof(carmen_robot_and_trailer_motion_command_t)));

	v.v = tv;
	v.phi = rv;
	v.timestamp = carmen_get_time();
	v.host = carmen_get_host();

	msg[0].v = v.v;
	msg[0].phi = v.phi;
	msg[0].x = 0.0;
	msg[0].y = 0.0;
	msg[0].theta = 0.0;
	msg[0].beta = 0.0;
	msg[0].time = 1.0;


	carmen_obstacle_avoider_publish_base_ackerman_motion_command(msg, num_commands, v.timestamp);
	free(msg);
}

void sig_handler(int x)
{
	if(x == SIGINT) {
		send_base_velocity_command(0, 0);
		carmen_close_joystick(&joystick);
		carmen_ipc_disconnect();
		printf("Disconnected from robot.\n");
		exit(0);
	}
}

void read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
			{"robot", "max_t_vel", CARMEN_PARAM_DOUBLE, &max_max_tv, 1, NULL},
			{"robot", "max_r_vel", CARMEN_PARAM_DOUBLE, &max_max_rv, 1, NULL},
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
			{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
			{"robot", "maximum_acceleration_reverse", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_reverse, 1, NULL},
			{"robot", "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward, 1, NULL},
			{"robot", "maximum_deceleration_reverse", CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_reverse, 1, NULL},
			{"robot", "desired_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate, 1, NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
			{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}

int 
main(int argc, char **argv)
{
	double command_tv = 0.0;
	double command_rv = 0.0;
//	double f_timestamp;
	behavior_selector_state_message.task = 0;
	behavior_selector_state_message.algorithm = 0;
	behavior_selector_state_message.route_planner_state = 0;
	behavior_selector_state_message.offroad_planner_request = 0;
	behavior_selector_state_message.low_level_state_flags = 0; //seting state to GOING_FOWARD
	behavior_selector_state_message.low_level_state = Free_Running;
				
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	define_messages();

	read_parameters(argc, argv);

	signal(SIGINT, sig_handler);
	
	if (carmen_initialize_joystick(&joystick) < 0)
		carmen_die("Erorr: could not find joystick.\n");

	fprintf(stderr, "1. Set the \"throttle control\" to zero (to the left)\n2. Press \"START\" button to activate the joystick.\n");

	// f_timestamp = carmen_get_time();

	while(1)
	{
		carmen_ipc_sleep(0.1);
		carmen_get_joystick_state(&joystick);
		// if(carmen_get_joystick_state(&joystick) >= 0)
		{
			if (joystick.axes[0]) // Right anolog directinal (right and left controls the steering wheel angle)
			{
				if (joystick.axes[0] > 100)
				{
					command_rv += -0.005;
				}
				if (joystick.axes[0] < -100)
				{
					command_rv += 0.005;
				}
				command_rv = carmen_clamp(-robot_config.max_phi, command_rv, robot_config.max_phi);
			}
			if (joystick.axes[1]) // Right anolog directinal (up and down set steering wheel angle to zero)
			{
				if (joystick.axes[1] > 20000)
				{
					command_rv = 0.0;
				}
				if (joystick.axes[1] < -20000)
				{
					command_rv = 0.0;
				}
			}

			if (joystick.axes[4])    // Left anolog directinal (up and down controls the velocity)
			{
				if (joystick.axes[4] > 100)
				{
					command_tv += -0.02;
				}
				if (joystick.axes[4] < -100)
				{
					command_tv += 0.02;
				}
				command_tv = carmen_clamp(-robot_config.max_v, command_tv, robot_config.max_v);

				if (behavior_selector_state_message.low_level_state_flags == 0 && command_tv < 0.0)
					command_tv = 0.0;
				if (behavior_selector_state_message.low_level_state_flags == CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS && command_tv > 0.0)
					command_tv = 0.0;
			}
			if (joystick.axes[3])    // Left anolog directinal (right and left set velocity to zero)
			{
				if (joystick.axes[3] > 20000)
				{
					command_tv = 0.0;
				}
				if (joystick.axes[3] < -20000)
				{
					command_tv = 0.0;
				}
			}

			// if (joystick.buttons[1])   // B button int the x-box control
			// {
			// 	printf("1\n");
			// }
			if (joystick.buttons[6] && command_tv == 0.0)  // Back button int the x-box control (arrow left)
			{
				behavior_selector_state_message.low_level_state_flags = CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;
				behavior_selector_state_message.low_level_state = Free_Reverse_Running;
				publish_behavior_selector_current_state(&behavior_selector_state_message);
				// printf("6\n");
				printf("REARWARD\n");
			}
			if (joystick.buttons[7] && command_tv == 0.0)  // Start button int the x-box control (arrow right)
			{
				behavior_selector_state_message.low_level_state_flags = 0; //seting state to GOING_FOWARD
				behavior_selector_state_message.low_level_state = Free_Running;
				publish_behavior_selector_current_state(&behavior_selector_state_message);
				// printf("7\n");
				printf("DRIVE\n");
			}

			// if (joystick.axes[0]){
			// 	printf("0 %d\n", joystick.axes[0]);
			// }

			// if (joystick.axes[1]){
			// 	printf("1 %d\n", joystick.axes[1]);
			// }
			// if (joystick.axes[2]){
			// 	printf("2 %d\n", joystick.axes[2]);
			// }
			// if (joystick.axes[3]){
			// 	printf("3 %d\n", joystick.axes[3]);
			// }
			// if (joystick.axes[4]){
			// 	printf("4 %d\n", joystick.axes[4]);
			// }
			// if (joystick.axes[5]){
			// 	printf("5 %d\n", joystick.axes[5]);
			// }
			// if (joystick.axes[6]){
			// 	printf("6 %d\n", joystick.axes[6]);
			// }
			// if (joystick.axes[7]){
			// 	printf("7 %d\n", joystick.axes[7]);
			// }
			// if (joystick.axes[8]){
			// 	printf("8 %d\n", joystick.axes[8]);
			// }
			// if (joystick.axes[9]){
			// 	printf("9 %d\n", joystick.axes[9]);
			// }

			// if (joystick.buttons[0])
			// {
			// 	printf("0\n");
			// // }
			// if (joystick.buttons[1])   // B button int the x-box control
			// {
			// 	printf("1\n");
			// }
			// if (joystick.buttons[2])
			// {
			// 	printf("2\n");
			// }
			// if (joystick.buttons[3])
			// {
			// 	printf("3\n");
			// }
			// if (joystick.buttons[4])
			// {
			// 	printf("4\n");
			// }
			// if (joystick.buttons[5])
			// {
			// 	printf("5\n");
			// }
			// if (joystick.buttons[6])  // Back button int the x-box control (arrow left)
			// {
			// 	printf("6\n");
			// }
			// if (joystick.buttons[7])  // Start button int the x-box control (arrow right)
			// {
			// 	printf("7\n");
			// }
			
			if (joystick_activated)
			{
				fprintf(stderr,"V %.2f   Phi %.2f\n", command_tv, command_rv);
				send_base_velocity_command(command_tv, command_rv);
			}
			if (joystick.buttons[8])
			{
				joystick_activated = !joystick_activated;
				
				if (joystick_activated)
					fprintf(stderr,"Joystick activated!\n");
				else
				{
					fprintf(stderr,"Joystick deactivated!\n");
					command_tv = 0.0;
					command_rv = 0.0;
					send_base_velocity_command(0.0, 0.0);
				}
				carmen_ipc_sleep(0.3);
			}
		}
		// else if(joystick_activated && carmen_get_time() - f_timestamp > 0.5)
		// {
		// 	send_base_velocity_command(command_tv, command_rv);
		// 	f_timestamp = carmen_get_time();
		// }
	}
	sig_handler(SIGINT);
	return 0;
}
