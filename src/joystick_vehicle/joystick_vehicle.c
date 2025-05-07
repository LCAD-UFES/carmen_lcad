#include <carmen/carmen.h>
#include <carmen/joyctrl.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/simulator_ackerman.h>
#include <carmen/behavior_selector_interface.h>

#define MAX_AXIS			32767.0
#define V_NON_LINEARITY		4.0
#define PHI_NON_LINEARITY	4.0

static carmen_joystick_type joystick;
static int joystick_activated = 0;
carmen_robot_ackerman_config_t robot_config;
double robot_max_velocity_reverse;

carmen_behavior_selector_state_message behavior_selector_state_message;

static int direct_v_and_phi_mode = 0;
static int show_state = 0;
static double overwrite_phi = -1.0;
static double overwrite_v = -1.0;
static double overwrite_v_rev = 1.0;

static const double delta_command_v_factor = 0.3; // Controla a demora para a velocidade chegar no comando (valor maior = mais rápido a convergir)

double
non_linear_range(double x, double non_linear_factor)
{
	// value deve estar na faixa [0.0, 1.0] e a saida eh entre [0.0, 1.0]
	// non_linear_factor = 0.1 eh praticamente linear. Valores proximos de 4.0 sao interessantes
	// Teste com:
	// gnuplot> non_linear_factor = 4.0
	// gnuplot> plot [0:1] (exp(non_linear_factor * x) - 1.0) / (exp(non_linear_factor) - 1.0)

	return (exp(non_linear_factor * x) - 1.0) / (exp(non_linear_factor) - 1.0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Publishers																					//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
publish_base_velocity_command(double tv, double rv)
{
	static carmen_base_ackerman_velocity_message v;
	int num_commands = 1;
	carmen_robot_and_trailers_motion_command_t *msg =
				(carmen_robot_and_trailers_motion_command_t *) (malloc(num_commands * sizeof(carmen_robot_and_trailers_motion_command_t)));

	v.v = tv;
	v.phi = rv;
	v.timestamp = carmen_get_time();
	v.host = carmen_get_host();

	msg[0].v = v.v;
	msg[0].phi = v.phi;
	msg[0].x = 0.0;
	msg[0].y = 0.0;
	msg[0].theta = 0.0;
	msg[0].num_trailers = 0;
	msg[0].time = 1.0;

	carmen_obstacle_avoider_publish_base_ackerman_motion_command(msg, num_commands, v.timestamp);
	//carmen_base_ackerman_publish_motion_command(msg,num_commands,v.timestamp);

	free(msg);
}


void
publish_behavior_selector_current_state(carmen_behavior_selector_state_message *msg)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


void
direct_v_and_phi_joystick_mode(double *command_v, double *command_phi) 
{
	//if (joystick.axes[4] <= 0)	// para frente (v >= 0.0)
	//	*command_v = *command_v + 0.3 * ((non_linear_range(-((double) joystick.axes[4] / MAX_AXIS), V_NON_LINEARITY) * robot_config.max_v) - *command_v);
	//else	// para tras (v < 0.0)
	//	*command_v = *command_v + 0.3 * ((non_linear_range(((double) joystick.axes[4] / MAX_AXIS), V_NON_LINEARITY) * robot_max_velocity_reverse) - *command_v);
	if (joystick.axes[4] <= 0)	// para frente (v >= 0.0)
		*command_v = *command_v + delta_command_v_factor * ((non_linear_range(-((double) joystick.axes[4] / MAX_AXIS), V_NON_LINEARITY) * robot_config.max_v) - *command_v);
	else	// para tras (v < 0.0)
		*command_v = *command_v + delta_command_v_factor * ((non_linear_range(((double) joystick.axes[4] / MAX_AXIS), V_NON_LINEARITY) * robot_max_velocity_reverse) - *command_v);
	*command_v = carmen_clamp(robot_max_velocity_reverse, *command_v, robot_config.max_v);

	if (*command_v >= -0.001)
	{
		behavior_selector_state_message.low_level_state_flags = 0; //seting state to GOING_FOWARD
		behavior_selector_state_message.low_level_state = Free_Running;
	}
	else
	{
		behavior_selector_state_message.low_level_state_flags = CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;
		behavior_selector_state_message.low_level_state = Free_Reverse_Running;
	}

	//if (joystick.axes[0] <= 0)	// para frente (v >= 0.0)
	//	*command_phi = non_linear_range(-((double) joystick.axes[0] / MAX_AXIS), PHI_NON_LINEARITY) * robot_config.max_phi;
	//else	// para tras (v < 0.0)
	// 	*command_phi = -non_linear_range(((double) joystick.axes[0] / MAX_AXIS), PHI_NON_LINEARITY) * robot_config.max_phi;
	if (joystick.axes[0] <= 0)	// para frente (v >= 0.0)
		*command_phi = -(((double) joystick.axes[0]) / MAX_AXIS) * robot_config.max_phi;
	else	// para tras (v < 0.0)
		*command_phi = -(((double) joystick.axes[0]) / MAX_AXIS) * robot_config.max_phi;
	// if (joystick.axes[0] <= 0)	// para frente (v >= 0.0)
	// 	*command_phi = *command_phi + 0.06 * ((non_linear_range(-((double) joystick.axes[0] / MAX_AXIS), PHI_NON_LINEARITY) * robot_config.max_phi) - *command_phi);
	// else	// para tras (v < 0.0)
	// 	*command_phi = *command_phi - 0.06 * ((non_linear_range(((double) joystick.axes[0] / MAX_AXIS), PHI_NON_LINEARITY) * robot_config.max_phi) + *command_phi);

	*command_phi = carmen_clamp(-robot_config.max_phi, *command_phi, robot_config.max_phi);

	if (joystick_activated)
	{
		publish_behavior_selector_current_state(&behavior_selector_state_message);
		fprintf(stderr, "V %.2f   Phi %.2f\n", *command_v, *command_phi);
		publish_base_velocity_command(*command_v, *command_phi);
	}

	if (joystick.buttons[8])
	{
		joystick_activated = !joystick_activated;
		if (joystick_activated)
			fprintf(stderr, "Joystick activated!\n");
		else
		{
			fprintf(stderr, "Joystick deactivated!\n");	
			*command_v = 0.0;
			*command_phi = 0.0;
			publish_base_velocity_command(0.0, 0.0);
		}
		carmen_ipc_sleep(0.3);
	}
}


void
default_joystick_mode(double *command_v, double *command_phi)
{
	if (joystick.axes[0]) // Right anolog directinal (right and left controls the steering wheel angle)
	{
		if (joystick.axes[0] > 100)
			*command_phi += -0.005;

		if (joystick.axes[0] < -100)
			*command_phi += 0.005;

		*command_phi = carmen_clamp(-robot_config.max_phi, *command_phi, robot_config.max_phi);
	}
	if (joystick.axes[1]) // Right anolog directinal (up and down set steering wheel angle to zero)
	{
		if (joystick.axes[1] > 20000)
			*command_phi = 0.0;

		if (joystick.axes[1] < -20000)
			*command_phi = 0.0;
	}
	if (joystick.axes[4]) // Left anolog directinal (up and down controls the velocity)
	{
		if (joystick.axes[4] > 100)
			*command_v += -0.02;

		if (joystick.axes[4] < -100)
			*command_v += 0.02;

		*command_v = carmen_clamp(-robot_config.max_v, *command_v, robot_config.max_v);
		if (behavior_selector_state_message.low_level_state_flags == 0 && *command_v < 0.0)
			*command_v = 0.0;

		if (behavior_selector_state_message.low_level_state_flags == CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS && *command_v > 0.0)
			*command_v = 0.0;
	}
	if (joystick.axes[3]) // Left anolog directinal (right and left set velocity to zero)
	{
		if (joystick.axes[3] > 20000)
			*command_v = 0.0;

		if (joystick.axes[3] < -20000)
			*command_v = 0.0;
	}
	if (joystick.buttons[6] && *command_v == 0.0) // Back button int the x-box control (arrow left)
	{
		behavior_selector_state_message.low_level_state_flags = CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;
		behavior_selector_state_message.low_level_state = Free_Reverse_Running;
		publish_behavior_selector_current_state(&behavior_selector_state_message);
		// printf("6\n");
		printf("REARWARD\n");
	}
	if (joystick.buttons[7] && *command_v == 0.0) // Start button int the x-box control (arrow right)
	{
		behavior_selector_state_message.low_level_state_flags = 0; //seting state to GOING_FOWARD
		behavior_selector_state_message.low_level_state = Free_Running;
		publish_behavior_selector_current_state(&behavior_selector_state_message);
		// printf("7\n");
		printf("DRIVE\n");
	}
	if (joystick_activated)
	{
		fprintf(stderr, "V %.2f   Phi %.2f\n", *command_v, *command_phi);
		publish_base_velocity_command(*command_v, *command_phi);
	}
	if (joystick.buttons[8])
	{
		joystick_activated = !joystick_activated;
		if (joystick_activated)
			fprintf(stderr, "Joystick activated!\n");
		else
		{
			fprintf(stderr, "Joystick deactivated!\n");
			*command_v = 0.0;
			*command_phi = 0.0;
			publish_base_velocity_command(0.0, 0.0);
		}
		carmen_ipc_sleep(0.1);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
sig_handler(int x)
{
	if (x == SIGINT)
	{
		publish_base_velocity_command(0, 0);
		carmen_close_joystick(&joystick);
		carmen_ipc_disconnect();
		printf("Disconnected from robot.\n");
		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, IPC_VARIABLE_LENGTH, CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}


void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
			{"robot", "max_velocity_reverse", CARMEN_PARAM_DOUBLE, &robot_max_velocity_reverse, 1, NULL},
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

	carmen_param_allow_unfound_variables(1);
	carmen_param_t optional_param_list[] =
	{
			{(char *) "commandline", (char *) "direct_v_and_phi_mode", CARMEN_PARAM_ONOFF, &direct_v_and_phi_mode, 0, NULL},
			{(char *) "commandline", (char *) "show_state", CARMEN_PARAM_ONOFF, &show_state, 0, NULL},
			{(char *) "commandline", (char *) "overwrite_phi", CARMEN_PARAM_DOUBLE, &overwrite_phi, 0, NULL},
			{(char *) "commandline", (char *) "overwrite_v", CARMEN_PARAM_DOUBLE, &overwrite_v, 0, NULL},
			{(char *) "commandline", (char *) "overwrite_v_rev", CARMEN_PARAM_DOUBLE, &overwrite_v_rev, 0, NULL},
	};
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	if(overwrite_phi > 0)
		robot_config.max_phi = overwrite_phi;
	if(overwrite_v > 0)
		robot_config.max_v = overwrite_v;
	if(overwrite_v_rev < 0)
		robot_max_velocity_reverse = overwrite_v_rev;
}


int 
main(int argc, char **argv)
{
	double command_v = 0.0;
	double command_phi = 0.0;

	memset(&behavior_selector_state_message, 0, sizeof(behavior_selector_state_message));
	behavior_selector_state_message.low_level_state = Free_Running;
				
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	define_messages();
	read_parameters(argc, argv);
	signal(SIGINT, sig_handler);
	
	if (carmen_initialize_joystick(&joystick) < 0)
		carmen_die("Erorr: could not find joystick.\n");

	fprintf(stderr, "\nPress \"START\" (or \"SELECT\", or \"SHARE\"...) button to activate the joystick.\n");

	while (1)
	{
		carmen_ipc_sleep(0.05);

		carmen_get_joystick_state(&joystick);
		if (show_state)
			print_joystick_state(joystick);

		if (!direct_v_and_phi_mode)
			default_joystick_mode(&command_v, &command_phi);
		else
			direct_v_and_phi_joystick_mode(&command_v, &command_phi);
	}

	sig_handler(SIGINT);

	return (0);
}
