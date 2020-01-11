/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

//#include <carmen/base_ackerman_messages.h>
//#include <carmen/base_ackerman_interface.h>
#include <carmen/carmen.h>
#include <carmen/joyctrl.h>
//usar a mensagem do obstacle avoider
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/simulator_ackerman.h>
//#include <carmen/base_interface.h>


static double min_max_tv = 0.0;// , min_max_rv = 0.0;
static double max_max_tv = 1.8, max_max_rv = 0.2;

static double max_allowed_tv = 1.5, max_allowed_rv = 1.5;

static carmen_joystick_type joystick;
static int joystick_activated=0;
static int throttle_mode=0;
carmen_robot_ackerman_config_t robot_config;
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

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
	
}

void send_base_velocity_command(double tv, double rv)
{


	// converting tv from meters to km/h
	tv = tv * 3.6;
	// New Message
	IPC_RETURN_TYPE err;
	static carmen_base_ackerman_velocity_message v;
	int num_commands = 1;
	carmen_ackerman_motion_command_t* message_pioneer =
				(carmen_ackerman_motion_command_t*) (malloc(num_commands * sizeof(carmen_ackerman_motion_command_t)));

	v.v = tv;
	v.phi = rv;
	v.timestamp = carmen_get_time();
	v.host = carmen_get_host();

	if (v.v > robot_config.max_v)
		v.v = robot_config.max_v;
	else if (v.v < -robot_config.max_v)
		v.v = -robot_config.max_v;

	if (v.phi > robot_config.max_phi)
		v.phi = robot_config.max_phi;
	else if (v.phi < -robot_config.max_phi)
		v.phi = -robot_config.max_phi;



	if (0)
		fprintf(stderr,"%.2f %.2f\n",v.v, v.phi);


	message_pioneer[0].v = v.v;
	message_pioneer[0].phi = v.phi;
	message_pioneer[0].x = 0.0;
	message_pioneer[0].y = 0.0;
	message_pioneer[0].theta = 0.0;
	message_pioneer[0].time = 1.0;



	//err = IPC_publishData(CARMEN_BASE_ACKERMAN_VELOCITY_NAME, &v);
	//carmen_test_ipc(err, "Could not publish", CARMEN_BASE_ACKERMAN_VELOCITY_NAME);
	carmen_obstacle_avoider_publish_base_ackerman_motion_command(message_pioneer, num_commands, v.timestamp);
	free(message_pioneer);


	

	// Old Message
/*
	IPC_RETURN_TYPE err;
		static carmen_base_velocity_message v;

		v.tv = tv;
		v.rv = rv;
		v.timestamp = carmen_get_time();
		v.host = carmen_get_host();

		if (v.tv > max_allowed_tv)
			v.tv = max_allowed_tv;
		else if (v.tv < -max_allowed_tv)
			v.tv = -max_allowed_tv;

		if (v.rv > max_allowed_rv)
			v.rv = max_allowed_rv;
		else if (v.rv < -max_allowed_rv)
			v.rv = -max_allowed_rv;

		if (1)
			fprintf(stderr, "%.2f %.2f\n", v.tv, v.rv);

		err = IPC_publishData(CARMEN_BASE_VELOCITY_NAME, &v);
		carmen_test_ipc(err, "Could not publish", CARMEN_BASE_VELOCITY_NAME);
*/
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

int main(int argc, char **argv)
{
	double command_tv = 0, command_rv = 0;
	double max_tv = 0;
	double f_timestamp;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	define_messages();
	if (carmen_initialize_joystick(&joystick) < 0)
		carmen_die("Erorr: could not find joystick.\n");

	if (joystick.nb_axes != 9 || joystick.nb_buttons != 12)
		fprintf(stderr,"This seems to be *NOT* a \"Logitech WingMan Cordless Rumble Pad\",\nbut I will start anyway (be careful!).\n\n");

	read_parameters(argc, argv);

	signal(SIGINT, sig_handler);

	fprintf(stderr,"1. Set the \"throttle control\" to zero (to the left)\n2. Press \"START\" button to activate the joystick.\n");

//	if (throttle_mode)
//		fprintf(stderr,"3. Use the \"throttle control\" to control the speed!\n\n");

	f_timestamp = carmen_get_time();
	while(1) {
		carmen_ipc_sleep(0.1);
		if(carmen_get_joystick_state(&joystick) >= 0) {

			max_tv = min_max_tv + (-joystick.axes[2] + 32767.0) / (32767.0 * 2.0) * (max_max_tv - min_max_tv);
			double delta_t = carmen_get_time() - f_timestamp;
			//double velocidade =  joystick.axes[1] + ((joystick.axes[1]<0)-(joystick.axes[1]>0))*4000;
			//double angulo_roda =joystick.axes[3] +1000 + ((joystick.axes[3]<0)-(joystick.axes[3]>0))*4000;
			double new_command_rv;
			double desired_command_rv;
			double new_command_tv;
			double desired_command_tv;
			//      max_rv = min_max_rv + (-joystick.axes[2] + 32767.0) / (32767.0 * 2.0) * (max_max_rv - min_max_rv);

/*			if (throttle_mode) {
				command_tv = max_tv;

				if(joystick.axes[6] && command_tv > 0)
					command_tv *= -1;

				if (fabs(joystick.axes[1] / 32767.0) > 0.25   ||
						fabs(joystick.axes[0] / 32767.0) > 0.25)
					command_tv = 0;
			}
			else */
				//command_tv = +1 * velocidade / 32767.0 * max_max_tv;

			if (joystick.axes[1])
			{
				desired_command_tv = (float) (+1.0 * joystick.axes[1] / 32767.0 * robot_config.max_v);
				if(desired_command_tv < 1.0 && desired_command_tv > -1.0)
					desired_command_tv = 0.0;
	//			fprintf(stderr,"%.2f, %.2f\n", desired_command_tv, desired_command_rv);
				new_command_tv = compute_new_velocity(&robot_config, command_tv, desired_command_tv, delta_t);
				command_tv = new_command_tv;
			}


			if (joystick.axes[3])
			{
				desired_command_rv = (float)( -1.0 * joystick.axes[3] / 32767.0 * robot_config.max_phi);
				if(desired_command_rv < 0.13 && desired_command_rv > -0.13)
					desired_command_rv = 0.0;
				new_command_rv = compute_new_phi(&robot_config, command_tv, command_rv, desired_command_rv, delta_t);
				command_rv = new_command_rv;
			}
			else
				command_rv = 0;



			//command_tv = -1 * joystick.axes[4] / 32767.0 * max_tv;
			//command_tv = +1 * joystick.axes[1] / 32767.0 * max_tv;

			if (joystick_activated){
//				if(command_tv < 0.07 && command_tv > -0.07)
//					command_tv = 0;
//				if(command_rv < 0.13 && command_rv > -0.13)
//					command_rv = 0;
				fprintf(stderr,"%.2f, %.2f\n", command_tv, command_rv);
				send_base_velocity_command(command_tv, command_rv);
			}

/*
			if (joystick.buttons[7] || joystick.buttons[9] || joystick.buttons[10] ||
					joystick.buttons[6]) {
				throttle_mode = !throttle_mode;
				fprintf(stderr,"throttle control is %s! ", (throttle_mode?"ON":"OFF"));
				if (throttle_mode)
					fprintf(stderr,"Use the \"throttle control\" to control the speed!\n");
				else
					fprintf(stderr,"\n");
			}
*/

/*
			if (joystick.buttons[3]){
				system("espeak -v pt -s 200 -p 60 'Mamãe!'");
			}
			if (joystick.buttons[2]){
				system("espeak -v pt -s 100 -p 0 'Mamãe!'");
			}
*/
			if (joystick.buttons[8]) {
				joystick_activated = !joystick_activated;
				if (joystick_activated)
					fprintf(stderr,"Joystick activated!\n");
				else {
					fprintf(stderr,"Joystick deactivated!\n");
					send_base_velocity_command(0, 0);
				}
			}
		}
		else if(joystick_activated && carmen_get_time() - f_timestamp > 0.5) {
			send_base_velocity_command(command_tv, command_rv);
			f_timestamp = carmen_get_time();
		}
	}
	sig_handler(SIGINT);
	return 0;
}
