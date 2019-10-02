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

//#include <carmen/base_interface.h>


static double min_max_tv = 0.0;// , min_max_rv = 0.0;
static double max_max_tv = 1.8, max_max_rv = 0.6;

static double max_allowed_tv = 1.5, max_allowed_rv = 1.5;

static carmen_joystick_type joystick;
static int joystick_activated=0;
static int throttle_mode=0;

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

	if (v.v > max_allowed_tv)
		v.v = max_allowed_tv;
	else if (v.v < -max_allowed_tv)
		v.v = -max_allowed_tv;

	if (v.phi > max_allowed_rv)
		v.phi = max_allowed_rv;
	else if (v.phi < -max_allowed_rv)
		v.phi = -max_allowed_rv;


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
			{"robot", "max_r_vel", CARMEN_PARAM_DOUBLE, &max_max_rv, 1, NULL}
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
				command_tv = +1 * joystick.axes[1] / 32767.0 * max_max_tv;

			if (joystick.axes[3])
				command_rv = -1 * joystick.axes[3] / 32767.0 * max_max_rv;
			else
				command_rv = 0;



			//command_tv = -1 * joystick.axes[4] / 32767.0 * max_tv;
			//command_tv = +1 * joystick.axes[1] / 32767.0 * max_tv;

			if (joystick_activated){
				if(command_tv < 0.07 && command_tv > -0.07)
					command_tv = 0;
				if(command_rv < 0.13 && command_rv > -0.13)
					command_rv = 0;
//				printf("%.2f, %.2f\n", command_tv, command_rv);
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
