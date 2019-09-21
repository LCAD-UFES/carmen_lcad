/*****************************************************************************
 *  Copyright (c) 2012, lcad.inf.ufes.br; 2009, OpenJAUS.com
 *  All rights reserved.
 *
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD
 *  license.  See the LICENSE file for details.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: main.c
//
// Written By: Alberto F. De Souza
//
// Version: 0.0.1
//
// Date: 02/21/2012
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/signal.h>
#include <termios.h>
#include <curses.h>
#include <math.h>
#include <jaus.h>				// Header file for JAUS types, structs and messages
#include <openJaus.h>				// Header file for the OpenJAUS specific C/C++ code base
#include <torc.h>

struct termios storedTermio;

double g_throttle_command = MIN_THROTTLE;
double g_steering_command = (MAX_STEERING - MIN_STEERING) / 2.0 + MIN_STEERING;
double g_brakes_command = MAX_BRAKES;
double g_velocity_command = (MAX_VELOCITY - MIN_VELOCITY) / 2.0 + MIN_VELOCITY;
double g_acceleration_command = 1.0;
double g_atan_curvature_command = (MAX_ARCTAN_DESIRED_CURVATURE - MIN_ARCTAN_DESIRED_CURVATURE) / 2.0 + MIN_ARCTAN_DESIRED_CURVATURE;
double g_rate_change_curvature_command = 0.06;
JausUnsignedShort g_time_duration_command = MOTION_PROFILE_TIME_DURATION; // milliseconds
JausByte g_turn_signal_command = 0;
JausByte g_horn_status_command = 0;
JausByte g_headlights_status_command = 0;
int g_gear_command = 0;
int g_engine_command = 1; // Engine On

int g_num_errors = 0;
int *g_error = NULL;
ErrorDescription g_error_description[] = 
{
	{9101, XGV_ERROR,   "steering initialization error"},
	{9103, XGV_ERROR,   "steering comms error"},
	{9104, XGV_ERROR,   "steering feedback error"},
	{9106, XGV_ERROR,   "throttle comms error"},
	{9107, XGV_ERROR,   "speed feedback error"},
	{9108, XGV_ERROR,   "wheel speed feedback error"},
	{9109, XGV_ERROR,   "brake error"},
	{9110, XGV_ERROR,   "brake comms error"},
	{9111, XGV_ERROR,   "brake reduced duty cycle"},
	{9113, XGV_ERROR,   "shifter comms error"},
	{9115, XGV_ERROR,   "transmission comms error"},
	{9137, XGV_ERROR,   "SafeStop comms error"},
	{9138, XGV_ERROR,   "SafeStop link loss"},
	{9139, XGV_ERROR,   "check fuses"},
	{9143, XGV_WARNING, "E-stop disable"},
	{9144, XGV_WARNING, "E-stop pause"},
	{9145, XGV_ERROR,   "power distribution comms error"},
	{9149, XGV_ERROR,   "devices comms error"},
	{9150, XGV_ERROR,   "XGV_WARNING horn error"},
	{9153, XGV_ERROR,   "XGV_WARNING light error"},
	{9156, XGV_ERROR,   "XGV_WARNING module comms error"},
	{9157, XGV_ERROR,   "motion profiles timeout"},
	{9158, XGV_ERROR,   "wrench efforts timeout"},
	{9159, XGV_ERROR,   "TORC CAN bus error"},
	{9160, XGV_ERROR,   "Ford CAN bus error"},
	{9161, XGV_ERROR,   "fuel level feedback error"},
	{9162, XGV_ERROR,   "door status comms error"},
	{9163, XGV_ERROR,   "12V Battery Voltage"},
	{9164, XGV_ERROR,   "CPU Usage High"},
	{9165, XGV_ERROR,   "Reset to defaults failed"},
	{9175, XGV_WARNING, "parking brake applied"},
	{9176, XGV_WARNING, "door open"},
	{9177, XGV_WARNING, "steering not initialized"},
	{9178, XGV_WARNING, "not ready to run"},
	{9179, XGV_WARNING, "restarting controller"},
	{9180, XGV_WARNING, "updating XGV controller firmware"},
	{-1,   XGV_ERROR,   "unknown XGV error code"},
	{0,    XGV_WARNING, "no error"}
};


int g_control_mode = SOFTWARE_PAUSE_MODE;

double g_XGV_throttle = 0.0;
double g_XGV_steering = 0.0;
double g_XGV_brakes = 0.0;
double g_XGV_velocity = 0.0;
double g_XGV_atan_curvature = 0.0;
double g_XGV_right_front_wheel_speed = 0.0;
double g_XGV_left_front_wheel_speed = 0.0;
double g_XGV_right_rear_wheel_speed = 0.0;
double g_XGV_left_rear_wheel_speed = 0.0;
JausByte g_XGV_turn_signal = 0;
JausByte g_XGV_horn_status = 0;
JausByte g_XGV_headlights_status = 0;
int g_XGV_main_propulsion = 0;
int g_XGV_main_fuel_supply = 0;
int g_XGV_parking_brake = 0;
int g_XGV_gear = 128; // Neutral
int g_XGV_num_errors = 0;
JausInteger g_XGV_error[MAX_ERRORS];
unsigned int g_XGV_component_status = 0;

double g_wheels_speed_update_freq = 0.0;
double g_atan_curvature_update_freq = 0.0;

double factor = 1.0;


void
get_errors_descriptions()
{
	int i, j, found;
	
	if (g_error != NULL)
		free(g_error);
	g_error = (int *) malloc(g_XGV_num_errors * sizeof(int));
	
	for (i = 0; i < g_XGV_num_errors; i++)
	{
		found = 0;
		for (j = 0; g_error_description[j].error_code != 0; j++)
		{
			if (g_XGV_error[i] == g_error_description[j].error_code)
			{
				g_error[i] = j;
				found = 1;
				break;
			}
		}
		if (!found)
		{
			if (g_XGV_error[i] == 0)
				g_error[i] = j; // no error
			else
				g_error[i] = j - 1; // error code not recognized
		}
	}
}


void 
print_interface()
{
	int row, col, i;
	char temp[1024];
	char errors[1024];
	
	clear();
	
	row = 1;
	col = 1;
	mvprintw(row++, col, " XGV_CCU Help");
	mvprintw(row++, col, "   w - Wrench Efforts Mode");
	mvprintw(row++, col, "   m - Motion Profiles Mode");
	mvprintw(row++, col, "   Up, Down, Left Wright - Throttle and Steering");
	mvprintw(row++, col, "   b, B - Breaks+, Breaks-");
	mvprintw(row++, col, "   SPACE - Maximum Brake, Minimum Throttle, Central Steering");
	mvprintw(row++, col, "   a, s, d, f - Gear: L, D, N, R");
	mvprintw(row++, col, "   z, x - Engine: On, Off");
	mvprintw(row++, col, "   1, 2, 3, 4 - Turn Signal: Off, Left, Right, Flashes");
	mvprintw(row++, col, "   5 - Horn (On/Off)");
	mvprintw(row++, col, "   6, 7, 8 - Headlights: Off, Parking lights, On");
	mvprintw(row++, col, "   9, 0 - (if Headlight=On) High beams (On/Off), Fog lights (On/Off)");
	mvprintw(row++, col, " ESC - Exit XGV_CCU");
	
	row++;
	mvprintw(row++, col, "g_throttle_command = %lf,   g_XGV_throttle  = %lf", g_throttle_command, g_XGV_throttle);
	mvprintw(row++, col, "g_steering_command = %lf,   g_XGV_steering  = %lf", g_steering_command, g_XGV_steering);
	mvprintw(row++, col, "g_brakes_command   = %lf,   g_XGV_brakes    = %lf", g_brakes_command, g_XGV_brakes);

	row++;
	mvprintw(row++, col, "g_atan_curvature_command = %lf,  g_XGV_atan_curvature = %lf", g_atan_curvature_command, g_XGV_atan_curvature);
	mvprintw(row++, col, "g_velocity_command       = %lf,  g_XGV_velocity       = %lf", g_velocity_command, g_XGV_velocity);
	
	mvprintw(row++, col, "Wheels Speed: Front(L,R), Rear(L,R) = (%lf,%lf), (%lf,%lf)",
		 	g_XGV_right_front_wheel_speed, g_XGV_left_front_wheel_speed,
			g_XGV_right_rear_wheel_speed, g_XGV_left_rear_wheel_speed);
	
	row++;
	mvprintw(row++, col, "Wheels Speed Update (Hz): = %lf, Atan Curvature Update (Hz): = %lf",
			g_wheels_speed_update_freq, g_atan_curvature_update_freq);

	row++;
	mvprintw(row++, col, "g_XGV_main_propulsion  = %d", g_XGV_main_propulsion);
	mvprintw(row++, col, "g_XGV_main_fuel_supply = %d", g_XGV_main_fuel_supply);
	mvprintw(row++, col, "g_XGV_parking_brake    = %d", g_XGV_parking_brake);
	if (g_XGV_gear == 0)
		mvprintw(row++, col, "g_XGV_gear             = P");
	if (g_XGV_gear == 1)
		mvprintw(row++, col, "g_XGV_gear             = L");
	if ((g_XGV_gear >= 2) && (g_XGV_gear <= 127))
		mvprintw(row++, col, "g_XGV_gear             = D");
	if (g_XGV_gear == 128)
		mvprintw(row++, col, "g_XGV_gear             = N");
	if ((g_XGV_gear >= 129) && (g_XGV_gear <= 255))
		mvprintw(row++, col, "g_XGV_gear             = R");
	
	if (g_XGV_turn_signal == 0)
		strcpy(temp, "Turn Signal = Off");
	else if (g_XGV_turn_signal == 1)
		strcpy(temp, "Turn Signal = Left");
	else if (g_XGV_turn_signal == 2)
		strcpy(temp, "Turn Signal = Right");
	else if (g_XGV_turn_signal == 3)
		strcpy(temp, "Turn Signal = Flashes");
	
	if (g_XGV_horn_status == 0)
		strcat(temp, ", Horn = Off");
	else if (g_XGV_horn_status == 1)
		strcat(temp, ", Horn = On");
	
	if ((g_XGV_headlights_status & 7) == 0)
		strcat(temp, ", Headlights = Off");
	else if ((g_XGV_headlights_status & 7) == 1)
		strcat(temp, ", Parking lights");
	else if ((g_XGV_headlights_status & 7) == 2)
	{
		strcat(temp, ", Headlights = On");
		if (g_XGV_headlights_status & 8)
			strcat(temp, ", High beans = On");
		if (g_XGV_headlights_status & 0x10)
			strcat(temp, ", Fog lights = On");
	}
	strcat(temp, ", g_XGV_component_status = ");
	sprintf(temp + strlen(temp), "%d", g_XGV_component_status);
	mvprintw(row++, col, "%s", temp);
	
	get_errors_descriptions();
	row++;
	if (g_XGV_num_errors == 0)
		strcpy(errors, "");
	else if (g_XGV_num_errors == 1)
	{
		sprintf(errors, ", error: (%d) %s", g_error_description[g_error[0]].error_code, g_error_description[g_error[0]].error_description);
	}
	else
	{
		strcpy(errors, ", errors:");
		for (i = 0; i < g_XGV_num_errors; i++)
			sprintf(errors + strlen(errors), " (%d) %s ", g_error_description[g_error[i]].error_code, g_error_description[g_error[i]].error_description);
	}
	switch (g_control_mode)
	{
		case SOFTWARE_PAUSE_MODE:
			mvprintw(row++, col, "mode: SOFTWARE_PAUSE_MODE%s", errors);
			break;
			
		case WRENCH_EFFORTS_MODE:
			mvprintw(row++, col, "mode: WRENCH_EFFORTS_MODE%s", errors);
			break;
			
		case MOTION_PROFILES_MODE:
			mvprintw(row++, col, "mode: MOTION_PROFILES_MODE%s", errors);
			break;			
	}
	
	//move(0,0);
	refresh();
}	


void
manage_errors()
{
	if (g_XGV_num_errors == 0)
	{
		g_num_errors = 0;
		print_interface();
	}
	else
	{
		g_num_errors = g_XGV_num_errors;
//		get_errors_descriptions();
		print_interface();
	}
}


void
change_control_mode_to_wrench_efforts(OjCmpt XGV_CCU)
{
	send_release_control_message(XGV_CCU, JAUS_MOTION_PROFILE_DRIVER);
	send_request_control_message(XGV_CCU, JAUS_PRIMITIVE_DRIVER);
	send_resume_message(XGV_CCU, JAUS_PRIMITIVE_DRIVER);
	g_control_mode = WRENCH_EFFORTS_MODE;
}


void
change_control_mode_to_motion_profiles(OjCmpt XGV_CCU)
{
	send_release_control_message(XGV_CCU, JAUS_PRIMITIVE_DRIVER);
	send_request_control_message(XGV_CCU, JAUS_MOTION_PROFILE_DRIVER);
	send_resume_message(XGV_CCU, JAUS_PRIMITIVE_DRIVER);
	send_resume_message(XGV_CCU, JAUS_MOTION_PROFILE_DRIVER);
	g_control_mode = MOTION_PROFILES_MODE;
}


void
user_interface(OjCmpt XGV_CCU)
{
	struct termios newTermio;
	int running = 1;
	char choice[8] = {0};
	int count = 0;

	// Prepare terminal for receiving key commands
	tcgetattr(0, &storedTermio);
	memcpy(&newTermio, &storedTermio, sizeof(struct termios));

	newTermio.c_lflag &= (~ICANON);
	newTermio.c_lflag &= (~ECHO);
	newTermio.c_cc[VTIME] = 0;
	newTermio.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &newTermio);

	// Start up Curses window
	initscr();
	//cbreak();
	//noecho();
	//nodelay(stdscr, 1);	// Don't wait at the getch() function if the user hasn't hit a key
	keypad(stdscr, 1); // Allow Function key input and arrow key input

	while (running)
	{
		print_interface();
		memset(choice, 0, 8);
		count = read(0, &choice, 8);
		if (count == 1)
		{
			switch (choice[0])
			{
				case 27: // ESC
					printf("Shutting Down XGV_CCU...\n");
					running = 0;
					break;

				case 'W':
				case 'w':
					change_control_mode_to_wrench_efforts(XGV_CCU);
					break;

				case 'M':
				case 'm':
					change_control_mode_to_motion_profiles(XGV_CCU);
					break;

				case ' ':
					g_brakes_command = MAX_BRAKES;
					g_throttle_command = MIN_THROTTLE;
					g_steering_command = (MAX_STEERING - MIN_STEERING) / 2.0 + MIN_STEERING;
					
					g_velocity_command = (MAX_VELOCITY - MIN_VELOCITY) / 2.0 + MIN_VELOCITY;					
					g_acceleration_command = 1.0;
					g_atan_curvature_command = (MAX_ARCTAN_DESIRED_CURVATURE - MIN_ARCTAN_DESIRED_CURVATURE) / 2.0 + MIN_ARCTAN_DESIRED_CURVATURE;
					g_rate_change_curvature_command = 0.06;
					break;

				case 'a':
					g_gear_command = 1; // Low
					send_set_discrete_devices_message(XGV_CCU);
					break;
					
				case 's':
					g_gear_command = 2; // Drive
					send_set_discrete_devices_message(XGV_CCU);
					break;
					
				case 'd':
					g_gear_command = 128; // Neutral
					send_set_discrete_devices_message(XGV_CCU);
					break;
					
				case 'f':
					g_gear_command = 129; // Reverse
					send_set_discrete_devices_message(XGV_CCU);
					break;
					
				case 'z':
					g_engine_command = 1; // Turn engine On
					send_set_engine_message(XGV_CCU, g_engine_command);
					break;
					
				case 'x':
					g_engine_command = 0; // Turn engine Off
					send_set_engine_message(XGV_CCU, g_engine_command);
					break;
					
				case '1':
					g_turn_signal_command = 0;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '2':
					g_turn_signal_command = 1;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '3':
					g_turn_signal_command = 2;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '4':
					g_turn_signal_command = 3;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '5':
					if (g_horn_status_command == 0)
						g_horn_status_command = 1;
					else if (g_horn_status_command == 1)
						g_horn_status_command = 0;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '6':
					g_headlights_status_command = 0;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '7':
					g_headlights_status_command = 1;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '8':
					g_headlights_status_command = 2;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '9':
					if ((g_headlights_status_command & 7) == 2)
						g_headlights_status_command ^= 8;
					send_set_signals_message(XGV_CCU);
					break;
					
				case '0':
					if ((g_headlights_status_command & 7) == 2)
						g_headlights_status_command ^= 0x10;
					send_set_signals_message(XGV_CCU);
					break;

				case 'k':
					factor /= 2.0;
					break;

				case 'K':
					factor *= 2.0;
					break;

				case 'b':
					g_brakes_command += factor * (MAX_BRAKES - MIN_BRAKES) / 100.0;
					if (g_brakes_command > MAX_BRAKES)
						g_brakes_command = MAX_BRAKES;
					break;

				case 'B':
					g_brakes_command -= factor * (MAX_BRAKES - MIN_BRAKES) / 100.0;
					if (g_brakes_command < MIN_BRAKES)
						g_brakes_command = MIN_BRAKES;
					break;

				default:
					break;
			}
		}
		else
		{
			switch (choice[2])
			{
				case 65: // Up
					g_throttle_command += factor * (MAX_THROTTLE - MIN_THROTTLE) / 100.0;
					if (g_throttle_command > MAX_THROTTLE)
						g_throttle_command = MAX_THROTTLE;
					g_brakes_command = 0.0;

					g_velocity_command += factor * (MAX_VELOCITY - MIN_VELOCITY) / 100.0;
					if (g_velocity_command > MAX_VELOCITY)
						g_velocity_command = MAX_VELOCITY;
					break;

				case 66: // Down
					g_throttle_command -= factor * (MAX_THROTTLE - MIN_THROTTLE) / 100.0;
					if (g_throttle_command < MIN_THROTTLE)
					{
						g_throttle_command = MIN_THROTTLE;
						g_brakes_command += factor * (MAX_BRAKES - MIN_BRAKES) / 100.0;
						if (g_brakes_command > MAX_BRAKES)
							g_brakes_command = MAX_BRAKES;
					}
					g_velocity_command -= factor * (MAX_VELOCITY - MIN_VELOCITY) / 100.0;
					if (g_velocity_command < MIN_VELOCITY)
						g_velocity_command = MIN_VELOCITY;
					break;

				case 67: // Right
					g_steering_command += factor * (MAX_STEERING - MIN_STEERING) / 100.0;
					if (g_steering_command > MAX_STEERING)
						g_steering_command = MAX_STEERING;

					g_atan_curvature_command += factor * (MAX_ARCTAN_DESIRED_CURVATURE - MIN_ARCTAN_DESIRED_CURVATURE) / 100.0;
					if (g_atan_curvature_command > MAX_ARCTAN_DESIRED_CURVATURE)
						g_atan_curvature_command = MAX_ARCTAN_DESIRED_CURVATURE;
					break;
					
				case 68: // Left
					g_steering_command -= factor * (MAX_STEERING - MIN_STEERING) / 100.0;
					if (g_steering_command < MIN_STEERING)
						g_steering_command = MIN_STEERING;
					
					g_atan_curvature_command -= factor * (MAX_ARCTAN_DESIRED_CURVATURE - MIN_ARCTAN_DESIRED_CURVATURE) / 100.0;
					if (g_atan_curvature_command < MIN_ARCTAN_DESIRED_CURVATURE)
						g_atan_curvature_command = MIN_ARCTAN_DESIRED_CURVATURE;
					break;

				default:
					break;
			}
		}
	}
	endwin();
	tcsetattr(0, TCSANOW, &storedTermio);
}


void 
xgv_ccu_state_machine(OjCmpt XGV_CCU)			// Ready state callback function of the xgv_ccu
{
	if (g_control_mode == WRENCH_EFFORTS_MODE)
		send_set_wrench_efforts_message(XGV_CCU);
	else if (g_control_mode == MOTION_PROFILES_MODE)
		send_set_motion_profile_message(XGV_CCU);
	if ((g_XGV_num_errors != 0) || (g_num_errors != 0))
		manage_errors();
}


OjCmpt
create_xgv_ccu_component(char *xgv_ccu_name, int xgv_ccu_component_id, double xgv_ccu_state_machine_update_rate)
{
	OjCmpt XGV_CCU;		// Variable that will store the component reference
	
	XGV_CCU = ojCmptCreate(xgv_ccu_name, xgv_ccu_component_id, xgv_ccu_state_machine_update_rate);	// Create the component
	if (XGV_CCU == NULL)
	{
		printf("Error creating %s component. Is ojNodeManager running?\n", xgv_ccu_name);
		return(NULL);
	}
	
	ojCmptAddService(XGV_CCU, xgv_ccu_component_id);				// Add XGV_CCU service type
	add_xgv_ccu_component_service_messages(XGV_CCU, xgv_ccu_component_id);
	
	//TODO função abaixo foi comentada pois gera 50Htz de mensagens desnecessarias de xgv_ccu_state_machine.
//	ojCmptSetStateCallback(XGV_CCU, JAUS_READY_STATE, xgv_ccu_state_machine);	// Set ready state callback
	ojCmptSetState(XGV_CCU, JAUS_READY_STATE);					// Set the current state to ready
	return (XGV_CCU);
}


OjCmpt
create_xgv_ccu_component_ojTorc(char *xgv_ccu_name, int xgv_ccu_component_id, double xgv_ccu_state_machine_update_rate)
{
	OjCmpt XGV_CCU;		// Variable that will store the component reference

	XGV_CCU = ojCmptCreate(xgv_ccu_name, xgv_ccu_component_id, xgv_ccu_state_machine_update_rate);	// Create the component
	if (XGV_CCU == NULL)
	{
		printf("Error creating %s component. Is ojNodeManager running?\n", xgv_ccu_name);
		return(NULL);
	}

	ojCmptAddService(XGV_CCU, xgv_ccu_component_id);				// Add XGV_CCU service type
	add_xgv_ccu_component_service_messages(XGV_CCU, xgv_ccu_component_id);

	ojCmptSetStateCallback(XGV_CCU, JAUS_READY_STATE, xgv_ccu_state_machine);	// Set ready state callback
	ojCmptSetState(XGV_CCU, JAUS_READY_STATE);					// Set the current state to ready
	return (XGV_CCU);
}


void
shutdown_module(int a __attribute__ ((unused)))
{
	endwin();
	tcsetattr(0, TCSANOW, &storedTermio);

	exit(0);
}


int 
main(void)
{
	OjCmpt XGV_CCU;
	int *xgv_ccu_service_connections;

	signal(SIGINT, shutdown_module);

	XGV_CCU = create_xgv_ccu_component_ojTorc(XGV_CCU_NAME, XGV_CCU_COMPONENTE_ID, XGV_CCU_STATE_MACHINE_UPDATE_RATE);
	if (!XGV_CCU)
		exit(1);

	register_xgv_ccu_messages_callbacks(XGV_CCU);
	ojCmptSetAuthority(XGV_CCU, 6);
	ojCmptRun(XGV_CCU);				// Begin running the XGV_CCU state machine
	xgv_ccu_service_connections = create_xgv_ccu_service_connections(XGV_CCU);

	user_interface(XGV_CCU);

	terminate_xgv_ccu_service_connections(XGV_CCU, xgv_ccu_service_connections);
	ojCmptDestroy(XGV_CCU);				// Shutdown and destroy component

	return (0);
}
