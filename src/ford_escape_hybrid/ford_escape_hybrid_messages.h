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

/** @addtogroup simulator **/
// @{

/** \file simulator_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef FORD_ESCAPE_HYBRID_MESSAGES_H
#define FORD_ESCAPE_HYBRID_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define	XGV_MANUAL_OVERRIDE_FLAG	0x10000

typedef struct {
  double g_XGV_throttle;
  double g_XGV_steering;
  double g_XGV_brakes;

  unsigned int g_XGV_component_status;

  int g_XGV_main_propulsion;
  int g_XGV_main_fuel_supply;
  int g_XGV_parking_brake;
  int g_XGV_gear;

  int g_XGV_turn_signal;
  int g_XGV_horn_status;
  int g_XGV_headlights_status;

  double timestamp;
  char *host;
} carmen_ford_escape_status_message;

#define CARMEN_FORD_ESCAPE_STATUS_NAME "carmen_ford_escape_status"
#define CARMEN_FORD_ESCAPE_STATUS_FMT  "{double,double,double,int,int,int,int,int,int,int,int,double,string}"


typedef struct {
  int num_errors;
  int *error;
  double timestamp;
  char *host;
} carmen_ford_escape_error_message;

#define CARMEN_FORD_ESCAPE_ERROR_NAME "carmen_ford_escape_error"
#define CARMEN_FORD_ESCAPE_ERROR_FMT  "{int,<{int}:1>,double,string}"

typedef enum {
	SIGNAL_OFF, SIGNAL_LEFT, SIGNAL_RIGHT, SIGNAL_FLASHERS
} carmen_base_ackerman_turn_signal_status;
//high beams farol alto
//low beams

typedef enum {
	HEADLIGHT_OFF, HEADLIGHT_PARKING_LIGHTS, HEADLIGHT_ON
} carmen_ford_escape_headlights_status;

//IN MESSAGE
typedef struct {
	carmen_base_ackerman_turn_signal_status turn_signal;
	int horn_status; // 0 or 1
	carmen_ford_escape_headlights_status headlight_status;
	int high_beams;//if the headlight status is on, determines if the vehicle is using high beams or low beams
	int fog_lights;//if the headlight status is on, determines if fog lights is on
	double timestamp;
	char *host;
} carmen_ford_escape_signals_message;

#define CARMEN_FORD_ESCAPE_SIGNAL_NAME	"carmen_ford_escape_signals"
#define CARMEN_FORD_ESCAPE_SIGNAL_FMT		"{int, int, int, int, int, double, string}"

typedef struct {
  double kp, ki, kd;
  double timestamp;
	char *host;
} tune_pid_gain_parameters_message;

#define TUNE_PID_GAIN_PARAMENTERS_NAME "tune_pid_gain_parameters"
#define TUNE_PID_GAIN_PARAMENTERS_FMT "{double, double, double, double, string}"


#ifdef __cplusplus
}
#endif

#endif


// @}
