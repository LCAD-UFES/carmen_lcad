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

/** @addtogroup simulator libsimulator_interface **/
// @{

/** \file simulator_interface.h
 * \brief Definition of the interface of the module simulator.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef FORD_ESCAPE_HYBRID_INTERFACE_H
#define FORD_ESCAPE_HYBRID_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ford_escape_hybrid_messages.h"

void
carmen_ford_escape_subscribe_status_message(carmen_ford_escape_status_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void
carmen_ford_escape_unsubscribe_status_message(carmen_handler_t handler);

void
carmen_ford_escape_publish_status_message(carmen_ford_escape_status_message *msg, double timestamp);

void
carmen_ford_escape_publish_signals_message(carmen_ford_escape_signals_message *msg, double timestamp);

void
carmen_ford_escape_subscribe_tune_pid_gain_parameters_message(tune_pid_gain_parameters_message *message,  carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_ford_escape_unsubscribe_tune_pid_gain_parameters_message(carmen_handler_t handler);

void
carmen_ford_escape_publish_tune_pid_gain_parameters_message(tune_pid_gain_parameters_message *msg, double timestamp);

/*
void
carmen_ford_escape_hybrid_steering_PID_controler(double *steering_command, 
	double atan_desired_curvature, double atan_current_curvature, double delta_t);

void
carmen_ford_escape_hybrid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command, 
	double desired_velocity, double current_velocity, double delta_t);

void 
carmen_ford_escape_hybrid_read_pid_parameters(int argc, char *argv[]);
*/

#ifdef __cplusplus
}
#endif

#endif
// @}
