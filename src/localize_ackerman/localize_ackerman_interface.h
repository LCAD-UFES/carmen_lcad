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


/** @addtogroup localize liblocalize_interface **/
// @{

/** \file localize_interface.h
 * \brief Definition of the interface of the module localize.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

 
#ifndef CARMEN_LOCALIZE_ACKERMAN_INTERFACE_H
#define CARMEN_LOCALIZE_ACKERMAN_INTERFACE_H

#include "localize_ackerman_messages.h"
#include <carmen/robot_ackerman_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_localize_ackerman_subscribe_globalpos_message(carmen_localize_ackerman_globalpos_message
					    *globalpos,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how);

void
carmen_localize_ackerman_unsubscribe_globalpos_message(carmen_handler_t handler);

void 
carmen_localize_ackerman_subscribe_particle_message(carmen_localize_ackerman_particle_message
					   *particle, carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);

void
carmen_localize_ackerman_unsubscribe_particle_message(carmen_handler_t handler);

void 
carmen_localize_ackerman_subscribe_particle_prediction_message(carmen_localize_ackerman_particle_message
					   *particle, carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);

void
carmen_localize_ackerman_unsubscribe_particle_prediction_message(carmen_handler_t handler);

void 
carmen_localize_ackerman_subscribe_particle_correction_message(carmen_localize_ackerman_particle_message
					   *particle, carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);

void
carmen_localize_ackerman_unsubscribe_particle_correction_message(carmen_handler_t handler);

void 
carmen_localize_ackerman_subscribe_initialize_message(carmen_localize_ackerman_initialize_message *init_msg,
					     carmen_handler_t handler, 
					     carmen_subscribe_t subscribe_how);

void
carmen_localize_ackerman_unsubscribe_initialize_message(carmen_handler_t handler);

void
carmen_localize_ackerman_subscribe_initialize_placename_message(carmen_localize_ackerman_initialize_placename_message
						       *initialize,
						       carmen_handler_t handler,
						       carmen_subscribe_t subscribe_how);

void
carmen_localize_ackerman_unsubscribe_initialize_placename_message(carmen_handler_t handler);

void 
carmen_localize_ackerman_subscribe_sensor_message(carmen_localize_ackerman_sensor_message
					 *sensor_msg, carmen_handler_t handler,
					 carmen_subscribe_t subscribe_how);

void
carmen_localize_ackerman_unsubscribe_sensor_message(carmen_handler_t handler);

void 
carmen_localize_ackerman_initialize_gaussian_command(carmen_point_t mean,
					    carmen_point_t std, double beta);

void
carmen_localize_ackerman_initialize_gaussian_time_command(carmen_point_t mean,
					    carmen_point_t std, double timestamp);

void carmen_localize_ackerman_initialize_uniform_command(void);

void carmen_localize_ackerman_initialize_placename_command(char *placename);

void
carmen_localize_ackerman_correct_laser(carmen_robot_ackerman_laser_message *laser,
			      carmen_localize_ackerman_globalpos_message *globalpos);

int carmen_localize_ackerman_get_map(int global, carmen_map_t *map);

int carmen_localize_ackerman_get_globalpos(carmen_localize_ackerman_globalpos_message **globalpos);

void
carmen_localize_ackerman_define_globalpos_messages();

void
carmen_localize_ackerman_publish_globalpos_message(carmen_localize_ackerman_globalpos_message *message);

void
carmen_localize_ackerman_publish_particles_message(carmen_localize_ackerman_particle_message *message);

void
carmen_localize_ackerman_publish_sensor_message(carmen_localize_ackerman_sensor_message *message);

#ifdef __cplusplus
}
#endif

#endif
// @}
