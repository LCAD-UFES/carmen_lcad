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

#include <carmen/carmen.h>
#include "localize_ackerman_messages.h"
#include <carmen/robot_ackerman_messages.h>


static const unsigned int timeout = 5000;


void
carmen_localize_ackerman_subscribe_globalpos_message(carmen_localize_ackerman_globalpos_message 
		*globalpos,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME,
			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT,
			globalpos, sizeof(carmen_localize_ackerman_globalpos_message),
			handler, subscribe_how);
}


void
carmen_localize_ackerman_unsubscribe_globalpos_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, handler);
}


void
carmen_localize_ackerman_subscribe_particle_message(carmen_localize_ackerman_particle_message 
		*particle,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME,
			CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT,
			particle, sizeof(carmen_localize_ackerman_particle_message),
			handler, subscribe_how);
}


void
carmen_localize_ackerman_unsubscribe_particle_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, handler);
}


void 
carmen_localize_ackerman_subscribe_particle_prediction_message(carmen_localize_ackerman_particle_message
					   *particle, carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME,
			CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT,
			particle, sizeof(carmen_localize_ackerman_particle_message),
			handler, subscribe_how);
}


void
carmen_localize_ackerman_unsubscribe_particle_prediction_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME, handler);
}


void 
carmen_localize_ackerman_subscribe_particle_correction_message(carmen_localize_ackerman_particle_message
					   *particle, carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME,
			CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT,
			particle, sizeof(carmen_localize_ackerman_particle_message),
			handler, subscribe_how);
}


void
carmen_localize_ackerman_unsubscribe_particle_correction_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME, handler);
}


void
carmen_localize_ackerman_subscribe_sensor_message(carmen_localize_ackerman_sensor_message 
		*sensor,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME,
			CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT,
			sensor, sizeof(carmen_localize_ackerman_sensor_message),
			handler, subscribe_how);
}


void
carmen_localize_ackerman_unsubscribe_sensor_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, handler);
}


void
carmen_localize_ackerman_subscribe_initialize_message(carmen_localize_ackerman_initialize_message 
		*initialize,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME,
			CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT,
			initialize,
			sizeof(carmen_localize_ackerman_initialize_message),
			handler, subscribe_how);
}


void
carmen_localize_ackerman_unsubscribe_initialize_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, handler);
}


void
carmen_localize_ackerman_subscribe_initialize_placename_message(carmen_localize_ackerman_initialize_placename_message 
		*initialize,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_NAME,
			CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_FMT,
			initialize,
			sizeof(carmen_localize_ackerman_initialize_placename_message),
			handler, subscribe_how);
}


void
carmen_localize_ackerman_unsubscribe_initialize_placename_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_NAME, handler);
}


void 
carmen_localize_ackerman_initialize_gaussian_command(carmen_point_t mean, carmen_point_t std)
{
	static carmen_localize_ackerman_initialize_message init;
	static int first = 1;
	IPC_RETURN_TYPE err;

	if (first)
	{
		err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
		first = 0;
	}

	init.timestamp = carmen_get_time();
	init.host = carmen_get_host();
	init.distribution = CARMEN_INITIALIZE_GAUSSIAN;
	init.num_modes = 1;
	init.mean = &mean;
	init.std = &std;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, &init);
	carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
}


void
carmen_localize_ackerman_initialize_gaussian_time_command(carmen_point_t mean, carmen_point_t std, double timestamp)
{
	static carmen_localize_ackerman_initialize_message init;
	static int first = 1;
	IPC_RETURN_TYPE err;

	if (first)
	{
		err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
		first = 0;
	}

	init.timestamp = timestamp;
	init.host = carmen_get_host();
	init.distribution = CARMEN_INITIALIZE_GAUSSIAN;
	init.num_modes = 1;
	init.mean = &mean;
	init.std = &std;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, &init);
	carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
}


void
carmen_localize_ackerman_initialize_uniform_command(void)
{
	static carmen_localize_ackerman_initialize_message init;
	static int first = 1;
	IPC_RETURN_TYPE err;

	if (first)
	{
		err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
		first = 0;
	}

	init.timestamp = carmen_get_time();
	init.host = carmen_get_host();
	init.distribution = CARMEN_INITIALIZE_UNIFORM;
	init.num_modes = 0;
	init.mean = NULL;
	init.std = NULL;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, &init);
	carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);
}


void
carmen_localize_ackerman_initialize_placename_command(char *placename)
{
	static carmen_localize_ackerman_initialize_placename_message init;
	static int first = 1;
	IPC_RETURN_TYPE err;

	if(first)
	{
		err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_NAME);
		first = 0;
	}

	init.timestamp = carmen_get_time();
	init.host = carmen_get_host();
	init.placename = placename;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_NAME, &init);
	carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_NAME);
}


void
carmen_localize_ackerman_correct_laser(carmen_robot_ackerman_laser_message *laser,
		carmen_localize_ackerman_globalpos_message *globalpos)
{
	int backwards;
	double dr1, dt, dr2;
	double dx, dy;
	double dtheta;

	dx = laser->laser_pose.x - globalpos->odometrypos.x;
	dy = laser->laser_pose.y - globalpos->odometrypos.y;
	dtheta = laser->laser_pose.theta - globalpos->odometrypos.theta;

	dt = sqrt(dx * dx + dy * dy);
	backwards = (dx * cos(laser->laser_pose.theta) + dy * sin(laser->laser_pose.theta) < 0);

	/* The dr1/dr2 code becomes unstable if dt is too small. */
	if (dt < 0.05)
	{
		dr1 = carmen_normalize_theta(laser->laser_pose.theta - globalpos->odometrypos.theta) / 2.0;
		dr2 = dr1;
	}
	else
	{
		if (backwards)
			dr1 = carmen_normalize_theta(atan2(-dy, -dx) - globalpos->odometrypos.theta);
		else
			dr1 = carmen_normalize_theta(atan2(dy, dx) - globalpos->odometrypos.theta);

		dr2 = carmen_normalize_theta(dtheta - dr1);
	}

	if (backwards)
		dt = -dt;

	laser->laser_pose.x = globalpos->globalpos.x + dt * cos(globalpos->globalpos.theta + dr1);
	laser->laser_pose.y = globalpos->globalpos.y + dt * sin(globalpos->globalpos.theta + dr1);
	laser->laser_pose.theta = carmen_normalize_theta(globalpos->globalpos.theta + dr1 + dr2);
}


int
carmen_localize_ackerman_get_map(int global, carmen_map_t *map)
{
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_map_query_message msg;
	carmen_localize_ackerman_map_message *response = NULL;
	int index;

#ifndef NO_ZLIB
	//int uncompress_return;
	int uncompress_size;
	unsigned long uncompress_size_result;
	unsigned char *uncompressed_data;
#endif

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);
		initialized = 1;
	}

	msg.map_is_global_likelihood = global;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_queryResponseData(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, &msg, (void **)&response, timeout);
	carmen_test_ipc(err, "Could not get map", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);

#ifndef NO_ZLIB
	if (response && response->compressed)
	{
		uncompress_size = response->config.x_size*response->config.y_size;
		uncompressed_data = (unsigned char *)calloc(uncompress_size, sizeof(double));
		carmen_test_alloc(uncompressed_data);
		uncompress_size_result = uncompress_size*sizeof(double);
		/*uncompress_return =*/ carmen_uncompress(
				(unsigned char *)uncompressed_data,
				(unsigned long *)&uncompress_size_result,
				(unsigned char *)response->data,
				(unsigned long)response->size);
		response->data = uncompressed_data;
		response->size = uncompress_size_result;
	}
#else
	if (response && response->compressed)
	{
		carmen_warn("Received compressed map from server. This program was\n"
				"compiled without zlib support, so this map cannot be\n"
				"used. Sorry.\n");
		free(response->data);
		free(response);
		response = NULL;
	}
#endif

	if (response)
	{
		if (map)
		{
			map->config = response->config;
			map->complete_map = (double *)response->data;
			map->map = (double **)calloc(map->config.x_size, sizeof(double *));
			carmen_test_alloc(map->map);

			for (index = 0; index < map->config.x_size; index++)
				map->map[index] = map->complete_map+index*map->config.y_size;
		}
		else
			free(response->data);

		free(response);
	}

	return 0;
}


int
carmen_localize_ackerman_get_globalpos(carmen_localize_ackerman_globalpos_message **globalpos)
{
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_globalpos_query_message *msg;
	static int initialized = 0;

	if(!initialized)
	{
		err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_DEFAULT_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME);
		initialized = 1;
	}

	msg = carmen_default_message_create();
	err = IPC_queryResponseData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, msg, (void **)globalpos, timeout);
	carmen_test_ipc_return_int(err, "Could not query localize globalpos", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME);

	return 0;
}


void
carmen_localize_ackerman_define_globalpos_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


void
carmen_localize_ackerman_publish_globalpos_message(carmen_localize_ackerman_globalpos_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


void
carmen_localize_ackerman_publish_particles_message(carmen_localize_ackerman_particle_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);
}


void
carmen_localize_ackerman_publish_sensor_message(carmen_localize_ackerman_sensor_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);
}


