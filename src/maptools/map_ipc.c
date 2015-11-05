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
#include "map_io.h"
#ifndef NO_ZLIB
#include <zlib.h>
#endif

/*****************************************
 * ipc library for the map server        *
 *****************************************/

static char *filename = NULL;
static char *map_zone_name = NULL;

static void
hmap_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_hmap_request_message req;
	carmen_hmap_message hmap_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &req,
			sizeof(carmen_default_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	if (carmen_map_read_hmap_chunk(filename, &hmap_msg.hmap) < 0)
		memset(&hmap_msg, 0, sizeof(carmen_hmap_message));

	hmap_msg.timestamp = carmen_get_time();
	hmap_msg.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_MAP_HMAP_NAME, &hmap_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_HMAP_NAME);

	carmen_map_free_hmap(&hmap_msg.hmap);
}

static void
publish_map_zone(void)
{
	carmen_map_zone_message msg;
	IPC_RETURN_TYPE err;

	msg.zone_name = carmen_new_string(map_zone_name);

	err = IPC_publishData(CARMEN_MAP_ZONE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_MAP_ZONE_NAME);

	if (msg.zone_name)
		free(msg.zone_name);
}

static void
change_map_zone_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_map_change_map_zone_request request;
	carmen_map_change_map_zone_response response;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;
	carmen_hmap_t hmap;
	int i;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &request,
			sizeof(carmen_map_change_map_zone_request));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	if (carmen_map_read_hmap_chunk(filename, &hmap) < 0)
		response.err_msg = carmen_new_string("can't read hmap chunk");
	else {
		for (i = 0; i < hmap.num_zones; i++)
			if (!strcmp(hmap.zone_names[i], request.zone_name))
				break;
		if (i == hmap.num_zones)
			response.err_msg = carmen_new_string("can't find zone - %s", request.zone_name);
		else {
			response.err_msg = NULL;
			map_zone_name = carmen_new_string(request.zone_name);
			publish_map_zone();
		}
	}

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_CHANGE_MAP_ZONE_RESPONSE_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_CHANGE_MAP_ZONE_RESPONSE_NAME);

	if (response.err_msg)
		free(response.err_msg);

	carmen_map_publish_update();
}

/* handler for map requests.
   loads a map from a file and sends it
   back to the client */

#ifndef NO_ZLIB

static void
compress_map_msg(carmen_grid_map_message *map_msg, carmen_map_t *map)
{
	unsigned long compress_buf_size;
	int compress_return = -1;

	compress_buf_size = map->config.x_size*map->config.y_size*1.01+12;
	compress_buf_size *= sizeof(double);
	map_msg->map = (unsigned char *)
    						calloc(compress_buf_size, sizeof(unsigned char));
	carmen_test_alloc(map_msg->map);
	compress_return = carmen_compress(
			(unsigned char *)map_msg->map,
			(unsigned long *)&compress_buf_size,
			(unsigned char *)map->complete_map,
			(unsigned long)map->config.x_size * map->config.y_size*sizeof(double),
			Z_DEFAULT_COMPRESSION);
	if (compress_return != Z_OK)
	{
		free(map_msg->map);
		map_msg->map = (unsigned char *)map->complete_map;
		map_msg->size = map->config.x_size * map->config.y_size*sizeof(double);
		map_msg->compressed = 0;
	}
	else
	{
		map_msg->size = compress_buf_size;
		map_msg->compressed = 1;
		free(map->complete_map);
	}
}
#endif


static void
get_map_origin(char *filename, carmen_map_config_t *config)
{
	char map_name[1000], *div_char, *aux;

	config->x_origin = config->y_origin = 0;

	if (filename == NULL)
		return;

	div_char = strrchr(filename, '/');

	if (div_char == NULL)
	{
		return;
	}

	div_char++;

	strcpy(map_name, div_char);

	div_char = strrchr(map_name, '_');

	if (div_char != NULL && (map_name[0] == 'm' || map_name[0] == 'g'))
	{
		aux = strrchr(map_name, '.');

		*aux = '\0';
		*div_char = '\0';
		div_char++;

		if((isdigit(*(map_name + 1)) || *(map_name + 1) == '-') && (isdigit(*div_char) || *div_char == '-'))
		{
			config->x_origin = atoi(map_name + 1);
			config->y_origin = atoi(div_char);
		}
	}
}


static void
assemble_named_map_msg(char *name, carmen_grid_map_message *map_msg)
{
	carmen_map_t map;
	int ret_val;

	if (name)
		ret_val = carmen_map_read_named_gridmap_chunk(filename, name, &map);
	else
		ret_val = carmen_map_read_gridmap_chunk(filename, &map);

	get_map_origin(filename, &map.config);

	if(ret_val < 0)
	{
		memset(map_msg, 0, sizeof(carmen_grid_map_message));
		map_msg->err_mesg = (char *)calloc(17, sizeof(char));
		carmen_test_alloc(map_msg->err_mesg);
		strcpy(map_msg->err_mesg, "Unknown error!");

		map_msg->timestamp = carmen_get_time();
		map_msg->host = carmen_get_host();

		return;
	}

	free(map.map);

	//#ifdef NO_ZLIB_MAP
#ifndef NO_ZLIB
	compress_map_msg(map_msg, &map);
	//#endif
#else
	map_msg->map = (unsigned char *)map.complete_map;
	map_msg->size = map.config.x_size * map.config.y_size*sizeof(double);
	map_msg->compressed = 0;
#endif

	map_msg->config = map.config;
	map_msg->err_mesg = (char *)calloc(1, sizeof(char));
	carmen_test_alloc(map_msg->err_mesg);
	map_msg->err_mesg[0] = '\0';

	map_msg->timestamp = carmen_get_time();
	map_msg->host = carmen_get_host();
}

static void
assemble_map_msg(carmen_grid_map_message *map_msg)
{
	assemble_named_map_msg(map_zone_name, map_msg);
}

static void
gridmap_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_gridmap_request_message req;
	carmen_grid_map_message map_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &req,
			sizeof(carmen_default_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	assemble_map_msg(&map_msg);

	err = IPC_respondData(msgRef, CARMEN_MAP_GRIDMAP_NAME, &map_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_GRIDMAP_NAME);

	free(map_msg.map);
	free(map_msg.err_mesg);
}

static void
named_gridmap_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_named_gridmap_request req;
	carmen_grid_map_message map_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &req,
			sizeof(carmen_named_gridmap_request));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	assemble_named_map_msg(req.name, &map_msg);

	err = IPC_respondData(msgRef, CARMEN_MAP_GRIDMAP_NAME, &map_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_GRIDMAP_NAME);

	free(map_msg.map);
	free(map_msg.err_mesg);
}


static void
placelist_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_placelist_request_message req;
	carmen_map_placelist_t placelist;
	carmen_map_placelist_message placelist_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &req,
			sizeof(carmen_default_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	if (map_zone_name) {
		if (carmen_map_read_named_places_chunk(filename, map_zone_name, &placelist) == 0) {
			placelist_msg.places = placelist.places;
			placelist_msg.num_places = placelist.num_places;
		}
		else {
			placelist_msg.places = NULL;
			placelist_msg.num_places = 0;
			placelist.num_places = 0;
		}
	}
	else {
		if (carmen_map_read_places_chunk(filename, &placelist) == 0) {
			placelist_msg.places = placelist.places;
			placelist_msg.num_places = placelist.num_places;
		}
		else {
			placelist_msg.places = NULL;
			placelist_msg.num_places = 0;
			placelist.num_places = 0;
		}
	}

	placelist_msg.timestamp = carmen_get_time();
	placelist_msg.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_MAP_PLACELIST_NAME, &placelist_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_PLACELIST_NAME);

	if (placelist.num_places > 0)
		free(placelist.places);
}

static void
named_placelist_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_map_named_placelist_request req;
	carmen_map_placelist_t placelist;
	carmen_map_placelist_message placelist_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &req,
			sizeof(carmen_map_named_placelist_request));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	if (carmen_map_read_named_places_chunk(filename, req.name, &placelist) == 0)
	{
		placelist_msg.places = placelist.places;
		placelist_msg.num_places = placelist.num_places;
	}
	else
	{
		placelist_msg.places = NULL;
		placelist_msg.num_places = 0;
		placelist.num_places = 0;
	}

	placelist_msg.timestamp = carmen_get_time();
	placelist_msg.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_MAP_PLACELIST_NAME, &placelist_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_PLACELIST_NAME);

	if (placelist.num_places > 0)
		free(placelist.places);
}

static void
offlimits_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_offlimits_request_message query;
	carmen_offlimits_p offlimits;
	int list_length;
	carmen_map_offlimits_message offlimits_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query,
			sizeof(carmen_default_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	if(carmen_map_read_offlimits_chunk(filename, &offlimits, &list_length) < 0)
	{
		offlimits_msg.offlimits_list = NULL;
		offlimits_msg.list_length = 0;
	} else {
		offlimits_msg.offlimits_list = offlimits;
		offlimits_msg.list_length = list_length;
	}

	offlimits_msg.timestamp = carmen_get_time();
	offlimits_msg.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_MAP_OFFLIMITS_NAME, &offlimits_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_OFFLIMITS_NAME);

	if (list_length > 0)
		free(offlimits);
}

static void
named_offlimits_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_map_named_offlimits_request query;
	carmen_offlimits_p offlimits;
	int list_length;
	carmen_map_offlimits_message offlimits_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query,
			sizeof(carmen_map_named_offlimits_request));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	if(carmen_map_read_named_offlimits_chunk(filename, query.name, &offlimits, &list_length) < 0)
	{
		offlimits_msg.offlimits_list = NULL;
		offlimits_msg.list_length = 0;
	}
	else {
		offlimits_msg.offlimits_list = offlimits;
		offlimits_msg.list_length = list_length;
	}

	offlimits_msg.timestamp = carmen_get_time();
	offlimits_msg.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_MAP_OFFLIMITS_NAME, &offlimits_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_OFFLIMITS_NAME);

	if (list_length > 0)
		free(offlimits);
}

static void
global_offset_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_global_offset_request_message query;
	carmen_map_global_offset_message global_offset_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query,
			sizeof(carmen_default_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	carmen_map_read_global_offset_chunk(filename,
			&global_offset_msg.global_offset);

	global_offset_msg.timestamp = carmen_get_time();
	global_offset_msg.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_MAP_GLOBAL_OFFSET_NAME,
			&global_offset_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_GLOBAL_OFFSET_NAME);
}

static void
named_global_offset_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_map_named_global_offset_request query;
	carmen_map_global_offset_message global_offset_msg;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query,
			sizeof(carmen_map_named_global_offset_request));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data",
			IPC_msgInstanceName(msgRef));

	carmen_map_read_named_global_offset_chunk(filename, query.name,
			&global_offset_msg.global_offset);

	global_offset_msg.timestamp = carmen_get_time();
	global_offset_msg.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_MAP_GLOBAL_OFFSET_NAME,
			&global_offset_msg);
	carmen_test_ipc(err, "Could not respond", CARMEN_MAP_GLOBAL_OFFSET_NAME);
}

/* set up the ipc connection with central
   subscribe to map requests
   define map messages */
int
carmen_map_initialize_ipc(void)
{
	IPC_RETURN_TYPE err;

	/* define messages created by this module */
	err = IPC_defineMsg(CARMEN_MAP_HMAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_HMAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_HMAP_NAME);

	err = IPC_defineMsg(CARMEN_HMAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_HMAP_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_MAP_ZONE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_ZONE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_ZONE_NAME);

	err = IPC_defineMsg(CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_CHANGE_MAP_ZONE_REQUEST_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_CHANGE_MAP_ZONE_RESPONSE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_CHANGE_MAP_ZONE_RESPONSE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_CHANGE_MAP_ZONE_RESPONSE_NAME);

	err = IPC_defineMsg(CARMEN_MAP_GRIDMAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_GRIDMAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_GRIDMAP_NAME);

	err = IPC_defineMsg(CARMEN_MAP_GRIDMAP_UPDATE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_GRIDMAP_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_MAP_GRIDMAP_UPDATE_NAME);

	err = IPC_defineMsg(CARMEN_GRIDMAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GRIDMAP_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_NAMED_GRIDMAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAMED_GRIDMAP_REQUEST_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAMED_GRIDMAP_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_PLACELIST_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_PLACELIST_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_NAMED_PLACELIST_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAMED_PLACELIST_REQUEST_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAMED_PLACELIST_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_MAP_PLACELIST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_PLACELIST_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_PLACELIST_NAME);

	err = IPC_defineMsg(CARMEN_OFFLIMITS_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_OFFLIMITS_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_NAMED_OFFLIMITS_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAMED_OFFLIMITS_REQUEST_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAMED_OFFLIMITS_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_MAP_OFFLIMITS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_OFFLIMITS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_OFFLIMITS_NAME);

	err = IPC_defineMsg(CARMEN_GLOBAL_OFFSET_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GLOBAL_OFFSET_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_MAP_GLOBAL_OFFSET_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_GLOBAL_OFFSET_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_GLOBAL_OFFSET_NAME);

	/* setup incoming message handlers */
	err = IPC_subscribe(CARMEN_HMAP_REQUEST_NAME, hmap_request_handler,
			NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_HMAP_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_HMAP_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME, change_map_zone_request_handler,
			NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_GRIDMAP_REQUEST_NAME, gridmap_request_handler,
			NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_GRIDMAP_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_GRIDMAP_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_NAMED_GRIDMAP_REQUEST_NAME, named_gridmap_request_handler,
			NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_NAMED_GRIDMAP_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_NAMED_GRIDMAP_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_PLACELIST_REQUEST_NAME,
			placelist_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_PLACELIST_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_PLACELIST_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_NAMED_PLACELIST_REQUEST_NAME,
			named_placelist_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_NAMED_PLACELIST_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_NAMED_PLACELIST_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_OFFLIMITS_REQUEST_NAME,
			offlimits_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_OFFLIMITS_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_OFFLIMITS_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_NAMED_OFFLIMITS_REQUEST_NAME,
			named_offlimits_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_NAMED_OFFLIMITS_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_NAMED_OFFLIMITS_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_GLOBAL_OFFSET_REQUEST_NAME,
			global_offset_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_GLOBAL_OFFSET_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_GLOBAL_OFFSET_REQUEST_NAME, 100);

	err = IPC_subscribe(CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME,
			named_global_offset_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME);
	IPC_setMsgQueueLength(CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME, 100);

	return 0;
}

void
carmen_map_publish_update(void)
{
	carmen_grid_map_message map_msg;
	IPC_RETURN_TYPE err;

	assemble_map_msg(&map_msg);

	err = IPC_publishData(CARMEN_MAP_GRIDMAP_UPDATE_NAME, &map_msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_MAP_GRIDMAP_UPDATE_NAME);

	free(map_msg.map);
	free(map_msg.err_mesg);
}

void
carmen_map_set_filename(char *new_filename)
{
	if (filename != NULL)
		free(filename);

	map_zone_name = NULL;

	filename = (char *)calloc(strlen(new_filename)+1, sizeof(char));
	carmen_test_alloc(filename);

	strcpy(filename, new_filename);
	carmen_map_publish_update();
}

