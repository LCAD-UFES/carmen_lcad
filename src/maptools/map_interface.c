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

/**********************************************
 * library of function for mapserver clients  *
 **********************************************/
#include <carmen/carmen.h>
#ifndef NO_ZLIB
#include <zlib.h>
#endif

carmen_map_t **map_update;
carmen_handler_t *map_update_handler_external;
char ***zone_update;
carmen_handler_t *zone_update_handler_external;

static carmen_map_p superimposed_map = NULL;

/* subscribe to incoming gridmap messages */

static int context_array_size = 0;
static IPC_CONTEXT_PTR *context_array = NULL;

static int
get_context_id(void)
{
	int index = 0;
	IPC_CONTEXT_PTR current_context = IPC_getContext();

	if (context_array == NULL)
		return -1;

	while (context_array[index] != NULL)
	{
		if (context_array[index] == current_context)
			return index;
		index++;
	}

	return -1;

}

static int 
add_context(void)
{
	int index;

	if (context_array == NULL)
	{
		context_array = (IPC_CONTEXT_PTR *)calloc(10, sizeof(IPC_CONTEXT_PTR));
		carmen_test_alloc(context_array);

		map_update = (carmen_map_t **)
			calloc(10, sizeof(carmen_map_t *));
		carmen_test_alloc(map_update);

		map_update_handler_external = (carmen_handler_t *)
			calloc(10, sizeof(carmen_handler_t));
		carmen_test_alloc(map_update_handler_external);

		zone_update = (char ***)
			calloc(10, sizeof(char **));
		carmen_test_alloc(zone_update);

		zone_update_handler_external = (carmen_handler_t *)
			calloc(10, sizeof(carmen_handler_t));
		carmen_test_alloc(zone_update_handler_external);

		context_array_size = 10;
		context_array[0] = IPC_getContext();

		return 0;
	}

	index = 0;
	while (index < context_array_size && context_array[index] != NULL)
		index++;

	if (index == context_array_size)
	{
		context_array_size += 10;
		context_array = (IPC_CONTEXT_PTR *)realloc
				(context_array, context_array_size*sizeof(IPC_CONTEXT_PTR));
		carmen_test_alloc(context_array);
		memset(context_array+index, 0, 10*sizeof(IPC_CONTEXT_PTR));
	}

	context_array[index] = IPC_getContext();
	return index;
}

static void 
map_update_interface_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err = IPC_OK;
	FORMATTER_PTR formatter;
	int context_id;
	carmen_grid_map_message map_msg;
	carmen_map_t *new_map;
	int i;

//#ifdef NO_ZLIB_MAP
#ifndef NO_ZLIB
/* Variaveis usadas na compressao do mapa da interface */
  unsigned long uncompress_size;
#endif
//#endif


	context_id = get_context_id();

	if (context_id < 0)
	{
		carmen_warn("Bug detected: invalid context\n");
		IPC_freeByteArray(callData);
		return;
	}

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &map_msg,
			sizeof(carmen_grid_map_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall",
			IPC_msgInstanceName(msgRef));

	new_map = map_update[context_id];
	if (new_map->map != NULL)
		free(new_map->map);
	if (new_map->complete_map != NULL)
		free(new_map->complete_map);

	new_map->config = map_msg.config;

	new_map->complete_map = (double *)
    		calloc(new_map->config.x_size*new_map->config.y_size, sizeof(double));
	carmen_test_alloc(new_map->complete_map);

//#ifndef NO_ZLIB_MAP
#ifndef NO_ZLIB
	if (map_msg.compressed)
	{
		uncompress_size = new_map->config.x_size*new_map->config.y_size*
				sizeof(double);
		carmen_uncompress(
				(unsigned char *)new_map->complete_map,
				&uncompress_size,
				(unsigned char *)map_msg.map,
				(unsigned long)map_msg.size);
	}
	else
	{
		memcpy(new_map->complete_map, map_msg.map, map_msg.size);
	}
//#endif
#else
	if (map_msg.compressed)
	{
		carmen_warn("Received compressed map from server. This program was\n"
				"compiled without zlib support, so this map cannot be\n"
				"used. Sorry.\n");

		memset(new_map, 0, sizeof(carmen_map_t));

		if (map_msg.map)
			free(map_msg.map);
		if (map_msg.err_mesg)
			free(map_msg.err_mesg);

		return;
	}
	else
	{
		memcpy(new_map->complete_map, map_msg.map, map_msg.size);
	}
#endif 
	new_map->map = (double **)
    		calloc(map_msg.config.x_size, sizeof(double *));
	carmen_test_alloc(new_map->map);
	for(i = 0; i < map_msg.config.x_size; i++)
		new_map->map[i] = new_map->complete_map +
		i*map_msg.config.y_size;

	if (map_msg.map)
		free(map_msg.map);
	if (map_msg.err_mesg)
		free(map_msg.err_mesg);

	if (map_update_handler_external[context_id])
		map_update_handler_external[context_id](map_update[context_id]);
}

void 
carmen_map_subscribe_gridmap_update_message(carmen_map_t *map,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	IPC_RETURN_TYPE err = IPC_OK;
	int context_id;

	err = IPC_defineMsg(CARMEN_MAP_GRIDMAP_UPDATE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_GRIDMAP_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_MAP_GRIDMAP_UPDATE_NAME);

	if (subscribe_how == CARMEN_UNSUBSCRIBE)
	{
		IPC_unsubscribe(CARMEN_MAP_GRIDMAP_UPDATE_NAME,
				map_update_interface_handler);
		return;
	}

	context_id = get_context_id();
	if (context_id < 0)
		context_id = add_context();

	if (map)
		map_update[context_id] = map;
	else if (map_update[context_id] == NULL)
	{
		map_update[context_id] =
				(carmen_map_t *)calloc
				(1, sizeof(carmen_map_t));
		carmen_test_alloc(map_update[context_id]);
	}

	map_update_handler_external[context_id] = handler;

	err = IPC_subscribe(CARMEN_MAP_GRIDMAP_UPDATE_NAME,
			map_update_interface_handler, NULL);
	if (subscribe_how == CARMEN_SUBSCRIBE_LATEST)
		IPC_setMsgQueueLength(CARMEN_MAP_GRIDMAP_UPDATE_NAME, 1);
	else
		IPC_setMsgQueueLength(CARMEN_MAP_GRIDMAP_UPDATE_NAME, 100);

	carmen_test_ipc(err, "Could not subscribe", CARMEN_MAP_GRIDMAP_UPDATE_NAME);
}

static void 
zone_update_interface_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err = IPC_OK;
	FORMATTER_PTR formatter;
	int context_id;
	carmen_map_zone_message msg;
	char **new_zone_ptr;

	context_id = get_context_id();

	if (context_id < 0)
	{
		carmen_warn("Bug detected: invalid context\n");
		IPC_freeByteArray(callData);
		return;
	}

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_map_zone_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall",
			IPC_msgInstanceName(msgRef));

	new_zone_ptr = zone_update[context_id];
	if (*new_zone_ptr != NULL)
		free(*new_zone_ptr);

	*new_zone_ptr = carmen_new_string(msg.zone_name);
	zone_update_handler_external[context_id](zone_update[context_id]);
}

void
carmen_map_subscribe_map_zone_message(char **zone_name,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	IPC_RETURN_TYPE err = IPC_OK;
	int context_id;

	err = IPC_defineMsg(CARMEN_MAP_ZONE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_ZONE_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_MAP_ZONE_NAME);

	if (subscribe_how == CARMEN_UNSUBSCRIBE)
	{
		IPC_unsubscribe(CARMEN_MAP_ZONE_NAME,
				map_update_interface_handler);
		return;
	}

	context_id = get_context_id();
	if (context_id < 0)
		context_id = add_context();

	if (zone_name)
		zone_update[context_id] = zone_name;
	else if (zone_update[context_id] == NULL)
	{
		zone_update[context_id] = (char **) calloc(1, sizeof(char *));
		carmen_test_alloc(zone_update[context_id]);
	}

	zone_update_handler_external[context_id] = handler;

	err = IPC_subscribe(CARMEN_MAP_ZONE_NAME,
			zone_update_interface_handler, NULL);
	if (subscribe_how == CARMEN_SUBSCRIBE_LATEST)
		IPC_setMsgQueueLength(CARMEN_MAP_ZONE_NAME, 1);
	else
		IPC_setMsgQueueLength(CARMEN_MAP_ZONE_NAME, 100);

	carmen_test_ipc(err, "Could not subscribe", CARMEN_MAP_ZONE_NAME);
}

/* request hmap (hierarchical map) from server */
int
carmen_map_get_hmap(carmen_hmap_p hmap)
{
	IPC_RETURN_TYPE err;
	static carmen_hmap_request_message *query;
	static carmen_hmap_message *response;
	unsigned int timeout = 10000;
	int i;

	err = IPC_defineMsg(CARMEN_HMAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_HMAP_REQUEST_NAME);

	query = carmen_default_message_create();
	err = IPC_queryResponseData(CARMEN_HMAP_REQUEST_NAME, query,
			(void **)&response, timeout);
	if (err != IPC_OK)
		return -1;

	if (hmap) {
		hmap->num_zones = response->hmap.num_zones;
		hmap->zone_names = (char **) calloc(hmap->num_zones, sizeof(char *));
		carmen_test_alloc(hmap->zone_names);
		for (i = 0; i < hmap->num_zones; i++) {
			hmap->zone_names[i] = (char *) calloc(strlen(response->hmap.zone_names[i]) + 1, sizeof(char));
			carmen_test_alloc(hmap->zone_names[i]);
			strcpy(hmap->zone_names[i], response->hmap.zone_names[i]);
		}
		hmap->num_links = response->hmap.num_links;
		hmap->links = (carmen_hmap_link_p) calloc(hmap->num_links, sizeof(carmen_hmap_link_t));
		carmen_test_alloc(hmap->links);
		for (i = 0; i < hmap->num_links; i++) {
			hmap->links[i].type = response->hmap.links[i].type;
			hmap->links[i].degree = response->hmap.links[i].degree;
			hmap->links[i].keys = (int *) calloc(hmap->links[i].degree, sizeof(int));
			carmen_test_alloc(hmap->links[i].keys);
			memcpy(hmap->links[i].keys, response->hmap.links[i].keys, hmap->links[i].degree * sizeof(int));
			hmap->links[i].num_points = response->hmap.links[i].num_points;
			hmap->links[i].points = (carmen_point_p) calloc(hmap->links[i].num_points, sizeof(carmen_point_t));
			carmen_test_alloc(hmap->links[i].points);
			memcpy(hmap->links[i].points, response->hmap.links[i].points,
					hmap->links[i].num_points * sizeof(carmen_point_t));
		}
	}

	carmen_map_free_hmap(&response->hmap);

	return 0;
}

void
carmen_map_free_hmap(carmen_hmap_p hmap)
{
	int i;

	if (hmap->zone_names) {
		for (i = 0; i < hmap->num_zones; i++)
			free(hmap->zone_names[i]);
		free(hmap->zone_names);
	}
	if (hmap->links) {
		for (i = 0; i < hmap->num_links; i++) {
			free(hmap->links[i].keys);
			free(hmap->links[i].points);
		}
		free(hmap->links);
	}
}

int carmen_map_change_map_zone(char *zone_name)
{
	IPC_RETURN_TYPE err;
	static carmen_map_change_map_zone_request request;
	static carmen_map_change_map_zone_response *response;
	unsigned int timeout = 10000;

	err = IPC_defineMsg(CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_CHANGE_MAP_ZONE_REQUEST_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME);

	request.zone_name = calloc(strlen(zone_name) + 1, sizeof(char));
	carmen_test_alloc(request.zone_name);
	strcpy(request.zone_name, zone_name);
	request.host = carmen_get_host();
	request.timestamp = carmen_get_time();
	err = IPC_queryResponseData(CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME, &request,
			(void **)&response, timeout);
	carmen_test_ipc_return_int(err, "Could not get map_request",
			CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME);

	if (response->err_msg) {
		fprintf(stderr, "Can't change map zone: %s\n", response->err_msg);
		free(response);
		return -1;
	}

	free(response);
	return 0;
}

/* send a request for a gridmap 
int
carmen_map_get_gridmap_by_name(char *name, carmen_map_p client_map)
{
	IPC_RETURN_TYPE err;
	static carmen_gridmap_request_message *query;
	static carmen_named_gridmap_request named_query;
	static carmen_grid_map_message *response;
	unsigned int timeout = 10000;
	int i;
#ifndef NO_ZLIB
	unsigned long uncompress_size;
#endif
	if (name) {
		err = IPC_defineMsg(CARMEN_NAMED_GRIDMAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_NAMED_GRIDMAP_REQUEST_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAMED_GRIDMAP_REQUEST_NAME);

		named_query.name = calloc(strlen(name) + 1, sizeof(char));
		carmen_test_alloc(named_query.name);
		strcpy(named_query.name, name);
		named_query.host = carmen_get_host();
		named_query.timestamp = carmen_get_time();
		err = IPC_queryResponseData(CARMEN_NAMED_GRIDMAP_REQUEST_NAME, &named_query,
				(void **)&response, timeout);
	}
	else {
		err = IPC_defineMsg(CARMEN_GRIDMAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_DEFAULT_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_GRIDMAP_REQUEST_NAME);

		query = carmen_default_message_create();
		err = IPC_queryResponseData(CARMEN_GRIDMAP_REQUEST_NAME, query,
				(void **)&response, timeout);
	}

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get map_request",
				CARMEN_GRIDMAP_REQUEST_NAME);
		carmen_warn("\nDid you remember to start the mapserver, or give a map "
				"to the paramServer?\n");
		return -1;
	}

	if (response->size == 0)
	{
		carmen_warn("Error receiving map: %s\n", response->err_mesg);
		return -1;
	}

	if (client_map)
	{
		client_map->config = response->config;
		strcpy(client_map->config.origin, "from_paramdaemon");

		client_map->complete_map = (double *) calloc(client_map->config.x_size*client_map->config.y_size, sizeof(double));
		carmen_test_alloc(client_map->complete_map);

#ifndef NO_ZLIB
		if (response->compressed)
		{
			uncompress_size = client_map->config.x_size * client_map->config.y_size*sizeof(double);

			carmen_uncompress(
					(unsigned char *)client_map->complete_map,
					&uncompress_size,
					(unsigned char *)response->map,
					(unsigned long)response->size);
		}
		else
		{
			memcpy(client_map->complete_map, response->map, response->size);
		}
#else
		if (response->compressed)
		{
			carmen_warn("Received compressed map from server. This program was\n"
					"compiled without zlib support, so this map cannot be\n"
					"used. Sorry.\n");

			memset(client_map, 0, sizeof(carmen_map_t));

			if (response->map)
				free(response->map);
			if (response->err_mesg)
				free(response->err_mesg);
			free(response);

			return -1;
		}
		else
		{
			memcpy(client_map->complete_map, response->map, response->size);
		}
#endif
		client_map->map = (double **) calloc(response->config.x_size, sizeof(double *));
		carmen_test_alloc(client_map->map);

		for(i = 0; i < response->config.x_size; i++)
			client_map->map[i] = client_map->complete_map + i * response->config.y_size;
	}

	if (response->map)
		free(response->map);
	if (response->err_mesg)
		free(response->err_mesg);
	free(response);

	return 0;
}
*/

/*
int
carmen_map_get_gridmap(carmen_map_p client_map)
{
	return carmen_map_get_gridmap_by_name(NULL, client_map);
}
*/

/*
void 
carmen_placelist_interface_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
 				   void *clientData __attribute__ ((unused)))
{
  carmen_map_placelist_message internal_placelist;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err = IPC_OK;
  int i;

  formatter = IPC_msgInstanceFormatter(msgRef);

  if (client_placelist)
  err = IPC_unmarshallData(formatter, callData, &internal_placelist, 
			     sizeof(carmen_map_placelist_message));

  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));

  if (client_placelist)
    {
      client_placelist->num_places = internal_placelist.num_places;
       client_placelist->places = (carmen_place_p)calloc
	(internal_placelist.num_places, sizeof(carmen_place_t));
      carmen_test_alloc(client_placelist->places);
      for (i = 0; i < client_placelist->num_places; i++)
	memcpy(&client_placelist->places[i],&internal_placelist.places[i],
	       sizeof(carmen_place_t));
      free (internal_placelist.places);
    }
  if (internal_placelist_handler)
    internal_placelist_handler();
}
 */

/* subscribe to incoming placelist messages */
void 
carmen_map_subscribe_placelist_message(carmen_map_placelist_p placelist 
		__attribute__ ((unused)),
		void (*handler)(void)
		__attribute__ ((unused)))
{
	carmen_warn("Subscribing to placelist messages is currently unsupported.\n");

	return;
}

/* send a request for a placelist */
int
carmen_map_get_placelist_by_name(char *name, carmen_map_placelist_p placelist)
{

	IPC_RETURN_TYPE err;
	static carmen_placelist_request_message *query;
	static carmen_map_named_placelist_request named_query;
	static carmen_map_placelist_message *response;
	unsigned int timeout = 10000;

	if (name) {
		err = IPC_defineMsg(CARMEN_NAMED_PLACELIST_REQUEST_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_NAMED_PLACELIST_REQUEST_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAMED_PLACELIST_REQUEST_NAME);


		named_query.name = calloc(strlen(name) + 1, sizeof(char));
		carmen_test_alloc(named_query.name);
		strcpy(named_query.name, name);
		named_query.host = carmen_get_host();
		named_query.timestamp = carmen_get_time();

		err = IPC_queryResponseData(CARMEN_NAMED_PLACELIST_REQUEST_NAME, &named_query,
				(void **)&response, timeout);
		carmen_test_ipc_return_int(err, "Could not get placelist",
				CARMEN_NAMED_PLACELIST_REQUEST_NAME);
	}
	else {
		err = IPC_defineMsg(CARMEN_PLACELIST_REQUEST_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_DEFAULT_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_PLACELIST_REQUEST_NAME);

		query = carmen_default_message_create();

		err = IPC_queryResponseData(CARMEN_PLACELIST_REQUEST_NAME, query,
				(void **)&response, timeout);
		carmen_test_ipc_return_int(err, "Could not get placelist",
				CARMEN_PLACELIST_REQUEST_NAME);
	}

	if (placelist)
	{
		placelist->places = response->places;
		placelist->num_places = response->num_places;
	}
	else
		free(response->places);

	free(response);

	return 0;
}

int
carmen_map_get_placelist(carmen_map_placelist_p placelist)
{
	return carmen_map_get_placelist_by_name(NULL, placelist);
}

/* send a request for a gridmap */
int
carmen_map_get_offlimits_by_name(char *name, carmen_offlimits_p *offlimits, int *list_length)
{

	IPC_RETURN_TYPE err;
	static carmen_offlimits_request_message *query;
	static carmen_map_named_offlimits_request named_query;
	static carmen_map_offlimits_message *response;
	unsigned int timeout = 10000;

	if (name) {
		err = IPC_defineMsg(CARMEN_NAMED_OFFLIMITS_REQUEST_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_NAMED_OFFLIMITS_REQUEST_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAMED_OFFLIMITS_REQUEST_NAME);

		named_query.name = calloc(strlen(name) + 1, sizeof(char));
		carmen_test_alloc(named_query.name);
		strcpy(named_query.name, name);
		named_query.host = carmen_get_host();
		named_query.timestamp = carmen_get_time();

		err = IPC_queryResponseData(CARMEN_NAMED_OFFLIMITS_REQUEST_NAME, &named_query,
				(void **)&response, timeout);
		carmen_test_ipc(err, "Could not get map_request",
				CARMEN_NAMED_OFFLIMITS_REQUEST_NAME);
	}
	else {
		err = IPC_defineMsg(CARMEN_OFFLIMITS_REQUEST_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_DEFAULT_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_OFFLIMITS_REQUEST_NAME);

		query = carmen_default_message_create();

		err = IPC_queryResponseData(CARMEN_OFFLIMITS_REQUEST_NAME, query,
				(void **)&response, timeout);
		carmen_test_ipc(err, "Could not get map_request",
				CARMEN_OFFLIMITS_REQUEST_NAME);
	}

	if (err != IPC_OK)
	{
		carmen_warn("\nDid you remember to start the mapserver, or give "
				"a map to the paramServer?\n");
		return -1;
	}

	if (offlimits)
	{
		*list_length = response->list_length;
		*offlimits = response->offlimits_list;
	}

	free(response);

	return 0;
}

int
carmen_map_get_offlimits(carmen_offlimits_p *offlimits, int *list_length)
{
	return carmen_map_get_offlimits_by_name(NULL, offlimits, list_length);
}

int 
carmen_map_apply_offlimits_chunk_to_map(carmen_offlimits_p offlimits_list,
		int list_length, carmen_map_p map)
{
	int current_x, current_y, x, y;
	carmen_bresenham_param_t b_params;
	int i;
	carmen_offlimits_t cur_seg;

	/* This function requires a map to markup */
	if(offlimits_list == NULL || list_length == 0 ||
			map == NULL || map->complete_map == NULL)
		return -1;

	for(i = 0; i < list_length; i++)
	{
		cur_seg = offlimits_list[i];
		switch(cur_seg.type)
		{
		case CARMEN_OFFLIMITS_POINT_ID:
			if(cur_seg.x1 >= 0 && cur_seg.y1 >= 0 &&
					cur_seg.x1 < map->config.x_size &&
					cur_seg.y1 < map->config.y_size)
				map->map[cur_seg.x1][cur_seg.y1] += 2.0;
			break;

		case CARMEN_OFFLIMITS_LINE_ID:
			carmen_get_bresenham_parameters(cur_seg.x1, cur_seg.y1, cur_seg.x2,
					cur_seg.y2, &b_params);
			do
			{
				carmen_get_current_point(&b_params, &current_x, &current_y);
				if(current_x >= 0 && current_y >= 0 &&
						current_x < map->config.x_size &&
						current_y < map->config.y_size)
					map->map[current_x][current_y] += 2.0;
			}
			while(carmen_get_next_point(&b_params));
			break;
		case CARMEN_OFFLIMITS_RECT_ID:
			for(x = cur_seg.x1; x <= cur_seg.x2; x++)
				for(y = cur_seg.y1; y <= cur_seg.y2; y++)
					if(x >= 0 && y >= 0 &&
							x < map->config.x_size && y < map->config.y_size)
						map->map[x][y] += 2.0;
			break;
		default:
			break;
		}
	}
	return 0;
}

int carmen_map_get_global_offset_by_name(char *name, 
		carmen_global_offset_t *global_offset)
{

	IPC_RETURN_TYPE err;
	static carmen_global_offset_request_message *query;
	static carmen_map_named_global_offset_request named_query;
	static carmen_map_global_offset_message *response;
	unsigned int timeout = 10000;

	if (name) {
		err = IPC_defineMsg(CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME);

		named_query.name = calloc(strlen(name) + 1, sizeof(char));
		carmen_test_alloc(named_query.name);
		strcpy(named_query.name, name);
		named_query.host = carmen_get_host();
		named_query.timestamp = carmen_get_time();

		err = IPC_queryResponseData(CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME,
				&named_query, (void **)&response, timeout);
		carmen_test_ipc(err, "Could not get map_request",
				CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME);
	} else {
		err = IPC_defineMsg(CARMEN_GLOBAL_OFFSET_REQUEST_NAME,
				IPC_VARIABLE_LENGTH, CARMEN_DEFAULT_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_GLOBAL_OFFSET_REQUEST_NAME);

		query = carmen_default_message_create();

		err = IPC_queryResponseData(CARMEN_GLOBAL_OFFSET_REQUEST_NAME, query,
				(void **)&response, timeout);
		carmen_test_ipc(err, "Could not get map_request",
				CARMEN_GLOBAL_OFFSET_REQUEST_NAME);
	}

	if (err != IPC_OK) {
		carmen_warn("\nDid you remember to start the mapserver, or give "
				"a map to the paramServer?\n");
		return -1;
	}

	if (global_offset) {
		*global_offset = response->global_offset;
	}

	free(response);

	return 0;
}

int
carmen_map_get_global_offset(carmen_global_offset_t *global_offset)
{
	return carmen_map_get_global_offset_by_name(NULL, global_offset);
}

carmen_inline int 
carmen_map_to_world(carmen_map_point_p carmen_map_point, 
		carmen_world_point_p world_point)
{
	double x = carmen_map_point->x * carmen_map_point->map->config.resolution;
	double y = carmen_map_point->y * carmen_map_point->map->config.resolution;

	world_point->pose.x = x;
	world_point->pose.y = y;
	world_point->pose.theta = 0;
	world_point->map = carmen_map_point->map;

	return 0;
}

carmen_inline int 
carmen_world_to_map(carmen_world_point_p world_point, carmen_map_point_p carmen_map_point)
{
	if (!world_point || !carmen_map_point || !world_point->map)
		return (-1);

	int x = carmen_round((world_point->pose.x - world_point->map->config.x_origin) / world_point->map->config.resolution);
	int y = carmen_round((world_point->pose.y - world_point->map->config.y_origin) / world_point->map->config.resolution);

	if (x < 0 || x >= world_point->map->config.x_size || y < 0 || y >= world_point->map->config.y_size)
		return (-1);

	carmen_map_point->x = x;
	carmen_map_point->y = y;

	carmen_map_point->map = world_point->map;

	return (0);
}

carmen_inline int 
carmen_map_to_trajectory(carmen_map_point_p carmen_map_point, carmen_traj_point_p traj_point)
{
	double x = carmen_map_point->x * carmen_map_point->map->config.resolution;
	double y = carmen_map_point->y * carmen_map_point->map->config.resolution;

	traj_point->x = x;
	traj_point->y = y;
	traj_point->theta = 0.0;
	traj_point->t_vel = 0.0;
	traj_point->r_vel = 0.0;

	return 0;
}


carmen_inline int
carmen_map_to_ackerman_trajectory(carmen_map_point_p carmen_map_point,
		carmen_ackerman_traj_point_p traj_point)
{
	double x = (carmen_map_point->x * carmen_map_point->map->config.resolution) + carmen_map_point->map->config.x_origin;
	double y = (carmen_map_point->y * carmen_map_point->map->config.resolution) + carmen_map_point->map->config.y_origin;

	traj_point->x = x;
	traj_point->y = y;
	traj_point->theta = 0.0;
	traj_point->v = 0.0;
	traj_point->phi = 0.0;

	return 0;
}


carmen_inline int
carmen_point_to_map(carmen_point_p point, carmen_map_point_p map_point, 
		carmen_map_p map)
{
	int x = carmen_round(point->x / map->config.resolution);
	int y = carmen_round(point->y / map->config.resolution);

	map_point->x = x;
	map_point->y = y;
	map_point->map = map;

	return 0;
}

carmen_inline int 
carmen_trajectory_to_map(carmen_traj_point_p traj_point, 
		carmen_map_point_p carmen_map_point,
		carmen_map_p map)
{
	int x = carmen_round(traj_point->x / map->config.resolution);
	int y = carmen_round(traj_point->y / map->config.resolution);

	carmen_map_point->x = x;
	carmen_map_point->y = y;

	carmen_map_point->map = map;

	return 0;
}

carmen_inline int
carmen_ackerman_trajectory_to_map(carmen_robot_and_trailer_traj_point_t *traj_point,
		carmen_map_point_p carmen_map_point,
		carmen_map_p map)
{
	int x = carmen_round((traj_point->x - map->config.x_origin)/ map->config.resolution);
	int y = carmen_round((traj_point->y - map->config.y_origin) / map->config.resolution);

	carmen_map_point->x = x;
	carmen_map_point->y = y;

	carmen_map_point->map = map;

	return 0;
}

/* distance between two carmen_map_points in grid cells */
carmen_inline double 
carmen_distance_map(carmen_map_point_p p1, carmen_map_point_p p2) 
{
	return hypot(p1->x-p2->x, p1->y-p2->y);
}

carmen_inline double 
carmen_distance_world(carmen_world_point_p p1, carmen_world_point_p p2) 
{
	return hypot(p1->pose.x-p2->pose.x, p1->pose.y-p2->pose.y);
}

carmen_inline int
carmen_map_cmp(carmen_map_point_p p1, carmen_map_point_p p2)
{
	if (p1->x == p2->x && p1->y == p2->y)
		return 0;

	return 1;
}

carmen_inline int
carmen_world_cmp(carmen_world_point_p p1, carmen_world_point_p p2)
{
	if (fabs(p1->pose.x - p2->pose.x) < 1.0 &&
			fabs(p1->pose.y - p2->pose.y) < 1.0 &&
			fabs(carmen_normalize_theta(p1->pose.theta - p2->pose.theta)) < .017)
		return 0;

	return 1;
}


carmen_inline void
carmen_map_copy(carmen_map_p dst_map, carmen_map_p src_map) 
{
	if (dst_map == NULL)
		carmen_die("dst_map == NULL in carmen_map_copy()\n");
	if (dst_map->complete_map == NULL)
		carmen_die("dst_map->complete_map == NULL in carmen_map_copy()\n");
	if (dst_map->map == NULL)
		carmen_die("dst_map->map == NULL in carmen_map_copy()\n");
	if ((dst_map->config.x_size != src_map->config.x_size) || (dst_map->config.y_size != src_map->config.y_size))
		carmen_die("dst_map size different from src_map size in carmen_map_copy()\n");
	
	memcpy(dst_map->complete_map, src_map->complete_map, sizeof(double) * src_map->config.x_size * src_map->config.y_size);
}


carmen_inline carmen_map_p
carmen_map_clone(carmen_map_p map) 
{
	carmen_map_p new_map;
	int i;

	new_map = (carmen_map_p) calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(new_map);

	new_map->complete_map = (double *) calloc(map->config.x_size * map->config.y_size, sizeof(double));
	carmen_test_alloc(new_map->complete_map);

	new_map->map = (double **) calloc(map->config.x_size, sizeof(double *));
	carmen_test_alloc(new_map->map);

	for (i = 0; i < map->config.x_size; i++)
		new_map->map[i] = new_map->complete_map + i * map->config.y_size;
	
	new_map->config = map->config;
	carmen_map_copy(new_map, map);

	return (new_map);
}


carmen_inline void
carmen_map_destroy(carmen_map_p *map)
{
	free((*map)->complete_map);
	free((*map)->map);
//	if ((*map)->config.map_name != NULL)
//		free((*map)->config.map_name);
	free((*map));
	*map = NULL;
}


void
carmen_map_interface_set_superimposed_map(carmen_map_p map)
{
	superimposed_map = map;
}


carmen_map_p
carmen_map_interface_get_superimposed_map()
{
	return (superimposed_map);
}


carmen_map_t *
carmen_map_interface_create_new_empty_map(carmen_map_config_t *reference_config)
{
	int i, j;
	carmen_map_t *new_empty_map;
	
	new_empty_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
	new_empty_map->config = *reference_config;

	new_empty_map->complete_map = (double *) malloc(sizeof(double) * new_empty_map->config.x_size * new_empty_map->config.y_size);
	carmen_test_alloc(new_empty_map->complete_map);
	new_empty_map->map = (double **) malloc(sizeof(double *) * new_empty_map->config.x_size);
	carmen_test_alloc(new_empty_map->map);

	for (i = 0; i < new_empty_map->config.x_size; i++)
	{
		new_empty_map->map[i] = &(new_empty_map->complete_map[i * new_empty_map->config.y_size]);
		for (j = 0; j < new_empty_map->config.y_size; j++)
		{
			new_empty_map->map[i][j] = -1.0;
		}
	}
	return (new_empty_map);
}


carmen_map_t *
carmen_map_interface_create_new_empty_log_odds_map(carmen_map_config_t *reference_config)
{
	int i;
	carmen_map_t *new_empty_map;

	new_empty_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
	new_empty_map->config = *reference_config;

	// This line initializes the maps with zeros!
	new_empty_map->complete_map = (double *) calloc(new_empty_map->config.x_size * new_empty_map->config.y_size, sizeof(double));
	carmen_test_alloc(new_empty_map->complete_map);

	new_empty_map->map = (double **) malloc(sizeof(double *) * new_empty_map->config.x_size);
	carmen_test_alloc(new_empty_map->map);

	for (i = 0; i < new_empty_map->config.x_size; i++)
		new_empty_map->map[i] = &(new_empty_map->complete_map[i * new_empty_map->config.y_size]);

	return (new_empty_map);
}
