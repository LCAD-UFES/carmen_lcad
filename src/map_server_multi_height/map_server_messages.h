/*
 * map_server_messages.h
 *
 *  Created on: 26/09/2012
 *      Author: romulo
 */

#include <carmen/carmen.h>
#include <carmen/mapper_messages.h>


#ifndef MAP_SERVER_MESSAGES_H_
#define MAP_SERVER_MESSAGES_H_

#define CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME "carmen_map_server_request_current_offline_map_name"
#define CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_FMT CARMEN_DEFAULT_MESSAGE_FMT

typedef carmen_mapper_map_message carmen_map_server_current_map_name;

#define CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_NAME  "carmen_map_server_current_offline_map_name"
#define CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_FMT   CARMEN_MAPPER_MAP_MESSAGE_FMT

typedef carmen_mapper_map_message carmen_map_server_offline_map_message;

#define CARMEN_MAP_SERVER_OFFLINE_MAP_NAME 	"carmen_map_server_offline_map_message"
#define CARMEN_MAP_SERVER_OFFLINE_MAP_FMT 	CARMEN_MAPPER_MAP_MESSAGE_FMT
#define CARMEN_MAP_SERVER_OFFLINE_MAP_LEVEL1_NAME 	"carmen_map_server_offline_map_level1_message"

typedef carmen_mapper_map_message carmen_map_server_road_map_message;

#define CARMEN_MAP_SERVER_ROAD_MAP_NAME 	"carmen_map_server_road_map_message"
#define CARMEN_MAP_SERVER_ROAD_MAP_FMT 	CARMEN_MAPPER_MAP_MESSAGE_FMT

typedef carmen_mapper_compact_map_message carmen_map_server_compact_cost_map_message;

#define CARMEN_MAP_SERVER_COMPACT_COST_MAP_NAME 	"carmen_map_server_compact_cost_map_message"
#define CARMEN_MAP_SERVER_COMPACT_COST_MAP_FMT 	CARMEN_MAPPER_COMPACT_MAP_MESSAGE_FMT

typedef carmen_mapper_compact_map_message carmen_map_server_compact_lane_map_message;

#define CARMEN_MAP_SERVER_COMPACT_LANE_MAP_NAME	"carmen_map_server_compact_lane_map_message"
#define CARMEN_MAP_SERVER_COMPACT_LANE_MAP_FMT 	CARMEN_MAPPER_COMPACT_MAP_MESSAGE_FMT


typedef enum {
	CARMEN_MAP_SERVER_LANE_MAP
} carmen_map_server_cost_map_type;

//typedef struct {
//	double *complete_map;
//	int size;
//	carmen_map_config_t config;
//	int cost_map_type;
//	double timestamp;
//	char *host;
//} carmen_map_server_cost_map;

typedef carmen_mapper_map_message carmen_map_server_cost_map;

#define CARMEN_MAP_SERVER_COST_MAP_NAME		"carmen_map_server_cost_map_name"
#define CARMEN_MAP_SERVER_COST_MAP_FMT CARMEN_MAPPER_MAP_MESSAGE_FMT
//#define CARMEN_MAP_SERVER_COST_MAP_FMT		"{<double:2>, int, {int, int, double, [byte:64], string, double, double}, int, double, string}"

typedef carmen_mapper_map_message carmen_map_server_lane_map;

#define CARMEN_MAP_SERVER_LANE_MAP_NAME		"carmen_map_server_lane_map_name"
#define CARMEN_MAP_SERVER_LANE_MAP_FMT CARMEN_MAPPER_MAP_MESSAGE_FMT

typedef struct {
	carmen_map_config_t config;
	int size;
	double *complete_mean_remission_map;
	double *complete_variance_remission_map;
	double *complete_map;
	//double *complete_distance;
	double *complete_prob;
	double *complete_gprob;
	//short int *complete_x_offset;
	//short int *complete_y_offset;
	double timestamp;
	char *host;
} carmen_map_server_localize_map_message;

#define CARMEN_MAP_SERVER_LOCALIZE_MAP_NAME		"carmen_map_server_localize_map_name"
//#define CARMEN_MAP_SERVER_LOCALIZE_MAP_FMT		"{{int, int, double, [byte:64], string, double, double}, int, <double:2>, <double:2>, <double:2>, <double:2>, <double:2>, <double:2>, <short:2>, <short:2>, double, string}"
#define CARMEN_MAP_SERVER_LOCALIZE_MAP_FMT		"{{int, int, double, [byte:64], string, double, double}, int, <double:2>, <double:2>, <double:2>, <double:2>, <double:2>, double, string}"

#endif /* MAP_SERVER_MESSAGES_H_ */
