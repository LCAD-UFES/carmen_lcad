#ifndef CARMEN_OBSTACLE_DISTANCE_MAPPER_MESSAGES_H
#define CARMEN_OBSTACLE_DISTANCE_MAPPER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	carmen_map_config_t config;
	int size;
	short int *complete_x_offset;
	short int *complete_y_offset;
	double timestamp;
	char *host;
} carmen_mapper_distance_map_message;

#define CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_NAME	"carmen_mapper_distance_map_message"
#define CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_FMT	"{{int, int, double, [byte:64], string, double, double}, int, <short:2>, <short:2>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
