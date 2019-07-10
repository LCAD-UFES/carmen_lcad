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
} carmen_obstacle_distance_mapper_map_message;

#define CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_NAME	"carmen_obstacle_distance_mapper_map_message"
#define CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_FMT		"{{int, int, double, [byte:64], string, double, double}, int, <short:2>, <short:2>, double, string}"

#define CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_NAME(level)	(level ? "carmen_obstacle_distance_mapper_map_level"#level"_message" : "carmen_obstacle_distance_mapper_map_message")
#define CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_FMT			CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_FMT

typedef struct
{
	carmen_map_config_t config;
	int size;
	short int *coord_x;
	short int *coord_y;
	char *x_offset;
	char *y_offset;
	double timestamp;
	char *host;
} carmen_obstacle_distance_mapper_compact_map_message;

#define CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME	"carmen_obstacle_distance_mapper_compact_map_message"
#define CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_FMT		"{{int, int, double, [byte:64], string, double, double}, int, <short:2>, <short:2>, <byte:2>, <byte:2>, double, string}"

#define CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_NAME(level)	(level ? "carmen_obstacle_distance_mapper_compact_map_level"#level"_message" : "carmen_obstacle_distance_mapper_compact_map_message")
#define CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_FMT			CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_FMT

#define CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_NAME	"carmen_obstacle_distance_mapper_compact_lane_contents_message"
#define CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_FMT	"{{int, int, double, [byte:64], string, double, double}, int, <short:2>, <short:2>, <byte:2>, <byte:2>, double, string}"

#define CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_NAME	"carmen_behaviour_selector_compact_lane_contents_message"
#define CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_FMT		"{{int, int, double, [byte:64], string, double, double}, int, <short:2>, <short:2>, <byte:2>, <byte:2>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
