#include <vector>
#include <carmen/carmen.h>
#include <carmen/moving_average.h>

using namespace std;

#define MOVING_OBJECT_HISTORY_SIZE 	40
#define MAX_NON_DETECTION_COUNT		15

typedef struct
{
	int valid;
	int v_valid;
	int index;
	double width;
	double length;
	carmen_ackerman_traj_point_t pose;
	double lateral_v;
	vector <carmen_position_t> moving_object_points;
	double timestamp;
} moving_object_history_t;

typedef struct
{
	int id;
	moving_object_history_t history[MOVING_OBJECT_HISTORY_SIZE];
//	double v;
	carmen_moving_average_c average_width;
	carmen_moving_average_c average_length;
	carmen_ackerman_traj_point_t pose;
	carmen_moving_average_c average_longitudinal_v;
	carmen_moving_average_c average_lateral_v;
	int non_detection_count;
} moving_object_t;

typedef struct
{
	int category;
	double width;
	double length;
} vehicle_category_t;


carmen_moving_objects_point_clouds_message *
obstacle_distance_mapper_datmo(carmen_route_planner_road_network_message *road_network,
		carmen_map_t &occupancy_map, carmen_map_server_offline_map_message *offline_map, double timestamp);

void
obstacle_distance_mapper_remove_moving_objects_from_occupancy_map(carmen_map_t *occupancy_map,
		carmen_moving_objects_point_clouds_message *moving_objects);

void
obstacle_distance_mapper_restore_moving_objects_to_occupancy_map(carmen_map_t *occupancy_map,
		carmen_moving_objects_point_clouds_message *moving_objects);

carmen_compact_map_t *
obstacle_distance_mapper_uncompress_occupancy_map(carmen_map_t *occupancy_map, carmen_compact_map_t *compact_occupancy_map,
		carmen_mapper_compact_map_message *message);

void
obstacle_distance_mapper_free_moving_objects_message(carmen_moving_objects_point_clouds_message *moving_objects);

