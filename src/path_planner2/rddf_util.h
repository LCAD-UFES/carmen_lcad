
#ifndef RDDF_UTIL_H_
#define RDDF_UTIL_H_

#include <carmen/carmen.h>
#include "rddf_messages.h"

#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>
#include <kml/engine/kml_file.h>

typedef struct
{
	carmen_point_t pose;
	double phi;
	double max_velocity;
	double timestamp;
	double driver_velocity;
} carmen_rddf_waypoint;

typedef std::vector<kmldom::PlacemarkPtr> placemark_vector_t;

void carmen_rddf_play_open_kml();
void carmen_rddf_play_save_waypoints(char *carmen_rddf_filename);
void carmen_rddf_play_add_waypoint(double latitude, double longitude);
void carmen_rddf_play_add_waypoint_speed(double latitude, double longitude, double max_speed, double driver_speed, double theta, double timestamp);

void carmen_rddf_play_open_kml(const char *filename, placemark_vector_t *placemark_vector);
int carmen_rddf_play_copy_kml(kmldom::PlacemarkPtr waypoint, carmen_fused_odometry_message *message, int *waypoint_annotation);

carmen_rddf_waypoint * carmen_rddf_play_load_rddf_from_file(char *rddf_filename, int *out_waypoint_vector_size);
void carmen_rddf_play_save_rddf_to_file(char *rddf_filename, carmen_rddf_waypoint *waypoint_vector, int size);

bool carmen_rddf_play_annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_ackerman_traj_point_t annotation_point);
bool carmen_rddf_play_annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_vector_3D_t annotation_point);
bool carmen_rddf_play_annotation_is_forward(carmen_point_t robot_pose, carmen_vector_3D_t annotation_point);

bool carmen_unpack_multi_paths_message (carmen_rddf_multi_path_message *message, int *tree_path_num, int **tree_path_sizes,
		double **tree_path_costs_ref, carmen_ackerman_traj_point_t ***multi_path_ref);

#endif
