
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

typedef struct
{
	carmen_annotation_t annotation;
	size_t index;
} annotation_and_index;

typedef std::vector<kmldom::PlacemarkPtr> placemark_vector_t;

void carmen_rddf_play_open_kml();
void carmen_rddf_play_save_waypoints(char *carmen_rddf_filename);
void carmen_rddf_play_add_waypoint(double latitude, double longitude);
void carmen_rddf_play_add_waypoint_speed(double latitude, double longitude, double max_speed, double driver_speed, double theta, double timestamp);

void carmen_rddf_play_open_kml(const char *filename, placemark_vector_t *placemark_vector);
int carmen_rddf_play_copy_kml(kmldom::PlacemarkPtr waypoint, carmen_fused_odometry_message *message, int *waypoint_annotation);

carmen_rddf_waypoint * carmen_rddf_play_load_rddf_from_file(char *rddf_filename, int *out_waypoint_vector_size);
void carmen_rddf_play_save_rddf_to_file(char *rddf_filename, carmen_rddf_waypoint *waypoint_vector, int size);

//bool carmen_rddf_play_annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_ackerman_traj_point_t annotation_point);
bool carmen_rddf_play_annotation_is_forward(carmen_robot_and_trailer_traj_point_t robot_pose, carmen_vector_3D_t annotation_point);
bool carmen_rddf_play_annotation_is_forward(carmen_robot_and_trailer_traj_point_t robot_pose, carmen_robot_and_trailer_traj_point_t annotation_point);
//bool carmen_rddf_play_annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_vector_3D_t annotation_point);
bool carmen_rddf_play_annotation_is_forward(carmen_point_t robot_pose, carmen_vector_3D_t annotation_point);

void carmen_rddf_play_get_parameters(int argc, char** argv);
char *carmen_rddf_play_parse_input_command_line_parameters(int argc, char **argv);
void carmen_rddf_play_clear_rddf_loop_flag();

void carmen_rddf_play_load_annotation_file(char *carmen_annotation_filename);
void carmen_rddf_play_load_index(char *rddf_filename);
int carmen_rddf_play_pose_out_of_map_coordinates(carmen_point_t pose, carmen_map_p map);
void carmen_rddf_play_check_reset_traffic_sign_state(carmen_point_t new_pose);
int carmen_rddf_play_find_nearest_poses_by_road_map(carmen_point_t initial_pose, carmen_map_p road_map,
		carmen_robot_and_trailer_traj_point_t *poses_ahead, carmen_robot_and_trailer_traj_point_t *poses_back,
		int *num_poses_back, int num_poses_ahead_max);
int carmen_rddf_play_find_nearest_poses_ahead(double x, double y, double yaw, double v, double timestamp /* only for debugging */,
		carmen_robot_and_trailer_traj_point_t *poses_ahead, carmen_robot_and_trailer_traj_point_t *poses_back,
		int *num_poses_back, int num_poses_ahead_max, int *rddf_annotations);
void carmen_rddf_play_clear_annotations(int *rddf_annotations, int num_annotations);
void carmen_rddf_play_clear_annotations();
void carmen_rddf_play_set_annotations(carmen_point_t robot_pose);
void carmen_check_for_annotations(carmen_point_t robot_pose,
		carmen_robot_and_trailer_traj_point_t *carmen_rddf_poses_ahead, carmen_robot_and_trailer_traj_point_t *carmen_rddf_poses_back,
		int carmen_rddf_num_poses_ahead, int carmen_rddf_num_poses_back, double timestamp);
void carmen_rddf_play_updade_annotation_vector(crud_t action, carmen_annotation_t old_annotation, carmen_annotation_t new_annotation);


#endif
