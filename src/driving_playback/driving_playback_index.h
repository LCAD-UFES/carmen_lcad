
#ifndef RDDF_INDEX_H_
#define RDDF_INDEX_H_

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

// void carmen_driving_playback_create_index_from_rddf_log(char *rddf_filename);
int carmen_search_next_poses_index(double x, double y, double yaw, double timestamp /* only for debugging */,
		carmen_robot_and_trailers_traj_point_t *poses_ahead, int n, int *annotations, double distance_between_waypoints);
int carmen_find_poses_around(double x, double y, double yaw, double timestamp /* only for debugging */,
		carmen_robot_and_trailers_traj_point_t *poses_ahead, int n);
void carmen_driving_playback_load_index(char *rddf_filename);
int carmen_driving_playback_index_exists(char *rddf_filename);

void carmen_driving_playback_index_add(const carmen_fused_odometry_message *fused_odometry_message, long data_offset, long data_length, int annotation);
void carmen_driving_playback_index_save(char *rddf_filename);

#ifdef __cplusplus
}
#endif

#endif /* RDDF_INDEX_H_ */
