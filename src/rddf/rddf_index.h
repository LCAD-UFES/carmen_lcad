/*
 * rddf_index.h
 *
 *  Created on: 17/07/2012
 *      Author: filipe
 */

#ifndef RDDF_INDEX_H_
#define RDDF_INDEX_H_

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

// void carmen_rddf_create_index_from_rddf_log(char *rddf_filename);
int carmen_search_next_poses_index(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int n, int *annotations, int perform_loop);
int carmen_find_poses_around(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t *poses_ahead, int n);
void carmen_rddf_load_index(char *rddf_filename);
int carmen_rddf_index_exists(char *rddf_filename);

void carmen_rddf_index_add(const carmen_fused_odometry_message *fused_odometry_message, long data_offset, long data_length, int annotation);
void carmen_rddf_index_save(char *rddf_filename);

#ifdef __cplusplus
}
#endif

#endif /* RDDF_INDEX_H_ */
