#ifndef VELODYNE_INTENSITY_DRAWER_H_
#define VELODYNE_INTENSITY_DRAWER_H_

#include "viewer_3D.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct velodyne_intensity_drawer velodyne_intensity_drawer;

velodyne_intensity_drawer* create_velodyne_intensity_drawer(carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t sensor_board_pose);
void velodyne_intensity_drawer_add_velodyne_message(velodyne_intensity_drawer* v_drawer, carmen_velodyne_partial_scan_message* velodyne_message, carmen_pose_3D_t car_fused_pose, carmen_vector_3D_t car_fused_velocity, double car_fused_time);
void draw_velodyne_intensity(velodyne_intensity_drawer* v_drawer);
void destroy_velodyne_intensity_drawer(velodyne_intensity_drawer* v_drawer);

#ifdef __cplusplus
}
#endif

#endif
