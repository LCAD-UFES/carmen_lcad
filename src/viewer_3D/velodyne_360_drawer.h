#ifndef VELODYNE_360_DRAWER_H_
#define VELODYNE_360_DRAWER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct velodyne_360_drawer velodyne_360_drawer;

velodyne_360_drawer* create_velodyne_360_drawer(carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t sensor_board_pose);
int velodyne_360_drawer_get_num_points_current (velodyne_360_drawer* v_drawer);
double velodyne_360_drawer_get_angle_current (velodyne_360_drawer* v_drawer, int index);

void destroy_velodyne_360_drawer(velodyne_360_drawer* v_drawer);
void add_velodyne_message(velodyne_360_drawer* v_drawer, carmen_velodyne_partial_scan_message* velodyne_message);
void draw_velodyne_360(velodyne_360_drawer* v_drawer, carmen_pose_3D_t car_pose);

#ifdef __cplusplus
}
#endif

#endif
