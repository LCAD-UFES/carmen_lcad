#ifndef VARIABLE_VELODYNE_DRAWER_H_
#define VARIABLE_VELODYNE_DRAWER_H_

typedef struct variable_velodyne_drawer variable_velodyne_drawer;

variable_velodyne_drawer*
create_variable_velodyne_drawer(int flipped,carmen_pose_3D_t velodyne_pose, int max_scans_number, int vertical_resolution, int camera,
		int stereo_velodyne_vertical_roi_ini, int stereo_velodyne_vertical_roi_end, int stereo_velodyne_horizontal_roi_ini, int stereo_velodyne_horizontal_roi_end, int bumblebee_basic_width, int bumblebee_basic_height);

void destroy_variable_velodyne_drawer(variable_velodyne_drawer* v_drawer);
void add_variable_velodyne_message(variable_velodyne_drawer *v_drawer, carmen_velodyne_variable_scan_message *velodyne_message, carmen_pose_3D_t car_pose, carmen_pose_3D_t sensor_board_1_pose);
void draw_variable_velodyne(variable_velodyne_drawer* v_drawer);

#endif
