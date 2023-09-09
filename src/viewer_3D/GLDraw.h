#ifndef GLDRAW_H_
#define GLDRAW_H_

#include <carmen/carmen.h>
#include "draw_car.h"
#include "map_drawer.h"

#ifdef __cplusplus
extern "C"
{
#endif

void initGl(int width, int height, carmen_vector_3D_t robot_size);

void set_camera(carmen_pose_3D_t pose);
carmen_pose_3D_t get_camera_pose();
carmen_pose_3D_t get_camera_offset();
void set_camera_offset(carmen_pose_3D_t offset);
void move_camera(carmen_vector_3D_t displacement);
void rotate_camera(carmen_orientation_3D_t rotation);
void rotate_camera_offset(carmen_orientation_3D_t rotation);
void set_background_color(double r, double g, double b);

void enable_free_mode();
void disable_free_mode(carmen_orientation_3D_t orientation);
void reset_camera_position(carmen_vector_3D_t robot_size);
int get_camera_mode();
void set_camera_mode(int mode, carmen_vector_3D_t robot_size);
void move_front_camera(double moviment);

void set_laser(float x, float y, float z, float roll, float pitch, float yaw);

void draw_all(carmen_fused_odometry_message* odometry_message, int odometrySize, int odometryStart, carmen_laser_laser_message* laser_message, int laserSize, int laserStart);

void reset_camera();
void draw_stereo_point_cloud(point_cloud *stereo_point_cloud, int stereo_point_cloud_size);
void draw_laser_points(point_cloud *laser_points, int laser_size);
void draw_laser_points_vbo(point_cloud *laser_points, int laser_size);
void draw_laser_mesh(point_cloud *laser_readings, int laser_size);
void draw_moving_objects_point_clouds (point_cloud *moving_objects_point_clouds, int cloud_size,
		carmen_vector_3D_t offset, CarDrawer *car_drawer, double mapper_map_grid_res);
void draw_velodyne_points(point_cloud *velodyne_points, int cloud_size);
void draw_velodyne_points_color(point_cloud *velodyne_points, int cloud_size);
void draw_gps(carmen_vector_3D_t *gps_trail, int *gps_nr, int size);
void draw_gps_xsens_xyz(carmen_vector_3D_t *gps_trail, int size);
void draw_odometry(carmen_vector_3D_t *odometry_trail, int size);
void draw_localize_ackerman(carmen_vector_3D_t* localize_ackerman_trail, int size);
void draw_particles(carmen_vector_3D_t* particles_pos, double* particles_weight, int num_particles, int color);
void draw_laser_rays(point_cloud current_reading, carmen_vector_3D_t laser_position);
void draw_xsens_orientation(carmen_orientation_3D_t xsens_orientation, double xsens_yaw_bias, carmen_pose_3D_t xsens_pose, carmen_pose_3D_t sensor_board_pose, carmen_pose_3D_t car_pose);
void draw_gps_orientation(double gps_orientation, int gps_heading_valid, carmen_pose_3D_t gps_pose, carmen_pose_3D_t sensor_board_pose, carmen_pose_3D_t car_pose);
void draw_orientation_instruments(carmen_orientation_3D_t orientation, double r, double g, double b);
void draw_gps_fault_signal(void);
void draw_map_image(carmen_vector_3D_t gps_position_at_turn_on, carmen_vector_3D_t map_center, double square_size, IplImage *img, double robot_wheel_radius);
void draw_remission_map_image(carmen_vector_3D_t gps_position_at_turn_on, carmen_vector_3D_t map_center, double square_size, IplImage *img, double robot_wheel_radius);
void draw_localize_image(bool draw_image_base, carmen_vector_3D_t gps_position_at_turn_on, carmen_pose_3D_t pose, char *image, int width, int height, int square_size);
void draw_tracking_moving_objects(moving_objects_tracking_t *moving_objects_tracking, int current_num_point_clouds,
		carmen_vector_3D_t offset, CarDrawer *car_drawer, int draw_particles_flag);
void draw_ldmrs_objects(carmen_laser_ldmrs_object *ldmrs_objects_tracking, int num_ldmrs_objects, double min_velocity, CarDrawer *car_drawer);
void saveScreenshotToFile(std::string filename, int windowWidth, int windowHeight);
void cleanTexture();
#ifdef __cplusplus
}
#endif

#endif
