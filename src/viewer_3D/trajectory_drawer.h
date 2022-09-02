#ifndef TRAJECTORY_DRAWER_H_
#define TRAJECTORY_DRAWER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _trajectory_drawer
{
	double r;
	double g;
	double b;

	carmen_robot_and_trailers_pose_t *path;
	carmen_vector_3D_t *path_segment_color;
	int path_size;
	double path_point_size;

	carmen_pose_3D_t *goals;
	int goals_size;

	carmen_vector_3D_t robot_size;
	double distance_between_rear_car_and_rear_wheels;
	double availability_timestamp;

	double persistence_time;

	carmen_semi_trailer_config_t semi_trailer_config;
} trajectory_drawer;

trajectory_drawer *create_trajectory_drawer(double r, double g, double b, carmen_vector_3D_t robot_size,
		double distance_between_rear_car_and_rear_wheels, carmen_semi_trailer_config_t semi_trailer_config, double path_point_size = 5.0, double persistence_time = 0.1);
void destroy_trajectory_drawer(trajectory_drawer *t_drawer);
void add_trajectory_message(trajectory_drawer *t_drawer, carmen_navigator_ackerman_plan_message *message);
void add_base_ackerman_trajectory_message(trajectory_drawer *t_drawer, carmen_base_ackerman_motion_command_message *message);
void add_rrt_trajectory_message(trajectory_drawer *t_drawer, rrt_path_message *message);
void add_path_goals_and_annotations_message(trajectory_drawer *t_drawer, carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message, carmen_vector_3D_t robot_size, double distance_between_rear_car_and_rear_wheels);
void draw_trajectory(trajectory_drawer *t_drawer, carmen_vector_3D_t offset, int draw_waypoints_flag, int draw_robot_waypoints_flag, int semi_trailer_engaged);

#ifdef __cplusplus
}
#endif

#endif
