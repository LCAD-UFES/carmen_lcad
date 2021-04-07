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

	carmen_vector_3D_t *path;
	carmen_vector_3D_t *path_segment_color;
	int path_size;

	carmen_pose_3D_t *goals;
	int goals_size;

	carmen_vector_3D_t car_size;
} trajectory_drawer;

trajectory_drawer* create_trajectory_drawer(double r, double g, double b);
void destroy_trajectory_drawer(trajectory_drawer *t_drawer);
void add_trajectory_message(trajectory_drawer *t_drawer, carmen_navigator_ackerman_plan_message *message);
void add_path_goals_and_annotations_message(trajectory_drawer *t_drawer, carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message);
void draw_trajectory(trajectory_drawer *t_drawer, carmen_vector_3D_t offset);

#ifdef __cplusplus
}
#endif

#endif
