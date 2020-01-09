#ifndef SYMOTHA_DRAWER_H_
#define SYMOTHA_DRAWER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct symotha_parameters symotha_parameters;

symotha_parameters* create_SYMOTHA_drawer(int argc, char **argv);
void destroy_trajectory_drawer(trajectory_drawer* t_drawer);
void add_trajectory_message(trajectory_drawer* t_drawer, carmen_navigator_ackerman_plan_message *message);
void add_goal_list_message(trajectory_drawer* t_drawer, carmen_behavior_selector_goal_list_message *goals);
void draw_trajectory(trajectory_drawer* t_drawer, carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose);

#ifdef __cplusplus
}
#endif

#endif
