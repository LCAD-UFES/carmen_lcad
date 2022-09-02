#ifndef MODEL_PREDICTIVE_PLANNER_INTERFACE_H_
#define MODEL_PREDICTIVE_PLANNER_INTERFACE_H_

#include "model_predictive_planner_message.h"

#ifdef __cplusplus
extern "C" {
#endif

//Defines
void carmen_model_predictive_planner_define_motion_plan_message();
void carmen_model_predictive_planner_define_all();

//Subscribers
void carmen_model_predictive_planner_subscribe_motion_plan_message(carmen_model_predictive_planner_motion_plan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

//Unsubscribers
void carmen_model_predictive_planner_unsubscribe_motion_plan_message(carmen_handler_t handler);


//Publishers
void carmen_model_predictive_planner_publish_motion_plan_message(carmen_robot_and_trailers_traj_point_t *plan, int plan_length);

#ifdef __cplusplus
}
#endif

#endif /* RRT_PLANNER_INTERFACE_H_ */
