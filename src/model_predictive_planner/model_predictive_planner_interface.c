#include <carmen/carmen.h>
#include "model_predictive_planner_interface.h"

//Defines
void
carmen_model_predictive_planner_define_motion_plan_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME);
}

void
carmen_model_predictive_planner_define_all()
{
	carmen_model_predictive_planner_define_motion_plan_message();
}

//Subscribbers
void
carmen_model_predictive_planner_subscribe_motion_plan_message(carmen_model_predictive_planner_motion_plan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_FMT,
                             message, sizeof (carmen_model_predictive_planner_motion_plan_message), handler, subscribe_how);
}

//Unsubscribers
void
carmen_model_predictive_planner_unsubscribe_motion_plan_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, handler);
}

//Publishers
void
carmen_model_predictive_planner_publish_motion_plan_message(carmen_robot_and_trailers_traj_point_t *plan, int plan_length)
{
    IPC_RETURN_TYPE err;
	carmen_model_predictive_planner_motion_plan_message motion_plan_msg;

	static int first_time = 1;

	if (first_time)
	{
		first_time = 0;
		carmen_model_predictive_planner_define_motion_plan_message();
	}

	motion_plan_msg.plan = plan;
	motion_plan_msg.plan_length = plan_length;
	motion_plan_msg.timestamp = carmen_get_time();
	motion_plan_msg.host = carmen_get_host();

    err = IPC_publishData(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, &motion_plan_msg);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME);
}
