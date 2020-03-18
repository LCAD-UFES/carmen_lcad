#include <carmen/carmen.h>
#include <carmen/frenet_path_planner_interface.h>


void
carmen_frenet_path_planner_subscribe_plan_message(carmen_frenet_path_planner_plan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_NAME, CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_FMT,
                             message, sizeof(carmen_frenet_path_planner_plan_message), handler, subscribe_how);
}


void
carmen_frenet_path_planner_unsubscribe_plan_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_NAME, handler);
}


void
carmen_frenet_path_planner_define_messages()
{
    IPC_RETURN_TYPE err;

    err = IPC_defineMsg(CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_NAME);
}


void
carmen_frenet_path_planner_publish_plan_message(carmen_frenet_path_planner_plan_message *message)
{
    IPC_RETURN_TYPE err;

    err = IPC_publishData(CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_NAME);
}
