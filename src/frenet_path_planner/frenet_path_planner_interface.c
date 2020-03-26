#include <carmen/carmen.h>
#include <carmen/frenet_path_planner_interface.h>


void
carmen_frenet_path_planner_define_messages()
{
    IPC_RETURN_TYPE err;

    err = IPC_defineMsg(CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME);

    err = IPC_defineMsg(CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_NAME);
}


void
carmen_frenet_path_planner_subscribe_set_of_paths_message(carmen_frenet_path_planner_set_of_paths *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME, CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_FMT,
                             message, sizeof(carmen_frenet_path_planner_set_of_paths), handler, subscribe_how);
}


void
carmen_frenet_path_planner_unsubscribe_set_of_paths_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME, handler);
}


void
carmen_frenet_path_planner_publish_set_of_paths_message(carmen_frenet_path_planner_set_of_paths *message)
{
    IPC_RETURN_TYPE err;

    err = IPC_publishData(CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME);
}


void
carmen_frenet_path_planner_subscribe_selected_path_message(carmen_frenet_path_planner_selected_path *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_NAME, CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_FMT,
                             message, sizeof(carmen_frenet_path_planner_selected_path), handler, subscribe_how);
}


void
carmen_frenet_path_planner_unsubscribe_selected_path_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_NAME, handler);
}


void
carmen_frenet_path_planner_publish_selected_path_message(carmen_frenet_path_planner_selected_path *message)
{
    IPC_RETURN_TYPE err;

    err = IPC_publishData(CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_NAME);
}
