#include <carmen/carmen.h>
#include <carmen/yolo_detector_interface.h>

void
yolo_detector_subscribe_yolo_detector_message(yolo_detector_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(YOLO_DETECTOR_MESSAGE_NAME, YOLO_DETECTOR_MESSAGE_FMT,
                             message, sizeof (carmen_route_planner_road_network_message), handler, subscribe_how);
}


void
yolo_detector_unsubscribe_yolo_detector_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(YOLO_DETECTOR_MESSAGE_NAME, handler);
}


void
yolo_detector_define_messages()
{
    IPC_RETURN_TYPE err;

    err = IPC_defineMsg(YOLO_DETECTOR_MESSAGE_NAME, IPC_VARIABLE_LENGTH, YOLO_DETECTOR_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", YOLO_DETECTOR_MESSAGE_NAME);
}


void
yolo_detector_publish_yolo_detector_message(yolo_detector_message *yolo_detector_yolo_detector_message)
{
    IPC_RETURN_TYPE err;

    err = IPC_publishData(YOLO_DETECTOR_MESSAGE_NAME, yolo_detector_yolo_detector_message);
    carmen_test_ipc_exit(err, "Could not publish", YOLO_DETECTOR_MESSAGE_NAME);
}