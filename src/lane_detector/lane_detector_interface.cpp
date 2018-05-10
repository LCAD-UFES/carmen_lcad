#include <carmen/carmen.h>
#include <carmen/lane_detector_messages.h>
#include <carmen/lane_detector_interface.h>
#include <iostream>

// ==============================================================
// UNSUBSCRIBES
void
carmen_elas_lane_analysis_unsubscribe(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *)CARMEN_LANE_NAME, handler);
}

// ==============================================================
// SUBSCRIBES
void
carmen_elas_lane_analysis_subscribe(carmen_lane_detector_lane_message_t *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_LANE_NAME, (char *)CARMEN_LANE_FMT,
			message, sizeof(carmen_lane_detector_lane_message_t),
			handler, subscribe_how);
}

// ==============================================================
// PUBLISHES
IPC_RETURN_TYPE
carmen_lane_publish_message(carmen_lane_detector_lane_message_t *message)
{
	IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_LANE_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish!", CARMEN_LANE_NAME);
    return err;
}
