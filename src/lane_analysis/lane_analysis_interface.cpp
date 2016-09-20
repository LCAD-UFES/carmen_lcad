#include <carmen/carmen.h>
#include <carmen/lane_analysis_messages.h>
#include <carmen/lane_analysis_interface.h>
#include <iostream>


// ==============================================================
// UNSUBSCRIBES
void carmen_elas_lane_analysis_unsubscribe(carmen_handler_t handler) {
	carmen_unsubscribe_message((char *)CARMEN_ELAS_LANE_ANALYSIS_NAME, handler);
}


// ==============================================================
// SUBSCRIBES
void carmen_elas_lane_analysis_subscribe(carmen_elas_lane_analysis_message * message, carmen_handler_t handler, carmen_subscribe_t subscribe_how) {
	carmen_subscribe_message((char *)CARMEN_ELAS_LANE_ANALYSIS_NAME, (char *)CARMEN_ELAS_LANE_ANALYSIS_FMT,
			message, sizeof(carmen_elas_lane_analysis_message),
			handler, subscribe_how);
}

// ==============================================================
// PUBLISHES
IPC_RETURN_TYPE carmen_elas_lane_analysis_publish_message(carmen_elas_lane_analysis_message * message) {
	IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_ELAS_LANE_ANALYSIS_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish!", CARMEN_ELAS_LANE_ANALYSIS_NAME);
    return err;
}
