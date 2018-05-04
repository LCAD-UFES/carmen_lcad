#include <carmen/lane_detector_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

// UNSUBSCRIBES
void carmen_elas_lane_analysis_unsubscribe(carmen_handler_t handler);

// SUBSCRIBES
void carmen_elas_lane_analysis_subscribe(carmen_lane_detector_lane_message_t *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

// PUBLISHES
IPC_RETURN_TYPE carmen_lane_publish_message(carmen_lane_detector_lane_message_t *message);

#ifdef __cplusplus
}
#endif
