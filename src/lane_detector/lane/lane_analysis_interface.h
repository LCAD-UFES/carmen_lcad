#include <carmen/lane_analysis_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

// UNSUBSCRIBES
void carmen_elas_lane_analysis_unsubscribe(carmen_handler_t handler);

// SUBSCRIBES
void carmen_elas_lane_analysis_subscribe(carmen_elas_lane_analysis_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

// PUBLISHES
IPC_RETURN_TYPE carmen_elas_lane_analysis_publish_message(carmen_elas_lane_analysis_message *message);

#ifdef __cplusplus
}
#endif
