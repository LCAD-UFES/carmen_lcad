#include <carmen/lane_analysis_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

// UNSUBSCRIBES
void carmen_elas_lane_estimation_unsubscribe(carmen_handler_t handler);
void carmen_elas_lane_markings_type_unsubscribe(carmen_handler_t handler);
void carmen_elas_adjacent_lanes_unsubscribe(carmen_handler_t handler);

// SUBSCRIBES
void carmen_elas_lane_estimation_subscribe(carmen_elas_lane_estimation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
void carmen_elas_lane_markings_type_subscribe(carmen_elas_lane_markings_type_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
void carmen_elas_adjacent_lanes_subscribe(carmen_elas_adjacent_lanes_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

// PUBLISHES
IPC_RETURN_TYPE carmen_elas_lane_estimation_publish_message(carmen_elas_lane_estimation_message *message);
IPC_RETURN_TYPE carmen_elas_lane_markings_type_publish_message(carmen_elas_lane_markings_type_message *message);
IPC_RETURN_TYPE carmen_elas_adjacent_lanes_publish_message(carmen_elas_adjacent_lanes_message *message);

#ifdef __cplusplus
}
#endif
