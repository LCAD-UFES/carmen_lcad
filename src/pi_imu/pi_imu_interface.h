#include <carmen/pi_imu_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

// UNSUBSCRIBES
void carmen_pi_imu_unsubscribe(carmen_handler_t handler);

// SUBSCRIBES
void carmen_pi_imu_subscribe(carmen_pi_imu_message_t *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

// PUBLISHES
IPC_RETURN_TYPE carmen_pi_imu_publish_message(carmen_pi_imu_message_t *message);

#ifdef __cplusplus
}
#endif
