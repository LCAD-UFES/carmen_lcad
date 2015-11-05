#include <carmen/ultrasonic_filter_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_ultrasonic_sonar_sensor_subscribe(carmen_ultrasonic_sonar_sensor_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_ultrasonic_sonar_sensor_unsubscribe(carmen_handler_t handler);

void
ipc_ultrasonic_sonar_sensor_subscribe(ipc_ultrasonic_sensor_message_t *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
ipc_ultrasonic_sonar_sensor_unsubscribe(carmen_handler_t handler);

void
carmen_ultrasonic_sonar_sensor_define_messages();

#ifdef __cplusplus
}
#endif


// @}

