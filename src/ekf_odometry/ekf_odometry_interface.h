#include <carmen/ekf_odometry_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_ekf_odometry_subscribe_odometry_message(carmen_ekf_odometry_odometry_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_ekf_odometry_unsubscribe_odometry_message(carmen_handler_t handler);

void
carmen_ekf_odometry_define_messages();

#ifdef __cplusplus
}
#endif


// @}

