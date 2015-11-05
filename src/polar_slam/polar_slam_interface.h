#include <carmen/polar_slam_messages.h>

#ifdef __cplusplus
extern "C" 
{
#endif

void
carmen_polar_slam_subscribe_message(carmen_polar_slam_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_polar_slam_unsubscribe_message(carmen_handler_t handler);

void
carmen_polar_slam_define_messages();

#ifdef __cplusplus
}
#endif


// @}

