#include <carmen/visual_odometry_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_visual_odometry_subscribe_pose6d_message(carmen_visual_odometry_pose6d_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_visual_odometry_unsubscribe_pose6d_message(carmen_handler_t handler);

void
carmen_visual_odometry_subscribe_image_message(carmen_visual_odometry_image_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_visual_odometry_unsubscribe_image_message(carmen_handler_t handler);

void
carmen_visual_odometry_define_messages();

#ifdef __cplusplus
}
#endif


// @}

