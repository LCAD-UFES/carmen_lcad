
#ifndef _CARMEN_MOVING_OBJECTS3_INTERFACE_H_
#define _CARMEN_MOVING_OBJECTS3_INTERFACE_H_

#ifdef __cplusplus
extern "C" 
{
#endif

	#include <carmen/moving_objects3_messages.h>

	void carmen_subscribe_velodyne_projected_message(carmen_velodyne_projected_on_ground_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void carmen_publish_velodyne_projected_message(carmen_velodyne_projected_on_ground_message *message);
	void carmen_unsubscribe_velodyne_projected_message(carmen_handler_t handler);

	void carmen_subscribe_moving_objects3_particles_message(carmen_moving_objects3_particles_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void carmen_publish_moving_objects3_particles_message(carmen_moving_objects3_particles_message *message);
	void carmen_unsubscribe_moving_objects3_particles_message(carmen_handler_t handler);

	void carmen_subscribe_moving_objects3_virtual_scan_message(carmen_moving_objects3_virtual_scan_message *message,
			carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void carmen_publish_moving_objects3_virtual_scan_message(carmen_moving_objects3_virtual_scan_message *message);
	void carmen_unsubscribe_moving_objects3_virtual_scan_message(carmen_handler_t handler);

	void carmen_moving_objects3_define_messages();

#ifdef __cplusplus
}
#endif
#endif
// @}

