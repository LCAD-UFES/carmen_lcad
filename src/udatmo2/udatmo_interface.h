#ifndef _CARMEN_UDATMO_INTERFACE_H_
#define _CARMEN_UDATMO_INTERFACE_H_


#ifdef __cplusplus
extern "C"
{
#endif


#include "udatmo_messages.h"


carmen_udatmo_moving_obstacles_message *carmen_udatmo_new_moving_obstacles_message(size_t size);


void carmen_udatmo_init_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, size_t size);


void carmen_udatmo_clear_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message);


#define carmen_udatmo_delete_moving_obstacles_message(MESSAGE) \
	if (MESSAGE != NULL) do {carmen_udatmo_clear_moving_obstacles_message(MESSAGE); MESSAGE = NULL;} while (0)


void carmen_udatmo_publish_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message);


void carmen_udatmo_subscribe_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message,
													  void (*handler)(carmen_udatmo_moving_obstacles_message*),
													  carmen_subscribe_t subscribe_how);


void carmen_udatmo_unsubscribe_moving_obstacles_message(carmen_handler_t handler);


/**
 * @brief Write the moving obstacles at the given message to the given output for display, starting at the given offset.
 *
 * @param message Moving obstacle message to be displayed.
 * @param offset Offset into the output message at the start of which moving obstacles will be written.
 * @param out Output display message.
 */
void carmen_udatmo_fill_virtual_laser_message(carmen_udatmo_moving_obstacles_message *message, int offset, carmen_mapper_virtual_laser_message *out);


/**
 * @brief Display the given moving obstacles on the viewer GUI.
 *
 * Moving obstacles are displayed by creating a virtual laser message then publishing it.
 */
void carmen_udatmo_display_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, carmen_robot_ackerman_config_t *robot_config);


#ifdef __cplusplus
}
#endif


#endif

// @}
