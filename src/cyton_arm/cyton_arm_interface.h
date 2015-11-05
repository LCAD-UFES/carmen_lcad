#ifndef CARMEN_CYTON_ARM_INTERFACE_H
#define CARMEN_CYTON_ARM_INTERFACE_H

#include <carmen/cyton_arm_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_cyton_arm_subscribe_joint_command_message(carmen_cyton_arm_joint_command_message *arm_command,
												carmen_handler_t handler,
												carmen_subscribe_t subscribe_how);

void
carmen_cyton_arm_unsubscribe_joint_command_message(carmen_handler_t handler);

void
carmen_cyton_arm_subscribe_point_command_message(carmen_cyton_arm_point_command_message *arm_command,
												carmen_handler_t handler,
												carmen_subscribe_t subscribe_how);

void
carmen_cyton_arm_unsubscribe_point_command_message(carmen_handler_t handler);

void
carmen_cyton_arm_subscribe_state_message(carmen_cyton_arm_state_message *state,
										carmen_handler_t handler,
										carmen_subscribe_t subscribe_how);

void
carmen_cyton_arm_unsubscribe_state_message(carmen_handler_t handler);

void
carmen_cyton_arm_subscribe_reset_message(carmen_cyton_arm_reset_message *reset,
										carmen_handler_t handler,
										carmen_subscribe_t subscribe_how);

void
carmen_cyton_arm_unsubscribe_reset_message(carmen_handler_t handler);


#ifdef __cplusplus
}
#endif

#endif
