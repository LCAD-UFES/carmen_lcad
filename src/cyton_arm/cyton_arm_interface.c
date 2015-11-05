#include <carmen/carmen.h>
#include <carmen/cyton_arm_messages.h>
#include <carmen/cyton_arm_interface.h>

void
carmen_cyton_arm_subscribe_joint_command_message(carmen_cyton_arm_joint_command_message *arm_command,
												carmen_handler_t handler,
												carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_CYTON_ARM_JOINT_COMMAND_NAME,
							 CARMEN_CYTON_ARM_JOINT_COMMAND_FMT,
							 arm_command, sizeof(carmen_cyton_arm_joint_command_message),
							 handler, subscribe_how);
}

void
carmen_cyton_arm_unsubscribe_joint_command_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_CYTON_ARM_JOINT_COMMAND_NAME, handler);
}

void
carmen_cyton_arm_subscribe_point_command_message(carmen_cyton_arm_point_command_message *arm_command,
												carmen_handler_t handler,
												carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_CYTON_ARM_POINT_COMMAND_NAME,
							 CARMEN_CYTON_ARM_POINT_COMMAND_FMT,
							 arm_command, sizeof(carmen_cyton_arm_point_command_message),
							 handler, subscribe_how);
}

void
carmen_cyton_arm_unsubscribe_point_command_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_CYTON_ARM_POINT_COMMAND_NAME, handler);
}

void
carmen_cyton_arm_subscribe_state_message(carmen_cyton_arm_state_message *state,
										carmen_handler_t handler,
										carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_CYTON_ARM_STATE_NAME,
							 CARMEN_CYTON_ARM_STATE_FMT,
							 state, sizeof(carmen_cyton_arm_state_message),
							 handler, subscribe_how);
}

void
carmen_cyton_arm_unsubscribe_state_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_CYTON_ARM_STATE_NAME, handler);
}

void
carmen_cyton_arm_subscribe_reset_message(carmen_cyton_arm_reset_message *reset,
										carmen_handler_t handler,
										carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_CYTON_ARM_RESET_NAME,
							 CARMEN_DEFAULT_MESSAGE_FMT,
							 reset, sizeof(carmen_cyton_arm_reset_message),
							 handler, subscribe_how);
}

void
carmen_cyton_arm_unsubscribe_reset_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_CYTON_ARM_RESET_NAME, handler);
}
