
#include <carmen/carmen.h>
#include <carmen/rlcontrol_interface.h>

void
carmen_rl_control_subscribe_message(carmen_rl_control_message *message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_RL_CONTROL_NAME, CARMEN_RL_CONTROL_FMT,
			message, sizeof(carmen_rl_control_message), handler, subscribe_how);
}

void
carmen_rl_control_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_RL_CONTROL_NAME, handler);
}


