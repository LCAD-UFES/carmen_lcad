#include <carmen/parking_assistant_messages.h>

#ifdef __cplusplus
extern "C" 
{
#endif

void
carmen_parking_assistant_subscribe_goal(carmen_parking_assistant_goal_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_parking_assistant_unsubscribe_goal(carmen_handler_t handler);

void
carmen_parking_assistant_subscribe_parking_space(carmen_parking_assistant_parking_space_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_parking_assistant_unsubscribe_parking_space(carmen_handler_t handler);

void
carmen_parking_assistant_define_messages();

#ifdef __cplusplus
}
#endif


// @}

