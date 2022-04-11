#ifndef USER_APP_SERVER_INTERFACE_H
#define USER_APP_SERVER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include "user_app_server_messages.h"


void
carmen_user_app_server_subscribe_execute_mission_message(carmen_user_app_server_execute_mission_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_user_app_server_subscribe_update_mission_message(carmen_user_app_server_update_mission_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_user_app_server_publish_execute_mission_message(const char *mission, double timestamp);

void
carmen_user_app_server_publish_update_mission_message(int mission_uptate_code, double timestamp);


#ifdef __cplusplus
}
#endif

#endif
