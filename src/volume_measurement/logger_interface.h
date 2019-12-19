#ifndef LOGGER_INTERFACE_H_
#define LOGGER_INTERFACE_H_

#include "logger_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void carmen_logger_subscribe_comment_message(carmen_logger_comment_message *msg, 
					     carmen_handler_t t_handler,
					     carmen_subscribe_t subscribe_how );
  
void carmen_logger_comment(char *text);


#ifdef __cplusplus
}
#endif

#endif 
