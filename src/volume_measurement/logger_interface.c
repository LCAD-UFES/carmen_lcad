#include <carmen/carmen.h>
#include <carmen/logger_messages.h>

void
carmen_logger_comment(char *text)
{
  IPC_RETURN_TYPE err;

  static carmen_logger_comment_message msg;
  static int first = 1;
  
  if(first) {
  	 msg.host = carmen_get_host();
    err = IPC_defineMsg(CARMEN_LOGGER_COMMENT_NAME, IPC_VARIABLE_LENGTH, 
			CARMEN_LOGGER_COMMENT_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_LOGGER_COMMENT_NAME);
    first = 0;
  }
  msg.text = text;
  msg.timestamp = carmen_get_time();
  
  err = IPC_publishData(CARMEN_LOGGER_COMMENT_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_LOGGER_COMMENT_NAME);
}

void carmen_logger_subscribe_comment_message(carmen_logger_comment_message *msg, 
					     carmen_handler_t handler,
					     carmen_subscribe_t subscribe_how )
{
  carmen_subscribe_message(CARMEN_LOGGER_COMMENT_NAME,
			   CARMEN_LOGGER_COMMENT_FMT,
			   msg, sizeof(carmen_logger_comment_message),
			   handler, subscribe_how);
}	
