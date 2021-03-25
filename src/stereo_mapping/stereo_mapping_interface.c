#include <carmen/carmen.h>
#include <carmen/stereo_mapping_messages.h>
#include <carmen/stereo_mapping_interface.h>

char* carmen_stereo_mapping_get_messagename(int camera) {
  switch(camera)
  {
  case 1:
    return CARMEN_STEREO_MAPPING_MSG_0_NAME;

  case 2:
    return CARMEN_STEREO_MAPPING_MSG_1_NAME;
  }

  return CARMEN_STEREO_MAPPING_MSG_0_NAME;
}


void
carmen_stereo_mapping_subscribe_message(carmen_stereo_mapping_message *message,
    carmen_handler_t handler,
    carmen_subscribe_t subscribe_how,
    int camera)
{
  char* msg_name = carmen_stereo_mapping_get_messagename(camera);

  carmen_subscribe_message(msg_name,
      CARMEN_STEREO_MAPPING_FMT,
      message, sizeof(carmen_stereo_mapping_message), handler, subscribe_how);
}

void
carmen_stereo_mapping_unsubscribe_message(carmen_handler_t handler, int camera)
{
  char* msg_name = carmen_stereo_mapping_get_messagename(camera);

  carmen_unsubscribe_message(msg_name, handler);
}


IPC_RETURN_TYPE
carmen_stereo_mapping_define_messages(int camera)
{
  IPC_RETURN_TYPE err;
  char *stereo_mapping_msg_name = carmen_stereo_mapping_get_messagename(camera);

  err = IPC_defineMsg(stereo_mapping_msg_name, IPC_VARIABLE_LENGTH, CARMEN_STEREO_MAPPING_FMT);
  carmen_test_ipc_exit(err, "Could not define", stereo_mapping_msg_name);

  return err;
}
