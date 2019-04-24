#include <carmen/carmen.h>
#include <carmen/minoru_messages.h>

void
carmen_minoru_subscribe_stereoimage(carmen_minoru_stereoimage_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_MINORU_STEREOIMAGE_NAME,
                           CARMEN_MINORU_STEREOIMAGE_FMT,
                           message, sizeof(carmen_minoru_stereoimage_message),
			   handler, subscribe_how);
}


void
carmen_minoru_unsubscribe_stereoimage(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_MINORU_STEREOIMAGE_NAME, handler);
}


void
carmen_minoru_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_MINORU_STEREOIMAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_MINORU_STEREOIMAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_MINORU_STEREOIMAGE_NAME);
}

void
carmen_minoru_define_bumblebee_fake_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME, IPC_VARIABLE_LENGTH, CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME);
}

IPC_RETURN_TYPE
carmen_minoru_publish_message(
		const carmen_minoru_stereoimage_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_MINORU_STEREOIMAGE_NAME, (carmen_minoru_stereoimage_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MINORU_STEREOIMAGE_NAME);

	return err;
}

IPC_RETURN_TYPE
carmen_minoru_publish_bumblebee_fake_message(
		const carmen_minoru_stereoimage_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME, (carmen_minoru_stereoimage_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME);

	return err;
}
