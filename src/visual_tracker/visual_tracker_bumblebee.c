#include <carmen/carmen.h>
#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>

static carmen_visual_tracker_test_message m_test;

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("Visual Tracker Bumblebee: disconnected.\n");
    exit(0);
  }
}

void
read_parameters(int argc, char** argv)
{
}

void
publish_visual_tracker_test_message(carmen_visual_tracker_test_message *message)
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME);
}

void
bumblebee_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
	m_test.timestamp = message->timestamp;
	m_test.host = message->host;

	m_test.channels = 3;
	m_test.height = message->height;
	m_test.width = message->width;

	m_test.size = m_test.height * m_test.width * m_test.channels;

	if (m_test.size != message->image_size)
		carmen_die("Wrong image size!");

	m_test.image = message->raw_left;

	publish_visual_tracker_test_message(&m_test);
}

void
carmen_visual_tracker_subscribe_bumblebee_messages_by_camera(int camera)
{
	switch(camera)
	{
		case 1:
			carmen_bumblebee_basic_subscribe_stereoimage1(NULL, (carmen_handler_t)bumblebee_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
		case 2:
			carmen_bumblebee_basic_subscribe_stereoimage2(NULL, (carmen_handler_t)bumblebee_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
		default:
			carmen_die("Unknown command line parameter %d . \nUsage:\n stereo <camera_number>", camera);
	}
}

void
carmen_visual_tracker_define_messages()
{
	carmen_visual_tracker_define_message_test();
}

int
main(int argc, char **argv)
{
	if (argc != 2)
		carmen_die("%s: Wrong number of parameters. stereo requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number> \n", argv[0], argc-1, argv[0]);

	int camera = atoi(argv[1]);

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	read_parameters(argc, argv);

	carmen_visual_tracker_define_messages();

	carmen_visual_tracker_subscribe_bumblebee_messages_by_camera(camera);

	carmen_ipc_dispatch();

	return (0);
}
