#include <carmen/carmen.h>
#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>
#include "visual_tracker_util.h"

#include <Detector.h>

static carmen_visual_tracker_output_message message_output;

Detector *m_tld = NULL;

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("Visual Tracker: disconnected.\n");
    exit(0);
  }
}

void
read_parameters(int argc, char** argv)
{
	argc = argc;
	argv = argv;
}

void
publish_visual_tracker_output_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME, &message_output);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME);
}

void
visual_tracker_train_message_handler(carmen_visual_tracker_train_message* message)
{
	IplImage *frame = cvCreateImage(cvSize(message->width, message->height), IPL_DEPTH_8U, message->channels);

	copy_RGB_image_to_BGR_image(message->image, frame, message->channels);

	TLDRect box(message->rect.x, message->rect.y, message->rect.width, message->rect.height);

	if (m_tld)
	{
		free(m_tld);
		m_tld = NULL;
	}
	m_tld = new Detector();
	if (m_tld)
	{
		m_tld->init(frame, &box);
		m_tld->signalBoundingBox(true);
		m_tld->signalConfidence(true);
		carmen_warn("[%d; %.1f; %.1f]\n", 0,
				box.x()+box.width()/2.0, box.y()+box.height()/2.0);
	}
	cvReleaseImage(&frame);
}

void
visual_tracker_test_message_handler(carmen_visual_tracker_test_message* message)
{
	IplImage *frame = cvCreateImage(cvSize(message->width, message->height), IPL_DEPTH_8U, message->channels);

	copy_RGB_image_to_BGR_image(message->image, frame, message->channels);

	if (m_tld && m_tld->initialized())
	{
		static int fn = 0;
		m_tld->process(frame);

		message_output.confidence = m_tld->getOutputConfidence();

		TLDRect box = m_tld->getOutputRect();

		message_output.rect.x = box.x();
		message_output.rect.y = box.y();
		message_output.rect.width = box.width();
		message_output.rect.height = box.height();

		carmen_warn("[%d; %.1f; %.1f]\n", ++fn,
				box.x()+box.width()/2.0, box.y()+box.height()/2.0);

		message_output.host = message->host;
		message_output.timestamp = message->timestamp;

		publish_visual_tracker_output_message();
	}
	cvReleaseImage(&frame);
}

void
carmen_visual_tracker_subscribe_messages()
{
	carmen_visual_tracker_subscribe_train(NULL, (carmen_handler_t)visual_tracker_train_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_visual_tracker_subscribe_test(NULL, (carmen_handler_t)visual_tracker_test_message_handler, CARMEN_SUBSCRIBE_ALL);
}

void
carmen_visual_tracker_define_messages()
{
	carmen_visual_tracker_define_message_output();
}


int
main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);

  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_module);

  read_parameters(argc, argv);

  carmen_visual_tracker_define_messages();

  carmen_visual_tracker_subscribe_messages();

  carmen_ipc_dispatch();

  return (0);
}
