/*
 * voice_recognition_test.cpp
 *
 *  Created on: 17/10/2012
 *      Author: filipe
 */

#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <carmen/carmen.h>
#include <carmen/voice_recognition_interface.h>


IplImage *voice_command_image = NULL;
int voice_command_image_is_initialized = 0;


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("voice_recognition_test: disconnected.\n");

		exit(0);
	}
}


void
carmen_voice_recognition_message_handler(carmen_voice_recognition_message *message)
{
	printf("%lf new message received\n", message->timestamp);

	if (!voice_command_image_is_initialized)
	{
		voice_command_image = cvCreateImage(cvSize(message->width, message->height), IPL_DEPTH_8U, 3);
		voice_command_image_is_initialized = 1;
	}

	memcpy(voice_command_image->imageData, message->image, message->image_size);

	cvShowImage("voice_command", voice_command_image);
	cvWaitKey(10);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_voice_recognition_subscribe_message(NULL, (carmen_handler_t) carmen_voice_recognition_message_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_ipc_dispatch();
	return 0;
}
