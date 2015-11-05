/*
 * Copyright � 2011 Paul Nader 
 *
 * This file is part of QOpenTLD.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "visualControl.h"
#include <carmen/carmen.h>
#include <carmen/visual_tracker_util.h>

VisualControl *camshift = NULL;

void bumblebee_message_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
	static int fn = 1;
	char name[255];
	CvRect rect_translate = cvRect(160, 90, 10, 12);
	CvRect rect_rotate = cvRect(177, 84, 10, 12);
	CvRect rect_scale = cvRect(162, 94, 10, 12);
//	CvRect *rect = NULL;
	CvRect *rect = &rect_translate;
	sprintf(name, "camshift-translate-%05d.jpg", fn++);
//	CvRect *rect = &rect_rotate;
//	sprintf(name, "camshift-rotate-%05d.jpg", fn++);
//	CvRect *rect = &rect_scale;
//	sprintf(name, "camshift-scale-%05d.jpg", fn++);

	CvSize size = cvSize(message->width, message->height);
	IplImage *frame_left = cvCreateImage(size, IPL_DEPTH_8U, 3);
	copy_RGB_image_to_BGR_image(message->raw_left, frame_left, 3);

	cvSaveImage(name, frame_left, 0);

	if (camshift)
		camshift->trackVisualControl(frame_left);
	else
	{
		camshift = new VisualControl(frame_left->width, frame_left->height);
		camshift->startVisualControl(frame_left, rect);
	}
	carmen_warn("%.1f,%.1f,%.1f,%.1f\n",
			camshift->getRectCoordLeftUp().x,
			camshift->getRectCoordLeftUp().y,
			camshift->getRectCoordRightDown().x,
			camshift->getRectCoordRightDown().y);
}

void shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("Visual Tracker View: disconnected.\n");
		exit(0);
	}
}

int main(int argc, char **argv)
{
	/* Do Carmen Initialization*/
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Subscribe to sensor messages */
	carmen_bumblebee_basic_subscribe_stereoimage1(NULL, (carmen_handler_t) bumblebee_message_handler, CARMEN_SUBSCRIBE_ALL);

	/* Main loop */
	carmen_ipc_dispatch();

	return 0;
}
