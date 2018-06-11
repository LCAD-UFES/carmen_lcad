/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include "camera_hw_interface.h"
#include "camera_messages.h"


void
shutdown_module(int signo __attribute__ ((unused)))
{
	carmen_camera_shutdown();
	exit(0);
}


void
carmen_camera_publish_image_message(carmen_camera_image_t *image)
{
	static carmen_camera_image_message msg;

	IPC_RETURN_TYPE err;

	msg.host = carmen_get_host();
	msg.timestamp = image->timestamp;

	msg.image_size = image->image_size;
	msg.width = image->width;
	msg.height = image->height;
	msg.bytes_per_pixel = image->bytes_per_pixel;
	msg.image = image->image;

	err = IPC_publishData(CARMEN_CAMERA_IMAGE_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_CAMERA_IMAGE_NAME);
}


int
main(int argc, char **argv)
{
	carmen_camera_image_t *image;
	IPC_RETURN_TYPE err;
	double interframe_sleep = 0.05;
	int param_err, fps;

	/* connect to IPC server */
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	err = IPC_defineMsg(CARMEN_CAMERA_IMAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_CAMERA_IMAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_CAMERA_IMAGE_NAME);

	carmen_param_allow_unfound_variables(0);
	param_err = carmen_param_get_int("camera_fps", &fps, NULL);

	if (param_err < 0 || fps <= 0)
		carmen_die("Could not find valid parameter in carmen.ini file: camera_fps\n");

	interframe_sleep = 1.0/(double)fps;
	image = carmen_camera_start(argc, argv);

	if (image == NULL)
		exit(-1);

	signal(SIGINT, shutdown_module);

	while(1)
	{
		carmen_camera_grab_image(image);
		if(image->is_new)
		{
			carmen_camera_publish_image_message(image);
			carmen_warn("c");
			image->is_new = 0;
		}

		// We are rate-controlling the camera.

		carmen_publish_heartbeat("camera_daemon");
		carmen_ipc_sleep(interframe_sleep);
	}
	return 0;
}
