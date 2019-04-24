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
#include "../camera_hw_interface.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

static carmen_camera_image_t buffer_image;
static CvCapture *capture = 0;
static int camera = 0;

void
copy_opencv_image_to_carmen_image(const IplImage *opencv, carmen_camera_image_t *carmen)
{
	int i;
	for(i = 0; i < (opencv->height * opencv->width); i++)
	{
		carmen->image[3 * i + 2] = opencv->imageData[3 * i];
		carmen->image[3 * i + 1] = opencv->imageData[3 * i + 1];
		carmen->image[3 * i] = opencv->imageData[3 * i + 2];
	}
}

void read_parameters(int argc, char **argv)
{
	if (argc != 2)
		carmen_die("%s: Wrong number of parameters. %s requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number>\n", argv[0], argv[0], argc-1, argv[0]);

	camera = atoi(argv[1]);
}

carmen_camera_image_t *
carmen_camera_start(int argc, char **argv)
{
	carmen_camera_image_t *image = NULL;

	read_parameters(argc, argv);

	capture = cvCaptureFromCAM( camera );
	if ( capture )
	{
		IplImage* frame = cvQueryFrame( capture );
		if ( frame )
		{
			buffer_image.width = frame->width;
			buffer_image.height = frame->height;
			buffer_image.bytes_per_pixel = frame->nChannels;

			buffer_image.image_size =
					buffer_image.width*
					buffer_image.height*
					buffer_image.bytes_per_pixel;

			buffer_image.is_new = 0;
			buffer_image.timestamp = 0;

			buffer_image.image = (char *)calloc(buffer_image.image_size, sizeof(char));
			carmen_test_alloc(buffer_image.image);

			copy_opencv_image_to_carmen_image(frame, &buffer_image);

			image = (carmen_camera_image_t *)calloc(1, sizeof(carmen_camera_image_t));
			memcpy(image, &buffer_image, sizeof(carmen_camera_image_t));
			image->image = (char *)calloc(image->image_size, sizeof(char));
			carmen_test_alloc(image->image);
			memcpy(image->image, buffer_image.image, sizeof(char)*image->image_size);
		}
		else
			carmen_die("ERROR: camera frame is null...\n");
	}
	else
		carmen_die("ERROR: camera capture is NULL \n" );

	return image;
}

void carmen_camera_shutdown(void)
{
	cvReleaseCapture( &capture );
}

void carmen_camera_grab_image(carmen_camera_image_t *image)
{
	IplImage* frame = cvQueryFrame( capture );
	if ( frame )
	{
		copy_opencv_image_to_carmen_image(frame, &buffer_image);
		memcpy(image->image, buffer_image.image, sizeof(char)*image->image_size);
		image->timestamp = carmen_get_time();
		image->is_new = 1;
	}
	else
		image->is_new = 0;
}

