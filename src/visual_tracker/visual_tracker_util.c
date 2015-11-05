/*
 * visual_tracker_util.c
 *
 *  Created on: 29/11/2011
 *      Author: avelino
 */

#include "visual_tracker_util.h"

void
copy_RGB_image_to_BGR_image(unsigned char *original, IplImage *copy, int nchannels)
{
	int i, j;

	for(i = 0; i < copy->height; i++)
	{
		unsigned char* data = (unsigned char*)copy->imageData + (i*copy->widthStep);
		if(nchannels==3)
			for(j = 0; j < copy->width; j++)
			{
				data[nchannels*j+2] = original[nchannels*(i*copy->width+j)+0];
				data[nchannels*j+1] = original[nchannels*(i*copy->width+j)+1];
				data[nchannels*j+0] = original[nchannels*(i*copy->width+j)+2];
			}
		else
			for(j = 0; j < copy->width; j++)
			{
				data[j] = original[i*copy->width+j];
			}
	}
}


void
copy_BGR_image_to_RGB_image(IplImage *original, unsigned char *copy, int nchannels)
{
	int i, j;

	for (i = 0; i < original->height; i++)
	{
		unsigned char *data = (unsigned char *)original->imageData + (i * original->widthStep);
		if(nchannels==3)
		{
			for(j = 0; j < original->width; j++)
			{
				copy[nchannels*(i*original->width+j)+2] = data[nchannels*j+0];
				copy[nchannels*(i*original->width+j)+1] = data[nchannels*j+1];
				copy[nchannels*(i*original->width+j)+0] = data[nchannels*j+2];
			}
		}
		else
		{
			for(j = 0; j < original->width; j++)
			{
				copy[i*original->width+j] = data[j];
			}
		}
	}
}


