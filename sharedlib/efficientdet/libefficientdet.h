#ifndef LIBEFFICIENTDET_H
#define LIBEFFICIENTDET_H

#include <vector>
#include <carmen/carmen_darknet_interface.hpp>
/*
darnet_interface to use struct bbox_t
struct bbox_t
{
	unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;					// confidence - probability that the object was found correctly
	unsigned int obj_id;		// class of object - from range [0, classes-1]
	unsigned int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
};*/

void
initialize_Efficientdet(float min_score);

std::vector<bbox_t>
run_EfficientDet(unsigned char *image, int width, int height);

#endif
