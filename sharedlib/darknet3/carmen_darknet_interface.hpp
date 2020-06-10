
#ifndef __DARKNET_INTERFACE_H__
#define __DARKNET_INTERFACE_H__

#include <vector>


struct bbox_t
{
	unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;					// confidence - probability that the object was found correctly
	unsigned int obj_id;		// class of object - from range [0, classes-1]
	unsigned int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
};


struct image_t
{
	int h;						// height
	int w;						// width
	int c;						// number of chanels (3 - for RGB)
	float *data;				// pointer to the image data
};

char **
get_classes_names(char *classes_names_file);

void*
load_yolo_network(char *cfg, char *weights, int clear);

std::vector<bbox_t>
run_YOLO(unsigned char *data, int c, int w, int h, void *net_config, char **classes_names, float threshold, float hier_thresh);


#endif
