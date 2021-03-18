#include "../carmen_darknet_interface.hpp"
#include "data.h"
#include "yolo_v2_class.hpp"    // imported functions for integrate

Detector* detector = NULL;

char **
get_classes_names(char *classes_names_file)
{
	return (get_labels(classes_names_file));
}

void* 
load_yolo_network(char *cfg, char *weights, int clear)
{
	detector = new Detector(cfg, weights);
	return (detector);
}

image 
convert_image_msg_to_darknet_image(unsigned int w, unsigned int h, unsigned char *data)
{
	unsigned int c = 3;    // Number of channels
    image image = make_image(w, h, c);

    if (data == NULL)
    	return image;

    for(unsigned int k = 0; k < c; ++k)
    {
    	for(unsigned int j = 0; j < h; ++j)
    	{
    		for(unsigned int i = 0; i < w; ++i)
    		{
    			int dst_index = i + (w * j) + (w * h * k);
    			int src_index = k + (c * i) + (c * w * j);
    			image.data[dst_index] = (float) (data[src_index] / 255.0);    // Darknet pixels vary from 0 to 1 so we must divide the carmen pixel by its max value (255)
    		}
    	}
    }
    return (image);
} 


std::vector<bbox_t>
run_YOLO(unsigned char *data, int w, int h, float threshold)
{
	image im = convert_image_msg_to_darknet_image(w, h, data);
	image_t img;
    img.c = im.c;
    img.data = im.data;
    img.h = im.h;
    img.w = im.w;
	
	std::vector<bbox_t> bbox_vector = detector->detect(img, threshold);

	free_image(im);
	
	return (bbox_vector);
}