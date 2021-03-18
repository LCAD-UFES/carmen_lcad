#include "../carmen_darknet_interface.hpp"
#include "data.h"
#include "yolo_v2_class.hpp"    // imported functions from DLL

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
    std::cout << "Net1 width: " << detector->get_net_width() << " height:" << detector->get_net_height() << std::endl;
	std::cout << "endereço* " << detector << std::endl;
	return (detector);
}


std::vector<bbox_t>
extract_predictions(image img, detection *dets, int num, float thresh, int classes_number, char **classes_names)
{
	int i,j;
	std::vector<bbox_t> bbox_vector;

	for(i = 0; i < num; ++i)
	{
		char labelstr[4096] = {0};
		int obj_id = -1;

		for(j = 0; j < classes_number; ++j)
		{
			if (dets[i].prob[j] > thresh)
			{
				if (obj_id < 0)   //TODO Get the class with the higher probability ????
				{
					strcat(labelstr, classes_names[j]);
					obj_id = j;
				}
				else
				{
					strcat(labelstr, ", ");
					strcat(labelstr, classes_names[j]);
				}
				//printf("%s: %.0f%%\n", classes_names[j], dets[i].prob[j]*100);
			}
		}
		if (obj_id >= 0)
		{
			box b = dets[i].bbox;

			int left  = (b.x - (b.w / 2.0)) * img.w;
			int right = (b.x + (b.w / 2.0)) * img.w;
			int top   = (b.y - (b.h / 2.0)) * img.h;
			int bot   = (b.y + (b.h / 2.0)) * img.h;

			if(left < 0)
				left = 0;
			if(right > img.w-1)
				right = img.w-1;
			if(top < 0)
				top = 0;
			if(bot > img.h-1)
				bot = img.h-1;

			bbox_t bbox;

			bbox.x = left;
			bbox.y = top;
			bbox.w = right - left;
			bbox.h = bot - top;
			bbox.obj_id = obj_id;
			bbox.prob = dets[i].prob[obj_id]*100;

			bbox_vector.push_back(bbox);
		}
	}
    return (bbox_vector);
}

image						// TODO the image crop may also easily be made here
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
    			image.data[dst_index] = (float) (data[src_index] / 255.0);    // Darknet pixels vary from 0 to 1 so we must divide the carmen pixel by its maximun value (255)
    		}
    	}
    }
    return (image);
} 


std::vector<bbox_t>
run_YOLO(unsigned char *data, int c, int w, int h, void *net_config, char **classes_names, float threshold, float hier_thresh)
{
	float nms = 0.45;
	int nboxes = 0;
	// detector = (Detector*) net_config;
	std::cout << "recebido " << detector << std::endl;
	
	image im = convert_image_msg_to_darknet_image(w, h, data);
	// std::cout << "Net2 width: " << detector->get_net_width() << " height:" << detector->get_net_height() << std::endl;
	// image im = resize_image(img_original, detector->get_net_width(), detector->get_net_height());
	image_t img;
    img.c = im.c;
    img.data = im.data;
    img.h = im.h;
    img.w = im.w;
	
	std::vector<bbox_t> bbox_vector = detector->detect(img);
	std::cout << "detectado " << bbox_vector.size() << std::endl;
	// detector->free_image(img);

    // detector->free_image(img);
    //show_console_result(bbox_vector, obj_names);
	// network *net = (network*) net_config;

	// image img = convert_image_msg_to_darknet_image(w, h, data);

	// image sized = resize_image(img, net->w, net->h);

	// network_predict(*net, sized.data);

	// detection *dets = get_network_boxes(net, img.w, img.h, threshold, hier_thresh, 0, 1, &nboxes, 0);

	// // Remove coincident bboxes
	// if (nms)
	// 	do_nms_sort(dets, nboxes, net->layers[net->n-1].classes, nms);

	// std::vector<bbox_t> bbox_vector = extract_predictions(img, dets, nboxes, threshold, net->layers[net->n-1].classes, classes_names);

	// free_detections(dets, nboxes);
	free_image(im);

	return (bbox_vector);
}