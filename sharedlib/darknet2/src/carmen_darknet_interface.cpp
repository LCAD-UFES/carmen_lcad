#include "network.h"     // It can not be inside extern because it includes cuda and cuda use templates
extern "C"
{
#include "data.h"
#include "demo.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "option_list.h"
#include "stb_image.h"
}
#include "../carmen_darknet_interface.hpp"


char **
get_classes_names(char *classes_names_file)
{
	return (get_labels(classes_names_file));
}


void*
initialize_YOLO(char *cfg_file_name, char *weights_file_name)
{
	network *net;

	net = load_network(cfg_file_name, weights_file_name, 0);

	set_batch_network(net, 1);

	printf("------- Number of Classes: %d -------\n", net->layers[net->n-1].classes);

	return (net);
}


//void
//save_image_file(char* img, int w, int h, char* path)
//{
//	image out_img;
//
//	out_img.w = w;
//	out_img.h = h;
//	out_img.c = 3;
//	out_img.data = img;
//
//	save_image(out_img, path);
//}


void
test_YOLO(void *net, char **classes_names, char *image_path)
{
	char buff[256], *input = buff;
	float nms = 0.45;
	int nboxes = 0;

	strncpy(input, image_path, 256);

	image im = load_image_color(input,0,0);

	network *network_config = (network*) net;

	image **alphabet = load_alphabet();

	image sized = letterbox_image(im, network_config->w, network_config->h);

	//layer l = network_config->layers[network_config->n-1];

	float *X = sized.data;

	network_predict(network_config, X);

	//printf("%s: Predicted in %f seconds.\n", input, what_time_is_it_now()-time);

	detection *dets = get_network_boxes(network_config, im.w, im.h, 0.5, 0.5, 0, 1, &nboxes);

	if (nms)
		do_nms_sort(dets, nboxes, 80, nms);
	//do_nms_sort(dets, nboxes, l.classes, nms); // l.classes do not work when darknet is compiled for cuda set l.classes = 80 to coco dataset

	draw_detections(im, dets, nboxes, 0.5, classes_names, alphabet, 80);

	save_image(im, "predictions");

	free_image(im);
	free_image(sized);
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
			bbox.track_id = 0;

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
    			image.data[dst_index] = (float) (data[src_index] / 255.0);    // 255 because of conversion Uchar (byte) ti float
    		}
    	}
    }
    return (image);
}

std::vector<bbox_t>
run_YOLO(unsigned char *data, int w, int h, void *net_config, char **classes_names, float threshold=0.5, float hier_thresh=0.5)
{
        float nms = 0.45;
        int nboxes = 0;

        network *net = (network*) net_config;

        image img = convert_image_msg_to_darknet_image(w, h, data);

        image sized = letterbox_image(img, net->w, net->h);

        float *X = sized.data;

        network_predict(net, X);

        detection *dets = get_network_boxes(net, img.w, img.h, threshold, hier_thresh, 0, 1, &nboxes);

        // Remove coincident bboxes
        if (nms)
            do_nms_sort(dets, nboxes, net->layers[net->n-1].classes, nms);

        std::vector<bbox_t> bbox_vector = extract_predictions(img, dets, nboxes, threshold, net->layers[net->n-1].classes, classes_names);

        free_detections(dets, nboxes);
        free_image(sized);
        free_image(img);
        return (bbox_vector);
}

void
save_predictions_to_file_VOC_pascal(detection *dets, int num, float thresh, int classes_number, char **classes_names, char* file_path)
{
	int i,j;

	FILE* output_file = fopen(file_path, "w");

	for(i = 0; i < num; ++i)
	{
		char labelstr[4096] = {0};
		int obj_id = -1;

		for(j = 0; j < classes_number; ++j)
		{
			if (dets[i].prob[j] > thresh)
			{
				if (obj_id < 0)
				{
					strcat(labelstr, classes_names[j]);
					obj_id = j;
				}
				else
				{
					strcat(labelstr, ", ");
					strcat(labelstr, classes_names[j]);
				}
			}
		}
		if (obj_id >= 0)
			fprintf(output_file, "%d %f %f %f %f\n", obj_id, dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h);
	}
	fclose(output_file);
}


void
run_YOLO_VOC_Pascal(unsigned char *data, int w, int h, void *net_config, char **classes_names, float threshold, char* file_path)
{
	float nms = 0.45;
	int nboxes = 0;

	network *net = (network*) net_config;

	image img = convert_image_msg_to_darknet_image(w, h, data);

	image sized = letterbox_image(img, net->w, net->h);

	float *X = sized.data;

	network_predict(net, X);

	detection *dets = get_network_boxes(net, img.w, img.h, threshold, threshold, 0, 1, &nboxes);

	if (nms)    // Remove coincident bboxes
		do_nms_sort(dets, nboxes, net->layers[net->n-1].classes, nms);

	save_predictions_to_file_VOC_pascal(dets, nboxes, 0.5, net->layers[net->n-1].classes, classes_names, file_path);

	free_detections(dets, nboxes);
	free_image(sized);
	free_image(img);
}
