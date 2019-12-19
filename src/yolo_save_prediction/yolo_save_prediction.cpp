#include "yolo_save_prediction.hpp"
#include "network.h"

char **classes_names;
void *network_struct;

FILE *
open_file(int argc, char **argv)
{
	if (argc < 6)
		printf("--- Wrong number of parameters. ---\nUsage: %s <image_list> <output_folder> <[1|0] coco|yours> <class_names_file> <cfg_file> <weights_file>\n", argv[0]);

	FILE *file = fopen(argv[1], "r");

	if (file == NULL)
	{
		printf("------------------------------------------------------------------------------------------\n");
		printf("Failed! COLD NOT OPEN FILE: %s\n", argv[1]);
		printf("------------------------------------------------------------------------------------------\n");
		exit(0);
	}
	else
	{
		return file;
	}
}

void save_yolo_predictions_to_file(vector<bbox_t> predictions, char *file_path)
{
	FILE *output_file = fopen(file_path, "w");

	for (int i = 0; i < predictions.size(); i++)
	{
		int id = 0;
		switch (predictions[i].obj_id)
		{
			case 0:
			{
				id = 2;
				fprintf(output_file, "%d %f %f %f %f %f\n", id, predictions[i].prob, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
				break;
			}
			case 2:
			{
				id = 0;
				fprintf(output_file, "%d %f %f %f %f %f\n", id, predictions[i].prob, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
				break;
			}
			case 5:
			{
				id = 1;
				fprintf(output_file, "%d %f %f %f %f %f\n", id, predictions[i].prob, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
				break;
			}
			case 7:
			{
				id = 1;
				fprintf(output_file, "%d %f %f %f %f %f\n", id, predictions[i].prob, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
				break;
			}
			case 9:
			{
				id = 3;
				fprintf(output_file, "%d %f %f %f %f %f\n", id, predictions[i].prob, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
				break;
			}
		}
		
	}
	fclose(output_file);
}

void save_my_predictions_to_file(vector<bbox_t> predictions, char *file_path)
{
	FILE *output_file = fopen(file_path, "w");

	for (int i = 0; i < predictions.size(); i++)
	{
		fprintf(output_file, "%d %f %f %f %f %f\n", predictions[i].obj_id, predictions[i].prob, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
		// printf("%d %f %f %f %f %f\n", predictions[i].obj_id, predictions[i].prob, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
	}
	fclose(output_file);
}

void show_object_detections(Mat open_cv_image, vector<bbox_t> predictions)
{
	char object_info[25];
	char frame_rate[25];

	cvtColor(open_cv_image, open_cv_image, COLOR_RGB2BGR);

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		sprintf(object_info, "%s %d", classes_names[predictions[i].obj_id], (int)predictions[i].prob);

		putText(open_cv_image, object_info, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);

		rectangle(open_cv_image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
							Scalar(0, 0, 255), 1);
	}

	imshow("Neural Object Detector", open_cv_image);
	waitKey(1);
}

int replace_file_type(char *image_file)
{
	int size = strlen(image_file);

	for (int i = size; i > 0; i--)
	{
		if (image_file[i] == '.')
		{
			image_file[i + 1] = 't';
			image_file[i + 2] = 'x';
			image_file[i + 3] = 't';
			image_file[i + 4] = '\0';
			return 1;
		}
	}
	printf("Invalid image file %s\n", image_file);
	return 0;
}

void compute_output_file_path(char *output_dir, char *image_path)
{
	char *tok = strtok(image_path, "/");
	char *path = tok;

	while (tok != NULL)
	{
		path = tok;
		tok = strtok(NULL, "/");
	}

	sprintf(image_path, "%s/%s", output_dir, path);

	replace_file_type(image_path);
}

void run_yolo_on_dataset(FILE *image_list, bool show_detections, char *output_dir, int original)
{
	Mat open_cv_image;
	char file_path[1024];

	fscanf(image_list, "%s", file_path);
	printf("executando em modo: %d",original);
	while (!feof(image_list))
	{
		open_cv_image = imread(string(file_path), CV_LOAD_IMAGE_COLOR);
		cvtColor(open_cv_image, open_cv_image, COLOR_RGB2BGR);

		if (!open_cv_image.data)
		{
			printf("Could not open open_cv_image %s\n", file_path);
			continue;
		}
		printf("Loading Image %s\n", file_path);

		//Rect myROI(280, 70, 720, 480);               // Uncomment these lines to select a crop region on the image
		//image = image(myROI);

		compute_output_file_path(output_dir, file_path);

		printf("Saving Result File %s\n", file_path);

		std::vector<bbox_t> predictions = run_YOLO(open_cv_image.data, open_cv_image.cols, open_cv_image.rows, network_struct, classes_names, 0.2);
		
		//colocar o show_object_detections(open_cv_image,predictions) aqui

		if (original == 1)
		{
			save_yolo_predictions_to_file(predictions, file_path);
		}
		else
		{
			save_my_predictions_to_file(predictions, file_path);
		}

		fscanf(image_list, "%s", file_path);
	}
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
			bbox_t bbox;
			bbox.x = dets[i].bbox.x;
			bbox.y = dets[i].bbox.y;
			bbox.w = dets[i].bbox.w;
			bbox.h = dets[i].bbox.h;
			bbox.obj_id = obj_id;
			bbox.prob = dets[i].prob[obj_id];

			bbox_vector.push_back(bbox);
		}
	}
    return (bbox_vector);
}

std::vector<bbox_t>
run_YOLO(unsigned char *data, int w, int h, void *net_config, char **classes_names, float threshold)
{
        float nms = 0.45;
        int nboxes = 0;

        network *net = (network*) net_config;

        image img = convert_image_msg_to_darknet_image(w, h, data);

        image sized = letterbox_image(img, net->w, net->h);

        float *X = sized.data;

        network_predict(net, X);

        detection *dets = get_network_boxes(net, img.w, img.h, threshold,
				 threshold, 0, 1, &nboxes);

        // Remove coincident bboxes
        if (nms)
            do_nms_sort(dets, nboxes, net->layers[net->n-1].classes, nms);

        std::vector<bbox_t> bbox_vector = extract_predictions(img, dets, nboxes, threshold, net->layers[net->n-1].classes, classes_names);

        free_detections(dets, nboxes);
        free_image(sized);
        free_image(img);
        return (bbox_vector);
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

void initializer(char **argv)
{
	classes_names = get_classes_names(argv[4]);
	network_struct = initialize_YOLO(argv[5], argv[6]);
}

bool find_show_arg(int argc, char **argv)
{
	for (int i = 0; i < argc; ++i)
	{
		if (!argv[i])
			continue;

		if (0 == strcmp(argv[i], "-show"))
			return (true);
	}
	return (false);
}

int main(int argc, char **argv)
{
	FILE *image_list = open_file(argc, argv);

	/* int status = mkdir(argv[2], 0777);
	if (status == -1)
		printf("Warning: Directory %s already exists.\n", argv[2]);
	else if (status != 0)
		exit(printf("ERROR: Could not create directory '%s'\n", argv[2]));
 */
	bool show_detections = find_show_arg(argc, argv);

	initializer(argv);

	run_yolo_on_dataset(image_list, show_detections, argv[2] /*output dir*/, atoi(argv[3]) /*1 para yolo 0 para personalizado*/);

	fclose(image_list);

	return 0;
}
