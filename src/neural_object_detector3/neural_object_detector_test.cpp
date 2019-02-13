#include "neural_object_detector.hpp"

char **classes_names;
void *network_struct;


FILE*
open_file(int argc, char **argv)
{
	if (argc < 3)
		carmen_die("--- Wrong number of parameters. ---\nUsage: %s <image_list> <output_folder>\n", argv[0]);

	FILE* file = fopen(argv[1], "r");

	if (file == NULL)
	{
		printf ("------------------------------------------------------------------------------------------\n");
		printf ("Failed! COLD NOT OPEN FILE: %s\n", argv[1]);
		printf ("------------------------------------------------------------------------------------------\n");
		exit(0);
	}
	else
	{
		return file;
	}
}


void
save_predictions_to_file(vector<bbox_t> predictions, char* file_path)
{
	FILE* output_file = fopen(file_path, "w");

	for (int i = 0; i < predictions.size(); i++)
	{
		fprintf(output_file, "%d %d %d %d %d\n", predictions[i].obj_id, predictions[i].x, predictions[i].y, predictions[i].w, predictions[i].h);
	}
	fclose(output_file);
}


void
show_object_detections(Mat open_cv_image, vector<bbox_t> predictions)
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

int
replace_file_type(char* image_file)
{
	int size = strlen(image_file);

	for (int i = size; i > 0; i--)
	{
		if (image_file[i] == '.')
		{
			image_file[i+1] = 't';
			image_file[i+2] = 'x';
			image_file[i+3] = 't';
			image_file[i+4] = '\0';
			return 1;
		}
	}
	printf("Invalid image file %s\n", image_file);
	return 0;
}


void
compute_output_file_path(char *output_dir, char *image_path)
{
	char *tok = strtok (image_path, "/");
	char *path = tok;

	while (tok != NULL)
	{
		path = tok;
		tok = strtok (NULL, "/");
	}

	sprintf(image_path, "%s/%s", output_dir, path);

	replace_file_type(image_path);
}


void
run_yolo_on_dataset(FILE* image_list, bool show_detections, char *output_dir)
{
	Mat open_cv_image;
	char file_path[1024];

	fscanf(image_list, "%s", file_path);

	while (!feof(image_list))
	{
		open_cv_image = imread(string(file_path), CV_LOAD_IMAGE_COLOR);
		cvtColor(open_cv_image, open_cv_image, COLOR_RGB2BGR);

		if(!open_cv_image.data)
		{
			printf("Could not open open_cv_image %s\n", file_path);
			continue;
		}
		printf("Loading Image %s\n", file_path);

		//Rect myROI(280, 70, 720, 480);               // Uncomment these lines to select a crop region on the image
		//image = image(myROI);

		compute_output_file_path(output_dir, file_path);

		printf("Saving Result File %s\n", file_path);

		run_YOLO_VOC_Pascal(open_cv_image.data, open_cv_image.cols, open_cv_image.rows, network_struct, classes_names, 0.2, file_path);

		fscanf(image_list, "%s", file_path);
	}
}


void
initializer(char* argv)
{
	classes_names = get_classes_names((char*) "../../sharedlib/darknet2/data/coco.names");
	network_struct = initialize_YOLO((char*) "/dados/marcelo/darknet2/cfg/yolov3-voc_lane.cfg", (char*) argv);

//	classes_names = get_classes_names((char*) "../../sharedlib/darknet2/data/coco.names");
//	network_struct = initialize_YOLO((char*) "../../sharedlib/darknet2/cfg/yolov3.cfg", (char*) "../../sharedlib/darknet2/yolov3.weights");
}


bool
find_show_arg(int argc, char **argv)
{
    for(int i = 0; i < argc; ++i)
    {
        if(!argv[i])
        	continue;

        if(0 == strcmp(argv[i], "-show"))
        	return (true);
    }
    return (false);
}


int
main(int argc, char **argv)
{
	FILE* image_list = open_file(argc, argv);

	int status = mkdir(argv[2], 0777);
	if (status == -1)
		printf("Warning: Directory %s already exists.\n", argv[2]);
	else if (status != 0)
		exit(printf("ERROR: Could not create directory '%s'\n", argv[2]));

	bool show_detections = find_show_arg(argc, argv);

	initializer(argv[3]);

    run_yolo_on_dataset(image_list, show_detections, argv[2]);

    fclose (image_list);

    return 0;
}
