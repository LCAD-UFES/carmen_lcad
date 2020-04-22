#include "neural_object_detector.hpp"

char **classes_names;
void *network_struct;


FILE*
open_file(int argc, char **argv)
{
	if (argc < 3)
		carmen_die("--- Wrong number of parameters. ---\nUsage: %s <image_list> <output_folder> OPTNIONAL: -show -save_images\n", argv[0]);

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
show_object_detections(Mat open_cv_image, vector<bbox_t> predictions, bool show_detections, bool save_images, char* file_path_with_extension)
{
	char object_info[25];
    char frame_rate[25];

    cvtColor(open_cv_image, open_cv_image, COLOR_RGB2BGR);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
		rectangle(open_cv_image, Point(predictions[i].x + 1, predictions[i].y - 3), Point(predictions[i].x + 100, predictions[i].y - 15), Scalar(0, 0, 0), -1, 8, 0);

        sprintf(object_info, "%s %d", classes_names[predictions[i].obj_id], (int)predictions[i].prob);

        putText(open_cv_image, object_info, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);

        rectangle(open_cv_image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)), Scalar(255, 0, 255), 4);
    }

	if (save_images)
	{
		printf("Saving Result Image %s\n", file_path_with_extension);
		imwrite(file_path_with_extension, open_cv_image, {CV_IMWRITE_PNG_COMPRESSION, 9});
	}
	if (show_detections)
	{
		imshow("Neural Object Detector", open_cv_image);
    	waitKey(1);
	}
}

int
remove_file_type(char* image_file)
{
	int size = strlen(image_file);

	for (int i = size; i > 0; i--)
	{
		if (image_file[i] == '.')
		{
			image_file[i+1] = '\0';
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

	remove_file_type(image_path);
}


void
run_yolo_on_dataset(FILE* image_list, bool show_detections, bool save_images, char *output_dir)
{
	Mat open_cv_image;
	char file_path[1024];
	char file_path_with_extension[1024];

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

		sprintf(file_path_with_extension, "%s%s", file_path, "txt");

		printf("Saving Result File %s\n", file_path_with_extension);

		vector<bbox_t> predictions = run_YOLO_and_save_predictions(open_cv_image.data, open_cv_image.cols, open_cv_image.rows, network_struct, classes_names, 0.2, file_path_with_extension);

		if (show_detections || save_images)
		{
			sprintf(file_path_with_extension, "%s%s", file_path, "png");
			show_object_detections(open_cv_image, predictions, show_detections, save_images, file_path_with_extension);
		}

		fscanf(image_list, "%s", file_path);
	}
}


void
initializer(char* argv)
{
	char* carmen_home = getenv("CARMEN_HOME");
	char classes_names_path[1024];
	char yolo_cfg_path[1024];
	char yolo_weights_path[1024];

	sprintf(classes_names_path, "%s/sharedlib/darknet2/data/coco.names", carmen_home);
	sprintf(yolo_cfg_path, "%s/sharedlib/darknet2/cfg/yolov3.cfg", carmen_home);
	sprintf(yolo_weights_path, "%s/sharedlib/darknet2/yolov3.weights", carmen_home);

	classes_names = get_classes_names(classes_names_path);

	network_struct = initialize_YOLO( yolo_cfg_path, yolo_weights_path);
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


bool
find_save_arg(int argc, char **argv)
{
    for(int i = 0; i < argc; ++i)
    {
        if(!argv[i])
        	continue;

        if(0 == strcmp(argv[i], "-save_images"))
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
	bool save_images = find_save_arg(argc, argv);

	initializer(argv[3]);

    run_yolo_on_dataset(image_list, show_detections, save_images, argv[2]);

    fclose (image_list);

    return 0;
}
