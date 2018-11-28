#include "neural_object_detector.hpp"

char **classes_names;
void *network_struct;


FILE*
open_file(int argc, char **argv)
{
	if (argc != 2)
		carmen_die("--- Wrong number of parameters. ---\nUsage: %s <camera_number>\n", argv[0]);

	FILE* file = fopen(argv[1], "r");

	if (file == NULL)
	{
		cerr << "\n" <<
				"------------------------------------------------------------------------------------------" << endl <<
				"Failed! COLD NOT OPEN " << "FILE" << ": " << argv << endl <<
				"------------------------------------------------------------------------------------------" << "\n\n";
		exit(0);
	}
	else
	{
		return file;
	}
}


void
save_predictions_to_file(vector<bbox_t> predictions, char* line, FILE* results)
{
	for (int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].obj_id == 9) // 9 is the ID of trafficlight, see /data/coco.names for objects IDs
		{
			fprintf(results, "%s %d %d %d %d\n", line, predictions[i].x, predictions[i].y, predictions[i].x + predictions[i].w, predictions[i].y + predictions[i].h);
		}
	}
}


void
show_object_detections(Mat rgb_image, vector<bbox_t> predictions)
{
	char object_info[25];
    char frame_rate[25];

    cvtColor(rgb_image, rgb_image, COLOR_RGB2BGR);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
        sprintf(object_info, "%d %s %.2f", predictions[i].obj_id, "TrafficLight", predictions[i].prob);

        rectangle(rgb_image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
        		Scalar(0, 0, 255), 1);

        putText(rgb_image, object_info, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
    }

    imshow("Neural Object Detector", rgb_image);
    waitKey(300);
}


void
run_yolo_on_dataset(FILE* image_list, bool show_detections)
{
	Mat image;
	string image_path;

	fscanf (image_list, "%s", image_path);

	while (!feof(image_list))
	{
		image = imread(image_path, CV_LOAD_IMAGE_COLOR);
		if(!image.data)
		{
			cout <<  "Could not open image" << image_path << endl ;
			continue;
		}

		FILE* output_file = fopen("results.txt", "w");
		//Rect myROI(280, 70, 720, 480);               // Uncomment these lines to select a crop region on the image
		//image = image(myROI);

		printf ("Image %s Loaded!\n", image_path);
		vector<bbox_t> predictions = run_YOLO(image.data, image.row, image.cols, network_struct, classes_names, 0.2);

		if (show_detections)
			show_object_detections(image, predictions);

		save_predictions_to_file(predictions, image_path, output_file);

		a = fscanf (image_list, "%s", image_path);
	}
	fclose (results);
}


void
initializer()
{
	initialize_transformations(board_pose, camera_pose, &transformer);

	classes_names = get_classes_names((char*) "../../sharedlib/darknet2/data/coco.names");

	network_struct = initialize_YOLO((char*) "../../sharedlib/darknet2/cfg/yolov3.cfg", (char*) "../../sharedlib/darknet2/yolov3.weights");
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

	bool show_detections = find_show_arg(argc, argv);

	initializer();

    run_yolo_on_dataset(image_list, show_detections);

    fclose (image_list);

    return 0;
}
