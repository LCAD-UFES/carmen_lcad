#include "neural_object_detector2.hpp"

#define DISPLAY_DETECTIONS

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
run_yolo_on_dataset(Detector *darknet, FILE* image_list)
{
	char line [200] = "image_0000_1487599086.855343.png";
	vector<bbox_t> predictions;
	Mat image;
	FILE* results = fopen("results.txt", "w");

	int a = fscanf (image_list, "%s", line);

	while (!feof(image_list))
	{
		image = imread(line, 1); // imread(filename, flag)   The flag is ImreadModes, 1 is for color image, 0 for greyscale

		//Rect myROI(280, 70, 720, 480);    // Uncomment these lines to select a crop region on the image
		//image = image(myROI);

		printf ("Image %s Loaded!\n", line);

		predictions = darknet->detect(image, 0.2);    // Arguments (img, threshold)

		#ifdef DISPLAY_DETECTIONS
			show_object_detections(image, predictions);
		#endif

		save_predictions_to_file(predictions, line, results);

		a = fscanf (image_list, "%s", line);
	}
	fclose (results);
}


int
main(int argc, char **argv)
{
	FILE* image_list = open_file(argc, argv);

    int gpu = 1;
    int device_id = 0;

    string darknet_home = getenv("DARKNET_HOME");  // Get environment variable pointing path of darknet

    if (darknet_home.empty())
        printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");

    string cfg_filename = darknet_home + "/cfg/neural_object_detector_yolo.cfg";
    string weight_filename = darknet_home + "/yolo.weights";
    string class_names_file = darknet_home + "/data/coco.names";

    Detector *darknet = new Detector(cfg_filename, weight_filename, device_id);

    carmen_test_alloc(darknet);

    setlocale(LC_ALL, "C");

    run_yolo_on_dataset(darknet, image_list);

    fclose (image_list);

    return 0;
}
