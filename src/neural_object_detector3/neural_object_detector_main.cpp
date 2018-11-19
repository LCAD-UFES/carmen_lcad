#include "neural_object_detector.hpp"


int camera;
int camera_side;
char **classes_names;
void *network_struct;
carmen_velodyne_partial_scan_message *velodyne_msg;
carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;
carmen_point_t   globalpos;


void
display(Mat image, vector<bbox_t> predictions, vector<image_cartesian> points, vector<vector<image_cartesian>> points_inside_bbox,
		vector<vector<image_cartesian>> filtered_points, double fps, unsigned int image_width, unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height)
{
	char object_info[25];
    char frame_rate[25];

    cvtColor(image, image, COLOR_RGB2BGR);

    sprintf(frame_rate, "FPS = %.2f", fps);

    putText(image, frame_rate, Point(10, 25), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
    	//printf("---%d \n", predictions[i].obj_id);
        sprintf(object_info, "%d %s %d", predictions[i].obj_id, classes_names[predictions[i].obj_id], (int)predictions[i].prob);

        rectangle(image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
        		Scalar(0, 0, 255), 1);

        putText(image, object_info/*(char*) "Obj"*/, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
    }

	//show_all_points(image, image_width, image_height, crop_x, crop_y, crop_width, crop_height);
    //show_LIDAR(image, points_inside_bbox,    0, 0, 255);				// Blue points are all points inside the bbox
    //show_LIDAR(image, filtered_points, 0, 255, 0); 						// Green points are filtered points

    //resize(image, image, Size(600, 300));
    imshow("Neural Object Detector", image);
    //imwrite("Image.jpg", image);
    waitKey(1);
}


unsigned char *
crop_raw_image(int image_width, int image_height, unsigned char *raw_image, int crop_x, int crop_y, int crop_w, int crop_h)
{
	unsigned char *cropped_image = (unsigned char *) malloc (crop_w * crop_h * 3 * sizeof(unsigned char));  // Only works for 3 channels image
	int row, col, index;

	crop_x = (crop_x - 2) * 3;
	crop_y = (crop_y - 2) * image_width * 3;
	crop_w    = crop_x + ((crop_w + 1) * 3);
	crop_h    = crop_y + ((crop_h + 1) * image_width * 3);
	image_height   = image_height * image_width * 3;
	image_width   *= 3;

	for (row = 0, index = 0; row < image_height; row += image_width)
	{
		for (col = 0; col < image_width; col += 3)
		{
			if (col > crop_x && col < crop_w && row > crop_y && row < crop_h)
			{
				cropped_image[index]     = raw_image[row + col];
				cropped_image[index + 1] = raw_image[row + col + 1];
				cropped_image[index + 2] = raw_image[row + col + 2];

				index += 3;
			}
		}
	}
	return (cropped_image);
}


void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	if (image_msg == NULL)
		return;

	double fps;
	static double start_time = 0.0;
	unsigned char *img;

	if (camera_side == 0)
		img = image_msg->raw_left;
	else
		img = image_msg->raw_right;

	vector<bbox_t> predictions;
	vector<image_cartesian> points;
	vector<vector<image_cartesian>> points_inside_bbox;
	vector<vector<image_cartesian>> filtered_points;
	vector<image_cartesian> positions;

	unsigned char *cropped_img = crop_raw_image(image_msg->width, image_msg->height, img, 160, 40, 320, 320);

	predictions = run_YOLO(cropped_img, 320, 320, network_struct, classes_names, 0.5);

	Mat open_cv_image = Mat(320, 320, CV_8UC3, cropped_img, 0);

	fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();
	display(open_cv_image, predictions, points, points_inside_bbox, filtered_points, fps, image_msg->width, image_msg->height, 0, 0, image_msg->width, image_msg->height);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	velodyne_msg = velodyne_message;

	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_msg);
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	globalpos.theta = globalpos_message->globalpos.theta;
	globalpos.x = globalpos_message->globalpos.x;
	globalpos.y = globalpos_message->globalpos.y;
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Neural Object Detector: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters(int argc, char **argv)
{
	if ((argc != 3))
		carmen_die("%s: Wrong number of parameters. neural_object_detector requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>", argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image

    int num_items;

    char bumblebee_string[256];
    char camera_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the cameri ID
    sprintf(camera_string, "%s%d", "camera", camera);

    carmen_param_t param_list[] =
    {
		{bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
		{bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
		{bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
		{bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
		{bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

		{(char *) "velodyne", (char *) "x",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne", (char *) "y",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne", (char *) "z",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne", (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
		{camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
		{camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
		{camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
		{camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
		{camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL }
    };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
}


void
initialize_YOLO_detector()
{
	classes_names = get_classes_names((char*) "../../sharedlib/darknet2/cfg/coco.data", (char*) "../../sharedlib/darknet2/data/names.list");

	network_struct = initialize_YOLO((char*) "../../sharedlib/darknet2/cfg/yolov3.cfg", (char*) "../../sharedlib/darknet2/yolov3.weights");
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	subscribe_messages();

	signal(SIGINT, shutdown_module);

	initialize_YOLO_detector();

	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

//	test_YOLO(network_struct, classes_names, (char*) "dog.jpeg");

	return 0;
}
