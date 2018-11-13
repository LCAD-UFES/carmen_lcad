#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <string>
#include <cstdlib>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <carmen/darknet.h>

using namespace std;
using namespace cv;

int camera;
int camera_side;


// These parameters are given in percentual form
typedef struct
{
	double fx_factor;
	double fy_factor;
	double cu_factor;
	double cv_factor;
	double pixel_size;
	double baseline;
} carmen_camera_parameters;

carmen_velodyne_partial_scan_message *velodyne_msg;
carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;
carmen_point_t   globalpos;

char **classes_names;
image **alphabet;
network *network_config;

typedef struct
{
    unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;					// confidence - probability that the object was found correctly
    unsigned int obj_id;		// class of object - from range [0, classes-1]
    unsigned int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
}bbox_t;


typedef struct
{
	int shot_number;
	int ray_number;
    int image_x;
    int image_y;
    double cartesian_x;
    double cartesian_y;
    double cartesian_z;
}image_cartesian;


void
show_detections(Mat image, vector<bbox_t> predictions, vector<image_cartesian> points, vector<vector<image_cartesian>> points_inside_bbox,
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


vector<bbox_t>
extract_predictions(image im, detection *dets, int num, float thresh, image **alphabet, int classes, char* line)
{
	int i,j;
	vector<bbox_t> bbox_vector;

	for(i = 0; i < num; ++i)
	{
		char labelstr[4096] = {0};
		int clas = -1;

		for(j = 0; j < classes; ++j)
		{
			if (dets[i].prob[j] > thresh)
			{
				if (clas < 0)
				{
					strcat(labelstr, classes_names[j]);
					clas = j;
				}
				else
				{
					strcat(labelstr, ", ");
					strcat(labelstr, classes_names[j]);
				}
				//printf("%s: %.0f%%\n", classes_names[j], dets[i].prob[j]*100);
			}
		}
		if (clas >= 0)
		{
			box b = dets[i].bbox;

			int left  = (b.x - (b.w / 2.0)) * im.w;
			int right = (b.x + (b.w / 2.0)) * im.w;
			int top   = (b.y - (b.h / 2.0)) * im.h;
			int bot   = (b.y + (b.h / 2.0)) * im.h;

			if(left < 0)
				left = 0;
			if(right > im.w-1)
				right = im.w-1;
			if(top < 0)
				top = 0;
			if(bot > im.h-1)
				bot = im.h-1;

			bbox_t bbox;

			bbox.x = left;
			bbox.y = top;
			bbox.w = right - left;
			bbox.h = bot - top;
			bbox.obj_id = clas;
			bbox.prob = dets[i].prob[clas]*100;
			bbox.track_id = 0;

			bbox_vector.push_back(bbox);
		}
	}
    return (bbox_vector);
}

vector<bbox_t>
test_YOLO()
{
	char buff[256], *input = buff;
	float nms = 0.45;
	int nboxes = 0;

	strncpy(input, (char*) "../../sharedlib/darknet2/data/dog.jpg", 256);

	image im = load_image_color(input,0,0);

	image sized = letterbox_image(im, network_config->w, network_config->h);

	layer l = network_config->layers[network_config->n-1];

	float *X = sized.data;

	network_predict(network_config, X);

	//printf("%s: Predicted in %f seconds.\n", input, what_time_is_it_now()-time);

	detection *dets = get_network_boxes(network_config, im.w, im.h, 0.5, 0.5, 0, 1, &nboxes);

	if (nms)
		do_nms_sort(dets, nboxes, 80, nms);
	//do_nms_sort(dets, nboxes, l.classes, nms); // l.classes do not work when darknet is compiled for cuda set l.classes = 80 to coco dataset

	//draw_detections(im, dets, nboxes, 0.5, classes_names, alphabet, 80);

	vector<bbox_t> bbox_vector = extract_predictions(im, dets, nboxes, 0.5, alphabet, 80, input);

	//save_image(im, "predictions");

	free_image(im);
	free_image(sized);

	return (bbox_vector);
}


vector<bbox_t>
run_YOLO(image *im)
{
	char buff[256], *input = buff;
	float nms = 0.45;
	int nboxes = 0;

	//save_image(*im, "predictions");

	image sized = letterbox_image(*im, network_config->w, network_config->h);

	layer l = network_config->layers[network_config->n-1];

	float *X = sized.data;

	network_predict(network_config, X);

	//printf("%s: Predicted in %f seconds.\n", input, what_time_is_it_now()-time);

	detection *dets = get_network_boxes(network_config, im->w, im->h, 0.5, 0.5, 0, 1, &nboxes);

	if (nms)
		do_nms_sort(dets, nboxes, 80, nms);
	//do_nms_sort(dets, nboxes, l.classes, nms); // l.classes do not work when darknet is compiled for cuda set l.classes = 80 to coco dataset

	//draw_detections(im, dets, nboxes, 0.5, classes_names, alphabet, 80);

	vector<bbox_t> bbox_vector = extract_predictions(*im, dets, nboxes, 0.5, alphabet, 80, input);

	//save_image(im, "predictions");

	//free_image(im);
	free_image(sized);

	return (bbox_vector);
}


image
convert_image_msg_to_darknet_image(unsigned int image_size, unsigned int w, unsigned int h, unsigned char *data)
{
    int c = 3;
    image image = make_image(w, h, c);

    if (data == NULL)
    	return image;

    for(int k = 0; k < c; ++k)
    {
    	for(int j = 0; j < h; ++j)
    	{
    		for(int i = 0; i < w; ++i)
    		{
    			int dst_index = i + (w * j) + (w * h * k);
    			int src_index = k + (c * i) + (c * w* j);
    			image.data[dst_index] = (float) (data[src_index] / 255.0); //
    		}
    	}
    }
    return (image);
}


void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	if (image_msg == NULL)
		return;

	double fps;
	static double start_time = 0.0;

	vector<image_cartesian> points;
	vector<vector<image_cartesian>> points_inside_bbox;
	vector<vector<image_cartesian>> filtered_points;
	vector<image_cartesian> positions;

	double time = what_time_is_it_now();
	float nms = 0.45;
	float thresh = 0.2;
	float hier_thresh = 0.5;

	image darknet_image = convert_image_msg_to_darknet_image(image_msg->image_size, image_msg->width, image_msg->height, image_msg->raw_right);
//	im.w = image_msg->width;
//	im.h = image_msg->height;
//	im.c = 3;
//	im.data = float_image;

	//save_image(im, "predictions");

	vector<bbox_t> predictions = run_YOLO(&darknet_image);

	//printf("Predic %d\n", (int)predictions.size());

	Mat open_cv_image = Mat(image_msg->height, image_msg->width, CV_8UC3, image_msg->raw_right, 0);

	show_detections(open_cv_image, predictions, points, points_inside_bbox, filtered_points, fps, image_msg->width, image_msg->height, 0, 0, image_msg->width, image_msg->height);
	imshow("Neural Object Detector", open_cv_image);
	waitKey(1);

	free_image(darknet_image);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	velodyne_msg = velodyne_message;

    //carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_msg);
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
initialize_YOLO()
{
	list *options = read_data_cfg((char*) "../../sharedlib/darknet2/cfg/coco.data");
	char *name_list = option_find_str(options, (char*) "names", (char*) "../../sharedlib/darknet2/data/names.list");

	classes_names = get_labels(name_list);
	alphabet = load_alphabet();
	network_config = load_network((char*) "../../sharedlib/darknet2/cfg/yolov3.cfg", (char*) "../../sharedlib/darknet2/yolov3.weights", 0);

	set_batch_network(network_config, 1);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	subscribe_messages();

	signal(SIGINT, shutdown_module);

	initialize_YOLO();

	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

	return 0;
}
