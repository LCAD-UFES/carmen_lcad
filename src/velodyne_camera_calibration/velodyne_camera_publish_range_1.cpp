#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/visual_tracker_interface.h>
#include <string>
#include <carmen/velodyne_camera_calibration.h>
#include <tf.h>

using namespace cv;
using namespace tf;

const int RANGE_WINDOW_ZOOM = 5;
const int WINDOW_WIDTH = 1366;

int bumblebee_received = 0;

int camera;
int camera_id;
int bumblebee_basic_width;
int bumblebee_basic_height;

carmen_bumblebee_basic_stereoimage_message bumblebee_message;

static carmen_pose_3D_t camera_pose; // Camera pose in relation to sensor board
static carmen_pose_3D_t velodyne_pose; //velodyne pose in relation to sensor board

static carmen_camera_parameters camera_parameters;

static cv::Rect box;
static double confidence;
std::vector<carmen_velodyne_points_in_cam_t> laser_in_cam_px;

void
show_velodyne(carmen_velodyne_partial_scan_message *velodyne_message)
{
	if (!bumblebee_received)
		return;

	static int first = 1;

	std::vector<double> median_ranges;
	double average_range = 0.0;
	int count_range = 0;


	if (first)
	{
		namedWindow("Bumblebee");
		//moveWindow("Bumblebee", 900, 300);

		first = 0;
	}

	static Mat *bumb_image = new Mat(Size(bumblebee_message.width, bumblebee_message.height), CV_8UC3);
	static Mat *bumb_zoom = new Mat(Size(bumblebee_message.width / 2, bumblebee_message.height / 2), CV_8UC3);

	static Mat *bum_image_640 = new Mat(Size(640, 480), CV_8UC3);


	memset(bumb_image->data, 0, bumb_image->step * bumb_image->rows * sizeof(uchar));

	for (int i = 0; i < bumblebee_message.height; i++)
	{
		for (int j = 0; j < bumblebee_message.width; j++)
		{
			bumb_image->data[i * bumb_image->step + 3 * j + 0] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 2];
			bumb_image->data[i * bumb_image->step + 3 * j + 1] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 1];
			bumb_image->data[i * bumb_image->step + 3 * j + 2] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 0];
		}
	}


	float r = 0;
//	float b = 0;
//	float g = 0;
	const float MAX_RANGE = 30.0;
	const float MIN_RANGE = 0.5;
	cv::Rect mini_box;
	mini_box.x = box.x + (box.width/3);
	mini_box.y = box.y + (box.height/3);
	mini_box.width = (box.width/3);
	mini_box.height = (box.height/3);
	Scalar yellow = CV_RGB(255, 255, 0);
	Scalar white = CV_RGB(255, 255, 255);

	laser_in_cam_px = carmen_velodyne_camera_calibration_lasers_points_in_camera(velodyne_message, camera_parameters,
																				 velodyne_pose, camera_pose,
																				 bumblebee_message.width,
																				 bumblebee_message.height);

	rectangle(*bumb_image, cv::Point(box.x, box.y),cv::Point(box.x + box.width, box.y + box.height), Scalar( 0, r, 0 ), 1, 4 );
	rectangle(*bumb_image, cv::Point(mini_box.x, mini_box.y),cv::Point(mini_box.x + mini_box.width, mini_box.y + mini_box.height), yellow, 1, 4 );

	std::vector<double> ranges_d;

	for(int i = 0; i < laser_in_cam_px.size(); i++)
	{
		if((laser_in_cam_px.at(i).ipx > box.x) && ( laser_in_cam_px.at(i).ipx < (box.x + box.width)) && (laser_in_cam_px.at(i).ipy > box.y) &&
				(laser_in_cam_px.at(i).ipy < (box.y + box.height)))
		{
			if (laser_in_cam_px.at(i).laser_polar.length <= MIN_RANGE)
				laser_in_cam_px.at(i).laser_polar.length = MAX_RANGE;

			if (laser_in_cam_px.at(i).laser_polar.length > MAX_RANGE)
				laser_in_cam_px.at(i).laser_polar.length = MAX_RANGE;

			r = laser_in_cam_px.at(i).laser_polar.length / MAX_RANGE;
			r *= 255;
			r = 255 - r;
			circle(*bumb_image, cv::Point(laser_in_cam_px.at(i).ipx, laser_in_cam_px.at(i).ipy), 2, Scalar(0, r, 0), -1);
			count_range += 1;

			if((laser_in_cam_px.at(i).ipx > mini_box.x) && ( laser_in_cam_px.at(i).ipx < (mini_box.x + mini_box.width)) && (laser_in_cam_px.at(i).ipy > mini_box.y) &&
							(laser_in_cam_px.at(i).ipy < (mini_box.y + mini_box.height)))
			{
				median_ranges.push_back(laser_in_cam_px.at(i).laser_polar.length);
//				ranges_d.push_back(laser_in_cam_px.at(i).laser_polar.length);
				circle(*bumb_image, cv::Point(laser_in_cam_px.at(i).ipx, laser_in_cam_px.at(i).ipy), 2, white, -1);
//				average_range += laser_in_cam_px.at(i).laser_polar.length;
			}



			//printf("%f\t %f\t %f\t \n",r, range, velodyne_message->partial_scan[j].distance[i] );

		}

	}

	if (median_ranges.size() > 0/**/)
	{
		std::sort(median_ranges.begin(), median_ranges.end());
		int middle = median_ranges.size() / 2;
		double interval = median_ranges.back() - median_ranges.at(0);
		double median = median_ranges.at(middle);

//		double averange_distance = average_range / count_range;

		double threshold_ = 3.0;
		cv::Size s = bumb_image->size();
		rectangle(*bumb_image, cv::Point(0, 0), cv::Point(s.width, 50), Scalar::all(0), -1);
		char str[256];
		char yesno[4];
		if (interval < threshold_)
			sprintf(yesno,"%s", "YES");
		else
			sprintf(yesno,"%s", "NO");

		sprintf(str,"Median range: %lf  interval: %lf GOOD: %s Points mini: %ld Points: %d \n",median, interval, yesno, median_ranges.size(), count_range);
		putText(*bumb_image, str, Point2f(10,20), FONT_HERSHEY_PLAIN, 1.5,  Scalar(255,255,255));
	}

	resize(*bumb_image, *bum_image_640, bum_image_640->size());

	resize(*bumb_image, *bumb_zoom, bumb_zoom->size());

	imshow("Bumblebee", *bum_image_640);

	int k = waitKey(5);

}


void
bumblebee_basic_image_handler(carmen_bumblebee_basic_stereoimage_message *bumblebee_basic_message)
{
	bumblebee_received = 1;

//	static int n = 0;
//	static double time = 0;
//
//	if (bumblebee_basic_message->timestamp - time > 1.0)
//	{
//		printf("BUMB: %d\n", n);
//		time = bumblebee_basic_message->timestamp;
//		n = 0;
//	}
//	else
//		n++;
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
//	static int n = 0;
//	static double time = 0;
//
//	if (velodyne_message->timestamp - time > 1.0)
//	{
//		printf("VELO: %d\n", n);
//		time = velodyne_message->timestamp;
//		n = 0;
//	}
//	else
//		n++;

	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message);
	show_velodyne(velodyne_message);
}

void
carmen_visual_tracker_output_message_handler(carmen_visual_tracker_output_message *visual_tracker_output_message)
{

	box.x = visual_tracker_output_message->rect.x;
	box.y = visual_tracker_output_message->rect.y;
	box.width = visual_tracker_output_message->rect.width;
	box.height = visual_tracker_output_message->rect.height;

	confidence = visual_tracker_output_message->confidence;

}

int
read_parameters(int argc, char **argv, int camera)
{
	int num_items;
	char bumblebee_string[256];
	char camera_string[256];

	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);
	sprintf(camera_string, "%s%d", "camera", camera);

	carmen_param_t param_list[] = {
			{ bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
			{ bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
			{ bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
			{ bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
			{ bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

			{(char *) "velodyne",  (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char *) "velodyne",  (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char *) "velodyne",  (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char *) "velodyne",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char *) "velodyne",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char *) "velodyne",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{ camera_string, (char*) "x", CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
			{ camera_string, (char*) "y", CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
			{ camera_string, (char*) "z", CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
			{ camera_string, (char*) "roll", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
			{ camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
			{ camera_string, (char*) "yaw", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL },

			{ bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL },
			{ bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL } };

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


int
main(int argc, char **argv)
{
	if (argc != 2)
	{
		fprintf(stderr, "%s: Wrong number of parameters. Requires 2 parameter and received %d. \n Usage: %s <camera_number>\n>", argv[0], argc - 1, argv[0]);
		exit(1);
	}

	camera_id = atoi(argv[1]);
//	camera_side = atoi(argv[2]);

	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv, camera_id);

	carmen_bumblebee_basic_subscribe_stereoimage(camera_id, &bumblebee_message,
			(carmen_handler_t) bumblebee_basic_image_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
			(carmen_handler_t)velodyne_partial_scan_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_visual_tracker_subscribe_output(NULL,
				(carmen_handler_t)carmen_visual_tracker_output_message_handler,
				CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}

