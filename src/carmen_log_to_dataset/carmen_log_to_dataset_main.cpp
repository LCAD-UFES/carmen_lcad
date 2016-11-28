#include <carmen/carmen.h>

#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/highgui/highgui.hpp>


int camera = 8;


void
arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message)
{
	const int column_correspondence[32] =
	{
		0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
		24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
	};

	int i, j;
	unsigned short original_distances[32];

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message->partial_scan[i].distance, 32 * sizeof(unsigned short));

		for (j = 0; j < 32; j++)
		{
			velodyne_message->partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
image_handler(carmen_bumblebee_basic_stereoimage_message* image_msg)
{
	static int image_number = 0;
	char image_filename[256];
	char timestamps_filename[256];

	FILE* timestamps;

	cv::Mat *right_image = NULL;
	cv::Mat *left_image = NULL;

	right_image = new cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3);
	left_image = new cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3);

	memcpy(right_image->data, image_msg->raw_right, image_msg->image_size * sizeof(char));
	memcpy(left_image->data, image_msg->raw_left, image_msg->image_size * sizeof(char));

	cv::cvtColor(*left_image, *left_image, CV_BGR2RGB);
	cv::cvtColor(*right_image, *right_image, CV_BGR2RGB);

	// write left image
	sprintf(image_filename, "/home/luan/dataset/log_images/images_left/%08d.png", image_number);
	cv::imwrite(image_filename, *left_image);

	// write right image
	sprintf(image_filename, "/home/luan/dataset/log_images/images_right/%08d.png", image_number);
	cv::imwrite(image_filename, *right_image);

	// write timestamps file
	sprintf(timestamps_filename, "/home/luan/dataset/log_images/timestamps.txt");

	if(image_number == 0)
		timestamps = fopen(timestamps_filename, "w");
	else
		timestamps = fopen(timestamps_filename, "a+");

	fprintf(timestamps, "%lf\n", image_msg->timestamp);
	fclose(timestamps);

	image_number++;
}


void
velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{

	static int velodyne_number = 0;
	char velodyne_filename[256];
	char timestamps_filename[256];

	FILE* timestamps;
	FILE* velodyne_data;

	const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
	};

	arrange_velodyne_vertical_angles_to_true_position(velodyne_message);

	// write velodyne
	sprintf(velodyne_filename, "/home/luan/dataset/log_velodyne/velodyne/%08d.bin", velodyne_number);
	velodyne_data = fopen(velodyne_filename, "w");

	for(int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		for(int j = 0; j < 32; j++)
		{
			carmen_vector_3D_t point_3d;
			float data[4];

			carmen_sphere_coord_t sphere_point;
			double hangle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle));
			double vangle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[j]));
			double range = velodyne_message->partial_scan[i].distance[j] / 500.0;
			float intensity = velodyne_message->partial_scan[i].intensity[j] / 255.0;

			sphere_point.horizontal_angle = -hangle;
			sphere_point.vertical_angle = vangle;
			sphere_point.length = range;

			point_3d = carmen_covert_sphere_to_cartesian_coord(sphere_point);

			data[0] = (float) point_3d.x;
			data[1] = (float) point_3d.y;
			data[2] = (float) point_3d.z;
			data[3] = intensity;

			fwrite(data,sizeof(float),4,velodyne_data);

		}
	}

	fclose(velodyne_data);

	// write timestamps file
	sprintf(timestamps_filename, "/home/luan/dataset/log_velodyne/timestamps.txt");

	if(velodyne_number == 0)
		timestamps = fopen(timestamps_filename, "w");
	else
		timestamps = fopen(timestamps_filename, "a+");

	fprintf(timestamps, "%lf\n", velodyne_message->timestamp);
	fclose(timestamps);

	velodyne_number++;
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		cvDestroyAllWindows();

		printf("Moving Objects: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera,
    		NULL, (carmen_handler_t) image_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
			(carmen_handler_t) velodyne_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}
