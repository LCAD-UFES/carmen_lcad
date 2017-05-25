#include <carmen/carmen.h>

#include <carmen/velodyne_interface.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


static void _mkdir(const char *dir)
{
	char tmp[256];
	char *p = NULL;
	size_t len;

	snprintf(tmp, sizeof(tmp),"%s",dir);
	len = strlen(tmp);
	if(tmp[len - 1] == '/')
		tmp[len - 1] = 0;
	for(p = tmp + 1; *p; p++)
		if(*p == '/')
		{
			*p = 0;
			mkdir(tmp, S_IRWXU);
			*p = '/';
		}
	mkdir(tmp, S_IRWXU);
}


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
	unsigned char original_intensities[32];

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message->partial_scan[i].distance, 32 * sizeof(unsigned short));
		memcpy(original_intensities, velodyne_message->partial_scan[i].intensity, 32 * sizeof(unsigned char));

		for (j = 0; j < 32; j++)
		{
			velodyne_message->partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
			velodyne_message->partial_scan[i].intensity[column_correspondence[j]] = original_intensities[j];
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{

	static int velodyne_number = 0;
	char map_filename[256];
	char timestamps_filename[256];

	double CAR_HEIGHT = 0.28 + 1.394 + 0.48;

	double map_resolution = 0.05;
	double x_min = -30.0;
	double x_max = 30.0;
	double y_min = -15.0;
	double y_max = 15.0;
	double z_min = -CAR_HEIGHT;//-13.7;
	double z_max = 0.0;//11.0;

	int width = (x_max - x_min)/map_resolution;
	int height = (y_max - y_min)/map_resolution;

	FILE* timestamps;

	const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
	};

	arrange_velodyne_vertical_angles_to_true_position(velodyne_message);

	// write velodyne
	_mkdir("/dados/dataset/map");
	sprintf(map_filename, "/dados/dataset/map/%lf.png", velodyne_message->timestamp);

	cv::Mat *map = NULL;
	map = new cv::Mat(cv::Size(width, height), CV_8UC3);


	for(int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		for(int j = 0; j < 23; j++)
		{
			carmen_vector_3D_t point_3d;
			carmen_sphere_coord_t sphere_point;
			double hangle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle));
			double vangle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[j]));
			double range = velodyne_message->partial_scan[i].distance[j] / 500.0;
			float intensity = (float)velodyne_message->partial_scan[i].intensity[j] / 255.0;

			sphere_point.horizontal_angle = -hangle;
			sphere_point.vertical_angle = vangle;
			sphere_point.length = range;

			point_3d = carmen_covert_sphere_to_cartesian_coord(sphere_point);

			if (point_3d.x >= x_min && point_3d.x < x_max
					&& point_3d.y >= y_min && point_3d.y < y_max)
			{
				int x = (point_3d.x - x_min)/map_resolution;
				int y = (point_3d.y - y_min)/map_resolution;

				cv::Vec3b color = map->at<cv::Vec3b>(height - y - 1, x);
				double k = (point_3d.z - z_min)/(z_max - z_min);

				if (k < 0.0)
					k = 0.0;

				if (color[2] <= k * 255)
				{
					color[0] = k * 0 + (1 - k) * 255;
					color[1] = 255 * intensity * 10;
					color[2] = k * 255 + (1 - k) * 0;

					map->at<cv::Vec3b>(height - y - 1, x) = color;
				}
			}

		}
	}

//	printf("z_min = %lf z_max = %lf\n", z_min, z_max);

	cv::imwrite(map_filename, *map);

	// write timestamps file
	sprintf(timestamps_filename, "/dados/dataset/timestamps.txt");

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

		printf("Carmen velodyne to map: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
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
