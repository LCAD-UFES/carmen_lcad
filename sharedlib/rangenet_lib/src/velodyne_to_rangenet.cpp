// Saves pointclouds from the log in txt files
// First line: number of points (needs later correction)
// Other lines: x y z intensity (of each point)
#include "rangenet_inference.h"
#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <string>
#include <carmen/velodyne_camera_calibration.h>
#include <tf.h>
#include <iostream>
#include <fstream>
#include <vector>

//namespace pl = rangenet::inference;

using namespace tf;

const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67};

inline float round(float val)
{
	if (val < 0)
		return ceil(val - 0.5);
	return floor(val + 0.5);
}

void pointcloud_to_rangenet(carmen_velodyne_partial_scan_message *velodyne_message)
{
	double timestamp = velodyne_message->timestamp;
	std::string scan = std::to_string(timestamp);
	uint32_t num_points = 32 * velodyne_message->number_of_32_laser_shots;
	std::vector<float> data(4 * num_points);
	int vertical_resolution = 32;
	int line = 0;
	int i, j;
	for (j = 0; j < vertical_resolution; j++)
	{
		for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++, line++)
		{
			double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[j]));
			double h = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
			double range = (((double)velodyne_message->partial_scan[i].distance[j]) / 500.0);
			double intensity = ((double)velodyne_message->partial_scan[i].intensity[j]) / 100.0;
			if (range > 0 && range < 200) // this causes n_points to become wrong (needs later correction)
			{
				tf::Point point = spherical_to_cartesian(h, v, range);
				data[line * 4] = (float)(round(point.x() * 1000.0) / 1000.0);
				data[(line * 4) + 1] = (float)(round(point.y() * 1000.0) / 1000.0);
				data[(line * 4) + 2] = (float)(round(point.z() * 1000.0) / 1000.0);
				data[(line * 4) + 3] = (float)(round(intensity * 100.0) / 100.0);
			}
			else
			{
				data[line * 4] = 0.0;
				data[(line * 4) + 1] = 0.0;
				data[(line * 4) + 2] = 0.0;
				data[(line * 4) + 3] = 0.0;
			}
		}
	}
	rangenet_infer(data, num_points, scan);
}

void velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message);
	pointcloud_to_rangenet(velodyne_message);
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	initialize_rangenet();

	carmen_velodyne_subscribe_partial_scan_message(NULL,
												   (carmen_handler_t)velodyne_partial_scan_message_handler,
												   CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}
