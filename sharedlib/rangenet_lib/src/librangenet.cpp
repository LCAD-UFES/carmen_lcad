#include "rangenet_inference.h"

// standalone lib h
#include "librangenet.h"

#include <vector>
#include <carmen/carmen.h>
#include <prob_map.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>

void
librangenet_initialize()
{
	initialize_rangenet();
}

int *
librangenet_process_moving_obstacles_cells(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params)
{
	int *rangenet_segmented;
	double timestamp = velodyne_message->timestamp;
	std::string scan = std::to_string(timestamp);
	int shots_to_squeeze = velodyne_message->number_of_32_laser_shots;
	int vertical_resolution = sensors_params[sensor_number].vertical_resolution;
	uint32_t num_points = vertical_resolution * velodyne_message->number_of_32_laser_shots;
	std::vector<float> data(4 * num_points);
	int line = 0;
	int i, j;
	for (j = 0; j < vertical_resolution; j++)
	{
		for (i = 0; i < shots_to_squeeze; i++, line++)
		{
			double vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sensors_params[sensor_number].vertical_correction[j]));
			double horizontal_angle = carmen_normalize_theta(-carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
			double range = (((double)velodyne_message->partial_scan[i].distance[sensors_params[sensor_number].ray_order[j]]) / 500.0);
			double intensity = ((double)velodyne_message->partial_scan[i].intensity[sensors_params[sensor_number].ray_order[j]]) / 100.0;
			if(range > 0 && range < 200) // this causes n_points to become wrong (needs later correction)
			{
				tf::Point point = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
				data[line * 4]= (float) (round(point.x() * 1000.0) / 1000.0);
				data[(line * 4) + 1] = (float) (round(point.y() * 1000.0) / 1000.0);
				data[(line * 4) + 2] = (float) (round(point.z() * 1000.0) / 1000.0);
				data[(line * 4) + 3] = (float) (round(intensity * 100.0) / 100.0);
			}else{
				data[line * 4] = 0.0;
				data[(line * 4) + 1] = 0.0;
				data[(line * 4) + 2] = 0.0;
				data[(line * 4) + 3] = 0.0;
			}
		}
	}
	rangenet_segmented = rangenet_process_point_cloud(data, num_points, scan);

	return rangenet_segmented;
}

