#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <string>
#include <carmen/velodyne_camera_calibration.h>
#include <tf.h>
#include <iostream>
#include <fstream>

using namespace tf;

const static double sorted_vertical_angles[32] =
{
	-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
	-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
	-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
	5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
};

void
write_pointcloud_txt(carmen_velodyne_partial_scan_message *velodyne_message)
{
    double timestamp = velodyne_message->timestamp;
    ofstream point_cloud_file;
    point_cloud_file.open(std::to_string(timestamp) + ".txt");
    int n_points = 32*velodyne_message->number_of_32_laser_shots;
    point_cloud_file << n_points << "\n";
    int i, j;

    for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
    {
        for (j = 0; j < 32; j++)
        {
            double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[j]));
            double h = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
            double range = (((double) velodyne_message->partial_scan[i].distance[j]) / 500.0);
			double intensity = (((double) velodyne_message->partial_scan[i].intensity[j]));
            if(range > 0 && range < 200)
            {
                tf::Point point = spherical_to_cartesian(h, v, range);
                point_cloud_file << point.x() << " " << point.y() << " " << point.z() << " " << intensity << "\n";
            }
        }
    }
    point_cloud_file.close();
}

void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message);
    write_pointcloud_txt(velodyne_message);
}

int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
			(carmen_handler_t)velodyne_partial_scan_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}
