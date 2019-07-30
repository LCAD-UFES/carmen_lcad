
#include "ouster_config.h"
#include <carmen/velodyne_interface.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

using namespace pcl;


static int ouster_sensor_id = 0;
pcl::visualization::PCLVisualizer *viewer = NULL;
std::vector<double> xyz_lookup_table;


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("ouster_viewer: disconnected.\n");
		exit(0);
	}
}


int 
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
            {(char*) "commandline", (char*) "sensor_id", CARMEN_PARAM_INT, &ouster_sensor_id, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


void
variable_scan_handler(carmen_velodyne_variable_scan_message *message)
{
    viewer->removeAllPointClouds();

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    for (int i = 0; i < message->number_of_shots; i += 1)
    {
        for (int j = 0; j < message->partial_scan[i].shot_size; j += 1)
        {
            double range = message->partial_scan[i].distance[j] / 1000.;
            // In the original example code from ouster, the horizontal angle was summed with the sin of 
            // the offset. However, the section in the manual that describes how to convert the sensor readings to cartesian  
            // coordinates, the angle is summed with the offset without the sin. As the manual's calculations seem more 
            // correct, I use them.
            // double h_angle = message->partial_scan[i].angle + std::sin(carmen_degrees_to_radians(ouster64_azimuth_offsets[j]));
            double h_angle = carmen_normalize_theta(message->partial_scan[i].angle + carmen_degrees_to_radians(ouster64_azimuth_offsets[j]));
            double v_angle = carmen_degrees_to_radians(ouster64_altitude_angles[j]);

            pcl::PointXYZRGB point;

            // In the original example code from ouster, they store the values multiplied by the range 
            // in a lookup table for efficiency. When a point clouds arrives, they only multiply the range by 
            // the value stored in the lookup table. The lookup table can be created because the horizontal and 
            // vertical angles are fixed accross readings.
            point.x = range * std::cos(v_angle) * std::cos(h_angle);
            point.y = range * (-std::cos(v_angle) * std::sin(h_angle));
            point.z = range * std::sin(v_angle);
            point.r = message->partial_scan[i].intensity[j];
            point.g = message->partial_scan[i].intensity[j];
            point.b = message->partial_scan[i].intensity[j];

            cloud->push_back(point);
        }
    }

    viewer->addPointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->spinOnce();

    static int count = 0;
    double current_time = carmen_get_time();
    static double time_last_print = 0;

    count++;

    if (current_time - time_last_print > 1.0)
    {
        printf("VariableScan message received on %lf. Frequency: %d Hz.\n", 
            message->timestamp, count
        );

        count = 0;
        time_last_print = current_time;
    }
}


void
imu_handler(carmen_velodyne_gps_message *message)
{
    static int count = 0;
    static double time_last_print = 0;

    double current_time = carmen_get_time();

    count++;

    if (current_time - time_last_print > 1.0)
    {
        printf("IMU message received on %lf. Frequency: %d Hz Acc: %.2lf %.2lf %.2lf Gyro: %.2lf %.2lf %.2lf.\n", 
            message->timestamp, count,
            message->accel1_x, message->accel2_x, message->accel3_x,
            message->gyro1, message->gyro2, message->gyro3
        );

        count = 0;
        time_last_print = current_time;
    }
}


int 
main(int argc, char** argv)
{
    carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

    viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
    viewer->setBackgroundColor(0, 0, 1);

    carmen_velodyne_subscribe_gps_message(NULL, 
        (carmen_handler_t) imu_handler, 
        CARMEN_SUBSCRIBE_LATEST
    );

    carmen_velodyne_subscribe_variable_scan_message(NULL, 
        (carmen_handler_t) variable_scan_handler, 
        CARMEN_SUBSCRIBE_LATEST,
        ouster_sensor_id);

    carmen_ipc_dispatch();
    return 0;
}

