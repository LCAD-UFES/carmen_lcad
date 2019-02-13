
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


std::vector<double> make_xyz_lut() 
{
    const int n = W * H;
    std::vector<double> xyz = std::vector<double>(3 * n, 0);

    for (int icol = 0; icol < W; icol++) 
    {
        double h_angle_0 = 2.0 * M_PI * icol / W;
        
        for (int ipx = 0; ipx < H; ipx++) 
        {
            int ind = 3 * (icol * H + ipx);
            double h_angle = std::sin(ouster64_azimuth_angles[ipx] * 2 * M_PI / 360.0) + h_angle_0;
            xyz[ind + 0] = std::cos(ouster64_altitude_angles[ipx] * 2 * M_PI / 360.0) * std::cos(h_angle);
            xyz[ind + 1] = -std::cos(ouster64_altitude_angles[ipx] * 2 * M_PI / 360.0) * std::sin(h_angle);
            xyz[ind + 2] = std::sin(ouster64_altitude_angles[ipx] * 2 * M_PI / 360.0);
        }
    }

    return xyz;
}


void
variable_scan_handler(carmen_velodyne_variable_scan_message *message)
{
    viewer->removeAllPointClouds();

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    for (int i = 0; i < message->number_of_shots; i++)
    {
        for (int j = 0; j < message->partial_scan[i].shot_size; j++)
        {
            double range = message->partial_scan[i].distance[j] / 1000.;
            double h_angle = std::sin(carmen_degrees_to_radians(ouster64_azimuth_angles[j])) + message->partial_scan[i].angle;
            double v_angle = carmen_degrees_to_radians(ouster64_altitude_angles[j]);

            pcl::PointXYZRGB point;

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
}


int 
main(int argc, char** argv)
{
    carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

    xyz_lookup_table = make_xyz_lut();    
    viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
    viewer->setBackgroundColor(0, 0, 1);

    carmen_velodyne_subscribe_variable_scan_message(NULL, 
        (carmen_handler_t) variable_scan_handler, 
        CARMEN_SUBSCRIBE_LATEST,
        ouster_sensor_id);

    carmen_ipc_dispatch();
    return 0;
}

