// Saves pointclouds from the log in txt files
// First line: number of points (needs later correction)
// Other lines: x y z intensity (of each point)

#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <string>
#include <carmen/velodyne_camera_calibration.h>
#include <tf.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>     /* getenv */
#include <carmen/libsqueeze_seg.h>

using namespace tf;

const static double sorted_vertical_angles[32] =
    {
        -30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
        -18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
        -6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
        5.3299999, 6.6700001, 8.0, 9.3299999, 10.67};
inline double round(double val)
{
    if (val < 0)
        return ceil(val - 0.5);
    return floor(val + 0.5);
}

void write_pointcloud_txt(carmen_velodyne_partial_scan_message *velodyne_message)
{
    int i, j, line;
    int vertical_resolution = 32;
    double timestamp = velodyne_message->timestamp;

    ofstream point_cloud_file;
    char *pPath = getenv ("CARMEN_HOME");
    point_cloud_file.open( std::string(pPath) + "/sharedlib/libsqueeze_seg_v2/data/samples_IARA/" + std::to_string(timestamp) + ".pcd");
    // point_cloud_file << "# Array shape: (32, " << velodyne_message->number_of_32_laser_shots << ", 5)\n";
    point_cloud_file << "# .PCD v.7 - Point Cloud Data file format\n";
    point_cloud_file << "VERSION .7\nFIELDS x y z intensity range\nSIZE 4 4 4 4 4\n";
    point_cloud_file << "TYPE F F F F F\nCOUNT 1 1 1 1 1\n";
    point_cloud_file << "WIDTH " + std::to_string(velodyne_message->number_of_32_laser_shots) + "\n";
    point_cloud_file << "HEIGHT " + std::to_string(vertical_resolution) + "\n";
    point_cloud_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    point_cloud_file << "POINTS " + std::to_string(vertical_resolution * velodyne_message->number_of_32_laser_shots) +"\n";
    point_cloud_file << "DATA ascii\n";

    //std::cout << "SqueezeSeg: pointCloud file in $CARMEN_HOME/sharedlib/salsanet/data/" << std::to_string(timestamp) << ".txt" << std::endl;
    for (j = vertical_resolution, line = 0; j > 0; j--)
    {
        for (i = 0; i <  velodyne_message->number_of_32_laser_shots; i++, line++)
        {
            double vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[j]));
            double horizontal_angle = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
            double range = (((double)velodyne_message->partial_scan[i].distance[j]) / 500.0);
            double intensity = ((double)velodyne_message->partial_scan[i].intensity[j]) / 100.0;
            if (range > 0 && range < 200)
            {
                tf::Point point = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
		        double x = round(point.x() * 100.0) / 100.0;
		        double y = round(point.y() * 100.0) / 100.0;
		        double z = round(point.z() * 100.0) / 100.0;
		        intensity = round(intensity * 100.0) / 100.0;
                point_cloud_file << x << "\t" << y << "\t" << z << "\t" << intensity << "\t" << range << "\n";
            } else {
                point_cloud_file << "0.00\t0.00\t0.00\t0.00\t0.00\n";
            }
        }
        point_cloud_file << "# New slice\n";
    }
    point_cloud_file.close();
}

void velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
    carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message);
    write_pointcloud_txt(velodyne_message);
}

int main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);

    carmen_velodyne_subscribe_partial_scan_message(NULL,
                                                   (carmen_handler_t)velodyne_partial_scan_message_handler,
                                                   CARMEN_SUBSCRIBE_LATEST);

    carmen_ipc_dispatch();

    return 0;
}
