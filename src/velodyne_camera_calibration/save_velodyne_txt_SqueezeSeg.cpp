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
    double timestamp = velodyne_message->timestamp;
    ofstream point_cloud_file;
    point_cloud_file.open("SqueezeSegV2/" + std::to_string(timestamp) + ".txt");
    // int n_points = 32 * velodyne_message->number_of_32_laser_shots;
    point_cloud_file << "#Array shape: (32, 16384, 5)\n";
    int i, j, k;
    int max_k = 512;
    cout << velodyne_message->number_of_32_laser_shots << " numero de capturas\n";
    for (j = 32; j > 0; j--)
    {
        for (i = 0, k = 0; i < velodyne_message->number_of_32_laser_shots && k < max_k; i++)
        {

            double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[j]));
            double h = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
            double range = (((double)velodyne_message->partial_scan[i].distance[j]) / 500.0);
            double intensity = ((double)velodyne_message->partial_scan[i].intensity[j]) * 10;
            double angle = velodyne_message->partial_scan[i].angle;
            // Catch points in frontal angle
            if (angle < 90 || angle > 270)
            {
                if (range > 0 && range < 200) // this causes n_points to become wrong (needs later correction)
                {
                    tf::Point point = spherical_to_cartesian(h, v, range);
                    double x = round(point.x() * 100.0) / 100.0;
                    double y = round(point.y() * 100.0) / 100.0;
                    double z = round(point.z() * 100.0) / 100.0;
                    intensity = intensity / 1000.0;
                    intensity = round(intensity * 100.0) / 100.0;
                    // range = round( range * 100.0) / 100.0;
                    double raiz_soma_quadrados = sqrt(x * x + y * y + z * z);
                    point_cloud_file << std::fixed << std::setprecision(2) << x << "\t" << y << "\t" << z << "\t" << intensity << "\t" << raiz_soma_quadrados << "\n";
                }
                else
                {
                    // Couldnt find a point, so, zeros.
                    point_cloud_file << "0.00\t0.00\t0.00\t0.00\t0.00\n";
                }
                k += 1;
            }
        }
        if (k % max_k == 0 && k > 0)
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
