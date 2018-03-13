#ifndef LOCALIZE_ACKERMAN_VELODYNE_H
#define LOCALIZE_ACKERMAN_VELODYNE_H

#define NUM_VELODYNE_POINT_CLOUDS	5


int velodyne_odometry_compute_odometry(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, double v, double phi);

void localize_ackerman_velodyne_laser_read_parameters(int argc, char **argv);

#endif // LOCALIZE_ACKERMAN_VELODYNE_H
