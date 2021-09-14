#ifndef LOCALIZE_ACKERMAN_VELODYNE_H
#define LOCALIZE_ACKERMAN_VELODYNE_H

#define NUM_VELODYNE_POINT_CLOUDS	5


int
localize_ackerman_velodyne_partial_scan_build_instanteneous_maps(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, double v, double phi);

int
localize_ackerman_velodyne_partial_scan_build_instanteneous_maps(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, double v, double phi,
		double map_center_x, double map_center_y);

int
localize_ackerman_variable_scan_build_instanteneous_maps(carmen_velodyne_variable_scan_message *msg, sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, double v, double phi);

int
localize_ackerman_variable_scan_build_instanteneous_maps(carmen_velodyne_variable_scan_message *msg, sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, double v, double phi,
		double map_center_x, double map_center_y);

int
localize_ackerman_velodyne_variable_scan_build_instanteneous_maps(carmen_velodyne_variable_scan_message *message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, double v, double phi);

int
localize_ackerman_velodyne_variable_scan_build_instanteneous_maps(carmen_velodyne_variable_scan_message *message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, double v, double phi,
		double map_center_x, double map_center_y);


void
localize_ackerman_velodyne_publish_frontlaser(double timestamp, double *laser_ranges);

carmen_laser_laser_message *
localize_ackerman_velodyne_create_frontlaser_message(double timestamp, double *laser_ranges);

void
localize_ackerman_velodyne_laser_read_parameters(int argc, char **argv);

#endif // LOCALIZE_ACKERMAN_VELODYNE_H
