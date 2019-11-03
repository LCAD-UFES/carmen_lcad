#ifndef CARMEN_VELODYNE_INTERFACE_H
#define CARMEN_VELODYNE_INTERFACE_H

#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>
//#include <carmen/rotation_geometry.h>


#ifdef __cplusplus
extern "C"
{
#endif

double *carmen_velodyne_get_vertical_correction();


int *carmen_velodyne_get_ray_order();


double* carmen_velodyne_get_delta_difference_mean();


double* carmen_velodyne_get_delta_difference_stddev();

void
carmen_velodyne_subscribe_partial_scan_message(carmen_velodyne_partial_scan_message *message,
										 carmen_handler_t handler,
										 carmen_subscribe_t subscribe_how);


void
carmen_velodyne_unsubscribe_partial_scan_message(carmen_handler_t handler);


// set sensor_id to -1 to use the default message
void
carmen_velodyne_subscribe_variable_scan_message(carmen_velodyne_variable_scan_message *message,
										 carmen_handler_t handler,
										 carmen_subscribe_t subscribe_how,
										 int sensor_id);

// set sensor_id to -1 to use the default message
void
carmen_velodyne_unsubscribe_variable_scan_message(carmen_handler_t handler, int sensor_id);

// set sensor_id to -1 to use the default message
IPC_RETURN_TYPE
carmen_velodyne_publish_variable_scan_message(carmen_velodyne_variable_scan_message *message, int sensor_id);


void
carmen_velodyne_subscribe_gps_message(carmen_velodyne_gps_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void
carmen_velodyne_unsubscribe_gps_message(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_velodyne_publish_gps_message(carmen_velodyne_gps_message *message);


void
carmen_velodyne_define_messages();


void
carmen_velodyne_variable_scan_update_points(carmen_velodyne_variable_scan_message *message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max, double timestamp);

void
carmen_velodyne_partial_scan_update_points(carmen_velodyne_partial_scan_message *velodyne_message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max, double timestamp);

void
carmen_velodyne_partial_scan_update_points_with_remission_check(carmen_velodyne_partial_scan_message *velodyne_message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max, double timestamp, int use_remission);


double
carmen_velodyne_estimate_shot_time(double sensor_last_timestamp, double sensor_timestamp, int shot_index, int number_of_shots);


/* Note: the individual shots still have to be alloc'd using the alloc shot function */
carmen_velodyne_variable_scan_message *
carmen_velodyne_alloc_variable_velodyne_message_and_shots(int n_shots);


void
carmen_velodyne_alloc_shot(carmen_velodyne_shot *shot, int shot_size);


carmen_velodyne_variable_scan_message *
carmen_velodyne_copy_variable_velodyne_message(
		carmen_velodyne_variable_scan_message *message);


void
carmen_velodyne_free_variable_velodyne_message(
		carmen_velodyne_variable_scan_message *message);


#ifdef __cplusplus
}
#endif

#endif
// @}

