#ifndef CARMEN_VELODYNE_INTERFACE_H
#define CARMEN_VELODYNE_INTERFACE_H

#include <carmen/velodyne_messages.h>
#include <carmen/carmen.h>
#include <carmen/rotation_geometry.h>


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

void
carmen_velodyne_subscribe_gps_message(carmen_velodyne_gps_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void
carmen_velodyne_unsubscribe_gps_message(carmen_handler_t handler);

void
carmen_velodyne_define_messages();

carmen_pose_3D_t
get_velodyne_pose_in_relation_to_car(int argc, char** argv);

void
carmen_velodyne_variable_scan_update_points(carmen_velodyne_variable_scan_message *message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max);

void
carmen_velodyne_partial_scan_update_points(carmen_velodyne_partial_scan_message *velodyne_message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max);


double
carmen_velodyne_estimate_shot_time(double sensor_last_timestamp, double sensor_timestamp, int shot_index, int number_of_shots);



#ifdef __cplusplus
}
#endif

#endif
// @}

