/*
 * graphslam_params.h
 *
 *  Created on: Oct 10, 2013
 *      Author: filipe
 */

#ifndef _GRAPHSLAM_PARAMS_H_
#define _GRAPHSLAM_PARAMS_H_

	#include <carmen/carmen.h>
	#include <prob_map.h>
	#include <carmen/stereo_velodyne.h>
	#include <carmen/localize_ackerman_velodyne.h>

	void init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intesity);
	void get_alive_sensors(int argc, char **argv);
	int *generates_ray_order(int size);
	void get_sensors_param(int argc, char **argv);
	void read_parameters(int argc, char **argv, carmen_localize_ackerman_param_p param, ProbabilisticMapParams *p_map_params);
	void read_parameters_without_mapper(int argc, char **argv, carmen_localize_ackerman_param_p param, ProbabilisticMapParams *p_map_params, char *remission_file);

#endif /* _GRAPHSLAM_PARAMS_H_ */
