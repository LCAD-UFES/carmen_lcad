#ifndef MONTE_CARLO_MOVING_OBJECTS_TRACKING_H
#define MONTE_CARLO_MOVING_OBJECTS_TRACKING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <carmen/carmen.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/velodyne_interface.h>
#include <vector>
#include <list>
#include <math.h>

#include "moving_objects.h"

using namespace std;


////////////////////////////////////////////////////////////////////////////////////////////
/* Normalize factors for bounding box distance calculations */
const double norm_factor_x = 10.0;
const double norm_factor_y = 10.0;
const double norm_factor_z = 10.0;

const double alpha_1 =  1.0; // desvio padrão da velocidade padrão 0.2
const double alpha_2 =  0.1; // desvio padrão de theta padrão 0.01
const double v_min   =  0.0; // velocidade mínima
const double v_max   = 25.0; // velocidade máxima


////////////////////////////////////////////////////////////////////////////////////////////

std::vector<particle_datmo_t>
algorithm_monte_carlo(std::vector<particle_datmo_t> particle_set_t_1, double x, double y, double delta_time,
		pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, object_geometry_t, carmen_vector_3D_t car_global_position, int num_association);

double
measurement_model(particle_datmo_t &particle_t, double x, double y, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud,
		object_geometry_t obj_geometry, carmen_vector_3D_t car_global_position);

#endif

