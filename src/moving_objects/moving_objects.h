#include "monte_carlo_moving_objects_tracking.h"
#include "moving_objects_messages.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <list>
#include <math.h>
#include <prob_map.h>
#include <prob_measurement_model.h>
#include <carmen/carmen.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_velodyne.h>
#include <tf.h>
#include <algorithm>
#include <time.h>

using namespace std;

#ifndef MOVING_OBJECTS_H
#define MOVING_OBJECTS_H

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////
//particle filter fixed parameters
////////////////////////////
#define MAXIMUM_HEIGHT_OF_OBSTACLE 2.5
const int num_of_particles = 300;

/* Thresholds for association */
const double threshold_association_dist = 2.5; //minimum distance between point clouds centroid used for association
const double threshold_max_dist_from_car = 25.0;
const double threshold_min_velocity = 3.0;
extern int num_of_models;
extern std::vector<object_model_features_t> object_models;

/* Thresholds for static objects analysis */
const double threshold_occupancy_rate = 0.5;
const double threshold_points_in_occupied_grid_rate = 0.5;


///////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _particle
{
	carmen_point_t    pose;
	double            velocity;
	double            weight;
	double            dist;
	int               class_id;
	object_model_features_t model_features;
	double            timestamp;
} particle_datmo;


struct _color_palette_and_association_data
{
	Eigen::Vector3f color_palette;
	int				num_color;
	int				num_association;
};

typedef struct _color_palette_and_association_data color_palette_and_association_data_t;

struct _object_point_cloud_data
{
	pcl::PointCloud<pcl::PointXYZ>	point_cloud;
	int								num_color_associate; // association and color
	int								label_associate; // label of association point cloud - 0: not associated; 1: associated
	carmen_pose_3D_t				car_global_pose;
	Eigen::Vector4f 				centroid;
	double							distance_object_pose_and_car_global_pose;
	double							linear_velocity;
	carmen_pose_3D_t				object_pose;
	object_geometry_t				geometry;
	int								geometric_model; // geometric model of the object - pedestrian: 0, motorcycle / bike/ car: 1, truck: 2, unknown: -1.
	object_model_features_t         model_features;
	double							object_density;
	double							orientation;
	double							delta_time_t_and_t_1;
	std::vector<particle_datmo>     particle_set;
	double							timestamp;
};

typedef struct _object_point_cloud_data object_point_cloud_data_t;


struct _moving_objects_input_data
{
	int					           num_velodyne_point_clouds;
	carmen_pose_3D_t	           car_fused_pose;
	carmen_vector_3D_t	           car_fused_velocity;
	double 				           car_fused_time;
	double 				           car_phi;
	carmen_pose_3D_t 	           velodyne_pose;
	carmen_pose_3D_t               sensor_board_1_pose;
	rotation_matrix                *sensor_board_1_to_car_matrix;
	int 				           number_of_sensors;
	double 				           robot_wheel_radius;
	carmen_robot_ackerman_config_t car_config;
	double                         safe_range_above_sensors;
	double 				           highest_sensor;
	int					           first_offline_map_message;
	carmen_pose_3D_t	           car_global_pose;
	int                            **indices_clusters;
};

typedef struct _moving_objects_input_data moving_objects_input_data_t;


void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points,
		int max_point_buffer);

int
detect_points_above_ground_in_vertical_beam(int i, const moving_objects_input_data_t &moving_objects_input, sensor_data_t *velodyne_data,
		sensor_parameters_t *velodyne_params, carmen_vector_3D_t *point_clouds, int last_num_points);

int
detect_points_above_ground(sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data, rotation_matrix *r_matrix_car_to_global,
		carmen_pose_3D_t *robot_pose, carmen_vector_3D_t *robot_velocity, double x_origin, double y_origin, int point_cloud_index,
		double phi, moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *point_clouds);

int
build_point_cloud_using_velodyne_message(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi, moving_objects_input_data_t moving_objects_input,
		carmen_vector_3D_t *point_clouds);

std::list<object_point_cloud_data_t>
detect_and_follow_moving_objects(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi, moving_objects_input_data_t moving_objects_input,
		carmen_vector_3D_t *point_clouds, carmen_map_p & occupancy_grid_map);

object_geometry_t
get_geom_based_on_class(int particle_class);

int
get_random_model_id(int);

object_model_features_t
get_obj_model_features(int model_id);

std::list<object_point_cloud_data_t>
get_list_point_clouds();

std::list<color_palette_and_association_data_t>
get_color_palette_and_association();

#ifdef __cplusplus
}
#endif

#endif
