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

//#define AJUSTE			// para ajuste dos parâmetros
#define BASELINE			// para o baseline

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////
//particle filter fixed parameters
////////////////////////////
#define MAXIMUM_HEIGHT_OF_OBSTACLE 2.5
const int num_of_particles = 400;

/* Thresholds for association */
const double threshold_association_dist = 2.5; //maximum distance between point clouds centroid used for association
const double threshold_max_dist_from_car = 25.0;
const double threshold_min_dist_from_car = 1.3;
const double threshold_min_velocity = 0.5;
const int threshold_idle_count = 4; //threshold for eliminating idle objects in association list

/* Thresholds for static objects analysis */
const double threshold_occupancy_rate = 0.5;
const double threshold_points_in_occupied_grid_rate = 0.1;

extern int frame;
extern int num_of_models;
extern std::vector<object_model_features_t> object_models;


///////////////////////////////////////////////////////////////////////////////////////////////

struct _object_features_3d
{
	int model_id;
//	char *model_type;
	object_geometry_t geometry;
	double red, green, blue;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

typedef struct _object_features_3d object_features_3d_t;

extern std::vector<object_features_3d_t> object_models_3d;

typedef struct _particle
{
	carmen_point_t pose;
	double velocity;
	double weight;
	double dist, norm_dist;
	int class_id;
	object_model_features_t model_features;
//	object_features_3d_t model_features_3d;
	double timestamp;
} particle_datmo_t;


struct _color_palette_and_association_data
{
	Eigen::Vector3f color_palette;
	int num_color;
	int num_association;
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
	std::vector<particle_datmo_t>   particle_set;
	int                             count_idle; // count how many frames object was frozen/stopped/idle
	particle_datmo_t                mean_particle; // mean particle of particle set
	bool							is_associated; // verify if point cloud object captured from velodyne was associated after association step
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


/* Structure used for ground truth measurement */
struct _ground_truth_objects_states
{
	double timestamp;
	carmen_pose_3D_t object_pose;
};

typedef struct _ground_truth_objects_states ground_truth_objects_states_t;

struct _ground_truth_object
{
	int obj_id;
	double width, length, height;
	int model_id; // Model Class ID: 0) cars; 11) bike/motorbike; 21) small truck; 31) bus; 41) pedestrian (only these at the moment)
	std::vector<ground_truth_objects_states_t> obj_states;
};

typedef struct _ground_truth_object ground_truth_object_t;


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

int
build_point_cloud_using_variable_velodyne_message(carmen_velodyne_variable_scan_message *velodyne_message,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity,
		double phi, moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *point_clouds);

std::list<object_point_cloud_data_t>
detect_and_follow_moving_objects_variable_scan(carmen_velodyne_variable_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi,
		moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *carmen_vector_3d_point_cloud,
		carmen_map_p & occupancy_grid_map);

object_geometry_t
get_geom_based_on_class(int particle_class);

int
get_random_model_id(int);

object_model_features_t
get_obj_model_features(int model_id);

object_features_3d_t
get_obj_model_features_3d(int model_id, vector<object_features_3d_t> object_models);

pcl::PointCloud<pcl::PointXYZ>::Ptr
get_model_point_cloud(int model_id, vector<object_features_3d_t> object_models);

std::list<object_point_cloud_data_t>
get_list_point_clouds();

std::list<color_palette_and_association_data_t>
get_color_palette_and_association();

void
print_object_details(object_point_cloud_data_t obj_point_cloud);

void
init_particle_set(object_point_cloud_data_t &object_pcloud, int num_particles, double x, double y, double num_models);

#ifdef __cplusplus
}
#endif

#endif
