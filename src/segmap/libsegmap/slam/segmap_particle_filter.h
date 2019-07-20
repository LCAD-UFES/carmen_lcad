
#ifndef _SEGMAP_PARTICLE_FILTER_H_
#define _SEGMAP_PARTICLE_FILTER_H_

#include <random>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_preproc.h>


class ParticleFilter
{
public:
	enum WeightType
	{
		WEIGHT_GPS,
		WEIGHT_SEMANTIC,
		WEIGHT_VISUAL,
		WEIGHT_ECC
	};

	void _compute_weights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, GridMap &map,
	                      Pose2d &gps, int *max_id, int *min_id);

	void _compute_weights(DataSample *sample, GridMap *map, SensorPreproc &preproc, int *max_id, int *min_id);

	void _normalize_weights(int min_id, int max_id);
	void _resample();

	// public:
	ParticleFilter(int n_particles, WeightType weight_type,
	               double x_std, double y_std, double th_std,
	               double v_std, double phi_std,
	               double pred_x_std, double pred_y_std, double pred_th_std,
	               double color_var_r, double color_var_g, double color_var_b,
	               int ecc_n_bins = 64);

	~ParticleFilter();

	void seed(int val);
	void reset(double x, double y, double th);
	void reset(Pose2d &pose);

	void predict(double v, double phi, double dt);
	double sensor_weight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud, GridMap &map);
	void correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, GridMap &map, Pose2d &gps);
	void correct(DataSample *sample, GridMap *map, SensorPreproc &preproc);

	Pose2d mean();
	Pose2d mode();
	Pose2d std();

	int _n;
	Pose2d *_p;
	double *_w;

	// p and w before resampling
	Pose2d *_p_bef;
	double *_w_bef;

	double _x_std, _y_std, _th_std, _v_std, _phi_std;
	double _pred_x_std, _pred_y_std, _pred_th_std;
	double _color_std_r, _color_std_g, _color_std_b;

	std::default_random_engine _random_generator;
	std::normal_distribution<double> _std_normal_distribution;

	Pose2d best;

	WeightType _weight_type;

	double _gauss();
	double _semantic_weight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud, GridMap &map);
	double _image_weight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud, GridMap &map);
	double _gps_weight(Pose2d &pose, Pose2d &gps);

	double _compute_particle_weight(GridMap &instantaneous_map, GridMap &map, Pose2d &gps, Pose2d &particle_pose);
	double* _get_cell_value_in_offline_map(int cell_linearized_position_in_inst_map, GridMapTile *tile, GridMap &map,
	                                                   double cos_particle_th, double sin_particle_th, double particle_x, double particle_y);
	double _weight_between_cells(double *inst_cell, double *off_cell);

	double _semantic_point_weight(int class_id, std::vector<double> &cell);
	double _image_point_weight(double r, double g, double b, std::vector<double> &cell);
	double _image_point_weight(double r, double g, double b, double *cell);


	double _ecc_weight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud, GridMap &map);

	int _ecc_pixel_step;
	int _ecc_n_bins;
	double *_color_histogram;
	double *_map_histogram;
	double *_joint_histogram;
};


#endif
