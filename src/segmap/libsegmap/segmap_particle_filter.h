
#ifndef _SEGMAP_PARTICLE_FILTER_H_
#define _SEGMAP_PARTICLE_FILTER_H_


#include <random>
#include <vector>
#include <cstdlib>
#include <pcl/io/io.h>
#include "segmap_car_config.h"
#include "segmap_grid_map.h"
#include "segmap_pose2d.h"
#include "segmap_util.h"


using namespace std;
using namespace pcl;


class ParticleFilter
{
public:
	int _n;
	Pose2d *_p;
	double *_w;

	// p and w before resampling
	Pose2d *_p_bef;
	double *_w_bef;

	double _x_std, _y_std, _th_std, _v_std, _phi_std;
	double _pred_x_std, _pred_y_std, _pred_th_std;
	double _color_var_r, _color_var_g, _color_var_b;

	std::default_random_engine _random_generator;
	std::normal_distribution<double> _std_normal_distribution;

	Pose2d best;

	double _gauss();
	double _semantic_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map);
	double _image_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map);
	void _compute_weights(PointCloud<PointXYZRGB>::Ptr cloud,
			GridMap &map, PointCloud<PointXYZRGB>::Ptr transformed_cloud,
			Matrix<double, 4, 4> &vel2car,
			double v, double phi, int *max_id, int *min_id);
	void _normalize_weights(int min_id, int max_id);
	void _resample();

	// public:
	ParticleFilter(int n_particles, double x_std, double y_std, double th_std,
			double v_std, double phi_std,
			double pred_x_std, double pred_y_std, double pred_th_std,
			double color_var_r, double color_var_g, double color_var_b);

	~ParticleFilter();

	void seed(int val);
	void reset(double x, double y, double th);

	void predict(double v, double phi, double dt);
	double sensor_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map);
	void correct(PointCloud<PointXYZRGB>::Ptr cloud, GridMap &map, PointCloud<PointXYZRGB>::Ptr transformed_cloud,
			Matrix<double, 4, 4> &vel2car, double v, double phi);

	Pose2d mean();
	Pose2d mode();
};


#endif
