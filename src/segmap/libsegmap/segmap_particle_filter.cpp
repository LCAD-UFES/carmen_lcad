
#include "segmap_particle_filter.h"
#include <cassert>

double
ParticleFilter::_gauss()
{
	return _std_normal_distribution(_random_generator);
}


// public:
ParticleFilter::ParticleFilter(int n_particles, double x_std, double y_std, double th_std,
		double v_std, double phi_std, double pred_x_std, double pred_y_std, double pred_th_std,
		double color_var_r, double color_var_g, double color_var_b)
{
	_n = n_particles;

	_p = (Pose2d *) calloc (_n, sizeof(Pose2d));
	_w = (double *) calloc (_n, sizeof(double));

	_p_bef = (Pose2d *) calloc (_n, sizeof(Pose2d));
	_w_bef = (double *) calloc (_n, sizeof(double));

	_x_std = x_std;
	_y_std = y_std;
	_th_std = th_std;
	_v_std = v_std;
	_phi_std = phi_std;

	_pred_x_std = pred_x_std;
	_pred_y_std = pred_y_std;
	_pred_th_std = pred_th_std;

	_color_var_r = color_var_r / pow(255., 2.);
	_color_var_g = color_var_g / pow(255., 2.);
	_color_var_b = color_var_b / pow(255., 2.);
}


ParticleFilter::~ParticleFilter()
{
	free(_p);
	free(_w);
	free(_p_bef);
	free(_w_bef);
}


void
ParticleFilter::seed(int val)
{
	srand(val);
	_random_generator.seed(val);
}


void
ParticleFilter::reset(double x, double y, double th)
{
	for (int i = 0; i < _n; i++)
	{
		_p[i].x = x + _gauss() * _x_std;
		_p[i].y = y + _gauss() * _y_std;
		_p[i].th = th + _gauss() * _th_std;

		_w[i] = _w_bef[i] = 1. / (double) _n;
		_p_bef[i] = _p[i];
	}

	best = _p[0];
}


void
ParticleFilter::predict(double v, double phi, double dt)
{
	double noisy_v, noisy_phi;

	for (int i = 0; i < _n; i++)
	{
		noisy_v = v;
		noisy_phi = phi;

		noisy_v += _gauss() * _v_std;
		noisy_phi = normalize_theta(noisy_phi + _gauss() * _phi_std);

		_p[i].x += noisy_v * dt * cos(_p[i].th);
		_p[i].y += noisy_v * dt * sin(_p[i].th);
		_p[i].th += noisy_v * dt * tan(noisy_phi) / distance_between_front_and_rear_axles;

		_p[i].x += _gauss() * _pred_x_std;
		_p[i].y += _gauss() * _pred_y_std;
		_p[i].th += _gauss() * _pred_th_std;
		_p[i].th = normalize_theta(_p[i].th);

		_p_bef[i] = _p[i];
		_w_bef[i] = _w[i];
	}

	best = _p[0];
}


double
ParticleFilter::_semantic_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
{
	double count;
	double unnorm_log_prob = 0.;
	PointXYZRGB point;

	for (int i = 0; i < transformed_cloud->size(); i++)
	{
		point = transformed_cloud->at(i);
		vector<double> v = map.read_cell(point);

		// cell observed at least once.
		// TODO: what to do when the cell was never observed?
		count = v[v.size() - 2];
		assert(count != 0);
		// add the log probability of the observed class.
		unnorm_log_prob += log(v[point.r] / count);
	}

	return unnorm_log_prob;
}


double
ParticleFilter::_image_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
{
	double n_valid_cells = 0;
	double unnorm_log_prob = 0.;
	PointXYZRGB point;

	for (int i = 0; i < transformed_cloud->size(); i++)
	{
		point = transformed_cloud->at(i);
		vector<double> v = map.read_cell(point);

		// cell observed at least once.
		// TODO: what to do when the cell was never observed?
		if (v[0] != -1)
		{
			unnorm_log_prob += fabs((point.r - v[2]) / 255.) +
						fabs((point.g - v[1]) / 255.) +
						fabs((point.b - v[0]) / 255.);

			n_valid_cells += 1;
		}
	}

	double c = 10.;  // constant to magnify the particle weights.
	unnorm_log_prob /= (3. * n_valid_cells);
	return exp(-c * unnorm_log_prob);
}


void
ParticleFilter::correct(PointCloud<PointXYZRGB>::Ptr cloud,
		GridMap &map, PointCloud<PointXYZRGB>::Ptr transformed_cloud,
		Matrix<double, 4, 4> &vel2car,
		double v, double phi)
{
	int i;
	double sum_weights, min_weight;

	int max_id = 0;
	int min_id = 0;

	fprintf(stderr, "DEBUG: Unormalized particle weights: ");
	for (i = 0; i < _n; i++)
	{
		//Matrix<double, 4, 4> tr = Pose2d::to_matrix(_p[i]) * vel2car;
		//transformPointCloud(*cloud, *transformed_cloud, tr);
		transform_pointcloud(cloud, transformed_cloud, _p[i], vel2car, v, phi);

		if (map._map_type == GridMapTile::TYPE_SEMANTIC)
			_w[i] = _semantic_weight(transformed_cloud, map);
		else
			_w[i] = _image_weight(transformed_cloud, map);

		_p_bef[i] = _p[i];

		if (_w[i] > _w[max_id])
			max_id = i;

		if (_w[i] < _w[min_id])
			min_id = i;

		fprintf(stderr, "%.4lf ", _w[i]);
	}
	fprintf(stderr, "\n");

	best = _p[max_id];
	min_weight = _w[min_id];
	sum_weights = 0.;

	for (i = 0; i < _n; i++)
	{
		_w[i] -= min_weight;
		sum_weights += _w[i];
	}

	//transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(_p[max_id]));
	//for(int i = 0; i < transformed_cloud->size(); i++)
	//{
	//	transformed_cloud->at(i).r = 255;
	//	transformed_cloud->at(i).g = 0;
	//	transformed_cloud->at(i).b = 0;
	//	map.add_point(transformed_cloud->at(i));
	//}

	//printf("Sum weights: %lf\n", sum_weights);

	// normalize the probabilities
	fprintf(stderr, "DEBUG: Normalized particle weights: ");
	for (i = 0; i < _n; i++)
	{
		if (sum_weights == 0)
			_w[i] = 1. / (double) _n;
		else
			_w[i] /= sum_weights;

		_w_bef[i] = _w[i];

		fprintf(stderr, "%.4lf ", _w[i]);
	}
	fprintf(stderr, "\n");

	// resample
	_p[0] = best; // elitism
	for (i = 1; i < _n; i++)
	{
		double unif = (double) rand() / (double) RAND_MAX;
		int pos = rand() % _n;
		double sum = _w[pos];

		while (sum < unif)
		{
			pos++;

			if (pos == _n)
				pos = 0;

			sum += _w[pos];
		}

		_p[i] = _p_bef[pos];
	}

	// make all particles equally likely after resampling
	for (i = 0; i < _n; i++)
		_w[i] = 1. / (double) _n;
}


Pose2d
ParticleFilter::mean()
{
	Pose2d p;

	// th_y and th_x are used to compute the mean of theta.
    // Note: The circular mean is different than the arithmetic mean.
    // For example, the arithmetic mean of the three angles 0°, 0° and 90° is
	// (0+0+90)/3 = 30°, but the vector mean is 26.565°. The difference is bigger
	// when the angles are widely distributed. If the angles are uniformly distributed
	// on the circle, then the resulting radius will be 0, and there is no circular mean.
	// (In fact, it is impossible to define a continuous mean operation on the circle.)
    // See https://en.wikipedia.org/wiki/Mean_of_circular_quantities
	// See https://rosettacode.org/wiki/Averages/Mean_angle
	double th_y = 0., th_x = 0.;

	for (int i = 0; i < _n; i++)
	{
		p.x += (_p[i].x * _w[i]);
		p.y += (_p[i].y * _w[i]);

		th_y += (sin(_p[i].th) * _w[i]);
		th_x += (cos(_p[i].th) * _w[i]);
	}

	p.th = atan2(th_y, th_x);
	return p;
}


Pose2d
ParticleFilter::mode()
{
	return best;
}


