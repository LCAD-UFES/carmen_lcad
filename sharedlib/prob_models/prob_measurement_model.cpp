#include <cmath>
#include <carmen/carmen.h>
#include "prob_measurement_model.h"
#include "prob_map.h"
#include "prob_transforms.h"
#include <assert.h>

static BeanRangeFinderMeasurementModelParams measurement_model_params;
static MeasurementModelTypes measurement_model_type = BeanRangeFinderModel;


/**
 * Computes the integral of the normal distribution N(u; expected_value_of_X, standard_deviation_of_X) from -infinity to u, i.e. the Cumulative Normal Distribution.
 * @param u Value of X at the upper limit of the integral.
 * @param average_of_X Average of X.
 * @param standard_deviation_of_X Standard deviation of X.
 * @returns Cumulative Normal Distribution up to u.
 */
double 
cumulative_normal_distribution(double u, double average_of_X, double standard_deviation_of_X)
{
	// See http://www.sitmo.com/doc/Calculating_the_Cumulative_Normal_Distribution and
	// http://en.wikipedia.org/wiki/Normal_distribution and
	// http://www.math.sfu.ca/~cbm/aands/page_932.htm
	const double b1 = 0.319381530;
	const double b2 = -0.356563782;
	const double b3 = 1.781477937;
	const double b4 = -1.821255978;
	const double b5 = 1.330274429;
	const double p = 0.2316419;
	const double c = 0.39894228;
	double x;

	x = (u - average_of_X) / standard_deviation_of_X;
	if (x >= 0.0)
	{
		double t = 1.0 / (1.0 + p * x);
		return (1.0 - c * exp(-x * x / 2.0) * t * (t * (t * (t * (t * b5 + b4) + b3) + b2) + b1));
	}
	else
	{
		double t = 1.0 / (1.0 - p * x);
		return (c * exp(-x * x / 2.0) * t * (t * (t * (t * (t * b5 + b4) + b3) + b2) + b1));
	}
}


/**
 * Computes the N(x; expected_value_of_X, standard_deviation_of_X), i.e., the value at x of the Normal distribution
 * of the random variable X with mean expected_value_of_X and variance equal to standard_deviation_of_X*standard_deviation_of_X.
 * @param x value x of the random variable.
 * @param expected_value_of_X Mean of the random variable.
 * @param standard_deviation_of_X Standard deviation of the random variable X.
 * @returns Value of the Normal at x
 *
 */
double 
normal_distribution(double x, double expected_value_of_X, double standard_deviation_of_X)
{
	double normal, n, a, b;

	n = 1.0 / (standard_deviation_of_X * sqrt(2.0 * M_PI));
	a = pow(x - expected_value_of_X, 2.0);
	b = 2.0 * pow(standard_deviation_of_X, 2.0);
	normal = n * exp(-a / b);

	return normal;
}


double 
phit(double ztk, double ztk_star)
{
	double n;

	if ((0.0 <= ztk) && (ztk < measurement_model_params.max_range))
	{
		n = 1.0 / (cumulative_normal_distribution(measurement_model_params.max_range, ztk, measurement_model_params.sigma_zhit) -
			   cumulative_normal_distribution(0, ztk, measurement_model_params.sigma_zhit));
		return (n * normal_distribution(ztk, ztk_star, measurement_model_params.sigma_zhit));
	}
	else
		return 0.0;
}


double 
prand(double ztk, double zmax)
{
	if ((0.0 <= ztk) && (ztk < zmax))
		return (1.0 / zmax);
	else
		return 0.0;
}


double 
pmax(double ztk, double zmax)
{
	if (ztk >= zmax)
		return 1.0;
	else
		return 0.0;
}


double 
pshort(double ztk, double ztk_star)
{
	double n, p;

	if ((0.0 <= ztk) && (ztk <= ztk_star))
	{
		n = 1.0 / (1.0 - exp(-measurement_model_params.lambda_short * ztk_star));
		p = n * measurement_model_params.lambda_short * exp(-measurement_model_params.lambda_short * ztk);
		if (p > 1.0)
			return (1.0);
		return (p);
	}
	else
		return 0.0;
}


double 
beam_range_finder_model_probability(double ztk_star, double ztk)
{
	double p;

	p = measurement_model_params.zhit * phit(ztk, ztk_star);
	p += measurement_model_params.zshort * pshort(ztk, ztk_star);
	p += measurement_model_params.zmax * pmax(ztk, measurement_model_params.max_range);
	p += measurement_model_params.zrand * prand(ztk, measurement_model_params.max_range);

	return p;
}


/**
 * This algorithm computes the likelihood of a rage scan zt given a pose xt and a map.
 * It assumes independence between the individual range measurements in the scan.
 * @See Table 6.1 of the book Probabilistic Robotics.
 */
double 
carmen_beam_range_finder_measurement_model(double *zt, carmen_point_t *xt, carmen_map_t *map)
{
	carmen_point_t zt_pose;
	double ztk_star, ztk;
	double p;
	double q = 1.0;

	assert(measurement_model_type == BeanRangeFinderModel);

	transform_robot_pose_to_laser_pose(&zt_pose, xt);

	for (int k = 0; k < measurement_model_params.laser_beams; k += measurement_model_params.sampling_step)
	{
		ztk = zt[k];
		ztk_star = carmen_ray_cast(zt_pose, k, map, measurement_model_params.max_range, measurement_model_params.start_angle, measurement_model_params.angle_step);
		if (ztk > measurement_model_params.max_range)
			ztk = measurement_model_params.max_range;
		if (ztk_star > measurement_model_params.max_range)
			ztk_star = measurement_model_params.max_range;

		p = beam_range_finder_model_probability(ztk_star, ztk);
		q = q * p;
	}
	return q;
}


void 
init_bean_range_finder_measurement_model(BeanRangeFinderMeasurementModelParams params)
{
	measurement_model_type = BeanRangeFinderModel;

	measurement_model_params = params;
	measurement_model_params.angle_step = measurement_model_params.fov_range / (double) (measurement_model_params.laser_beams - 1);

	tf_transform_robot_pose_to_laser_pose_initialize(&params);
}

