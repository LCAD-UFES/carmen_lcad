#include "moving_objects3_particle_filter.h"


moving_objects3_particle_t
sample_motion_model(moving_objects3_particle_t particle_t_1, double delta_time)
{
	moving_objects3_particle_t particle_t;

	// compute actual velocity
	double velocity_t = particle_t_1.velocity + carmen_uniform_random(-MAX_ACCELERATION * delta_time, MAX_ACCELERATION * delta_time);

	// find delta_thetas
	double delta_theta_1 = carmen_uniform_random(-MAX_ANGULAR_VELOCITY * delta_time, MAX_ANGULAR_VELOCITY * delta_time);
	double delta_theta_2 = carmen_uniform_random(-MAX_ANGULAR_VELOCITY * delta_time, MAX_ANGULAR_VELOCITY * delta_time);

	// find motion distance
	double motion_distance = velocity_t * delta_time;

	// find delta_x and delta_y
	double delta_x = motion_distance * cos(particle_t_1.pose.theta + delta_theta_1);
	double delta_y = motion_distance * sin(particle_t_1.pose.theta + delta_theta_1);

	// generate new particle sample
	particle_t.pose.x = particle_t_1.pose.x + delta_x;
	particle_t.pose.y = particle_t_1.pose.y + delta_y;
	particle_t.pose.theta = particle_t_1.pose.theta + delta_theta_1 + delta_theta_2;
	particle_t.velocity = velocity_t;
	particle_t.geometry.length = particle_t_1.geometry.length;
	particle_t.geometry.width = particle_t_1.geometry.width;

	return particle_t;
}


double
get_ray_cost(carmen_vector_2D_t end_point, rectangle_points r_a, rectangle_points r_b, rectangle_points r_c)
{
	const double c_occ = 1.0;
	const double c_b = 2.0;
	const double c_s = 0.0;
	const double c_p = 1.6;

	int intersect_a, intersect_b, intersect_c;

	intersect_c = check_ray_intersection(end_point, r_c);
	if (intersect_c)
		return c_p;

	intersect_b = check_ray_intersection(end_point, r_b);
	if (intersect_b && !intersect_c)
		return c_s;

	intersect_a = check_ray_intersection(end_point, r_a);
	if (intersect_a && !intersect_b)
		return c_b;

	return c_occ;
}


double
get_probability(double cost, double sigma)
{
	double p_x, variance;
	variance = sigma*sigma;

	p_x = (1.0/(sigma*sqrt(2*M_PI))) * exp(-0.5 *(cost)*(cost)/(variance));
	//p_x = exp(-(cost)*(cost)/(variance));

	return p_x;
}


double
get_sigma_from_cost(double cost)
{
	(void) cost;
	return 0.01;
}


//TODO incluir a parte S_t-1que considera a geometria
double
get_particle_weight(moving_objects3_particle_t particle_t, double *virtual_scan, int num_of_rays)
{
	double probability = 0.0;
	double cost,sigma;
	rectangle_points r_a, r_b, r_c;

	carmen_vector_2D_t end_point;

	generate_rectangles_points(particle_t.pose, particle_t.geometry.width, particle_t.geometry.length,
			&r_c, &r_b, &r_a,
			0.25);

	if (euclidean_distance(0.0,0.0,particle_t.pose.x,particle_t.pose.y) < 7.0)
	{
		return 0.0;
	}

	double angular_resolution = 2*M_PI / num_of_rays;
	double angle;

	for (int i = 0; i < num_of_rays; i++)
	{
		angle = (i * angular_resolution) - M_PI;
		transform_polar_coordinates_to_cartesian_coordinates(virtual_scan[i], angle, &end_point.x, &end_point.y);
		cost = get_ray_cost(end_point, r_a, r_b, r_c);
		sigma = get_sigma_from_cost(cost);
		probability += get_probability(cost, sigma);
	}
	return probability/num_of_rays;
}


double
measurement_model(moving_objects3_particle_t particle_t, carmen_velodyne_projected_on_ground_message velodyne_projected_on_ground)
{
	(void) particle_t;
	(void) velodyne_projected_on_ground;
	return 1.0;
}


void
normalize_weights(std::vector<moving_objects3_particle_t> &particle_set, double total_weight)
{
	double inv_total_weight = 1.0/total_weight;

	std::vector<moving_objects3_particle_t>::iterator it = particle_set.begin();
	std::vector<moving_objects3_particle_t>::iterator end = particle_set.end();

	for (; it != end; ++it)
	{
		it->weight *= inv_total_weight;
	}
}


std::vector<moving_objects3_particle_t>
resample(std::vector<moving_objects3_particle_t> particle_set_t)
{
	std::vector<moving_objects3_particle_t> particle_set;
	double inv_num_particles = 1.0/NUM_OF_PARTICLES;

	double r = carmen_uniform_random(0.0,inv_num_particles);
	double c = particle_set_t[0].weight;
	double U;
	int i = 0;

	for (int m = 0; m < NUM_OF_PARTICLES; m++)
	{
		U = r + (m * inv_num_particles);
		while (U > c)
		{
			i++;
			c += particle_set_t[i].weight;
		}
		particle_set.push_back(particle_set_t[i]);
	}
	return particle_set;
}


std::vector<moving_objects3_particle_t>
algorithm_particle_filter(std::vector<moving_objects3_particle_t> particle_set_t_1,
		double *virtual_scan, int num_of_rays,
		double delta_time)
{
	std::vector<moving_objects3_particle_t> particle_set_t;

	double total_weight = 0.0;

	std::vector<moving_objects3_particle_t>::iterator it = particle_set_t_1.begin();
	std::vector<moving_objects3_particle_t>::iterator end = particle_set_t_1.end();
	for (; it != end; ++it)
	{
		moving_objects3_particle_t particle_t;
		// Motion Model
		particle_t = sample_motion_model((*it), delta_time);

		// Measurement Model -> RANSAC
		// cost = measurement_model(particle_t, velodyne_projected_on_ground);

		// Weighing particles
		particle_t.weight = get_particle_weight(particle_t, virtual_scan, num_of_rays);
		total_weight += particle_t.weight;

		particle_set_t.push_back(particle_t);
	}

	// normalize particles weight
	normalize_weights(particle_set_t, total_weight);

	// resample
	particle_set_t = resample(particle_set_t);

	return particle_set_t;
}


std::vector<moving_objects3_particle_t>
importance_sampling(double *virtual_scan, int num_of_rays, int num_of_particles)
{
	std::vector<moving_objects3_particle_t> particle_set_t;

	double total_weight = 0.0;

	for (int i = 0; i < num_of_particles; )
	{
		moving_objects3_particle_t particle_t;

		particle_t.geometry.length = 4.5;
		particle_t.geometry.width = 1.6;
		particle_t.pose.theta = carmen_uniform_random(-M_PI, M_PI);
		particle_t.pose.x = carmen_uniform_random(-50,50);
		particle_t.pose.y = carmen_uniform_random(-50,50);
		particle_t.velocity = carmen_uniform_random(-25,25);

		// Weighing particles
		particle_t.weight = get_particle_weight(particle_t, virtual_scan, num_of_rays);
		total_weight += particle_t.weight;

		if (particle_t.weight > 0.1)
		{
			particle_set_t.push_back(particle_t);
			i++;
		}
	}

	// normalize particles weight
	normalize_weights(particle_set_t, total_weight);

	// resample
	particle_set_t = resample(particle_set_t);

	return particle_set_t;
}


std::vector<moving_objects3_particle_t>
scaling_series_particle_filter(std::vector<moving_objects3_particle_t> particle_set_t_1,
		double *virtual_scan, int num_of_rays,
		double delta_time)
{
	std::vector<moving_objects3_particle_t> particle_set_t;

	double total_weight = 0.0;

	std::vector<moving_objects3_particle_t>::iterator it = particle_set_t_1.begin();
	std::vector<moving_objects3_particle_t>::iterator end = particle_set_t_1.end();
	for (; it != end; ++it)
	{
		moving_objects3_particle_t particle_t;
		// Motion Model
		particle_t = sample_motion_model((*it), delta_time);

		// Measurement Model -> RANSAC
		// cost = measurement_model(particle_t, velodyne_projected_on_ground);

		// Weighing particles
		particle_t.weight = get_particle_weight(particle_t, virtual_scan, num_of_rays);
		total_weight += particle_t.weight;

		particle_set_t.push_back(particle_t);
	}

	// normalize particles weight
	normalize_weights(particle_set_t, total_weight);

	// resample
	particle_set_t = resample(particle_set_t);

	return particle_set_t;
}
