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

	return particle_t;
}


// TODO terminar implementação
double
measurement_model(moving_objects3_particle_t particle_t, carmen_velodyne_projected_on_ground_message velodyne_projected_on_ground)
{

	return 1.0;
}

//todo checar se é assim mesmo
double
get_particle_weight(double cost)
{
	double variance = 1.0;
	double n = 1.0;

	return n*exp(-cost/(variance));
}

std::vector<moving_objects3_particle_t>
algorithm_particle_filter(std::vector<moving_objects3_particle_t> particle_set_t_1,
		carmen_velodyne_projected_on_ground_message velodyne_projected_on_ground,
		double delta_time)
{
	std::vector<moving_objects3_particle_t> particle_set_t;
	moving_objects3_particle_t particle_t;
	double total_weight = 0.0;
	double cost = 0.0;

	std::vector<moving_objects3_particle_t>::iterator it = particle_set_t_1.begin();
	std::vector<moving_objects3_particle_t>::iterator end = particle_set_t_1.end();
	for (; it != end; ++it)
	{
		// Motion Model
		particle_t = sample_motion_model((*it), delta_time);

		// Measurement Model
		cost = measurement_model(particle_t, velodyne_projected_on_ground);

		// Weighing particles
		particle_t.weight = get_particle_weight(cost);
		total_weight += particle_t.weight;

		particle_set_t.push_back(particle_t);
	}

	// Weighing particles
	//normalize_weights(particle_set_t, total_weight);

	//resample(particle_set_t);

	return particle_set_t;
}
