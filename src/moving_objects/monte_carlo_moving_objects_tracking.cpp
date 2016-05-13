#include "monte_carlo_moving_objects_tracking.h"
#include <pcl/search/kdtree.h>


////////////////////////////////////////////////////////////////////////////////////////////


particle_datmo_t
sample_motion_model(double delta_time, particle_datmo_t particle_t_1)
{
	particle_datmo_t particle_t;
	carmen_point_t pose_t, pose_t_1;
	double v;

	pose_t_1.x     = particle_t_1.pose.x;
	pose_t_1.y     = particle_t_1.pose.y;
	pose_t_1.theta = particle_t_1.pose.theta;
	v              = particle_t_1.velocity;

	v = v + carmen_gaussian_random(0.0, alpha_1);
	if (v > v_max) v = v_max;
	if (v < v_min) v = v_min;

	pose_t_1.theta = pose_t_1.theta + carmen_gaussian_random(0.0, alpha_2);
	pose_t_1.theta = carmen_normalize_theta(pose_t_1.theta);

	pose_t.x = pose_t_1.x + delta_time*v*cos(pose_t_1.theta);
	pose_t.y = pose_t_1.y + delta_time*v*sin(pose_t_1.theta);

	particle_t.pose.theta = carmen_normalize_theta(pose_t_1.theta);
	particle_t.pose.x     = pose_t.x;
	particle_t.pose.y     = pose_t.y;
	particle_t.velocity   = v;
	particle_t.weight     = particle_t_1.weight;

	// Keep same class with 90% probability
	if ((rand() % 100) <= 90)
		particle_t.class_id = particle_t_1.class_id;
	else
		particle_t.class_id = get_random_model_id(num_of_models);

	particle_t.model_features = get_obj_model_features(particle_t.class_id);
//	particle_t.model_features_3d = get_obj_model_features_3d(particle_t.class_id, object_models_3d);

	return particle_t;
}


double
dist_nearest_neighbor(double x_z_t, double y_z_t, carmen_point_t pose_t)
{
	double distance;
	double d_x, d_y;

	d_x = x_z_t - pose_t.x;
	d_y = y_z_t - pose_t.y;

	distance = sqrt((d_x*d_x) + (d_y*d_y));

	return distance;
}


double
particle_weight_pose_reading_model(double dist)
{
	return exp(-dist);
}


double
particle_weight_by_normal_distribution(double dist, double sigma, double mean)
{
	double p_x, variance, const_part;

	variance = sigma*sigma;
	const_part = 1/sqrt(2*M_PI*(variance));
	p_x = const_part*exp(-0.5*(dist-mean)*(dist-mean)/(variance));

	return p_x;
}


double
particle_weight_by_normal_distribution(double dist, double mean, double inv_variance, double const_part)
{
	double p_x;

	p_x = const_part*exp(-0.5*(dist-mean)*(dist-mean)*(inv_variance));

	return p_x;
}


/* 2D Transformations required for calculating bounding box distance */
carmen_position_t
transf2d_bounding_box(double x_trans, double y_trans, double theta)
{
	// Rotate
	double x = x_trans*cos(theta) - y_trans*sin(theta);
	double y = x_trans*sin(theta) + y_trans*cos(theta);

	/* Since we're only interested in calculating the distances from points
	 * to boxes' closest side, we can "place" all points on the 1st
	 * quadrant of the new coordinate system. So: */
	x = fabs(x);
	y = fabs(y);

	carmen_position_t pt = {x,y};
	return pt;
}


double
dist_btw_point_and_box(carmen_position_t pt, double width_normalizer, double length_normalizer)
{
	/* Normalizing is required to work with different sizes of boxes */
	double W = 0.5*norm_factor_y;
	double L = 0.5*norm_factor_x;
	double x = pt.x*length_normalizer;
	double y = pt.y*width_normalizer;
	double dist;

	if (y > W)
		if (x > L) // region 1 - worst position
			dist = x + y;
		else // region 2 - intermediate position
			dist = y + (L - x);
	else
		if (x <= L) // region 3 - best position
			dist = (L - x) + (W - y);
		else // region 4 - intermediate position
			dist = x + (W - y);

	return dist;
}


// Essas 2 funções calculam o multiplicador do comprimento e da largura de acordo com um angulo. Esse multiplicador varia de 0 a 100 e o da largura é complementar ao do comprimento. Eles dependem da diferença de orientação do IARA pra um conjunto de partículas e do ângulo formado por esses 2 objetos. E ele serve pra sabermos pra qual parte do objeto estamos "olhando".

/*double
calculate_length_multiplier(double angle)
{
	while (angle < 0)
	{
		angle += M_PI;
	}
	while (angle >= M_PI)
	{
		angle -= M_PI;
	}

	if (angle >= 0 && angle <= M_PI/2)
	{
		return (100.0/(M_PI/2))*angle - 100;
	}
	else if (angle > M_PI/2 && angle < M_PI)
	{
		return (100.0/(M_PI/2))*angle;
	}
}*/

double
calculate_width_multiplier(double angle)
{
	while (angle < 0)
	{
		angle += M_PI;
	}
	while (angle >= M_PI)
	{
		angle -= M_PI;
	}

	if (angle >= 0 && angle <= M_PI/2)
	{
		return (100.0/(M_PI/2))*angle;
	}
	else if (angle > M_PI/2 && angle < M_PI)
	{
		return (100.0/(M_PI/2))*angle - 100;
	}
}


double
dist_bounding_box(carmen_point_t particle_pose, pcl::PointCloud<pcl::PointXYZ> &point_cloud, object_geometry_t model_geometry,
		object_geometry_t obj_geometry, carmen_vector_3D_t car_global_position, double x_pose, double y_pose)
{
	double sum = 0.0;
	long unsigned int pcl_size = point_cloud.size();
	carmen_position_t new_pt; //(x,y)
	double width_normalizer = norm_factor_y/model_geometry.width;
	double length_normalizer = norm_factor_x/model_geometry.length;
	double height_normalizer = norm_factor_x/model_geometry.height;

	/*** BOX POSITIONING ***/
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it)
	{
		new_pt = transf2d_bounding_box(car_global_position.x + it->x - particle_pose.x, car_global_position.y + it->y - particle_pose.y, -particle_pose.theta);
		sum += dist_btw_point_and_box(new_pt, width_normalizer, length_normalizer);
	}

	double penalty = 0.0;
	// Centroid's coordinates already global
	new_pt = transf2d_bounding_box(x_pose - particle_pose.x, y_pose - particle_pose.y, -particle_pose.theta);
	if (new_pt.x > 0.5*model_geometry.length || new_pt.y > 0.5*model_geometry.width)
	{
		new_pt.x *= length_normalizer;//4.5;
		new_pt.y *= width_normalizer;//2.1;
		penalty = sqrt(new_pt.x*new_pt.x + new_pt.y*new_pt.y);//new_pt.x + new_pt.y;//
	}

	/*** BOX DIMENSIONING ***/
	// Penalize based on the differences between dimensions of point cloud and model box
	double diff_length = (model_geometry.length - obj_geometry.length)*length_normalizer;
	double diff_width = (model_geometry.width - obj_geometry.width)*width_normalizer;
	double diff_height = (model_geometry.height - obj_geometry.height)*height_normalizer;
	double object_diagonal_xy = sqrt(obj_geometry.length*length_normalizer + obj_geometry.width*width_normalizer);
	double model_diagonal_xy = sqrt(model_geometry.length*length_normalizer + model_geometry.width*width_normalizer);
	double diff_diagonal = (model_diagonal_xy - object_diagonal_xy);
//	penalty += 2*diff_diagonal + diff_height;
	penalty += diff_length*diff_length + diff_width*diff_width + diff_height*diff_height + diff_diagonal*diff_diagonal;
	// Avoid division by zero
	if (pcl_size != 0)
		return sum/pcl_size + 0.2*penalty; //return normalized distance with penalty

	/* Else return very big distance */
	return 999999.0;
}


double
dist_nn_3d_model(pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud, pcl::PointCloud<pcl::PointXYZ> obj_cloud,
		carmen_point_t particle_pose, carmen_vector_3D_t car_global_position)
{
	double theta = -particle_pose.theta;
	pcl::PointXYZ point;
	double dist = 0.0;
	int k = 1; //k-nearest neighbor

	// kd-tree object
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(model_cloud);
	// This vector will store the output neighbors.
	std::vector<int> pointIndices(k);
	// This vector will store their squared distances to the search point.
	std::vector<float> squaredDistances(k);
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = obj_cloud.begin(); it != obj_cloud.end(); ++it)
	{
		// Necessary transformations (translation and rotation):
		float x = car_global_position.x + it->x - particle_pose.x;
		float y = car_global_position.y + it->y - particle_pose.y;
		point.z = it->z;
		point.x = x*cos(theta) - y*sin(theta);
		point.y = x*sin(theta) + y*cos(theta);

		if (kdtree.nearestKSearch(point, k, pointIndices, squaredDistances) > 0)
		{
//			dist += sqrt(double(squaredDistances[0]));
			dist += double(squaredDistances[0]);
		}
	}

	return dist;
}


double
measurement_model(particle_datmo_t &particle_t, double x, double y, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud,
		object_geometry_t obj_geometry, carmen_vector_3D_t car_global_position)
{
	double dist;

	/*** NEAREST NEIGHBOUR - VERSÃO DO EDUARDO: ***/
	//dist = dist_nearest_neighbor(x, y, particle_t.pose);

	/*** 2D BOUNDING BOX ***/
	dist = dist_bounding_box(particle_t.pose, pcl_cloud, particle_t.model_features.geometry,
			obj_geometry, car_global_position, x, y);

	/*** 3D MODEL POINT CLOUD ***/
//	pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	model_cloud = get_model_point_cloud(particle_t.class_id, object_models_3d);
//
//	dist = dist_nn_3d_model(model_cloud, pcl_cloud, particle_t.pose, car_global_position);

	return dist;
}


void
resample(std::vector<particle_datmo_t> &particle_set_t)
{
	/*** LOW VARIANCE RESAMPLER ALGORITHM ***/

//	EDUARDO'S VERSION:
//	static double *cumulative_sum = NULL;
//	static particle_datmo_t *temp_particles = NULL;
//	static int num_particles = particle_set_t.size();
//
//	particle_datmo_t *aux;
//	particle_datmo_t *copy_particle_set_t = NULL;
//	int i, which_particle;
//	double weight_sum;
//	double position, step_size;
//
//	/* Allocate memory necessary for resampling */
//	cumulative_sum = (double *)calloc(num_particles, sizeof(double));
//	carmen_test_alloc(cumulative_sum);
//	temp_particles = (particle_datmo_t*)malloc(sizeof(particle_datmo_t) * num_particles);
//	carmen_test_alloc(temp_particles);
//	copy_particle_set_t = (particle_datmo_t*)malloc(sizeof(particle_datmo_t) * num_particles);
//	carmen_test_alloc(copy_particle_set_t);
//
//	int j = 0;
//	for (std::vector<particle_datmo_t>::iterator it = particle_set_t.begin(); it != particle_set_t.end(); it++)
//	{
//		copy_particle_set_t[j].pose.x = it->pose.x;
//		copy_particle_set_t[j].pose.y = it->pose.y;
//		copy_particle_set_t[j].pose.theta = it->pose.theta;
//		copy_particle_set_t[j].velocity = it->velocity;
//		copy_particle_set_t[j].weight = it->weight;
//		copy_particle_set_t[j].class_id = it->class_id;
//		copy_particle_set_t[j].model_features = it->model_features;
//		copy_particle_set_t[j].dist = it->dist;
//		j++;
//	}
//
//	weight_sum = 0.0;
//	for (i = 0; i < num_particles; i++)
//	{
//		/* Sum the weights of all of the particles */
//		weight_sum += copy_particle_set_t[i].weight;
//		cumulative_sum[i] = weight_sum;
//	}
//
//	/* choose random starting position for low-variance walk */
//	position = carmen_uniform_random(0, weight_sum);
//	step_size = weight_sum / (double) num_particles;
//	which_particle = 0;
//
//	/* draw num_particles random samples */
//	for (i = 0; i < num_particles; i++)
//	{
//		position += step_size;
//
//		if (position > weight_sum)
//		{
//			position -= weight_sum;
//			which_particle = 0;
//		}
//
//		while(position > cumulative_sum[which_particle])
//			which_particle++;
//
//		temp_particles[i] = copy_particle_set_t[which_particle];
//	}
//
//	/* Switch particle pointers */
//	aux = copy_particle_set_t;
//	copy_particle_set_t = temp_particles;
//	temp_particles = aux;
//
//	int m = 0;
//	for (std::vector<particle_datmo_t>::iterator it = particle_set_t.begin(); it != particle_set_t.end(); it++)
//	{
//		it->pose.x = copy_particle_set_t[m].pose.x;
//		it->pose.y = copy_particle_set_t[m].pose.y;
//		it->pose.theta = copy_particle_set_t[m].pose.theta;
//		it->velocity = copy_particle_set_t[m].velocity;
//		it->weight = copy_particle_set_t[m].weight;
//		it->class_id = copy_particle_set_t[which_particle].class_id;
//		it->model_features = copy_particle_set_t[which_particle].model_features;
//		it->dist = copy_particle_set_t[which_particle].dist;
//		m++;
//	}
//
//	free(temp_particles);
//	free(cumulative_sum);
//	free(copy_particle_set_t);

	static double *cumulative_sum = NULL;
	static int num_particles = particle_set_t.size();

	particle_datmo_t *copy_particle_set_t = NULL;
	int i, which_particle;
	double weight_sum = 0.0;
	double position, step_size;

	/* Allocate memory necessary for resampling */
	cumulative_sum = (double *)calloc(num_particles, sizeof(double));
	carmen_test_alloc(cumulative_sum);
	copy_particle_set_t = (particle_datmo_t*)malloc(sizeof(particle_datmo_t) * num_particles);
	carmen_test_alloc(copy_particle_set_t);

	int j = 0;
	for (std::vector<particle_datmo_t>::iterator it = particle_set_t.begin(); it != particle_set_t.end(); it++)
	{
//		copy_particle_set_t[j].pose.x     = it->pose.x;
//		copy_particle_set_t[j].pose.y     = it->pose.y;
//		copy_particle_set_t[j].pose.theta = it->pose.theta;
//		copy_particle_set_t[j].velocity   = it->velocity;
//		copy_particle_set_t[j].weight     = it->weight;
//		copy_particle_set_t[j].class_id   = it->class_id;
//		copy_particle_set_t[j].model_features = it->model_features;
//		copy_particle_set_t[j].dist       = it->dist;
		copy_particle_set_t[j] = *it;

		/* change log weights back into probabilities */
		/* Sum the weights of all of the particles */
		weight_sum += it->weight;
		cumulative_sum[j] = weight_sum;

		j++;
	}

	/* choose random starting position for low-variance walk */
	position = carmen_uniform_random(0, weight_sum);
	step_size = weight_sum / (double) num_particles;
	which_particle = 0;

	/* draw num_particles random samples */
	for (i = 0; i < num_particles; i++)
	{
		position += step_size;

		if (position > weight_sum)
		{
			position -= weight_sum;
			which_particle = 0;
		}

		while (position > cumulative_sum[which_particle])
			which_particle++;

		particle_set_t[i] = copy_particle_set_t[which_particle];
//		particle_set_t[i].pose.x = copy_particle_set_t[which_particle].pose.x;
//		particle_set_t[i].pose.y = copy_particle_set_t[which_particle].pose.y;
//		particle_set_t[i].pose.theta = copy_particle_set_t[which_particle].pose.theta;
//		particle_set_t[i].velocity = copy_particle_set_t[which_particle].velocity;
//		particle_set_t[i].weight = copy_particle_set_t[which_particle].weight;
//		particle_set_t[i].class_id = copy_particle_set_t[which_particle].class_id;
//		particle_set_t[i].model_features = copy_particle_set_t[which_particle].model_features;
//		particle_set_t[i].dist = copy_particle_set_t[which_particle].dist;
	}

	free(cumulative_sum);
	free(copy_particle_set_t);
}


void
normalize_weights(std::vector<particle_datmo_t> &particle_set, double total_weight)
{
	/* normalize weights */
// FIXME - Se mudar a normalização não funciona

//	double max_weight = -DBL_MAX;
//	for (std::vector<particle_datmo_t>::iterator it = particle_set.begin(); it != particle_set.end(); ++it)
//	{
//		max_weight = it->weight > max_weight ? it->weight : max_weight;
//	}
//	for (std::vector<particle_datmo_t>::iterator it = particle_set.begin(); it != particle_set.end(); ++it)
//	{
//		it->weight = exp(it->weight - max_weight);
//		//printf("weight=%.15f  dist=%.6f  norm_dist=%.15f\n", it->weight, it->dist, it->norm_dist);
//	}

	double inv_total_weight = 1/total_weight;
	for (std::vector<particle_datmo_t>::iterator it = particle_set.begin(); it != particle_set.end(); ++it)
	{
		it->weight *= inv_total_weight;
//		printf("weight=%.15f  dist=%.6f  norm_dist=%.15f\n", it->weight, it->dist, it->norm_dist);
	}
}


void
compute_weights(std::vector<particle_datmo_t> &particle_set, double total_dist)
{
	double norm_dist; // normalized distance, values in [0.0, 10.0]
	double total_weight;
	double inv_total_dist = 1.0/total_dist; //multiplication >=20 times faster than division

	/* pre calculated parts of normal distribution PDF for better performance */
	double std_dev = 0.1;
//	double inv_variance = 1.0/std_dev*std_dev;
//	double const_part = inv_variance*(1.0/sqrt(2*M_PI));

	/* normalize distances */
	for (std::vector<particle_datmo_t>::iterator it = particle_set.begin(); it != particle_set.end(); ++it)
	{
		norm_dist = 1000*inv_total_dist*it->dist;
		it->norm_dist = norm_dist;
//		it->weight = particle_weight_by_normal_distribution(norm_dist, 0.0, inv_variance, const_part);
//		it->weight = exp(-it->dist*it->dist);
//		it->weight = exp(-norm_dist*norm_dist*inv_variance);
		it->weight = exp(-it->dist);
		total_weight += it->weight;
	}

	/* normalize weights */
	normalize_weights(particle_set, total_weight);
}


std::vector<particle_datmo_t>
algorithm_monte_carlo(std::vector<particle_datmo_t> particle_set_t_1, double x, double y, double delta_time,
		pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, object_geometry_t obj_geometry, carmen_vector_3D_t car_global_position)
{
	std::vector<particle_datmo_t> particle_set_t;
	particle_datmo_t particle_t;
	double total_weight = 0.0;
//	double total_dist = 0.0;

	/* Prediction */
	for (std::vector<particle_datmo_t>::iterator it = particle_set_t_1.begin(); it != particle_set_t_1.end(); ++it)
	{
		// Motion Model
		particle_t = sample_motion_model(delta_time, (*it));

		// Measurement Model
		particle_t.dist = measurement_model(particle_t, x, y, pcl_cloud, obj_geometry, car_global_position);

		// Weighing particles (still not normalized)
//		total_dist += particle_t.dist;
// FIXME - Se usar a gaussiana não funciona
//		particle_t.weight = -particle_t.dist * particle_t.dist * 100.0;
		particle_t.weight = particle_weight_pose_reading_model(particle_t.dist);
		total_weight += particle_t.weight;
//		particle_t.weight = particle_weight_by_normal_distribution(particle_t.dist, 1.0, 0.0);
//		particle_t.weight = exp(-0.5 * (particle_t.dist) * (particle_t.dist));

		particle_set_t.push_back(particle_t);
	}

	/* Weighing particles */
	normalize_weights(particle_set_t, total_weight);
//	compute_weights(particle_set_t, total_dist);

	/* Resample */
	resample(particle_set_t);

	return particle_set_t;
}
