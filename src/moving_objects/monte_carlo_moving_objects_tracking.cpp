#include "monte_carlo_moving_objects_tracking.h"


////////////////////////////////////////////////////////////////////////////////////////////


particle_datmo
sample_motion_model(double delta_time, particle_datmo particle_t_1)
{
	particle_datmo particle_t;
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
dist_btw_point_and_box(carmen_position_t pt, double box_width, double box_length)
{
	/* Normalizing is required to work with different sizes of boxes */
	double W = norm_factor_y/2;
	double L = norm_factor_x/2;
	double x = (pt.x/box_length)*norm_factor_x;
	double y = (pt.y/box_width)*norm_factor_y;
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


double
dist_bounding_box(carmen_point_t particle_pose, pcl::PointCloud<pcl::PointXYZ> &point_cloud, object_geometry_t obj_model,
		object_geometry_t obj_geometry, carmen_vector_3D_t car_global_position, double x_pose, double y_pose)
{
	double sum = 0.0;
	long unsigned int pcl_size = point_cloud.size();
	carmen_position_t new_pt; //(x,y)

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it)
	{
		new_pt = transf2d_bounding_box(car_global_position.x + it->x - particle_pose.x, car_global_position.y + it->y - particle_pose.y, -particle_pose.theta);
		sum += dist_btw_point_and_box(new_pt, obj_model.width, obj_model.length);
	}

	double penalty = 0.0;
	// Centroid's coordinates already global
	new_pt = transf2d_bounding_box(x_pose - particle_pose.x, y_pose - particle_pose.y, -particle_pose.theta);
	if (new_pt.x > obj_model.length/2 || new_pt.y > obj_model.width/2)
	{
		new_pt.x *= norm_factor_x/obj_model.length;//4.5;
		new_pt.y *= norm_factor_y/obj_model.width;//2.1;
		penalty = sqrt(new_pt.x*new_pt.x + new_pt.y*new_pt.y);//new_pt.x + new_pt.y;//
	}

	// Penalize based on the differences between dimensions of point cloud and model box
	double diff_length  = fabs(obj_model.length - obj_geometry.length)*(norm_factor_x/obj_geometry.length);
	double diff_width   = fabs(obj_model.width - obj_geometry.width)*(norm_factor_y/obj_geometry.width);
	double diff_height  = fabs(obj_model.height - obj_geometry.height)*(norm_factor_z/obj_geometry.height);
	penalty += diff_length + diff_width + diff_height;

	// Avoid division by zero
	if (pcl_size != 0)
		return sum/pcl_size + penalty; //return normalized distance with penalty
	return 999999.0;
}


double
measurement_model(particle_datmo &particle_t, double x, double y, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud,
		object_geometry_t obj_geometry, carmen_vector_3D_t car_global_position)
{
	double dist;

	dist = dist_bounding_box(particle_t.pose, pcl_cloud, particle_t.model_features.geometry,
			obj_geometry, car_global_position, x, y);

	particle_t.dist = dist;

	return dist;
}


void
resample(std::vector<particle_datmo> &particle_set_t)
{
	/*** LOW VARIANCE RESAMPLER ALGORITHM ***/
	static double *cumulative_sum = NULL;
	static int num_particles = particle_set_t.size();

	particle_datmo *copy_particle_set_t = NULL;
	int i, which_particle;
	double weight_sum = 0.0;
	double position, step_size;

	/* Allocate memory necessary for resampling */
	cumulative_sum = (double *)calloc(num_particles, sizeof(double));
	carmen_test_alloc(cumulative_sum);
	copy_particle_set_t = (particle_datmo*)malloc(sizeof(particle_datmo) * num_particles);
	carmen_test_alloc(copy_particle_set_t);

	int j = 0;
	for (std::vector<particle_datmo>::iterator it = particle_set_t.begin(); it != particle_set_t.end(); it++)
	{
		copy_particle_set_t[j].pose.x     = it->pose.x;
		copy_particle_set_t[j].pose.y     = it->pose.y;
		copy_particle_set_t[j].pose.theta = it->pose.theta;
		copy_particle_set_t[j].velocity   = it->velocity;
		copy_particle_set_t[j].weight     = it->weight;
		copy_particle_set_t[j].class_id   = it->class_id;
		copy_particle_set_t[j].model_features = it->model_features;
		copy_particle_set_t[j].dist       = it->dist;

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

		particle_set_t[i].pose.x     = copy_particle_set_t[which_particle].pose.x;
		particle_set_t[i].pose.y     = copy_particle_set_t[which_particle].pose.y;
		particle_set_t[i].pose.theta = copy_particle_set_t[which_particle].pose.theta;
		particle_set_t[i].velocity   = copy_particle_set_t[which_particle].velocity;
		particle_set_t[i].weight     = copy_particle_set_t[which_particle].weight;
		particle_set_t[i].class_id   = copy_particle_set_t[which_particle].class_id;
		particle_set_t[i].model_features = copy_particle_set_t[which_particle].model_features;
		particle_set_t[i].dist       = copy_particle_set_t[which_particle].dist;
	}

	free(cumulative_sum);
	free(copy_particle_set_t);
}


void
normalize_weights(std::vector<particle_datmo> &particle_set, double total_weight)
{
	for (std::vector<particle_datmo>::iterator it = particle_set.begin(); it != particle_set.end(); ++it)
		it->weight = (it->weight/total_weight);
}


std::vector<particle_datmo>
algorithm_monte_carlo(std::vector<particle_datmo> &particle_set_t_1, double x, double y, double delta_time,
		pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, object_geometry_t obj_geometry, carmen_vector_3D_t car_global_position)
{
	std::vector<particle_datmo> particle_set_t;
	particle_datmo particle_t;
	double total_weight = 0.0, dist;

	// Prediction
	for (std::vector<particle_datmo>::iterator it = particle_set_t_1.begin(); it != particle_set_t_1.end(); ++it)
	{
		// Motion Model
		particle_t = sample_motion_model(delta_time, (*it));

		// Measurement Model
		dist = measurement_model(particle_t, x, y, pcl_cloud, obj_geometry, car_global_position);

		// Weighing particles
		particle_t.weight = particle_weight_pose_reading_model(dist);
		total_weight += particle_t.weight;

		particle_set_t.push_back(particle_t);
	}

	// Normalize weights
	normalize_weights(particle_set_t, total_weight);

	// Resample
	resample(particle_set_t);

	return particle_set_t;
}
