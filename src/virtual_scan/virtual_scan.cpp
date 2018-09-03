/*
 * virtual_scan.cpp
 *
 *  Created on: Jul 17, 2017
 *	  Author: claudine
 */
#include <stdio.h>
#include <carmen/carmen.h>
#include <algorithm>
#include <string.h>
#include <cmath>
#include <carmen/moving_objects_messages.h>
#include <carmen/velodyne_interface.h>
#include <carmen/dbscan.h>
#include <carmen/matrix.h>
#include <carmen/kalman.h>
#include "virtual_scan.h"

using namespace std;


extern carmen_localize_ackerman_map_t localize_map;
extern double x_origin;
extern double y_origin;
extern double map_resolution;

extern int g_zi;


int
compare_angles(const void *a, const void *b)
{
	carmen_point_t *arg1 = (carmen_point_t *) a;
	carmen_point_t *arg2 = (carmen_point_t *) b;

	double delta_theta = carmen_normalize_theta(arg1->theta - arg2->theta);
	if (delta_theta < 0.0)
		return (-1);
	if (delta_theta > 0.0)
		return (1);

	return (0);
}


carmen_mapper_virtual_scan_message *
copy_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan)
{
	carmen_mapper_virtual_scan_message *extended_virtual_scan = (carmen_mapper_virtual_scan_message *) malloc(sizeof(carmen_mapper_virtual_scan_message));
	*extended_virtual_scan = *virtual_scan;
	extended_virtual_scan->virtual_scan_sensor = (carmen_virtual_scan_sensor_t *) malloc(virtual_scan->num_sensors * sizeof(carmen_virtual_scan_sensor_t));
	for (int i = 0; i < virtual_scan->num_sensors; i++)
	{
		extended_virtual_scan->virtual_scan_sensor[i] = virtual_scan->virtual_scan_sensor[i];
		extended_virtual_scan->virtual_scan_sensor[i].points = (carmen_point_t *) malloc(virtual_scan->virtual_scan_sensor[i].num_points * sizeof(carmen_point_t));
		memcpy((void *) extended_virtual_scan->virtual_scan_sensor[i].points, (void *) virtual_scan->virtual_scan_sensor[i].points, virtual_scan->virtual_scan_sensor[i].num_points * sizeof(carmen_point_t));
	}

	return (extended_virtual_scan);
}


carmen_mapper_virtual_scan_message *
sort_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan) // Sort rays according to theta
{
	carmen_mapper_virtual_scan_message *extended_virtual_scan = copy_virtual_scan(virtual_scan);

//	carmen_pose_3D_t world_pose = {{virtual_scan->globalpos.x, virtual_scan->globalpos.y, 0.0}, {0.0, 0.0, virtual_scan->globalpos.theta}};
//	world_pose = get_world_pose_with_velodyne_offset(world_pose);
//	extended_virtual_scan->velodyne_pos = {world_pose.position.x, world_pose.position.y, world_pose.orientation.yaw};

	for (int s = 0; s < extended_virtual_scan->num_sensors; s++)
	{
		for (int i = 0; i < extended_virtual_scan->virtual_scan_sensor[s].num_points; i++)
		{
			double theta = atan2(extended_virtual_scan->virtual_scan_sensor[s].points[i].y - extended_virtual_scan->virtual_scan_sensor[s].sensor_pos.y,
					extended_virtual_scan->virtual_scan_sensor[s].points[i].x - extended_virtual_scan->virtual_scan_sensor[s].sensor_pos.x);
			theta = carmen_normalize_theta(theta - extended_virtual_scan->virtual_scan_sensor[s].sensor_pos.theta);
			extended_virtual_scan->virtual_scan_sensor[s].points[i].theta = theta;
		}
		qsort((void *) (extended_virtual_scan->virtual_scan_sensor[s].points), (size_t) extended_virtual_scan->virtual_scan_sensor[s].num_points, sizeof(carmen_point_t), compare_angles);
	}
	return (extended_virtual_scan);
}


void
sort_segment_points_by_angle(virtual_scan_segment_t *segment, carmen_point_t sensor_pos)
{
	for (int i = 0; i < segment->num_points; i++)
	{
		double theta = atan2(segment->points[i].y - sensor_pos.y, segment->points[i].x - sensor_pos.x);
		theta = carmen_normalize_theta(theta + sensor_pos.theta);
		segment->points[i].theta = theta;
	}
	qsort((void *) (segment->points), (size_t) segment->num_points, sizeof(carmen_point_t), compare_angles);
}


carmen_mapper_virtual_scan_message *
filter_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan_extended)
{
	carmen_mapper_virtual_scan_message *virtual_scan_extended_filtered;
	virtual_scan_extended_filtered = (carmen_mapper_virtual_scan_message *) calloc(1, sizeof(carmen_mapper_virtual_scan_message));

	*virtual_scan_extended_filtered = *virtual_scan_extended;
	virtual_scan_extended_filtered->virtual_scan_sensor = (carmen_virtual_scan_sensor_t *) calloc(virtual_scan_extended->num_sensors, sizeof(carmen_virtual_scan_sensor_t));
	for (int s = 0; s < virtual_scan_extended->num_sensors; s++)
	{
		virtual_scan_extended_filtered->virtual_scan_sensor[s] = virtual_scan_extended->virtual_scan_sensor[s];
		int num_points = 0;
		virtual_scan_extended_filtered->virtual_scan_sensor[s].num_points = num_points;
		virtual_scan_extended_filtered->virtual_scan_sensor[s].points = NULL;
		for (int i = 0; i < virtual_scan_extended->virtual_scan_sensor[s].num_points; i++)
		{
	//		if (DIST2D(virtual_scan_extended->velodyne_pos, virtual_scan_extended->points[i]) < 30.0)
			{
				int x_index_map = (int) round((virtual_scan_extended->virtual_scan_sensor[s].points[i].x - x_origin) / map_resolution);
				int y_index_map = (int) round((virtual_scan_extended->virtual_scan_sensor[s].points[i].y - y_origin) / map_resolution);
				if ((x_index_map > 0) && (x_index_map < localize_map.config.x_size) &&
					(y_index_map > 0) && (y_index_map < localize_map.config.y_size))
				{
					if (localize_map.prob[x_index_map][y_index_map] < PROB_THRESHOLD)
					{
						virtual_scan_extended_filtered->virtual_scan_sensor[s].points = (carmen_point_t *) realloc(virtual_scan_extended_filtered->virtual_scan_sensor[s].points,
											sizeof(carmen_point_t) * (num_points + 1));
						virtual_scan_extended_filtered->virtual_scan_sensor[s].points[num_points] = virtual_scan_extended->virtual_scan_sensor[s].points[i];
						num_points++;
					}
				}
				else // Inclui (nao filtra) pontos fora do mapa pois o mapa pode estar simplesmente atrasado.
				{
					virtual_scan_extended_filtered->virtual_scan_sensor[s].points = (carmen_point_t *) realloc(virtual_scan_extended_filtered->virtual_scan_sensor[s].points,
										sizeof(carmen_point_t) * (num_points + 1));
					virtual_scan_extended_filtered->virtual_scan_sensor[s].points[num_points] = virtual_scan_extended->virtual_scan_sensor[s].points[i];
					num_points++;
				}
			}
		}
		virtual_scan_extended_filtered->virtual_scan_sensor[s].num_points = num_points;
	}

	return (virtual_scan_extended_filtered);
}


dbscan::Cluster
generate_cluster_with_all_points(carmen_point_t *points, int size)
{
	dbscan::Cluster cluster;

	for (int i = 0; i < size; i++)
		cluster.push_back(points[i]);

	return (cluster);
}


carmen_point_t
compute_segment_centroid(virtual_scan_segment_t virtual_scan_segment)
{
	carmen_point_t centroid = {0.0, 0.0, 0.0};

	for (int i = 0; i < virtual_scan_segment.num_points; i++)
	{
		centroid.x += virtual_scan_segment.points[i].x;
		centroid.y += virtual_scan_segment.points[i].y;
	}
	centroid.x /= (double) virtual_scan_segment.num_points;
	centroid.y /= (double) virtual_scan_segment.num_points;

	return (centroid);
}


virtual_scan_segment_classes_t *
segment_virtual_scan(carmen_mapper_virtual_scan_message *extended_virtual_scan)
{
	virtual_scan_segment_classes_t *virtual_scan_segments = (virtual_scan_segment_classes_t *) malloc(sizeof(virtual_scan_segment_classes_t));
	virtual_scan_segments->segment = NULL;
	virtual_scan_segments->segment_features = NULL;
	virtual_scan_segments->num_segments = 0;

	for (int s = 0; s < extended_virtual_scan->num_sensors; s++)
	{
		dbscan::Cluster single_cluster = generate_cluster_with_all_points(extended_virtual_scan->virtual_scan_sensor[s].points, extended_virtual_scan->virtual_scan_sensor[s].num_points);
		dbscan::Clusters clusters = dbscan::dbscan(8.0 * PEDESTRIAN_RADIUS * PEDESTRIAN_RADIUS, MINIMUN_CLUSTER_SIZE, single_cluster);

		if (clusters.size() > 0)
		{
			virtual_scan_segments->num_segments += clusters.size();
			virtual_scan_segments->segment = (virtual_scan_segment_t *) realloc(virtual_scan_segments->segment,
					sizeof(virtual_scan_segment_t) * virtual_scan_segments->num_segments);
			virtual_scan_segments->segment_features = (virtual_scan_segment_features_t *) realloc(virtual_scan_segments->segment_features,
					sizeof(virtual_scan_segment_features_t) * virtual_scan_segments->num_segments);

			for (unsigned int segment_id = 0; segment_id < clusters.size(); segment_id++)
			{
				int current_segment = virtual_scan_segments->num_segments - clusters.size() + segment_id;
				virtual_scan_segments->segment[current_segment].zi = g_zi;
				virtual_scan_segments->segment[current_segment].sensor = s;
				virtual_scan_segments->segment[current_segment].sensor_id = extended_virtual_scan->virtual_scan_sensor[s].sensor_id;
				virtual_scan_segments->segment[current_segment].sensor_pos = extended_virtual_scan->virtual_scan_sensor[s].sensor_pos;
				virtual_scan_segments->segment[current_segment].global_pos = extended_virtual_scan->virtual_scan_sensor[s].global_pos;
				virtual_scan_segments->segment[current_segment].sensor_v = extended_virtual_scan->virtual_scan_sensor[s].v;
				virtual_scan_segments->segment[current_segment].sensor_w = extended_virtual_scan->virtual_scan_sensor[s].w;
				virtual_scan_segments->segment[current_segment].global_pos = extended_virtual_scan->virtual_scan_sensor[s].global_pos;

				dbscan::Cluster cluster = clusters[segment_id];
				virtual_scan_segments->segment[current_segment].points = (carmen_point_t *) malloc(sizeof(carmen_point_t) * cluster.size());
				virtual_scan_segments->segment[current_segment].num_points = cluster.size();
				for (unsigned int i = 0; i < cluster.size(); i++)
					virtual_scan_segments->segment[current_segment].points[i] = cluster[i];

				virtual_scan_segments->segment[current_segment].centroid = compute_segment_centroid(virtual_scan_segments->segment[current_segment]);
				double centroid_angle = atan2(virtual_scan_segments->segment[current_segment].centroid.y - virtual_scan_segments->segment[current_segment].sensor_pos.y,
						virtual_scan_segments->segment[current_segment].centroid.x - virtual_scan_segments->segment[current_segment].sensor_pos.x);

				double angular_distance_to_timestamp = carmen_normalize_theta(centroid_angle - virtual_scan_segments->segment[current_segment].sensor_pos.theta) -
						extended_virtual_scan->virtual_scan_sensor[s].last_sensor_angle;
				double delta_t = (angular_distance_to_timestamp / (2 * M_PI)) * extended_virtual_scan->virtual_scan_sensor[s].time_spent_in_the_entire_sensor_sweep;
				virtual_scan_segments->segment[current_segment].precise_timestamp = extended_virtual_scan->virtual_scan_sensor[s].timestamp - delta_t;

//				printf("sensor_id %d, sensor_tess %lf, sensor_lsa %lf, sensor_dist %lf, segment_id %d, centroid_angle %lf, sensor_angle %lf, delta_t %lf\n",
//						virtual_scan_segments->segment[current_segment].sensor_id,
//						extended_virtual_scan->virtual_scan_sensor[s].time_spent_in_the_entire_sensor_sweep,
//						carmen_radians_to_degrees(extended_virtual_scan->virtual_scan_sensor[s].last_sensor_angle),
//						DIST2D(virtual_scan_segments->segment[current_segment].sensor_pos, virtual_scan_segments->segment[current_segment].centroid),
//						segment_id,
//						carmen_radians_to_degrees(centroid_angle), carmen_radians_to_degrees(virtual_scan_segments->segment[current_segment].sensor_pos.theta),
//						delta_t);

				sort_segment_points_by_angle(&(virtual_scan_segments->segment[current_segment]), extended_virtual_scan->virtual_scan_sensor[s].sensor_pos);
			}
		}
	}

	return (virtual_scan_segments);
}


static double
dist2(carmen_point_t v, carmen_point_t w)
{
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}


static double
dist2(carmen_position_t v, carmen_position_t w)
{
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}


carmen_point_t
distance_from_point_to_line_segment_vw(int *point_in_trajectory_is, carmen_point_t v, carmen_point_t w, carmen_point_t p)
{
	// Return minimum distance between line segment vw and points p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-points-and-a-line-segment
	double l2, t;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.
	if (sqrt(l2) < PEDESTRIAN_RADIUS)	  // v ~== w case // @@@ Alberto: Checar isso
	{
		*point_in_trajectory_is = SEGMENT_TOO_SHORT;
		return (v);
	}

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	if (t < 0.0) 	// p beyond the v end of the segment
	{
		*point_in_trajectory_is = POINT_BEFORE_SEGMENT;
		return (v);
	}
	if (t > 1.0)	// p beyond the w end of the segment
	{
		*point_in_trajectory_is = POINT_AFTER_SEGMENT;
		return (w);
	}

	// Projection falls on the segment
	p = v; // Copy other elements, like theta, etc.
	p.x = v.x + t * (w.x - v.x);
	p.y = v.y + t * (w.y - v.y);
	*point_in_trajectory_is = POINT_WITHIN_SEGMENT;

	return (p);
}


carmen_point_t
projection_of_point_into_line_segment_vw(carmen_point_t v, carmen_point_t w, carmen_point_t p)
{
	// See return minimum distance between line segment vw and points p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-points-and-a-line-segment
	double l2, t;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	p = v; // Copy other elements, like theta, etc.
	p.x = v.x + t * (w.x - v.x);
	p.y = v.y + t * (w.y - v.y);

	return (p);
}


double
distance_from_point_to_line_segment_vw(carmen_position_t v, carmen_position_t w, carmen_point_t p)
{
	// Return minimum distance between line segment vw and points p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-points-and-a-line-segment
	double l2, t;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

//	if (t < 0.0) 	// p beyond the v end of the segment
//	{
//	}
//	if (t > 1.0)	// p beyond the w end of the segment
//	{
//	}

	// Projection falls on the segment
	double x = v.x + t * (w.x - v.x);
	double y = v.y + t * (w.y - v.y);

	double dx = p.x - x;
	double dy = p.y - y;

	return (sqrt(dx * dx + dy * dy));
}


double
distance_from_point_to_line_segment_vw(carmen_point_t v, carmen_point_t w, carmen_point_t p)
{
	// Return minimum distance between line segment vw and points p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-points-and-a-line-segment
	double l2, t;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	// Projection falls on the segment
	double x = v.x + t * (w.x - v.x);
	double y = v.y + t * (w.y - v.y);

	double dx = p.x - x;
	double dy = p.y - y;

	return (sqrt(dx * dx + dy * dy));
}


void
set_segment_features(virtual_scan_segment_classes_t *virtual_scan_segment_classes, int segment,
		carmen_point_t first_point, carmen_point_t last_point, carmen_point_t farthest_point,
		carmen_point_t main_line_first_point, carmen_point_t main_line_last_point, double width, double length, int segment_class)
{
	virtual_scan_segment_classes->segment_features[segment].first_point = first_point;
	virtual_scan_segment_classes->segment_features[segment].last_point = last_point;
	virtual_scan_segment_classes->segment_features[segment].farthest_point = farthest_point;
	virtual_scan_segment_classes->segment_features[segment].main_line_first_point = main_line_first_point;
	virtual_scan_segment_classes->segment_features[segment].main_line_last_point = main_line_last_point;
	virtual_scan_segment_classes->segment_features[segment].width = width;
	virtual_scan_segment_classes->segment_features[segment].length = length;
	virtual_scan_segment_classes->segment_features[segment].segment_class = segment_class;
}


bool
segment_is_mass_point(virtual_scan_segment_t segment, carmen_point_t centroid)
{
	double max_distance_to_centroid = 0.0;
	for (int i = 0; i < segment.num_points; i++)
	{
		double distance = DIST2D(centroid, segment.points[i]);
		if (distance > max_distance_to_centroid)
			max_distance_to_centroid = distance;
	}

	if (max_distance_to_centroid < PEDESTRIAN_RADIUS)
		return (true);
	else
		return (false);
}


bool
segment_is_i_shaped(carmen_point_t first_point, carmen_point_t last_point, carmen_point_t farthest_point,
		double &width, double &length)
{
	double dist_farthest_first = DIST2D(farthest_point, first_point);
	double dist_farthest_last = DIST2D(farthest_point, last_point);
	if (dist_farthest_first > dist_farthest_last)
	{
		width = dist_farthest_last;
		length = dist_farthest_first;
	}
	else
	{
		width = dist_farthest_first;
		length = dist_farthest_last;
	}

	if ((width / length) < L_SMALL_SEGMENT_AS_A_PROPORTION_OF_THE_LARGE)
		return (true);
	else
		return (false);
}


void
remove_segment(virtual_scan_segment_classes_t *virtual_scan_segments, int victim)
{
	free(virtual_scan_segments->segment[victim].points);
	for (int j = victim; j < virtual_scan_segments->num_segments - 1; j++)
	{
		virtual_scan_segments->segment[j] = virtual_scan_segments->segment[j + 1];
		virtual_scan_segments->segment_features[j] = virtual_scan_segments->segment_features[j + 1];
	}
	virtual_scan_segments->num_segments--;
	virtual_scan_segments->segment = (virtual_scan_segment_t *) realloc(virtual_scan_segments->segment,
			sizeof(virtual_scan_segment_t) * virtual_scan_segments->num_segments);
	virtual_scan_segments->segment_features = (virtual_scan_segment_features_t *) realloc(virtual_scan_segments->segment_features,
			sizeof(virtual_scan_segment_features_t) * virtual_scan_segments->num_segments);
}


double
sum_of_square_distances_to_line_from_first_to_last_point(int first_point, int last_point, virtual_scan_segment_t segment)
{
	double sum = 0.0;

	for (int i = first_point; i <= last_point; i++)
		sum += pow(distance_from_point_to_line_segment_vw(segment.points[first_point], segment.points[last_point], segment.points[i]), 2.0);

	return (sum);
}


double
get_sqr_ratio(int first_point, int last_point, virtual_scan_segment_t segment)
{
	double first_to_last_distance = DIST2D(segment.points[first_point], segment.points[last_point]);
	double sqr_first_to_last_distance = first_to_last_distance * first_to_last_distance;
	double sqr_sum = sum_of_square_distances_to_line_from_first_to_last_point(first_point, last_point, segment);

	return (sqr_sum / sqr_first_to_last_distance);
}


void
find_principal_line_defining_points(carmen_point_t &first_point, carmen_point_t &last_point, virtual_scan_segment_t segment)
{
	int i = 0;
	int j = segment.num_points - 1;

	double initial_distance = DIST2D(segment.points[i], segment.points[j]);
	double best_sqr_ratio = get_sqr_ratio(i, j, segment);
	int best_i = i;
	int best_j = j;
	j--;
	while (DIST2D(segment.points[i], segment.points[j]) > 0.7 * initial_distance)
	{
		while (DIST2D(segment.points[i], segment.points[j]) > 0.7 * initial_distance)
		{
			double new_sqr_ratio = get_sqr_ratio(i, j, segment);
			if (new_sqr_ratio < best_sqr_ratio)
			{
				best_i = i;
				best_j = j;
				best_sqr_ratio = new_sqr_ratio;
			}
			j--;
		}
		i++;
		j = segment.num_points - 1;
	}

	first_point = projection_of_point_into_line_segment_vw(segment.points[best_i], segment.points[best_j], segment.points[0]);
	first_point.theta = segment.points[best_i].theta;
	last_point = projection_of_point_into_line_segment_vw(segment.points[best_i], segment.points[best_j], segment.points[segment.num_points - 1]);
	last_point.theta = segment.points[best_j].theta;
}


void
classify_segments(virtual_scan_segment_classes_t *virtual_scan_segments)
{
	virtual_scan_segment_classes_t *virtual_scan_segment_classes = virtual_scan_segments;

	double width = PEDESTRIAN_RADIUS, length = PEDESTRIAN_RADIUS;
	for (int i = 0; i < virtual_scan_segment_classes->num_segments; i++)
	{
		virtual_scan_segment_t segment = virtual_scan_segment_classes->segment[i];

//		double distance_centroid_to_sensor_pos = DIST2D(segment.centroid, segment.sensor_pos);
//		if (((segment.sensor_id == VELODYNE) && (distance_centroid_to_sensor_pos > MAX_VELODYNE_SEGMENT_DISTANCE)) ||
//			((segment.sensor_id == LASER_LDMRS) && (distance_centroid_to_sensor_pos <= MAX_VELODYNE_SEGMENT_DISTANCE)))
//		{
//			remove_segment(virtual_scan_segment_classes, i);
//			i--;
//			continue;
//		}

		carmen_point_t first_point = segment.points[segment.num_points / 150]; // O termo (segment.num_points / 150) remove os primeiros e ultimos ~1% do segmento para remover ruido
		carmen_point_t last_point = segment.points[segment.num_points - (1 + segment.num_points / 150)];

		carmen_point_t main_line_first_point = first_point;
		carmen_point_t main_line_last_point = last_point;
		carmen_point_t farthest_point = first_point;
		int segment_class;
		if (segment_is_mass_point(segment, segment.centroid))
			segment_class = MASS_POINT;
		else if (segment.num_points > MIN_SEGMENT_SIZE)
		{
			find_principal_line_defining_points(main_line_first_point, main_line_last_point, segment);
			if (DIST2D(main_line_first_point, first_point) > DIST2D(main_line_last_point, last_point))
				farthest_point = main_line_first_point;
			else
				farthest_point = main_line_last_point;
			if (segment_is_i_shaped(first_point, last_point, farthest_point, width, length))
				segment_class = I_SHAPED;
			else
				segment_class = L_SHAPED;
		}
		else
		{
			remove_segment(virtual_scan_segment_classes, i);
			i--;
			continue;
		}

		set_segment_features(virtual_scan_segment_classes, i, first_point, last_point, farthest_point,
				main_line_first_point, main_line_last_point, width, length, segment_class);
	}
}


int
is_last_box_model_hypotheses_empty(virtual_scan_box_model_hypotheses_t *box_model_hypotheses)
{
	int i = box_model_hypotheses->last_box_model_hypotheses;
	return (box_model_hypotheses->box_model_hypotheses[i].num_boxes == 0);
}


virtual_scan_box_model_t *
virtual_scan_append_box(virtual_scan_box_models_t *models, virtual_scan_segment_t segment)
{
	int n = models->num_boxes + 1;
	models->num_boxes = n;

	if (n == 1)
	{
		models->box = (virtual_scan_box_model_t *) malloc(sizeof(virtual_scan_box_model_t));
		models->box_points = (virtual_scan_segment_t *) malloc(sizeof(virtual_scan_segment_t));
	}
	else
	{
		models->box = (virtual_scan_box_model_t *) realloc(models->box, sizeof(virtual_scan_box_model_t) * n);
		models->box_points = (virtual_scan_segment_t *) realloc(models->box_points, sizeof(virtual_scan_segment_t) * n);
	}

	models->box_points[n - 1] = segment;

	virtual_scan_box_model_t *box = (models->box + n - 1);
	memset(box, '\0', sizeof(virtual_scan_box_model_t));

	return (box);
}


virtual_scan_box_model_hypotheses_t *
virtual_scan_new_box_model_hypotheses(int length)
{
	virtual_scan_box_model_hypotheses_t *hypotheses = (virtual_scan_box_model_hypotheses_t *) malloc(sizeof(virtual_scan_box_model_hypotheses_t));

	hypotheses->num_box_model_hypotheses = length;
	hypotheses->box_model_hypotheses = (virtual_scan_box_models_t *) calloc(length, sizeof(virtual_scan_box_models_t));
	hypotheses->last_box_model_hypotheses = 0;

	return (hypotheses);
}


virtual_scan_box_models_t *
virtual_scan_get_empty_box_models(virtual_scan_box_model_hypotheses_t *hypotheses)
{
	return (hypotheses->box_model_hypotheses + hypotheses->last_box_model_hypotheses);
}


virtual_scan_box_models_t *
virtual_scan_get_box_models(virtual_scan_box_model_hypotheses_t *hypotheses, int i)
{
	return (hypotheses->box_model_hypotheses + i);
}


void
append_pedestrian_to_box_models(virtual_scan_box_models_t *box_models, virtual_scan_segment_t segment, carmen_point_t centroid)
{
	virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, segment);
	box->c = PEDESTRIAN;
	box->x = centroid.x;
	box->y = centroid.y;
	box->width = 2.0 * PEDESTRIAN_RADIUS;
	box->length = 2.0 * PEDESTRIAN_RADIUS;
	box->theta = 0.0;
}


void
append_l_object(virtual_scan_box_models_t *box_models, virtual_scan_segment_t segment, carmen_point_t farthest_point,
		int category, virtual_scan_category_t categories[], double theta, double theta2)
{
	carmen_position_t p1 = {farthest_point.x + categories[category].length * cos(theta),
							farthest_point.y + categories[category].length * sin(theta)};
	carmen_position_t p2 = {farthest_point.x + categories[category].width * cos(theta2),
							farthest_point.y + categories[category].width * sin(theta2)};
	carmen_position_t length_center_position;
	length_center_position.x = (p1.x + p2.x) / 2.0;
	length_center_position.y = (p1.y + p2.y) / 2.0;

	virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, segment);
	box->c = categories[category].category;
	box->width = categories[category].width;
	box->length = categories[category].length;
	box->x = length_center_position.x;
	box->y = length_center_position.y;
	box->theta = theta;
}


void
append_l_object(virtual_scan_box_models_t *box_models, virtual_scan_segment_t segment, carmen_point_t farthest_point,
		int category, virtual_scan_category_t categories[], double theta, double theta2, double l, double w)
{
	carmen_position_t p1 = {farthest_point.x + l * cos(theta),
							farthest_point.y + l * sin(theta)};
	carmen_position_t p2 = {farthest_point.x + w * cos(theta2),
							farthest_point.y + w * sin(theta2)};
	carmen_position_t length_center_position;
	length_center_position.x = (p1.x + p2.x) / 2.0;
	length_center_position.y = (p1.y + p2.y) / 2.0;

	virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, segment);
	box->c = categories[category].category;
	box->width = w;
	box->length = l;
	box->x = length_center_position.x;
	box->y = length_center_position.y;
	box->theta = theta;
}


void
append_i_object(virtual_scan_box_models_t *box_models, virtual_scan_segment_t segment, carmen_point_t first_point,
		int category, virtual_scan_category_t categories[], double theta, double theta2)
{
	carmen_position_t p1 = {first_point.x + categories[category].length * cos(theta),
							first_point.y + categories[category].length * sin(theta)};
	carmen_position_t p2 = {first_point.x + categories[category].width * cos(theta2),
							first_point.y + categories[category].width * sin(theta2)};
	carmen_position_t length_center_position;
	length_center_position.x = (p1.x + p2.x) / 2.0;
	length_center_position.y = (p1.y + p2.y) / 2.0;

	virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, segment);
	box->c = categories[category].category;
	box->width = categories[category].width;
	box->length = categories[category].length;
	box->x = length_center_position.x;
	box->y = length_center_position.y;
	box->theta = theta;
}


void
append_i_object(virtual_scan_box_models_t *box_models, virtual_scan_segment_t segment, carmen_point_t first_point,
		int category, virtual_scan_category_t categories[], double theta, double theta2, double l, double w)
{
	carmen_position_t p1 = {first_point.x + l * cos(theta),
							first_point.y + l * sin(theta)};
	carmen_position_t p2 = {first_point.x + w * cos(theta2),
							first_point.y + w * sin(theta2)};
	carmen_position_t length_center_position;
	length_center_position.x = (p1.x + p2.x) / 2.0;
	length_center_position.y = (p1.y + p2.y) / 2.0;

	virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, segment);
	box->c = categories[category].category;
	box->width = w;
	box->length = l;
	box->x = length_center_position.x;
	box->y = length_center_position.y;
	box->theta = theta;
}


void
append_l_shaped_objects_to_box_models(virtual_scan_box_models_t *box_models, virtual_scan_segment_t segment,
		virtual_scan_segment_features_t segment_features, virtual_scan_category_t categories[])
{
	carmen_point_t first_point = segment_features.first_point;
	carmen_point_t last_point = segment_features.last_point;
	carmen_point_t farthest_point = segment_features.farthest_point;
//	for (int category = 0; category < 3; category++)
	for (int category = 1; category < 2; category++)
	{
		double theta, theta2, l, w;

		l = DIST2D(farthest_point, first_point);
		w = DIST2D(farthest_point, last_point);
		if ((categories[category].length > (0.7 * l)) &&
			(categories[category].width > (0.6 * w)) && (categories[category].width < (1.7 * w)))
		{
			theta = atan2(first_point.y - farthest_point.y, first_point.x - farthest_point.x);
			theta2 = atan2(last_point.y - farthest_point.y, last_point.x - farthest_point.x);
			if (true) //segment.num_points < 20)
				append_l_object(box_models, segment, farthest_point, category, categories, theta, theta2);
			else
				append_l_object(box_models, segment, farthest_point, category, categories, theta, theta2, l, w);
		}
		else
		{
			l = DIST2D(farthest_point, last_point);
			w = DIST2D(farthest_point, first_point);
			if (true) //(categories[category].length > l) &&
//				(categories[category].width > (0.6 * w)) && (categories[category].width < (1.7 * w)))
			{
				theta = atan2(last_point.y - farthest_point.y, last_point.x - farthest_point.x);
				theta2 = atan2(first_point.y - farthest_point.y, first_point.x - farthest_point.x);
				if (true) //segment.num_points < 20)
					append_l_object(box_models, segment, farthest_point, category, categories, theta, theta2);
				else
					append_l_object(box_models, segment, farthest_point, category, categories, theta, theta2, l, w);
			}
		}
	}
}


void
append_i_shaped_objects_to_box_models(virtual_scan_box_models_t *box_models, virtual_scan_segment_t segment,
		virtual_scan_segment_features_t segment_features, virtual_scan_category_t categories[])
{
	carmen_point_t first_point = segment_features.first_point;
	carmen_point_t last_point = segment_features.last_point;
	carmen_point_t farthest_point = segment_features.farthest_point;

	if (DIST2D(farthest_point, first_point) > DIST2D(farthest_point, last_point))
		last_point = farthest_point;
	else
		first_point = farthest_point;

//	for (int category = 0; category < 3; category++)
	for (int category = 1; category < 2; category++)
	{
		double theta, theta2;

		double l = DIST2D(first_point, last_point);
		if ((categories[category].length > l * 0.8))// &&
//			(categories[category].width < l * 0.6))
		{
			theta = atan2(last_point.y - first_point.y, last_point.x - first_point.x);
			theta2 = carmen_normalize_theta(theta - M_PI / 2.0);
			if (true) //segment.num_points < 20)
				append_i_object(box_models, segment, first_point, category, categories, theta, theta2);
			else
				append_i_object(box_models, segment, first_point, category, categories, theta, theta2, l, l * (categories[category].width / categories[category].length));

			theta = atan2(first_point.y - last_point.y, first_point.x - last_point.x);
			theta2 = carmen_normalize_theta(theta + M_PI / 2.0);
			if (true) //segment.num_points < 20)
				append_i_object(box_models, segment, last_point, category, categories, theta, theta2);
			else
				append_i_object(box_models, segment, last_point, category, categories, theta, theta2, l, l * (categories[category].width / categories[category].length));
		}

		if (l <= categories[category].width * 1.5)
		{
			theta = carmen_normalize_theta(atan2(last_point.y - first_point.y, last_point.x - first_point.x) - M_PI / 2.0);
			theta2 = carmen_normalize_theta(theta + M_PI / 2.0);
			if (true) //segment.num_points < 20)
				append_i_object(box_models, segment, first_point, category, categories, theta, theta2);
			else
				append_i_object(box_models, segment, first_point, category, categories, theta, theta2, l, l * (categories[category].width / categories[category].length));

			theta2 = carmen_normalize_theta(theta - M_PI / 2.0);
			if (true) //segment.num_points < 20)
				append_i_object(box_models, segment, last_point, category, categories, theta, theta2);
			else
				append_i_object(box_models, segment, last_point, category, categories, theta, theta2, l, l * (categories[category].width / categories[category].length));
		}
	}
}


int
virtual_scan_num_box_models(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	int total = 0;

	virtual_scan_box_models_t *hypotheses = virtual_scan_box_model_hypotheses->box_model_hypotheses;
	for (int i = 0, m = virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i < m; i++)
		total += hypotheses[i].num_boxes;

	return total;
}


void
virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
	{
		free(virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box);
		free(virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box_points);
	}
	free(virtual_scan_box_model_hypotheses->box_model_hypotheses);
	free(virtual_scan_box_model_hypotheses);
}


void
virtual_scan_free_segment_classes(virtual_scan_segment_classes_t *virtual_scan_segment_classes)
{
	if (virtual_scan_segment_classes != NULL)
	{
		for (int i = 0; i < virtual_scan_segment_classes->num_segments; i++)
			free(virtual_scan_segment_classes->segment[i].points);

		free(virtual_scan_segment_classes->segment);
		free(virtual_scan_segment_classes->segment_features);
		free(virtual_scan_segment_classes);
	}
}


void
virtual_scan_free_scan_extended(carmen_mapper_virtual_scan_message *virtual_scan_extended)
{
	if (virtual_scan_extended != NULL)
	{
		for (int i = 0; i < virtual_scan_extended->num_sensors; i++)
			free(virtual_scan_extended->virtual_scan_sensor[i].points);

		free(virtual_scan_extended->virtual_scan_sensor);
		free(virtual_scan_extended);
	}
}


virtual_scan_segment_classes_t *
virtual_scan_extract_segments(carmen_mapper_virtual_scan_message *virtual_scan_extended)
{
	virtual_scan_segment_classes_t *virtual_scan_segments = segment_virtual_scan(virtual_scan_extended);
	classify_segments(virtual_scan_segments);

	return (virtual_scan_segments);
}


void
plot_segment(virtual_scan_segment_t segment, virtual_scan_segment_features_t segment_features, int min_segment_size)
{
	if (segment.num_points > min_segment_size)
	{
		static bool first_time = true;
		static FILE *gnuplot_pipe;

		if (first_time)
		{
			gnuplot_pipe = popen("gnuplot", "w"); //("gnuplot -persist", "w") to keep last plot after program closes
			fprintf(gnuplot_pipe, "set size ratio -1\n");
			fprintf(gnuplot_pipe, "set xrange [-4:4]\n set yrange [-4:4]\n");

			first_time = false;
		}

		FILE *arq = fopen("caco_.txt", "w");
		for (int i = 0; i < segment.num_points; i++)
			fprintf(arq, "%lf %lf\n",
					segment.points[i].x - segment.points[0].x,
					segment.points[i].y - segment.points[0].y);
		fclose(arq);

		arq = fopen("caco__.txt", "w");
		fprintf(arq, "%lf %lf %.2lf\n",
				segment_features.main_line_first_point.x - segment.points[0].x,
				segment_features.main_line_first_point.y - segment.points[0].y,
				carmen_radians_to_degrees(carmen_normalize_theta(atan2(segment_features.main_line_first_point.y - segment.sensor_pos.y, segment_features.main_line_first_point.x - segment.sensor_pos.x) - segment.global_pos.theta)));
//				180.0 * ((segment_features.main_line_first_point.theta - segment.sensor_pos.theta) / M_PI));
		fprintf(arq, "%lf %lf %.2lf\n",
				segment_features.main_line_last_point.x - segment.points[0].x,
				segment_features.main_line_last_point.y - segment.points[0].y,
				carmen_radians_to_degrees(carmen_normalize_theta(atan2(segment_features.main_line_last_point.y - segment.sensor_pos.y, segment_features.main_line_last_point.x - segment.sensor_pos.x) - segment.global_pos.theta)));
//				180.0 * ((segment_features.main_line_last_point.theta - segment.sensor_pos.theta) / M_PI));
		fclose(arq);

		fprintf(gnuplot_pipe, "plot 'caco_.txt' u 1:2 w lp\n");
		fprintf(gnuplot_pipe, "replot 'caco__.txt' u 1:2 w p, 'caco__.txt' u 1:2:3 w labels offset 3\n");

		fflush(gnuplot_pipe);
	}
}


virtual_scan_box_model_hypotheses_t *
virtual_scan_fit_box_models(virtual_scan_segment_classes_t *virtual_scan_segment_classes, double frame_timestamp)
{
	virtual_scan_category_t categories[] = {{BUS, 2.5, 15.0}, {CAR, 1.7, 4.1}, {BIKE, 0.5, 2.1}}; // Trung-Dung Vu Thesis

	int num_segments = virtual_scan_segment_classes->num_segments;
	virtual_scan_box_model_hypotheses_t *box_model_hypotheses = virtual_scan_new_box_model_hypotheses(num_segments);
	box_model_hypotheses->frame_timestamp = frame_timestamp;
	for (int i = 0; i < num_segments; i++)
	{
		virtual_scan_segment_features_t segment_features = virtual_scan_segment_classes->segment_features[i];
		int segment_class = segment_features.segment_class;
		virtual_scan_segment_t segment = virtual_scan_segment_classes->segment[i];

		virtual_scan_box_models_t *box_models = virtual_scan_get_empty_box_models(box_model_hypotheses);
//		if (segment_class == MASS_POINT)
//			append_pedestrian_to_box_models(box_models, segment, virtual_scan_segment_classes->segment_features[i].centroid);
		if (segment_class == L_SHAPED) // L-shape segments segments will generate bus and car hypotheses
		{
			plot_segment(segment, segment_features, 30);
			append_l_shaped_objects_to_box_models(box_models, segment, virtual_scan_segment_classes->segment_features[i], categories);
		}
		if (segment_class == I_SHAPED) // I-shape segments segments will generate bus, car and bike hypotheses
		{
			plot_segment(segment, segment_features, 30);
			append_i_shaped_objects_to_box_models(box_models, segment, virtual_scan_segment_classes->segment_features[i], categories);
		}

		if (!is_last_box_model_hypotheses_empty(box_model_hypotheses))
			box_model_hypotheses->last_box_model_hypotheses++;
	}

	return (box_model_hypotheses);
}
