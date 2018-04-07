/*
 * virtual_scan.cpp
 *
 *  Created on: Jul 17, 2017
 *	  Author: claudine
 */
#include <carmen/carmen.h>
#include <algorithm>
#include <string.h>
#include <cmath>
#include <carmen/moving_objects_messages.h>
#include "virtual_scan.h"


extern carmen_localize_ackerman_map_t localize_map;
extern double x_origin;
extern double y_origin;
extern double map_resolution;

extern int g_zi;
extern virtual_scan_extended_t *g_virtual_scan_extended[NUMBER_OF_FRAMES_T];

virtual_scan_track_set_t *best_track_set = NULL;


int
compare_angles(const void *a, const void *b)
{
	carmen_point_t *arg1 = (carmen_point_t *) a;
	carmen_point_t *arg2 = (carmen_point_t *) b;

	// The contents of the array are being sorted in ascending order
	if (arg1->theta < arg2->theta)
		return -1;
	if (arg1->theta > arg2->theta)
		return 1;
	return 0;
}


virtual_scan_extended_t *
sort_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan) // Verify if rays are unordered according to theta
{
	virtual_scan_extended_t *extended_virtual_scan = (virtual_scan_extended_t *) malloc(sizeof(virtual_scan_extended_t));
	extended_virtual_scan->points = (carmen_point_t *) malloc(virtual_scan->num_points * sizeof(carmen_point_t));
	extended_virtual_scan->num_points = virtual_scan->num_points;
	extended_virtual_scan->globalpos = virtual_scan->globalpos;
	extended_virtual_scan->timestamp = virtual_scan->timestamp;
	for (int i = 0; i < virtual_scan->num_points; i++)
	{
		extended_virtual_scan->points[i].x = virtual_scan->points[i].x;
		extended_virtual_scan->points[i].y = virtual_scan->points[i].y;
		double theta = atan2(virtual_scan->points[i].y - virtual_scan->globalpos.y, virtual_scan->points[i].x - virtual_scan->globalpos.x);
		theta = carmen_normalize_theta(theta - virtual_scan->globalpos.theta);
		extended_virtual_scan->points[i].theta = theta;
	}
	qsort((void *) (extended_virtual_scan->points), (size_t) extended_virtual_scan->num_points, sizeof(carmen_point_t), compare_angles);

	return (extended_virtual_scan);
}


virtual_scan_extended_t *
filter_virtual_scan(virtual_scan_extended_t *virtual_scan_extended)
{
	virtual_scan_extended_t *virtual_scan_extended_filtered;
	virtual_scan_extended_filtered = (virtual_scan_extended_t *) calloc(1, sizeof(virtual_scan_extended_t));

	int num_points = 0;
	for (int i = 0; i < virtual_scan_extended->num_points; i++)
	{
		int x_index_map = (int) round((virtual_scan_extended->points[i].x - x_origin) / map_resolution);
		int y_index_map = (int) round((virtual_scan_extended->points[i].y - y_origin) / map_resolution);
		if ((x_index_map > 0) && (x_index_map < localize_map.config.x_size) &&
			(y_index_map > 0) && (y_index_map < localize_map.config.y_size))
		{
			if (localize_map.prob[x_index_map][y_index_map] < PROB_THRESHOLD)
			{
				virtual_scan_extended_filtered->points = (carmen_point_t *) realloc(virtual_scan_extended_filtered->points,
									sizeof(carmen_point_t) * (num_points + 1));
				virtual_scan_extended_filtered->points[num_points]=virtual_scan_extended->points[i];
				num_points++;
			}
		}
		else // Inclui (nao filtra) pontos fora do mapa pois o mapa pode estar simplesmente atrasado.
		{
			virtual_scan_extended_filtered->points = (carmen_point_t *) realloc(virtual_scan_extended_filtered->points,
								sizeof(carmen_point_t) * (num_points + 1));
			virtual_scan_extended_filtered->points[num_points]=virtual_scan_extended->points[i];
			num_points++;
		}
	}
	virtual_scan_extended_filtered->num_points = num_points;
	virtual_scan_extended_filtered->timestamp = virtual_scan_extended->timestamp;

	fprintf(stdout,"virtual_scan_extended->num_points = %d\n", virtual_scan_extended->num_points);
	fprintf(stdout,"virtual_scan_extended_filtered->num_points = %d\n\n", virtual_scan_extended_filtered->num_points);
	fflush(stdout);

	return virtual_scan_extended_filtered;
}


virtual_scan_segments_t *
segment_virtual_scan(virtual_scan_extended_t *extended_virtual_scan)
{
	int segment_id = 0;
	int begin_segment = 0;
	virtual_scan_segments_t *virtual_scan_segments;

	virtual_scan_segments = (virtual_scan_segments_t *) malloc(sizeof(virtual_scan_segments_t));
	virtual_scan_segments->num_segments = 0;
	virtual_scan_segments->timestamp = extended_virtual_scan->timestamp;
	virtual_scan_segments->segment = NULL;
	for (int i = 1; i < extended_virtual_scan->num_points; i++)
	{
		if ((DIST2D(extended_virtual_scan->points[i],  extended_virtual_scan->points[i - 1]) > 1.5) ||
			(i == extended_virtual_scan->num_points - 1))
		{
			virtual_scan_segments->segment = (virtual_scan_segment_t *) realloc(virtual_scan_segments->segment,
					sizeof(virtual_scan_segment_t) * (segment_id + 1));
			if (i < extended_virtual_scan->num_points - 1)
				virtual_scan_segments->segment[segment_id].num_points = i - begin_segment;
			else
				virtual_scan_segments->segment[segment_id].num_points = i - begin_segment + 1; // TODO: checar isso
			virtual_scan_segments->segment[segment_id].points = &(extended_virtual_scan->points[begin_segment]);
			begin_segment = i;
			segment_id++;
			virtual_scan_segments->num_segments++;
		}
	}

	return (virtual_scan_segments);
}


static double
dist2(carmen_point_t v, carmen_point_t w)
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
	if (l2 < PEDESTRAIN_SIZE)	  // v ~== w case // @@@ Alberto: Checar isso
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


carmen_point_t
compute_segment_centroid(virtual_scan_segment_t virtual_scan_segment)
{
	carmen_point_t centroid;

	for (int i = 0; i < virtual_scan_segment.num_points; i++)
	{
		centroid.x += virtual_scan_segment.points[i].x;
		centroid.y += virtual_scan_segment.points[i].y;
	}
	centroid.x /= virtual_scan_segment.num_points;
	centroid.y /= virtual_scan_segment.num_points;

	return(centroid);
}


virtual_scan_segment_classes_t *
classify_segments(virtual_scan_segments_t *virtual_scan_segments)
{
	virtual_scan_segment_classes_t *virtual_scan_segment_classes;
	virtual_scan_segment_classes = (virtual_scan_segment_classes_t *) malloc(sizeof(virtual_scan_segment_classes_t));
	int num_segments = virtual_scan_segments->num_segments;
	virtual_scan_segment_classes->num_segments = num_segments;
	virtual_scan_segment_classes->segment = (virtual_scan_segment_t *) malloc(sizeof(virtual_scan_segment_t)*num_segments);
	virtual_scan_segment_classes->segment_features = (virtual_scan_segment_features_t *) malloc(sizeof(virtual_scan_segment_features_t)*num_segments);

	for (int i = 0; i < virtual_scan_segments->num_segments; i++)
	{
		int num_points = virtual_scan_segments->segment[i].num_points;
		carmen_point_t first_point = virtual_scan_segments->segment[i].points[0];
		carmen_point_t last_point = virtual_scan_segments->segment[i].points[num_points - 1];
		double maximum_distance_to_line_segment = 0.0;
		carmen_point_t farthest_point = first_point;
		double width = 0.0;
		double length = 0.0;
		int segment_class = MASS_POINT;
//		double average_distance = 0.0;
		int j;
		for (j = 0; j < num_points; j++)
		{
			carmen_point_t point = virtual_scan_segments->segment[i].points[j];
			int point_type;
			// Finds the projection of the points to the line that connects v to w
			carmen_point_t point_within_line_segment = distance_from_point_to_line_segment_vw(&point_type, first_point, last_point, point);
			if (point_type == SEGMENT_TOO_SHORT)
			{
				segment_class = MASS_POINT;
				break;
			}
			else if (point_type == POINT_WITHIN_SEGMENT)
			{
				double distance = DIST2D(point, point_within_line_segment);
				if (distance > maximum_distance_to_line_segment)
				{
					maximum_distance_to_line_segment = distance;
					farthest_point = point;
				}
			}
		}

		if (j >= (num_points - 1))
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
			double alpha = atan2(length, width);
			double height = width * sin(alpha);
			if (maximum_distance_to_line_segment < height * L_SMALL_SEGMENT_AS_A_PROPORTION_OF_THE_LARGE)
				segment_class = I_SHAPED;
			else
				segment_class = L_SHAPED;
		}

		carmen_point_t centroid = compute_segment_centroid(virtual_scan_segments->segment[i]);

		virtual_scan_segment_classes->segment[i].num_points = num_points;
		virtual_scan_segment_classes->segment[i].points = virtual_scan_segments->segment[i].points;
		virtual_scan_segment_classes->segment_features[i].first_point = first_point;
		virtual_scan_segment_classes->segment_features[i].last_point = last_point;
		virtual_scan_segment_classes->segment_features[i].maximum_distance_to_line_segment = maximum_distance_to_line_segment;
		virtual_scan_segment_classes->segment_features[i].farthest_point = farthest_point;
		virtual_scan_segment_classes->segment_features[i].width = width;
		virtual_scan_segment_classes->segment_features[i].length = length;
		virtual_scan_segment_classes->segment_features[i].segment_class = segment_class;
		virtual_scan_segment_classes->segment_features[i].centroid = centroid;
		//		virtual_scan_segment_classes->segment_features[i].average_distance_to_line_segment = average_distance;

	}
	virtual_scan_segment_classes->timestamp = virtual_scan_segments->timestamp;

	return (virtual_scan_segment_classes);
}


int
is_last_box_model_hypotheses_empty(virtual_scan_box_model_hypotheses_t *box_model_hypotheses)
{
	int i = box_model_hypotheses->last_box_model_hypotheses;
	return (box_model_hypotheses->box_model_hypotheses[i].num_boxes == 0);
}


virtual_scan_box_model_t *
virtual_scan_append_box(virtual_scan_box_models_t *models, virtual_scan_segment_t box_points)
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

	models->box_points[n - 1] = box_points;

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
	return hypotheses;
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
append_pedestrian_to_box_models(virtual_scan_box_models_t *box_models, virtual_scan_segment_t box_points, carmen_point_t centroid)
{
	virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, box_points);
	box->c = PEDESTRIAN;
	box->x = centroid.x;
	box->y = centroid.y;
}


void
append_l_shaped_objects_to_box_models(virtual_scan_box_models_t *box_models, virtual_scan_segment_t box_points,
		carmen_point_t first_point, carmen_point_t last_point, carmen_point_t farthest_point, virtual_scan_category_t categories[])
{
	for (int j = 0; j < 2; j++) // Indexes the model categories
	{
		carmen_point_t length_point = first_point;
		carmen_point_t width_point = last_point;
		for (int o = 0; o < 2; o++) // Indexes the matching orientation of model categories
		{
			std::swap(length_point, width_point);
			double l = DIST2D(farthest_point, length_point);
			double w = DIST2D(farthest_point, width_point);
			if (categories[j].length < l || categories[j].width < w)
				continue;

			double rho = atan2(length_point.y - farthest_point.y, length_point.x - farthest_point.x);
			double phi = atan2(width_point.y - farthest_point.y, width_point.x - farthest_point.x);
			carmen_position_t length_center_position =
				{ farthest_point.x + 0.5 * categories[j].length * cos(rho), farthest_point.y + 0.5 * categories[j].length * sin(rho) };
			for (double direction = 0; direction <= M_PI; direction += M_PI)
			{
				virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, box_points);
				box->c = categories[j].category;
				box->width = categories[j].width;
				box->length = categories[j].length;
				box->x = length_center_position.x + 0.5 * categories[j].width * cos(phi);
				box->y = length_center_position.y + 0.5 * categories[j].width * sin(phi);
				box->theta = carmen_normalize_theta(rho + direction);
			}
		}
	}
}


void
append_i_shaped_objects_to_box_models(virtual_scan_box_models_t *box_models, virtual_scan_segment_t box_points,
		carmen_point_t first_point, carmen_point_t last_point, virtual_scan_category_t categories[])
{
	for (int j = 0; j < 3; j++)
	{
		double l = DIST2D(first_point, last_point);
		if (l <= categories[j].length)
		{
			double theta = atan2(last_point.y - first_point.y, last_point.x - first_point.x);
			carmen_position_t center_positions[] =
				{/* center from first points */
				{ first_point.x + 0.5 * categories[j].length * cos(theta), first_point.y + 0.5 * categories[j].length * sin(theta) }, /* center from last points */
				{ last_point.x - 0.5 * categories[j].length * cos(theta), last_point.y - 0.5 * categories[j].length * sin(theta) }, /* center of segment */
				{ 0.5 * (first_point.x + last_point.x), 0.5 * (first_point.y + last_point.y) } };

			// Compute two boxes for the center from first_point and two more from last_point
			for (int p = 0; p < 3; p++)
			{
				carmen_position_t length_center_position = center_positions[p];
				for (double delta_phi = -M_PI_2; delta_phi < M_PI; delta_phi += M_PI)
				{
					double phi = carmen_normalize_theta(theta + delta_phi);
					for (double direction = 0; direction <= M_PI; direction += M_PI)
					{
						virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, box_points);
						box->c = categories[j].category;
						box->width = categories[j].width;
						box->length = categories[j].length;
						box->x = length_center_position.x + 0.5 * categories[j].width * cos(phi);
						box->y = length_center_position.y + 0.5 * categories[j].width * sin(phi);
						box->theta = carmen_normalize_theta(theta + direction);
					}
				}
			}
		}
		if (l <= categories[j].width)
		{
			double theta = atan2(last_point.y - first_point.y, last_point.x - first_point.x);
			carmen_position_t center_positions[] =
				{/* center from first points */
				{ first_point.x + 0.5 * categories[j].width * cos(theta), first_point.y + 0.5 * categories[j].width * sin(theta) }, /* center from last points */
				{ last_point.x - 0.5 * categories[j].width * cos(theta), last_point.y - 0.5 * categories[j].width * sin(theta) }, /* center of segment */
				{ 0.5 * (first_point.x + last_point.x), 0.5 * (first_point.y + last_point.y) } };

			// Compute two boxes for the center from first_point and two more from last_point
			for (int p = 0; p < 3; p++)
			{
				carmen_position_t width_center_position = center_positions[p];
				for (double delta_phi = -M_PI_2; delta_phi < M_PI; delta_phi += M_PI)
				{
					double phi = carmen_normalize_theta(theta + delta_phi);
					for (double direction = 0; direction <= M_PI; direction += M_PI)
					{
						virtual_scan_box_model_t *box = virtual_scan_append_box(box_models, box_points);
						box->c = categories[j].category;
						box->width = categories[j].width;
						box->length = categories[j].length;
						box->x = width_center_position.x + 0.5 * categories[j].length * cos(phi);
						box->y = width_center_position.y + 0.5 * categories[j].length * sin(phi);
						box->theta = carmen_normalize_theta(phi + direction);
					}
				}
			}
		}
	}
}


virtual_scan_box_model_hypotheses_t *
virtual_scan_fit_box_models(virtual_scan_segment_classes_t *virtual_scan_segment_classes)
{
	virtual_scan_category_t categories[] = {{BUS, 2.5, 12.0}, {CAR, 1.7, 4.5}, {BIKE, 0.5, 2.1}}; // Trung-Dung Vu Thesis

	int num_segments = virtual_scan_segment_classes->num_segments;
	virtual_scan_box_model_hypotheses_t *box_model_hypotheses = virtual_scan_new_box_model_hypotheses(num_segments);
	box_model_hypotheses->timestamp = virtual_scan_segment_classes->timestamp;
	for (int i = 0; i < num_segments; i++)
	{
		int segment_class = virtual_scan_segment_classes->segment_features[i].segment_class;
		carmen_point_t first_point = virtual_scan_segment_classes->segment_features[i].first_point;
		carmen_point_t last_point = virtual_scan_segment_classes->segment_features[i].last_point;
		carmen_point_t farthest_point = virtual_scan_segment_classes->segment_features[i].farthest_point;
		carmen_point_t centroid = virtual_scan_segment_classes->segment_features[i].centroid;
		virtual_scan_segment_t box_points = virtual_scan_segment_classes->segment[i];

		virtual_scan_box_models_t *box_models = virtual_scan_get_empty_box_models(box_model_hypotheses);
		if (segment_class == MASS_POINT)
			append_pedestrian_to_box_models(box_models, box_points, centroid);
		else if (segment_class == L_SHAPED) // L-shape segments segments will generate bus and car hypotheses
			append_l_shaped_objects_to_box_models(box_models, box_points, first_point, last_point, farthest_point, categories);
		else if (segment_class == I_SHAPED) // I-shape segments segments will generate bus, car and bike hypotheses
			append_i_shaped_objects_to_box_models(box_models, box_points, first_point, last_point, categories);

		if (!is_last_box_model_hypotheses_empty(box_model_hypotheses))
			box_model_hypotheses->last_box_model_hypotheses++;
	}

	return (box_model_hypotheses);
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
virtual_scan_free_segments(virtual_scan_segments_t *virtual_scan_segments)
{
	free(virtual_scan_segments->segment);
	free(virtual_scan_segments);
}


void
virtual_scan_free_segment_classes(virtual_scan_segment_classes_t *virtual_scan_segment_classes)
{
	if (virtual_scan_segment_classes != NULL)
	{
		free(virtual_scan_segment_classes->segment);
		free(virtual_scan_segment_classes->segment_features);
		free(virtual_scan_segment_classes);
	}
}


void
virtual_scan_free_scan_extended(virtual_scan_extended_t *virtual_scan_extended)
{
	if (virtual_scan_extended != NULL)
	{
		free(virtual_scan_extended->points);
		free(virtual_scan_extended);
	}
}


virtual_scan_segment_classes_t *
virtual_scan_extract_segments(virtual_scan_extended_t *virtual_scan_extended)
{
	virtual_scan_extended_t *virtual_scan_extended_filtered = filter_virtual_scan(virtual_scan_extended);
	virtual_scan_segments_t *virtual_scan_segments = segment_virtual_scan(virtual_scan_extended_filtered);
	virtual_scan_segment_classes_t *virtual_scan_segment_classes = classify_segments(virtual_scan_segments);
	virtual_scan_free_scan_extended(virtual_scan_extended_filtered);
	virtual_scan_free_segments(virtual_scan_segments);

	return (virtual_scan_segment_classes);
}


void 
create_hypothesis_vertex(int h, int i, int j, virtual_scan_neighborhood_graph_t* neighborhood_graph,
		virtual_scan_box_model_hypotheses_t* virtual_scan_box_model_hypotheses)
{
	neighborhood_graph->box_model_hypothesis[h] = (virtual_scan_box_model_hypothesis_t *) malloc(sizeof(virtual_scan_box_model_hypothesis_t));
	neighborhood_graph->box_model_hypothesis[h]->hypothesis = virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box[j];
	neighborhood_graph->box_model_hypothesis[h]->hypothesis_points = virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box_points[j];
	neighborhood_graph->box_model_hypothesis[h]->zi = g_zi;
	neighborhood_graph->box_model_hypothesis[h]->index = h;
	neighborhood_graph->box_model_hypothesis[h]->timestamp = virtual_scan_box_model_hypotheses->timestamp;
}


void
create_hypothesis_sibling_edges(int h, int previous_h, int num_boxes, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	neighborhood_graph->box_model_hypothesis_edges[h] =
			(virtual_scan_box_model_hypothesis_edges_t *) malloc(sizeof(virtual_scan_box_model_hypothesis_edges_t));
	neighborhood_graph->box_model_hypothesis_edges[h]->size = num_boxes - 1;
	if (neighborhood_graph->box_model_hypothesis_edges[h]->size != 0)
	{
		neighborhood_graph->box_model_hypothesis_edges[h]->edge = (int *) malloc((num_boxes - 1) * sizeof(int));
		neighborhood_graph->box_model_hypothesis_edges[h]->edge_type = (int *) malloc((num_boxes - 1) * sizeof(int));
		int e = 0;
		int reference_h = h - previous_h;
		for (int k = 0; k < num_boxes; k++)
		{
			if (k != reference_h)
			{
				neighborhood_graph->box_model_hypothesis_edges[h]->edge[e] = k;
				neighborhood_graph->box_model_hypothesis_edges[h]->edge_type[e] = SIBLING_EDGE;
				e++;
			}
		}
	}
	else
	{
		neighborhood_graph->box_model_hypothesis_edges[h]->edge = NULL;
		neighborhood_graph->box_model_hypothesis_edges[h]->edge_type = NULL;
	}
}


bool
is_parent(int candidate_parent, int child, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (neighborhood_graph->box_model_hypothesis[candidate_parent]->hypothesis.c == neighborhood_graph->box_model_hypothesis[child]->hypothesis.c)
	{
		double distance = DIST2D(neighborhood_graph->box_model_hypothesis[candidate_parent]->hypothesis, neighborhood_graph->box_model_hypothesis[child]->hypothesis);
		double delta_t = neighborhood_graph->box_model_hypothesis[child]->timestamp - neighborhood_graph->box_model_hypothesis[candidate_parent]->timestamp;

		if (distance < delta_t * VMAX)
			return (true);
	}

	return (false);
}


void
create_hypothesis_parent_child_edges(int child, virtual_scan_neighborhood_graph_t *neighborhood_graph, double current_timestamp)
{
	int num_candidate_parent = 0;
	while ((num_candidate_parent < neighborhood_graph->size) && (neighborhood_graph->box_model_hypothesis[num_candidate_parent]->timestamp != current_timestamp))
		num_candidate_parent++;

	for (int candidate_parent = 0; candidate_parent < num_candidate_parent; candidate_parent++)
	{
		if (is_parent(candidate_parent, child, neighborhood_graph))
		{
			int parent = candidate_parent;

			int e = neighborhood_graph->box_model_hypothesis_edges[child]->size;
			neighborhood_graph->box_model_hypothesis_edges[child]->edge = (int *) realloc(neighborhood_graph->box_model_hypothesis_edges[child]->edge,
					(e + 1) * sizeof(int));
			neighborhood_graph->box_model_hypothesis_edges[child]->edge_type = (int *) realloc(neighborhood_graph->box_model_hypothesis_edges[child]->edge_type,
					(e + 1) * sizeof(int));

			neighborhood_graph->box_model_hypothesis_edges[child]->edge[e] = parent;
			neighborhood_graph->box_model_hypothesis_edges[child]->edge_type[e] = PARENT_EDGE;

			neighborhood_graph->box_model_hypothesis_edges[child]->size += 1;

			e = neighborhood_graph->box_model_hypothesis_edges[parent]->size;
			neighborhood_graph->box_model_hypothesis_edges[parent]->edge = (int *) realloc(neighborhood_graph->box_model_hypothesis_edges[parent]->edge,
					(e + 1) * sizeof(int));
			neighborhood_graph->box_model_hypothesis_edges[parent]->edge_type = (int *) realloc(neighborhood_graph->box_model_hypothesis_edges[parent]->edge_type,
					(e + 1) * sizeof(int));

			neighborhood_graph->box_model_hypothesis_edges[parent]->edge[e] = child;
			neighborhood_graph->box_model_hypothesis_edges[parent]->edge_type[e] = CHILD_EDGE;

			neighborhood_graph->box_model_hypothesis_edges[parent]->size += 1;
		}
		candidate_parent++;
	}
}


void
free_neighborhood_graph_vextexes(virtual_scan_neighborhood_graph_t *neighborhood_graph, int vextexes_to_remove)
{
	for (int i = 0; i < vextexes_to_remove; i++)
	{
		free(neighborhood_graph->box_model_hypothesis[i]);
		free(neighborhood_graph->box_model_hypothesis_edges[i]->edge);
		free(neighborhood_graph->box_model_hypothesis_edges[i]->edge_type);
		free(neighborhood_graph->box_model_hypothesis_edges[i]);
	}
}


void
move_neighborhood_graph_vextexes(virtual_scan_neighborhood_graph_t *neighborhood_graph, int vextexes_to_remove)
{
	for (int i = 0; i < (neighborhood_graph->size - vextexes_to_remove); i++)
	{
		neighborhood_graph->box_model_hypothesis[i] = neighborhood_graph->box_model_hypothesis[i + vextexes_to_remove];
		neighborhood_graph->box_model_hypothesis_edges[i] = neighborhood_graph->box_model_hypothesis_edges[i + vextexes_to_remove];
		neighborhood_graph->vertex_selected[i] = neighborhood_graph->vertex_selected[i + vextexes_to_remove];
	}
}


void
remove_edge(virtual_scan_neighborhood_graph_t *neighborhood_graph, int vertex, int edge)
{
	for (int i = edge; i < neighborhood_graph->box_model_hypothesis_edges[vertex]->size - 1; i++)
	{
		neighborhood_graph->box_model_hypothesis_edges[vertex]->edge[i] = neighborhood_graph->box_model_hypothesis_edges[vertex]->edge[i + 1];
		neighborhood_graph->box_model_hypothesis_edges[vertex]->edge_type[i] = neighborhood_graph->box_model_hypothesis_edges[vertex]->edge_type[i + 1];
	}

	neighborhood_graph->box_model_hypothesis_edges[vertex]->size -= 1;
	neighborhood_graph->box_model_hypothesis_edges[vertex]->edge = (int *) realloc(neighborhood_graph->box_model_hypothesis_edges[vertex]->edge,
			neighborhood_graph->box_model_hypothesis_edges[vertex]->size * sizeof(int));
	neighborhood_graph->box_model_hypothesis_edges[vertex]->edge_type = (int *) realloc(neighborhood_graph->box_model_hypothesis_edges[vertex]->edge_type,
			neighborhood_graph->box_model_hypothesis_edges[vertex]->size * sizeof(int));
}


void
rename_neighborhood_graph_vextexes_ids(virtual_scan_neighborhood_graph_t *neighborhood_graph, int vextexes_to_remove)
{
	for (int i = 0; i < (neighborhood_graph->size - vextexes_to_remove); i++)
	{
		neighborhood_graph->box_model_hypothesis[i]->index -= vextexes_to_remove;
		if (neighborhood_graph->box_model_hypothesis[i]->index < 0)
			carmen_die("Error: neighborhood_graph->box_model_hypothesis[i]->index < 0 in rename_neighborhood_graph_vextexes_ids().\n");

		int j = 0;
		while (j < neighborhood_graph->box_model_hypothesis_edges[i]->size)
		{
			neighborhood_graph->box_model_hypothesis_edges[i]->edge[j] -= vextexes_to_remove;
			if (neighborhood_graph->box_model_hypothesis_edges[i]->edge[j] < 0)
				remove_edge(neighborhood_graph, i, j);
			else
				j++;
		}
	}
}


int
remove_graph_vertexes_of_victim_timestamp(double victim_timestamp, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	int vextexes_to_remove = 0;
	while ((vextexes_to_remove < neighborhood_graph->size) && (neighborhood_graph->box_model_hypothesis[vextexes_to_remove]->timestamp == victim_timestamp))
		vextexes_to_remove++;

	free_neighborhood_graph_vextexes(neighborhood_graph, vextexes_to_remove);
	move_neighborhood_graph_vextexes(neighborhood_graph, vextexes_to_remove);
	rename_neighborhood_graph_vextexes_ids(neighborhood_graph, vextexes_to_remove);

	neighborhood_graph->size -= vextexes_to_remove;
	neighborhood_graph->box_model_hypothesis =
			(virtual_scan_box_model_hypothesis_t **) realloc(neighborhood_graph->box_model_hypothesis, neighborhood_graph->size * sizeof(virtual_scan_box_model_hypothesis_t *));
	neighborhood_graph->box_model_hypothesis_edges =
			(virtual_scan_box_model_hypothesis_edges_t **) realloc(neighborhood_graph->box_model_hypothesis_edges, neighborhood_graph->size * sizeof(virtual_scan_box_model_hypothesis_edges_t *));
	neighborhood_graph->vertex_selected = (bool *) realloc(neighborhood_graph->vertex_selected, neighborhood_graph->size * sizeof(bool));

	return (vextexes_to_remove);
}


virtual_scan_box_model_hypothesis_t *
update_track_according_to_new_graph(virtual_scan_track_t *track, int vextexes_removed_from_graph)
{
	int i = 0;
	while (i < track->size)
	{
		track->box_model_hypothesis[i].index -= vextexes_removed_from_graph;
		if (track->box_model_hypothesis[i].index < 0) // Remove hypothesis
		{
			for (int j = i; j < track->size - 1; j++)
				track->box_model_hypothesis[j] = track->box_model_hypothesis[j + 1];

			track->size--;
			track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track->box_model_hypothesis,
					track->size * sizeof(virtual_scan_box_model_hypothesis_t));
		}
		else
			i++;
	}

	if (track->size != 0)
		return (track->box_model_hypothesis);
	else
		return (NULL);
}


virtual_scan_track_set_t *
update_best_track_set_according_to_new_graph(virtual_scan_track_set_t *best_track_set, int vextexes_removed_from_graph)
{
	if (best_track_set == NULL)
		return (NULL);

	int i = 0;
	while (i < best_track_set->size)
	{
		virtual_scan_track_t *track = best_track_set->tracks[i];
		if (update_track_according_to_new_graph(track, vextexes_removed_from_graph) == NULL) // Remove track
		{
			for (int j = i; j < best_track_set->size - 1; j++)
				best_track_set->tracks[j] = best_track_set->tracks[j + 1];

			best_track_set->size--;
			best_track_set->tracks = (virtual_scan_track_t **) realloc(best_track_set->tracks, best_track_set->size * sizeof(virtual_scan_track_t *));
		}
		else
			i++;
	}

	if (best_track_set->size != 0)
		return (best_track_set);
	else
		return (NULL);
}


virtual_scan_neighborhood_graph_t *
first_neighborhood_graph_update(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	virtual_scan_neighborhood_graph_t *neighborhood_graph = (virtual_scan_neighborhood_graph_t *) malloc(sizeof(virtual_scan_neighborhood_graph_t));
	carmen_test_alloc(neighborhood_graph);
	neighborhood_graph->last_frames_timetamps = (double *) malloc(NUMBER_OF_FRAMES_T * sizeof(double));

	int num_hypotheses = 0;
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
		for (int j = 0; j < virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes; j++)
			num_hypotheses++;

	neighborhood_graph->size = 0;
	neighborhood_graph->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t **) malloc(num_hypotheses * sizeof(virtual_scan_box_model_hypothesis_t *));
	neighborhood_graph->box_model_hypothesis_edges =
			(virtual_scan_box_model_hypothesis_edges_t **) malloc(num_hypotheses * sizeof(virtual_scan_box_model_hypothesis_edges_t *));
	neighborhood_graph->vertex_selected = (bool *) malloc(num_hypotheses * sizeof(bool));

	for (int i = 0; i < num_hypotheses; i++)
		neighborhood_graph->vertex_selected[i] = false;

	int h = 0;
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
	{
		int num_boxes = virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes;
		int previous_h = h;
		for (int j = 0; j < num_boxes; j++, h++)
			create_hypothesis_vertex(h, i, j, neighborhood_graph, virtual_scan_box_model_hypotheses);
		h = previous_h;
		for (int j = 0; j < num_boxes; j++, h++)
			create_hypothesis_sibling_edges(h, previous_h, num_boxes, neighborhood_graph);
	}
	neighborhood_graph->size = num_hypotheses;

	neighborhood_graph->number_of_frames_filled = 0;
	neighborhood_graph->last_frames_timetamps[neighborhood_graph->number_of_frames_filled] = virtual_scan_box_model_hypotheses->timestamp;
	neighborhood_graph->number_of_frames_filled = 1;

	return (neighborhood_graph);
}


virtual_scan_neighborhood_graph_t *
neighborhood_graph_update(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	int num_hypotheses = 0;
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
		for (int j = 0; j < virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes; j++)
			num_hypotheses++;

	int first_new_vertex_to_add = neighborhood_graph->size;
	neighborhood_graph->box_model_hypothesis =
			(virtual_scan_box_model_hypothesis_t **) realloc(neighborhood_graph->box_model_hypothesis, (neighborhood_graph->size + num_hypotheses) * sizeof(virtual_scan_box_model_hypothesis_t *));
	neighborhood_graph->box_model_hypothesis_edges =
			(virtual_scan_box_model_hypothesis_edges_t **) realloc(neighborhood_graph->box_model_hypothesis_edges, (neighborhood_graph->size + num_hypotheses) * sizeof(virtual_scan_box_model_hypothesis_edges_t *));
	neighborhood_graph->vertex_selected = (bool *) realloc(neighborhood_graph->vertex_selected, (neighborhood_graph->size + num_hypotheses) * sizeof(bool));

	for (int i = first_new_vertex_to_add; i < (neighborhood_graph->size + num_hypotheses); i++)
		neighborhood_graph->vertex_selected[i] = false;

	int h = first_new_vertex_to_add; // Each vertex hold an hypothesis
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
	{
		int num_boxes = virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes;
		int previous_h = h;
		for (int j = 0; j < num_boxes; j++, h++)
			create_hypothesis_vertex(h, i, j, neighborhood_graph, virtual_scan_box_model_hypotheses);
		h = previous_h;
		for (int j = 0; j < num_boxes; j++, h++)
		{
			create_hypothesis_sibling_edges(h, previous_h, num_boxes, neighborhood_graph);
			create_hypothesis_parent_child_edges(h, neighborhood_graph, virtual_scan_box_model_hypotheses->timestamp);
		}
	}
	neighborhood_graph->size += num_hypotheses;

	if (neighborhood_graph->number_of_frames_filled == NUMBER_OF_FRAMES_T)
	{
		double victim_timestamp = neighborhood_graph->last_frames_timetamps[0];
		for (int i = 0; i < NUMBER_OF_FRAMES_T - 1; i++)
			neighborhood_graph->last_frames_timetamps[i] = neighborhood_graph->last_frames_timetamps[i + 1];

		int vextexes_removed_from_graph = remove_graph_vertexes_of_victim_timestamp(victim_timestamp, neighborhood_graph);
		best_track_set = update_best_track_set_according_to_new_graph(best_track_set, vextexes_removed_from_graph);

		neighborhood_graph->number_of_frames_filled -= 1;
	}

	neighborhood_graph->last_frames_timetamps[neighborhood_graph->number_of_frames_filled] = virtual_scan_box_model_hypotheses->timestamp;
	neighborhood_graph->number_of_frames_filled += 1;

	return (neighborhood_graph);
}


void
print_neighborhood_graph(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	FILE *graph = fopen("graph.txt", "a");
	for (int i = 0; i < neighborhood_graph->size; i++)
	{
		fprintf(graph, "%d %c -", i, neighborhood_graph->box_model_hypothesis[i]->hypothesis.c);
		for (int j = 0; j < neighborhood_graph->box_model_hypothesis_edges[i]->size; j++)
			fprintf(graph, " %c(%d, %d)", neighborhood_graph->box_model_hypothesis_edges[i]->edge_type[j], i, neighborhood_graph->box_model_hypothesis_edges[i]->edge[j]);
		fprintf(graph, "\n");
	}
	fprintf(graph, "\n");
	fclose(graph);
}


virtual_scan_neighborhood_graph_t *
virtual_scan_update_neighborhood_graph(virtual_scan_neighborhood_graph_t *neighborhood_graph, virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	if (virtual_scan_box_model_hypotheses == NULL)
		return (neighborhood_graph);

	if (neighborhood_graph == NULL)
		neighborhood_graph = first_neighborhood_graph_update(virtual_scan_box_model_hypotheses);
	else
		neighborhood_graph = neighborhood_graph_update(virtual_scan_box_model_hypotheses, neighborhood_graph);

//	print_neighborhood_graph(neighborhood_graph);

	return (neighborhood_graph);
}


int *
get_v_star(int &size, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	size = 0;
	for (int i = 0; i < neighborhood_graph->size; i++)
	{
		if (!neighborhood_graph->vertex_selected[i])
			size++;
	}

	if (size != 0)
	{
		int *v_star = (int *) malloc(size * sizeof(int));
		for (int i = 0, j = 0; i < neighborhood_graph->size; i++)
		{
			if (!neighborhood_graph->vertex_selected[i])
			{
				v_star[j] = i;
				j++;
			}
		}
		return (v_star);
	}
	else
		return (NULL);
}


carmen_point_t
vect2d(carmen_point_t p1, carmen_point_t p2)
{
    carmen_point_t temp;

    temp.x = (p2.x - p1.x);
    temp.y = -1.0 * (p2.y - p1.y);

    return (temp);
}


int
point_inside_scaled_rectangle(carmen_point_t point, virtual_scan_box_model_t hypothesis, double scale)
{
	carmen_point_t A, B, C, D, m;

	double length = hypothesis.length * scale;
	double width = hypothesis.width * scale;

	m = point;

	A.x = hypothesis.x - length / 2.0;
	A.y = hypothesis.y - width / 2.0;
	B.x = hypothesis.x + length / 2.0;
	B.y = hypothesis.y - width / 2.0;
	C.x = hypothesis.x + length / 2.0;
	C.y = hypothesis.y + width / 2.0;
	D.x = hypothesis.x - length / 2.0;
	D.y = hypothesis.y + width / 2.0;

	carmen_point_t AB = vect2d(A, B);
	double C1 = -1.0 * (AB.y * A.x + AB.x * A.y);
	double D1 = (AB.y * m.x + AB.x * m.y) + C1;

	carmen_point_t AD = vect2d(A, D);
	double C2 = -1.0 * (AD.y * A.x + AD.x * A.y);
	double D2 = (AD.y * m.x + AD.x * m.y) + C2;

	carmen_point_t BC = vect2d(B, C);
	double C3 = -1.0 * (BC.y * B.x + BC.x * B.y);
	double D3 = (BC.y * m.x + BC.x * m.y) + C3;

	carmen_point_t CD = vect2d(C, D);
	double C4 = -1.0 * (CD.y * C.x + CD.x * C.y);
	double D4 = (CD.y * m.x + CD.x * m.y) + C4;

	if ((0.0 >= D1 && 0.0 >= D4 && 0.0 <= D2 && 0.0 >= D3))
		return (1);
	else
		return (0);
}


void
get_points_inside_and_outside_scaled_rectangle(carmen_point_t *&points_inside_rectangle, carmen_point_t *&points_outside_rectangle,
		int &num_points_inside_rectangle, int &num_points_outside_rectangle,
		virtual_scan_box_model_hypothesis_t *box_model_hypothesis, double scale)
{
	int num_points = g_virtual_scan_extended[box_model_hypothesis->zi]->num_points;
	carmen_point_t *point = g_virtual_scan_extended[box_model_hypothesis->zi]->points;

	points_inside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	points_outside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	num_points_inside_rectangle = num_points_outside_rectangle = 0;
	for (int i = 0; i < num_points; i++)
	{
		if (point_inside_scaled_rectangle(point[i], box_model_hypothesis->hypothesis, scale))
		{
			points_inside_rectangle[num_points_inside_rectangle] = point[i];
			num_points_inside_rectangle++;
		}
		else
		{
			points_outside_rectangle[num_points_outside_rectangle] = point[i];
			num_points_outside_rectangle++;
		}
	}
}


void
get_points_inside_and_outside_scaled_rectangle(carmen_point_t *&points_inside_rectangle, carmen_point_t *&points_outside_rectangle,
		int &num_points_inside_rectangle, int &num_points_outside_rectangle, carmen_point_t *point, int num_points,
		virtual_scan_box_model_hypothesis_t *box_model_hypothesis, double scale)
{
	points_inside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	points_outside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	num_points_inside_rectangle = num_points_outside_rectangle = 0;
	for (int i = 0; i < num_points; i++)
	{
		if (point_inside_scaled_rectangle(point[i], box_model_hypothesis->hypothesis, scale))
		{
			points_inside_rectangle[num_points_inside_rectangle] = point[i];
			num_points_inside_rectangle++;
		}
		else
		{
			points_outside_rectangle[num_points_outside_rectangle] = point[i];
			num_points_outside_rectangle++;
		}
	}
}


double
get_distance_to_hypothesis_rectangle(carmen_point_t zd_point, virtual_scan_box_model_hypothesis_t *box_model_hypothesis)
{
	virtual_scan_box_model_t hypothesis = box_model_hypothesis->hypothesis;
	carmen_point_t v, w;
	double d, new_d;

	v.x = hypothesis.x - hypothesis.length / 2.0;
	v.y = hypothesis.y - hypothesis.width / 2.0;
	w.x = hypothesis.x + hypothesis.length / 2.0;
	w.y = hypothesis.y - hypothesis.width / 2.0;
	d = distance_from_point_to_line_segment_vw(v, w, zd_point);

	w.x = hypothesis.x - hypothesis.length / 2.0;
	w.y = hypothesis.y + hypothesis.width / 2.0;
	new_d = distance_from_point_to_line_segment_vw(v, w, zd_point);
	if (new_d < d)
		d = new_d;

	v.x = hypothesis.x + hypothesis.length / 2.0;
	v.y = hypothesis.y + hypothesis.width / 2.0;
	new_d = distance_from_point_to_line_segment_vw(v, w, zd_point);
	if (new_d < d)
		d = new_d;

	w.x = hypothesis.x + hypothesis.length / 2.0;
	w.y = hypothesis.y - hypothesis.width / 2.0;
	new_d = distance_from_point_to_line_segment_vw(v, w, zd_point);
	if (new_d < d)
		d = new_d;

	return (d);
}


void
get_Zs_and_Zd_of_hypothesis(carmen_point_t *&Zs_in, int &Zs_in_size, carmen_point_t *&Zs_out, int &Zs_out_size,
		carmen_point_t *&Zd, int &Zd_size, virtual_scan_box_model_hypothesis_t *box_model_hypothesis)
{
	int num_points_inside_large_rectangle;
	carmen_point_t *points_inside_large_rectangle;

	get_points_inside_and_outside_scaled_rectangle(points_inside_large_rectangle, Zs_out,
			num_points_inside_large_rectangle, Zs_out_size, box_model_hypothesis, 1.3);

	get_points_inside_and_outside_scaled_rectangle(Zs_in, Zd, Zs_in_size, Zd_size,
			points_inside_large_rectangle, num_points_inside_large_rectangle, box_model_hypothesis, 0.7);

	free(points_inside_large_rectangle);
}


double
PM1(carmen_point_t *Zd, int Zd_size, virtual_scan_box_model_hypothesis_t *box_model_hypothesis)
{
	double sum = 0.0;
	for (int i = 0; i < Zd_size; i++)
		sum += get_distance_to_hypothesis_rectangle(Zd[i], box_model_hypothesis);

	return (sum);
}


double
Det(double a, double b, double c, double d)
{
	return (a * d - b * c);
}


bool
line_to_line_intersection(double x1, double y1, double x2, double y2,
	double x3, double y3, double x4, double y4)
{
	//http://mathworld.wolfram.com/Line-LineIntersection.html
	//https://gist.github.com/TimSC/47203a0f5f15293d2099507ba5da44e6

	double detL1 = Det(x1, y1, x2, y2);
	double detL2 = Det(x3, y3, x4, y4);
	double x1mx2 = x1 - x2;
	double x3mx4 = x3 - x4;
	double y1my2 = y1 - y2;
	double y3my4 = y3 - y4;

	double xnom = Det(detL1, x1mx2, detL2, x3mx4);
	double ynom = Det(detL1, y1my2, detL2, y3my4);
	double denom = Det(x1mx2, y1my2, x3mx4, y3my4);
	if (denom == 0.0) //Lines don't seem to cross
		return (false);

	double ixOut = xnom / denom;
	double iyOut = ynom / denom;
	if(!std::isfinite(ixOut) || !std::isfinite(iyOut)) //Probably a numerical issue
		return (false);

	return (true);
}


int
line_to_point_crossed_rectangle(carmen_point_t point, virtual_scan_box_model_t hypothesis, carmen_point_t origin)
{
	double x1, y1, x2, y2, x3, y3, x4, y4;

	x1 = origin.x;
	y1 = origin.y;
	x2 = point.x;
	y2 = point.y;

	x3 = hypothesis.x - hypothesis.length / 2.0;
	y3 = hypothesis.y - hypothesis.width / 2.0;
	x4 = hypothesis.x + hypothesis.length / 2.0;
	y4 = hypothesis.y - hypothesis.width / 2.0;
	if (line_to_line_intersection(x1, y1, x2, y2, x3, y3, x4, y4))
		return (1);

	x4 = hypothesis.x - hypothesis.length / 2.0;
	y4 = hypothesis.y + hypothesis.width / 2.0;
	if (line_to_line_intersection(x1, y1, x2, y2, x3, y3, x4, y4))
		return (1);

	x3 = hypothesis.x + hypothesis.length / 2.0;
	y3 = hypothesis.y + hypothesis.width / 2.0;
	if (line_to_line_intersection(x1, y1, x2, y2, x3, y3, x4, y4))
		return (1);

	x4 = hypothesis.x + hypothesis.length / 2.0;
	y4 = hypothesis.y - hypothesis.width / 2.0;
	if (line_to_line_intersection(x1, y1, x2, y2, x3, y3, x4, y4))
		return (1);

	return (0);
}


double
PM2(carmen_point_t *Zs, int Zs_size, virtual_scan_box_model_hypothesis_t *box_model_hypothesis)
{
	double sum = 0.0;
	for (int i = 0; i < Zs_size; i++)
		sum += (double) line_to_point_crossed_rectangle(Zs[i], box_model_hypothesis->hypothesis, g_virtual_scan_extended[box_model_hypothesis->zi]->globalpos);

	return (sum);
}


void
compute_posterior_probability_components(virtual_scan_track_set_t *track_set)
{
	if (track_set == NULL)
		return;

	for (int i = 0; i < track_set->size; i++)
	{
		virtual_scan_track_t *track = track_set->tracks[i];
		for (int h = 0; h < track->size; h++)
		{
			virtual_scan_box_model_hypothesis_t *box_model_hypothesis = &(track->box_model_hypothesis[h]);

			carmen_point_t *Zs_out, *Zs_in, *Zd; int Zs_out_size, Zs_in_size, Zd_size;
			get_Zs_and_Zd_of_hypothesis(Zs_out, Zs_out_size, Zs_in, Zs_in_size, Zd, Zd_size, box_model_hypothesis);

			box_model_hypothesis->dn = PM1(Zd, Zd_size, box_model_hypothesis);
			box_model_hypothesis->c2 = PM2(Zs_out, Zs_out_size, box_model_hypothesis);
			box_model_hypothesis->c3 = 0.0;
			free(Zs_out); free(Zs_in); free(Zd);
		}
	}
}


virtual_scan_track_t *
track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (neighborhood_graph != NULL)
	{
		int v_star_size;
		int *v_star = get_v_star(v_star_size, neighborhood_graph);
		if (v_star != NULL)
		{
			int rand_v = carmen_int_random(v_star_size);
			virtual_scan_track_t *new_track = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
			new_track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) malloc(sizeof(virtual_scan_box_model_hypothesis_t));
			new_track->size = 1;
			new_track->box_model_hypothesis[0] = *(neighborhood_graph->box_model_hypothesis[v_star[rand_v]]);
			neighborhood_graph->vertex_selected[v_star[rand_v]] = true;

			free(v_star);
			return (new_track);
		}
		else
			return (NULL);
	}
	else
		return (NULL);
}


virtual_scan_track_set_t *
add_track(virtual_scan_track_set_t *track_set_n_1, virtual_scan_track_t *new_track)
{
	if (track_set_n_1 == NULL)
	{
		track_set_n_1 = (virtual_scan_track_set_t *) malloc(sizeof(virtual_scan_track_set_t));
		track_set_n_1->tracks = (virtual_scan_track_t **) malloc(sizeof(virtual_scan_track_t *));
		track_set_n_1->tracks[0] = new_track;
		track_set_n_1->size = 1;
	}
	else
	{
		track_set_n_1->tracks = (virtual_scan_track_t **) realloc(track_set_n_1->tracks, (track_set_n_1->size + 1) * sizeof(virtual_scan_track_t *));
		track_set_n_1->tracks[track_set_n_1->size] = new_track;
		track_set_n_1->size += 1;
	}

	return (track_set_n_1);
}


void
add_hypothesis_at_the_end(virtual_scan_track_t *track, virtual_scan_box_model_hypothesis_t *hypothesis, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (track == NULL)
		carmen_die("track == NULL in add_hypothesis_at_the_end()\n");

	if (hypothesis == NULL)
		return;

	track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track->box_model_hypothesis,
			(track->size + 1) * sizeof(virtual_scan_box_model_hypothesis_t));

	track->box_model_hypothesis[track->size] = *hypothesis;
	track->size += 1;

	neighborhood_graph->vertex_selected[hypothesis->index] = true;
}


void
add_hypothesis_at_the_beginning(virtual_scan_track_t *track, virtual_scan_box_model_hypothesis_t *hypothesis, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (track == NULL)
		carmen_die("track == NULL in add_hypothesis_at_the_beginning()\n");

	if (hypothesis == NULL)
		return;

	track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track->box_model_hypothesis,
			(track->size + 1) * sizeof(virtual_scan_box_model_hypothesis_t));
	for (int i = track->size; i > 0; i--)
		track->box_model_hypothesis[i] = track->box_model_hypothesis[i - 1];

	track->box_model_hypothesis[0] = *hypothesis;
	track->size += 1;

	neighborhood_graph->vertex_selected[hypothesis->index] = true;
}


int *
get_neighbors_within_v_star(int neighbor_type, int &number_of_neighbors, virtual_scan_box_model_hypothesis_edges_t *hypothesis_neighbors,
		virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (hypothesis_neighbors == NULL)
		return (NULL);

	number_of_neighbors = 0;
	for (int i = 0; i < hypothesis_neighbors->size; i++)
	{
		// if (neighborhood_graph->vertex_selected[hypothesis_neighbors->edge[i]] == true), this neighbor is within v_star
		if ((hypothesis_neighbors->edge_type[i] == neighbor_type) && !(neighborhood_graph->vertex_selected[hypothesis_neighbors->edge[i]]))
			number_of_neighbors++;
	}

	if (number_of_neighbors != 0)
	{
		int *neighbors = (int *) malloc(number_of_neighbors * sizeof(int));
		for (int i = 0, j = 0; i < hypothesis_neighbors->size; i++)
		{
			// if (neighborhood_graph->vertex_selected[hypothesis_neighbors->edge[i]] == true), this neighbor is within v_star
			if ((hypothesis_neighbors->edge_type[i] == neighbor_type) && !(neighborhood_graph->vertex_selected[hypothesis_neighbors->edge[i]]))
			{
				neighbors[j] = i;
				j++;
			}
		}
		return (neighbors);
	}
	else
		return (NULL);
}


virtual_scan_box_model_hypothesis_t *
get_child_hypothesis_in_v_star_and_at_the_end_of_track(virtual_scan_track_t *track, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	virtual_scan_box_model_hypothesis_t end_of_track = track->box_model_hypothesis[track->size - 1];
	virtual_scan_box_model_hypothesis_edges_t *hypothesis_neighbors = neighborhood_graph->box_model_hypothesis_edges[end_of_track.index];
	int number_of_children;
	int *hypothesis_children = get_neighbors_within_v_star(CHILD_EDGE, number_of_children, hypothesis_neighbors, neighborhood_graph);
	if (hypothesis_children != NULL)
	{
		int rand_hypothesis = carmen_int_random(number_of_children);
		virtual_scan_box_model_hypothesis_t *child_hypothesis = neighborhood_graph->box_model_hypothesis[hypothesis_children[rand_hypothesis]];
		free(hypothesis_children);

		return (child_hypothesis);
	}
	else
		return (NULL);
}


virtual_scan_box_model_hypothesis_t *
get_child_hypothesis_in_v_star_and_at_the_beginning_of_track(virtual_scan_track_t *track, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	virtual_scan_box_model_hypothesis_t beginning_of_track = track->box_model_hypothesis[0];
	virtual_scan_box_model_hypothesis_edges_t *hypothesis_neighbors = neighborhood_graph->box_model_hypothesis_edges[beginning_of_track.index];
	int number_of_parents;
	int *hypothesis_parents = get_neighbors_within_v_star(PARENT_EDGE, number_of_parents, hypothesis_neighbors, neighborhood_graph);
	if (hypothesis_parents != NULL)
	{
		int rand_hypothesis = carmen_int_random(number_of_parents);
		virtual_scan_box_model_hypothesis_t *parent_hypothesis = neighborhood_graph->box_model_hypothesis[hypothesis_parents[rand_hypothesis]];
		free(hypothesis_parents);

		return (parent_hypothesis);
	}
	else
		return (NULL);
}


void
track_extension(virtual_scan_track_t *track, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	virtual_scan_box_model_hypothesis_t *hypothesis;

	while (carmen_uniform_random(0.0, 1.0) > GAMMA)
	{
		if (carmen_int_random(2) == 0)
		{
			hypothesis = get_child_hypothesis_in_v_star_and_at_the_end_of_track(track, neighborhood_graph);
			add_hypothesis_at_the_end(track, hypothesis, neighborhood_graph);
		}
		else
		{
			hypothesis = get_child_hypothesis_in_v_star_and_at_the_beginning_of_track(track, neighborhood_graph);
			add_hypothesis_at_the_beginning(track, hypothesis, neighborhood_graph);
		}
	}
}


void
track_reduction(virtual_scan_track_t *track)
{
	if (track->size > 2)
	{
		int r = carmen_int_random(track->size - 2) + 1; // r entre 1 e (track->size - 2). Note que, diferente do paper, nossa numeracao comecca de 0
		if (carmen_int_random(2) == 0)
		{	// forward reduction
			track->size = r + 1;
		}
		else
		{	// backward reduction
			track->size -= r;
			memmove((void *) &(track->box_model_hypothesis[0]), (void *) &(track->box_model_hypothesis[r]),
					track->size * sizeof(virtual_scan_box_model_hypothesis_t));
		}
		track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track->box_model_hypothesis,
				track->size * sizeof(virtual_scan_box_model_hypothesis_t));
	}
}


//void
//track_death(virtual_scan_track_t *track)
//{
//
//}
//
//
//void
//track_split(virtual_scan_track_t *track)
//{
//
//}
//
//
//void
//track_merge(virtual_scan_track_set_t *track_set, int rand_track, int rand_track2)
//{
//
//}
//
//
//void
//track_diffusion(virtual_scan_track_t *track)
//{
//
//}


virtual_scan_track_set_t *
copy_track_set(virtual_scan_track_set_t *track_set_n_1)
{
	if (track_set_n_1 != NULL)
	{
		virtual_scan_track_set_t *track_set = (virtual_scan_track_set_t *) malloc(sizeof(virtual_scan_track_set_t));
		track_set->tracks = (virtual_scan_track_t **) malloc(track_set_n_1->size * sizeof(virtual_scan_track_t *));
		track_set->size = track_set_n_1->size;
		for (int i = 0; i < track_set_n_1->size; i++)
		{
			track_set->tracks[i] = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
			track_set->tracks[i]->size = track_set_n_1->tracks[i]->size;
			track_set->tracks[i]->box_model_hypothesis =
					(virtual_scan_box_model_hypothesis_t *) malloc(track_set_n_1->tracks[i]->size * sizeof(virtual_scan_box_model_hypothesis_t));
			for (int j = 0; j < track_set_n_1->tracks[i]->size; j++)
				track_set->tracks[i]->box_model_hypothesis[j] = track_set_n_1->tracks[i]->box_model_hypothesis[j];
		}

		return (track_set);
	}
	else
		return (NULL);
}


virtual_scan_track_set_t *
propose_track_set_according_to_q(virtual_scan_neighborhood_graph_t *neighborhood_graph, virtual_scan_track_set_t *track_set_n_1)
{
#define NUMBER_OF_TYPES_OF_MOVES	3

	int rand_track;

	int rand_move = carmen_int_random(NUMBER_OF_TYPES_OF_MOVES);
	if ((track_set_n_1 != NULL) && (track_set_n_1->size != 0))
		rand_track = carmen_int_random(track_set_n_1->size);
	else
		rand_track = -1;

	virtual_scan_track_set_t *track_set = copy_track_set(track_set_n_1);
	switch (rand_move)
	{
		case 0:	// Birth
			{
				virtual_scan_track_t *new_track = track_birth(neighborhood_graph);
				if (new_track != NULL)
				{
					track_set = add_track(track_set, new_track);
					track_extension(new_track, neighborhood_graph);
				}
			}
			break;
		case 1:	// Extension
			if (rand_track != -1)
				track_extension(track_set->tracks[rand_track], neighborhood_graph);
			break;
		case 2:	// Reduction
			if (rand_track != -1)
				track_reduction(track_set->tracks[rand_track]);
			break;
//		case 3: //Death
//			if (rand_track != -1)
//				track_death(&(track_set->tracks[rand_track]));
//			break;
//		case 4:	// Split
//			if (rand_track != -1)
//			{
//				if (track_set->tracks[rand_track].size >= 4)
//					track_split(&(track_set->tracks[rand_track]));
//			}
//			break;
//		case 5:	// Merge
//			if (rand_track != -1)
//			{
//				int rand_track2 = carmen_int_random(track_set->size);
//				track_merge(track_set, rand_track, rand_track2);
//			}
//			break;
//		case 6:	// Diffusion
//			if (rand_track != -1)
//				track_diffusion(&track_set->tracks[rand_track]);
//			break;
	}

	compute_posterior_probability_components(track_set);

	return (track_set);
}


double
sum_of_tracks_lengths(virtual_scan_track_set_t *track_set)
{
	double sum = 0.0;

	for (int i = 0; i < track_set->size; i++)
		sum += track_set->tracks[i]->size;

	return (sum);
}


double
sum_of_measurements_that_fall_inside_object_models_in_track_set(virtual_scan_track_set_t *track_set)
{
	double sum = 0.0;

	for (int i = 0; i < track_set->size; i++)
	{
		for (int j = 0; j < track_set->tracks[i]->size; j++)
			sum += track_set->tracks[i]->box_model_hypothesis[j].c3;
	}

	return (sum);
}


double
sum_of_number_of_non_maximal_measurements_that_fall_behind_the_object_model(virtual_scan_track_set_t *track_set)
{
	double sum = 0.0;

	for (int i = 0; i < track_set->size; i++)
	{
		for (int j = 0; j < track_set->tracks[i]->size; j++)
			sum += track_set->tracks[i]->box_model_hypothesis[j].c2;
	}

	return (sum);
}


double
sum_of_dn_of_tracks(virtual_scan_track_set_t *track_set)
{
	double sum = 0.0;

	for (int i = 0; i < track_set->size; i++)
	{
		for (int j = 0; j < track_set->tracks[i]->size; j++)
			sum += track_set->tracks[i]->box_model_hypothesis[j].dn;
	}

	return (sum);
}


double
probability_of_track_set_given_measurements(virtual_scan_track_set_t *track_set)
{
#define lambda_L	0.5
#define lambda_1	0.5
#define lambda_2	0.5
#define lambda_3	0.5

	if (track_set == NULL)
		return (0.0);

	double Slen = sum_of_tracks_lengths(track_set);
	double Sms1 = sum_of_dn_of_tracks(track_set);
	double Sms2 = sum_of_number_of_non_maximal_measurements_that_fall_behind_the_object_model(track_set);
	double Sms3 = sum_of_measurements_that_fall_inside_object_models_in_track_set(track_set);

	double p_w_z = exp(lambda_L * Slen - lambda_1 * Sms1 - lambda_2 * Sms2 - lambda_3 * Sms3);

	return (p_w_z);
}


void
free_track_set(virtual_scan_track_set_t *track_set_victim)
{
	if (track_set_victim != NULL)
	{
		for (int i = 0; i < track_set_victim->size; i++)
		{
			free(track_set_victim->tracks[i]->box_model_hypothesis);
			free(track_set_victim->tracks[i]);
		}
		free(track_set_victim->tracks);
		free(track_set_victim);
	}
}


carmen_moving_objects_point_clouds_message *
get_moving_objects_from_track_set(virtual_scan_track_set_t *best_track_set)
{
	carmen_moving_objects_point_clouds_message *message = (carmen_moving_objects_point_clouds_message *) malloc(sizeof(carmen_moving_objects_point_clouds_message));
	message->host = carmen_get_host();
	message->timestamp = carmen_get_time();

	int num_moving_objects = 0;
	for (int i = 0; i < best_track_set->size; i++)
		num_moving_objects += best_track_set->tracks[i]->size;

	message->point_clouds = (t_point_cloud_struct *) malloc(sizeof(t_point_cloud_struct) * num_moving_objects);
	message->num_point_clouds = num_moving_objects;

	int k = 0;
	for (int i = 0; i < best_track_set->size; i++)
	{
		for (int j = 0; j < best_track_set->tracks[i]->size; j++)
		{
			virtual_scan_box_model_t *box = &(best_track_set->tracks[i]->box_model_hypothesis[j].hypothesis);

			message->point_clouds[k].r = 0.0;
			message->point_clouds[k].g = 0.0;
			message->point_clouds[k].b = 1.0;
			message->point_clouds[k].linear_velocity = 0;
			message->point_clouds[k].orientation = box->theta;
			message->point_clouds[k].object_pose.x = box->x;
			message->point_clouds[k].object_pose.y = box->y;
			message->point_clouds[k].object_pose.z = 0.0;
			message->point_clouds[k].height = 0;
			message->point_clouds[k].length = box->length;
			message->point_clouds[k].width = box->width;
			message->point_clouds[k].geometric_model = box->c;
			message->point_clouds[k].point_size = 0; // 1
//			message->point_clouds[k].num_associated = timestamp_moving_objects_list[current_vector_index].objects[i].id;

			object_model_features_t &model_features = message->point_clouds[k].model_features;
			model_features.model_id = box->c;
			model_features.model_name = (char *) "name?";
			model_features.geometry.length = box->length;
			model_features.geometry.width = box->width;

//			message->point_clouds[k].points = (carmen_vector_3D_t *) malloc(1 * sizeof(carmen_vector_3D_t));
//			message->point_clouds[k].points[0].x = box->x;
//			message->point_clouds[k].points[0].y = box->y;
//			message->point_clouds[k].points[0].z = 0.0;

			k++;
		}
	}

	return (message);
}


void
virtual_scan_free_moving_objects(carmen_moving_objects_point_clouds_message *moving_objects)
{
	free(moving_objects->point_clouds);
	free(moving_objects);
}


double
A(virtual_scan_track_set_t *track_set_n, virtual_scan_track_set_t *track_set_prime)
{
	double pi_w_prime = probability_of_track_set_given_measurements(track_set_prime);
	double pi_w_n_1 = probability_of_track_set_given_measurements(track_set_n);
	double q_w_w_prime = 1.0 / NUMBER_OF_TYPES_OF_MOVES;
	double q_w_prime_w = 1.0 / NUMBER_OF_TYPES_OF_MOVES;

	if (q_w_w_prime == 0.0)
		return (0.0);
	else if (pi_w_n_1 == 0.0)
		return (1.0);

	double A_w_w_prime = carmen_fmin(1.0, (pi_w_prime * q_w_w_prime) / (pi_w_n_1 * q_w_prime_w));

	return (A_w_w_prime);
}


carmen_moving_objects_point_clouds_message *
virtual_scan_infer_moving_objects(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	virtual_scan_track_set_t *track_set_n = copy_track_set(best_track_set);
	int n;
	for (n = 0; n < MCMC_MAX_ITERATIONS; n++)
	{
		virtual_scan_track_set_t *track_set_prime = propose_track_set_according_to_q(neighborhood_graph, track_set_n);
		virtual_scan_track_set_t *track_set_victim = track_set_prime;
		double U = carmen_uniform_random(0.0, 1.0);
		if (U < A(track_set_n, track_set_prime))
		{
			track_set_victim = track_set_n;
			track_set_n = track_set_prime;
			if (probability_of_track_set_given_measurements(track_set_n) > probability_of_track_set_given_measurements(best_track_set))
			{
				free_track_set(best_track_set);
				best_track_set = copy_track_set(track_set_n);
			}
		}
		free_track_set(track_set_victim);
	}
	free_track_set(track_set_n);

	carmen_moving_objects_point_clouds_message *moving_objects = get_moving_objects_from_track_set(best_track_set);

	return (moving_objects);
}
