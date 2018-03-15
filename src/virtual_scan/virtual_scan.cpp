/*
 * virtual_scan.cpp
 *
 *  Created on: Jul 17, 2017
 *	  Author: claudine
 */
#include <carmen/carmen.h>
#include <algorithm>
#include <string.h>
#include "virtual_scan.h"

#define PROB_THRESHOLD	-2.14

#define	POINT_WITHIN_SEGMENT		0
#define	SEGMENT_TOO_SHORT			1
#define	POINT_BEFORE_SEGMENT		2
#define	POINT_AFTER_SEGMENT			3

#define	BUS			0 // Width: 2,4 m to 2,6 m; Length: 10 m to 14 m;
#define	CAR			1 // Width: 1,8 m to 2,1; Length: 3,9 m to 5,3 m
#define	BIKE		2 // Width: 1,20 m; Length: 2,20 m
#define	PEDESTRIAN	3

#define PEDESTRAIN_SIZE	0.3 // Pedestrian approximate size (from the top) in meters
#define	L_SMALL_SEGMENT_AS_A_PROPORTION_OF_THE_LARGE	0.3

#define	MCMC_MAX_ITERATIONS	100


virtual_scan_extended_t extended_virtual_scan; // Melhorar
carmen_point_t extended_virtual_scan_points[10000]; // Melhorar
extern carmen_localize_ackerman_map_t localize_map;
extern double x_origin;
extern double y_origin;
extern double map_resolution;

virtual_scan_track_t *best_tracks = NULL;
int g_number_of_tracks = 0;


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
	extended_virtual_scan.points = extended_virtual_scan_points;
	extended_virtual_scan.num_points = virtual_scan->num_points;
	extended_virtual_scan.globalpos = virtual_scan->globalpos;
	extended_virtual_scan.timestamp = virtual_scan->timestamp;
	for (int i = 0; i < virtual_scan->num_points; i++)
	{
		extended_virtual_scan.points[i].x = virtual_scan->points[i].x;
		extended_virtual_scan.points[i].y = virtual_scan->points[i].y;
		double theta = atan2(virtual_scan->points[i].y - virtual_scan->globalpos.y, virtual_scan->points[i].x - virtual_scan->globalpos.x);
		theta = carmen_normalize_theta(theta - virtual_scan->globalpos.theta);
		extended_virtual_scan.points[i].theta = theta;
	}
	qsort((void *) (extended_virtual_scan.points), (size_t) extended_virtual_scan.num_points, sizeof(carmen_point_t), compare_angles);

	return (&extended_virtual_scan);
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
		if (localize_map.prob[x_index_map][y_index_map] < PROB_THRESHOLD)
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
				virtual_scan_segments->segment[segment_id].num_points = i - begin_segment + 1;
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
		carmen_point_t farthest_point;
		double width = 0.0;
		double length = 0.0;
		int segment_class = MASS_POINT;
//		double average_distance = 0.0;
		for (int j = 0; j < num_points; j++)
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
					{
						segment_class = L_SHAPED;
					}
				}
			}
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


//virtual_scan_box_models_t *
//virtual_scan_new_box_models(void)
//{
//	return (virtual_scan_box_models_t*) calloc(1, sizeof(virtual_scan_box_models_t));
//}


virtual_scan_box_model_t *
virtual_scan_append_box(virtual_scan_box_models_t *models)
{
	int n = models->num_boxes + 1;
	models->num_boxes = n;

	if (n == 1)
		models->box = (virtual_scan_box_model_t*) malloc(sizeof(virtual_scan_box_model_t));
	else
		models->box = (virtual_scan_box_model_t*) realloc(models->box, sizeof(virtual_scan_box_model_t) * n);

	virtual_scan_box_model_t *box = (models->box + n - 1);
	memset(box, '\0', sizeof(virtual_scan_box_model_t));
	return box;
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


virtual_scan_box_model_hypotheses_t *
virtual_scan_fit_box_models(virtual_scan_segment_classes_t *virtual_scan_segment_classes)
{
	virtual_scan_category_t categories[] = {{BUS, 2.5, 12}, {CAR, 1.9, 5.1}, {BIKE, 1.2, 2.2}};

	int num_segments = virtual_scan_segment_classes->num_segments;
	virtual_scan_box_model_hypotheses_t *box_model_hypotheses = virtual_scan_new_box_model_hypotheses(num_segments);
	box_model_hypotheses->timestamp = virtual_scan_segment_classes->timestamp;
	for (int i = 0; i < num_segments; i++)
	{
		int segment_class = virtual_scan_segment_classes->segment_features[i].segment_class;
		carmen_point_t first_point = virtual_scan_segment_classes->segment_features[i].first_point;
		carmen_point_t last_point = virtual_scan_segment_classes->segment_features[i].last_point;
		carmen_point_t farthest_point = virtual_scan_segment_classes->segment_features[i].farthest_point;
		//double width = virtual_scan_segment_classes->segment_features[i].width;
		//double length = virtual_scan_segment_classes->segment_features[i].length;
		carmen_point_t centroid = virtual_scan_segment_classes->segment_features[i].centroid;
		virtual_scan_box_models_t *box_models = virtual_scan_get_empty_box_models(box_model_hypotheses);
		if (segment_class == MASS_POINT)
		{
			virtual_scan_box_model_t *box = virtual_scan_append_box(box_models);
			box->c = PEDESTRIAN;
			box->x = centroid.x;
			box->y = centroid.y;
		}
		else if (segment_class == L_SHAPED) // L-shape segments segments will generate bus and car hypotheses
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
					double phi = atan2(width_point.y  - farthest_point.y, width_point.x  - farthest_point.x);

					carmen_position_t length_center_position = {
						farthest_point.x + 0.5 * categories[j].length * cos(rho),
						farthest_point.y + 0.5 * categories[j].length * sin(rho)
					};

					for (double direction = 0; direction <= M_PI; direction += M_PI)
					{
						virtual_scan_box_model_t *box = virtual_scan_append_box(box_models);
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
		else if (segment_class == I_SHAPED) // I-shape segments segments will generate bus, car and bike hypotheses
		{
			for (int j = 0; j < 3; j++)
			{
				double l = DIST2D(first_point, last_point);

				if (l <= categories[j].length)
				{
					double theta = atan2(last_point.y - first_point.y, last_point.x - first_point.x);

					carmen_position_t center_positions[] = {
						/* center from first points */ {
							first_point.x + 0.5 * categories[j].length * cos(theta),
							first_point.y + 0.5 * categories[j].length * sin(theta)
						},
						/* center from last points */ {
							last_point.x - 0.5 * categories[j].length * cos(theta),
							last_point.y - 0.5 * categories[j].length * sin(theta)
						},
						/* center of segment */ {
							0.5 * (first_point.x + last_point.x),
							0.5 * (first_point.y + last_point.y)
						}
					};

					// Compute two boxes for the center from first_point and two more from last_point
					for (int p = 0; p < 3; p++)
					{
						carmen_position_t length_center_position = center_positions[p];
						for (double delta_phi = -M_PI_2; delta_phi < M_PI; delta_phi += M_PI)
						{
							double phi = carmen_normalize_theta(theta + delta_phi);
							for (double direction = 0; direction <= M_PI; direction += M_PI)
							{
								virtual_scan_box_model_t *box = virtual_scan_append_box(box_models);
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

					carmen_position_t center_positions[] = {
						/* center from first points */ {
							first_point.x + 0.5 * categories[j].width * cos(theta),
							first_point.y + 0.5 * categories[j].width * sin(theta)
						},
						/* center from last points */ {
							last_point.x - 0.5 * categories[j].width * cos(theta),
							last_point.y - 0.5 * categories[j].width * sin(theta)
						},
						/* center of segment */ {
							0.5 * (first_point.x + last_point.x),
							0.5 * (first_point.y + last_point.y)
						}
					};

					// Compute two boxes for the center from first_point and two more from last_point
					for (int p = 0; p < 3; p++)
					{
						carmen_position_t width_center_position = center_positions[p];
						for (double delta_phi = -M_PI_2; delta_phi < M_PI; delta_phi += M_PI)
						{
							double phi = carmen_normalize_theta(theta + delta_phi);
							for (double direction = 0; direction <= M_PI; direction += M_PI)
							{
								virtual_scan_box_model_t *box = virtual_scan_append_box(box_models);
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
		if (!is_last_box_model_hypotheses_empty(box_model_hypotheses))
			box_model_hypotheses->last_box_model_hypotheses++;
	}

	return(box_model_hypotheses);
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
virtual_scan_free_extended(virtual_scan_extended_t *virtual_scan_extended)
{
	free(virtual_scan_extended->points);
	free(virtual_scan_extended);
}


void
virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
		free(virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box);
	free(virtual_scan_box_model_hypotheses->box_model_hypotheses);
	free(virtual_scan_box_model_hypotheses);
}


void
virtual_scan_free_message(virtual_scan_extended_t *virtual_scan_extended_filtered)
{
	free(virtual_scan_extended_filtered->points);
	free(virtual_scan_extended_filtered);
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
	free(virtual_scan_segment_classes->segment);
	free(virtual_scan_segment_classes->segment_features);
	free(virtual_scan_segment_classes);
}


void
virtual_scan_free_moving_objects(virtual_scan_moving_objects_t *moving_objects)
{

}


virtual_scan_segment_classes_t *
virtual_scan_extract_segments(virtual_scan_extended_t *virtual_scan_extended)
{
	virtual_scan_extended_t *virtual_scan_extended_filtered = filter_virtual_scan(virtual_scan_extended);
	virtual_scan_segments_t *virtual_scan_segments = segment_virtual_scan(virtual_scan_extended_filtered);
	virtual_scan_segment_classes_t *virtual_scan_segment_classes = classify_segments(virtual_scan_segments);
	virtual_scan_free_message(virtual_scan_extended_filtered);
	virtual_scan_free_segments(virtual_scan_segments);

	return virtual_scan_segment_classes;
}


void
virtual_scan_update_neighborhood_graph(virtual_scan_neighborhood_graph_t *neighborhood_graph, virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
}


int *
get_v_star(int *size, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	*size = 0;
	for (int i = 0; i < neighborhood_graph->size; i++)
	{
		if (!neighborhood_graph->box_model_hypothesis[i].selected)
			*size++;
	}

	if (*size != 0)
	{
		int *v_star = (int *) malloc(*size * sizeof(int));
		for (int i = 0, j = 0; i < neighborhood_graph->size; i++)
		{
			if (!neighborhood_graph->box_model_hypothesis[i].selected)
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


virtual_scan_track_t *
track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (neighborhood_graph != NULL)
	{
		int v_star_size;
		int *v_star = get_v_star(&v_star_size, neighborhood_graph);
		if (v_star != NULL)
		{
			int rand_v = rand() % v_star_size;
			virtual_scan_track_t *new_track = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
			new_track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) malloc(sizeof(virtual_scan_box_model_hypothesis_t));
			new_track->size = 1;
			new_track->box_model_hypothesis[0] = neighborhood_graph->box_model_hypothesis[v_star[rand_v]];
			neighborhood_graph->box_model_hypothesis[v_star[rand_v]].selected = true;

			free(v_star);
			return (new_track);
		}
		else
			return (NULL);
	}
	else
		return (NULL);
}


int
add_track(virtual_scan_track_t *tracks_n_1, virtual_scan_track_t new_track)
{
	if (tracks_n_1 == NULL)
	{
		tracks_n_1 = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
		tracks_n_1[0] = new_track;
	}
	else
	{
		tracks_n_1 = (virtual_scan_track_t *) realloc(tracks_n_1, (g_number_of_tracks + 1) * sizeof(virtual_scan_track_t));
		tracks_n_1[g_number_of_tracks] = new_track;
	}

	return (g_number_of_tracks);
}


void
track_extension(virtual_scan_track_t *track, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{

}


virtual_scan_track_t *
propose_tracks_according_to_q(virtual_scan_neighborhood_graph_t *neighborhood_graph, virtual_scan_track_t *tracks_n_1)
{
#define NUMBER_OF_TYPES_OF_MOVES	7

	int rand_track = rand() % g_number_of_tracks;
	int rand_var = rand() % NUMBER_OF_TYPES_OF_MOVES;
	switch (rand_var)
	{
		case 0:	// Birth
			virtual_scan_track_t *new_track = track_birth(neighborhood_graph);
			if (new_track != NULL)
			{
				int track_added = add_track(tracks_n_1, *new_track);
				track_extension(&(tracks_n_1[track_added]), neighborhood_graph);
				g_number_of_tracks += 1;
			}
			break;
		case 1:	// Extension
			track_extension(&(tracks_n_1[rand_track]), neighborhood_graph);
			break;
		case 2:	// Reduction
			track_reduction(tracks_n_1[rand_track]);
			break;
		case 3: //Death
			if (track_death(tracks_n_1[rand_track]))
				g_number_of_tracks -= 1;
			break;
		case 4:	// Split
			if (track_size(tracks_n_1[rand_track]) >= 4)
			{
				if (track_split(tracks_n_1[rand_track]))
					g_number_of_tracks += 1;
			}
			break;
		case 5:	// Merge
			int rand_track2 = rand() % g_number_of_tracks;
			if (track_merge(tracks_n_1, rand_track, rand_track2))
				g_number_of_tracks -= 1;
			break;
		case 6:	// Diffusion
			track_diffusion(tracks_n_1[rand_track]);
			break;
	}

	virtual_scan_track_t *tracks = copy_tracks(tracks_n_1);
	return (tracks);
}


double
probability_of_tracks_given_measurement(virtual_scan_track_t *tracks)
{
	return (0.5);
}


void
free_tracks(virtual_scan_track_t *tracks_victim)
{

}


virtual_scan_moving_objects_t *
get_moving_objects_from_track_set(virtual_scan_track_t *best_tracks)
{
	return (NULL);
}


double
A(virtual_scan_track_t *tracks_n, virtual_scan_track_t *tracks_prime)
{
	return (0.5);
}


virtual_scan_moving_objects_t *
virtual_scan_infer_moving_objects(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	virtual_scan_track_t *tracks_victim;
	virtual_scan_track_t *tracks_n = best_tracks;
	for (int n = 0; n < MCMC_MAX_ITERATIONS; n++)
	{
		virtual_scan_track_t *tracks_prime = propose_tracks_according_to_q(neighborhood_graph, tracks_n);
		tracks_victim = tracks_prime;
		double U = carmen_uniform_random(0.0, 1.0);
		if (U < A(tracks_n, tracks_prime))
		{
			tracks_n = tracks_prime;
			if (probability_of_tracks_given_measurement(tracks_n) > probability_of_tracks_given_measurement(best_tracks))
			{
				tracks_victim = best_tracks;
				best_tracks = tracks_n;
			}
		}
		free_tracks(tracks_victim);
	}

	virtual_scan_moving_objects_t *moving_objects = get_moving_objects_from_track_set(best_tracks);

	return (moving_objects);
}


void
virtual_scan_moving_objects_publish(virtual_scan_moving_objects_t *moving_objects)
{

}
