#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/moving_objects_messages.h>
#include <vector>
#include <carmen/matrix.h>
#include <carmen/kalman.h>
#include "virtual_scan.h"


using namespace std;

extern carmen_mapper_virtual_scan_message *g_virtual_scan_extended[NUMBER_OF_FRAMES_T];
extern int g_zi;

virtual_scan_track_set_t *best_track_set = NULL;
virtual_scan_track_set_t *global_track_set = NULL;

int g_track_id = -1;	// A atribuicao de track_id nao esta funcionando direito...


void
print_track_set(virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph, int num_proposal)
{
	FILE *track_sets = fopen("track_sets.txt", "a");

	if (track_set == NULL)
	{
		fprintf(track_sets, "\nGraph Id %d, Num Proposal %d - track_set == NULL\n", neighborhood_graph->graph_id, num_proposal);
		fclose(track_sets);

		return;
	}

	double prob = probability_of_track_set_given_measurements(track_set, false);

	fprintf(track_sets, "\nGraph Id %d Num Proposal %d - Num tracks = %d, prob = %lf\n", neighborhood_graph->graph_id, num_proposal, track_set->size, prob);
	for (int i = 0; i < neighborhood_graph->size; i++)
		fprintf(track_sets, "v[%d] %d, ", i, track_set->vertex_selected[i]);
	fprintf(track_sets, "\n");

	for (int i = 0; i < track_set->size; i++)
	{
		fprintf(track_sets, "track %d, id %d: ", i, track_set->tracks[i]->track_id);
		for (int j = 0; j < track_set->tracks[i]->size; j++)
			fprintf(track_sets, "h %d - %c, index %d, zi %d, v %lf;  ", j, track_set->tracks[i]->box_model_hypothesis[j].hypothesis.c,
					track_set->tracks[i]->box_model_hypothesis[j].index,
					track_set->tracks[i]->box_model_hypothesis[j].hypothesis_points.zi, track_set->tracks[i]->box_model_hypothesis[j].hypothesis_state.v);
		fprintf(track_sets, "\n");
	}

	fprintf(track_sets, "\n");
	fclose(track_sets);
}


void
print_track_set(virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	static int call_number = 0;

	FILE *track_sets = fopen("global_track_set.txt", "a");

	if (track_set == NULL)
		return;

	fprintf(track_sets, "\nGraph Id %d\n", neighborhood_graph->graph_id);

	fprintf(track_sets, "+++++++++++++++++++++ call number %d: \n", call_number++);

	for (int i = 0; i < track_set->size; i++)
	{
		fprintf(track_sets, "track %d: ", i);
		for (int j = 0; j < track_set->tracks[i]->size; j++)
			fprintf(track_sets, "h %d - %c, index %d, zi %d, v %lf;  ", j, track_set->tracks[i]->box_model_hypothesis[j].hypothesis.c,
					track_set->tracks[i]->box_model_hypothesis[j].index,
					track_set->tracks[i]->box_model_hypothesis[j].hypothesis_points.zi, track_set->tracks[i]->box_model_hypothesis[j].hypothesis_state.v);
		fprintf(track_sets, "\n");
	}

	fprintf(track_sets, "\n");
	fclose(track_sets);
}
 

bool
do_not_have_sibling_in_track_set(virtual_scan_box_model_hypothesis_edges_t *hypothesis_edges, virtual_scan_track_set_t *track_set)
{
	for (int k = 0; k < hypothesis_edges->size; k++)
	{
		if (hypothesis_edges->edge_type[k] == SIBLING_EDGE)
		{
			for (int i = 0; i < track_set->size; i++)
			{
				for (int j = 0; j < track_set->tracks[i]->size; j++)
				{
					int h = track_set->tracks[i]->box_model_hypothesis[j].index;
					if (hypothesis_edges->edge[k] == h)
						return (false);
				}
			}
		}
	}

	return (true);
}


int *
get_v_star(int &size, virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	size = 0;

	if (track_set != NULL)
	{
		for (int i = 0; i < neighborhood_graph->size; i++)
		{
			if (!track_set->vertex_selected[i])
				if (do_not_have_sibling_in_track_set(neighborhood_graph->box_model_hypothesis_edges[i], track_set))
					size++;
		}

		if (size != 0)
		{
			int *v_star = (int *) malloc(size * sizeof(int));
			for (int i = 0, j = 0; i < neighborhood_graph->size; i++)
			{
				if (!track_set->vertex_selected[i])
				{
					if (do_not_have_sibling_in_track_set(neighborhood_graph->box_model_hypothesis_edges[i], track_set))
					{
						v_star[j] = i;
						j++;
					}
				}
			}
			return (v_star);
		}
		else
			return (NULL);
	}
	else
	{
		size = neighborhood_graph->size;
		if (size != 0)
		{
			int *v_star = (int *) malloc(size * sizeof(int));
			for (int i = 0; i < size; i++)
				v_star[i] = i;

			return (v_star);
		}
		else
			return (NULL);
	}
}


void
free_track(virtual_scan_track_t *track)
{
	for (int i = 0; i < track->size; i++)
	{
		if (track->box_model_hypothesis[i].hypothesis_state.imm != NULL)
			delete track->box_model_hypothesis[i].hypothesis_state.imm;
	}
	free(track->box_model_hypothesis);
	free(track);
}


void
free_track_but_keep_imm(virtual_scan_track_t *track)
{
	free(track->box_model_hypothesis);
	free(track);
}


void
free_track_set(virtual_scan_track_set_t *track_set_victim)
{
	if (track_set_victim != NULL)
	{
		for (int i = 0; i < track_set_victim->size; i++)
			free_track(track_set_victim->tracks[i]);

		free(track_set_victim->tracks);
		free(track_set_victim->vertex_selected);
		free(track_set_victim);
	}
}


inline carmen_position_t
vect2d(carmen_position_t p1, carmen_position_t p2)
{
	carmen_position_t temp;

    temp.x = (p2.x - p1.x);
    temp.y = -(p2.y - p1.y);

    return (temp);
}


inline carmen_position_t
rotate_rectangle_vertice(carmen_position_t unrotated_vertice, carmen_position_t rectangle_center, double cos_theta, double sin_theta)
{
	carmen_position_t rotated_vertice;

	rotated_vertice.x = rectangle_center.x + cos_theta * (unrotated_vertice.x - rectangle_center.x) - sin_theta * (unrotated_vertice.y - rectangle_center.y);
	rotated_vertice.y = rectangle_center.y + sin_theta * (unrotated_vertice.x - rectangle_center.x) + cos_theta * (unrotated_vertice.y - rectangle_center.y);

	return (rotated_vertice);
}


int
point_inside_scaled_rectangle(carmen_point_t point, virtual_scan_box_model_t hypothesis, double scale)
{	// https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
	carmen_position_t A, B, C, D, m;

	double length_div_2 = (hypothesis.length * scale) / 2.0;
	double width_div_2 = (hypothesis.width * scale) / 2.0;
	carmen_position_t hypothesis_center = {hypothesis.x, hypothesis.y};

	double cos_theta = cos(hypothesis.theta);
	double sin_theta = sin(hypothesis.theta);

	A.x = hypothesis.x - length_div_2;
	A.y = hypothesis.y - width_div_2;
	A = rotate_rectangle_vertice(A, hypothesis_center, cos_theta, sin_theta);

	B.x = hypothesis.x + length_div_2;
	B.y = hypothesis.y - width_div_2;
	B = rotate_rectangle_vertice(B, hypothesis_center, cos_theta, sin_theta);

	C.x = hypothesis.x + length_div_2;
	C.y = hypothesis.y + width_div_2;
	C = rotate_rectangle_vertice(C, hypothesis_center, cos_theta, sin_theta);

	D.x = hypothesis.x - length_div_2;
	D.y = hypothesis.y + width_div_2;
	D = rotate_rectangle_vertice(D, hypothesis_center, cos_theta, sin_theta);

	m = {point.x, point.y};
	carmen_position_t AB = vect2d(A, B);
	double C1 = -(AB.y * A.x + AB.x * A.y);
	double D1 = (AB.y * m.x + AB.x * m.y) + C1;

	carmen_position_t BC = vect2d(B, C);
	double C2 = -(BC.y * B.x + BC.x * B.y);
	double D2 = (BC.y * m.x + BC.x * m.y) + C2;

	carmen_position_t CD = vect2d(C, D);
	double C3 = -(CD.y * C.x + CD.x * C.y);
	double D3 = (CD.y * m.x + CD.x * m.y) + C3;

	carmen_position_t DA = vect2d(D, A);
	double C4 = -(DA.y * A.x + DA.x * A.y);
	double D4 = (DA.y * m.x + DA.x * m.y) + C4;

	if ((D1 > 0.0) && (D4 > 0.0) && (D2 > 0.0) && (D3 > 0.0))
		return (1);
	else
		return (0);
}


void
get_points_inside_and_outside_scaled_rectangle(carmen_point_t *&points_inside_rectangle, carmen_point_t *&points_outside_rectangle,
		int &num_points_inside_rectangle, int &num_points_outside_rectangle,
		virtual_scan_box_model_hypothesis_t *box_model_hypothesis, double scale)
{
	int num_points = g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].num_points;
	carmen_point_t *point = g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].points;
	carmen_position_t sensor_pos = {g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].sensor_pos.x,
			g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].sensor_pos.y};

	points_inside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	points_outside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	num_points_inside_rectangle = num_points_outside_rectangle = 0;
	for (int i = 0; i < num_points; i++)
	{
		double velodyne_to_point_angle = atan2(sensor_pos.y - point[i].y, sensor_pos.x - point[i].x);
		bool acceptable_angle_diff = carmen_normalize_theta(velodyne_to_point_angle - point[i].theta) < (M_PI / 1.5);
		if (acceptable_angle_diff && point_inside_scaled_rectangle(point[i], box_model_hypothesis->hypothesis, scale))
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
	carmen_position_t sensor_pos = {g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].sensor_pos.x,
			g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].sensor_pos.y};

	points_inside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	points_outside_rectangle = (carmen_point_t *) malloc(sizeof(carmen_point_t) * num_points);
	num_points_inside_rectangle = num_points_outside_rectangle = 0;
	for (int i = 0; i < num_points; i++)
	{
		double velodyne_to_point_angle = atan2(sensor_pos.y - point[i].y, sensor_pos.x - point[i].x);
		bool acceptable_angle_diff = carmen_normalize_theta(velodyne_to_point_angle - point[i].theta) < (M_PI / 1.5);
		if (acceptable_angle_diff && point_inside_scaled_rectangle(point[i], box_model_hypothesis->hypothesis, scale))
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
	carmen_position_t hypothesis_center = {hypothesis.x, hypothesis.y};
	carmen_position_t v, w;
	double d, new_d;

	double cos_theta = cos(hypothesis.theta);
	double sin_theta = sin(hypothesis.theta);

	v.x = hypothesis.x - hypothesis.length / 2.0;
	v.y = hypothesis.y - hypothesis.width / 2.0;
	v = rotate_rectangle_vertice(v, hypothesis_center, cos_theta, sin_theta);

	w.x = hypothesis.x + hypothesis.length / 2.0;
	w.y = hypothesis.y - hypothesis.width / 2.0;
	w = rotate_rectangle_vertice(w, hypothesis_center, cos_theta, sin_theta);
	d = distance_from_point_to_line_segment_vw(v, w, zd_point);

	w.x = hypothesis.x - hypothesis.length / 2.0;
	w.y = hypothesis.y + hypothesis.width / 2.0;
	w = rotate_rectangle_vertice(w, hypothesis_center, cos_theta, sin_theta);
	new_d = distance_from_point_to_line_segment_vw(v, w, zd_point);
	if (new_d < d)
		d = new_d;

	v.x = hypothesis.x + hypothesis.length / 2.0;
	v.y = hypothesis.y + hypothesis.width / 2.0;
	v = rotate_rectangle_vertice(v, hypothesis_center, cos_theta, sin_theta);
	new_d = distance_from_point_to_line_segment_vw(v, w, zd_point);
	if (new_d < d)
		d = new_d;

	w.x = hypothesis.x + hypothesis.length / 2.0;
	w.y = hypothesis.y - hypothesis.width / 2.0;
	w = rotate_rectangle_vertice(w, hypothesis_center, cos_theta, sin_theta);
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
PM2(carmen_point_t *Zs_out, int Zs_out_size, virtual_scan_box_model_hypothesis_t *box_model_hypothesis)
{
	carmen_position_t sensor_pos = {g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].sensor_pos.x,
			g_virtual_scan_extended[box_model_hypothesis->hypothesis_points.zi]->virtual_scan_sensor[box_model_hypothesis->hypothesis_points.sensor].sensor_pos.y};

	carmen_rectangle_t rectangle = {box_model_hypothesis->hypothesis.x, box_model_hypothesis->hypothesis.y, box_model_hypothesis->hypothesis.theta,
			box_model_hypothesis->hypothesis.length, box_model_hypothesis->hypothesis.width};
	carmen_position_t nearest_intersection;

	double sum = 0.0;
	for (int i = 0; i < Zs_out_size; i++)
	{
		carmen_position_t point = {Zs_out[i].x, Zs_out[i].y};
		double velodyne_to_point_angle = atan2(sensor_pos.y - point.y, sensor_pos.x - point.x);
		bool acceptable_angle_diff = carmen_normalize_theta(velodyne_to_point_angle - Zs_out[i].theta) < (M_PI / 1.5);
		if (acceptable_angle_diff)
			sum += (double) carmen_line_to_point_crossed_rectangle(&nearest_intersection, sensor_pos, point, rectangle);
	}

	return (sum);
}


void
compute_hypothesis_posterior_probability_components(virtual_scan_box_model_hypothesis_t *box_model_hypothesis, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if ((box_model_hypothesis == NULL) || box_model_hypothesis->already_examined)
		return;

	carmen_point_t *Zs_out, *Zs_in, *Zd; int Zs_out_size, Zs_in_size, Zd_size;
	get_Zs_and_Zd_of_hypothesis(Zs_in, Zs_in_size, Zs_out, Zs_out_size, Zd, Zd_size, box_model_hypothesis);

	box_model_hypothesis->dn = PM1(Zd, Zd_size, box_model_hypothesis) / ((double) Zd_size + 1.0);
	box_model_hypothesis->c2 = PM2(Zs_out, Zs_out_size, box_model_hypothesis) / ((double) Zd_size + 1.0);
	box_model_hypothesis->c3 = 0.0;

	box_model_hypothesis->already_examined = true;
	neighborhood_graph->box_model_hypothesis[box_model_hypothesis->index]->already_examined = true;

	free(Zs_out); free(Zs_in); free(Zd);
}


virtual_scan_track_set_t *
add_track(virtual_scan_track_set_t *track_set_n_1, virtual_scan_track_t *new_track, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (track_set_n_1 == NULL)
	{
		track_set_n_1 = (virtual_scan_track_set_t *) malloc(sizeof(virtual_scan_track_set_t));
		track_set_n_1->tracks = (virtual_scan_track_t **) malloc(sizeof(virtual_scan_track_t *));
		track_set_n_1->tracks[0] = new_track;
		track_set_n_1->size = 1;
		track_set_n_1->vertex_selected = (bool *) calloc (neighborhood_graph->size, sizeof(bool));
	}
	else
	{
		track_set_n_1->tracks = (virtual_scan_track_t **) realloc(track_set_n_1->tracks, (track_set_n_1->size + 1) * sizeof(virtual_scan_track_t *));
		track_set_n_1->tracks[track_set_n_1->size] = new_track;
		track_set_n_1->size += 1;
	}

	return (track_set_n_1);
}


virtual_scan_track_set_t *
track_birth(virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (neighborhood_graph != NULL)
	{
		int v_star_size;
		int *v_star = get_v_star(v_star_size, track_set, neighborhood_graph);
		if (v_star != NULL)
		{
			virtual_scan_track_t *new_track = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
			new_track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) malloc(sizeof(virtual_scan_box_model_hypothesis_t));
			new_track->size = 1;
			new_track->track_id = ++g_track_id;

			int rand_v = carmen_int_random(v_star_size);
			new_track->box_model_hypothesis[0] = *(neighborhood_graph->box_model_hypothesis[v_star[rand_v]]);
			track_set = add_track(track_set, new_track, neighborhood_graph);
			track_set->vertex_selected[v_star[rand_v]] = true;

			free(v_star);

			compute_hypothesis_posterior_probability_components(&(new_track->box_model_hypothesis[0]), neighborhood_graph);

			return (track_set);
		}
		else
			return (track_set);
	}
	else
		return (NULL);
}


void
add_hypothesis_at_the_end(virtual_scan_track_t *track, virtual_scan_box_model_hypothesis_t *hypothesis)
{
	track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track->box_model_hypothesis,
			(track->size + 1) * sizeof(virtual_scan_box_model_hypothesis_t));

	track->box_model_hypothesis[track->size] = *hypothesis;
	track->size += 1;
}


void
add_hypothesis_at_the_end(virtual_scan_track_set_t *track_set, int track_id, virtual_scan_box_model_hypothesis_t *hypothesis)
{
	if (hypothesis == NULL)
		return;

	virtual_scan_track_t *track = track_set->tracks[track_id];
	add_hypothesis_at_the_end(track, hypothesis);

	track_set->vertex_selected[hypothesis->index] = true;
}


void
add_hypothesis_at_the_beginning(virtual_scan_track_set_t *track_set, int track_id, virtual_scan_box_model_hypothesis_t *hypothesis)
{
	if (hypothesis == NULL)
		return;

	virtual_scan_track_t *track = track_set->tracks[track_id];
	track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track->box_model_hypothesis,
			(track->size + 1) * sizeof(virtual_scan_box_model_hypothesis_t));
	for (int i = track->size; i > 0; i--)
		track->box_model_hypothesis[i] = track->box_model_hypothesis[i - 1];

	track->box_model_hypothesis[0] = *hypothesis;
	track->size += 1;

	track_set->vertex_selected[hypothesis->index] = true;
}


bool
hypothesis_in_v_star(int hypothesis, int *v_star, int v_star_size)
{
	for (int i = 0; i < v_star_size; i++)
		if (v_star[i] == hypothesis)
			return (true);

	return (false);
}


int *
get_neighbors_within_v_star(int neighbor_type, int &number_of_neighbors, virtual_scan_box_model_hypothesis_edges_t *hypothesis_neighbors, int *v_star, int v_star_size)
{
	if (hypothesis_neighbors == NULL)
		return (NULL);

	number_of_neighbors = 0;
	for (int i = 0; i < hypothesis_neighbors->size; i++)
	{
		if ((hypothesis_neighbors->edge_type[i] == neighbor_type) && hypothesis_in_v_star(hypothesis_neighbors->edge[i], v_star, v_star_size))
			number_of_neighbors++;
	}

	if (number_of_neighbors != 0)
	{
		int *neighbors = (int *) malloc(number_of_neighbors * sizeof(int));
		for (int i = 0, j = 0; i < hypothesis_neighbors->size; i++)
		{
			if ((hypothesis_neighbors->edge_type[i] == neighbor_type) && hypothesis_in_v_star(hypothesis_neighbors->edge[i], v_star, v_star_size))
			{
				neighbors[j] = hypothesis_neighbors->edge[i];
				j++;
			}
		}
		return (neighbors);
	}
	else
		return (NULL);
}


virtual_scan_box_model_hypothesis_t *
get_child_hypothesis_in_v_star_and_at_the_end_of_track(virtual_scan_track_t *track, virtual_scan_neighborhood_graph_t *neighborhood_graph,
		int *v_star, int v_star_size)
{
	virtual_scan_box_model_hypothesis_t end_of_track = track->box_model_hypothesis[track->size - 1];
	virtual_scan_box_model_hypothesis_edges_t *hypothesis_neighbors = neighborhood_graph->box_model_hypothesis_edges[end_of_track.index];
	int number_of_children;
	int *hypothesis_children = get_neighbors_within_v_star(CHILD_EDGE, number_of_children, hypothesis_neighbors, v_star, v_star_size);
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
get_parent_hypothesis_in_v_star_and_at_the_beginning_of_track(virtual_scan_track_t *track, virtual_scan_neighborhood_graph_t *neighborhood_graph,
		int *v_star, int v_star_size)
{
	virtual_scan_box_model_hypothesis_t beginning_of_track = track->box_model_hypothesis[0];
	virtual_scan_box_model_hypothesis_edges_t *hypothesis_neighbors = neighborhood_graph->box_model_hypothesis_edges[beginning_of_track.index];
	int number_of_parents;
	int *hypothesis_parents = get_neighbors_within_v_star(PARENT_EDGE, number_of_parents, hypothesis_neighbors, v_star, v_star_size);
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
track_extension(virtual_scan_track_set_t *track_set, int track_id, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (track_set == NULL)
		return;

	virtual_scan_track_t *track = track_set->tracks[track_id];
	virtual_scan_box_model_hypothesis_t *hypothesis;
	do
	{
		int v_star_size;
		int *v_star = get_v_star(v_star_size, track_set, neighborhood_graph);
		if (v_star != NULL)
		{
			if (carmen_int_random(2) == 0)
			{
				hypothesis = get_child_hypothesis_in_v_star_and_at_the_end_of_track(track, neighborhood_graph, v_star, v_star_size);
				compute_hypothesis_posterior_probability_components(hypothesis, neighborhood_graph);

				add_hypothesis_at_the_end(track_set, track_id, hypothesis);
			}
			else
			{
				hypothesis = get_parent_hypothesis_in_v_star_and_at_the_beginning_of_track(track, neighborhood_graph, v_star, v_star_size);
				compute_hypothesis_posterior_probability_components(hypothesis, neighborhood_graph);

				add_hypothesis_at_the_beginning(track_set, track_id, hypothesis);
			}
			free(v_star);
//			if (neighborhood_graph->graph_id == 7)
//			{
//				print_track_set(track_set, neighborhood_graph, 777);
//				printf("pare\n");
//			}
		}
		else
			break;
	} while (carmen_uniform_random(0.0, 1.0) > GAMMA);

	update_hypotheses_state(track);
}


void
track_reduction(virtual_scan_track_set_t *track_set, int track_id)
{
	virtual_scan_track_t *track = track_set->tracks[track_id];
	int previous_size = track->size;

	if (track->size > 2)
	{
		int r = carmen_int_random(track->size - 2) + 1; // r entre 1 e (track->size - 2). Note que, diferente do paper, nossa numeracao comecca de 0
		if (carmen_int_random(2) == 0)
		{	// forward reduction
			track->size = r + 1;

			for (int i = track->size; i < previous_size; i++)
			{
				if (track->box_model_hypothesis[i].hypothesis_state.imm != NULL)
				{
					delete track->box_model_hypothesis[i].hypothesis_state.imm;
					track->box_model_hypothesis[i].hypothesis_state.imm = NULL;
				}
				track_set->vertex_selected[track->box_model_hypothesis[i].index] = false;
			}
		}
		else
		{	// backward reduction
			track->size -= r;

			for (int i = 0; i < r; i++)
			{
				if (track->box_model_hypothesis[i].hypothesis_state.imm != NULL)
				{
					delete track->box_model_hypothesis[i].hypothesis_state.imm;
					track->box_model_hypothesis[i].hypothesis_state.imm = NULL;
				}
				track_set->vertex_selected[track->box_model_hypothesis[i].index] = false;
			}

			memmove((void *) &(track->box_model_hypothesis[0]), (void *) &(track->box_model_hypothesis[r]),
					track->size * sizeof(virtual_scan_box_model_hypothesis_t));
		}

		track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track->box_model_hypothesis,
				track->size * sizeof(virtual_scan_box_model_hypothesis_t));

		update_hypotheses_state(track);
	}
}


virtual_scan_track_set_t *
track_death(virtual_scan_track_set_t *track_set, int victim)
{
	for (int i = 0; i < track_set->tracks[victim]->size; i++)
		track_set->vertex_selected[track_set->tracks[victim]->box_model_hypothesis[i].index] = false;

	if (track_set->size > 1)
	{
		free_track(track_set->tracks[victim]);
		memmove((void *) &(track_set->tracks[victim]), &(track_set->tracks[victim + 1]), (track_set->size - (victim + 1)) * sizeof(virtual_scan_track_t *));
		// Tinha que fazer um realloc do track_set aqui...
		track_set->size -= 1;
	}
	else
	{
		free_track_set(track_set);
		track_set = NULL;
	}

	return (track_set);
}


virtual_scan_track_set_t *
track_removal(virtual_scan_track_set_t *track_set, int victim)
{	// As hipoteses de uma track removida nao retornam para o pool de hipoteses (vertex_selected nao eh alterado).
	if (track_set->size > 1)
	{
		free_track(track_set->tracks[victim]);
		memmove((void *) &(track_set->tracks[victim]), &(track_set->tracks[victim + 1]), (track_set->size - (victim + 1)) * sizeof(virtual_scan_track_t *));
		// Tinha que fazer um realloc do track_set aqui...
		track_set->size -= 1;
	}
	else
	{
		free_track_set(track_set);
		track_set = NULL;
	}

	return (track_set);
}


void
track_split(virtual_scan_track_set_t *track_set, int track_id)
{
	if (track_set->tracks[track_id]->size >= 4)
	{
		virtual_scan_track_t *old_track = track_set->tracks[track_id];

		int s = carmen_int_random(old_track->size - 3) + 1; // s entre 1 e (track->size - 3). Note que, diferente do paper, nossa numeracao comecca de 0
		int old_track_previous_size = old_track->size;
		old_track->size = s + 1;

		track_set->tracks = (virtual_scan_track_t **) realloc(track_set->tracks, (track_set->size + 1) * sizeof(virtual_scan_track_t *));
		track_set->tracks[track_set->size] = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
		virtual_scan_track_t *new_track = track_set->tracks[track_set->size];

		new_track->size = old_track_previous_size - old_track->size;
		new_track->track_id = ++g_track_id;
		new_track->box_model_hypothesis =
				(virtual_scan_box_model_hypothesis_t *) malloc(new_track->size * sizeof(virtual_scan_box_model_hypothesis_t));
		for (int j = 0; j < new_track->size; j++)
			new_track->box_model_hypothesis[j] = old_track->box_model_hypothesis[j + old_track->size];

		old_track->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(old_track->box_model_hypothesis,
				old_track->size * sizeof(virtual_scan_box_model_hypothesis_t));

		track_set->size += 1;

		update_hypotheses_state(new_track);
		update_hypotheses_state(old_track);
	}
}


bool
get_candidate_pair_of_tracks_to_merge(int &idx_track1, int &idx_track2, virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	int *track1_candidates = (int *) malloc(sizeof(int));
	int *track2_candidates = (int *) malloc(sizeof(int));

	int num_candidates = 0;
	for (int track_i = 0; track_i < track_set->size; track_i++)
	{
		for (int track_j = 0; track_j < track_set->size; track_j++)
		{
			if (track_i != track_j)
			{
				int h1_vertex_index = track_set->tracks[track_i]->box_model_hypothesis[track_set->tracks[track_i]->size - 1].index;
				int h2_vertex_index = track_set->tracks[track_j]->box_model_hypothesis[0].index;
				virtual_scan_box_model_hypothesis_edges_t *h2_edges = neighborhood_graph->box_model_hypothesis_edges[h2_vertex_index];
				for (int i = 0; i < h2_edges->size; i++)
				{
					if ((h2_edges->edge_type[i] == PARENT_EDGE) && (h2_edges->edge[i] == h1_vertex_index))
					{
						track1_candidates[num_candidates] = track_i;
						track2_candidates[num_candidates] = track_j;
						num_candidates++;
						track1_candidates = (int *) realloc(track1_candidates, (num_candidates + 1) * sizeof(int));
						track2_candidates = (int *) realloc(track2_candidates, (num_candidates + 1) * sizeof(int));
					}
				}
			}
		}
	}

	if (num_candidates != 0)
	{
		int rand_candidate = carmen_int_random(num_candidates);
		idx_track1 = track1_candidates[rand_candidate];
		idx_track2 = track2_candidates[rand_candidate];
	}

	free(track1_candidates);
	free(track2_candidates);

	if (num_candidates != 0)
		return (true);
	else
		return (false);
}


void
track_merge(virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	int idx_track1, idx_track2;
	if (get_candidate_pair_of_tracks_to_merge(idx_track1, idx_track2, track_set, neighborhood_graph))
	{
		virtual_scan_track_t *track1 = track_set->tracks[idx_track1];
		virtual_scan_track_t *track2 = track_set->tracks[idx_track2];

		track1->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track1->box_model_hypothesis,
				(track1->size + track2->size) * sizeof(virtual_scan_box_model_hypothesis_t));

		for (int j = 0; j < track2->size; j++)
		{
			track1->box_model_hypothesis[j + track1->size] = track2->box_model_hypothesis[j];
			track1->box_model_hypothesis[j + track1->size].hypothesis_state.imm = NULL; // O estado do imm da track2 nao pode ser aproveitado na track1
		}

		// remocao de track2 do track_set
		free_track(track2);
		memmove((void *) &(track_set->tracks[idx_track2]), (void *) &(track_set->tracks[idx_track2 + 1]),
				(track_set->size - (idx_track2 + 1)) * sizeof(virtual_scan_track_t *));
		// Tinha que fazer um realloc do track_set aqui...
		track_set->size -= 1;

		update_hypotheses_state(track1);
	}
}


bool
get_candidate_pair_of_tracks_to_switch(int &idx_track1, int &idx_track2, int &p_found, int &q_found, virtual_scan_track_set_t *track_set,
		virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	int *track1_candidates = (int *) malloc(sizeof(int));
	int *track2_candidates = (int *) malloc(sizeof(int));
	int *p_candidates = (int *) malloc(sizeof(int));
	int *q_candidates = (int *) malloc(sizeof(int));

	int num_candidates = 0;
	for (int track_1 = 0; track_1 < track_set->size; track_1++)
	{
		for (int track_2 = 0; track_2 < track_set->size; track_2++)
		{
			if (track_1 != track_2)
			{
				for (int p = 0; p < track_set->tracks[track_1]->size; p++)
				{
					for (int q = 0; q < track_set->tracks[track_2]->size; q++)
					{
						int track_1_vertex_index = track_set->tracks[track_1]->box_model_hypothesis[p].index;
						int track_2_vertex_index = track_set->tracks[track_2]->box_model_hypothesis[q].index;
						virtual_scan_box_model_hypothesis_edges_t *track_1_edges = neighborhood_graph->box_model_hypothesis_edges[track_1_vertex_index];
						virtual_scan_box_model_hypothesis_edges_t *track_2_edges = neighborhood_graph->box_model_hypothesis_edges[track_2_vertex_index];
						for (int i = 0; i < track_2_edges->size; i++)
						{
							for (int j = 0; j < track_1_edges->size; j++)
							{
								if (((track_2_edges->edge_type[i] == PARENT_EDGE) && (track_2_edges->edge[i] == track_1_vertex_index)) &&
									((track_1_edges->edge_type[j] == PARENT_EDGE) && (track_1_edges->edge[j] == track_2_vertex_index)))
								{
									track1_candidates[num_candidates] = track_1;
									track2_candidates[num_candidates] = track_2;
									p_candidates[num_candidates] = p;
									q_candidates[num_candidates] = q;

									num_candidates++;
									track1_candidates = (int *) realloc(track1_candidates, (num_candidates + 1) * sizeof(int));
									track2_candidates = (int *) realloc(track2_candidates, (num_candidates + 1) * sizeof(int));
									p_candidates = (int *) realloc(p_candidates, (num_candidates + 1) * sizeof(int));
									q_candidates = (int *) realloc(q_candidates, (num_candidates + 1) * sizeof(int));
								}
							}
						}
					}
				}
			}
		}
	}

	if (num_candidates != 0)
	{
		int rand_candidate = carmen_int_random(num_candidates);
		idx_track1 = track1_candidates[rand_candidate];
		idx_track2 = track2_candidates[rand_candidate];
		p_found = p_candidates[rand_candidate];
		q_found = q_candidates[rand_candidate];
	}

	free(track1_candidates);
	free(track2_candidates);
	free(p_candidates);
	free(q_candidates);

	if (num_candidates != 0)
		return (true);
	else
		return (false);
}


void
track_switch(virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	int idx_track1, idx_track2, p, q;
	if (get_candidate_pair_of_tracks_to_switch(idx_track1, idx_track2, p, q, track_set, neighborhood_graph))
	{
		virtual_scan_track_t *track1 = track_set->tracks[idx_track1];
		virtual_scan_track_t *track2 = track_set->tracks[idx_track2];

		virtual_scan_box_model_hypothesis_t *box_model_hypothesis_copy =
				(virtual_scan_box_model_hypothesis_t *) malloc((track1->size - p - 1) * sizeof(virtual_scan_box_model_hypothesis_t));

		for (int i = p + 1; i < track1->size; i++)
			box_model_hypothesis_copy[i - (p + 1)] = track1->box_model_hypothesis[i];

		track1->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track1->box_model_hypothesis,
							(p + 1 + (track2->size - q - 1)) * sizeof(virtual_scan_box_model_hypothesis_t));
		for (int i = p + 1; (i - (p + 1)) < (track2->size - q - 1); i++)
			track1->box_model_hypothesis[i] = track2->box_model_hypothesis[i - (p + 1) + q + 1];

		track2->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) realloc(track2->box_model_hypothesis,
							(q + 1 + (track1->size - p - 1)) * sizeof(virtual_scan_box_model_hypothesis_t));
		for (int i = q + 1; (i - (q + 1)) < (track1->size - p - 1); i++)
			track2->box_model_hypothesis[i] = box_model_hypothesis_copy[i - (q + 1)];

		free(box_model_hypothesis_copy);

		update_hypotheses_state(track1);
		update_hypotheses_state(track2);
	}
}

//void
//track_diffusion(virtual_scan_track_t *track, int track_id)
//{
//
//}


void
add_track_to_track_set(virtual_scan_track_set_t *track_set, virtual_scan_track_t *track)
{
	int i = track_set->size;
	track_set->tracks = (virtual_scan_track_t **) realloc(track_set->tracks, (i + 1) * sizeof(virtual_scan_track_t *));
	track_set->tracks[i] = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
	track_set->tracks[i]->size = track->size;
	track_set->tracks[i]->track_id = track->track_id;
	track_set->tracks[i]->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) malloc(track->size * sizeof(virtual_scan_box_model_hypothesis_t));

	for (int j = 0; j < track->size; j++)
	{
		track_set->tracks[i]->box_model_hypothesis[j] = track->box_model_hypothesis[j];
		// deep copy of imm
		if (track->box_model_hypothesis[j].hypothesis_state.imm != NULL)
			track_set->tracks[i]->box_model_hypothesis[j].hypothesis_state.imm = new imm_state_t(*(track->box_model_hypothesis[j].hypothesis_state.imm));
	}

	track_set->size = i + 1;
}


virtual_scan_track_set_t *
copy_track_set(virtual_scan_track_set_t *track_set_n_1, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (track_set_n_1 != NULL)
	{
		virtual_scan_track_set_t *track_set = (virtual_scan_track_set_t *) malloc(sizeof(virtual_scan_track_set_t));

		track_set->tracks = NULL;
		track_set->size = 0;
		for (int i = 0; i < track_set_n_1->size; i++)
			add_track_to_track_set(track_set, track_set_n_1->tracks[i]);

		track_set->vertex_selected = (bool *) malloc(neighborhood_graph->size * sizeof(bool));
		memcpy((void *) track_set->vertex_selected, (void *) track_set_n_1->vertex_selected, neighborhood_graph->size * sizeof(bool));

		return (track_set);
	}
	else
		return (NULL);
}


virtual_scan_track_set_t *
copy_track_set(virtual_scan_track_set_t *track_set_n_1)
{
	if (track_set_n_1 != NULL)
	{
		virtual_scan_track_set_t *track_set = (virtual_scan_track_set_t *) malloc(sizeof(virtual_scan_track_set_t));

		track_set->tracks = NULL;
		track_set->size = 0;
		for (int i = 0; i < track_set_n_1->size; i++)
			add_track_to_track_set(track_set, track_set_n_1->tracks[i]);

		return (track_set);
	}
	else
		return (NULL);
}


bool
stop_condition(virtual_scan_track_set_t *track_set, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (track_set == NULL)
		return (false);

	int num_hypothesis = 0;
	for (int i = 0; i < track_set->size; i++)
		num_hypothesis += track_set->tracks[i]->size;

	int num_taken_vertexes = 0;
	for (int i = 0; i < neighborhood_graph->size; i++)
		num_taken_vertexes += (track_set->vertex_selected[i] == 0)? 0: 1;

	if (num_hypothesis != num_taken_vertexes)
		return (true);
	else
		return (false);
}


double
average_track_velocity(virtual_scan_track_t *track)
{
	if (track->size < 3)
		return (-1.0); // velocidade invalida - velocidades sao sempre positivas e orientadas por um angulo que nao eh considerado aqui

	double average_v = 0.0;
	for (int i = 0; i < track->size; i++)
		average_v += track->box_model_hypothesis[i].hypothesis_state.v;

	average_v = fabs(average_v / (double) track->size);

	return (average_v);
}


bool
too_slow(virtual_scan_track_t *track, double v)
{
	if (v < 0.0)
		return (false); // velocidade invalida - velocidades sao sempre positivas e orientadas por um angulo que nao eh considerado aqui

	double min_v = GET_MIN_V_PER_OBJECT_CLASS(track->box_model_hypothesis[0].hypothesis.c);
	if (v < min_v)
		return (true);

	return (false);
}


virtual_scan_track_set_t *
filter_track_set(virtual_scan_track_set_t *track_set)
{
	if (track_set == NULL)
		return (NULL);

	int i = 0;
	while (i < track_set->size)
	{
		int previous_track_set_size = track_set->size;

		if (too_slow(track_set->tracks[i], average_track_velocity(track_set->tracks[i])))	// Tracks de objetos parados
			track_set = track_removal(track_set, i);	// As hipoteses de uma track removida nao retornal para o pool de hipoteses

		if (track_set == NULL)
			return (NULL);

		if (previous_track_set_size == track_set->size)
			i++;
	}

	return (track_set);
}


virtual_scan_track_set_t *
propose_track_set_according_to_q(virtual_scan_neighborhood_graph_t *neighborhood_graph, virtual_scan_track_set_t *track_set_n_1)
{
#define NUMBER_OF_TYPES_OF_MOVES	7

//	static int num_proposal = 0;

	int rand_track;

	int rand_move = carmen_int_random(NUMBER_OF_TYPES_OF_MOVES);
	if ((track_set_n_1 != NULL) && (track_set_n_1->size != 0))
		rand_track = carmen_int_random(track_set_n_1->size);
	else
		rand_track = -1;

	virtual_scan_track_set_t *track_set = copy_track_set(track_set_n_1, neighborhood_graph);

//	if (neighborhood_graph->graph_id == 7)
//	{
//		char *move[] = {(char *) "Birth", (char *) "Extension", (char *) "reduction", (char *) "Death", (char *) "Split",
//				(char *) "Merge", (char *) "Switch", (char *) "Diffusion"};
//		print_track_set(track_set, neighborhood_graph, 777);
//		FILE *track_sets = fopen("track_sets.txt", "a");
//		fprintf(track_sets, "\nrand_move %s, rand_track %d\n", move[rand_move], rand_track);
//		fclose(track_sets);
//	}

	switch (rand_move)
	{
		case 0:	// Birth
			track_set = track_birth(track_set, neighborhood_graph);
			if (track_set != NULL)
				track_extension(track_set, track_set->size - 1, neighborhood_graph);
			break;
		case 1:	// Extension
			if (rand_track != -1)
				track_extension(track_set, rand_track, neighborhood_graph);
			break;
		case 2:	// Reduction
			if (rand_track != -1)
				track_reduction(track_set, rand_track);
			break;
		case 3: // Death
			if (rand_track != -1)
				track_set = track_death(track_set, rand_track);
			break;
		case 4:	// Split
			if (rand_track != -1)
				track_split(track_set, rand_track);
			break;
		case 5:	// Merge
			if (rand_track != -1)
				track_merge(track_set, neighborhood_graph);
			break;
		case 6:	// Switch
			if (rand_track != -1)
				track_switch(track_set, neighborhood_graph);
			break;
//		case 7:	// Diffusion
//			if (rand_track != -1)
//				track_diffusion(track_set, rand_track);
//			break;
	}

//	if (neighborhood_graph->graph_id == 7)
//		print_track_set(track_set, neighborhood_graph, 888);

	return (track_set);
}


void
virtual_scan_free_moving_objects(carmen_moving_objects_point_clouds_message *moving_objects)
{
	if (moving_objects != NULL)
	{
		free(moving_objects->point_clouds);
		free(moving_objects);
	}
}


double
A(virtual_scan_track_set_t *track_set_n, virtual_scan_track_set_t *track_set_prime)
{
	double pi_w_prime = probability_of_track_set_given_measurements(track_set_prime);
	double pi_w_n_1 = probability_of_track_set_given_measurements(track_set_n);
	double q_w_w_prime = 1.0 / NUMBER_OF_TYPES_OF_MOVES;
	double q_w_prime_w = 1.0 / NUMBER_OF_TYPES_OF_MOVES;

	if (pi_w_prime == 0.0)
		return (0.0);
	else if (pi_w_n_1 == 0.0)
		return (1.0);

	double A_w_w_prime = carmen_fmin(1.0, (pi_w_prime * q_w_w_prime) / (pi_w_n_1 * q_w_prime_w));

	return (A_w_w_prime);
}


int
largest_track_size(virtual_scan_track_set_t *track_set)
{
	if (track_set == NULL)
		return (0);

	int max_track_size = 0;
	for (int i = 0; i < track_set->size; i++)
		if (track_set->tracks[i]->size > max_track_size)
			max_track_size = track_set->tracks[i]->size;

	return (max_track_size);
}


virtual_scan_track_set_t *
filter_best_track_set(virtual_scan_track_set_t *best_track_set)
{
	if (best_track_set == NULL)
		return (NULL);

	int i = 0;
	while (i < best_track_set->size)
	{
		int previous_track_set_size = best_track_set->size;

		virtual_scan_box_model_hypothesis_t last_hypothesis = best_track_set->tracks[i]->box_model_hypothesis[best_track_set->tracks[i]->size - 1];
		int delta_frames = g_zi - last_hypothesis.hypothesis_points.zi;
		if (delta_frames < 0)
			delta_frames += NUMBER_OF_FRAMES_T;
		if ((best_track_set->tracks[i]->size < 3) || (delta_frames >= 3) || (fabs(last_hypothesis.hypothesis_state.v) < 0.1))
			best_track_set = track_death(best_track_set, i);

		if (best_track_set == NULL)
			return (NULL);

		if (previous_track_set_size == best_track_set->size)
			i++;
	}

	return (best_track_set);
}


int
find_nearest_hypothesis_in_track(virtual_scan_track_t *track, virtual_scan_box_model_t hypothesis)
{
	int nearest_hypothesis = 0;
	double min_distance = DIST2D(track->box_model_hypothesis[nearest_hypothesis].hypothesis, hypothesis);
	for (int i = 1; i < track->size; i++)
	{
		double distance = DIST2D(track->box_model_hypothesis[i].hypothesis, hypothesis);
		if (distance < min_distance)
		{
			min_distance = distance;
			nearest_hypothesis = i;
		}
	}
	return (nearest_hypothesis);
}


bool
has_superposition(virtual_scan_track_t *global_track, virtual_scan_track_t *track)
{
	int hypotheses_considered = 0;
	double sum = 0.0;
	for (int i = 0; i < global_track->size; i++)
	{
		int nearest_hypothesis = find_nearest_hypothesis_in_track(track, global_track->box_model_hypothesis[i].hypothesis);
		double distance = DIST2D(track->box_model_hypothesis[nearest_hypothesis].hypothesis, global_track->box_model_hypothesis[i].hypothesis);
		if (distance < 0.5)
		{
			sum += distance;
			hypotheses_considered++;
		}
	}

	double average_distance = sum / (double) hypotheses_considered;

	if ((hypotheses_considered > 1) && (average_distance < 0.5))
		return (true);
	else
		return (false);
}


void
merge_tracks(virtual_scan_track_t *global_track, virtual_scan_track_t *track)
{
	virtual_scan_box_model_t last_hypothesis_of_global_track = global_track->box_model_hypothesis[global_track->size - 1].hypothesis;
	int nearest_hypothesis_in_track = find_nearest_hypothesis_in_track(track, last_hypothesis_of_global_track);
	for (int i = nearest_hypothesis_in_track; i < track->size; i++)
		add_hypothesis_at_the_end(global_track, &(track->box_model_hypothesis[i]));
}


void
merge_track_with_global_track_set(virtual_scan_track_set_t *global_track_set, virtual_scan_track_t *track)
{
	bool track_has_being_merged = false;
	for (int i = global_track_set->size - 1; i >= 0; i--)
	{
		virtual_scan_track_t *global_track = global_track_set->tracks[i];
//		if ((track->box_model_hypothesis[0].frame_timestamp - global_track->box_model_hypothesis[0].frame_timestamp) > 2.0 * NUMBER_OF_FRAMES_T * (1.0 / 20.0))
//			break;
		if (has_superposition(global_track, track))
		{
			merge_tracks(global_track, track);
			track_has_being_merged = true;
			break;
		}
	}

	if (!track_has_being_merged)
	{
		add_track_to_track_set(global_track_set, track);
		if (global_track_set->size > 100)
			track_removal(global_track_set, 0);
	}
}


void
update_global_track_set(virtual_scan_track_set_t *best_track_set)
{
	if (best_track_set == NULL)
		return;

	if (global_track_set == NULL)
	{
		global_track_set = copy_track_set(best_track_set);
		return;
	}

	for (int i = 0; i < best_track_set->size; i++)
		merge_track_with_global_track_set(global_track_set, best_track_set->tracks[i]);
}


virtual_scan_track_set_t *
virtual_scan_infer_moving_objects(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	virtual_scan_track_set_t *track_set_n = copy_track_set(best_track_set, neighborhood_graph);
	for (int n = 0; n < MCMC_MAX_ITERATIONS; n++)
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
				best_track_set = copy_track_set(track_set_n, neighborhood_graph);
			}
		}
		else if ((track_set_prime != NULL) && (track_set_n != NULL) && (track_set_prime->size > track_set_n->size))
			g_track_id--;	// Nesta iteracao nasceu uma track que nao foi aproveitada

		free_track_set(track_set_victim);
	}
	free_track_set(track_set_n);

	best_track_set = filter_best_track_set(best_track_set);

	best_track_set = filter_track_set(best_track_set);

	update_global_track_set(best_track_set);

	double best_track_prob = probability_of_track_set_given_measurements(best_track_set, true);
	printf("num_tracks %d, largest track %d, prob %lf\n", (best_track_set)? best_track_set->size: 0, largest_track_size(best_track_set), best_track_prob);
	print_track_set(best_track_set, neighborhood_graph, 0);

	print_track_set(global_track_set, neighborhood_graph);

	return (best_track_set);
}
