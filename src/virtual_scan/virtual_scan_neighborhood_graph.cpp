#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/moving_objects_messages.h>
#include <vector>
#include <carmen/matrix.h>
#include <carmen/kalman.h>
#include "virtual_scan.h"


extern virtual_scan_track_set_t *best_track_set;


void
create_hypothesis_vertex(int h, int i, int j, virtual_scan_neighborhood_graph_t *neighborhood_graph,
		virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	neighborhood_graph->box_model_hypothesis[h] = (virtual_scan_box_model_hypothesis_t *) calloc(1, sizeof(virtual_scan_box_model_hypothesis_t));
	neighborhood_graph->box_model_hypothesis[h]->hypothesis = virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box[j];
	neighborhood_graph->box_model_hypothesis[h]->hypothesis_points = virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box_points[j];
	neighborhood_graph->box_model_hypothesis[h]->frame_timestamp = virtual_scan_box_model_hypotheses->frame_timestamp;
	neighborhood_graph->box_model_hypothesis[h]->index = h;
	neighborhood_graph->box_model_hypothesis[h]->hypothesis_state.imm = NULL;
	neighborhood_graph->box_model_hypothesis[h]->already_examined = false;
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
				neighborhood_graph->box_model_hypothesis_edges[h]->edge[e] = k + previous_h;
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
		double delta_t = neighborhood_graph->box_model_hypothesis[child]->hypothesis_points.precise_timestamp - neighborhood_graph->box_model_hypothesis[candidate_parent]->hypothesis_points.precise_timestamp;
		int delta_zi = neighborhood_graph->box_model_hypothesis[child]->hypothesis_points.zi - neighborhood_graph->box_model_hypothesis[candidate_parent]->hypothesis_points.zi;
		if (delta_zi < 0)
			delta_zi += NUMBER_OF_FRAMES_T;

		if ((distance < delta_t * VMAX) && (delta_zi <= 2))
			return (true);
	}

	return (false);
}


void
create_hypothesis_parent_child_edges(int child, virtual_scan_neighborhood_graph_t *neighborhood_graph, double current_timestamp)
{
	int num_candidate_parent = 0; // @@@ Alberto: rever o uso do timestamp aqui. Nao da para usar o zi?
	while ((num_candidate_parent < neighborhood_graph->size) && (neighborhood_graph->box_model_hypothesis[num_candidate_parent]->frame_timestamp != current_timestamp))
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
	}
}


void
free_neighborhood_graph_vextexes(virtual_scan_neighborhood_graph_t *neighborhood_graph, int vextexes_to_remove)
{
	for (int i = 0; i < vextexes_to_remove; i++)
	{
		if (neighborhood_graph->box_model_hypothesis[i]->hypothesis_state.imm != NULL)
			delete neighborhood_graph->box_model_hypothesis[i]->hypothesis_state.imm;
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
	while ((vextexes_to_remove < neighborhood_graph->size) && (neighborhood_graph->box_model_hypothesis[vextexes_to_remove]->frame_timestamp == victim_timestamp))
		vextexes_to_remove++;

	free_neighborhood_graph_vextexes(neighborhood_graph, vextexes_to_remove);
	move_neighborhood_graph_vextexes(neighborhood_graph, vextexes_to_remove);
	rename_neighborhood_graph_vextexes_ids(neighborhood_graph, vextexes_to_remove);

	neighborhood_graph->size -= vextexes_to_remove;
	neighborhood_graph->box_model_hypothesis =
			(virtual_scan_box_model_hypothesis_t **) realloc(neighborhood_graph->box_model_hypothesis, neighborhood_graph->size * sizeof(virtual_scan_box_model_hypothesis_t *));
	neighborhood_graph->box_model_hypothesis_edges =
			(virtual_scan_box_model_hypothesis_edges_t **) realloc(neighborhood_graph->box_model_hypothesis_edges, neighborhood_graph->size * sizeof(virtual_scan_box_model_hypothesis_edges_t *));

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
			if (track->box_model_hypothesis[i].hypothesis_state.imm != NULL)
				delete track->box_model_hypothesis[i].hypothesis_state.imm;

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
update_best_track_set_due_to_vertexes_removed_from_graph(virtual_scan_track_set_t *best_track_set, int vextexes_removed_from_graph, virtual_scan_neighborhood_graph_t *neighborhood_graph)
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
	{
		memmove((void *) &(best_track_set->vertex_selected[0]), (void *) &(best_track_set->vertex_selected[vextexes_removed_from_graph]),
				neighborhood_graph->size * sizeof(bool));
		best_track_set->vertex_selected = (bool *) realloc(best_track_set->vertex_selected, neighborhood_graph->size * sizeof(bool));

		return (best_track_set);
	}
	else
	{
		free(best_track_set->vertex_selected);
		free(best_track_set);

		return (NULL);
	}
}


void
update_best_track_set_due_to_vertexes_added_to_graph(virtual_scan_track_set_t *best_track_set, int vextexes_added_to_graph, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (best_track_set == NULL)
		return;

	best_track_set->vertex_selected = (bool *) realloc(best_track_set->vertex_selected, (neighborhood_graph->size + vextexes_added_to_graph) * sizeof(bool));
	for (int i = neighborhood_graph->size; i < (neighborhood_graph->size + vextexes_added_to_graph); i++)
		best_track_set->vertex_selected[i] = false;
}


virtual_scan_neighborhood_graph_t *
first_neighborhood_graph_update(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	virtual_scan_neighborhood_graph_t *neighborhood_graph = (virtual_scan_neighborhood_graph_t *) malloc(sizeof(virtual_scan_neighborhood_graph_t));
	carmen_test_alloc(neighborhood_graph);
	neighborhood_graph->graph_id = 0;
	neighborhood_graph->last_frames_timetamps = (double *) malloc(NUMBER_OF_FRAMES_T * sizeof(double));

	int num_hypotheses = 0;
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
		for (int j = 0; j < virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes; j++)
			num_hypotheses++;

	neighborhood_graph->size = 0;
	neighborhood_graph->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t **) malloc(num_hypotheses * sizeof(virtual_scan_box_model_hypothesis_t *));
	neighborhood_graph->box_model_hypothesis_edges =
			(virtual_scan_box_model_hypothesis_edges_t **) malloc(num_hypotheses * sizeof(virtual_scan_box_model_hypothesis_edges_t *));

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
	neighborhood_graph->last_frames_timetamps[neighborhood_graph->number_of_frames_filled] = virtual_scan_box_model_hypotheses->frame_timestamp;
	neighborhood_graph->number_of_frames_filled = 1;

	return (neighborhood_graph);
}


virtual_scan_neighborhood_graph_t *
neighborhood_graph_update(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses, virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	neighborhood_graph->graph_id += 1;

	int num_hypotheses = 0;
	for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
		for (int j = 0; j < virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes; j++)
			num_hypotheses++;

	int first_new_vertex_to_add = neighborhood_graph->size;
	neighborhood_graph->box_model_hypothesis =
			(virtual_scan_box_model_hypothesis_t **) realloc(neighborhood_graph->box_model_hypothesis, (neighborhood_graph->size + num_hypotheses) * sizeof(virtual_scan_box_model_hypothesis_t *));
	neighborhood_graph->box_model_hypothesis_edges =
			(virtual_scan_box_model_hypothesis_edges_t **) realloc(neighborhood_graph->box_model_hypothesis_edges, (neighborhood_graph->size + num_hypotheses) * sizeof(virtual_scan_box_model_hypothesis_edges_t *));

	update_best_track_set_due_to_vertexes_added_to_graph(best_track_set, num_hypotheses, neighborhood_graph);

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
			create_hypothesis_parent_child_edges(h, neighborhood_graph, virtual_scan_box_model_hypotheses->frame_timestamp);
		}
	}
	neighborhood_graph->size += num_hypotheses;

	if (neighborhood_graph->number_of_frames_filled == NUMBER_OF_FRAMES_T)
	{
		double victim_timestamp = neighborhood_graph->last_frames_timetamps[0];
		for (int i = 0; i < NUMBER_OF_FRAMES_T - 1; i++)
			neighborhood_graph->last_frames_timetamps[i] = neighborhood_graph->last_frames_timetamps[i + 1];

		int vextexes_removed_from_graph = remove_graph_vertexes_of_victim_timestamp(victim_timestamp, neighborhood_graph);
		best_track_set = update_best_track_set_due_to_vertexes_removed_from_graph(best_track_set, vextexes_removed_from_graph, neighborhood_graph);

		neighborhood_graph->number_of_frames_filled -= 1;
	}

	neighborhood_graph->last_frames_timetamps[neighborhood_graph->number_of_frames_filled] = virtual_scan_box_model_hypotheses->frame_timestamp;
	neighborhood_graph->number_of_frames_filled += 1;

	return (neighborhood_graph);
}


void
print_neighborhood_graph(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	FILE *graph = fopen("graph.txt", "a");
	for (int i = 0; i < neighborhood_graph->size; i++)
	{
		fprintf(graph, "Graph Id %d, h %d %c -", neighborhood_graph->graph_id, i, neighborhood_graph->box_model_hypothesis[i]->hypothesis.c);
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

	print_neighborhood_graph(neighborhood_graph);

	return (neighborhood_graph);
}
