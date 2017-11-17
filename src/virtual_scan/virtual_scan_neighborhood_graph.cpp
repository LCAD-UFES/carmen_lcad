#include <cassert>

#include <carmen/carmen.h>
#include "virtual_scan.h"
#include "virtual_scan_neighborhood_graph.h"

#define MAX_SPEED 60
#define MAX_NUM_DISCONNECTED_SUB_GRAPHS 10

void
virtual_scan_initiate_elements(virtual_scan_elements_t *elements)
{
	elements->num_pointers = 0;
	elements->pointers = NULL;
}


void
virtual_scan_add_elements(virtual_scan_elements_t *elements, void *element)
{
	if (elements->num_pointers == 0)
	{
		elements->pointers = (void **) malloc(sizeof(void *) * 1);
		elements->pointers[0] = element;
		elements->num_pointers++;
		return;
	}
	elements->pointers = (void **) realloc(elements->pointers, sizeof(void *) * (elements->num_pointers + 1));
	elements->pointers[elements->num_pointers] = element;
	elements->num_pointers++;
}


void *
virtual_scan_read_elements(virtual_scan_elements_t *elements, int index_element)
{
	return elements->pointers[index_element];
}


virtual_scan_neighborhood_graph_t *
virtual_scan_alloc_neighborhood_graph ()
{
	virtual_scan_neighborhood_graph_t *neigborhood_graph = (virtual_scan_neighborhood_graph_t *)
			calloc(1, sizeof(virtual_scan_neighborhood_graph_t));
	return neigborhood_graph;
}


void
virtual_scan_compute_graph_nodes (virtual_scan_complete_sub_graph_t *complete_sub_graph, virtual_scan_box_models_t *hypothesys, double timestamp)
{
	int num_boxes = hypothesys->num_boxes;
	virtual_scan_graph_node_t *graph_nodes = (virtual_scan_graph_node_t *)
					malloc(sizeof(virtual_scan_graph_node_t) * num_boxes);
	complete_sub_graph->nodes = graph_nodes;
	complete_sub_graph->num_nodes = num_boxes;
	for (int j = 0, n = num_boxes; j < n; j++)
	{
		graph_nodes[j].box_model = hypothesys->box[j];
		graph_nodes[j].timestamp = timestamp;
		virtual_scan_initiate_elements(&graph_nodes[j].siblings);
		virtual_scan_initiate_elements(&graph_nodes[j].parents);
		virtual_scan_initiate_elements(&graph_nodes[j].children);
		for (int k = 0, l = 0; k < (num_boxes - 1); k++, l++)
		{
			if (j == l)
				l++;
			virtual_scan_add_elements(&graph_nodes[j].siblings, graph_nodes + l);
		}
	}
}


virtual_scan_disconnected_sub_graph_t *
virtual_scan_compute_disconnected_sub_graph(virtual_scan_box_model_hypotheses_t *box_model_hypotheses,
		virtual_scan_extended_t *virtual_scan_extended)
{
	virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph = (virtual_scan_disconnected_sub_graph_t *)
			calloc(1, sizeof(virtual_scan_disconnected_sub_graph_t));
	virtual_scan_box_models_t *hypotheses = box_model_hypotheses->box_model_hypotheses;
	int num_box_model_hypotheses = box_model_hypotheses->last_box_model_hypotheses;
	virtual_scan_complete_sub_graph_t *complete_sub_graphs = (virtual_scan_complete_sub_graph_t *)
			malloc(sizeof(virtual_scan_complete_sub_graph_t) * num_box_model_hypotheses);
	disconnected_sub_graph->sub_graphs = complete_sub_graphs;
	disconnected_sub_graph->num_sub_graphs = num_box_model_hypotheses;
	disconnected_sub_graph->timestamp = box_model_hypotheses->timestamp;
	disconnected_sub_graph->virtual_scan_extended = virtual_scan_extended;

	for (int i = 0, m = num_box_model_hypotheses; i < m; i++)
	{
		assert(hypotheses[i].num_boxes > 0);
		virtual_scan_compute_graph_nodes(complete_sub_graphs+i, hypotheses+i, disconnected_sub_graph->timestamp);
	}
	return disconnected_sub_graph;
}


virtual_scan_graph_node_t *
next(virtual_scan_disconnected_sub_graph_iterator_t *iterator)
{
	int i = iterator->sub_graph_index;
	int j = iterator->node_index;

	virtual_scan_disconnected_sub_graph_t *disconnected_subgraph = iterator->disconnected_sub_graph;
	if (i >= disconnected_subgraph->num_sub_graphs)
		return NULL;
	virtual_scan_complete_sub_graph_t *complete_sub_graph = disconnected_subgraph->sub_graphs + i;
	virtual_scan_graph_node_t *graph_node = complete_sub_graph->nodes + j;

	iterator->node_index++;
	if (iterator->node_index >= complete_sub_graph->num_nodes)
	{
		iterator->sub_graph_index++;
		iterator->node_index = 0;
	}

	return(graph_node);
}


int
is_parent_child(virtual_scan_graph_node_t *graph_node_parent, virtual_scan_graph_node_t *graph_node_child)
{
	if (graph_node_parent->box_model.c != graph_node_child->box_model.c)
		return 0;
	double delta_distance = (graph_node_child->timestamp - graph_node_parent->timestamp) * MAX_SPEED;
	if (DIST2D(graph_node_parent->box_model,  graph_node_child->box_model) >= delta_distance)
		return 0;
	return 1;
}


void
connect_parent_child(virtual_scan_graph_node_t *graph_node_parent, virtual_scan_graph_node_t *graph_node_child)
{
	virtual_scan_add_elements(&graph_node_parent->children, graph_node_child);
	virtual_scan_add_elements(&graph_node_child->parents, graph_node_parent);
}


void
connect_disconnected_subgraphs(
		virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph_parent,
		virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph_child)
{
	virtual_scan_disconnected_sub_graph_iterator_t iterator_parent = {disconnected_sub_graph_parent, 0, 0};
	for ( ; ; )
	{
		virtual_scan_disconnected_sub_graph_iterator_t iterator_child = {disconnected_sub_graph_child, 0, 0};
		virtual_scan_graph_node_t *graph_node_parent = next(&iterator_parent);
		if (graph_node_parent == NULL)
			break;
		for ( ; ; )
		{
			virtual_scan_graph_node_t *graph_node_child = next(&iterator_child);
			if (graph_node_child == NULL)
				break;
			if (is_parent_child(graph_node_parent, graph_node_child))
				connect_parent_child(graph_node_parent, graph_node_child);
		}
	}
}


void
clear_node(virtual_scan_graph_node_t *node)
{
	free(node->siblings.pointers);
	free(node->parents.pointers);
	free(node->children.pointers);
}


void
clear_complete_subgraph(virtual_scan_complete_sub_graph_t *sub_graph)
{
	for (int i = 0; i < sub_graph->num_nodes; i++)
	{
		clear_node(&sub_graph->nodes[i]);
	}
	free(sub_graph->nodes);
}


void
free_disconnected_subgraph(virtual_scan_disconnected_sub_graph_t *sub_graph)
{
	if (sub_graph->previous != NULL)
		sub_graph->previous->next = sub_graph->next;
	if (sub_graph->next != NULL)
		sub_graph->next->previous = sub_graph->previous;
	for (int i = 0; i < sub_graph->num_sub_graphs; i++)
	{
		clear_complete_subgraph(&sub_graph->sub_graphs[i]);
	}
	if ((sub_graph->previous != NULL) && (sub_graph->next != NULL))
		connect_disconnected_subgraphs(sub_graph->previous, sub_graph->next);
	virtual_scan_free_extended(sub_graph->virtual_scan_extended);
	free(sub_graph->sub_graphs);
}


void
free_oldest_disconnected_subgraph(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	virtual_scan_disconnected_sub_graph_t *first = neighborhood_graph->first;
	virtual_scan_disconnected_sub_graph_t *second = first->next;

	virtual_scan_disconnected_sub_graph_iterator_t iterator = {second, 0, 0};
	for ( ; ; )
	{
		virtual_scan_graph_node_t *graph_node = next(&iterator);
		if (graph_node == NULL)
			break;
		free(graph_node->parents.pointers);
		graph_node->parents.num_pointers = 0;
		graph_node->parents.pointers = NULL;
	}
	neighborhood_graph->first = second;
	free_disconnected_subgraph(first);
}


void
update_neighborhood_graph (
		virtual_scan_neighborhood_graph_t *neighborhood_graph,
		virtual_scan_box_model_hypotheses_t *box_model_hypotheses,
		virtual_scan_extended_t *virtual_scan_extended)
{
	virtual_scan_disconnected_sub_graph_t *new_disconnected_sub_graph = virtual_scan_compute_disconnected_sub_graph(box_model_hypotheses, virtual_scan_extended);
	neighborhood_graph->num_sub_graphs++;
	if (neighborhood_graph->first == NULL)
	{
		neighborhood_graph->first = new_disconnected_sub_graph;
		neighborhood_graph->last = new_disconnected_sub_graph;
		return;
	}
	connect_disconnected_subgraphs(neighborhood_graph->last, new_disconnected_sub_graph);
	neighborhood_graph->last->next = new_disconnected_sub_graph;
	new_disconnected_sub_graph->previous = neighborhood_graph->last;
	new_disconnected_sub_graph->next = NULL;
	neighborhood_graph->last = new_disconnected_sub_graph;
	if (neighborhood_graph->num_sub_graphs > MAX_NUM_DISCONNECTED_SUB_GRAPHS)
	{
		free_oldest_disconnected_subgraph(neighborhood_graph);
		neighborhood_graph->num_sub_graphs--;
	}
}
