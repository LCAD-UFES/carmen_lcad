#include <carmen/carmen.h>
#include "virtual_scan.h"
#include "virtual_scan_neighborhood_graph.h"

#define MAX_SPEED 60


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
	complete_sub_graph->graph_nodes = graph_nodes;
	complete_sub_graph->num_graph_nodes = num_boxes;
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
virtual_scan_compute_disconnected_sub_graph(virtual_scan_box_model_hypotheses_t *box_model_hypotheses)
{
	virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph = (virtual_scan_disconnected_sub_graph_t *)
			malloc(sizeof(virtual_scan_disconnected_sub_graph_t));
	virtual_scan_box_models_t *hypotheses = box_model_hypotheses->box_model_hypotheses;
	int num_box_model_hypotheses = box_model_hypotheses->num_box_model_hypotheses;
	virtual_scan_complete_sub_graph_t *complete_sub_graphs = (virtual_scan_complete_sub_graph_t *)
			malloc(sizeof(virtual_scan_complete_sub_graph_t) * num_box_model_hypotheses);
	disconnected_sub_graph->complete_sub_graphs = complete_sub_graphs;
	disconnected_sub_graph->num_complete_sub_graphs = num_box_model_hypotheses;
	disconnected_sub_graph->timestamp = box_model_hypotheses->timestamp;

	for (int i = 0, m = num_box_model_hypotheses; i < m; i++)
	{
		virtual_scan_compute_graph_nodes(complete_sub_graphs+i, hypotheses+i, disconnected_sub_graph->timestamp);
	}
	return disconnected_sub_graph;
}


virtual_scan_graph_node_t *
next(virtual_scan_disconnected_sub_graph_iterator_t *iterator)
{
	int i = iterator->subgraph_index;
	int j = iterator->node_index;

	virtual_scan_disconnected_sub_graph_t *disconnected_subgraph = iterator->disconnected_sub_graph;
	if (i >= disconnected_subgraph->num_complete_sub_graphs)
		return NULL;
	virtual_scan_complete_sub_graph_t *complete_sub_graph = disconnected_subgraph->complete_sub_graphs + i;
	virtual_scan_graph_node_t *graph_node = complete_sub_graph->graph_nodes + j;

	iterator->node_index++;
	if (iterator->node_index >= complete_sub_graph->num_graph_nodes)
	{
		iterator->subgraph_index++;
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
		virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph_child
){
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
update_neighborhood_graph (
		virtual_scan_neighborhood_graph_t *neighborhood_graph,
		virtual_scan_box_model_hypotheses_t *box_model_hypotheses
) {
	virtual_scan_disconnected_sub_graph_t *new_disconnected_sub_graph = virtual_scan_compute_disconnected_sub_graph(box_model_hypotheses);
	if (neighborhood_graph->first_disconnected_sub_graph == NULL)
	{
		neighborhood_graph->first_disconnected_sub_graph = new_disconnected_sub_graph;
		neighborhood_graph->last_disconnected_sub_graph = new_disconnected_sub_graph;
		return;
	}
	connect_disconnected_subgraphs(neighborhood_graph->last_disconnected_sub_graph, new_disconnected_sub_graph);
	new_disconnected_sub_graph->previous_disconnected_sub_graph = neighborhood_graph->last_disconnected_sub_graph;
	neighborhood_graph->last_disconnected_sub_graph = new_disconnected_sub_graph;
}
