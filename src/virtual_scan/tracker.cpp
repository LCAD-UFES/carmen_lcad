#include "tracker.h"

namespace virtual_scan
{

Tracker::Tracker(int n):
	n_mc(n),
	neighborhood_graph(virtual_scan_alloc_neighborhood_graph ()),
	tracks_n(new Tracks(&rd)),
	tracks_star(new Tracks(&rd)),
	uniform(0.0, 1.0)
{
}

void Track::append_front(virtual_scan_graph_node_t *node)
{
	graph_nodes.push_front(node);
}

void Track::append_back(virtual_scan_graph_node_t *node)
{
	graph_nodes.push_back(node);
}

virtual_scan_graph_node_t *Track::front_node()
{
	return graph_nodes.front();
}

virtual_scan_graph_node_t *Track::back_node()
{
	return graph_nodes.back();
}

Tracks::Tracks(std::random_device *rd_):
	rd(rd_)
{
}

bool Tracks::track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	std::uniform_int_distribution <> uniform_disconnected_sub_graph(0, neighborhood_graph->num_sub_graphs);
	int n = uniform_disconnected_sub_graph(*rd);
	virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph = neighborhood_graph->first;
	for (int i = 0; i < n; i++)
		disconnected_sub_graph = disconnected_sub_graph->next;

	// Verifying if there is a complete_sub_graph not selected yet
	std::vector <int> indexes;
	virtual_scan_complete_sub_graph_t *complete_sub_graph;
	for (int i = 0; i < disconnected_sub_graph->num_sub_graphs; i++)
	{
		if (!disconnected_sub_graph->sub_graphs[i].selected)
			indexes.push_back(i);
	}
	if (indexes.size() == 0)
		return false;

	std::uniform_int_distribution <> uniform_complete_sub_graph(0, indexes.size());
	n = indexes[uniform_complete_sub_graph(*rd)];
	complete_sub_graph = disconnected_sub_graph->sub_graphs + n;

	complete_sub_graph->selected = 1;
	std::uniform_int_distribution <> uniform_graph_node(0, complete_sub_graph->num_nodes);
	n = uniform_graph_node(*rd);
	virtual_scan_graph_node_t *node = complete_sub_graph->nodes + n;

	Track tau;
	tau.append_front(node);

	track_extension(neighborhood_graph, &tau);

	if (tau.size() >= 2)
	{
		tracks.push_back(tau);
	}

	return true;
}

bool Tracks::forward_extension(Track *tau)
{
	virtual_scan_graph_node_t *graph_node = tau->back_node();
	int num_children = graph_node->children.num_pointers;
	if (num_children == 0)
		return false;

	// Verifying if there is a child node not selected yet
	std::vector <int> indexes;
	virtual_scan_graph_node_t *child;
	for (int i = 0; i < graph_node->children.num_pointers; i++)
	{
		child = graph_node->children.pointers[i];
		if (!child->complete_sub_graph->selected)
			indexes.push_back(i);
	}
	if (indexes.size() == 0)
		return false;

	std::uniform_int_distribution <> uniform_sort_child(0, indexes.size());
	int n = indexes[uniform_sort_child(*rd)];
	child = graph_node->children.pointers[n];
	tau->append_back(child);

	return true;
}

bool Tracks::backward_extension(Track *tau)
{
	virtual_scan_graph_node_t *graph_node = tau->front_node();
	int num_parents = graph_node->parents.num_pointers;
	if (num_parents == 0)
		return false;

	// Verifying if there is a parent node not selected yet
	std::vector <int> indexes;
	virtual_scan_graph_node_t *parent;
	for (int i = 0; i < graph_node->parents.num_pointers; i++)
	{
		parent = graph_node->parents.pointers[i];
		if (!parent->complete_sub_graph->selected)
			indexes.push_back(i);
	}
	if (indexes.size() == 0)
		return false;

	std::uniform_int_distribution <> uniform_sort_parent(0, indexes.size());
	int n = indexes[uniform_sort_parent(*rd)];
	parent = graph_node->parents.pointers[n];
	tau->append_front(parent);

	return true;
}

void Tracks::track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph, Track *tau)
{
	std::uniform_int_distribution <> uniform_forward_backward_extension(0, 2); // 0 denotes forward extension and 1 backward extension
	int n = uniform_forward_backward_extension(*rd);

	if (n == 0 && forward_extension(tau)==true)// Forward extension
		return;
	// Backward extension
	backward_extension(tau);
}

void Tracks::track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{

}

Tracks *Tracks::propose(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	Tracks *tracks = new Tracks(*this);
	tracks->track_birth(neighborhood_graph);

	return tracks;
}

double Tracks::P()
{
	
}

Tracks *Tracker::track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses)
{
	update_neighborhood_graph(neighborhood_graph, box_model_hypotheses);
	for (int n = 0; n < n_mc; n++)
	{
		Tracks *tracks_new = tracks_n->propose(neighborhood_graph);
		double U = uniform(rd);
		if (U < (tracks_new->P() / tracks_n->P())) 
		{
			if (tracks_n != tracks_star)
				delete tracks_n;
			tracks_n = tracks_new;
			if (P(tracks_n) > P(tracks_star))
			{
				delete tracks_star;
				tracks_star = tracks_n;
			}		
		}
		else 
			delete tracks_new;	
	}
	
	return tracks_star;
}

}

