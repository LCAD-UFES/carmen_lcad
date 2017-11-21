#include "tracker.h"

namespace virtual_scan
{

Track::~Track()
{
	for (int i = 0; i < graph_nodes.size(); i++)
		graph_nodes[i]->complete_sub_graph->selected = 0;
}

int Track::size()
{
	return graph_nodes.size();
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

void Track::track_forward_reduction(int r)
{
	for (int i = r + 1; i < graph_nodes.size(); i++)
		graph_nodes[i]->complete_sub_graph->selected = 0;
	graph_nodes.erase(graph_nodes.begin() + (r + 1), graph_nodes.end());
}

void Track::track_backward_reduction(int r)
{
	for (int i = 0; i < r; i++)
			graph_nodes[i]->complete_sub_graph->selected = 0;
	graph_nodes.erase(graph_nodes.begin(), graph_nodes.begin() + r);
}

void Track::track_move(Track *track, int s)
{
	for (int i = s + 1; i < graph_nodes.size(); i++)
	{
		track->graph_nodes.push_back(graph_nodes[i]);
	}
	graph_nodes.erase(graph_nodes.begin() + (s + 1), graph_nodes.end());
}

bool Track::track_merge(Track *track)
{
	virtual_scan_graph_node_t *last_node = this->graph_nodes.back();
	virtual_scan_graph_node_t *first_node = track->graph_nodes.front();
	for (int i = 0; i < first_node->parents.num_pointers; i++)
	{
		virtual_scan_graph_node_t *parent = first_node->parents.pointers[i];
		if (parent == last_node)
		{
			track->track_move(this, -1);
			return true;
		}
	}
	return false;
}

Tracks::Tracks(std::random_device *rd_):
	rd(rd_)
{
}

inline int random_int(int a, int b, std::random_device *rd)
{
	std::uniform_int_distribution <> u(a, b);
	return u(*rd);
}

bool Tracks::track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	int n = random_int(0, neighborhood_graph->num_sub_graphs, rd);
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

	n = indexes[random_int(0, indexes.size(), rd)];
	complete_sub_graph = disconnected_sub_graph->sub_graphs + n;

	complete_sub_graph->selected = 1;
	n = random_int(0, complete_sub_graph->num_nodes, rd);

	virtual_scan_graph_node_t *node = complete_sub_graph->nodes + n;

	tracks.emplace_back();
	Track *tau = &tracks.back();
	tau->append_front(node);
	track_extension(neighborhood_graph, tau);

	if (tau->size() < 2)
	{
		tracks.pop_back();
	}

	return true;
}

bool Tracks::track_death()
{
	if (tracks.size() == 0)
		return false;
	int n = random_int(0, tracks.size(), rd); // Selects the track to be deleted
	tracks.erase(tracks.begin() + n);

	return true;
}

bool Tracks::track_forward_extension(Track *tau)
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

	int n = indexes[random_int(0, indexes.size(), rd)];
	child = graph_node->children.pointers[n];
	tau->append_back(child);

	return true;
}

bool Tracks::track_backward_extension(Track *tau)
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

	int n = indexes[random_int(0, indexes.size(), rd)];
	parent = graph_node->parents.pointers[n];
	tau->append_front(parent);

	return true;
}

bool Tracks::track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph, Track *tau)
{
	int n = random_int(0, 2, rd); // 0 denotes forward extension and 1 backward extension

	if (n == 0 && forward_extension(tau)==false)// Forward extension
		return backward_extension(tau);
	if (n == 1 && backward_extension(tau)==false)// Backward extension
		return forward_extension(tau);
	return true;
}

bool Tracks::track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (tracks.size() == 0)
		return false;
	int n = random_int(0, tracks.size(), rd);
	Track *tau = &tracks[n];

	return track_extension(neighborhood_graph, tau);
}

bool Tracks::track_reduction(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (tracks.size() == 0)
		return false;
	int n = random_int(0, 2, rd); // 0 denotes forward reduction and 1 backward reduction
	int r = random_int(0, tracks.size(), rd); // Selects the track index to be reduced
	Track *tau = &tracks[r];
	r = random_int(1, tracks.size() - 1, rd); // Selects the cutting index
	if (n == 0) // Forward reduction
	{
		tau->track_forward_reduction(r);
	}
	else // Backward reduction
	{
		tau->track_backward_reduction(r);
	}

	return true;
}

bool Tracks::track_split(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	// Verifying if there is a track with 4 or more nodes
	std::vector <int> indexes;
	for (int i = 0; i < tracks.size(); i++)
	{
		if (tracks[i].size() >= 4)
			indexes.push_back(i);
	}
	if (indexes.size() == 0)
		return false;

	int n = indexes[random_int(0, indexes.size(), rd)]; // Selects the track index to be split
	Track *tau = &tracks[n];
	int s = random_int(1, tau->size() - 2, rd); // Selects the splitting index
	tracks.emplace_back(); // Add a new object Track to the end of the vector
	Track *tau_new = &tracks.back();
	tau->track_move(tau_new, s);

	return true;
}

bool Tracks::track_merge(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	if (tracks.size() < 2)
		return false;

	Track *tau_1 = &tracks[random_int(0, tracks.size(), rd)];
	Track *tau_2;
	int i;
	do
	{
		i = random_int(0, tracks.size(), rd);
		tau_2 = &tracks[i];
	}
	while (tau_1 == tau_2);

	if (!tau_1->merge(tau2))
		return false;

	tracks.erase(tracks.begin() + i);

	return true;
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

Tracker::Tracker(int n):
	n_mc(n),
	neighborhood_graph(virtual_scan_alloc_neighborhood_graph ()),
	tracks_n(new Tracks(&rd)),
	tracks_star(new Tracks(&rd)),
	uniform(0.0, 1.0)
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

