#include "tracks.h"

#include "random.h"

#include <algorithm>

namespace virtual_scan
{


bool Tracks::create(virtual_scan_neighborhood_graph_t *graph)
{
	int n = random_int(0, graph->num_sub_graphs);
	virtual_scan_disconnected_sub_graph_t *subgraph = virtual_scan_get_disconnected_sub_graph(graph, n);
		
	// Verifying if there is a complete_sub_graph not selected yet
	std::vector <int> indexes;
	for (int i = 0; i < subgraph->num_sub_graphs; i++)
	{
		if (!subgraph->sub_graphs[i].selected)
			indexes.push_back(i);
	}

	if (indexes.size() == 0)
		return false;

	virtual_scan_complete_sub_graph_t *nodes = subgraph->sub_graphs + indexes[random_int(0, indexes.size())];
	nodes->selected = 1;

	virtual_scan_graph_node_t *node = nodes->nodes + random_int(0, nodes->num_nodes);

	tracks.emplace_back();
	Track &tau = tracks.back();
	tau.push_front(node);
	extend(tau);

	if (tau.size() < 2)
	{
		tracks.pop_back();
		return false;
	}

	return true;
}


bool Tracks::destroy()
{
	if (tracks.size() == 0)
		return false;

	int n = random_int(0, tracks.size()); // Selects the track to be deleted
	tracks.erase(tracks.begin() + n);

	return true;
}


bool Tracks::extend()
{
	if (tracks.size() == 0)
		return false;

	int n = random_int(0, tracks.size());
	return extend(tracks[n]);
}


bool Tracks::extend(Track &tau)
{
	int n = random_int(0, 2); // 0 denotes forward extension and 1 backward extension

	if (n == 0 && extend_forward(tau) == false) // Forward extension
		return extend_backward(tau);

	if (n == 1 && extend_backward(tau) == false) // Backward extension
		return extend_forward(tau);

	return true;
}


bool Tracks::extend_forward(Track &tau)
{
	const virtual_scan_elements_t &children = tau.back_node()->children;
	int num_children = children.num_pointers;
	if (num_children == 0)
		return false;

	// Verifying if there is a child node not selected yet
	std::vector<int> indexes;
	for (int i = 0, n = children.num_pointers; i < n; i++)
	{
		virtual_scan_graph_node_t *child = (virtual_scan_graph_node_t*) children.pointers[i];
		if (!child->complete_sub_graph->selected)
			indexes.push_back(i);
	}

	if (indexes.size() == 0)
		return false;

	int n = indexes[random_int(0, indexes.size())];
	tau.push_back((virtual_scan_graph_node_t*) children.pointers[n]);

	return true;
}


bool Tracks::extend_backward(Track &tau)
{
	const virtual_scan_elements_t &parents = tau.back_node()->parents;
	int num_parents = parents.num_pointers;
	if (num_parents == 0)
		return false;

	// Verifying if there is a parent node not selected yet
	std::vector<int> indexes;
	for (int i = 0; i < parents.num_pointers; i++)
	{
		virtual_scan_graph_node_t *parent = (virtual_scan_graph_node_t*) parents.pointers[i];
		if (!parent->complete_sub_graph->selected)
			indexes.push_back(i);
	}

	if (indexes.size() == 0)
		return false;

	int n = indexes[random_int(0, indexes.size())];
	tau.push_front((virtual_scan_graph_node_t*) parents.pointers[n]);

	return true;
}


bool Tracks::reduce()
{
	if (tracks.size() == 0)
		return false;

	Track &tau = tracks[random_int(0, tracks.size())]; // Selects the track index to be reduced
	int r = random_int(1, tau.size() - 1); // Selects the cutting index

	int mode = random_int(0, 2); // 0 denotes forward reduction and 1 backward reduction
	if (mode == 0) // Forward reduction
		tau.pop_back(r);
	else // Backward reduction
		tau.pop_front(r);

	return true;
}

bool Tracks::split()
{
	// Verifying if there is a track with 4 or more nodes
	std::vector<int> indexes;
	for (int i = 0, n = tracks.size(); i < n; i++)
	{
		if (tracks[i].size() >= 4)
			indexes.push_back(i);
	}

	if (indexes.size() == 0)
		return false;

	int n = indexes[random_int(0, indexes.size())]; // Selects the track index to be split
	Track &tau = tracks[n];
	int s = random_int(1, tau.size() - 2); // Selects the splitting index
	tracks.emplace_back(); // Add a new object Track to the end of the vector
	Track &tau_new = tracks.back();
	tau.pop_back(s, tau_new);

	return true;
}

bool Tracks::merge()
{
	std::vector <std::pair <int, int>> pairs;
	for (int i = 0, n = tracks.size(); i < n; i++)
	{
		Track &tau_1 = tracks[i];
		for (int j = i + 1; j < n; j++)
		{
			Track &tau_2 = tracks[j];
			if (tau_1.is_mergeable(tau_2))
				pairs.push_back(std::make_pair(i, j));
			else if (tau_2.is_mergeable(tau_1))
				pairs.push_back(std::make_pair(j, i));
		}
	}

	if (pairs.size() < 1)
		return false;

	int n = random_int(0, pairs.size());
	Track &tau_1 = tracks[pairs[n].first];
	Track &tau_2 = tracks[pairs[n].second];
	tau_1.merge(tau_2);

	tracks.erase(tracks.begin() + pairs[n].second);

	return true;
}


/**
 * @brief A track segment swap operation.
 */
class Swap
{
	/** @brief Swap index for the first track. */
	int p;

	/** @brief Swap index for the second track. */
	int q;

	/**
	 * @brief Return whether `node_1` is a parent of `node_2`.
	 */
	bool is_parent(virtual_scan_graph_node_t *node_1, virtual_scan_graph_node_t *node_2)
	{
		const virtual_scan_elements_t &parents = node_2->parents;
		for (int i = 0, n = parents.num_pointers; i < n; i++)
		{
			virtual_scan_graph_node_t *parent = (virtual_scan_graph_node_t*) parents.pointers[i];
			if (parent == node_1)
				return true;
		}

		return false;
	}

	/**
	 * @brief Plan a swap between the given tracks.
	 *
	 * Return whether the planning operation was successful.
	 */
	bool plan(int i, int j, Track::S &tracks)
	{
		Track &a = tracks[i];
		Track &b = tracks[j];

		int m = a.size() - 1;
		int n = b.size() - 1;
		for (p = 0; p < m; p++)
		{
			virtual_scan_graph_node_t *t_p = a.at_node(p);
			virtual_scan_graph_node_t *t_p_plus_1 = a.at_node(p + 1);
			for (q = 0; q < n; q++)
			{
				virtual_scan_graph_node_t *t_q = b.at_node(q);
				virtual_scan_graph_node_t *t_q_plus_1 = b.at_node(q + 1);
				if (is_parent(t_p, t_q_plus_1) && is_parent(t_q, t_p_plus_1))
				{
					this->i = i;
					this->j = j;
					return true;
				}
			}
		}

		return false;
	}

public:
	/** @brief Index of the first track. */
	int i;

	/** @brief Index of the second track. */
	int j;

	/**
	 * @brief Deafult constructor.
	 */
	Swap():
		p(-1),
		q(-1),
		i(-1),
		j(-1)
	{
		// Nothing to do.
	}

	/**
	 * @brief Create a new swap operation for the given track pair.
	 */
	Swap(int i, int j, Track::S &tracks):
		p(-1),
		q(-1),
		i(-1),
		j(-1)
	{
		if (!plan(i, j, tracks))
			plan(j, i, tracks);
	}

	/**
	 * @brief perform the swap.
	 */
	bool operator () (Track::S &tracks)
	{
		if (!valid())
			return false;

		Track &a = tracks[i];
		Track &b = tracks[j];
		Track temp;

		a.pop_back(p, temp);
		b.pop_back(q, a);
		temp.pop_back(-1, b);

		return true;
	}

	/**
	 * @brief Check whether this swap is valid.
	 */
	bool valid() const
	{
		return (i >= 0 && j >= 0);
	}
};


bool Tracks::swap()
{
	std::vector<Swap> swappings;
	for (int i = 0, n = tracks.size(); i < n; i++)
	{
		for (int j = i + 1; j < n; j++)
		{
			Swap swapping(i, j, tracks);
			if (swapping.valid())
				swappings.push_back(swapping);
		}
	}

	if (swappings.size() < 1)
		return false;

	int n = random_int(0, swappings.size());
	Swap &swapping = swappings[n];
	swapping(tracks);

	P_wZ.swap(swapping.i, swapping.j, tracks);

	return true;
}


bool Tracks::diffuse()
{
	if (tracks.size() < 1)
		return false;

	int i = random_int(0, tracks.size());
	Track &track = tracks[i];

	int j = track.diffuse();

	P_wZ.diffuse(i, j, tracks);

	return true;
}


double Tracks::P_M1(int i, virtual_scan_neighborhood_graph_t *neighborhood_graph) const
{
	virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph = virtual_scan_get_disconnected_sub_graph(neighborhood_graph, i);
	virtual_scan_extended_t *reading = disconnected_sub_graph->virtual_scan_extended;
	const carmen_point_t &globalpos = reading->globalpos;

	std::vector<ObstacleView> w_i;
	for (int j = 0, n = tracks.size(); j < n; j++)
		tracks[j].push_view(i, globalpos, w_i);

	std::sort(w_i.begin(), w_i.end());

	double p = 1.0;
	for (int j = 0, k = 0, m = reading->num_points, n = w_i.size(); j < m; j++)
	{
		const ObstacleView &view = w_i[k];
		const carmen_point_t &point = reading->points[j];
		while (view < point)
		{
			k++;
			if (k >= n)
				return p;
		}

		if (view > point)
			continue;

		p *= view.P_M1(point);
	}

	return p;
}

/*
Track &Ttracks::operator[] (size_t index)
{
	return tracks[index];
}


const Track &Ttracks::operator[] (size_t index) const
{
	return tracks[index];
}
*/

Tracks::P Tracks::propose(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	Tracks::P tracks(new Tracks(*this));

	bool result = false;
	while (!result)
	{
		int n = random_int(0, 8);
		switch(n)
		{
			case 0:
				result = tracks->create(neighborhood_graph);
				break;
			case 1:
				result = tracks->destroy();
				break;
			case 2:
				result = tracks->diffuse();
				break;
			case 3:
				result = tracks->extend();
				break;
			case 4:
				result = tracks->merge();
				break;
			case 5:
				result = tracks->reduce();
				break;
			case 6:
				result = tracks->split();
				break;
			case 7:
				result = tracks->swap();
				break;
		}
	}

	return tracks;
}


} // namespace virtual_scan
