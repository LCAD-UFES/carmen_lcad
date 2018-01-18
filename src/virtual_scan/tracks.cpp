#include "tracks.h"

#include "random.h"

#include <algorithm>

namespace virtual_scan
{

#ifdef DEBUG
	#define LOG(message) std::cout << message << std::endl
#else
	#define LOG(message)
#endif


ObstaclePose::S Tracks::obstacles() const
{
	ObstaclePose::S poses;
	for (auto track = begin(), n = end(); track != n; ++track)
		poses.push_back(track->back());

	return poses;
}


bool Tracks::create(Graph &graph)
{
	LOG("Before create: " << *this);

	Subgraph &subgraph = random_choose(graph);

	// Verifying if there is a complete_sub_graph not selected yet
	std::vector<Cluster::S::iterator> unselected;
	for (auto cluster = subgraph.begin(), n = subgraph.end(); cluster != n; ++cluster)
		if (!cluster->selected())
			unselected.push_back(cluster);

	if (unselected.size() == 0)
	{
		LOG("After create (unselected == {}): " << *this);
		return false;
	}

	Cluster &cluster = *random_choose(unselected);
	Node &node = random_choose(cluster);

	emplace_back();
	Track &tau = back();
	tau.push_back(&node);

	extend(tau);
	if (tau.size() < 2)
	{
		pop_back();
		LOG("After create (failed extension): " << *this);
		return false;
	}

	PwZ.create(size() - 1, *this);

	LOG("After create (success): " << *this);

	return true;
}


bool Tracks::destroy()
{
	LOG("Before destroy: " << *this);

	if (size() == 0)
	{
		LOG("After destroy (no tracks): " << *this);
		return false;
	}

	// Randomly select a track and destroy it.
	size_t n = random_int(0, size());
	PwZ.destroy(n, *this);
	destroy(n);

	LOG("After destroy (success): " << *this);

	return true;
}


bool Tracks::destroy(size_t n)
{
	// Move the selected track to the end of the sequence if it's not already there.
	if (n < (size() - 1))
		std::swap(at(n), back());

	// Remove the selected track.
	pop_back();

	return true;
}


bool Tracks::extend()
{
	if (size() == 0)
		return false;

	return extend(random_choose(*this));
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


inline Node::Edges unselected(const Node::Edges &nodes)
{
	Node::Edges found;
	for (auto i = nodes.begin(), n = nodes.end(); i != n; ++i)
		if (!(*i)->selected())
			found.push_back(*i);

	return found;
}


bool Tracks::extend_forward(Track &tau)
{
	LOG("Before forward extension: " << *this);

	Node::Edges children = unselected(tau.back().node->children);
	if (children.size() == 0)
	{
		LOG("After forward extension (no children): " << *this);
		return false;
	}

	tau.push_back(random_choose(children));

	PwZ.extend_forward(tau);

	LOG("After forward extension (success): " << *this);

	return true;
}


bool Tracks::extend_backward(Track &tau)
{
	LOG("Before backward extension: " << *this);

	Node::Edges parents = unselected(tau.back().node->parents);
	if (parents.size() == 0)
	{
		LOG("After backward extension (no parents): " << *this);
		return false;
	}

	tau.push_front(random_choose(parents));

	PwZ.extend_backward(tau);

	LOG("After backward extension (success): " << *this);

	return true;
}


bool Tracks::reduce()
{
	LOG("Before reduction: " << *this);
	if (size() == 0)
	{
		LOG("After reduction (no tracks): " << *this);
		return false;
	}

	int i = random_int(0, size()); // Selects the track index to be reduced
	Track &tau = at(i);
	int r = random_int(1, tau.size() - 1); // Selects the cutting index

	int mode = random_int(0, 2); // 0 denotes forward reduction and 1 backward reduction
	if (mode == 0) // Forward reduction
	{
		PwZ.pop_back(i, r, *this);
		tau.pop_back(r);
		LOG("After forward reduction (success): " << *this);
	}
	else // Backward reduction
	{
		PwZ.pop_front(i, r, *this);
		tau.pop_front(r);
		LOG("After backward reduction (success): " << *this);
	}

	return true;
}


bool Tracks::split()
{
	LOG("Before split: " << *this);

	// Verifying if there is a track with 4 or more nodes
	std::vector<Track*> found;
	for (int i = 0, n = size(); i < n; i++)
		if (at(i).size() >= 4)
			found.push_back(&at(i));

	if (found.size() == 0)
	{
		LOG("After split (no suitable tracks): " << *this);
		return false;
	}

	// Selects the track index to be split.
	Track &tau_1 = *random_choose(found);

	// Create new track to receive section split from tau_1.
	emplace_back(); 
	Track &tau_2 = back();

	tau_1.pop_back(random_int(1, tau_1.size() - 1), tau_2); // Split tau at a random index

	PwZ.split(tau_1, tau_2);

	LOG("After split (success): " << *this);

	return true;
}

bool Tracks::merge()
{
	LOG("Before merge: " << *this);

	std::vector <std::pair<int, int>> pairs;
	for (int i = 0, n = size(); i < n; i++)
	{
		Track &tau_1 = at(i);
		for (int j = i + 1; j < n; j++)
		{
			Track &tau_2 = at(j);
			if (tau_1.is_mergeable(tau_2))
				pairs.push_back(std::make_pair(i, j));
			else if (tau_2.is_mergeable(tau_1))
				pairs.push_back(std::make_pair(j, i));
		}
	}

	if (pairs.size() == 0)
	{
		LOG("After merge (no suitable pairs): " << *this);
		return false;
	}

	std::pair<int, int> &pair = random_choose(pairs);
	Track &tau_1 = at(pair.first);
	Track &tau_2 = at(pair.second);
	tau_1.merge(tau_2);

	PwZ.merge(tau_1, tau_2);
	destroy(pair.second);

	LOG("After merge (success): " << *this);

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
	bool is_parent(Node *node_1, Node *node_2)
	{
		const Node::Edges &parents = node_2->parents;
		for (auto i = parents.begin(), n = parents.end(); i != n; ++i)
			if (*i == node_1)
				return true;

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
			Node *t_p = a[p].node;
			Node *t_p_plus_1 = a[p + 1].node;
			for (q = 0; q < n; q++)
			{
				Node *t_q = b[q].node;
				Node *t_q_plus_1 = b[q + 1].node;
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
	LOG("Before swap: " << *this);

	std::vector<Swap> swappings;
	for (int i = 0, n = size(); i < n; i++)
	{
		for (int j = i + 1; j < n; j++)
		{
			Swap swapping(i, j, *this);
			if (swapping.valid())
				swappings.push_back(swapping);
		}
	}

	if (swappings.size() == 0)
	{
		LOG("After swap (no swappable tracks): " << *this);
		return false;
	}

	int n = random_int(0, swappings.size());
	Swap &swapping = swappings[n];
	swapping(*this);

	PwZ.swap(swapping.i, swapping.j, *this);

	LOG("After swap (success): " << *this);

	return true;
}


bool Tracks::diffuse()
{
	LOG("Before diffusion: " << *this);

	if (size() == 0)
	{
		LOG("After diffusion (no tracks): " << *this);
		return false;
	}

	int i = random_int(0, size());
	Track &track = at(i);

	int j = track.diffuse();

	PwZ.diffuse(i, j, *this);

	LOG("After diffusion (success): " << *this);

	return true;
}


Tracks::P Tracks::propose(Graph &graph)
{
	Tracks::P tracks(new Tracks(*this));

	// Create a vector of change operations.
	std::vector<std::function<bool()>> operations = {
		[&]() {return tracks->create(graph);},
		[&]() {return tracks->destroy();},
		[&]() {return tracks->diffuse();},
		[&]() {return tracks->extend();},
		[&]() {return tracks->merge();},
		[&]() {return tracks->reduce();},
		[&]() {return tracks->split();},
		[&]() {return tracks->swap();}
	};

	// Shuffle the operation vector.
	std::shuffle(operations.begin(), operations.end(), std::mt19937(RD()));

	// Tries the shuffled operations one by one, returning the changed
	// Tracks object as soon as one succeeds.
	for (std::function<bool()> &operation: operations)
		if (operation())
			return tracks;

	// If no operations suceed, return an empty Tracks reference.
	return Tracks::P();
}


void Tracks::update(const Readings &readings)
{
	double timestamp = readings.front().timestamp;

	for (size_t i = 0, m = size(); i < m; i++)
	{
		Track &track = at(i);
		size_t n = track.size();
		size_t j = 0;

		while (j < n && track.front().node->timestamp < timestamp)
			j++;

		if (j == 0)
			continue;

		if (j < n)
		{
			PwZ.shorten(i, j, *this);
			track.pop_front(j);
		}
		else
		{
			PwZ.destroy(i, *this);
			destroy(i);
		}
	}

	PwZ.update(readings);
}


std::ostream &operator << (std::ostream &out, const Tracks &tracks)
{
	out << "Tracks (total = " << tracks.size() << "):";

	for (const Track &track: tracks)
		out << std::endl << "  " << track;

	return out;
}


} // namespace virtual_scan
