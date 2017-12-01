#include "virtual_scan_tracker.h"

#include "parameters.h"

#include <algorithm>
#include <limits>

namespace virtual_scan
{

Obstacle::Obstacle()
{
	graph_node = NULL;
	x = 0.0;
	y = 0.0;
	theta = 0.0;
}


ObstacleView::ObstacleView():
	range(std::make_pair(0.0, 0.0)),
	pose({0, 0, 0})
{
	// Nothing to do.
}


template<class T> double angle(const T &p)
{
	return atan2(p.y, p.x);
}


template<class T> T rotate(const T &p, double o)
{
	double x = p.x;
	double y = p.y;

	double cos_o = cos(o);
	double sin_o = sin(o);

	T p_o;
	p_o.x = x * cos_o - y * sin_o;
	p_o.y = x * sin_o + y * cos_o;

	return p_o;
}


template<class P, class S> P shift(const P &p, const S &s)
{
	P p_s = p;
	p_s.x += s.x;
	p_s.y += s.y;
	return p_s;
}


inline carmen_position_t make_corner(double x, double y, const carmen_point_t &pose)
{
	carmen_position_t c = {x, y};
	return shift(rotate(c, pose.theta), pose);
}


ObstacleView::ObstacleView(const Obstacle &obstacle, const carmen_point_t &globalpos)
{
	// Compute obstacle view orientation relative to observer.
	pose.theta = carmen_normalize_theta(obstacle.theta - globalpos.theta);

	// Shift view's reference frame so that observer stands in (0, 0).
	pose.x = obstacle.x - globalpos.x;
	pose.y = obstacle.y - globalpos.y;

	// Rotate view's reference frame clockwise by o radians.
	pose = rotate(pose, -globalpos.theta);

	// Compute the positions of the obstacle's four corners.
	std::vector<carmen_position_t> corners;
	double w_2 = 0.5 * obstacle.graph_node->box_model.width;
	double l_2 = 0.5 * obstacle.graph_node->box_model.length;
	corners.push_back(make_corner(-l_2, -w_2, pose));
	corners.push_back(make_corner(l_2, -w_2, pose));
	corners.push_back(make_corner(l_2, w_2, pose));
	corners.push_back(make_corner(-l_2, w_2, pose));

	// Compute the obstacle's side lines.
	for (int i = 0, n = corners.size(); i < n; i++)
		sides.emplace_back(corners[i], corners[(i + 1) % n]);

	// Record the angles of the corners that have a clear view of the observer.
	std::vector<double> angles;
	for (int i = 0, m = corners.size(); i < m; i++)
	{
		bool obstructed = false;
		const carmen_position_t &corner = corners[i];
		for (int j = 0, n = sides.size(); j < n && !obstructed; j++)
			obstructed = sides[j].obstructs(corner);

		if (!obstructed)
			angles.push_back(angle(corner));
	}

	// Select the smaller and largest angles for the view range.
	std::sort(angles.begin(), angles.end());
	range.first = angles.front();
	range.second = angles.back();
}


Obstacle::Obstacle(virtual_scan_graph_node_t *graph_node)
{
	this->graph_node = graph_node;
	x = this->graph_node->box_model.x;
	y = this->graph_node->box_model.y;
	theta = this->graph_node->box_model.theta;
}


bool ObstacleView::operator < (const ObstacleView &that) const
{
	return this->range.first < that.range.first;
}


bool ObstacleView::operator < (const carmen_point_t &point) const
{
	// TODO: Check for corner cases (e.g. angles > 90 degrees).
	return this->range.first < point.theta;
}


bool ObstacleView::operator > (const carmen_point_t &point) const
{
	// TODO: Check for corner cases (e.g. angles > 90 degrees).
	return this->range.second > point.theta;
}


double ObstacleView::P_M1(const carmen_point_t &p) const
{
	Line ray({p.x, p.y});

	double d_min = std::numeric_limits<double>::max();
	carmen_position_t c = {d_min, d_min};

	// Find the obstacle point closest to the observer alongside the ray.
	for (int i = 0, n = sides.size(); i < n; i++)
	{
		std::pair<double, double> crossing = ray.crosspoint(sides[i]);
		if (crossing.first < d_min && 0 <= crossing.second && crossing.second <= 1.0)
		{
			d_min = crossing.first;
			c = ray(d_min);
		}
	}

	double d_n = DIST2D(p, c);

    return LAMBDA_1 * std::exp(-LAMBDA_1 * d_n);
}


virtual_scan_graph_node_t *Obstacle::operator -> ()
{
	return(graph_node);
}



Track::~Track()
{
	for (int i = 0, n = graph_nodes.size(); i < n; i++)
		graph_nodes[i]->complete_sub_graph->selected = 0; //graph_nodes[i].graph_node->complete_sub_graph->selected = 0;
}

size_t Track::size() const
{
	return graph_nodes.size();
}

void Track::append_front(virtual_scan_graph_node_t *node)
{
	graph_nodes.push_front(node); // graph_nodes.push_front(Obstacle(node));
}

void Track::append_back(virtual_scan_graph_node_t *node)
{
	graph_nodes.push_back(node);
}

virtual_scan_graph_node_t *Track::front_node()
{
	return graph_nodes.front().graph_node;
}

virtual_scan_graph_node_t *Track::back_node()
{
	return graph_nodes.back().graph_node;
}

void Track::track_forward_reduction(int r)
{
	for (int i = r + 1, n = graph_nodes.size(); i < n; i++)
		graph_nodes[i]->complete_sub_graph->selected = 0;
	graph_nodes.erase(graph_nodes.begin() + (r + 1), graph_nodes.end());
}

void Track::track_backward_reduction(int r)
{
	for (int i = 0; i < r; i++)
			graph_nodes[i]->complete_sub_graph->selected = 0;
	graph_nodes.erase(graph_nodes.begin(), graph_nodes.begin() + r);
}

void Track::track_move(Track *tau, int s)
{
	for (int i = s + 1, n = graph_nodes.size(); i < n; i++)
	{
		tau->graph_nodes.push_back(graph_nodes[i]);
	}
	graph_nodes.erase(graph_nodes.begin() + (s + 1), graph_nodes.end());
}

bool Track::is_mergeable(Track *tau)
{
	virtual_scan_graph_node_t *last_node = this->graph_nodes.back().graph_node;
	virtual_scan_graph_node_t *first_node = tau->graph_nodes.front().graph_node;
	for (int i = 0; i < first_node->parents.num_pointers; i++)
	{
		virtual_scan_graph_node_t *parent = (virtual_scan_graph_node_t *) first_node->parents.pointers[i];
		if (parent == last_node)
		{
			return true;
		}
	}
	return false;
}

void Track::track_merge(Track *tau)
{
	tau->track_move(this, -1);
}

inline bool is_parent(virtual_scan_graph_node_t *node_1, virtual_scan_graph_node_t *node_2)
{
	for (int i = 0; i < node_2->parents.num_pointers; i++)
	{
		virtual_scan_graph_node_t *parent = (virtual_scan_graph_node_t *) node_2->parents.pointers[i];
		if (parent == node_1)
		{
			return true;
		}
	}
	return false;
}

std::pair <int, int> Track::is_switchable(Track *tau)
{
	for (int p = 0, m = this->size() - 1; p < m; p++)
	{
		virtual_scan_graph_node_t *t_p = this->graph_nodes[p].graph_node;
		virtual_scan_graph_node_t *t_p_plus_1 = this->graph_nodes[p + 1].graph_node;
		for (int q = 0, n = tau->size() - 1; q < n; q++)
		{
			virtual_scan_graph_node_t *t_q = tau->graph_nodes[q].graph_node;
			virtual_scan_graph_node_t *t_q_plus_1 = tau->graph_nodes[q + 1].graph_node;
			if (is_parent(t_p, t_q_plus_1) && is_parent(t_q, t_p_plus_1))
				return(std::make_pair(p,q));
		}
	}
	return (std::make_pair(-1, -1));
}

void Track::track_switch(Track *tau, std::pair <int, int> break_point_pair)
{
	Track tau_temp;

	int p = break_point_pair.first;
	int q = break_point_pair.second;

	track_move(&tau_temp, p);
	tau->track_move(this, q);
	tau_temp.track_move(tau, -1);
}

inline int random_int(int a, int b, std::random_device *rd)
{
	std::uniform_int_distribution <> u(a, b);
	return u(*rd);
}

void Track::track_update(std::random_device *rd)
{
	std::normal_distribution <> normal;
	int n = random_int(0, this->size(), rd);
	Obstacle *obstacle = &this->graph_nodes[n];
	obstacle->x += normal(*rd);
	obstacle->y += normal(*rd);
	obstacle->theta = carmen_normalize_theta(obstacle->theta + normal(*rd));
}


ObstacleView Track::view(int t, const carmen_point_t &globalpos) const
{
	return ObstacleView(graph_nodes[t], globalpos);
}


double Track::P_L(double lambda_L, int T)
{
	// f(x) = exp(lambda_L * x)
	// F(x) = exp(lambda_L * x) / lambda_L
	// P_L = f(x) / int_0^T f(x) dx

	return (lambda_L * exp(lambda_L * size())) / (exp(lambda_L * T) - 1);
}

double Track::P_T() const
{
    // TODO: implement function.
    return 1.0;
}

Tracks::Tracks(std::random_device *rd_):
	rd(rd_)
{
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
	track_extension(tau);

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
		child = (virtual_scan_graph_node_t *) graph_node->children.pointers[i];
		if (!child->complete_sub_graph->selected)
			indexes.push_back(i);
	}
	if (indexes.size() == 0)
		return false;

	int n = indexes[random_int(0, indexes.size(), rd)];
	child = (virtual_scan_graph_node_t *) graph_node->children.pointers[n];
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
		parent = (virtual_scan_graph_node_t *) graph_node->parents.pointers[i];
		if (!parent->complete_sub_graph->selected)
			indexes.push_back(i);
	}
	if (indexes.size() == 0)
		return false;

	int n = indexes[random_int(0, indexes.size(), rd)];
	parent = (virtual_scan_graph_node_t *) graph_node->parents.pointers[n];
	tau->append_front(parent);

	return true;
}

bool Tracks::track_extension(Track *tau)
{
	int n = random_int(0, 2, rd); // 0 denotes forward extension and 1 backward extension

	if (n == 0 && forward_extension(tau)==false)// Forward extension
		return backward_extension(tau);
	if (n == 1 && backward_extension(tau)==false)// Backward extension
		return forward_extension(tau);
	return true;
}

bool Tracks::track_extension()
{
	if (tracks.size() == 0)
		return false;
	int n = random_int(0, tracks.size(), rd);
	Track *tau = &tracks[n];

	return track_extension(tau);
}

bool Tracks::track_reduction()
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

bool Tracks::track_split()
{
	// Verifying if there is a track with 4 or more nodes
	std::vector <int> indexes;
	for (int i = 0, n = tracks.size(); i < n; i++)
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

bool Tracks::track_merge()
{
	std::vector <std::pair <int, int>> pairs;
	for (int i = 0, n = tracks.size(); i < n; i++)
	{
		Track *tau_1 = &tracks[i];
		for (int j = i + 1; j < n; j++)
		{
			Track *tau_2 = &tracks[j];
			if (tau_1->is_mergeable(tau_2))
				pairs.push_back(std::make_pair(i, j));
			else if (tau_2->is_mergeable(tau_1))
				pairs.push_back(std::make_pair(j, i));
		}
	}

	if (pairs.size() < 1)
		return false;

	int n = random_int(0, pairs.size(), rd);
	Track *tau_1 = &tracks[pairs[n].first];
	Track *tau_2 = &tracks[pairs[n].second];
	tau_1->track_merge(tau_2);

	tracks.erase(tracks.begin() + pairs[n].second);

	return true;
}

bool Tracks::track_switch()
{
	std::vector <std::pair <int, int>> track_pairs;
	std::vector <std::pair <int, int>> break_points_pairs;
	for (int i = 0, n = tracks.size(); i < n; i++)
	{
		Track *tau_1 = &tracks[i];
		for (int j = i + 1; j < n; j++)
		{
			Track *tau_2 = &tracks[j];
			for (int k = 0; k < 2; k++)
			{
				std::pair <int, int> break_points = tau_1->is_switchable(tau_2);
				if (break_points.first != -1)
				{
					track_pairs.push_back(std::make_pair(i, j));
					break_points_pairs.push_back(break_points);
				}
				std::swap(tau_1, tau_2);
				std::swap(i, j);
			}
		}
	}

	if (track_pairs.size() < 1)
		return false;

	int n = random_int(0, track_pairs.size(), rd);
	Track *tau_1 = &tracks[track_pairs[n].first];
	Track *tau_2 = &tracks[track_pairs[n].second];
	tau_1->track_switch(tau_2, break_points_pairs[n]);

	return true;
}

bool Tracks::track_diffusion()
{
	if (tracks.size() < 1)
		return false;
	int n = random_int(0, tracks.size(), rd);
	Track *tau = &tracks[n];
	tau->track_update(rd);
	return true;
}


double Tracks::P_M1(int i, virtual_scan_neighborhood_graph_t *neighborhood_graph) const
{
	virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph = virtual_scan_get_disconnected_sub_graph(neighborhood_graph, i);
	virtual_scan_extended_t *reading = disconnected_sub_graph->virtual_scan_extended;
	const carmen_point_t &globalpos = reading->globalpos;

	std::vector<ObstacleView> views;
	for (int j = 0, n = tracks.size(); j < n; j++)
		views.push_back(tracks[j].view(i, globalpos));

	std::sort(views.begin(), views.end());

	double p = 1.0;

	int k = 0;
	for (int j = 0, n = reading->num_points; j < n; j++)
	{
		const ObstacleView &view = views[k];
		const carmen_point_t &point = reading->points[j];
		while (view < point)
			k = (k + 1) % views.size();

		if (view > point)
			continue;

		p *= view.P_M1(point);
	}

	return p;
}


Tracks *Tracks::propose(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	Tracks *tracks = new Tracks(*this);

	bool result = false;
	while (!result)
	{
		int n = random_int(0, 8, rd);
		switch(n)
		{
			case 0:
				result = tracks->track_birth(neighborhood_graph);
				break;
			case 1:
				result = tracks->track_death();
				break;
			case 2:
				result = tracks->track_diffusion();
				break;
			case 3:
				result = tracks->track_extension();
				break;
			case 4:
				result = tracks->track_merge();
				break;
			case 5:
				result = tracks->track_reduction();
				break;
			case 6:
				result = tracks->track_split();
				break;
			case 7:
				result = tracks->track_switch();
				break;
		}
	}

	return tracks;
}

double Tracks::P()
{
	return 0.0;
}

Tracker::Tracker(int n, int T):
	n_mc(n),
	neighborhood_graph(virtual_scan_initiate_neighborhood_graph (T)),
	tracks_n(new Tracks(&rd)),
	tracks_star(new Tracks(&rd)),
	uniform(0.0, 1.0)
{
}

// MCMC Sampler
Tracks *Tracker::track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses, virtual_scan_extended_t *virtual_scan_extended)
{
	update_neighborhood_graph(neighborhood_graph, box_model_hypotheses, virtual_scan_extended);
	for (int n = 0; n < n_mc; n++)
	{
		Tracks *tracks_new = tracks_n->propose(neighborhood_graph);
		double U = uniform(rd);
		if (U < (tracks_new->P() / tracks_n->P())) 
		{
			if (tracks_n != tracks_star)
				delete tracks_n;
			tracks_n = tracks_new;
			if (tracks_n->P() > tracks_star->P())
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

