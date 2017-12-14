#include "neighborhood_graph.h"

#include "parameters.h"

namespace virtual_scan
{

static const Model *BIKE = new Model(1.2, 2.2);

static const Model *BUS = new Model(2.5, 12.0);

static const Model *CAR = new Model(1.9, 5.1);

static const Model *PEDESTRIAN = new Model(0.0, 0.0);

static const Model *UNKNOWN = new Model(0.0, 0.0);

static std::vector<const Model*> VEHICLES = {BIKE, BUS, CAR};


Model::Model(double width, double length):
	width(width),
	length(length)
{
	// Nothing to do.
}


Node::Node():
	timestamp(0),
	model(UNKNOWN),
	selected_latch(NULL)
{
	// Nothing to do.
}


Node::Node(double timestamp, const Pose &origin, const Pose &local, const Model *model, int *selected):
	timestamp(timestamp),
	model(model),
	selected_latch(selected)
{
	pose.global = origin.project_global(local);
	pose.local = local;
}


void Node::adopt(Node &child)
{
	// Only nodes of same model can have child / parent relations.
	if (this->model != child.model)
		return;

	// Reject the operation if it would suggest the obstacle
	// moved at a speed higher than the maximum limit.
	double ds = distance(this->pose.global, child.pose.global);
	double dt = std::abs(this->timestamp - child.timestamp);
	if (ds / dt > V_MAX)
		return;

	this->children.push_back(&child);
	child.parents.push_back(this);
}


void Node::connect(Node &node)
{
	this->siblings.push_back(&node);
	node.siblings.push_back(this);
}


void Node::select()
{
	if (selected_latch != NULL)
		(*selected_latch)++;
}


void Node::deselect()
{
	if (selected_latch != NULL && *selected_latch > 0)
		(*selected_latch)--;
}


bool Node::selected() const
{
	return (selected_latch != NULL ? *selected_latch > 0 : false);
}


Cluster::Cluster():
	selected_latch(0)
{
	// Nothing to do.
}


Cluster::Cluster(const Segment &segment):
	selected_latch(0)
{
	switch (segment.shape)
	{
		case I_SHAPED   : init_I_SHAPED(segment);   break;
		case L_SHAPED   : init_L_SHAPED(segment);   break;
		case POINT_MASS : init_POINT_MASS(segment); break;
	}
}


void Cluster::add_node(double timestamp, const Pose &origin, const Pose &local, const Model *model)
{
	emplace_back(timestamp, origin, local, model, &selected_latch);
	Node &node = back();
	for (auto i = rbegin() + 1, n = rend(); i != n; i++)
		node.connect(*i);
}


inline std::vector<PointXY> find_side_points(double length, double theta, const PointXY &first_point, const PointXY &last_point)
{
	std::vector<PointXY> points;

	double cos_o = std::cos(theta);
	double sin_o = std::sin(theta);

	points.emplace_back(
		first_point.x + 0.5 * length * cos_o,
		first_point.y + 0.5 * length * sin_o
	);

	points.emplace_back(
		last_point.x - 0.5 * length * cos_o,
		last_point.y - 0.5 * length * sin_o
	);

	points.emplace_back(
		0.5 * (first_point.x + last_point.x),
		0.5 * (first_point.y + last_point.y)
	);

	return points;
}


void Cluster::init_I_SHAPED(const Segment &segment)
{
	const PointXY &first_point = segment.front();
	const PointXY &last_point = segment.back();
	double theta = angle(first_point, last_point);
	double d = distance(first_point, last_point);

	for (auto i = VEHICLES.begin(), n = VEHICLES.end(); i != n; ++i)
	{
		const Model *model = *i;
		double length = model->length;
		double width = model->width;
		double l_2 = 0.5 * length;
		double w_2 = 0.5 * width;

		if (d <= length)
		{
			// Compute two boxes for the center from first_point and two more from last_point
			std::vector<PointXY> side_points = find_side_points(length, theta, first_point, last_point);
			for (auto p_side = side_points.begin(), n = side_points.end(); p_side != n; ++p_side)
			{
				for (double delta_phi = -M_PI_2; delta_phi < M_PI; delta_phi += M_PI)
				{
					double phi = carmen_normalize_theta(theta + delta_phi);
					PointXY position = *p_side + PointXY(w_2 * cos(phi), w_2 * sin(phi));
					for (double direction = 0; direction <= M_PI; direction += M_PI)
					{
						double o = carmen_normalize_theta(theta + direction);
						add_node(segment.timestamp, segment.origin, Pose(position, o), model);
					}
				}
			}
		}

		if (d <= width)
		{
			// Compute two boxes for the center from first_point and two more from last_point
			std::vector<PointXY> side_points = find_side_points(width, theta, first_point, last_point);
			for (auto p_side = side_points.begin(), n = side_points.end(); p_side != n; ++p_side)
			{
				for (double delta_phi = -M_PI_2; delta_phi < M_PI; delta_phi += M_PI)
				{
					double phi = carmen_normalize_theta(theta + delta_phi);
					PointXY position = *p_side + PointXY(l_2 * cos(phi), l_2 * sin(phi));
					for (double direction = 0; direction <= M_PI; direction += M_PI)
					{
						double o = carmen_normalize_theta(phi + direction);
						add_node(segment.timestamp, segment.origin, Pose(position, o), model);
					}
				}
			}
		}
	}
}


void Cluster::init_L_SHAPED(const Segment &segment)
{
	const PointXY &corner = segment.corner;
	PointXY length_tip = segment.front();
	PointXY width_tip = segment.back();

	for (auto i = VEHICLES.begin(), n = VEHICLES.end(); i != n; ++i)
	{
		const Model *model = *i;
		double l_2 = 0.5 * model->length;
		double w_2 = 0.5 * model->width;

		for (int j = 0; j < 2; j++) // Indexes the matching orientation of model categories
		{
			std::swap(length_tip, width_tip);
			bool too_long = model->length < distance(corner, length_tip);
			bool too_wide = model->width < distance(corner, width_tip);
			if (too_long || too_wide)
				continue;

			double rho = angle(corner, length_tip);
			double phi = angle(corner, width_tip);

			PointXY position = corner
				+ PointXY(l_2 * cos(rho), l_2 * sin(rho))  // Shift along the length side
				+ PointXY(w_2 * cos(phi), w_2 * sin(phi)); // Shift parallel to the width side

			for (double direction = 0; direction <= M_PI; direction += M_PI)
			{
				double o = carmen_normalize_theta(rho + direction);
				add_node(segment.timestamp, segment.origin, Pose(position, o), model);
			}
		}
	}
}


void Cluster::init_POINT_MASS(const Segment &segment)
{
	emplace_back(segment.timestamp, segment.origin, Pose(segment.centroid()), PEDESTRIAN, &selected_latch);	
}


bool Cluster::selected() const
{
	return (selected_latch > 0);
}


Subgraph::iterator_nodes::iterator_nodes()
{
	// Nothing to do.
}


Subgraph::iterator_nodes::iterator_nodes(const Cluster::S::iterator &cluster, const Node::S::iterator &node):
	cluster(cluster),
	node(node)
{
	// Nothing to do.
}


bool Subgraph::iterator_nodes::operator != (const iterator_nodes &that) const
{
	return (this->cluster != that.cluster || this->node != that.node);
}


Subgraph::iterator_nodes &Subgraph::iterator_nodes::operator ++ ()
{
	++node;
	if (node == cluster->end())
	{
		++cluster;
		node = cluster->begin();
	}

	return *this;
}


Node &Subgraph::iterator_nodes::operator * ()
{
	return *node;
}


Node *Subgraph::iterator_nodes::operator -> ()
{
	return node.operator->();
}


Subgraph::Subgraph()
{
	// Nothing to do.
}


Subgraph::Subgraph(const Reading &reading):
	timestamp(reading.timestamp)
{
	Segment::S segments = split_segments(reading);
	for (auto i = segments.begin(), n = segments.end(); i != n; ++i)
		emplace_back(*i);
}


Subgraph::Subgraph(Subgraph &parent, const Reading &reading):
	Subgraph(reading)
{
	for (auto i = parent.begin_nodes(), m = parent.end_nodes(); i != m; ++i)
		for (auto j = this->begin_nodes(), n = this->end_nodes(); j != n; ++j)
			i->adopt(*j);
}


Subgraph::iterator_nodes Subgraph::begin_nodes()
{
	if (size() > 0)
		return iterator_nodes(begin(), front().begin());
	else
		return iterator_nodes();
}


Subgraph::iterator_nodes Subgraph::end_nodes()
{
	if (size() > 0)
		return iterator_nodes(end(), back().end());
	else
		return iterator_nodes();
}


void Graph::update(const Reading &reading)
{
	if (size() == T)
	{
		pop_front();

		// Remove all dangling parent node pointers from the now-oldest subgraph.
		for (auto i = front().begin_nodes(), n = front().end_nodes(); i != n; ++i)
			i->parents.clear();
	}

	if (size() > 0)
		emplace_back(back(), reading);
	else
		emplace_back(reading);
}


} // namespace virtual_scan
