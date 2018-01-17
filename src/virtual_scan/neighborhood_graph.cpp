#include "neighborhood_graph.h"

#include "parameters.h"

#include <iomanip>

namespace virtual_scan
{

static const Model *BIKE = new Model(Model::BIKE, "bike", 1.2, 2.2);

static const Model *BUS = new Model(Model::BUS, "bus", 2.5, 12.0);

static const Model *CAR = new Model(Model::CAR, "car", 1.9, 5.1);

static const Model *PEDESTRIAN = new Model(Model::PEDESTRIAN, "pedestrian", 0.0, 0.0);

static const Model *UNKNOWN = new Model(Model::UNKNOWN, "unknown", 0.0, 0.0);

static std::vector<const Model*> VEHICLES = {BIKE, BUS, CAR};


Model::Model(ID id, const std::string &name, double width, double length):
	id(id),
	name(name),
	width(width),
	length(length)
{
	// Nothing to do.
}


std::ostream &operator << (std::ostream &out, const Model &model)
{
	switch (model.id)
	{
		case Model::BIKE       : out << "Bike, "; break;
		case Model::BUS        : out << "Bus, "; break;
		case Model::CAR        : out << "Car, "; break;
		case Model::PEDESTRIAN : out << "Pedestrian, "; break;
		case Model::UNKNOWN    : out << "Unknown, "; break;
	}

	return out << "(width, length) = " << std::setprecision(2) << '(' << model.width << ", " << model.length << ')';
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


void Node::add_child(Node &child)
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


void Node::add_sibling(Node &node)
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
	selected_latch(0),
	nodes(NULL),
	i_0(0),
	i_n(0)
{
	// Nothing to do.
}


Cluster::Cluster(const Segment &segment, Node::S &nodes):
	selected_latch(0),
	nodes(&nodes),
	i_0(nodes.size())
{
	switch (segment.shape)
	{
		case I_SHAPED   : init_I_SHAPED(segment);   break;
		case L_SHAPED   : init_L_SHAPED(segment);   break;
		case POINT_MASS : init_POINT_MASS(segment); break;
	}

	i_n = nodes.size();
}


void Cluster::add_node(double timestamp, const Pose &origin, const Pose &local, const Model *model)
{
	nodes->emplace_back(timestamp, origin, local, model, &selected_latch);
	Node &node = nodes->back();
	for (auto i = nodes->rbegin() + 1, n = nodes->rend(); i != n; i++)
		node.add_sibling(*i);
}


inline std::vector<PointXY> find_side_points(double length, double theta, const PointXY &first_point, const PointXY &last_point)
{
	std::vector<PointXY> points;
	points.reserve(3); // Pre-allocate space for the points.

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
	nodes->emplace_back(segment.timestamp, segment.origin, Pose(segment.centroid()), PEDESTRIAN, &selected_latch);	
}


Node &Cluster::operator [] (size_t index)
{
	return (*nodes)[i_0 + index];
}


const Node &Cluster::operator [] (size_t index) const
{
	return (*nodes)[i_0 + index];
}


bool Cluster::selected() const
{
	return (selected_latch > 0);
}


size_t Cluster::size() const
{
	return i_n - i_0;
}


Subgraph::Subgraph():
	timestamp(0)
{
	// Nothing to do.
}


Subgraph::Subgraph(const Reading &reading)
{
	assign(reading);
}


Subgraph::Subgraph(Subgraph &that, const Reading &reading)
{
	assign(that, reading);
}


void Subgraph::assign(const Reading &reading)
{
	clear();

	timestamp = reading.timestamp;
	Segment::S segments = split_segments(reading);
	for (auto segment = segments.begin(), n = segments.end(); segment != n; ++segment)
		emplace_back(*segment, nodes);
}


void Subgraph::assign(Subgraph &that, const Reading &reading)
{
	assign(reading);
	for (auto parent = that.nodes.begin(), m = that.nodes.end(); parent != m; ++parent)
		for (auto child = this->nodes.begin(), n = this->nodes.end(); child != n; ++child)
			parent->add_child(*child);
}


void Subgraph::clear()
{
	Cluster::S::clear();
	nodes.clear();
}


Graph::Graph():
	offset(0)
{
	subgraphs.reserve(T);
}


Subgraph &Graph::operator [] (size_t index)
{
	return subgraphs[(offset + index) % T];
}


const Subgraph &Graph::operator [] (size_t index) const
{
	return subgraphs[(offset + index) % T];
}


Subgraph &Graph::back()
{
	return subgraphs[offset > 0 ? offset - 1 : size() - 1];
}


const Subgraph &Graph::back() const
{
	return subgraphs[offset > 0 ? offset - 1 : size() - 1];
}


Subgraph &Graph::front()
{
	return subgraphs[offset];
}


const Subgraph &Graph::front() const
{
	return subgraphs[offset];
}


void Graph::update(const Reading &reading)
{
	if (subgraphs.size() == T)
	{
		// Replace the oldest subgraph with a new one.
		front().assign(back(), reading);

		// Update the offset index, wrapping it around if necessary.
		offset = (offset + 1) % T;

		// Remove all dangling parent node pointers from the now-oldest subgraph.
		Node::S &nodes = front().nodes;
		for (auto node = nodes.begin(), n = nodes.end(); node != n; ++node)
			node->parents.clear();
	}
	else if (size() > 0)
		subgraphs.emplace_back(subgraphs.back(), reading);
	else
		subgraphs.emplace_back(reading);
}


size_t Graph::size() const
{
	return subgraphs.size();
}


} // namespace virtual_scan
