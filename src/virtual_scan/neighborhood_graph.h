#ifndef VIRTUAL_SCAN_NEIGHBORHOOD_GRAPH_H
#define VIRTUAL_SCAN_NEIGHBORHOOD_GRAPH_H

#include "point.h"
#include "reading.h"
#include "segment.h"

// Double-ended queues have better performance than vectors for end-insertion,
// while still allowing random access. This makes them preferable as the default
// collection for graph structures, which are constructed dynamically and seldom
// accessed randomly (iterated access being the most common case).
#include <deque>

// Still, in some cases vectors remain the best option, particularly in the
// Graph class where a circular buffer is implemented.
#include <vector>

#include <iostream>

namespace virtual_scan
{

/**
 * @brief Obstacle model.
 */
struct Model
{
	/** @brief Model identifier. */
	enum ID
	{
		BIKE,
		BUS,
		CAR,
		PEDESTRIAN,
		UNKNOWN
	};

	/** @brief Model identifier. */
	ID id;

	/** @brief Model name. */
	std::string name;

	/** @brief Model width. */
	double width;

	/** @brief Model length. */
	double length;

	/**
	 * @brief Create a new model of given settings.
	 */
	Model(ID id, const std::string &name, double width, double length);
};

/**
 * Write a model to an output stream.
 */
std::ostream &operator << (std::ostream &out, const Model &model);

/**
 * @brief A single node in the graph, representing a particular hypothesis.
 */
struct Node
{
	/** @brief Node sequence type. */
	typedef std::deque<Node> S;

	/** @brief Type for a sequence of edges connecting this node to others. */
	typedef std::deque<Node*> Edges;

	/** @brief Time when this node was generated. */
	double timestamp;

	/** @brief Model for the obstacle hypothesis represented in this node. */
	const Model *model;

	/** @brief Obstacle hypothesis pose relative to the originating reading and the global reference frame. */
	struct {Pose global; Pose local;} pose;

	/** @brief Siblings of this node (i.e. other nodes constructed from the same reading segment). */
	Edges siblings;

	/** @brief Children of this node, representing possible future configurations of the underlying obstacle. */
	Edges children;

	/** @brief Parents of this node, representing possible past configurations of the underlying obstacle. */
	Edges parents;

	/**
	 * @brief Default constructor.
	 */
	Node();

	/**
	 * @brief Create a new node of given settings.
	 */
	Node(double timestamp, const Pose &origin, const Pose &local, const Model *model, int *selected);

	/**
	 * Make the given node a child of this one.
	 */
	void add_child(Node &child);

	/**
	 * Make the given node a sibling of this one.
	 */
	void add_sibling(Node &node);

	/**
	 * @brief Mark this node (and its whole parent cluster) as selected as a track pose.
	 */
	void select();

	/**
	 * @brief Unmark this node (and possibly its whole parent cluster) as selected as a track pose.
	 */
	void deselect();

	/**
	 * @brief Return whether this node (or another one in the same cluster) is currently part of some track.
	 */
	bool selected() const;

private:
	/** @brief Pointer to the selection counter for the cluster this node belongs to. */
	int *selected_latch;
};

/**
 * @brief A collection of nodes which are all siblings of each other.
 */
struct Cluster
{
	/** @brief Cluster sequence type. */
	typedef std::deque<Cluster> S;

	/** @brief Alias to the `Node` type, needed to make the `Cluster` type work with some template functions (notably `random_choose`). */
	typedef Node value_type;

	/**
	 * @brief Default constrcutor.
	 */
	Cluster();

	/**
	 * @brief Create a new node cluster from the given reading segment.
	 *
	 * Created nodes are stored in the given sequence.
	 */
	Cluster(const Segment &segment, Node::S &nodes);

	/**
	 * @brief Return a reference to the node at the given index.
	 */
	Node &operator [] (size_t index);

	/**
	 * @brief Return a reference to the node at the given index.
	 */
	const Node &operator [] (size_t index) const;

	/**
	 * @brief Return whether the cluster is selected.
	 */
	bool selected() const;

	/**
	 * @brief Return the number of nodes in this cluster.
	 */
	size_t size() const;

private:
	/** @brief Indicates whether one node in this cluster is currently being used as an obstacle hypothesys. */
	int selected_latch;

	/** @brief Pointer to the sequence storing the nodes that are part of this cluster. */
	Node::S *nodes;

	/** @begin Index to the first node of this cluster. */
	size_t i_0;

	/** @begin Index to the position just past the last node of this cluster. */
	size_t i_n;

	/**
	 * @brief Add a new node to this cluster, connecting it to any pre-existing nodes.
	 *
	 * The new node is created at the given collection.
	 */
	void add_node(double timestamp, const Pose &origin, const Pose &local, const Model *model);

	/**
	 * @brief Initialize this cluster with nodes created from an I-shaped segment.
	 */
	void init_I_SHAPED(const Segment &segment);

	/**
	 * @brief Initialize this cluster with nodes created from an L-shaped segment.
	 */
	void init_L_SHAPED(const Segment &segment);

	/**
	 * @brief Initialize this cluster with nodes created from a point-mass segment.
	 */
	void init_POINT_MASS(const Segment &segment);
};

/**
 * @brief Collection of nodes created all at the same time point.
 */
struct Subgraph: Cluster::S
{
	/** @brief Subgraph sequence type. */
	typedef std::vector<Subgraph> S;


	/** @brief Time of the reading used to generate this subgraph. */
	double timestamp;

	/** @brief Sequence of nodes contained in this subgraph. */
	Node::S nodes;

	/**
	 * @brief Default constructor.
	 */
	Subgraph();

	/**
	 * @brief Create a new subgraph from the given reading.
	 */
	Subgraph(const Reading &reading);

	/**
	 * @brief Create a new subgraph from the given reading, connected to the given subgraph.
	 */
	Subgraph(Subgraph &that, const Reading &reading);

	/**
	 * @brief Update this subgraph with data from the given reading.
	 */
	void assign(const Reading &reading);

	/**
	 * @brief Update this subgraph with data from the given reading and connect it to the given subgraph.
	 */
	void assign(Subgraph &that, const Reading &reading);

	/**
	 * @brief Remove all clusters and nodes from this subgraph.
	 */
	void clear();
};

/**
 * @brief A neighborhood graph representing possible explanation of sensor reading segments.
 */
struct Graph
{
	/** @brief Value type alias, needed to make this type work with some template functions (notably `random_choose`). */
	typedef Subgraph value_type;

	/**
	 * @brief Default constructor.
	 */
	Graph();

	/**
	 * @brief Return the subgraph at the given index.
	 */
	Subgraph &operator [] (size_t index);

	/**
	 * @brief Return the subgraph at the given index.
	 */
	const Subgraph &operator [] (size_t index) const;

	/**
	 * @brief Return the most recent subgraph.
	 */
	Subgraph &back();

	/**
	 * @brief Return the most recent subgraph.
	 */
	const Subgraph &back() const;

	/**
	 * @brief Return the oldest subgraph.
	 */
	Subgraph &front();

	/**
	 * @brief Return the oldest subgraph.
	 */
	const Subgraph &front() const;

	/**
	 * @brief Update this graph with data from the given sensor reading.
	 */
	void update(const Reading &reading);

	/**
	 * @brief Return the number of subgraphs.
	 */
	size_t size() const;

private:
	/** @brief Sequence of subgraphs. */
	Subgraph::S subgraphs;

	/**
	 * @brief Index offset pointing to the oldest subgraph.
	 *
	 * When the subgraph sequence is full, the subgraph at this position is updated
	 * to represent the most recent state, and then incremented (and possibly wrapped
	 * around) to point to the now-oldest subgraph.
	 */
	size_t offset;
};

} // namespace virtual_scan

#endif
