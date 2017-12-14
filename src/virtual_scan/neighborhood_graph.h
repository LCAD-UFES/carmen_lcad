#ifndef VIRTUAL_SCAN_NEIGHBORHOOD_GRAPH_H
#define VIRTUAL_SCAN_NEIGHBORHOOD_GRAPH_H

#include "point.h"
#include "reading.h"
#include "segment.h"

#include <deque>
#include <vector>

namespace virtual_scan
{

/**
 * @brief Obstacle model.
 */
struct Model
{
	/** @brief Model width. */
	double width;

	/** @brief Model length. */
	double length;

	/**
	 * @brief Create a new model of given settings.
	 */
	Model(double width, double length);
};

/**
 * @brief A single node in the graph, representing a particular hypothesis.
 */
struct Node
{
	/** @brief Node sequence type. */
	typedef std::vector<Node> S;

	/** @brief Type for a sequence of edges connecting this node to others. */
	typedef std::vector<Node*> Edges;

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
	void adopt(Node &child);

	/**
	 * Make the given node a sibling of this one.
	 */
	void connect(Node &node);

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
struct Cluster: Node::S
{
	/** @brief Cluster sequence type. */
	typedef std::vector<Cluster> S;

	/**
	 * @brief Default constrcutor.
	 */
	Cluster();

	/**
	 * @brief Create a new node cluster from the given reading segment.
	 */
	Cluster(const Segment &segment);

	/**
	 * @brief Return whether the cluster is selected.
	 */
	bool selected() const;

private:
	/** @brief Indicates whether one node in this cluster is currently being used as an obstacle hypothesys. */
	int selected_latch;

	/**
	 * @brief Add a new node to this cluster, connecting it to any pre-existing nodes.
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

struct Subgraph: Cluster::S
{
	/** @brief Subgraph sequence type. */
	typedef std::deque<Subgraph> S;

	/**
	 * @brief Iterator over nodes in this subgraph.
	 */
	class iterator_nodes
	{
		/** @brief Iterator over clusters. */
		Cluster::S::iterator cluster;

		/** @brief Iterator over nodes in a cluster. */
		Node::S::iterator node;

	public:
		/**
		 * @brief Default constructor.
		 */
		iterator_nodes();

		/**
		 * @brief Create a new node iterator started at the given position.
		 */
		iterator_nodes(const Cluster::S::iterator &cluster, const Node::S::iterator &node);

		/**
		 * @brief Check whether this iterator is different from another.
		 */
		bool operator != (const iterator_nodes &that) const;

		/**
		 * @brief Moves this iterator one element forward.
		 */
		iterator_nodes &operator ++ ();

		/**
		 * @brief Return a reference to the node currently pointed by this iterator.
		 */
		Node &operator * ();

		/**
		 * @brief Return a pointer to the node currently pointed by this iterator.
		 */
		Node *operator -> ();
	};

	/** @brief Time of the reading used to generate this subgraph. */
	double timestamp;

	/**
	 * @brief Default constructor.
	 */
	Subgraph();

	/**
	 * @brief Create a new subgraph from the given reading.
	 */
	Subgraph(const Reading &reading);

	/**
	 * @brief Create a new subgraph from the given reading, connected to the given parent subgraph.
	 */
	Subgraph(Subgraph &parent, const Reading &reading);

	/**
	 * @brief Return an iterator at the beggining of this reading.
	 */
	iterator_nodes begin_nodes();

	/**
	 * @brief Return the past-the-end iterator.
	 */
	iterator_nodes end_nodes();
};

/**
 * @brief A neighborhood graph representing possible explanation of sensor reading segments.
 */
struct Graph: Subgraph::S
{
	/**
	 * @brief Update this graph with data from the given sensor reading.
	 */
	void update(const Reading &reading);
};

} // namespace virtual_scan

#endif
