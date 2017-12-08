#ifndef VIRTUAL_SCAN_OBSTACLE_H
#define VIRTUAL_SCAN_OBSTACLE_H

#include "rectangle.h"
#include "virtual_scan.h"
#include "virtual_scan_neighborhood_graph.h"

namespace virtual_scan
{

/**
 * @pose An obstacle's pose at a given moment, relative to the global reference frame.
 */
struct ObstaclePose
{
	/** @brief Pointer to the neighborhood graph node related to this pose. */
	virtual_scan_graph_node_t *node;
	
	/** @brief Position of this pose in the global x-axis. */
	double x;

	/** @brief Position of this pose in the global y-axis. */
	double y;

	/** @brief Angle of this pose relative to the global x-axis */
	double theta;

	/**
	 * @brief Default constructor.
	 */
	ObstaclePose();

	/**
	 * @brief Create a new pose related to the given node.
	 */
	ObstaclePose(virtual_scan_graph_node_t *graph_node);

	/**
	 * @brief Class destructor.
	 */
	~ObstaclePose();
};

/**
 * @brief A representation of an obstacle pose from a given point of view.
 */
struct ObstacleView: private Rectangle
{
	/** @brief Sensor field of view range obstructed by the obstacle, as a pair of angles. */
	std::pair<double, double> range;

	/**
	 * @brief Default constructor.
	 */
	ObstacleView();

	/**
	 * @brief Create a new view for the given obstacle from the given point of view.
	 */
	ObstacleView(const ObstaclePose &pose, const carmen_point_t &origin);

	/**
	 * @brief Defines a ordering of views by the beginning of the angle range.
	 */
	bool operator < (const ObstacleView &that) const;

	/**
	 * @brief Checks whether a sensor reading is "before" the view in the sensor's field of view.
	 */
	bool operator < (const carmen_point_t &point) const;

	/**
	 * @brief Checks whether a sensor reading is "after" the view in the sensor's field of view.
	 */
	bool operator > (const carmen_point_t &point) const;

	/**
	 * @brief Compute the probability that a sensor reading is explained by this obstacle view.
	 */
	double distance(const carmen_point_t &point) const;
};

} // namespace virtual_scan

#endif
