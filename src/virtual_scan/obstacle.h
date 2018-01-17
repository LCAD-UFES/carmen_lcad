#ifndef VIRTUAL_SCAN_OBSTACLE_H
#define VIRTUAL_SCAN_OBSTACLE_H

#include "neighborhood_graph.h"
#include "point.h"
#include "rectangle.h"

#include <deque>

namespace virtual_scan
{

/**
 * @pose An obstacle's pose at a given moment, relative to the global reference frame.
 */
struct ObstaclePose
{
	/** @brief Obstacle pose sequence type. */
	typedef std::deque<ObstaclePose> S;

	/** @brief Pointer to the neighborhood graph node related to this pose. */
	Node *node;

	/** @brief Obstacle pose relative to the global reference frame. */
	Pose global;

	/** @brief Obsacle pose relative to the node's local frame. */
	Pose local;
	
	/**
	 * @brief Default constructor.
	 */
	ObstaclePose();

	/**
	 * @brief Create a new pose related to the given node.
	 */
	ObstaclePose(Node *node);

	/**
	 * @brief Class destructor.
	 */
	~ObstaclePose();

	/**
	 * @brief Project a point (given relative to `origin`) to this pose's coordinate system.
	 */
	PointXY project_local(const Pose &origin, const PointXY &point) const;
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
	 * @brief Create a new view for the given obstacle from the view of its originating sensor reading.
	 */
	ObstacleView(const ObstaclePose &pose);

	/**
	 * @brief Create a new view for the given obstacle from given coordinate system. 
	 */
	ObstacleView(const Pose &origin, const ObstaclePose &pose);

	/**
	 * @brief Defines a ordering of views by the beginning of the angle range.
	 */
	bool operator < (const ObstacleView &that) const;

	/**
	 * @brief Checks whether a sensor reading is "before" the view in the sensor's field of view.
	 */
	bool operator < (const PointOD &point) const;

	/**
	 * @brief Checks whether a sensor reading is "after" the view in the sensor's field of view.
	 */
	bool operator > (const PointOD &point) const;

	/**
	 * @brief Compute the distance between this view and a sensor reading point.
	 */
	double distance(const PointXY &point) const;
};

/**
 * @brief Print an obstacle pose to an output stream.
 */
std::ostream &operator << (std::ostream &out, const ObstaclePose &pose);

} // namespace virtual_scan

#endif
