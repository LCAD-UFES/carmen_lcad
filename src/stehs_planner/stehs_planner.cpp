#include "stehs_planner.hpp"


// constructor
StehsPlanner::StehsPlanner() {}


/*std::vector<CircleNode>
StehsPlanner::Expand(CircleNodePtr current)
{
	//stehs_planner.goal;

}*/

double
StehsPlanner::Distance(const carmen_ackerman_traj_point_t &a, const carmen_ackerman_traj_point_t &b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	double distance = sqrt(dx * dx + dy * dy);

	return (distance);
}


carmen_point_t
traj_to_point_t(carmen_ackerman_traj_point_t traj)
{
	carmen_point_t point;

	point.x = traj.x;
	point.y = traj.y;
	point.theta = traj.theta;

	return (point);
}


double
StehsPlanner::ObstacleDistance(const carmen_ackerman_traj_point_t &point)
{
	return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(traj_to_point_t(point), distance_map)); // TODO verificar se esta funcao foi modificada
}


void
StehsPlanner::SpaceExploration()
{

	// create the start circle node
	CircleNodePtr start_node =  new CircleNode(start.x, start.y, ObstacleDistance(start), 0.0, Distance(start, goal), nullptr);

	// create the goal circle node
	/*CircleNodePtr goal_node = new CircleNode(goal.x, goal.y, ObstacleDistance(goal), DBL_MAX, DBL_MAX, nullptr);

	// create the priority queue
	//std::priority_queue<CircleNodePtr, CircleNodePtrComparator> open_set;

	std::vector<CircleNodePtr> closed_set;

	//open_set.push(start_node);

	while (!open_set.empty())
	{
		// get the circle wich is the closest to the goal node
		CircleNodePtr current = open_set.top();
		open_set.pop();

		if (goal_node->f < current->f)
		{
			BuildCirclePath(goal_node);
			return;
		}
		else if (!Exist(current, closed_set))
		{
			std::vector<CircleNode> children = Expand(current);

			// iterator
			std::vector<CircleNodePtr>::iterator it = children->begin();
			std::vector<CircleNodePtr>::iterator end = children->end();

			while (it != end)
			{
				open_set.push(*it);

				// update the iterator
				++it;

			}

			if (current->circle.Overlap(goal_node->circle))
			{

				if (current->f < goal_node->g)
				{
					goal_node->g = current->f;
					goal_node->parent = current;
				}
			}

			closed_set.push_back(current);

		}
	}*/
}

//
//void
//StehsPlanner::SpaceTimeExploration() {}
//
////
//void
//StehsPlanner::HeuristicSearch() {}
//
//
////
//std::list<carmen_ackerman_motion_command_t>
//BuildPath();
