#ifndef DATOMIC_MOVING_OBJECT
#define DATOMIC_MOVING_OBJECT

#include <carmen/carmen.h>

namespace udatmo
{

struct Obstacle
{
	int index;

	carmen_ackerman_traj_point_t pose;

	carmen_ackerman_traj_point_t car_pose;

	double timestamp;

	Obstacle()
	{
		index = 0;
		memset(&pose, 0, sizeof(carmen_ackerman_traj_point_t));
		memset(&car_pose, 0, sizeof(carmen_ackerman_traj_point_t));
		timestamp = 0;
	}

	Obstacle(int goal_index,
			 carmen_position_t obstacle,
			 carmen_ackerman_traj_point_t current_pose,
			 double rddf_timestamp)
	{
		pose.x = obstacle.x;
		pose.y = obstacle.y;
		pose.theta = 0;
		pose.v = 0;
		pose.phi = 0;
		car_pose = current_pose;
		index = goal_index;
		timestamp = rddf_timestamp;
	}
};

} // namespace udatmo

#endif
