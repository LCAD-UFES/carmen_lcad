#ifndef DATOMIC_MOVING_OBJECT
#define DATOMIC_MOVING_OBJECT

#include <carmen/carmen.h>

namespace udatmo
{

struct Observation
{
	carmen_ackerman_traj_point_t pose;

	double timestamp;

	Observation()
	{
		memset(&pose, 0, sizeof(carmen_ackerman_traj_point_t));
		timestamp = 0;
	}

	Observation(carmen_position_t obstacle, double timestamp)
	{
		pose.x = obstacle.x;
		pose.y = obstacle.y;
		pose.theta = 0;
		pose.v = 0;
		pose.phi = 0;
		this->timestamp = timestamp;
	}
};

} // namespace udatmo

#endif
