#ifndef DATOMIC_MOVING_OBJECT
#define DATOMIC_MOVING_OBJECT

#include <carmen/carmen.h>

namespace udatmo
{

struct Obstacle
{
	bool valid;

	carmen_ackerman_traj_point_t pose;

	carmen_ackerman_traj_point_t car_pose;

	double timestamp;
};

} // namespace udatmo

#endif
