#ifndef DATOMIC_MOVING_OBJECT
#define DATOMIC_MOVING_OBJECT

#include <carmen/carmen.h>

namespace udatmo
{

struct Obstacle
{
	bool valid;

	int index;

	int rddf_pose_index;

	carmen_robot_and_trailer_traj_point_t pose;

	carmen_robot_and_trailer_traj_point_t car_pose;

	carmen_point_t rddf_front_car_pose;

	double timestamp;
};

} // namespace udatmo

#endif
