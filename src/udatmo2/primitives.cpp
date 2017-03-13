#include "primitives.h"

std::ostream &operator << (std::ostream &out, const carmen_position_t &pose)
{
	return out << "(" << pose.x << ", " << pose.y << ")";
}

std::ostream &operator << (std::ostream &out, const carmen_point_t &pose)
{
	return out << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")";
}

std::ostream &operator << (std::ostream &out, const carmen_ackerman_traj_point_t &pose)
{
	return out << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ", " << pose.v << ", " << pose.phi << ")";
}
