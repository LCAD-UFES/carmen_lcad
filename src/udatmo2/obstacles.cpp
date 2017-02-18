#include "obstacles.h"

#include <cstring>

namespace udatmo
{

Obstacles::Obstacles(double timestamp):
	obstacles_(NUM_OBSTACLES)
{
	this->timestamp = timestamp;
	for (int i = 0; i < NUM_OBSTACLES; i++)
	{
		carmen_datmo_moving_obstacle &obstacle = obstacles_[0];
		memset(&obstacle, 0, sizeof(carmen_datmo_moving_obstacle));
		obstacle.rddf_index = -1;
	}
}

carmen_datmo_moving_obstacle &Obstacles::operator [] (int index)
{
	return obstacles_[index];
}

size_t Obstacles::size() const
{
	return obstacles_.size();
}

} // namespace udatmo
