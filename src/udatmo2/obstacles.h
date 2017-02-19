#ifndef UDATMO_OBSTACLES_H
#define UDATMO_OBSTACLES_H

#include "udatmo_messages.h"

#include <vector>

#define NUM_OBSTACLES 1

namespace udatmo
{

class Obstacles
{
	std::vector<carmen_datmo_moving_obstacle> obstacles_;

public:
	double timestamp;

	Obstacles(double timestamp);

	carmen_datmo_moving_obstacle &operator [] (int index);

	size_t size() const;
};

} // namespace udatmo

#endif
