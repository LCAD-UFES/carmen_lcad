#include "obstacle.h"

#include "logging.h"


#define MOVING_OBSTACLES_OBSERVATIONS 40


namespace udatmo
{


Obstacle::Obstacle():
	index(-1)
{
	pose.x = 0;
	pose.y = 0;
	pose.theta = nan("");
	pose.v = nan("");
	pose.phi = 0;
}


Obstacle::Obstacle(const Observation &observation)
{
	update(observation);
	pose.theta = nan("");
	pose.v = nan("");
	pose.phi = 0;
}


void Obstacle::update(const Observation &observation)
{
	track.push_front(observation);
	CARMEN_LOG(trace, "Obstacle track size: " << track.size());

	index = observation.index;
	pose.x = observation.position.x;
	pose.y = observation.position.y;
	updateMovement();
}


void Obstacle::updateMovement()
{
	static double l = 0.9;

	// Latest observation (higher timestamp).
	const Observation &o2 = track.front();

	// Drop previous observations until the time difference threshold is reached.
	for (double t2 = o2.timestamp; t2 - track.back().timestamp > 1.0;)
		track.pop_back();

	// Previous observation (lower timestamp).
	const Observation &o1 = track.back();

	// Compute speed and angle between observations.
	double dt = o2.timestamp - o1.timestamp;
	double vt = distance(o1, o2) / dt;
	double theta = angle(o1, o2);

	if (isnan(pose.v))
	{
		pose.v = vt;
		pose.theta = theta;
	}
	else
	{
		pose.v     = l * pose.v     + (1.0 - l) * vt;
		pose.theta = l * pose.theta + (1.0 - l) * theta;
	}
}


double Obstacle::timestamp() const
{
	return (track.size() > 0 ? track[0].timestamp : -1);
}


} // namespace udatmo
