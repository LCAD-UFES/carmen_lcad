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
	pose.theta = 0;
	pose.v = 0;
	pose.phi = 0;
}


Obstacle::Obstacle(const Observation &observation)
{
	update(observation);
	pose.theta = 0;
	pose.v = 0;
	pose.phi = 0;
}


void Obstacle::update(const Observation &observation)
{
	// Only add observation to track if it's reasonably apart from latest observation.
	if (track.size() > 0 && distance(observation, track.front()) < 0.01)
	{
		// Also, if this obstacle is tracking, remove oldest observation.
		if (track.size() > 1)
			track.pop_back();

		return;
	}

	track.push_front(observation);
	while (track.size() > MOVING_OBSTACLES_OBSERVATIONS)
		track.pop_back();

	const carmen_position_t &position = observation.position;
	index = observation.index;
	pose.x = position.x;
	pose.y = position.y;
	updateMovement();
}


void Obstacle::updateMovement()
{
	double v = 0.0;
	double theta = 0.0;
	double count = 0.0;
	for (int i = track.size() - 2; i >= 0 ; i--)
	{
		// Observations are inserted at the front of the sequence,
		// so o1 is "older" (i.e. smaller timestamp) than o2.
		const Observation &o1 = track[i + 1];
		const Observation &o2 = track[i];

		double vt = 0.0; // invalid v

		double t = o2.timestamp - o1.timestamp;
		if (0.01 < t && t < 0.2)
			vt = distance(o1, o2) / t;

		if (vt > 60.0)
			vt = 0.0;

		if (vt > 0.01)
		{
			v += vt;
			theta += angle(o1, o2);
			count += 1.0;
		}
	}

	CARMEN_LOG(trace, "Estimated speed (over " << count << " readings): " << v / count);

	if (count > 0)
	{
		pose.theta = theta / count;
		pose.v = v / count;
	}
}


double Obstacle::timestamp() const
{
	return (track.size() > 0 ? track[0].timestamp : -1);
}


bool Obstacle::tracking() const
{
	return (track.size() > 1);
}


} // namespace udatmo
