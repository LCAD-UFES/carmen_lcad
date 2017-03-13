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
	// If necessary, remove oldest observation to make room for the latest one.
	while (track.size() >= MOVING_OBSTACLES_OBSERVATIONS)
		track.pop_back();

	track.push_front(observation);
	CARMEN_LOG(trace, "Obstacle track size: " << track.size());

	index = observation.index;
	pose.x = observation.position.x;
	pose.y = observation.position.y;
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

		double dt = o2.timestamp - o1.timestamp;
		if (dt < 0.00001)
			continue;

		v += distance(o1, o2) / dt;
		theta += angle(o1, o2);
		count += 1.0;
	}

	if (count > 0)
	{
		pose.theta = theta / count;
		pose.v = v / count;
	}
	else
		pose.v = 0;

	CARMEN_LOG(trace, "Obstacle estimates over " << count << " pairing(s): v = " << pose.v << ", theta = " << pose.theta);
}


double Obstacle::timestamp() const
{
	return (track.size() > 0 ? track[0].timestamp : -1);
}


} // namespace udatmo
