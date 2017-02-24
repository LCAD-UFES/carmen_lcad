#include "obstacle.h"

#define MOVING_OBSTACLES_OBSERVATIONS 40

namespace udatmo
{

Obstacle::Obstacle():
	index(-1)
{
	pose.x = 0;
	pose.y = 0;
	pose.theta = 0;
}

Obstacle::Obstacle(const carmen_ackerman_traj_point_t &robot_pose, Observation &observation)
{
	update(observation);

	// Pose attributes must be assigned after update() call to avoid being overwritten.
	pose.theta = robot_pose.theta;
	pose.v = robot_pose.v;
	pose.phi = 0;
}


void Obstacle::update(const carmen_ackerman_traj_point_t &robot_pose, Observations &observations)
{
	if (this->observations.size() == 0)
		return;

	const Observation &o1 = this->observations.front();
	const Observation &o2 = observations.front();
	double t = truncate(o2.timestamp - o1.timestamp, 0.01, 0.2);
	double r = 1.5 * pose.v * t;

	for (Observations::iterator i = observations.begin(), n = observations.end(); i != n; ++i)
	{
		Observation &observation = *i;
		if (distance(pose, observation.position) <= r)
		{
			update(observation);
			update(robot_pose);
			observations.erase(i);
			return;
		}
	}

	if (this->observations.size() > 1)
		this->observations.pop_back();

	misses++;

	pose.x += pose.v * t * cos(pose.theta);
	pose.y += pose.v * t * sin(pose.theta);
}


void Obstacle::update(const Observation &observation)
{
	misses = 0;
	observations.push_front(observation);
	while (observations.size() > MOVING_OBSTACLES_OBSERVATIONS)
		observations.pop_back();

	const carmen_position_t &position = observation.position;
	index = observation.index;
	pose.x = position.x;
	pose.y = position.y;
}


void Obstacle::update(const carmen_ackerman_traj_point_t &robot_pose)
{
	double v = 0.0;
	double theta = 0.0;
	double count = 0.0;
	for (int i = observations.size() - 2; i >= 0 ; i--)
	{
		// Observations are inserted at the front of the sequence,
		// so o1 is "older" (i.e. smaller timestamp) than o2.
		const Observation &o1 = observations[i + 1];
		const Observation &o2 = observations[i];

		// distance in the direction of the robot: https://en.wikipedia.org/wiki/Vector_projection
		double d = distance(o1, o2) * cos(angle(o1, o2) - robot_pose.theta);
		double t = o2.timestamp - o1.timestamp;

		double vt = -1.0; // invalid v
		if (0.01 < t && t < 0.2)
			vt = d / t;

		if (vt > 60.0)
			vt = -1.0;

		if (vt > -0.00001)
		{
			v += vt;
			theta += angle(o1, o2);
			count += 1.0;
		}
	}

	if (count > 0)
	{
		pose.theta = theta / count;
		pose.v = v / count;
	}
}


double Obstacle::timestamp() const
{
	return (observations.size() > 0 ? observations[0].timestamp : -1);
}


bool Obstacle::valid() const
{
	return (observations.size() > 1 && misses < 3);
}

} // namespace udatmo
