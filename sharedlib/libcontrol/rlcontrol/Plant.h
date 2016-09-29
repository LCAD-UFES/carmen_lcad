
#ifndef PLANT_H_
#define PLANT_H_

#include <map>
#include <deque>
#include <list>
#include <cmath>
#include "State.h"

using namespace std;


const double LATENCY = 1.0; // sec
const double LATENCY_SIGMA = 0.0001;
const double EFFORT_TO_ACCELERATION = 0.7;
const double VELOCITY_STD = 0.5;
const double SIN_PERIOD_DURATION = 1.0 / 10.0;
const double SAMPLING_TIME = 1.0 / 40.0;
const double DELTA_ANGLE = (2 * M_PI * SAMPLING_TIME) / SIN_PERIOD_DURATION;
const double INITIAL_ANGLE = -4 * M_PI;


class Plant
{
	int _num_samples_per_iteration;
	double _size_future_profile;
	double _current_angle;
	double _future_angle;
	double _current_v;
	double _current_time;
	double _difference_between_expected_and_measured_v;

	deque<double> _desired_v;
	deque<double> _measured_v;

	deque<double> _commands;
	deque<double> _commands_timestamps;

	bool _new_experiment_started;
	void _updateState();

public:
	Plant(double size_future_profile);
	State getState();
	double act(double acceleration);
	bool newExperimentStarted();
};



#endif
