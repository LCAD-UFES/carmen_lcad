#include "Plant.h"
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include "Util.h"


void
Plant::_updateState()
{
	double command_timestamp, command;

	// *********************************************************
	// remove commands that do not affect the robot anymore
	// *********************************************************

//	li = _commands.begin();
//	lj = _commands_timestamps.begin();
//
//	while (li != _commands.end() && lj != _commands_timestamps.end())
//	{
//		command = *li;
//		command_timestamp = *lj;
//
//		/* +SAMPLING_TIME to keep the command for 1 iteration after the latency time */
//		if (_current_time - command_timestamp > (LATENCY + SAMPLING_TIME))
//		{
//			li = _commands.erase(li);
//			lj = _commands_timestamps.erase(lj);
//		}
//		else
//		{
//			li++;
//			lj++;
//		}
//	}

	// *********************************************************
	// apply the commands
	// *********************************************************

	_current_v = 0;

	double sum_of_multipliers = 0;
	int i = 0;

	while (i < _commands.size())
	{
		command = _commands[i];
		command_timestamp = _commands_timestamps[i];

		double time_diff = (_current_time - command_timestamp);

		if (time_diff > LATENCY)
		{
			i++;
			continue;
		}

		// sigmoidal latency
		double latency_multiplier = exp(-pow(time_diff - LATENCY, 2) / LATENCY_SIGMA);

		if (latency_multiplier < -0.001 || latency_multiplier > 1.001)
			exit(printf("PROBLEM: invalid latency_multiplier: %lf current_timestamp: %lf command_timestamp: %lf LATENCY: %lf\n",
				latency_multiplier, _current_time, command_timestamp, LATENCY));

		_current_v += latency_multiplier * command;
		sum_of_multipliers += latency_multiplier;

		if (_current_v > 2.0)
			_current_v = 2.0;
		if (_current_v < -2.0)
			_current_v = -2.0;

		i++;
	}

	_current_v /= sum_of_multipliers;

	double desired_v, future_desired_v;

	if (_current_angle < 0)
		desired_v = 0;
	else
		desired_v = sin(_current_angle);

	_difference_between_expected_and_measured_v = fabs(_current_v - desired_v);

	// *********************************************************
	// update state
	// *********************************************************

	_current_angle += DELTA_ANGLE;
	_future_angle += DELTA_ANGLE;

	if (_future_angle > 2 * M_PI)
		_future_angle = INITIAL_ANGLE;

	if (_current_angle > (2 * M_PI))
	{
		_current_angle = INITIAL_ANGLE;
		_new_experiment_started = true;
	}
	else
		_new_experiment_started = false;

	if (_future_angle < 0)
		future_desired_v = 0;
	else
		future_desired_v = sin(_future_angle);

//	fprintf(stderr, "_future_angle: %.2lf future_desired_v: %.2lf\n", _future_angle, future_desired_v);
//	getchar();

	_measured_v.push_back(_current_v);
	_desired_v.push_back(future_desired_v);

	_measured_v.pop_front();
	_desired_v.pop_front();

	_current_time += SAMPLING_TIME;
}


Plant::Plant(double size_future_profile)
{
	_current_v = 0;
	_current_angle = INITIAL_ANGLE;
	_new_experiment_started = false;
	_size_future_profile = size_future_profile;
	_difference_between_expected_and_measured_v = 0;
	_current_time = 0;

	_num_samples_per_iteration = (int) ((double) size_future_profile / (double) SAMPLING_TIME);
	_future_angle = _current_angle;

	for (int i = 0; i < _num_samples_per_iteration; i++)
	{
		_measured_v.push_back(0);
		_commands.push_back(0);
		_commands_timestamps.push_back(_current_time);

		if (_future_angle < 0)
			_desired_v.push_back(0);
		else
			_desired_v.push_back(sin(_future_angle));

		_future_angle += DELTA_ANGLE;
		if (_future_angle > 2 * M_PI) _future_angle = INITIAL_ANGLE;

		_current_time += SAMPLING_TIME;
	}
}


State
Plant::getState()
{
	return State(
		vector<double>(_desired_v.begin(), _desired_v.end()),
		vector<double>(_measured_v.begin(), _measured_v.end()),
		vector<double>(_commands.begin(), _commands.end())
	);
}


double
Plant::act(double command)
{
	_commands.push_back(command);
	_commands_timestamps.push_back(_current_time);

	if (_commands.size() > _num_samples_per_iteration)
	{
		_commands.pop_front();
		_commands_timestamps.pop_front();
	}

	_updateState();

	double current_command = _commands[_commands.size() - 1];
	double last_command = _commands[_commands.size() - 2];

	double velocity_difference_punishment = -_difference_between_expected_and_measured_v;
	double unsmoothness_punishment = -abs(current_command - last_command) / 10.0;

	double reward = velocity_difference_punishment /*+ unsmoothness_punishment*/;
	return reward;
}


bool
Plant::newExperimentStarted()
{
	return _new_experiment_started;
}


