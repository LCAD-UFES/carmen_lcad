
#ifndef _RL_CONTROLLER_H_
#define _RL_CONTROLLER_H_

#include <list>
#include <vector>
#include "State.h"

using namespace std;

class RLController
{
protected:

	double _MAX_CONTROL;
	double _MIN_CONTROL;
	int _NUM_FUTURE_VELOCITIES;
	int _NUM_PAST_VELOCITIES;
	int _NUM_PAST_COMMANDS;

public:

	RLController()
	{
		_MAX_CONTROL = 1.0;
		_MIN_CONTROL = -1.0;
		_NUM_FUTURE_VELOCITIES = 40;
		_NUM_PAST_VELOCITIES = 40;
		_NUM_PAST_COMMANDS = 40;
	}

	virtual ~RLController() { }

	virtual void saveTrain() = 0;
	virtual void loadTrain(char *filename) = 0;

	virtual pair<double, double> selectAction(State state, FILE *f, bool use_greedy) = 0;
	virtual void train(State state, double reward, State next_state, pair<double, double> action_and_predicted_reward, FILE *f) = 0;
};


#endif

