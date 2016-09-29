
#ifndef STATE_H_
#define STATE_H_

#include <deque>

using namespace std;

class State
{
public:
	deque<double> desired;
	deque<double> measured;
	deque<double> last_commmands;

	State() {}

	State(deque<double> desired_param, deque<double> measured_param, deque<double> last_commmands_param)
	{
		desired = desired_param;
		measured = measured_param;
		last_commmands = last_commmands_param;
	}

	void operator=(State &state)
	{
		desired = state.desired;
		measured = state.measured;
		last_commmands = state.last_commmands;
	}

	void clear()
	{
		desired.clear();
		measured.clear();
		last_commmands.clear();
	}
};



#endif
