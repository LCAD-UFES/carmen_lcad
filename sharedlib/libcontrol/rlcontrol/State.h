
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
		desired = deque<double>(desired_param);
		measured = deque<double>(measured_param);
		last_commmands = deque<double>(last_commmands_param);
	}

	void operator=(State &state)
	{
		desired = deque<double>(state.desired);
		measured = deque<double>(state.measured);
		last_commmands = deque<double>(state.last_commmands);
	}

	void clear()
	{
		desired.clear();
		measured.clear();
		last_commmands.clear();
	}
};



#endif
