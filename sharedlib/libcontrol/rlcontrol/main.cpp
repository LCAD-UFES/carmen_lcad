#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <map>
#include "State.h"
#include "Plant.h"
#include "Dqn.h"
#include "Util.h"

using namespace std;


double
control_difference(double desired, double current)
{
	return (desired - current) / 20.0;
}


void
print_report(State state, double action, State next_state, double reward, FILE *output)
{
	fprintf(output, "desired: %.3lf action: %.3lf measured: %.3lf reward: %.3lf desired future: %.3lf\n",
		state.desired[0], action, next_state.measured[next_state.measured.size() - 1],
		reward, state.desired[state.desired.size() - 1]);

	fflush(output);
}


int
main()
{
	srand(time(NULL));

	Dqn dqn;
	Plant plant(2.0);
	State state, next_state;
	double reward;
	pair<double, double> action_and_predicted_reward;

	state = plant.getState();

	char file_name[1024];
	int iter = 0;
	int n = 0;
	bool use_greedy = true;

	sprintf(file_name, "data_%lf.greedy.txt", get_time());
	FILE *output = fopen(file_name, "w");

	while (1 /*iter < 50000*/)
	{
		action_and_predicted_reward = dqn.selectAction(state, output, use_greedy);

		reward = plant.act(action_and_predicted_reward.first);
		next_state = plant.getState();

		dqn.train(state, reward, next_state, action_and_predicted_reward, output);

		if (use_greedy)
			print_report(state, action_and_predicted_reward.first, next_state, reward, output);

		state = next_state;
		iter++;

		//getchar();

		if (plant.newExperimentStarted())
		{
			n++;

			if ((use_greedy && n > 80) || n >= 100)
			{
				fprintf(stderr, "********************** NEW OUTPUT FILE!!!!!!!!!!!!!\n");
				fflush(stderr);

				use_greedy = !use_greedy;

				if (use_greedy)
				{
					sprintf(file_name, "data_%lf.greedy.txt", get_time());
				//else
					//sprintf(file_name, "data_%lf.explor.txt", get_time());

				fclose(output);
				output = fopen(file_name, "w");
				}

				n = 0;
			}
		}
	}

	fclose(output);

	return 0;
}

