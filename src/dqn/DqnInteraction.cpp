

#include <cstring>
#include "DqnParams.h"
#include "DqnInteraction.h"


DqnInteration::DqnInteration()
{
	input = NULL;
	immediate_reward = -DBL_MAX;
	action = -1;

	last_commands = NULL;
	last_vs = NULL;
	last_phis = NULL;
	last_goal_poses = NULL;

	additional_data_vector = new vector<float>(DQN_NUM_ADDITIONAL_DATA);
}


DqnInteration::DqnInteration(DqnInteration *iter)
{
	input = new Mat();
	iter->input->copyTo(*input);

	immediate_reward = iter->immediate_reward;
	action = iter->action;

	last_commands = new std::deque<int>(*(iter->last_commands));
	last_vs = new std::deque<double>(*(iter->last_vs));
	last_phis = new std::deque<double>(*(iter->last_phis));
	last_goal_poses = new std::deque<carmen_point_t>(*(iter->last_goal_poses));
	additional_data_vector = new vector<float>(*(iter->additional_data_vector));
}


DqnInteration::DqnInteration(Mat *input_p, double immediate_reward_p,
		int action_p, std::deque<int> *last_commands_p,
		std::deque<double> *last_vs_p, std::deque<double> *last_phis_p,
		std::deque<carmen_point_t> *last_goal_poses_p)
{
	input = input_p;
	immediate_reward = immediate_reward_p;
	action = action_p;

	last_commands = last_commands_p;
	last_phis = last_phis_p;
	last_vs = last_vs_p;
	last_goal_poses = last_goal_poses_p;

	additional_data_vector = new vector<float>(DQN_NUM_ADDITIONAL_DATA);
}


void
DqnInteration::free()
{
	delete(input);
	delete(last_commands);
	delete(last_vs);
	delete(last_phis);
	delete(last_goal_poses);
	delete(additional_data_vector);
}


vector<float>*
DqnInteration::AdditionalDataVector()
{
	BuildDataVector(last_commands, last_vs, last_phis, last_goal_poses, additional_data_vector);
	return additional_data_vector;
}


void
DqnInteration::BuildDataVector(std::deque<int> *last_commands_p,
		std::deque<double> *last_vs_p, std::deque<double> *last_phis_p,
		std::deque<carmen_point_t> *last_goal_poses_p, vector<float>* vec)
{
	int i, p;

	assert(vec);

	if (vec->size() != DQN_NUM_ADDITIONAL_DATA)
		exit(printf("Error:: DqnInteration:: BuildDataVector:: output vector should be alloc'd previously\n"));

	vec->assign(DQN_NUM_ADDITIONAL_DATA, 0);

	p = 0;

	for (i = 0; i < last_commands_p->size(); i++)
		vec->at(p + i) = last_commands_p->at(i) / 10.0;

	p += DQN_NUM_PAST_COMMANDS_TO_STORE;

	for (i = 0; i < last_vs_p->size(); i++)
	{
		vec->at(p + 2 * i + 0) = last_vs_p->at(i) / 60.0;
		vec->at(p + 2 * i + 1) = last_phis_p->at(i) / M_PI;
	}

	p += (2 * DQN_NUM_PAST_ODOMS_TO_STORE);

	for (i = 0; i < last_goal_poses_p->size(); i++)
	{
		vec->at(p + 3 * i + 0) = last_goal_poses_p->at(i).x / 100.0;
		vec->at(p + 3 * i + 1) = last_goal_poses_p->at(i).y / 100.0;
		vec->at(p + 3 * i + 2) = last_goal_poses_p->at(i).theta / M_PI;
	}
}
