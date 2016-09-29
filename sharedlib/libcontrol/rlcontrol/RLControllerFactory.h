
#ifndef _RL_CONTROLLER_FACTORY_H_
#define _RL_CONTROLLER_FACTORY_H_

#include "RLController.h"
#include "ActorCriticController.h"
#include "QLearningController.h"


enum RLControllerType
{
	Q_LEARNING_CONTROLLER,
	ACTION_CRITIC_CONTROLLER
};


class RLControllerFactory
{
public:

	static RLController* build(RLControllerType type, char *solver_name)
	{
		if (type == Q_LEARNING_CONTROLLER)
		{
			return new QLearningController(solver_name);
		}
		else if (type == ACTION_CRITIC_CONTROLLER)
		{
			return new ActorCriticController(solver_name);
		}
		else
		{
			fprintf(stderr, "Error:: RLControllerFactory():: build:: Invalid RLControllerType!\n");
			return NULL;
		}
	}
};


#endif

