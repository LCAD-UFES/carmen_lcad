

#ifndef DQNEPISODE_H_
#define DQNEPISODE_H_


#include <vector>
#include "DqnInteraction.h"
#include "DqnUtil.h"


using namespace std;


class DqnEpisode
{
	vector<DqnInteration*> _interactions;
	//double _duration;
	//double _initial_time;
	double _total_reward;

	public:

		DqnEpisode();
		DqnEpisode(DqnEpisode *episode);

		void AddInteration(DqnInteration *interaction);

		vector<DqnInteration*>* GetInteractions();

		//double GetDuration();
		double GetTotalReward();
};

#endif



