
#include "DqnEpisode.h"


DqnEpisode::DqnEpisode()
{
	//_initial_time = -1;
	//_duration = 0;
	_total_reward = 0;
}


DqnEpisode::DqnEpisode(DqnEpisode *episode)
{
	_total_reward = episode->_total_reward;
	//_initial_time = episode->_initial_time;
	//_duration = episode->_duration;

	for (int i = 0; i < episode->GetInteractions()->size(); i++)
	{
		_interactions.push_back(new DqnInteration(episode->GetInteractions()->at(i)));
	}
}


void
DqnEpisode::AddInteration(DqnInteration *interaction)
{
	//if (_initial_time == -1)
		//_initial_time = interaction->rom_info->at(0);

	_interactions.push_back(interaction);
	//_duration = _initial_time - interaction->rom_info->at(0);
	_total_reward += interaction->immediate_reward;
}


vector<DqnInteration*>*
DqnEpisode::GetInteractions()
{
	return &_interactions;
}


//double
//DqnEpisode::GetDuration()
//{
//	return _duration;
//}


double
DqnEpisode::GetTotalReward()
{
	return _total_reward;
}

