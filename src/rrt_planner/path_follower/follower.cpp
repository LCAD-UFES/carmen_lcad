/*
 * Path_Follower.cpp
 *
 *  Created on: 09/02/2012
 *      Author: rradaelli
 */

#include "follower.h"
#include <stdio.h>
#include "../model/global_state.h"

Path_Follower *Follower::path_follower = 0;

void Follower::go()
{
	path_follower->go();
}

void Follower::stop()
{
	path_follower->stop();
}

void Follower::follow_path()
{
	path_follower->build_and_send_refined_path();
}

