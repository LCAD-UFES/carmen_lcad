/*
 * Path_Follower.h
 *
 *  Created on: 09/02/2012
 *      Author: rradaelli
 */

#ifndef PATH_FOLLOWER_H_
#define PATH_FOLLOWER_H_

using namespace std;
#include <list>
#include "path_follower.h"

class Follower
{
public:
	static void follow_path();
	static void go();
	static void stop();

	static Path_Follower *path_follower;
};

#endif /* PATH_FOLLOWER_H_ */
