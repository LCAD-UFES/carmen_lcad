/*
 * carmen_state.h
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#ifndef CARMEN_STATE_H_
#define CARMEN_STATE_H_

#include "map_config.h"
#include "robot_config.h"

class Carmen_State {
public:
	Carmen_State();
	virtual ~Carmen_State();
	static Carmen_State *get_instance();

	Map_Config *map_config;
	Robot_Config *robot_config;
};

#endif /* CARMEN_STATE_H_ */
