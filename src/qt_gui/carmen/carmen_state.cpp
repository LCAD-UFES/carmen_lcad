/*
 * carmen_state.cpp
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#include "carmen_state.h"

Carmen_State* instance = 0;

Carmen_State::Carmen_State() {
	map_config = new Map_Config();
	robot_config = new Robot_Config();
}

Carmen_State* Carmen_State::get_instance() {
	if(!instance) {
		instance = new Carmen_State();
	}
	return instance;
}

Carmen_State::~Carmen_State() {
	// TODO Auto-generated destructor stub
}
