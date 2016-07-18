/*
 * command.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#include "command.h"
#include <math.h>
#include "model/global_state.h"

Command::Command(double v, double phi)
{
	this->v	= v;
	this->phi = phi;
}
