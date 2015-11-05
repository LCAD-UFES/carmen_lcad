/*
 * command.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#include "command.h"
#include <math.h>
#include "global_state.h"

Command::Command(double v, double phi)
{
	this->v	= v;
	this->phi = phi;
}
//2 degrees = 0.034906585 radians
//3 degrees = 0.0523598776 radians
//3,5 degrees = 0,0610865238 radians
//4 degrees = 0.0698131701 radians
bool Command::operator==(const Command &command) const
{
	return fabs(v - command.v) < 0.01 && fabs(phi - command.phi) < 0.0610865238;
}

bool Command::operator!=(const Command &command) const
{
	return !(*this == command);
}
unsigned long int Command::get_key()
{
	static double vel_precision = 10.0; // resolucao de 0.1 m/s
	static double phi_precision = 4; // resolucao de 0.25 graus

	static int phi_size = ceil(carmen_radians_to_degrees(GlobalState::robot_config.max_phi) * 2 * phi_precision);
	int phi_d, vel;

	phi_d = ceil(carmen_radians_to_degrees(phi < 0 ? (-phi + GlobalState::robot_config.max_phi + 0.0174532925) : phi) * phi_precision);
	vel = ceil((v < 0 ? (-v + GlobalState::robot_config.max_vel) : v) * vel_precision);

	return vel * phi_size + phi_d;
}

unsigned long int Command::get_key(double &time)
{
	static double time_precision = 100; //2 casas decimais de precisao

	static unsigned long int time_size = ceil(100 * time_precision);
	unsigned long int time_d;

	time_d = ceil(time * time_precision);

	return get_key() * time_size + time_d;
}
