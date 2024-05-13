/*
 * command.h
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#ifndef COMMAND_H_
#define COMMAND_H_

class Command
{
public:
	Command(double v = 0, double phi = 0);

	bool operator ==(const Command &command) const;
	bool operator !=(const Command &command) const;
	unsigned long int get_key();
	unsigned long int get_key(double &time);

	double v;
	double phi;
};

#endif /* COMMAND_H_ */
