/*
 * control.h
 *
 *  Created on: Feb 26, 2010
 *      Author: uscr
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define PROP_SERIAL_PORT		"/dev/ttyUSB1"
#define PROP_SERIAL_BAUD		B115200

bool initializeController();
void setMotors(unsigned char front, unsigned char rear, unsigned char left, unsigned char right);
void shutdownController();

#endif /* CONTROL_H_ */
