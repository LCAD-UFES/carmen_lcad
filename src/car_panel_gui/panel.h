/*
 * panel.h
 *
 *  Created on: Apr 19, 2013
 *      Author: cayo
 */

#ifndef PANEL_H_
#define PANEL_H_

#include <math.h>

#include <carmen/carmen.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/robot_ackerman_interface.h>

#include "lights.h"
#include "arrow.h"
#include "steering.h"
#include "speedometer.h"
#include "accelerator.h"
#include "without_time.h"

typedef enum
{
	fused_odometry_t,
	robot_ackerman_motion_command_t,
	base_ackerman_motion_command_t,
	base_ackerman_odometry_t,
	localize_ackerman_globalpos_t
} handler_message_t;


int checkArguments(int argc, char *argv[]);

void display(void);

void reshape(int w, int h);

void keypress(unsigned char key, int x __attribute__ ((unused)), int y __attribute__ ((unused)));

void subscribe_messages(int);

void setTypeMessage(int);

void setTurnSignal(int turn_signal);

#endif /* PANEL_H_ */
