/*
 * panel.h
 *
 *  Created on: Apr 19, 2013
 *      Author: cayo
 */

#ifndef PANEL_H_
#define PANEL_H_

#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>

#include <carmen/carmen.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/robot_ackerman_interface.h>

#include "components/lights.h"
#include "components/arrow.h"
#include "components/steering.h"
#include "components/speedometer.h"
#include "components/accelerator.h"
#include "components/accelerators/without_time.h"
#include "components/accelerators/with_time.h"

typedef enum
{
	fused_odometry_t,
	robot_ackerman_motion_command_t,
	base_ackerman_motion_command_t,
	base_ackerman_odometry_t,
	localize_ackerman_globalpos_t
} handler_message_t;

static const float FarNear = 100.0f;

float angleSteering = 0.0;
float angleTireToSteering = 10.0;
double interval = 1.0;

Steering *steering;
Arrow *arrowLeft;
Arrow *arrowRight;
Lights *lights;
Speedometer *speedometer;
Accelerator *accelerator;

handler_message_t handler_message;

int checkArguments(int argc, char *argv[]);

void display(void);

void reshape(int w, int h);

void subscribe_messages(int, double);

void setTypeMessage(int);

#endif /* PANEL_H_ */
