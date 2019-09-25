 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef I200M_H
#define I200M_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Nclient.h"

int I200M_Initialize(struct N_RobotState *rs);

int I200M_SetAxes(struct N_RobotState *rs);

int I200M_GetAxes(struct N_RobotState *rs);

int I200M_SetJoystick(struct N_RobotState *rs);

int I200M_SetLift(struct N_RobotState *rs);

int I200M_GetLift(struct N_RobotState *rs);

int I200M_ZeroLift(struct N_RobotState *rs __attribute__ ((unused)),
		   unsigned char force);

int I200M_DeployLift(struct N_RobotState *rs __attribute__ ((unused)));

int I200M_RetractLift(struct N_RobotState *rs __attribute__ ((unused)));

int I200M_SetIntegratedConfiguration(struct N_RobotState *rs);

int I200M_GetIntegratedConfiguration(struct N_RobotState *rs);

int I200M_ResetClient();

long unsigned int I200M_MSecSinceBoot();

void I200M_ResetMotionTimer();

#ifdef __cplusplus
}
#endif

#endif
