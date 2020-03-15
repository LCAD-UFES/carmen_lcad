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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "Nclient.h"

static const unsigned short i200m_reg_table[4][9] = {
  {0x161, 0x15e, 0x167, 0x16e, 0x17c, 0x182, 0x17f, 0x188, 0x185},
  {0x162, 0x15f, 0x168, 0x16f, 0x17d, 0x183, 0x180, 0x189, 0x186},
  {0x163, 0x160, 0x169, 0x170, 0x17e, 0x184, 0x181, 0x18a, 0x187},
  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
};

typedef struct {
  short unsigned int reg;
  long int val;
} WriteType;

static WriteType i200m_wr_pair[15]; 
static long int i200m_timestamp_offset; 

int i200m_fd = -1; 

int I200M_Initialize(struct N_RobotState *rs)
{
  if (i200m_fd > 0) {
    return 0;
  } 

  if (rs->RobotType != 'x') {
    return N_UNSUPPORTED;
  } 

  i200m_fd = open("/dev/i200m", 2);
  if (i200m_fd < 0) 
    return N_UNKNOWN_ERROR;
  
  i200m_timestamp_offset = 0;
  return 0;
}

int I200M_SetAxes(struct N_RobotState *rs)
{
  long int value; 
  unsigned int i; 
  unsigned int write_index; 
  unsigned int read_index; 
  struct N_Axis *axis; 
  long int *pr_ptrs[3]; 
  long int in[3]; 

  if (i200m_fd < 0) {
    return N_UNINITIALIZED;
  }

  read_index = 0; 
  in[0] = 0x31c0;
  if (read(i200m_fd, &in, 4) < 0) {
    return N_UNKNOWN_ERROR;
  } 

  if ((in[0] != 0) && (in[0] != 8))
    return N_AXES_NOT_READY;
  
  for (i = 0; i <= 2; i++) {
    pr_ptrs[i] = NULL;
  }

  i200m_wr_pair[0].reg = 0x2d40;
  i200m_wr_pair[0].val = (rs->AxisSet.Global != 0);
  write_index = 1;
  read_index = 0;
  
  for (i = 0, axis = &(rs->AxisSet.Axis[0]); i < rs->AxisSet.AxisCount; 
       i++, axis++) {
    if (axis->Update == 0) 
      continue;

    axis->Update = 0;
    value = 0;
    if (axis->Mode == N_AXIS_STOP) {
      i200m_wr_pair[write_index].reg = i200m_reg_table[i][2] << 5;
      i200m_wr_pair[write_index].val = 1000;
      write_index++;

      i200m_wr_pair[write_index].reg = i200m_reg_table[i][1] << 5;
      i200m_wr_pair[write_index].val = 0;
      write_index++;
    } else { 
      if (axis->Mode == N_AXIS_ACCELERATION) { 
	i200m_wr_pair[write_index].reg = i200m_reg_table[i][0] << 5;
	i200m_wr_pair[write_index].val = 
	  (axis->Acceleration < 0) ? 0x80000000 : 0x7fffffff;
	write_index++;
      } else if (axis->Mode == N_AXIS_VELOCITY) { 
	i200m_wr_pair[write_index].reg = i200m_reg_table[i][0] << 5;
	i200m_wr_pair[write_index].val = 
	  (axis->DesiredSpeed < 0) ? 0x80000000 : 0x7fffffff;
	write_index++;	
      } else { 
	
	if (axis->Mode == N_AXIS_POSITION_RELATIVE) { 
	  pr_ptrs[read_index] = &(i200m_wr_pair[write_index].val);
	  in[read_index++] = rs->AxisSet.Global ? i200m_reg_table[i][8] :
	    i200m_reg_table[i][7] << 5;
	} 
	
	value = axis->DesiredPosition;

	i200m_wr_pair[write_index].reg = i200m_reg_table[i][0] << 5;
	i200m_wr_pair[write_index].val = axis->DesiredPosition;
	write_index++;	
      } 
      
      if (axis->Mode == N_AXIS_ACCELERATION) {
	i200m_wr_pair[write_index].reg = i200m_reg_table[i][1] << 5;
	i200m_wr_pair[write_index].val = 0x7fffffff;
	write_index++;	
      } else { 

	if ((axis->Mode != N_AXIS_VELOCITY) && 
	    (axis->DesiredSpeed < 0))
	  return N_INVALID_ARGUMENT;
	
	i200m_wr_pair[write_index].reg = i200m_reg_table[i][1] << 5;
	i200m_wr_pair[write_index].val = abs(axis->DesiredSpeed);
	write_index++;	
      } 
      
      if (axis->Acceleration < 0)
	return N_INVALID_ARGUMENT;
      
      i200m_wr_pair[write_index].reg = i200m_reg_table[i][2] << 5;
      i200m_wr_pair[write_index].val = axis->Acceleration;
      write_index++;	
    } 
  } 

  i200m_wr_pair[write_index].reg = 0x2e40;
  i200m_wr_pair[write_index].val = 0;
  write_index++;	

  if (read_index != 0) {
    if (read(i200m_fd, in, read_index * 4) < 0)
      return N_UNKNOWN_ERROR;
    for (i = 0; i < read_index; i++) {
      *(pr_ptrs[i]) += in[i];
    } 
  }
  
  if (write(i200m_fd, i200m_wr_pair, write_index * 8) < 0)
    return N_UNKNOWN_ERROR;
  
  return 0;
}

int I200M_GetAxes(struct N_RobotState *rs)
{
  unsigned char stamp; 
  int i; 
  unsigned int read_index; 
  long int in[rs->AxisSet.AxisCount * 7 + 2]; 
  long int timestamp = 0; 
  struct N_Axis *axis; 

  if (i200m_fd < 0) {
    return N_UNINITIALIZED;
  }

  read_index = 0;
  in[read_index] = 0x2d40;
  read_index++;
  in[read_index] = 0x31c0;
  read_index++;

  axis = &(rs->AxisSet.Axis[0]);
  stamp = 0;
  for (i = 0; i < (int)rs->AxisSet.AxisCount; i++, axis++) {
    if (axis->DataActive == 0)
      continue;

    stamp |= axis->TimeStampActive;
    
    in[read_index] = i200m_reg_table[i][0] << 5;
    read_index++;
    in[read_index] = i200m_reg_table[i][1] << 5;
    read_index++;
    in[read_index] = i200m_reg_table[i][2] << 5;
    read_index++;
    in[read_index] = i200m_reg_table[i][5] << 5;
    read_index++;
    in[read_index] = i200m_reg_table[i][6] << 5;
    read_index++;

    if (rs->AxisSet.Global != 0) {
      in[read_index] = i200m_reg_table[i][8] << 5;
    } else { 
      in[read_index] = i200m_reg_table[i][7] << 5;
    } 
    read_index++;

    in[read_index] = i200m_reg_table[i][4] << 5;
    read_index++;
  } 

  if (stamp != 0) {
    in[read_index] = 0x31e0;
    read_index++;
  } 
  
  if (read(i200m_fd, in, read_index * sizeof(long int)) < 0)
    return N_UNKNOWN_ERROR;

  if (stamp != 0) {
    timestamp = in[read_index - 1] + i200m_timestamp_offset;
  } 

  read_index = 0;
  rs->AxisSet.Global = in[read_index++] == 1;
  rs->AxisSet.Status = in[read_index++];

  axis = &(rs->AxisSet.Axis[0]);
  for (i = 0; i < (int)rs->AxisSet.AxisCount; i++, axis++) {
    if (axis->DataActive == 0)
      continue;
    
    if (axis->TimeStampActive != 0) {
      axis->TimeStamp = timestamp;
    } 

    axis->DesiredPosition = in[read_index++];
    axis->DesiredSpeed = in[read_index++];
    axis->Acceleration = in[read_index++];

    axis->TrajectoryPosition = in[read_index++];
    axis->TrajectoryVelocity = in[read_index++];
    axis->ActualPosition = in[read_index++];
    axis->ActualVelocity = in[read_index++];
    axis->InProgress = !((axis->TrajectoryPosition == axis->DesiredPosition) ||
			 ((axis->DesiredSpeed == 0) &&
			  (axis->TrajectoryVelocity == 0)));
  } 

  return 0;
}

int I200M_SetJoystick(struct N_RobotState *rs)
{
  int write_index; 
  long int bmask; 
  struct N_Joystick *jsp; 

  write_index = 0;
  bmask = 0;

  jsp = &(rs->Joystick);

  if (jsp->ButtonA || jsp->ButtonB) {
    bmask |= 0x40;
  } 
  if (jsp->ButtonB || jsp->ButtonC) {
    bmask |= 0x80;
  } 

  if (bmask != 0) {
    if ((jsp->X > 1.0) || (jsp->Y > 1.0)) {
      return N_INVALID_ARGUMENT;
    }

    i200m_wr_pair[write_index].reg = 0x2e20;
    i200m_wr_pair[write_index].val = 2;
    write_index++;
    i200m_wr_pair[write_index].reg = 0x2e60;
    i200m_wr_pair[write_index].val = 100.0 * jsp->X;
    write_index++;
    i200m_wr_pair[write_index].reg = 0x2e80;
    i200m_wr_pair[write_index].val = 100.0 * jsp->Y;
    write_index++;
    i200m_wr_pair[write_index].reg = 0x2ea0;
    i200m_wr_pair[write_index].val = bmask;
    write_index++;
  } else { 
    i200m_wr_pair[write_index].reg = 0x2e20;
    i200m_wr_pair[write_index].val = 0;
    write_index++;
  } 

  i200m_wr_pair[write_index].reg = 0x2e40;
  i200m_wr_pair[write_index].val = 0;
  write_index++;

  if (write(i200m_fd, i200m_wr_pair, write_index * 8) < 0)
    return N_UNKNOWN_ERROR;

  return 0;

}

int I200M_SetIntegratedConfiguration(struct N_RobotState *rs)
{
  struct N_Integrator *conf; 


  if (i200m_fd < 0)
    return N_UNINITIALIZED;

  conf = &(rs->Integrator);
  i200m_wr_pair[0].reg = 0x2d60;
  i200m_wr_pair[0].val = conf->x;
  i200m_wr_pair[1].reg = 0x2d80;
  i200m_wr_pair[1].val = conf->y; 
  i200m_wr_pair[2].reg = 0x2da0;
  i200m_wr_pair[2].val = conf->Rotation; 
  i200m_wr_pair[3].reg = 0x2e40;
  i200m_wr_pair[3].val = 0; 
  if (write(i200m_fd, i200m_wr_pair, 0x20) < 0)
    return N_UNKNOWN_ERROR;

  return 0;
}

int I200M_GetIntegratedConfiguration(struct N_RobotState *rs)
{
  long int in[4]; 
  struct N_Integrator *conf; 

  if (i200m_fd < 0)
    return N_UNINITIALIZED;

  conf = &(rs->Integrator);
  if (!conf->DataActive)
    return 0;

  in[0] = 0x30a0;
  in[1] = 0x30c0;
  in[2] = 0x30e0;

  if (conf->TimeStampActive)
    in[3] = 0x31e0;

  if (read(i200m_fd, in, conf->TimeStampActive ? 16 : 12) < 0)
    return N_UNKNOWN_ERROR;
  
  conf->x = in[0];
  conf->y = in[1];
  conf->Rotation = in[2];
  if (conf->TimeStampActive)
    conf->TimeStamp = in[3] + i200m_timestamp_offset;

  return 0;
}

int I200M_ResetClient()
{
  long int mode; 
  if (i200m_fd < 0)
    return N_UNINITIALIZED;

  mode = 0x2d40;
  if (read(i200m_fd, &mode, sizeof(mode)) < 0)
    return N_UNKNOWN_ERROR;

  return 0;
}

long unsigned int I200M_MSecSinceBoot()
{
  long int timeval; 
  if (i200m_fd < 0)
    return 0;

  timeval = 0x3200;
  if (read(i200m_fd, &timeval, sizeof(timeval)) < 0)
    return 0;

  return timeval;
}

void I200M_ResetMotionTimer()
{
  i200m_wr_pair[0].reg = 0x2e60;
  i200m_wr_pair[0].val = 0;

  write(i200m_fd, i200m_wr_pair, 8);
  return ;
}

