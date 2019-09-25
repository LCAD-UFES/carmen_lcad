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
 * Public License along with Foobar; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef CARMEN_ARM_LOW_LEVEL_H
#define CARMEN_ARM_LOW_LEVEL_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BASE_HAS_ARM
int carmen_arm_direct_initialize(char *model, char *dev);
int carmen_arm_direct_shutdown(void);
int carmen_arm_direct_reset(void);

int carmen_arm_direct_update_status(void);

  // velezj: RssII Arm
  void carmen_arm_control( void );
  void carmen_arm_reset( void );
  
void carmen_arm_direct_set_limits(double min_angle, double max_angle,
				  int min_pwm, int max_pwm);

#endif  

int carmen_arm_direct_num_velocities(int num_joints);
int carmen_arm_direct_num_currents(int num_joints);

void carmen_arm_direct_set(double *joint_angles, int num_joints);

void carmen_arm_direct_get_state(double *joint_angles, double *joint_currents,
				 double *joint_angular_vels, 
				 int *gripper_closed,
				 int num_joint_angles);
#ifdef __cplusplus
}
#endif

#endif
