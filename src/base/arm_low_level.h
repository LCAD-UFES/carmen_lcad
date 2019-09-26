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
  
  // typedef enum { CARMEN_MOTOR, CARMEN_SERVO } carmen_arm_joint_t;

  typedef struct {
    char *model_name;
    char *dev;
    int num_joints;
    carmen_arm_joint_t *joints;
    double *link_lengths;
  } carmen_arm_model_t, *carmen_arm_model_p;


#if 0  // for base & arm controlled on one board

  // passes in an base model pointer (e.g. orc) and tell arm how many joints
  int carmen_arm_direct_initialize(carmen_base_model_t *base_model, carmen_arm_model_t *arm_model);

#endif


  int carmen_arm_direct_initialize(carmen_arm_model_t *arm_model);

  int carmen_arm_direct_shutdown(void);

  // ----- sets ----- //

  // sets error and velocities to zero
  int carmen_arm_direct_reset(void);
  
  // sets safe joint use limits -- input array of limits for each element
  void carmen_arm_direct_set_limits(double *min_angle, double *max_angle,
				  int *min_pwm, int *max_pwm);
  
  // this is OPEN loop, should be part of a larger control loop
  // sets desired joint angles and implements control for next time step
  void carmen_arm_direct_update_joints(double *joint_angles);

  // ----- gets ----- //
  void carmen_arm_direct_get_state(double *joint_angles, double *joint_currents,
				 double *joint_angular_vels, 
				 int *gripper_closed );

#ifdef __cplusplus
}
#endif

#endif
