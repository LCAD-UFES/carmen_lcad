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

#ifndef CARMEN_DRIVE_LOW_LEVEL_H
#define CARMEN_DRIVE_LOW_LEVEL_H

#ifdef __cplusplus
extern "C" {
#endif


#if 0  // for base & arm controlled on one board

typdef void carmen_base_model_t, *carmen_base_model_p;

/************ Jared 3/9/06 *************
  For bases controlling an arm (e.g. orc), these three functions handle
  both drive and base setup/shutdown/reset, while the corresponding arm functions
  use the base model returned from carmen_base_direct_initialize_robot().  In
  particular, only the base part should handle opening/closing ports. Ideally, we'd
  separate these functions into base_direct_* and drive_direct_*.
****************************************/
int carmen_base_direct_reset(void);
carmen_base_model_p carmen_base_direct_initialize_robot(char *model, char *dev);
int carmen_base_direct_shutdown_robot(void);

#endif


int carmen_base_direct_reset(void);
int carmen_base_direct_initialize_robot(char *model, char *dev);
int carmen_base_direct_shutdown_robot(void);

int carmen_base_direct_sonar_on(void);
int carmen_base_direct_sonar_off(void);
int carmen_base_direct_set_acceleration(double acceleration);
int carmen_base_direct_set_deceleration(double deceleration);
int carmen_base_direct_set_velocity(double tv, double rv);
int carmen_base_direct_update_status(double* packet_timestamp);
int carmen_base_direct_get_state(double *displacement, double *rotation,
				 double *vl, double *vr);
int carmen_base_direct_get_integrated_state(double *x, double *y, 
					    double *theta, double *tv, 
					    double *rv);
int carmen_base_direct_get_sonars(double *ranges, carmen_point_t *positions,
				  int num_sonars);
int carmen_base_direct_get_bumpers(unsigned char *state, int num_bumpers);
int carmen_base_direct_send_binary_data(unsigned char *data, int size);
int carmen_base_direct_get_binary_data(unsigned char **data, int *size);

int carmen_base_query_encoders(double *disp_p, double *rot_p,
			       double *tv_p, double *rv_p);
#ifdef __cplusplus
}
#endif

#endif
