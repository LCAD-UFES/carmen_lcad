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

/** @addtogroup localize liblocalize_motion **/
// @{

/** 
 * \file localize_motion.h 
 * \brief Library for the new CARMEN motion_model.
 *
 * ...
 **/


#ifndef CARMEN_LOCALIZE_ACKERMAN_MOTION_H
#define CARMEN_LOCALIZE_ACKERMAN_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double mean_c_d;
  double mean_c_t;
  double std_dev_c_d;
  double std_dev_c_t;

  double mean_d_d;
  double mean_d_t;
  double std_dev_d_d;
  double std_dev_d_t;

  double mean_t_d;
  double mean_t_t;
  double std_dev_t_d;
  double std_dev_t_t;
} carmen_localize_ackerman_motion_model_t;//todo verify

carmen_localize_ackerman_motion_model_t *carmen_localize_ackerman_motion_initialize(int argc, char *argv[]);

double carmen_localize_ackerman_sample_noisy_downrange(double delta_t, 
					      double delta_theta,
					      carmen_localize_ackerman_motion_model_t *model);

double carmen_localize_ackerman_sample_noisy_crossrange(double delta_t, 
					       double delta_theta,
					       carmen_localize_ackerman_motion_model_t *model);

double carmen_localize_ackerman_sample_noisy_turn(double delta_t, 
					 double delta_theta,
					 carmen_localize_ackerman_motion_model_t *model);

#ifdef __cplusplus
}
#endif

#endif
// @}
