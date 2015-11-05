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

#include <carmen/carmen.h>
#include "localize_ackerman_motion.h"

static void install_params(carmen_localize_ackerman_motion_model_t *model,
			   int argc, char *argv[])
{
  carmen_param_t param_list[] = {
    {"localize", "mean_c_d", CARMEN_PARAM_DOUBLE, &(model->mean_c_d), 1, NULL},
    {"localize", "mean_c_t", CARMEN_PARAM_DOUBLE, &(model->mean_c_t), 1, NULL},
    {"localize", "std_dev_c_d", CARMEN_PARAM_DOUBLE, &(model->std_dev_c_d), 1, NULL},
    {"localize", "std_dev_c_t", CARMEN_PARAM_DOUBLE, &(model->std_dev_c_t), 1, NULL},

    {"localize", "mean_d_d", CARMEN_PARAM_DOUBLE, &(model->mean_d_d), 1, NULL},
    {"localize", "mean_d_t", CARMEN_PARAM_DOUBLE, &(model->mean_d_t), 1, NULL},
    {"localize", "std_dev_d_d", CARMEN_PARAM_DOUBLE, &(model->std_dev_d_d), 1, NULL},
    {"localize", "std_dev_d_t", CARMEN_PARAM_DOUBLE, &(model->std_dev_d_t), 1, NULL},

    {"localize", "mean_t_d", CARMEN_PARAM_DOUBLE, &(model->mean_t_d), 1, NULL},
    {"localize", "mean_t_t", CARMEN_PARAM_DOUBLE, &(model->mean_t_t), 1, NULL},
    {"localize", "std_dev_t_d", CARMEN_PARAM_DOUBLE, &(model->std_dev_t_d), 1, NULL},
    {"localize", "std_dev_t_t", CARMEN_PARAM_DOUBLE, &(model->std_dev_t_t), 1, NULL}
  };

  carmen_param_install_params(argc, argv, param_list, 
			      sizeof(param_list)/sizeof(param_list[0]));
}

carmen_localize_ackerman_motion_model_t *carmen_localize_ackerman_motion_initialize(int argc, char *argv[])
{
  carmen_localize_ackerman_motion_model_t *model;

  model = (carmen_localize_ackerman_motion_model_t *)
    calloc(1, sizeof(carmen_localize_ackerman_motion_model_t));
  carmen_test_alloc(model);

  install_params(model, argc, argv);

  return model;
}

double carmen_localize_ackerman_sample_noisy_downrange(double delta_t, 
					      double delta_theta,
					      carmen_localize_ackerman_motion_model_t 
					      *model)
{
  double downrange_mean, downrange_std_dev;
  double sample;

  downrange_mean = delta_t * model->mean_d_d + delta_theta * model->mean_d_t;
  downrange_std_dev = fabs(delta_t) * model->std_dev_d_d + fabs(delta_theta) * model->std_dev_d_t;

  if (downrange_std_dev < 1e-6)
    return downrange_mean;

  do 
  {
    sample = carmen_gaussian_random(downrange_mean, downrange_std_dev);
  } while (fabs(sample - downrange_mean) > 2 * downrange_std_dev);

  return sample; 
}

double carmen_localize_ackerman_sample_noisy_crossrange(double delta_t, 
					       double delta_theta,
					       carmen_localize_ackerman_motion_model_t 
					       *model)
{
  double crossrange_mean, crossrange_std_dev;
  double sample;

  crossrange_mean = delta_t * model->mean_c_d + delta_theta * model->mean_c_t;
  crossrange_std_dev = fabs(delta_t) * model->std_dev_c_d + fabs(delta_theta) * model->std_dev_c_t;

  if (crossrange_std_dev < 1e-6)
    return crossrange_mean;

  do 
  {
    sample = carmen_gaussian_random(crossrange_mean, crossrange_std_dev);
  } while (fabs(sample - crossrange_mean) > 2 * crossrange_std_dev);

  return sample; 
}

double carmen_localize_ackerman_sample_noisy_turn(double delta_t, double delta_theta,
					 carmen_localize_ackerman_motion_model_t *model)
{
  double turn_mean, turn_std_dev;
  double sample;

  turn_mean = delta_t * model->mean_t_d + delta_theta * model->mean_t_t;
  turn_std_dev = fabs(delta_t) * model->std_dev_t_d + fabs(delta_theta) * model->std_dev_t_t;

  if (turn_std_dev < 1e-6)
    return turn_mean;

  do 
  {
    sample = carmen_gaussian_random(turn_mean, turn_std_dev);
  } while (fabs(sample - turn_mean) > 2*turn_std_dev);

  return sample;
}
