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
#include <carmen/drive_low_level.h>
#include <sys/ioctl.h>
#include "Nclient.h"
#include <limits.h>

#define METRES_PER_INCH 0.0254
#define METRES_PER_SCOUT (METRES_PER_INCH/10.0)
#define DEGREES_PER_SCOUT 0.1

#define MAXV (1.0/METRES_PER_SCOUT)
#define MAX_SONARS 16

//      STANDARD_WHEELBASE   13.4 in 
#define STANDARD_WHEELBASE   0.34036

static int sonar_is_on = 0;
static int model_type;

static double time_since_last_command = 0;

int 
carmen_base_direct_sonar_on(void)
{
  int sn_order[MAX_SONARS] = {0, 2, 15, 1, 14, 3, 13, 4, 12, 5, 11, 6, 
			      10, 7, 9, 8};  
  int result;
  
  result = conf_sn(15, sn_order);
  if (result == FALSE)
    return -1;

  conf_tm(5); 
  if (result == FALSE)
    return -1;

  time_since_last_command = carmen_get_time();
  
  sonar_is_on = 1;
  
  return MAX_SONARS;
}

int 
carmen_base_direct_sonar_off(void)
{
  int result;
    
  result = conf_sn(0, NULL);

  sonar_is_on = 0;

  if (result == FALSE)
    return -1;

  time_since_last_command = carmen_get_time();
  
  return 0;
}

int 
carmen_base_direct_reset(void)
{
  int result;

  result = zr();
  if(result != 1)
    return -1;

  time_since_last_command = carmen_get_time();
  
  return 0;
}

int 
carmen_base_direct_initialize_robot(char *model, char *dev)
{
  int result;

  if (strlen(model) == 4 && carmen_strncasecmp(model, "N200", 4) == 0)
    model_type = MODEL_N200;
  else if (strlen(model) == 4 && carmen_strncasecmp(model, "N150", 4) == 0)
    model_type = MODEL_N150;
  else if (strlen(model) == 5 && carmen_strncasecmp(model, "SCOUT", 5) == 0)
    model_type = MODEL_SCOUT;
  else if (strlen(model) == 6 && carmen_strncasecmp(model, "SCOUT2", 6) == 0)
    model_type = MODEL_SCOUT2;
  else
    {
      carmen_warn("%s Unknown Nomad model %s:\nAcceptable models are\n"
		  "N200, N150, SCOUT and SCOUT2%s\n", carmen_red_code, model,
		  carmen_normal_code);
      return -1;
    }

  result = connect_robot(1, model_type, dev, 38400);
  if(result == FALSE)
    return -1;

  return 0;
}

int 
carmen_base_direct_shutdown_robot(void)
{
  st();
  lp();
  disconnect_robot(1);
  time_since_last_command = carmen_get_time();

  return 0;
}

int 
carmen_base_direct_set_acceleration(double acceleration)
{
  double acc;
  int result;

  acc = acceleration/METRES_PER_SCOUT;
  result = ac(acc, acc, 0.0);
  if (result == FALSE)
    return -1;

  time_since_last_command = carmen_get_time();

  return 0;
}

int 
carmen_base_direct_set_deceleration(double deceleration)
{
  double acc;
  int result;

  acc = deceleration/METRES_PER_SCOUT;
  result = ac(acc, acc, 0.0);
  if (result == FALSE)
    return -1;

  time_since_last_command = carmen_get_time();

  return 0;
}

int 
carmen_base_direct_set_velocity(double tv, double rv)
{
  double vl, vr;
  int result;

  if (model_type == MODEL_SCOUT || model_type == MODEL_SCOUT2)
    {
      vl = tv;
      vr = tv;
      
      vl -= 0.5 * rv * STANDARD_WHEELBASE;
      vr += 0.5 * rv * STANDARD_WHEELBASE;
      
      vl /= METRES_PER_SCOUT;
      vr /= METRES_PER_SCOUT;  
      
      if(vl > MAXV)
	vl = MAXV;
      else if(vl < -MAXV)
	vl = -MAXV;
      if(vr > MAXV)
	vr = MAXV;
      else if(vr < -MAXV)
	vr = -MAXV;
      
      result = vm(carmen_trunc(vl), carmen_trunc(vr), 0.0);
    }
  else
    {
      tv /= METRES_PER_SCOUT;
      rv = carmen_radians_to_degrees(rv) / DEGREES_PER_SCOUT;
      result = vm(tv, rv, 0.0);
    }

  if (result == FALSE)
    return -1;

  time_since_last_command = carmen_get_time();

  return 0;
}

int 
carmen_base_direct_update_status(double* packet_timestamp __attribute__ ((unused)))
{
  if (carmen_get_time() - time_since_last_command < .06)
    return 0;

  return gs();
}

int
carmen_base_direct_get_state(double *displacement, double *rotation,
			     double *tv, double *rv)
{
  static int initialised = 0;
  static double prev_x, prev_y, prev_theta;

  double x, y, theta;
  double delta_x, delta_y, delta_theta;

  x = ((double)State[STATE_CONF_X]) * METRES_PER_SCOUT;
  y = ((double)State[STATE_CONF_Y]) * METRES_PER_SCOUT;
  theta = ((double)State[STATE_CONF_STEER]) * DEGREES_PER_SCOUT;
  theta = carmen_normalize_theta(carmen_degrees_to_radians(theta));

  if (initialised) 
    {      
      delta_x = x - prev_x;
      delta_y = y - prev_y;
      delta_theta = theta - prev_theta;

      if (x < prev_x && prev_x - x > LONG_MAX/2)
	delta_x = (x + LONG_MAX / 2) - (prev_x - LONG_MAX/2);
      else if (x > prev_x && x - prev_x > LONG_MAX/2)
	delta_x = (x - LONG_MAX / 2) - (prev_x + LONG_MAX/2);

      if (y < prev_y && prev_y - y > LONG_MAX/2)
	delta_y = (y + LONG_MAX / 2) - (prev_y - LONG_MAX/2);
      else if (y > prev_y && y - prev_y > LONG_MAX/2)
	delta_y = (y - LONG_MAX / 2) - (prev_y + LONG_MAX/2);

      if (theta < prev_theta && prev_theta - theta > M_PI)
	delta_theta = (theta + M_PI) - (prev_theta - M_PI);
      else if (theta > prev_theta && theta - prev_theta > M_PI)
	delta_theta = (theta - M_PI) - (prev_theta + M_PI);
      
      if (displacement)
	*displacement = hypot(delta_x, delta_y);

      if (rotation)
	*rotation = delta_theta;
    }
  else 
    {
      initialised = 1;
    }

  prev_x = x;
  prev_y = y;
  prev_theta = theta;

  if (tv)
    *tv = ((double)State[STATE_VEL_TRANS]) * METRES_PER_SCOUT;
  if (rv)
    *rv = ((double)State[STATE_VEL_STEER]) * METRES_PER_SCOUT;

  return 0;
}

int
carmen_base_direct_get_integrated_state(double *x, double *y, double *theta,
					double *tv, double *rv)
{
  if (x)
    *x = ((double)State[STATE_CONF_X]) * METRES_PER_SCOUT;
  if (y)
    *y = ((double)State[STATE_CONF_Y]) * METRES_PER_SCOUT;
  if (theta) {
    *theta = ((double)State[STATE_CONF_STEER]) * DEGREES_PER_SCOUT;
    *theta = carmen_normalize_theta(carmen_degrees_to_radians(*theta));
  }

  if (tv)
    *tv = ((double)State[STATE_VEL_TRANS]) * METRES_PER_SCOUT;
  if (rv)
    *rv = ((double)State[STATE_VEL_STEER]) * METRES_PER_SCOUT;

  return 0;
}

int 
carmen_base_direct_send_binary_data(unsigned char *data, 
				    int size __attribute__ ((unused)))
{
  return special_request(data, NULL);
}

int 
carmen_base_direct_get_binary_data(unsigned char **data, int *size)
{
  if (data == NULL || size == NULL)
    return 0;

  (*data) = calloc(NUM_STATE, sizeof(long));
  carmen_test_alloc(*data);
  memcpy(*data, State, sizeof(long)*NUM_STATE);

  return 0;
}

int carmen_base_direct_get_bumpers(unsigned char *state __attribute__ ((unused)), 
				   int num_bumpers __attribute__ ((unused)))
{
  carmen_warn("Send binary data not supported by RFlex.\n");

  return 0;
}

void carmen_base_direct_arm_get(double servos[] __attribute__ ((unused)), 
				int num_servos __attribute__ ((unused)), 
				double *currents __attribute__ ((unused)), 
				int *gripper __attribute__ ((unused)))
{
  carmen_warn("%s not supported by pioneer.\n", __FUNCTION__);
}

void carmen_base_direct_arm_set(double servos[] __attribute__ ((unused)), 
				int num_servos __attribute__ ((unused)))
{
  carmen_warn("%s not supported by pioneer.\n", __FUNCTION__);
}

int carmen_base_direct_get_sonars(double *ranges, 
				  carmen_point_t *positions 
				  __attribute__ ((unused)),
				  int num_sonars)
{
  int i;

  if (ranges != NULL && sonar_is_on) {
    if (num_sonars > MAX_SONARS)
      num_sonars = MAX_SONARS;
    for(i = 0; i < num_sonars; i++)
      ranges[i] = ((double)State[i + STATE_SONAR_0]) * METRES_PER_INCH;
  } /* End of if (sonar != NULL && sonar_is_on) */
  
  return 0;
}
