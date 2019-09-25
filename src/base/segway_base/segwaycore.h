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

#ifndef SEGWAYCORE_H
#define SEGWAYCORE_H

#include <canlib.h>

#define      SEGWAY_BITRATE               BAUD_500K
#define      SEGWAY_STATUS1_ID            0x400
#define      SEGWAY_STATUS2_ID            0x401
#define      SEGWAY_STATUS3_ID            0x402
#define      SEGWAY_STATUS4_ID            0x403
#define      SEGWAY_STATUS5_ID            0x404
#define      SEGWAY_SHUTDOWN_ID           0x412
#define      SEGWAY_COMMAND_ID            0x413

#define      SEGWAY_VEL_SCALE_FACTOR      10
#define      SEGWAY_ACCEL_SCALE_FACTOR    11
#define      SEGWAY_TORQUE_SCALE_FACTOR   12
#define      SEGWAY_GAIN_SCHEDULE         13

#define      SEGWAY_LIGHT                 0
#define      SEGWAY_TALL                  1
#define      SEGWAY_HEAVY                 2

#define      SEGWAY_CHANNEL1              0
#define      SEGWAY_CHANNEL2              1

#define      SEGWAY_MAX_TV                1176
#define      SEGWAY_MAX_RV                1024

typedef struct {
  canHandle handle1, handle2;
  int status_ready;
  int first_odom;
  double start_theta;
  double voltage, pitch, pitch_rate, roll, roll_rate;
  double lw_velocity, rw_velocity, yaw_rate;
  double lw_displacement, rw_displacement;
  double fore_aft_displacement, yaw_displacement;
  double last_fa_displacement;
  int frame_counter;
  double x, y, theta;
} segway_t, *segway_p;

void segway_clear_status(segway_p segway);

void segway_update_status(segway_p segway);

void segway_print_status(segway_p segway);

void segway_initialize(segway_p segway);

void segway_free(segway_p segway);

void segway_kill(segway_p segway);

void segway_command(segway_p segway, double tv, double rv, 
		    unsigned short status_command, 
		    unsigned short status_parameter);

void segway_set_velocity(segway_p segway, double tv, double rv);

void segway_stop(segway_p segway);

void segway_set_max_velocity(segway_p segway, double percent);

void segway_set_max_acceleration(segway_p segway, double percent);

void segway_set_max_torque(segway_p segway, double percent);

void segway_set_gain_schedule(segway_p segway, int schedule);

#endif

