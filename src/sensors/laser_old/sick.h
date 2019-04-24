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

#ifndef CARMEN_SICK_H
#define CARMEN_SICK_H

#define LASER_BUFFER_SIZE                100000

#define CRC16_GEN_POL                    0x8005
#define CRC16_GEN_POL0                   0x80
#define CRC16_GEN_POL1                   0x05

#define LMS_PASSWORD                     "SICK_LMS"
#define PLS_PASSWORD                     "SICK_PLS"

#define BUFFER_SIZE                      16000
#define MAX_COMMAND_SIZE                 8196
#define MAX_NAME_LENGTH                  256

#define MAX_TIME_FOR_CLEAR               0.2
#define MAX_TIME_FOR_DATA                0.3
#define MAX_TIME_FOR_ACK                 0.1
#define MAX_TIME_FOR_ANSWER              0.1
#define MAX_TIME_FOR_SENSOR_DATA         0.5
#define MAX_TIME_FOR_CONFIG              3.0
#define MAX_TIME_FOR_GETTING_CONF        0.3
#define MAX_TIME_FOR_TESTING_BAUDRATE    1.0

#define INI                              -1
#define TIO                              0
#define STX                              0x02
#define UKN                              0x05
#define ACK                              0x06
#define DLE                              0x10
#define NAK                              0x15
#define LID                              0x80

#define RES_1_00_DEGREE                  0
#define RES_0_50_DEGREE                  1 
#define RES_0_25_DEGREE                  2

#ifndef TIOCGETP 
#define TIOCGETP                         0x5481
#define TIOCSETP                         0x5482
#define RAW                              1
#define CBREAK                           64
#endif

typedef enum { PLS, LMS } laser_model_t;
typedef enum { CM, MM, DM } range_res_t;
typedef enum { SICK_RANGE80M, SICK_RANGE160M, SICK_RANGE320M, SICK_REMISSION_NORM, SICK_REMISSION_DIRECT } range_dist_t;
typedef enum { N, E, O } parity_t;

typedef struct {
  int                fd;
  laser_model_t      type;
  char               *ttyport;
  int                baudrate;
  parity_t           parity;
  unsigned char      passwd[8];
  int                databits;
  int                stopbits;
  int                hwf;
  int                swf;
  int                laser_num;
} laser_device_t, *laser_device_p;

typedef struct {
  char device_name[MAX_NAME_LENGTH];
  laser_model_t type;
  range_res_t range_res;
  range_dist_t range_dist;
  unsigned char password[8];
  int laser_num;
  int detect_baudrate, use_highspeed;
  int start_baudrate, set_baudrate;
  int databits, stopbits;
  parity_t parity;
  int swf, hwf;
  int angle_range, angle_resolution;
  int use_remission;
  int rem_values;
  int num_values;
  int laser_flipped;
} laser_settings_t;

typedef struct {
  laser_settings_t settings;
  laser_device_t dev;

  int numvalues;
  double *range;
  int *glare, *wfv, *sfv;
// *** REI - START *** //
  double *remission;
  int remvalues;
// *** REI - END *** //
  unsigned char *buffer;
  long int buffer_position, processed_mark, packet_offset, packet_length;
  int new_reading;
  double timestamp;
  double packet_timestamp;
} sick_laser_t, *sick_laser_p;

void sick_start_laser(sick_laser_p laser);
void sick_stop_laser(sick_laser_p laser);
void sick_handle_laser(sick_laser_p laser);

#endif
