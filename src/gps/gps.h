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

#ifndef CARMEN_GPS_H
#define CARMEN_GPS_H

#ifdef __cplusplus
extern "C"
{
#endif

#define GPS_FREQUENCY			20

#define MAX_NAME_LENGTH                256
#define MAX_COMMAND_LENGTH             256

#define EPSILON                     0.0001

#define TIMEOUT                         -1
#define WRONG                            0
#define OK                               1

#define GGA_MESSAGE 1
#define HDT_MESSAGE 2
#define RMC_MESSAGE 3



enum PARITY_TYPE   { NO, EVEN, ODD };

typedef struct {
  char                       ttyport[MAX_NAME_LENGTH];
  int                        baud;
  enum PARITY_TYPE           parity;
  int                        fd;
  int                        databits;
  int                        stopbits;
  int                        hwf;
  int                        swf;
} SerialDevice;

extern carmen_gps_gpgga_message    * carmen_extern_gpgga_ptr;
extern carmen_gps_gphdt_message    * carmen_extern_gphdt_ptr;
extern carmen_gps_gprmc_message    * carmen_extern_gprmc_ptr;

int    carmen_gps_parse_data( char * line, int num_chars );

#ifdef __cplusplus
}
#endif

#endif
