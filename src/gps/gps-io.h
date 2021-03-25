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

#define DEFAULT_GPS_PORT      "/dev/ttyUSB0"
#define DEFAULT_GPS_BAUD      115200
#define DEFAULT_GPS_PARITY    NO
#define DEFAULT_GPS_DATABITS  8
#define DEFAULT_GPS_STOPBITS  1
#define DEFAULT_GPS_HWF       0
#define DEFAULT_GPS_SWF       0

void  DEVICE_set_params( SerialDevice dev );

void  DEVICE_set_baudrate( SerialDevice dev, int brate );

int   DEVICE_connect_port( SerialDevice *dev );

int   DEVICE_send( SerialDevice dev, unsigned char *cmd, int len );

int   DECVICE_recieve( SerialDevice dev, unsigned char *cmd, int *len );

int   DEVICE_read_data( SerialDevice dev );

int   DEVICE_bytes_waiting( int sd );

void  DEVICE_init_params( SerialDevice *p );

#define BUFFER_LENGTH         (2048)

