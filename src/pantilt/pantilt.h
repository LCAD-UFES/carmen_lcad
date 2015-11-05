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

#ifndef PANTILT_H
#define PANTILT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef MAX_STRING_LENGTH 
#define MAX_STRING_LENGTH    256
#endif

#ifndef rad2deg
#define rad2deg carmen_radians_to_degrees
#endif

#ifndef deg2rad
#define deg2rad carmen_degrees_to_radians
#endif

#define BUFFER_SIZE          4096

#define MAX_CMD_TIME         20
#define TIME_TO_UPDATE       1

#define CMD_ERROR            0
#define CMD_OK               1

#define MIN_PAN_DIFF         100
#define MIN_TILT_DIFF        100

typedef struct _dev *DEV_PTR;

typedef void  *Pointer;
typedef void (*DEVICE_OUTPUT_HND)(int, long );
typedef void (*DEVICE_SET_TIMEOUT)(DEV_PTR, int);
typedef void (*DEVICE_CANCEL_TIMEOUT)(DEV_PTR);
typedef void (*Handler)(Pointer, Pointer);

typedef struct {
  char * port;
  int    baud;   /* one of the codes B0 .. B9600 in ttydev.h */
  char   parity;
  int    bits;
  int    hwf;
  int    swf;
  int    fd;
} PantiltDeviceType, *PantiltDevicePtr;

typedef struct {
  int doff_pan;
  int doff_tilt;
  int toff_pan;
  int toff_tilt;
  int min_pan;
  int max_pan;
  int min_pan_deg;
  int max_pan_deg;
  int min_tilt;
  int max_tilt;
  int min_tilt_deg;
  int max_tilt_deg;
  int acc_pan;
  int acc_tilt;
  int tspd_pan;
  int tspd_tilt;
  int bspd_pan;
  int bspd_tilt;
  float res_pan;
  float res_tilt;
} PantiltSettingsType, *PantiltSettingsPtr;

typedef struct {
  int pan;
  int tilt;
} PantiltPosType, *PantiltPosPtr;

typedef struct {
  /* which argumnent (u: used) */
  int upmin;
  int upmax;
  int utmin;
  int utmax;
  /* values */
  double pmin;
  double pmax;
  double tmin;
  double tmax;
} PantiltLimitsType;

extern PantiltDeviceType    pDevice;
extern PantiltSettingsType  pSettings;
extern PantiltLimitsType    pLimits;
extern PantiltPosType       pPos;

int  WriteCommand            ( PantiltDeviceType* pDevice,
			       char *argument, int arg_length );
void ProcessLine             ( char *line, int length );
long numChars                ( int sd );

int  PantiltInitializeDevice ( PantiltDeviceType *pDevice );
void PantiltGetInfo          ( void );
void PantiltInit             ( int argc, char **argv );
void PantiltParams           ( int argc, char **argv );
void ipc_initialize_messages ( void );

int writeN(PantiltDeviceType dev, char *buf, int nChars);
int writeCMD(PantiltDeviceType dev, char *cmd, char *returnstr );

int prad2ticks( double rad );
int trad2ticks( double rad );
double pticks2rad( int ticks );
double tticks2rad( int ticks );
double pticks2deg( int ticks );
double tticks2deg( int ticks );
int pdeg2ticks( double degrees );
int tdeg2ticks( double degrees );

void  set_pan( double pan );
void  set_tilt( double tilt );
void  set_pantilt( double pan, double tilt );

#endif
