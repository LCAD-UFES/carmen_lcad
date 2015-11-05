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
#include <sys/ioctl.h>
#ifdef __APPLE__
#include <sys/termios.h>
#endif
#include <carmen/pantilt_messages.h>
#include <carmen/pantilt_interface.h>

#ifndef rad2deg
#define rad2deg carmen_radians_to_degrees
#endif

#ifndef deg2rad
#define deg2rad carmen_degrees_to_radians
#endif

#define KEYBOARD          0
#define MAX_CODE_LENGTH   5
#define PANTILT_DEBUG             0

#define SET_TMIN   -10
#define SET_TMAX    10
#define SET_PMIN   -120
#define SET_PMAX    120

#define TILT_STEP    2
#define PAN_STEP     7.5

static int     signalled;

double
fsgn( double val ) 
{
  if (val>0) { 
    return(1);
  } else if (val<0) {
    return(-1);
  } else {
    return(0);
  }
}

void
usage( char *prgname )
{
  fprintf( stderr,
	   "Usage: %s <pan> <tilt> or %s -keyboard\n",
	   prgname, prgname );
}
     
static void
catcher( int sig )
{
  signalled = sig;
}

long
numChars(int sd)
{
  long available=0;
  if (ioctl(sd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}    

int
main( int argc, char *argv[] )
{
  float  pan  = 0;
  float  tilt = 0;
  
  int           use_keyboard = 0;
#ifdef __APPLE__
  struct termios      cooked, raw;
#else
  struct termio	cooked, raw;
#endif
  unsigned char	c;
  unsigned int	i, numC, code[MAX_CODE_LENGTH];
  
  if (argc<2 || argc>3) {
    usage(argv[0]);
    exit(1);
  } else {
    if (argc==2) {
      if (strcmp( argv[1], "-keyboard")) {
	usage(argv[0]);
	exit(1);
     } else {
       use_keyboard=1;
     }
    } else {
      pan = atof(argv[1]);
      tilt = atof(argv[2]);
    }
  }
  carmen_ipc_initialize(argc, argv);

  if (!use_keyboard) {
    fprintf( stderr, "Move PANTILT: pan=%3.1f tilt=%3.1f\n", pan, tilt );
    carmen_pantilt_move( deg2rad(pan), deg2rad(tilt) );
  } else {
    fprintf( stderr, "************** KEYBOARD MODE ******************\n" );
    fprintf( stderr, "   press ESC to quit\n\n" );
    fprintf( stderr, "   PAN-MIN   = %d\n", SET_PMIN );
    fprintf( stderr, "   PAN-MAX   = %d\n", SET_PMAX );
    fprintf( stderr, "   TILT-MIN  = %d\n", SET_TMIN );
    fprintf( stderr, "   TILT-MAX  = %d\n", SET_TMAX );
    fprintf( stderr, "***********************************************\n" );
#ifdef __APPLE__
      for (i = SIGHUP; i <= SIGUSR2; i++) {
#else
    for (i = SIGHUP; i <= SIGPOLL; i++) {
#endif
      signal(c, catcher);
    }
#ifdef __APPLE__
      tcgetattr(KEYBOARD,&cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
#else
    ioctl(KEYBOARD, TCGETA, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termio));
#endif
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
#ifdef __APPLE__
      tcsetattr(KEYBOARD, TCSANOW, &cooked);
#else
    ioctl(KEYBOARD, TCSETA, &raw);
#endif
    signalled = 0;
    numC = 0;
    fprintf( stderr, "\r                      \rpan=%3.1f tilt=%3.1f",
	     pan, tilt );
    carmen_pantilt_move( deg2rad(pan), deg2rad(tilt) );
    while (!signalled) {
      if (numChars(KEYBOARD)>0) {
	while(numChars(KEYBOARD)>0) {
	  read(KEYBOARD, &c, 1);
	  if (numC<MAX_CODE_LENGTH) {
	    code[numC++] = c;
	  }
	}
	if (PANTILT_DEBUG) {
	  for (i=0; i<numC; i++)
	    fprintf( stderr, "%d ", code[i] );
	  fprintf( stderr, "\n" );
	}
	if (numC==1 && code[0]==27)
	  signalled=1;
	if (numC==3 && code[0]==27 && code[1]==91 &&
	    code[2]>=65 && code[2]<=68) {
	  switch( code[2] ) {
	  case 65:
	    if (tilt+TILT_STEP<=SET_TMAX) {
	      if (PANTILT_DEBUG) {
		fprintf( stderr, "<up>" );
	      } else {
		tilt+=TILT_STEP;
		fprintf( stderr, "\r                      \rpan=%3.1f tilt=%3.1f",
			 pan, tilt );
	      }
	    }
	    break;
	  case 66:
	    if (tilt-TILT_STEP>=SET_TMIN) {
	      if (PANTILT_DEBUG) {
		fprintf( stderr, "<down>" );
	      } else {
		tilt-=TILT_STEP;
		fprintf( stderr, "\r                      \rpan=%3.1f tilt=%3.1f",
			 pan, tilt );
	      }
	    }
	    break;
	  case 67:
	    if (pan+PAN_STEP<=SET_PMAX) {
	      if (PANTILT_DEBUG) {
		fprintf( stderr, "<right>" );
	      } else {
		pan+=PAN_STEP;
		fprintf( stderr, "\r                      \rpan=%3.1f tilt=%3.1f",
			 pan, tilt );
	      }
	    }
	    break;
	  case 68:
	    if (pan-PAN_STEP>=SET_PMIN) {
	      if (PANTILT_DEBUG) {
		fprintf( stderr, "<left>" );
	      } else {
		pan-=PAN_STEP;
		fprintf( stderr, "\r                      \rpan=%3.1f tilt=%3.1f",
			 pan, tilt );
	      }
	    }
	    break;
	  }
	  carmen_pantilt_move( deg2rad(pan), deg2rad(tilt) );
	}
	numC = 0;
      }
      IPC_listen(0);
      usleep(10000);
    }
    printf("\nBye...\n");
#ifdef __APPLE__
      tcsetattr(0, TCSANOW, &cooked);
#else
    ioctl(0, TCSETA, &cooked);
#endif
  }
  exit(0);
}
