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

/*
 * Ndirect.c
 *
 * Implementation file for direct connection to robot, bypassing the need for
 * the Nserver program.
 *
 * Copyright 1991-1998, Nomadic Technologies, Inc.
 *
 */

char cvsid_host_client_Ndirect_c[] = "$Header: /cvsroot/carmen/carmen/src/base/scoutlib/Ndirect.c,v 1.2 2006/04/06 00:30:08 stachnis Exp $";

/* includes */

#include <stdlib.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#ifdef __USE_BSD
#undef __USE_BSD
#include <memory.h>
#define __USE_BSD
#else
#include <memory.h>
#endif
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>          
#include <sys/socket.h>
#include <sys/time.h>           

#include "Nclient.h"


/* defines */

/* from command type.h  */

#define AC 1
#define SP 2
#define PR 3
#define PA 4
#define VM 5
#define MV 43
#define CT 6
#define GS 7
#define NAK 8
#define ST 9
#define LP 10
#define TK 11
#define DP 12
#define ZR 13
#define CONF_IR 14
#define CONF_SN 15
#define CONF_CP 16
#define CONF_LS 17
#define CONF_TM 18
#define GET_IR 19
#define GET_SN 20
#define GET_RC 21
#define GET_RV 22
#define GET_RA 23
#define GET_CP 24
#define GET_LS 25
#define SETUP_LS 26
#define GET_BP 27

#define CONF_SG 28
#define GET_SG 29

#define DA 30
#define WS 31

#define ADD_OBS 32
#define DELETE_OBS 33
#define MOVE_OBS 34

#define CONF_SER  35
#define PLACE_ROBOT 36

#define NUM_COMMANDS 36

#define SPECIAL 128

/* error types */
#define SERIAL_OPEN_ERROR 1
#define SERIAL_WRITE_ERROR 2
#define SERIAL_READ_ERROR 3
#define SERIAL_PKG_ERROR 4
#define SERIAL_TIMEOUT_ERROR 5

/* serial setting */
#define RE_XMIT 0            /* not re-transmit when failed */
#define NORMAL_TIMEOUT     1 /* 1 second */
#define CONNECT_TIMEOUT   10 /* 10 seconds */
#define SPECIAL_TIMEOUT   30 /* used for user define package */

/* 
 * Define the length of the user buffer (Maximal short).
 * Due to Protocol bytes, the effective length is 65526 
 */

#define USER_BUFFER_LENGTH	0xFFFF

/* from pos.h */

/* 
 * these macros enable the user to determine if the pos-attachment
 * is desired for a specific sensor, the argument is Smask[ SMASK_POS_DATA ]
 * to avoid overlap with Nclient.h suffix I for internal
 */

#define POS_INFRARED_PI(x)  ( ( (x) & POS_INFRARED ) ? 1 : 0 )
#define POS_SONAR_PI(x)     ( ( (x) & POS_SONAR    ) ? 1 : 0 )
#define POS_BUMPER_PI(x)    ( ( (x) & POS_BUMPER   ) ? 1 : 0 )
#define POS_LASER_PI(x)     ( ( (x) & POS_LASER    ) ? 1 : 0 )
#define POS_COMPASS_PI(x)   ( ( (x) & POS_COMPASS  ) ? 1 : 0 )


/* from datatype.h */

/* to build a long out ouf four bytes */

#define LONG_B(x,y,z,q) ((((x) << 24) & 0xFF000000) | \
                         (((y) << 16) & 0x00FF0000) | \
                         (((z) <<  8) & 0x0000FF00) | \
                         ( (q)        & 0x000000FF) )

/*
 * The voltages have different ranges to account for the fact that the
 * CPU measurement is taken after lossage on the slip-ring.
 */

#define RANGE_CPU_VOLTAGE        12.0
#define RANGE_MOTOR_VOLTAGE      12.85

/********************
 *                  *
 * Type definitions *
 *                  *
 ********************/

/*
 * PosDataAll is a struct that contains the Pos information for
 * all sensors. It is used to pass/store the Pos info within the 
 * server. It contains the laser, although the laser is not
 * used in the dual ported ram.
 */

typedef struct _PosDataAll
{
  PosData infrared [INFRAREDS];
  PosData sonar    [SONARS   ];
  PosData bumper;
  PosData laser;
  PosData compass;

} PosDataAll;


/********************
 *                  *
 * Global Variables *
 *                  *
 ********************/

long State[NUM_STATE];        /* State reading */
int  Smask[NUM_MASK];         /* Sensor mask */
int  Laser[2*NUM_LASER+1];    /* Laser reading */


/* connect_type == 1 ---> serial */
/* connect_type == 2 ---> socket */
int    connect_type           = 1;
int    model;
char  *device;
int    conn_value;
int    DEFAULT_SERIAL_BAUD    = 38400;
int    DEFAULT_ROBOT_TCP_PORT = 65001;
double LASER_CALIBRATION[8]   = { -0.003470,  0.000008, 0.011963,  0.001830,
				  27.5535913, 0.000428, 0.031102, -0.444624 };
double LASER_OFFSET[2]        = { 0, 0 };

/* dummy variables to stay compatible with Nclient.c */

char   SERVER_MACHINE_NAME[80] = "";
int    SERV_TCP_PORT           = -1;
char   Host_name[255]          = "";

/*******************
 *                 *
 * Local Variables *
 *                 *
 *******************/

static int             Fd=-1;
static unsigned char   buf[BUFSIZE];
static unsigned char   *bufp, *bufe;
static int             errorp = 0;
static int             wait_time;
static int             usedSmask[NUM_MASK];       /* mask vector */
static int             Robot_id = -1;

/* although called special, it is the user buffer */
static unsigned char   *special_buffer;
static unsigned short  special_answer_size;

/* this is where all the incoming posData is stored */
static PosDataAll posDataAll;
static unsigned long posDataTime;

/* for the voltages of motor/CPU as raw data */
static unsigned char voltageCPU;
static unsigned char voltageMotor;

/* the laser mode */
static int laser_mode = 51;

/*******************
 *                 *
 * Functions (fwd) *
 *                 *
 *******************/

/* function declarations */

static long posLongExtract    ( unsigned char *inbuf );
static unsigned long posUnsignedLongExtract( unsigned char *inbuf );
static int posPackageProcess  ( unsigned char *inbuf, PosData *posData );
static int timePackageProcess ( unsigned char *inbuf, unsigned long *timeS );
static int voltPackageProcess ( unsigned char *inbuf,
			        unsigned char *voltCPU,
			        unsigned char *voltMotor);
static float voltConvert   ( unsigned char reading , float range );



/*******************************************************
 *                                                     *
 * Helper functions for manipulating bytes and numbers *
 *                                                     *
 *******************************************************/

static int low_half(unsigned char num)
{
  return (num % 16);
}

static int high_half(unsigned char num)
{
  return (num / 16);
}

/*
static unsigned char high_byte_signed(int n)
{
  int sign_num;
  
  if (n < 0) sign_num = 128; else sign_num = 0;
  return (sign_num + (abs(n) / 256));
}
*/
/*
static unsigned char low_byte_signed(int n)
{
  return (abs(n) % 256);
}
*/
/*
static unsigned char high_byte_unsigned(int n)
{
  return (n / 256);
}
*/
/*
static unsigned char low_byte_unsigned(int n)
{
  return (n % 256);
}
*/
static void signed_int_to_two_bytes(int n, unsigned char *byte_ptr)
{
  int sign_num;
  
  *byte_ptr = (unsigned char)(abs(n) % 256);
  byte_ptr++;
  if (n < 0) sign_num = 128; else sign_num = 0;
  *byte_ptr = (unsigned char)(sign_num + (abs(n) / 256));
}

static void unsigned_int_to_two_bytes(int n, unsigned char *byte_ptr)
{
  *byte_ptr = (unsigned char)(abs(n) % 256);
  byte_ptr++;
  *byte_ptr = (unsigned char)(abs(n) / 256);
}

static int two_bytes_to_signed_int(unsigned char low_byte,
				   unsigned char high_byte)
{
  int num;
  
  if (high_byte > 127)
    num = (-256 * (high_byte - 128)) - low_byte;
  else
    num = 256 * high_byte + low_byte;
  return (num);
}

static unsigned int two_bytes_to_unsigned_int(unsigned char low_byte,
					      unsigned char high_byte)
{
  unsigned int num;
  
  num = 256 * high_byte + low_byte;
  return (num);
}

static long combine_bumper_vector(unsigned char b1,
				  unsigned char b2,
				  unsigned char b3)
{
  long num;
  
  num = b1 + (b2 * 256) + (b3 * 65536);
  return (num);
}

static unsigned char bits_to_byte(char bt0, char bt1, char bt2, char bt3,
				  char bt4, char bt5, char bt6, char bt7)
{
  unsigned char  rbyte;
  
  rbyte = (unsigned char)(bt0 + (2*bt1) + (4*bt2) + (8*bt3) + (16*bt4) +
			  (32*bt5) + (64*bt6) + (128*bt7));
  return (rbyte);
}

static int serial_ready (int fd, int wait)
{
  fd_set          lfdvar;
  int             ready;
  struct timeval  timeout;
  
  FD_ZERO(&lfdvar);
  FD_SET(fd, &lfdvar);
  
  timeout.tv_sec = wait;
  timeout.tv_usec = 0;
  
  ready = select(fd+1, &lfdvar, NULL, NULL, &timeout);
  return(ready);
}

/* 
 * creates a package by adding initial byte, checksum, and end byte
 * and send the package to robot
 */
static int Write_Pkg(int fd, unsigned char *outbuf)
{
  int i, outbufLen, chk_sum; 
  int nleft, nwritten;
  
  /* create a package */
  outbuf[0] = 1;
  chk_sum = 0;
  outbufLen = outbuf[1]+1;
  for (i=0; i<=outbufLen; i++)
    chk_sum = chk_sum + outbuf[i];
  chk_sum = chk_sum % 256;
  outbufLen++;
  outbuf[outbufLen] = chk_sum;
  outbufLen++;
  outbuf[outbufLen] = 92;
  outbufLen++;

  /* send package */
  for (nleft = outbufLen; nleft > 0; ) {
    nwritten = write(fd, outbuf, nleft);
    if (nwritten <= 0) {
      errorp = SERIAL_WRITE_ERROR;
      return(FALSE);
    }
    nleft = nleft - nwritten;
    outbuf = outbuf + nwritten;
  }
  return(TRUE);
}


/* 
 * read response from the robot 
 */
static unsigned char buf_fill(int fd, int conn_type)
{
  int n;
  
  if (conn_type == 1) 
  {
    n = read(fd, buf, BUFSIZE);
    if (n < 0) {
      printf("error reading serial port\n");
      errorp = SERIAL_READ_ERROR;
      return(0);
    }
    else {
      if (n == 0) {
	printf("serial port read timeout error\n");
	errorp = SERIAL_TIMEOUT_ERROR;
	return(0);
      }
    }
  } else {
    if (serial_ready(Fd, 100)) {
      n = read(Fd, buf, BUFSIZE);
      if (n < 0) {
        printf("TCP/IP communication broken.\n");
        errorp = SERIAL_READ_ERROR;
        return 0;
      } else {
        if (n == 0) {
          printf("TCP/IP select/read error\n");
          errorp = SERIAL_READ_ERROR;
          return(0);
        }
      }
    } else {
      printf("TCP/IP read timeout error.\n");
      errorp = SERIAL_TIMEOUT_ERROR;
      return 0;
    }
  }
  
  bufp = &buf[1];
  bufe = buf + n;
  return(buf[0]);
}

static unsigned char GETC(int fd, int conn_type)
{
  if (bufp<bufe)
    return(*bufp++);
  return(buf_fill(fd, conn_type));
}

/*
 * getting package from robot and 
 * check for package error: initial byte = 1, end byte = 92, check sum
 */
static int Read_Pkg(int fd, int conn_type, unsigned char *inbuf)
{
  int i, length = 0, chk_sum = 0;
  unsigned char ichar, ichar2;

  if (!(serial_ready (fd, wait_time))) {
    errorp = SERIAL_TIMEOUT_ERROR;
    return(FALSE);
  }

  errorp = 0;
  
  /* read the begin packet character, it should be 1 */
  ichar = (unsigned char)GETC(fd, conn_type);
  if (!errorp) {
    if (ichar != 1) {
      printf("start byte error: %u\n", ichar);
      errorp = SERIAL_PKG_ERROR;
    }
    else {
      chk_sum = 1;
    }
  }
  
  if (!errorp) {
    /* read length, it should be >0 */
    ichar = GETC(fd, conn_type); 
    if (!errorp) {
      chk_sum = chk_sum + (int)ichar;
    }
    ichar2 = GETC(fd, conn_type);
    if (!errorp) {
      length = two_bytes_to_unsigned_int (ichar, ichar2);
      if (length < 1) {
	printf("length byte error\n");
	errorp = SERIAL_PKG_ERROR;
      }
      else {
	chk_sum = chk_sum + (int)ichar2;
      }
    }
  }
  
  /* get the data portion of the message */
  i = 0;
  while ((!errorp) && (i<=length)) {
    ichar = GETC(fd, conn_type);
    if (!errorp) {
      inbuf[i] = ichar;
      /* printf("%u\n",  ichar); */
      chk_sum = chk_sum + (int)(ichar);
    }
    i++;
  }
  
  if (!errorp) {
    /* check chk_sum and end_pkg */
    if (((chk_sum - inbuf[length-1] - 92) % 256) != inbuf[length-1]) {
      printf("check sum error\n");
      errorp = SERIAL_PKG_ERROR;
    }
    
    if (inbuf[length] != 92) {
      printf("packet end error\n");
      errorp = SERIAL_PKG_ERROR;
    }
  }
  
  if ((errorp) && (errorp != SERIAL_TIMEOUT_ERROR)) {
    printf("emptying buffer\n");
    buf_fill(Fd, conn_type);  /* read everything else in the serial line into
		    buffer */
    bufp = bufe; /* and flush it */
  }
  
  if (errorp)
    return(FALSE);
  else
    return (TRUE);
}

/*********************************************************
 *
 *   Laser Calibration Stuff 
 *
 *********************************************************/

/* Transformation function accordingly to the calibration */
/* xi1 = pixel; yi1 = scanline */
static void ProjectPhy(double xi1, double yi1, double *x, double *y)
{
  double xi,yi;
  double den;		
  
  xi = xi1 - 254.5;
  yi = yi1 - 240.5; 
  
  den = (LASER_CALIBRATION[0]*xi + LASER_CALIBRATION[1]*yi + 1);
  
  *x = (LASER_CALIBRATION[2]*xi + LASER_CALIBRATION[3]*yi +
	LASER_CALIBRATION[4])/den + LASER_OFFSET[0];
  *y = (LASER_CALIBRATION[5]*xi + LASER_CALIBRATION[6]*yi +
	LASER_CALIBRATION[7])/den + LASER_OFFSET[1];
}  

static void convert_laser(int *laser)
{
  int i, num_points, offset, interval;
  double line_num;
  double laserx[483], lasery[483];
  
  num_points = laser[0];
  interval = NUM_LASER/(num_points-1);
  offset = 3 + (NUM_LASER-(num_points * interval))/2;
  for (i=1; i<=num_points; i++) {
    line_num = (double)(offset+(i-1)*interval);
    ProjectPhy((double)laser[i], line_num, &laserx[i], &lasery[i]); 
  }
  for (i=1; i<=num_points; i++) {
    laser[2*i-1] = (int)(laserx[i]*10.0);
    laser[2*i] = (int)(lasery[i]*10.0);
  }
  return;
}

/*********************************************************
 *
 *   Processing different types of packages received from the robot 
 *
 *********************************************************/

/* process the response received from the robot which
   encodes the state of the robot according to the mask */

static void Process_State_Pkg(unsigned char inbuf[BUFSIZE])
{
  int i, byte_count = 1;
  int low_half_used = FALSE;
  
  /* infrared */
  for (i = STATE_IR_0 ; i <= STATE_IR_15; i++)
    if (usedSmask[i] > 0) 
      {
	if (low_half_used == FALSE)
	  {
	    State[i] = low_half(inbuf[byte_count]);
	    low_half_used = TRUE;
	  }
	else
	  {
	    State[i] = high_half(inbuf[byte_count]);
	    byte_count++;
	    low_half_used = FALSE;
	  }
      }
  if (low_half_used == TRUE)
    byte_count++;
  
  /*
   * if the pos attachment was required we read it
   */
  
  if (POS_INFRARED_PI(usedSmask[SMASK_POS_DATA]))
    for (i = 0; i < INFRAREDS; i++)
      if (usedSmask[SMASK_IR_1 + i] > 0) 
	byte_count += posPackageProcess(&inbuf[byte_count], 
					&(posDataAll.infrared[i]));
  
  /* sonars */
  for (i = STATE_SONAR_0; i <= STATE_SONAR_15; i++) {
    if ( usedSmask[i] > 0 ) 
      {
	State[i] = inbuf[byte_count];
	byte_count++;
      }
  }
  
  /*
   * if the pos attachment was required we read it
   */
  
  if (POS_SONAR_PI(usedSmask[SMASK_POS_DATA]))
    for (i = 0; i < SONARS; i++) 
      if (usedSmask[SMASK_SONAR_1 + i] > 0)
	byte_count += posPackageProcess(&inbuf[byte_count], 
					&(posDataAll.sonar[i]));

  if (usedSmask[ SMASK_BUMPER ] > 0)
  {
    if (model == MODEL_SCOUT)
    {
      State[ STATE_BUMPER ] = combine_bumper_vector(inbuf[byte_count + 0], 
						    inbuf[byte_count + 1],
						    inbuf[byte_count + 2]);
    }
    else
    {
      State[ STATE_BUMPER ] = combine_bumper_vector(inbuf[byte_count + 2], 
						    inbuf[byte_count + 1],
						    inbuf[byte_count + 0]);
    }
    
    byte_count = byte_count + 3;
    
    /*
     * if the position attachment was requested for the bumper
     * we have to unpack the package. 
     */
    
    if (POS_BUMPER_PI(usedSmask[SMASK_POS_DATA]))
      byte_count += posPackageProcess(&inbuf[byte_count], 
				      &(posDataAll.bumper));
  }
  
  /* the position data */
  
  if (usedSmask[SMASK_CONF_X] > 0)
  {
    State[STATE_CONF_X] =  two_bytes_to_signed_int(inbuf[byte_count],
						   inbuf[byte_count+1]);
    byte_count = byte_count + 2;
  }
  
  if (usedSmask[SMASK_CONF_Y] > 0)
  {
    State[STATE_CONF_Y] = two_bytes_to_signed_int(inbuf[byte_count],
						  inbuf[byte_count+1]);
    byte_count = byte_count + 2;
  }
  
  if (usedSmask[SMASK_CONF_STEER] > 0)
  {
    State[STATE_CONF_STEER] = two_bytes_to_signed_int(inbuf[byte_count], 
						      inbuf[byte_count+1]);
    byte_count = byte_count + 2;
  }
  
  if (usedSmask[SMASK_CONF_TURRET] > 0)
  {
    State[STATE_CONF_TURRET] = two_bytes_to_signed_int(inbuf[byte_count], 
						       inbuf[byte_count+1]);
    byte_count = byte_count + 2;
  }
  
  /* the velocities */
  
  if (usedSmask[SMASK_VEL_TRANS] > 0)
  {
    State[STATE_VEL_TRANS] = two_bytes_to_signed_int(inbuf[byte_count], 
						     inbuf[byte_count+1]);
    byte_count = byte_count + 2;
  }
  
  if (usedSmask[SMASK_VEL_STEER] > 0)
  {
    State[SMASK_VEL_STEER] = two_bytes_to_signed_int(inbuf[byte_count],
						     inbuf[byte_count+1]);
    byte_count = byte_count + 2;
  }
  
  if (usedSmask[SMASK_VEL_TURRET] > 0)
  {
    State[STATE_VEL_TURRET] = two_bytes_to_signed_int(inbuf[byte_count],
						      inbuf[byte_count+1]);
    byte_count = byte_count + 2;
  }
  
  /* the compass value */

  if (usedSmask[SMASK_COMPASS] > 0)
  {
    State[STATE_COMPASS] = two_bytes_to_signed_int(inbuf[byte_count],
						   inbuf[byte_count+1]);
    byte_count = byte_count + 2;
      
    /*
     * if the position attachment was requested for the compass
     * we have to unpack the package. 
     */
    
    if (POS_COMPASS_PI(usedSmask[SMASK_POS_DATA]))
      byte_count += posPackageProcess(&inbuf[byte_count], 
				      &(posDataAll.compass));
  }

  /* laser */
  if (usedSmask[SMASK_LASER] > 0)
  {
    /* the number of points */
    Laser[0] = two_bytes_to_unsigned_int(inbuf[byte_count],
					 inbuf[byte_count+1]);
    byte_count = byte_count + 2;
    
    /* check the laser mode */
    if ((laser_mode&0x1e) == 0) /* Line mode */
    {
      if (Laser[0] > NUM_LASER/2)
      {
	printf("error in processing laser reply (1).\n");
	errorp = SERIAL_READ_ERROR;
	Laser[0] = 0;
	return;
      }
      for (i=1; i<=4*Laser[0]; i++) 
      {
	Laser[i] = two_bytes_to_signed_int(inbuf[byte_count],
					   inbuf[byte_count+1]);
	byte_count = byte_count+2;
      }
    }
    else /* Points of some kind */
    {
      if (Laser[0] > NUM_LASER) 
      {
	printf("error in processing laser reply (2).\n");
	errorp = SERIAL_READ_ERROR;
	Laser[0] = 0;
	return;
      }
      for (i=1; i<=Laser[0]; i++) 
      {
	Laser[i] = two_bytes_to_unsigned_int(inbuf[byte_count],
					     inbuf[byte_count+1]);
	byte_count = byte_count+2;
      }
    }
    if ((laser_mode&0x1e) == 19)
      convert_laser(Laser);
    
    /*
     * if the position attachment was requested for the laser
     * we have to get it from somewhere else 
     */
    
    if (POS_LASER_PI(usedSmask[SMASK_POS_DATA]))
      byte_count += posPackageProcess(&inbuf[byte_count], 
				      &(posDataAll.laser));
  }
  /* motor active */
  State[STATE_MOTOR_STATUS] = (long)inbuf[byte_count++];  

  /* process the 6811 time */
  byte_count += timePackageProcess(&inbuf[byte_count], &posDataTime);

  /* process the voltages of motor/CPU */
  byte_count += voltPackageProcess(&inbuf[byte_count], 
				   &voltageCPU, &voltageMotor);
}

/* process the response from the robot which encodes the
   active infrared reading */

static void Process_Infrared_Pkg(unsigned char inbuf[BUFSIZE])
{
  int i, byte_count = 1;
  int low_half_used = FALSE;

  /* 
   * the ir datum from one sensor is only a nibble, 
   * two of them are merged into one byte 
   */
  
  for (i = STATE_IR_0 ; i <=  STATE_IR_15; i++)
    if (low_half_used == FALSE) 
      {
	State[i] = low_half(inbuf[byte_count]);
	low_half_used = TRUE;
      }
    else 
      {
	State[i] = high_half(inbuf[byte_count]);
	byte_count++;
	low_half_used = FALSE;
      }

  /* align with next byte */
  if ( low_half_used )
    byte_count++;

  /*
   * if the pos attachment was required we read it
   */

  if ( POS_INFRARED_PI ( usedSmask[ SMASK_POS_DATA ] ) )
  {
    for (i=0; i<16; i++)
      byte_count += posPackageProcess ( &inbuf[byte_count], 
				        &( posDataAll.infrared[i] ) );
  }

  /* extract the time data for the 6811 */
  byte_count += timePackageProcess ( &inbuf[byte_count], &posDataTime );
}

/* process the response from the robot which encodes the
   active sonar reading */

static void Process_Sonar_Pkg(unsigned char inbuf[BUFSIZE])
{
  int i, byte_count = 1;

  /*
   * read the sensory data from the buffer
   */

  for (i = STATE_SONAR_0; i <= STATE_SONAR_15; i++) 
    {
      State[i] = inbuf[byte_count];
      byte_count++;
    }

  /*
   * if the pos attachment was required we read it
   */

  if ( POS_SONAR_PI ( usedSmask[ SMASK_POS_DATA ]) )
    for (i=0; i<16; i++) 
      byte_count += posPackageProcess ( &inbuf[byte_count], 
				        &( posDataAll.sonar[i] ) );
    
  /* extract the time data for the 6811 */
  byte_count += timePackageProcess ( &inbuf[byte_count], &posDataTime );
}

/* process the response from the robot which encodes the
   configuration of the robot */

static void Process_Configuration_Pkg(unsigned char inbuf[BUFSIZE])
{
  int byte_count = 1;
  
  State[ STATE_CONF_X ] = two_bytes_to_signed_int(inbuf[byte_count],
						  inbuf[byte_count+1]);
  byte_count = byte_count + 2;
  
  State[ STATE_CONF_Y ] = two_bytes_to_signed_int(inbuf[byte_count],
						  inbuf[byte_count+1]);
  byte_count = byte_count + 2;
  
  State[ STATE_CONF_STEER ] = two_bytes_to_signed_int(inbuf[byte_count],
						      inbuf[byte_count+1]);
  byte_count = byte_count + 2;
  
  State[ STATE_CONF_TURRET ] = two_bytes_to_signed_int(inbuf[byte_count],
						       inbuf[byte_count+1]);
}

static void Process_Velocity_Pkg(unsigned char inbuf[BUFSIZE])
{
  int byte_count = 1;
  
  State[ STATE_VEL_TRANS ] = two_bytes_to_signed_int(inbuf[byte_count],
						     inbuf[byte_count+1]);
  byte_count = byte_count + 2;
  
  State[ STATE_VEL_STEER ] = two_bytes_to_signed_int(inbuf[byte_count],
						     inbuf[byte_count+1]);
  byte_count = byte_count + 2;
  
  State[ STATE_VEL_TURRET ] = two_bytes_to_signed_int(inbuf[byte_count],
						      inbuf[byte_count+1]);
}

static void Process_Acceleration_Pkg(unsigned char inbuf[BUFSIZE] __attribute__ ((unused)))
{
}

/* process the response from the robot which encodes the
   compass reading of the robot */

static void Process_Compass_Pkg(unsigned char inbuf[BUFSIZE])
{
  int byte_count = 1;
  
  State[ STATE_COMPASS ] = two_bytes_to_unsigned_int(inbuf[byte_count],
						     inbuf[byte_count+1]);
  byte_count +=2;

  /*
   * if the position attachment was requested for the compass
   * we have to unpack the package. 
   */

  if ( POS_COMPASS_PI ( usedSmask[ SMASK_POS_DATA ] ) )
    byte_count += posPackageProcess ( &inbuf[byte_count], 
  				      &( posDataAll.compass ) );

  /* extract the time data for the 6811 */
  byte_count += timePackageProcess ( &inbuf[byte_count], &posDataTime );
}

/* process the response from the robot which encodes the
   compass reading of the robot */

static void Process_Compass_Conf_Pkg(unsigned char inbuf[BUFSIZE])
{
  int byte_count = 1;
  
  printf("compass calibration score x: %d y: %d z: %d\n",
	 inbuf[byte_count], 
	 inbuf[byte_count+1], 
	 inbuf[byte_count+2]);
}

/* process the response from the robot which encodes the
   bumper reading of the robot */

static void Process_Bumper_Pkg(unsigned char inbuf[BUFSIZE])
{
  int byte_count = 1;
  
  if (model == MODEL_SCOUT)
  {
    State[ STATE_BUMPER ] = combine_bumper_vector(inbuf[byte_count + 0], 
						  inbuf[byte_count + 1],
						  inbuf[byte_count + 2]);
  }
  else
  {
    State[ STATE_BUMPER ] = combine_bumper_vector(inbuf[byte_count + 2], 
						  inbuf[byte_count + 1],
						  inbuf[byte_count + 0]);
  }
  
  byte_count +=3;

  /*
   * if the position attachment was requested for the bumper
   * we have to unpack the package. 
   */

  if ( POS_BUMPER_PI ( usedSmask[ SMASK_POS_DATA ] ) )
    byte_count += posPackageProcess ( &inbuf[byte_count], 
  				      &( posDataAll.bumper ) );

  /* extract the time data for the 6811 */
  byte_count += timePackageProcess ( &inbuf[byte_count], &posDataTime );
}

static void Process_Laser_Point_Pkg(unsigned char inbuf[BUFSIZE])
{
  int i, byte_count = 1;
  
  Laser[0]=two_bytes_to_unsigned_int(inbuf[byte_count],inbuf[byte_count+1]);
  byte_count = byte_count+2;
  for (i=1; i<=Laser[0]; i++) {
    Laser[i] = two_bytes_to_signed_int(inbuf[byte_count], inbuf[byte_count+1]);
    byte_count = byte_count+2;
  }
  convert_laser(Laser);

  Laser[0] = inbuf[byte_count] + 256 * inbuf[byte_count+1];
  byte_count = byte_count + 2;
  
  if ( Laser[0] > NUM_LASER ) {
    printf("error in processing laser point reply\n");
    errorp = SERIAL_READ_ERROR;
    Laser[0] = 0;
    return;
  }
  for (i=1; i<=Laser[0]; i++) {
    Laser[i] = two_bytes_to_unsigned_int(inbuf[byte_count], 
					 inbuf[byte_count+1]);
    byte_count = byte_count+2;
  }
  if ((laser_mode == 51) || (laser_mode == 50) || (laser_mode == 19))
    convert_laser(Laser);

  /*
   * if the position attachment was requested for the laser
   * we have to unpack the package. 
   */

  if ( POS_LASER_PI ( usedSmask[ SMASK_POS_DATA ] ) )
    byte_count += posPackageProcess ( &inbuf[byte_count], 
  				      &( posDataAll.laser ) );

  /* extract the time data for the 6811 */
  byte_count += timePackageProcess ( &inbuf[byte_count], &posDataTime );
}

static void Process_Laser_Line_Pkg(unsigned char inbuf[BUFSIZE])
{ 
  int i, byte_count = 1;
  
  Laser[0] = inbuf[byte_count] + 256 * inbuf[byte_count+1];
  byte_count = byte_count + 2;

  if (Laser[0] > NUM_LASER) {
    printf("error in processing laser line reply\n");
    errorp = SERIAL_READ_ERROR;
    Laser[0] = 0;
    return;
  }
  for (i=1; i<=4*Laser[0]; i++) {
    Laser[i] = two_bytes_to_signed_int(inbuf[byte_count], 
				       inbuf[byte_count+1]);
    byte_count = byte_count+2;
  }

  /*
   * if the position attachment was requested for the laser
   * we have to unpack the package. 
   */

  if ( POS_LASER_PI ( usedSmask[ SMASK_POS_DATA ] ) )
    byte_count += posPackageProcess ( &inbuf[byte_count], 
  				      &( posDataAll.laser ) );

  /* extract the time data for the 6811 */
  byte_count += timePackageProcess ( &inbuf[byte_count], &posDataTime );
}

/* process the response from the robot which encodes special information */

static void Process_Special_Pkg(unsigned char inbuf[BUFSIZE])
{
  int data_size, i, byte_count = 1;
  
  data_size = two_bytes_to_unsigned_int(inbuf[1],inbuf[2]);
  special_answer_size = ( unsigned short ) data_size;

  if (data_size > MAX_USER_BUF) 
    data_size = MAX_USER_BUF;

  if ( special_buffer != (unsigned char *)NULL)
    {
      for (i=0; i<data_size; i++) 
	{
	  special_buffer[i] = inbuf[i+1];
	  byte_count++;
	}
    }
  else
    printf("Data buffer for user package is NULL pointer\n");
}

static int Process_Robot_Resp(unsigned char inbuf[BUFSIZE])
{
  switch (inbuf[0]) { /* type of the returned package */
  case AC:
  case SP:
  case PR:
  case PA:
  case VM:
  case MV:
  case CT:
  case GS:
  case ST:
  case LP:
  case DP:
  case DA:
  case WS:
  case ZR:
  case TK:
  case CONF_IR:
  case CONF_SN:
  case CONF_LS:
  case CONF_TM:
  case CONF_SG:
  case CONF_SER:
  case SETUP_LS:
    Process_State_Pkg(inbuf);
    break;
  case CONF_CP:
    Process_Compass_Conf_Pkg(inbuf);
    break;
  case NAK: /* Nak */
    printf("Nak\n");
    break;
  case GET_IR: /* Infrared */
    Process_Infrared_Pkg(inbuf);
    break;
  case GET_SN: /* Sonar */
    Process_Sonar_Pkg(inbuf);
    break;
  case GET_RC: /* Configuration */
    Process_Configuration_Pkg(inbuf);
    break;
  case GET_RV: /* Velocity */
    Process_Velocity_Pkg(inbuf);
    break;
  case GET_RA: /* Acceleration */
    Process_Acceleration_Pkg(inbuf);
    break;
  case GET_CP: /* Compass */
    Process_Compass_Pkg(inbuf);
    break;
  case GET_LS: /* Laser */
    Process_Laser_Point_Pkg(inbuf);
    break;
  case GET_BP: /* Bumper */
    Process_Bumper_Pkg(inbuf);
    break;
  case GET_SG: /* Laser line mode */
    Process_Laser_Line_Pkg(inbuf);
    break;
  case SPECIAL: /* User */
    Process_Special_Pkg(inbuf);
    break;
  default:
    printf("Invalid Robot Response\n");
    return(FALSE);
    break;
  }
  return(TRUE);
}

static int Comm_Robot(int fd, unsigned char command[BUFSIZE])
{
  unsigned char response[BUFSIZE];
  int respondedp;
  int re_xmitp, i;
  fd_set          lfdvar;
  struct timeval  timeout;

  if (fd == -1) {
    fprintf(stderr,
	    "Trying again to reestablish connection with the robot...\n"
	    "                          ");
    fflush(stderr);
    for (i = 0; (i < 2) &&
         (connect_robot(Robot_id, model, device, conn_value) == FALSE);
         i++)
    {
      sleep(5);
      fprintf(stderr, "Trying again...           ");
      fflush(stderr);
    }
    if (i == 2 &&
        connect_robot(Robot_id, model, device, conn_value) == FALSE)
    {
      fprintf(stderr, "Failed to reestablish connection.  Command aborted.\n");
      return FALSE;
    }
    fprintf(stderr, "Successful!  Continuing with command.\n");
  }
  re_xmitp = RE_XMIT;
  FD_ZERO(&lfdvar);
  FD_SET(fd, &lfdvar);
  
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;
  
  while (select(fd+1, &lfdvar, NULL, NULL, &timeout) != 0)
  {
    /* Flush buffer */
    respondedp = read(fd, response, BUFSIZE);
    /* Check for errors, such as lost connection. */
    if (respondedp <= 0 && errno != EWOULDBLOCK)
    {
      close(fd);
      Fd = -1;
      fprintf(stderr,
	      "Lost communication with robot.\nAttempting to reconnect...");
      fflush(stderr);
      for (i = 0; (i < 2) &&
           (connect_robot (Robot_id, model, device, conn_value) == FALSE);
           i++)
      {
	sleep(5);
	fprintf(stderr, "Trying again...           ");
	fflush(stderr);
      }
      if (i == 2 &&
          connect_robot (Robot_id, model, device, conn_value) == FALSE)
      {
	fprintf(stderr, "Unable to reconnect to robot.  Command aborted.\n");
	return FALSE;
      }
      else
      {
	fprintf(stderr, "Successful!  Continuing with command.\n");
      }
    }
  }
  
  Write_Pkg(fd, command);
  while (!(respondedp = Read_Pkg(fd, connect_type, response)) && (RE_XMIT)) {
    Write_Pkg(fd, command);
  }
  if (!respondedp) {
    printf("Last command packet transmitted:\n");
    for (i = 0; i < command[1]+4; i++)
      printf("%2.2x ", command[i]);
    printf("\n");

    return(FALSE);
  }
  else {
    if (Process_Robot_Resp (response)) {
      return(TRUE);
    }
    else {
      printf("error in robot response\n");
      return(FALSE); 
    }
  }
}

/************************************
 *                                  *
 * Robot Serial Interface Functions *
 *                                  *
 ************************************/

/* 
 * First some helper functions 
 */
void stuff_length_type(int length, int ptype, unsigned char *byte_ptr);
void stuff_length_type(int length, int ptype, unsigned char *byte_ptr)
{
     byte_ptr++; /* skip the first byte of the buffer, which is 
		    reserved for begin_pkg character */

     unsigned_int_to_two_bytes(length, byte_ptr);
     byte_ptr++; byte_ptr++;
     *byte_ptr = ptype;
}

void stuff_two_signed_int(int length, int ptype, int num1, int num2,
			  unsigned char *byte_ptr);
void stuff_two_signed_int(int length, int ptype, int num1, int num2,
			  unsigned char *byte_ptr)
{
     byte_ptr++; /* skip the first byte of the buffer, which is 
		    reserved for begin_pkg character */

     unsigned_int_to_two_bytes(length, byte_ptr);
     byte_ptr++; byte_ptr++;
     *byte_ptr = ptype;
     byte_ptr++;
     signed_int_to_two_bytes(num1, byte_ptr);
     byte_ptr++; byte_ptr++;
     signed_int_to_two_bytes(num2, byte_ptr);
}

void stuff_three_unsigned_int(int length, int ptype, int num1, int num2,
			      int num3, unsigned char *byte_ptr);
void stuff_three_unsigned_int(int length, int ptype, int num1, int num2,
			      int num3, unsigned char *byte_ptr)
{
     byte_ptr++; /* skip the first byte of the buffer, which is 
		    reserved for begin_pkg character */

     unsigned_int_to_two_bytes(length, byte_ptr);
     byte_ptr++; byte_ptr++;
     *byte_ptr = ptype;
     byte_ptr++;
     unsigned_int_to_two_bytes(num1, byte_ptr);
     byte_ptr++; byte_ptr++;
     unsigned_int_to_two_bytes(num2, byte_ptr);
     byte_ptr++; byte_ptr++;
     unsigned_int_to_two_bytes(num3, byte_ptr);
}

void stuff_three_signed_int(int length, int ptype, int num1, int num2,
			    int num3, unsigned char *byte_ptr);
void stuff_three_signed_int(int length, int ptype, int num1, int num2,
			    int num3, unsigned char *byte_ptr)
{
     byte_ptr++; /* skip the first byte of the buffer, which is 
		    reserved for begin_pkg character */

     unsigned_int_to_two_bytes(length, byte_ptr);
     byte_ptr++; byte_ptr++;
     *byte_ptr = ptype;
     byte_ptr++;
     signed_int_to_two_bytes(num1, byte_ptr);
     byte_ptr++; byte_ptr++;
     signed_int_to_two_bytes(num2, byte_ptr);
     byte_ptr++; byte_ptr++;
     signed_int_to_two_bytes(num3, byte_ptr);
}

/***************
 * FUNCTION:     posLongExtract
 * PURPOSE:      compose a long out of four bytes
 * ARGUMENTS:    unsigned char *inbuf : the pointer to the four bytes
 * ALGORITHM:    bit manipulation
 * RETURN:       long
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
static long posLongExtract( unsigned char *inbuf )
{
  long tmp;
  
  tmp = (long) LONG_B(inbuf[3],inbuf[2],inbuf[1],inbuf[0]);
  
  if ( tmp & (1L << 31) )
    return ( -(tmp & ~(1L << 31) ) );
  else
    return ( tmp );
}


/***************
 * FUNCTION:     posUnsignedLongExtract
 * PURPOSE:      compose an unsigned long out of four bytes
 * ARGUMENTS:    unsigned char *inbuf : the pointer to the four bytes
 * ALGORITHM:    bit manipulation
 * RETURN:       usigned long
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
static unsigned long posUnsignedLongExtract( unsigned char *inbuf )
{
  return ( (unsigned long) LONG_B(inbuf[3],inbuf[2],inbuf[1],inbuf[0]) );
}


/***************
 * FUNCTION:     posPackageProcess
 * PURPOSE:      processes the part of the package with pos information
 * ARGUMENTS:    unsigned char *inbuf : pointer to the data in chars
 *               PosData *posData : this is were the posData are written to
 * ALGORITHM:    regroup the bytes and assign variables
 * RETURN:       int (the number of bytes read from the buffer)
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
static int posPackageProcess ( unsigned char *inbuf, PosData *posData )
{
  int i = 0;

  /* copy the stuff from the buffer into the posData for the current robot */
  posData->config.configX      = posLongExtract(inbuf + i++ * sizeof(long));
  posData->config.configY      = posLongExtract(inbuf + i++ * sizeof(long));
  posData->config.configSteer  = posLongExtract(inbuf + i++ * sizeof(long));
  posData->config.configTurret = posLongExtract(inbuf + i++ * sizeof(long));
  posData->config.velTrans     = posLongExtract(inbuf + i++ * sizeof(long));
  posData->config.velSteer     = posLongExtract(inbuf + i++ * sizeof(long));
  posData->config.velTurret    = posLongExtract(inbuf + i++ * sizeof(long));
  posData->config.timeStamp    = posUnsignedLongExtract(inbuf + i++ * 
							sizeof(long));
  posData->timeStamp           = posUnsignedLongExtract(inbuf + i++ * 
							sizeof(long));
  
  return ( i * sizeof(long) );
}


/***************
 * FUNCTION:     timePackageProcess
 * PURPOSE:      processes the part of the package with the 6811 time
 * ARGUMENTS:    unsigned char *inbuf : pointer to the data in chars
 *               unsigned long *time : this is were the time is written to
 * ALGORITHM:    ---
 * RETURN:       static int 
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
static int timePackageProcess ( unsigned char *inbuf, unsigned long *timeS )
{
  *timeS = posUnsignedLongExtract( inbuf );

  return ( 4 );
}


/***************
 * FUNCTION:     voltPackageProcess
 * PURPOSE:      processes the part of the package with the voltages
 * ARGUMENTS:    unsigned char *inbuf : pointer to the data in chars
 *               unsigned long *time : this is were the time is written to
 * ALGORITHM:    ---
 * RETURN:       static int 
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
static int voltPackageProcess ( unsigned char *inbuf, 
			        unsigned char *voltCPU,
			        unsigned char *voltMotor)
{
  int i = 0;

  /* read the raw voltages out of the buffer */
  *voltCPU   = *(inbuf + i++);
  *voltMotor = *(inbuf + i++);
  
  return ( i );
}

/*****************************
 *                           *
 * Robot Interface Functions *
 *                           *
 *****************************/

/*
 * dummy function to maintain compatibility with Nclient
 *
 * create_robot - requests the server to create a robot with
 *                id = robot_id and establishes a connection with
 *                the robot. This function is disabled in this
 *                version of the software.
 * 
 * parameters:
 *    long robot_id -- id of the robot to be created. The robot
 *                     will be referred to by this id. If a process
 *                     wants to talk (connect) to a robot, it must
 *                     know its id.
 */
int create_robot ( long robot_id )
{
  Robot_id = robot_id;
  return ( TRUE );
}

/* Helper function for connect_robot */
static char *convertAddr ( char *name, char *addr )
{
  int addrInt[10];

  sscanf(name, "%d.%d.%d.%d", 
	 &(addrInt[0]), &(addrInt[1]), &(addrInt[2]), &(addrInt[3]));
  addr[0] = addrInt[0];
  addr[1] = addrInt[1];
  addr[2] = addrInt[2];
  addr[3] = addrInt[3];
  return ( addr );
}

int open_serial(char *port, unsigned short baud)
{
  struct termios info;  

  if (Fd != -1)
    close(Fd);
  if ((Fd=open(port, O_RDWR|O_NONBLOCK)) < 0)
    {
      perror("Error opening serial port");
      return 0;
    }
  
  if (tcgetattr(Fd, &info) < 0) 
    {
      perror("Error using TCGETS in ioctl.");
      close(Fd);
      Fd = -1;
      return 0;
    }
  
  /* restore old values to unhang the bastard, if hung */
  info.c_iflag=1280;
  info.c_oflag=5;
  info.c_cflag=3261;
  info.c_lflag=35387;
  
  if (tcsetattr(Fd, TCSANOW, &info) < 0) 
    { 
      perror("Error using TCSETS in ioctl.");
      close(Fd);
      Fd = -1;
      return 0;
    } 
  close(Fd);
  
  if ((Fd = open(port, O_RDWR)) == -1) {
    perror("Error opening serial port");
    errorp = SERIAL_OPEN_ERROR;
    return(FALSE);
  }
  
  if (tcgetattr(Fd,&info) < 0) {
    perror("Error using TCGETS in ioctl.");
    errorp = SERIAL_OPEN_ERROR;
    close(Fd);
    Fd = -1;
    return(FALSE);
  }
  
  if (baud != 4800 && baud != 9600 && baud != 19200 && baud != 38400)
  {
    if (baud != 0)
    {
      fprintf(stderr, "Invalid baud rate %d, using %d\n", baud,
              DEFAULT_SERIAL_BAUD);
    }
    baud = DEFAULT_SERIAL_BAUD;
  }
  
  info.c_iflag = 0;
  info.c_oflag = 0;
  info.c_lflag = 0;
  switch (baud) { /* serial port rate */
  case 4800:
    info.c_cflag = B4800 | CS8 | CREAD | CLOCAL;
    break;
  case 9600:
    info.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
    break;
  case 19200:
    info.c_cflag = B19200 | CS8 | CREAD | CLOCAL;
    break;
  case 38400:
    info.c_cflag = B38400 | CS8 | CREAD | CLOCAL;
    break;
  default:
    break;
  }
  /* set time out on serial read */
#if 1
  info.c_cc[VMIN] = 0;
  info.c_cc[VTIME] = 10;
#endif 
  wait_time = NORMAL_TIMEOUT;
  if (tcsetattr(Fd,TCSANOW,&info) < 0) { 
    perror("Error using TCSETS in ioctl.");
    errorp = SERIAL_OPEN_ERROR;
    close(Fd);
    Fd = -1;
    return(FALSE);
  }
  
  printf("Robot <-> Host serial communication setup\n");
  printf("(%d baud using %s)\n", baud, port);
  return(TRUE);

}

/*
 * connect_robot - requests the server to connect to the robot
 *                 with id = robot_id. In order to talk to the server,
 *                 the SERVER_MACHINE_NAME and SERV_TCP_PORT must be
 *                 set properly. If a robot with robot_id exists,
 *                 a connection is established with that robot. If
 *                 no robot exists with robot_id, no connection is
 *                 established. In this single robot version, the robot_id
 *                 is unimportant. You can call connect_robot with any
 *                 robot_id, it will connect to the robot.
 *
 * parameters:
 *    long robot_id -- robot's id. In this multiple robot version, in order
 *                     to connect to a robot, you must know it's id.
 *         model    -- robot type: 0 = Nomad 200, 1 = Nomad 150, 2 = Scout
 *         *dev     -- hostname for TCP, device file for serial ("/dev/" prefix
 *                     or ":" suffix means serial)
 *         conn     -- TCP port for TCP, baud rate for serial
 */
int connect_robot(long robot_id, ...)
{
  static char first = 1;
  struct hostent *hp;
  struct sockaddr_in serv_addr;
  int ret, retlen, i;
  unsigned char ir_mask[16],sn_mask[16],cf_mask[4],vl_mask[3];
  unsigned char cp_mask,bp_mask,ls_mask,pos_mask, byte;
  char addr[10];

  va_list args;
  
  if (first)
  {
    fprintf(stderr, "Ndirect version 2.6.13\n");
    fprintf(stderr, "Copyright 1991-1998, Nomadic Technologies, Inc.\n");
    first = 0;
  }
  
  va_start(args, robot_id);
  model = va_arg(args, int);
  device = va_arg(args, char *);
  conn_value = va_arg(args, int);
  if (strncmp(device, "/dev", 4) != 0 && device[strlen(device)-1] != ':')
  {
    connect_type = 2;
  }
  va_end(args);
  
  if (connect_type == 1) 
  {
    open_serial(device, conn_value);
    
    /* Send init_sensors to make sure that server and robot are synchronized */
    if (model == MODEL_N200)
    {
      init_sensors();
    }
    else
    {
      usedSmask[0] = 0;
      /* IR */
      for (i = 1; i < 17; i++)
	usedSmask[i] = 0;
      /* Sonar */
      for (i = 17; i < 33; i++)
	usedSmask[i] = 1;
      /* Bumper */
      usedSmask[33] = 1;
      /* Conf */
      for (i = 34; i < 38; i++)
	usedSmask[i] = 1;
      /* Velocity */
      for (i = 38; i < 41; i++)
	usedSmask[i] = 1;
      /* Motor */
      usedSmask[41] = 1;
      /* Laser */
      usedSmask[42] = 0;
      /* Compass */
      usedSmask[43] = 1;
    }
  } else {
    if (device[0] == 0)
      device = "localhost";
    if ( ((hp = gethostbyname(device)) == NULL))
    {
      convertAddr(device, addr);
      if (addr[0] != 0 || addr[1] != 0 || addr[2] != 0 || addr[3] != 0)
      {
	memset((char *) &serv_addr, 0, sizeof(serv_addr));
	memcpy((char *) &(serv_addr.sin_addr), addr, 4);
      }
      else
      {
	fprintf(stderr, "Machine %s not valid.\n", device);
	return FALSE;
      }
    }
    else
    {
      memset((char *) &serv_addr, 0, sizeof(serv_addr));
      memcpy((char *) &(serv_addr.sin_addr), hp->h_addr, hp->h_length);
    }
    
    serv_addr.sin_family = AF_INET;            /* address family */
    
    /* TCP port number */
    if (conn_value == 0)
    {
      conn_value = DEFAULT_ROBOT_TCP_PORT;
    }
    serv_addr.sin_port = htons(conn_value);
    
    if ((Fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      fprintf(stderr, "Error: in open_socket_to_send_data, socket failed.\n");
      return FALSE;
    }
    fcntl(Fd, F_SETFL, O_NDELAY);
    if (connect(Fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
      if (errno == EINPROGRESS) {
        fd_set          lfdvar;
        struct timeval  timeout;
        
        FD_ZERO(&lfdvar);
        FD_SET(Fd, &lfdvar);
        
        timeout.tv_sec = (long)CONNECT_TIMEOUT;
        timeout.tv_usec = (long)0;
        
        if (select(Fd+1, NULL, &lfdvar, NULL, &timeout) == 0) {
          fprintf(stderr, "Error: connect timed out.\n");
          close(Fd);
	  Fd = -1;
          return FALSE;
        } else {
	  errno = 0;
	  retlen = 4;
	  if (getsockopt(Fd, SOL_SOCKET, SO_ERROR, (char *)&ret, (unsigned int *) &retlen) == 0)
	  {
	    if (ret != 0)
	      errno = ret;
	    if (errno != 0)
	    {
	      perror("Error: connect failed");
	      close(Fd);
	      Fd = -1;
	      return FALSE;
	    }
	  }
	}
      } else {
        perror("Error: connect failed");
        close(Fd);
	Fd = -1;
        return FALSE;
      }
    }
    
    wait_time = NORMAL_TIMEOUT;
    
    printf("Robot <-> Host TCP/IP communication setup\n");
    printf("(machine %s on port %d)\n", device, conn_value);
    
    /* Read configuration data */
    if (model == MODEL_N200)
    {
      byte = GETC(Fd, connect_type);
      ir_mask[ 0] = byte & (1 << 0) ? 1 : 0;
      ir_mask[ 1] = byte & (1 << 1) ? 1 : 0;
      ir_mask[ 2] = byte & (1 << 2) ? 1 : 0;
      ir_mask[ 3] = byte & (1 << 3) ? 1 : 0;
      ir_mask[ 4] = byte & (1 << 4) ? 1 : 0;
      ir_mask[ 5] = byte & (1 << 5) ? 1 : 0;
      ir_mask[ 6] = byte & (1 << 6) ? 1 : 0;
      ir_mask[ 7] = byte & (1 << 7) ? 1 : 0;
      byte = GETC(Fd, connect_type);
      ir_mask[ 8] = byte & (1 << 0) ? 1 : 0;
      ir_mask[ 9] = byte & (1 << 1) ? 1 : 0;
      ir_mask[10] = byte & (1 << 2) ? 1 : 0;
      ir_mask[11] = byte & (1 << 3) ? 1 : 0;
      ir_mask[12] = byte & (1 << 4) ? 1 : 0;
      ir_mask[13] = byte & (1 << 5) ? 1 : 0;
      ir_mask[14] = byte & (1 << 6) ? 1 : 0;
      ir_mask[15] = byte & (1 << 7) ? 1 : 0;
      byte = GETC(Fd, connect_type);
      sn_mask[ 0] = byte & (1 << 0) ? 1 : 0;
      sn_mask[ 1] = byte & (1 << 1) ? 1 : 0;
      sn_mask[ 2] = byte & (1 << 2) ? 1 : 0;
      sn_mask[ 3] = byte & (1 << 3) ? 1 : 0;
      sn_mask[ 4] = byte & (1 << 4) ? 1 : 0;
      sn_mask[ 5] = byte & (1 << 5) ? 1 : 0;
      sn_mask[ 6] = byte & (1 << 6) ? 1 : 0;
      sn_mask[ 7] = byte & (1 << 7) ? 1 : 0;
      byte = GETC(Fd, connect_type);
      sn_mask[ 8] = byte & (1 << 0) ? 1 : 0;
      sn_mask[ 9] = byte & (1 << 1) ? 1 : 0;
      sn_mask[10] = byte & (1 << 2) ? 1 : 0;
      sn_mask[11] = byte & (1 << 3) ? 1 : 0;
      sn_mask[12] = byte & (1 << 4) ? 1 : 0;
      sn_mask[13] = byte & (1 << 5) ? 1 : 0;
      sn_mask[14] = byte & (1 << 6) ? 1 : 0;
      sn_mask[15] = byte & (1 << 7) ? 1 : 0;
      byte = GETC(Fd, connect_type);
      bp_mask    = byte & (1 << 0) ? 1 : 0;
      cf_mask[0] = byte & (1 << 1) ? 1 : 0;
      cf_mask[1] = byte & (1 << 2) ? 1 : 0;
      cf_mask[2] = byte & (1 << 3) ? 1 : 0;
      cf_mask[3] = byte & (1 << 4) ? 1 : 0;
      vl_mask[0] = byte & (1 << 5) ? 1 : 0;
      vl_mask[1] = byte & (1 << 6) ? 1 : 0;
      vl_mask[2] = byte & (1 << 7) ? 1 : 0;
      byte = GETC(Fd, connect_type);
      cp_mask = byte & 1;
      byte = GETC(Fd, connect_type);
      ls_mask = byte & 1;
      pos_mask = byte >> 1;
      
      usedSmask[0] = pos_mask;
      for (i = 0; i < 16; i++)
	usedSmask[i+1] = ir_mask[i];
      for (i = 0; i < 16; i++)
	usedSmask[i+17] = sn_mask[i];
      usedSmask[33] = bp_mask;
      for (i = 0; i < 4; i++)
	usedSmask[i+34] = cf_mask[i];
      for (i = 0; i < 3; i++)
	usedSmask[i+38] = vl_mask[i];
      usedSmask[42] = ls_mask;
      usedSmask[43] = cp_mask;
      
      /* get laser mode, num_points, processing */
      byte = GETC(Fd, connect_type);
      laser_mode = byte;
      byte = GETC(Fd, connect_type);
      byte = GETC(Fd, connect_type);
      byte = GETC(Fd, connect_type);
    }
    else
    {
      usedSmask[0] = 0;
      /* IR */
      for (i = 1; i < 17; i++)
	usedSmask[i] = 0;
      /* Sonar */
      for (i = 17; i < 33; i++)
	usedSmask[i] = 1;
      /* Bumper */
      usedSmask[33] = 1;
      /* Conf */
      for (i = 34; i < 38; i++)
	usedSmask[i] = 1;
      /* Velocity */
      for (i = 38; i < 41; i++)
	usedSmask[i] = 1;
      /* Motor */
      usedSmask[41] = 1;
      /* Laser */
      usedSmask[42] = 0;
      /* Compass */
      usedSmask[43] = 1;
    }
  }
  
  return TRUE;
}

/*
 * dummy function to maintain compatibility with Nclient
 *
 * disconnect_robot - requests the server to close the connect with robot
 *                    with id = robot_id. 
 *
 * parameters:
 *    long robot_id -- robot's id. In order to disconnect a robot, you
 *                     must know it's id.
 */
int disconnect_robot(long robot_id __attribute__ ((unused)))
{
  Robot_id = -1;
  return ( TRUE );
}

/* 
 * ac - sets accelerations of the robot. Currently it has no effect in 
 *      simulation mode.
 *
 * parameters:
 *    int t_ac, s_ac, r_ac -- the translation, steering, and turret
 *                            accelerations. t_ac is in 1/10 inch/sec^2
 *                            s_ac and r_ac are in 1/10 degree/sec^2.
 */
int ac(int t_ac, int s_ac, int r_ac)
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_three_unsigned_int(8, AC, t_ac, s_ac, r_ac, outbuf);
  return(Comm_Robot(Fd, outbuf));  
}

/*
 * sp - sets speeds of the robot, this function will not cause the robot to
 *      move. However, the set speed will be used when executing a pr()
 *      or a pa().
 *
 * parameters:
 *    int t_sp, s_sp, r_sp -- the translation, steering, and turret
 *                            speeds. t_sp is in 1/10 inch/sec and
 *                            s_sp and r_sp are in 1/10 degree/sec.
 */
int sp(int t_sp, int s_sp, int r_sp)
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_three_unsigned_int(8, SP, t_sp, s_sp, r_sp, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * pr - moves the motors of the robot by a relative distance, using the speeds
 *      set by sp(). The three parameters specify the relative distances for
 *      the three motors: translation, steering, and turret. All the three
 *      motors move concurrently if the speeds are not set to zero and the 
 *      distances to be traveled are non-zero. Depending on the timeout 
 *      period set (by function conf_tm(timeout)), the motion may 
 *      terminate before the robot has moved the specified distances
 *
 * parameters:
 *    int t_pr, s_pr, r_pr -- the specified relative distances of the
 *                            translation, steering, and turret motors.
 *                            t_pr is in 1/10 inch and s_pr and r_pr are
 *                            in 1/10 degrees.
 */
int pr(int t_pr, int s_pr, int r_pr)
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_three_signed_int(8, PR, t_pr, s_pr, r_pr, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * pa - moves the motors of the robot to the specified absolute positions 
 *      using the speeds set by sp().  Depending on the timeout period set 
 *      (by conf_tm()), the motion may terminate before the robot has 
 *      moved to the specified positions.
 *
 * parameters:
 *    int t_pa, s_pa, r_pa -- the specified absolute positions of the
 *                            translation, steering, and turret motors.
 *                            t_pa is in 1/10 inch and s_pa and r_pa are
 *                            in 1/10 degrees.
 */
int pa(int t_pa, int s_pa, int r_pa);
int pa(int t_pa, int s_pa, int r_pa)
{
  unsigned char outbuf[BUFSIZE];
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_three_signed_int(8, PA, t_pa, s_pa, r_pa, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * vm - velocity mode, command the robot to move at translational
 *      velocity = tv, steering velocity = sv, and rotational velocity =
 *      rv. The robot will continue to move at these velocities until
 *      either it receives another command or this command has been
 *      timeout (in which case it will stop its motion).
 *
 * parameters: 
 *    int t_vm, s_vm, r_vm -- the desired translation, steering, and turret
 *                            velocities. tv is in 1/10 inch/sec and
 *                            sv and rv are in 1/10 degree/sec.
 */
int vm(int t_v, int s_v, int r_v)
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_three_signed_int(8, VM, t_v, s_v, r_v, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * mv - move, send a generalized motion command to the robot.
 *      For each of the three axis (translation, steering, and
 *      turret) a motion mode (t_mode, s_mode, r_mode) can be 
 *      specified (using the values MV_IGNORE, MV_AC, MV_SP,
 *      MV_LP, MV_VM, and MV_PR defined above):
 *
 *         MV_IGNORE : the argument for this axis is ignored
 *                     and the axis's motion will remain 
 *                     unchanged.
 *         MV_AC :     the argument for this axis specifies
 *                     an acceleration value that will be used
 *                     during motion commands.
 *         MV_SP :     the argument for this axis specifies
 *                     a speed value that will be used during
 *                     position relative (PR) commands.
 *         MV_LP :     the arguemnt for this axis is ignored
 *                     but the motor is turned off.
 *         MV_VM :     the argument for this axis specifies
 *                     a velocity and the axis will be moved
 *                     with this velocity until a new motion
 *                     command is issued (vm,pr,mv) or 
 *                     recieves a timeout.
 *         MV_PR :     the argument for this axis specifies
 *                     a position and the axis will be moved
 *                     to this position, unless this command
 *                     is overwritten by another (vm,pr,mv).
 *
 * parameters: 
 *    int t_mode - the desired mode for the tranlation axis
 *    int t_mv   - the value for that axis, velocity or position,
 *                 depending on t_mode
 *    int s_mode - the desired mode for the steering axis
 *    int s_mv   - the value for that axis, velocity or position,
 *                 depending on t_mode
 *    int r_mode - the desired mode for the turret axis
 *    int r_mv   - the value for that axis, velocity or position,
 *                 depending on t_mode
 */
int mv(int t_mode, int t_mv, int s_mode, 
       int s_mv, int r_mode, int r_mv) 
{
  unsigned char outbuf[BUFSIZE];
  unsigned char *ptr;

  ptr = outbuf;
  
  /* skip the first byte for begin package */
  ptr++;

  /* the length into the packet (# of ints * sizeof(int)) */
  unsigned_int_to_two_bytes ( 6 * 2 + 2, ptr ); ptr += 2;

  /* the packet type */
  *(ptr++) = MV;

  /* translational axis - mode and value */
  unsigned_int_to_two_bytes ( t_mode , ptr ); ptr += 2;
  signed_int_to_two_bytes   ( t_mv   , ptr ); ptr += 2;

  /* steering axis - mode and value */
  unsigned_int_to_two_bytes ( s_mode , ptr ); ptr += 2;
  signed_int_to_two_bytes   ( s_mv   , ptr ); ptr += 2;

  /* turret  axis - mode and value */
  unsigned_int_to_two_bytes ( r_mode , ptr ); ptr += 2;
  signed_int_to_two_bytes   ( r_mv   , ptr ); ptr += 2;

  return ( Comm_Robot(Fd, outbuf) );
}

/*
 * ct - send the sensor mask, Smask, to the robot. You must first change
 *      the global variable Smask to the desired communication mask before
 *      calling this function. 
 *
 *      to avoid inconsistencies usedSmask is used in all other function.
 *      once ct is called the user accessible mask Smask is used to 
 *      redefine usedSmask. This avoids that a returning package is encoded
 *      with a different mask than the one it was sent with, in case
 *      the mask has been changed on the client side, but has not been 
 *      updated on the server side.
 */
int ct()
{
  int i;
  unsigned char b0, b1, b2, b3, b4, b5, b6;
  unsigned char outbuf[BUFSIZE], *byte_ptr;

  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  for ( i = 0; i < NUM_MASK; i++ )
    usedSmask[i] = Smask[i];
  
  /* first encode Mask */
  b0 = bits_to_byte (Smask[1], Smask[2], Smask[3], Smask[4],
		     Smask[5], Smask[6], Smask[7], Smask[8]);
  b1 = bits_to_byte (Smask[9], Smask[10], Smask[11], Smask[12],
		     Smask[13], Smask[14], Smask[15], Smask[16]);
  b2 = bits_to_byte (Smask[17], Smask[18], Smask[19], Smask[20],
		     Smask[21], Smask[22], Smask[23], Smask[24]);
  b3 = bits_to_byte (Smask[25], Smask[26], Smask[27], Smask[28],
		     Smask[29], Smask[30], Smask[31], Smask[32]);
  b4 = bits_to_byte (Smask[33], Smask[34], Smask[35], Smask[36],
		     Smask[37], Smask[38], Smask[39], Smask[30]);
  b5 = bits_to_byte (Smask[42], 0, 0, 0, 0, 0, 0, 0);
  /* we pack the pos mask into b6 */
  b6 = bits_to_byte(Smask[43], 
		    POS_INFRARED_PI(Smask[ SMASK_POS_DATA ]),
		    POS_SONAR_PI   (Smask[ SMASK_POS_DATA ]), 
		    POS_BUMPER_PI  (Smask[ SMASK_POS_DATA ]), 
		    POS_LASER_PI   (Smask[ SMASK_POS_DATA ]), 
		    POS_COMPASS_PI (Smask[ SMASK_POS_DATA ]),
		    0,0);
  
  stuff_length_type (9, CT, outbuf);
  byte_ptr = outbuf + 4;
  *byte_ptr = b0;
  byte_ptr++;
  *byte_ptr = b1;
  byte_ptr++;
  *byte_ptr = b2;
  byte_ptr++;
  *byte_ptr = b3;
  byte_ptr++;
  *byte_ptr = b4;
  byte_ptr++;
  *byte_ptr = b5;
  byte_ptr++;
  *byte_ptr = b6;
  return(Comm_Robot(Fd, outbuf));
}

/*
 * gs - get the current state of the robot according to the mask (of 
 *      the communication channel)
 */
int gs()
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, GS, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * st - stops the robot (the robot holds its current position)
 */
int st()
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, ST, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * lp - set motor limp (the robot may not hold its position).
 */
int lp ()
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, LP, outbuf);
  return(Comm_Robot(Fd, outbuf));
}


/*
 * tk - sends the character stream, talk_string, to the voice synthesizer
 *      to make the robot talk.
 *
 * parameters:
 *    char *talk_string -- the string to be sent to the synthesizer.
 */
int tk(char *talk_string)
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  int tkfd, i, length;
  
  if (model == MODEL_N200)
  {
    length = 3 + strlen(talk_string);
    stuff_length_type (length, TK, outbuf);
    byte_ptr = outbuf + 4;
    for (i=3; i<length; i++) {
      *byte_ptr = talk_string[i-3];
      byte_ptr++;
    }
    *byte_ptr = 0; /* null terminate the string */
    return(Comm_Robot(Fd, outbuf));
  }
  else
  {
    tkfd = open("/dev/dbtk", O_RDWR);
    if (tkfd >= 0)
    {
      write(tkfd, talk_string, strlen(talk_string));
      write(tkfd, "\n", 1);
      close(tkfd);
      
      return TRUE;
    }
  }
  
  return FALSE;
}

/*
 * dp - define the current position of the robot as (x,y)
 * 
 * parameters:
 *    int x, y -- the position to set the robot to.
 */
int dp(int x, int y)
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_two_signed_int (6, DP, x, y, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * zr - zeroing the robot, align steering and turret with bumper zero.
 *      The position, steering and turret angles are all set to zero.
 *      This function returns when the zeroing process has completed.
 */
int zr()
{
  unsigned char outbuf[BUFSIZE];
  int temp;
  
  wait_time =  NORMAL_TIMEOUT;
  /* zr() takes maximum of 120 seconds */
  stuff_length_type (2, ZR, outbuf);
  temp = Comm_Robot(Fd, outbuf);
  return(temp);
}

/*
 * conf_ir - configure infrared sensor system.
 *
 * parameters: 
 *    int history -- specifies the percentage dependency of the current 
 *                   returned reading on the previous returned reading.
 *                   It should be set between 0 and 10: 0 = no dependency 
 *                   10 = full dependency, i.e. the reading will not change
 *    int order[16] --  specifies the firing sequence of the infrared 
 *                      (#0 .. #15). You can terminate the order list by a 
 *                      "255". For example, if you want to use only the 
 *                      front three infrared sensors then set order[0]=0,
 *                      order[1]=1, order[2]=15, order[3]=255 (terminator).
 */
int conf_ir(int history, int order[16])
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  int i;
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_length_type (19, CONF_IR, outbuf);
  byte_ptr = outbuf + 4;
  if (history > 10)
    history = 10;
  *byte_ptr = (unsigned char)history;
  for (i=0; i<16; i++) {
    byte_ptr++;
    *byte_ptr = (unsigned char)order[i];
  }
  return(Comm_Robot(Fd, outbuf));
}

/*
 * conf_sn - configure sonar sensor system.
 *
 * parameters:
 *    int rate -- specifies the firing rate of the sonar in 4 milli-seconds 
 *                interval; 
 *    int order[16] -- specifies the firing sequence of the sonar (#0 .. #15).
 *                     You can terminate the order list by a "255". For 
 *                     example, if you want to use only the front three 
 *                     sensors, then set order[0]=0, order[1]=1, order[2]=15, 
 *                     order[3]=255 (terminator).
 */
int conf_sn(int rate, int order[16])
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  int i;
  
  stuff_length_type (19, CONF_SN, outbuf);
    byte_ptr = outbuf + 4;
    *byte_ptr = (unsigned char)rate;
  for (i=0; i<16; i++) {
    byte_ptr++;
    *byte_ptr = (unsigned char)order[i];
  }
  return(Comm_Robot(Fd, outbuf));
}

/*
 * conf_cp - configure compass system.
 * 
 * parameters:
 *    int mode -- specifies compass on/off: 0 = off ; 1 = on; 2 = calibrate.
 *                When you call conf_cp (2), the robot will rotate slowly 360
 *                degrees. You must wake till the robot stop rotating before
 *                issuing another command to the robot (takes ~3 minutes).
 */
int conf_cp(int mode)
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_length_type (3, CONF_CP, outbuf);
  byte_ptr = outbuf + 4;
  *byte_ptr = (unsigned char)mode;
  return(Comm_Robot(Fd, outbuf));
}

/*
 * conf_ls - configure laser sensor system:
 *
 * parameters:
 *    unsigned int mode -- specifies the on-board processing mode of the laser 
 *                         sensor data which determines the mode of the data 
 *                         coming back: 
 *                           the first bit specifies the on/off;
 *                           the second bit specifies point/line mode;
 *                           the third to fifth bits specify the 
 *                           returned data types: 
 *                             000 = peak pixel, 
 *                             001 = rise pixel, 
 *                             010 = fall pixel, 
 *                             011 = magnitude,
 *                             100 = distance;
 *                           the sixth bit specifies data integrity checking.
 *
 *   unsigned int threshold -- specifies the inverted acceptable brightness
 *                             of the laser line. 
 *
 *   unsigned int width -- specifies the acceptable width in terms
 *                         of number of pixels that are brighter than the 
 *                         set threshold.
 *  
 *   unsigned int num_data -- specifies the number of sampling points. 
 *   unsigned int processing --  specifies the number of neighboring 
 *                               pixels for averaging
 *
 * If you don't understand the above, try this one:
 *   conf_ls(51, 20, 20, 20, 4)
 */
int conf_ls(unsigned int mode, unsigned int threshold, unsigned int width,
	    unsigned int num_data, unsigned int processing)
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  laser_mode = mode;
  stuff_length_type (8, CONF_LS, outbuf);
  byte_ptr = outbuf + 4;
  if (mode == 51) 
    *byte_ptr = 35; /* special case */
  else
    *byte_ptr = (unsigned char)mode;
  byte_ptr++;
  *byte_ptr = (unsigned char)threshold;
  byte_ptr++;
  *byte_ptr = (unsigned char)width;
  byte_ptr++;
  unsigned_int_to_two_bytes(num_data, byte_ptr);
  byte_ptr = byte_ptr + 2;
  *byte_ptr = (unsigned char)processing;
  return(Comm_Robot(Fd, outbuf));
}

/*
 * conf_tm - sets the timeout period of the robot in seconds. If the
 *           robot has not received a command from the host computer
 *           for more than the timeout period, it will abort its 
 *           current motion
 * 
 * parameters:
 *    unsigned int timeout -- timeout period in seconds. If it is 0, there
 *                            will be no timeout on the robot.
 */
int conf_tm(unsigned char timeout)
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  
  stuff_length_type (3, CONF_TM, outbuf);
  byte_ptr = outbuf + 4;
  *byte_ptr = (unsigned char)timeout;
  return(Comm_Robot(Fd, outbuf));
}

int conf_ser(unsigned char port, unsigned short baud)
{
  unsigned char *outbuf, buffer[BUFSIZE];
  
  if (model == MODEL_N200)
  {
    return FALSE;
  }
  
  outbuf=buffer;
  stuff_length_type(5, CONF_SER, outbuf);
  outbuf+=4;
  *outbuf=port;
  outbuf++;
  unsigned_int_to_two_bytes(baud, outbuf);
  return(Comm_Robot(Fd, buffer));
}

/*
 * get_ir - get infrared data, independent of mask. However, only 
 *          the active infrared sensor readings are valid. It updates
 *          the State vector.
 */
int get_ir()
{
  unsigned char outbuf[BUFSIZE];
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_length_type (2, GET_IR, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_sn - get sonar data, independent of mask. However, only 
 *          the active sonar sensor readings are valid. It updates
 *          the State vector.
 */
int get_sn()
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, GET_SN, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_rc - get robot configuration data (x, y, th, tu), independent of 
 *          mask. It updates the State vector.
 */
int get_rc() 
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, GET_RC, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_rv - get robot velocities (translation, steering, and turret) data,
 *          independent of mask. It updates the State vector.
 */
int get_rv() 
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, GET_RV, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_ra - get robot acceleration (translation, steering, and turret) data,
 *          independent of mask. It updates the State vector.
 */
int get_ra() 
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, GET_RA, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_cp - get compass data, independent of mask. However, the
 *          data is valid only if the compass is on. It updates the
 *          State vector.
 */
int get_cp()
{
  unsigned char outbuf[BUFSIZE];
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_length_type (2, GET_CP, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_ls - get laser data point mode, independent of mask. However the
 *          data is valid only of the laser is on. It updates the Laser 
 *          vector.
 */
int get_ls()
{
  unsigned char outbuf[BUFSIZE];
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_length_type (2, GET_LS, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_bp - get bumper data, independent of mask. It updates the State
 *          vector.
 */
int get_bp()
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_length_type (2, GET_BP, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * conf_sg - configure laser sensor system line segment processing mode:
 *
 * parameters:
 *    unsigned int threshold -- specifies the threshold value for least-square
 *                             fitting. When the error term grows above the 
 *                             threshold, the line segment will be broken
 *    unsigned int min_points -- specifies the acceptable number of points
 *                              to form a line segment.
 *    unsigned int gap -- specifies the acceptable "gap" between two segments
 *                        while they can still be treated as one (in 1/10 inch)
 *
 * If you don't understand the above, try this one:
 *    conf_sg(50, 4, 30)
 */
int conf_sg(unsigned int threshold, unsigned int min_points, unsigned int gap)
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_length_type (5, CONF_SG, outbuf);
  byte_ptr = outbuf + 4;
  *byte_ptr = (unsigned char)threshold;
  byte_ptr++;
  *byte_ptr = (unsigned char)min_points;
  byte_ptr++;
  *byte_ptr = (unsigned char)gap;
  return(Comm_Robot(Fd, outbuf));
}

/*
 * get_sg - get laser data line mode, independent of mask. It updates
 *          the laser vector.
 */
int get_sg()
{
  unsigned char outbuf[BUFSIZE];
  
  if (model != MODEL_N200)
  {
    return FALSE;
  }
  
  stuff_length_type (2, GET_SG, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * da - define the current steering angle of the robot to be th
 *      and the current turret angle of the robot to be tu.
 * 
 * parameters:
 *    int th, tu -- the steering and turret orientations to set the
 *                  robot to.
 */
int da(int th, int tu)
{
  unsigned char outbuf[BUFSIZE];
  
  stuff_two_signed_int (6, DA, th, tu, outbuf);
  return(Comm_Robot(Fd, outbuf));
}

/*
 * ws - waits for stop of motors of the robot. This function is intended  
 *      to be used in conjunction with pr() and pa() to detect the desired
 *      motion has finished
 *
 * parameters:
 *    unsigned char t_ws, s_ws, r_ws -- These three parameters specify 
 *                                      which axis or combination of axis 
 *                                      (translation, steering, and turret) 
 *                                      to wait. 
 *    unsigned char timeout -- specifies how long to wait before timing out 
 *                             (return without stopping the robot).
 */
int ws(unsigned char t_ws, unsigned char s_ws,
       unsigned char r_ws, unsigned char timeout)
{
  unsigned char outbuf[BUFSIZE], *byte_ptr;
  int ret;
  
  stuff_length_type (6, WS, outbuf);
  byte_ptr = outbuf + 4;
  if (t_ws > 1) t_ws = 1;
  *byte_ptr = t_ws;
  byte_ptr++;
  if (s_ws > 1) s_ws = 1;
  *byte_ptr = s_ws;
  byte_ptr++;
  if (r_ws > 1) r_ws = 1;
  *byte_ptr = r_ws;
  byte_ptr++;
  *byte_ptr = timeout;
  
  wait_time =  timeout + NORMAL_TIMEOUT;
  ret = Comm_Robot(Fd, outbuf);
  wait_time = NORMAL_TIMEOUT;
  
  return ret;
}

/*
 * dummy function to maintain compatibility with Nclient
 *
 * get_rpx - get the position of all nearby robots
 */
int get_rpx(long *robot_pos)
{
  *robot_pos = -1;

  return ( FALSE );
}

/*****************************
 *                           *
 * World Interface Functions *
 *                           *
 * all dummy: compatibility  *
 *****************************/
/*
 * add_obstacle - creates an obstacle and adds it to the obstacle list
 *                of the robot environment. 
 * 
 * parameters:
 *    long obs[2*MAX_VERTICES+1] -- 
 *                The first element of obs specifies the number of 
 *                vertices of the polygonal obstacle (must be no greater 
 *                than MAX_VERTICES). The subsequent elements of obs 
 *                specifies the x and y coordinates of the vertices, 
 *                in counter-clockwise direction.
 */
int add_obstacle(long obs[2*MAX_VERTICES+1])
{
  obs[0] = obs[0];
  return ( TRUE );
}

/*
 * add_Obs - is the same as add_obstacle, for backward compatibility
 */
int add_Obs(long obs[2*MAX_VERTICES+1])
{
  return(add_obstacle(obs));
}

/*
 * delete_obstacle - deletes an obstacle specified by obs from the robot 
 *                   environment 
 * parameters:
 *    long obs[2*MAX_VERTICES+1] -- 
 *                The first element of obs specifies the number of 
 *                vertices of the polygonal obstacle (must be no greater 
 *                than MAX_VERTICES). The subsequent elements of obs 
 *                specifies the x and y coordinates of the vertices, 
 *                in counter-clockwise direction.
 */
int delete_obstacle(long obs[2*MAX_VERTICES+1])
{
  obs[0] = obs[0];
  return ( TRUE );
}

/*
 * delete_Obs - is the same as delete_obstacle, for backward compatibility
 */
int delete_Obs(long obs[2*MAX_VERTICES+1])
{
  return(delete_obstacle(obs));
}

/*
 * move_obstacle - moves the obstacle obs by dx along x direction and 
 *                 dy along y direction. obs is modified.
 *
 * parameters:
 *    long obs[2*MAX_VERTICES+1] -- 
 *                The first element of obs specifies the number of 
 *                vertices of the polygonal obstacle (must be no greater 
 *                than MAX_VERTICES). The subsequent elements of obs 
 *                specifies the x and y coordinates of the vertices, 
 *                in counter-clockwise direction.
 *    long dx, dy -- the x and y distances to translate the obstacle
 */
int move_obstacle(long obs[2*MAX_VERTICES+1], long dx, long dy)
{
  obs[0] = obs[0];
  dx = dx;
  dy = dy;
  return ( TRUE );
}

/*
 * move_Obs - is the same as move_obstacle, for backward compatibility
 */
int move_Obs(long obs[2*MAX_VERTICES+1], long dx, long dy)
{
  return(move_obstacle(obs, dx, dy));
}

/*
 * new_world - deletes all obstacles in the current robot world
 */
int new_world(void)
{
  return ( TRUE );
}

/*******************************
 *                             *
 * Miscellaneous robot control *
 *                             *
 *******************************/

/*
 * init_mask - initialize the sensor mask, Smask.
 */
void init_mask(void)
{
  int i;
  
  Smask[ SMASK_POS_DATA ] = 0;
  for (i=1; i<44; i++)
    Smask[i] = 1;
}

/*
 * init_sensors - initialize the sensor mask, Smask, and send it to the
 *                robot. It has no effect on the sensors 
 */
int init_sensors()
{
  int i;
  
  Smask[ SMASK_POS_DATA ] = 0;
  for (i=1; i<44; i++)
    Smask[i] = 1;
  return ( ct() );
}

/*
 * place_robot - places the robot at configuration (x, y, th, tu). 
 *               In simulation mode, it will place both the Encoder-robot
 *               and the Actual-robot at this configuration. In real robot
 *               mode, it will call dp(x, y) and da(th, tu).
 * 
 * parameters:
 *    int x, y -- x-y position of the desired robot configuration
 *    int th, tu -- the steering and turret orientation of the robot
 *                  desired configuration
 */
int place_robot(int x, int y, int th, int tu)
{
  if (dp(x, y) != TRUE || da(th, tu) != TRUE)
    return FALSE;
  
  return TRUE;
}

/*
 * special_request - sends a special request (stored in user_send_buffer) 
 *                   to the robot and waits for the robot's response (which
 *                   will be stored in user_receive_buffer). 
 * 
 * parameters:
 *    unsigned char *user_send_buffer -- stores data to be sent to the robot
 *                                       Should be a pointer to an array of
 *                                       MAX_USER_BUF elements
 *    unsigned char *user_receive_buffer -- stores data received from the robot
 *                                          Should be a pointer to an array of 
 *                                       MAX_USER_BUF elements
 */
int special_request(unsigned char *user_send_buffer,
		    unsigned char *user_receive_buffer)
{
  unsigned char outbuf[MAX_USER_BUF], *byte_ptr;
  int i, length, temp;
  
  length = 2 + user_send_buffer[0] + 256 * user_send_buffer[1];
  if (length>USER_BUFFER_LENGTH-5)
  {
    printf("Data + protocol bytes exceeding %d, truncating\n",
	   USER_BUFFER_LENGTH);
    /* num_data already includes the 4 bytes of user packets protocol */
    length  = USER_BUFFER_LENGTH-5; 
  }
  stuff_length_type(length, SPECIAL, outbuf);
    
  byte_ptr = outbuf + 4;
  for (i=0; i<length; i++) 
    {
      *byte_ptr = (unsigned char)user_send_buffer[i];
      byte_ptr++;
    }
    
  /* 
   * Comm_Robot will process the returned package and write the 
   * data into special_buffer that we assign to be the buffer
   * that the caller provided.
   */

  special_buffer = user_receive_buffer;

  wait_time = SPECIAL_TIMEOUT;
  temp = Comm_Robot(Fd, outbuf);
  wait_time = NORMAL_TIMEOUT;
  return ( temp );
}

/*******************************
 *                             *
 * Graphic Interface Functions *
 *                             *
 *******************************/

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * draw_robot - this function allows the client to draw a robot at
 *              configuration x, y, th, tu (using the robot world 
 *              coordinates). 
 * 
 * parameters:
 *    long x, y -- the x-y position of the robot.
 *    int th, tu -- the steering and turret orientation of the robot
 *    int mode - the drawing mode. If mode = 1, the robot is drawn in 
 *              BlackPixel using GXxor (using GXxor you can erase the trace 
 *              of robotby drawing over it). If mode = 2, the robot is 
 *              drawn in BlackPixel using GXxor and in addition, a small arrow
 *              is drawn at the center of the robot using GXcopy (using this 
 *              mode you can leave a trace of small arrow). If mode = 3, 
 *              the robot is drawn in BlackPixel using GXcopy. When mode > 3,
 *              the robot is drawn in color using GXxor.
 */
int draw_robot(long x, long y, int th, int tu, int mode)
{
  x = x;
  y = y;
  th = th;
  tu = tu;
  mode = mode;
  return ( TRUE );
}

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * draw_line - this function allows the client to draw a line from
 *             (x_1, y_1) to (x_2, y_2) (using the robot world coordinates). 
 *
 * parameters:
 *    long x_1, y_1, x_2, y_2 -- the two end-points of the line
 *    int mode -- the mode of drawing: when mode is 1, the drawing is 
 *                done in BlackPixel using GXcopy; when mode is 2, the drawing
 *                is done in BlackPixel using GXxor, when mode > 2, the drawing
 *                is done in color using GXxor.
 */
int draw_line(long x_1, long y_1, long x_2, long y_2, int mode)
{
  x_1 = x_1;
  y_1 = y_1;
  x_2 = x_2;
  y_2 = y_2;
  mode = mode;
  return ( TRUE );
}

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * draw_arc - this function allows the client to draw arc which is part
 *            of an ellipse (using the robot world coordinates). 
 *
 * parameters:
 *    long x_0, y_0, w, h -- (x_0, y_0) specifies the upper left corner of the 
 *                          rectangle bounding the ellipse while w and h
 *                          specifies the width and height of the bounding 
 *                          rectangle, respectively.
 *    int th1, th2 -- th1 and th2 specifies the angular range of the arc.
 *    int mode -- the mode of drawing: when mode is 1, the drawing is 
 *                done in BlackPixel using GXcopy; when mode is 2, the drawing
 *                is done in BlackPixel using GXxor, when mode > 2, the drawing
 *                is done in color using GXxor.
 */
int draw_arc(long x_0, long y_0, long w, long h, int th1, int th2, int mode)
{
  x_0 = x_0;
  y_0 = y_0;
  w   = w;
  h   = h;
  th1 = th1;
  th2 = th2;
  mode = mode;
  return ( TRUE );
}

/*************************************
 *                                   *
 * Miscellaneous Interface Functions *
 *                                   *
 *************************************/

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * server_is_running - this function queries the server to see
 *                     if it is up and running.  If so, this function
 *                     returns a TRUE, otherwise it returns FALSE.
 *                     This function is replaced by connect_robot, but 
 *                     is defined here for backward compatibility
 */
int server_is_running()
{
  return(connect_robot(1));
}

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * quit_server - this function allows the client to quit the server
 *               assuming this feature is enabled in the setup file
 *               of the server
 */
int quit_server(void)
{
  return ( TRUE );
}

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * real_robot - this function allows the client to switch to
 *              real robot mode in the server
 */
int real_robot(void)
{
  return ( TRUE );
}

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * simulated_robot - this function allows the client to switch to
 *                   simulated robot mode in the server
 */
int simulated_robot(void)
{
  return ( TRUE );
}

/*
 * dummy function - to maintain compatibility with Nclient
 *
 * predict_sensors - this function predicts the sensor reading of
 *                   the robot assuming it is at position (x, y)
 *                   and orientation th and tu using the map of the
 *                   simulated robot environment. The predicted sensor
 *                   data values are stored in "state" and "laser".
 * 
 * parameters:
 *    int x, y, th, tu -- the configuration of the robot
 *    long *state -- where to put the predicted state data
 *    int *laser -- where to put the predicted laser data
 */
int predict_sensors(int x, int y, int th, int tu, long *state, int *laser)
{
  x   = x;
  y   = y;
  th  = th;
  tu  = tu;
  state[0] = state[0];
  laser[0] = laser[0];
  return ( TRUE );
}

/* 
 * dummy function in Ndirect: needs server 
 *
 * motion_check - this function computes the intersection of a path
 *                specified by the parameters: type, a1, ..., a7 with
 *                the obstacles in the robot's environment. If there is
 *                collision, the function returns 1 and the x-y configuration
 *                of the robot is stored in collide[0] and collide[1] while
 *                collide[2] stores the inward normal of the obstacle edge
 *                that the robot collides with (this information can be
 *                used to calculate which bumper is hit.). If there is no
 *                collision, the function returns 0.
 *
 * parameters:
 *    long type - 0 if the path is a line segment
 *                1 if the path is an arc of circle
 *    double a1 a2 - x-y coordinates of the first point of the path (the path
 *                   is directional).
 *    depending on the value of type, a3 - a7 have different meanings.
 *    if (type == 0), line segment mode
 *      double a3 a4 are the x-y coordinates of the second point of the path
 *      a5, a6, a7 have no meaning
 *    if (type == 1), arc of circle mode
 *      double a3 is the angle (in radiance) of the vector connecting the 
 *                center of the circle to the first end-point of the arc
 *      double a4 is the angle of the vector connecting the center
 *                of the circle to the second end-point of the arc
 *      double a5 is the radius of the circle
 *      double a6 a7 are the x-y coordinate of the center of the circle
 */
int motion_check(long type, double a1, double a2, double a3, double a4,
		 double a5, double a6, double a7, double collide[3])
{
  type = type;
  a1   = a1;
  a2   = a2;
  a3   = a3;
  a4   = a4;
  a4   = a5;
  a4   = a6;
  a4   = a7;
  collide[0] = collide[0];
  return ( FALSE );
}

/*
 * dummy function in Ndirect: needs server 
 *
 * get_robot_conf - interactively getting the robot's conf, by clicking
 *                  the mouse in the server's Robot window
 * 
 * parameters:
 *    long *conf -- should be an array of 4 long integers. The configuration
 *                  of the robot is returned in this array.
 */
int get_robot_conf(long *conf)
{
  conf[0] = conf[0];
  return ( TRUE );
}

/*******************************************
 *                                         *
 * The following are helper functions for  *
 * developing user defined host <-> robot  *
 * communication                           *
 *                                         *
 *******************************************/

/*
 *  init_receive_buffer - sets the index to 4 which is the point
 *  at which data should begin to be extracted
 * 
 *  parameters:
 *     unsigned short *index -- is the buffer index
 */
int init_receive_buffer(unsigned short *index)
{
  *index = 4;
  return(*index);
}

/*
 *  extract_receive_buffer_header - extracts the header information:
 *  length, serial_number, and packettype from the beginning of the
 *  receive buffer.
 *
 *  parameters:
 *     unsigned short *length -- is the returns the number of chars in the buffer
 *
 *     unsigned char *serial_number -- returns the serial number to be
 *                                     assigned to the packet
 *     unsigned char *packet_type -- returns the type number to be
 *                                   assigned to the packet
 *     unsigned char *buffer -- is the receive buffer
 */
int extract_receive_buffer_header(unsigned short *length, 
				  unsigned char *serial_number, 
				  unsigned char *packet_type, 
				  unsigned char *buffer)
{
  unsigned short data;
  
  data = buffer[0] << 0;
  data |= buffer[1] << 8;
  *length = data;
  *serial_number = buffer[2];
  *packet_type = buffer[3];
  return(*length);
}

/*
 *  init_send_buffer - sets the index to 4 which is the point
 *  at which data should be inserted
 *
 *  parameters:
 *     int *index -- is the buffer index
 */
int init_send_buffer(unsigned short *index)
{
  *index = 4;
  return(*index);
}

/*
 *  stuff_send_buffer_header - loads the header information,
 *  length,serial_number, and packettype into the beginning of the
 *  buffer.  It should be called after the data has been stuffed,
 *  i.e. index represents the length of the packet.
 *
 *  parameters:
 *     unsigned short index -- is the buffer index which holds the number of chars
 *                  in the buffer
 *     unsigned char serial_number -- holds the serial number to be
 *                                    assigned to the packet
 *     unsigned char packet_type -- holds the type number to be
 *	                           assigned to the packet
 *
 *     unsigned char *buffer -- is the send buffer
 */
int stuff_send_buffer_header(unsigned short index, unsigned char serial_number, 
			     unsigned char packet_type, unsigned char *buffer)
{
  buffer[0] = (index >> 0) & 0xff;
  buffer[1] = (index >> 8) & 0xff;
  buffer[2] = serial_number;
  buffer[3] = packet_type;
  return(index);
}

/*
 *  stuffchar -  stuffs a 1 byte char into the send buffer
 *
 *  parameters:
 *     signed char data -- is the char to be stuffed
 *     unsigned char *buffer -- is the send buffer
 *     unsigned short *index -- is the buffer index which will be incremented
 *                              to reflect the bytes stuffed into the buffer
 */
int stuffchar(signed char data, unsigned char *buffer, unsigned short *index)
{
  if (data < 0)
  {
    data *= -1;
    data |= 0x80;
  }
  
  buffer[*index]   = data;
  *index += 1;
  return(*index);
}

/*
 *  stuff2byteint - stuffs a short int(2 bytes) into the send buffer
 *
 *  parameters:
 *     signed int data -- is the value which will be split apart and stuffed
 *	                  bytewise into the send buffer
 *     unsigned char *buffer -- is the send buffer
 *     unsigned short *index -- is the buffer index which will be incremented
 *                              to reflect the bytes stuffed into the buffer
 */
int stuff2byteint(signed short data,
		  unsigned char *buffer, unsigned short *index)
{
  if (data < 0)
  {
    data *= -1;
    data |= 0x8000;
  }
  
  buffer[*index]   = (data >> 0) & 0xff;
  *index += 1;
  buffer[*index]   = (data >> 8) & 0xff;
  *index += 1;
  
  return(*index);
}

/*
 *  stuff4byteint - stuffs a long int(4 bytes) into the send buffer
 *
 *  parameters:
 *     signed long data -- is the value which will be split apart and stuffed
 *	                   bytewise into the send buffer
 *     unsigned char *buffer -- is the send buffer
 *     unsigned short *index -- is the buffer index which will be incremented
 *	                        to reflect the bytes stuffed into the buffer
 */
int stuff4byteint(signed long data,
		  unsigned char *buffer, unsigned short *index)
{
  if (data < 0)
  {
    data *= -1;
    data |= 0x80000000L;
  }
  
  buffer[*index] = (data >> 0) & 0xff;
  *index += 1;
  buffer[*index] = (data >> 8) & 0xff;
  *index += 1;
  buffer[*index] = (data >> 16) & 0xff;
  *index += 1;
  buffer[*index] = (data >> 24) & 0xff;
  *index += 1;
  
  return(*index);
}

/*
 *  stuffuchar -  stuffs an unsigned char into the send buffer
 *
 *  parameters:
 *     unsigned char data -- is the char to be stuffed
 *     unsigned char *buffer -- is the send buffer
 *     unsigned short *index -- is the buffer index which will be incremented
 *                              to reflect the bytes stuffed into the buffer
 */
int stuffuchar(unsigned char data, unsigned char *buffer, unsigned short *index)
{
  buffer[*index]   = data;
  
  *index += 1;
  
  return(*index);
}

/*
 *  stuff2byteuint - stuffs an unsigned short int(2 bytes) into the send buffer
 *
 *  parameters:
 *     unsigned short data -- is the value which will be split apart and 
 *                            stuffed bytewise into the send buffer
 *     unsigned char *buffer -- is the send buffer
 *     unsigned short *index -- is the buffer index which will be incremented
 *	                        to reflect the bytes stuffed into the buffer
 */
int stuff2byteuint(unsigned short data,
		   unsigned char *buffer, unsigned short *index)
{
  buffer[*index]   = (data >> 0) & 0xff;
  *index += 1;
  buffer[*index]   = (data >> 8) & 0xff;
  *index += 1;
  
  return(*index);
}

/*
 *  stuff4byteuint - stuffs an unsigned long int(4 bytes) into the send buffer
 *
 *  parameters:
 *     unsigned long data -- is the value which will be split apart and stuffed
 *	                     bytewise into the send buffer
 *     unsigned char *buffer -- is the send buffer
 *     unsigned short *index -- is the buffer index which will be incremented
 *	                        to reflect the bytes stuffed into the buffer
 */
int stuff4byteuint(unsigned long data,
		   unsigned char *buffer, unsigned short *index)
{
  buffer[*index] = (data >> 0) & 0xff;
  *index += 1;
  buffer[*index] = (data >> 8) & 0xff;
  *index += 1;
  buffer[*index] = (data >> 16) & 0xff;
  *index += 1;
  buffer[*index] = (data >> 24) & 0xff;
  *index += 1;
  
  return(*index);
}

/*
 *  stuffdouble - stuffs a double(8 bytes) into the send buffer
 *
 *  parameters:
 *     double data -- is the value which will be split apart and stuffed
 *	              bytewise into the send buffer
 *     unsigned char *buffer -- is the send buffer
 *     unsigned short *index -- is the buffer index which will be incremented
 *	                        to reflect the bytes stuffed into the buffer
 */
int stuffdouble(double data, unsigned char *buffer, unsigned short *index)
{
  unsigned long long *tempp, temp;
  
  /* Assume that double is 64 bits and "long long" is 64 bits. */
  tempp = (unsigned long long *)&data;
  temp = *tempp;
  
  buffer[*index] = (temp >> 0) & 0xff;
  *index += 1;
  buffer[*index] = (temp >> 8) & 0xff;
  *index += 1;
  buffer[*index] = (temp >> 16) & 0xff;
  *index += 1;
  buffer[*index] = (temp >> 24) & 0xff;
  *index += 1;
  buffer[*index] = (temp >> 32) & 0xff;
  *index += 1;
  buffer[*index] = (temp >> 40) & 0xff;
  *index += 1;
  buffer[*index] = (temp >> 48) & 0xff;
  *index += 1;
  buffer[*index] = (temp >> 56) & 0xff;
  *index += 1;
  
  return(*index);
}

/*
 *  extractchar -  extracts a char from the receive buffer
 *
 *  parameters:
 *     unsigned char *buffer -- is the receive buffer which holds the data
 *     unsigned short *index -- is the receive buffer index which will be
 *                              incremented to reflect the position of the
 *                              next piece of data to be extracted
 */
signed char extractchar(unsigned char *buffer, unsigned short *index)
{
  char data;
  
  data = buffer[*index];
  *index += 1;
  
  if (data & 0x80)
  {
    data &= 0x7f;
    data *= -1;
  }
  return(data);
}

/*
 *  extract2byteint -  extracts a short int(2 bytes) from the receive buffer
 *
 *  parameters:
 *     unsigned char *buffer -- is the receive buffer which holds the data
 *     unsigned short *index -- is the receive buffer index which will be
 *                              incremented to reflect the position of the
 *                              next piece of data to be extracted
 */
signed short extract2byteint(unsigned char *buffer, unsigned short *index)
{
  signed short data;
  
  data = (signed short)buffer[*index] << 0;
  *index += 1;
  data |= (signed short)buffer[*index] << 8;
  *index += 1;
  
  if (data & 0x8000)
  {
    data &= 0x7fff;
    data *= -1;
  }
  
  return(data);
}

/*
 *  extract4byteint -  extracts a long int(4 bytes) from the receive buffer
 *
 *  parameters:
 *     unsigned char *buffer -- is the receive buffer which holds the data
 *     unsigned short *index -- is the receive buffer index which will be
 *                              incremented to reflect the position of the
 *                              next piece of data to be extracted
 */
signed long extract4byteint(unsigned char *buffer, unsigned short *index)
{
  signed long data;
  
  data = (signed long)buffer[*index] << 0;
  *index += 1;
  data |= (signed long)buffer[*index] << 8;
  *index += 1;
  data |= (signed long)buffer[*index] << 16;
  *index += 1;
  data |= (signed long)buffer[*index] << 24;
  *index += 1;
  
  if (data & 0x80000000)
  {
    data &= 0x7fffffff;
    data *= -1;
  }
  
  return(data);
}

/*
 *  extractuchar -  extracts an unsigned char from the receive buffer
 *
 *  parameters:
 *     unsigned char *buffer -- is the receive buffer which holds the data
 *     unsigned short *index -- is the receive buffer index which will be
 *                              incremented to reflect the position of the
 *                              next piece of data to be extracted
 */
unsigned char extractuchar(unsigned char *buffer, unsigned short *index)
{
  unsigned char data;
  
  data = buffer[*index];
  
  *index += 1;
  
  return(data);
}

/*
 *  extract2byteuint -  extracts an unsigned short int(2 bytes) from the 
 *                      receive buffer
 *
 *  parameters:
 *     unsigned char *buffer -- is the receive buffer which holds the data
 *     unsigned short *index -- is the receive buffer index which will be
 *                              incremented to reflect the position of the
 *                              next piece of data to be extracted
 */
unsigned short extract2byteuint(unsigned char *buffer, unsigned short *index)
{
  unsigned short data;
  
  data = (unsigned short)buffer[*index] << 0;
  *index += 1;
  data |= (unsigned short)buffer[*index] << 8;
  *index += 1;
  
  return(data);
}

/*
 *  extract4byteuint -  extracts an unsigned long int(4 bytes) from the 
 *                      receive buffer
 *
 *  parameters:
 *     unsigned char *buffer -- is the receive buffer which holds the data
 *     unsigned short *index -- is the receive buffer index which will be
 *                              incremented to reflect the position of the
 *                              next piece of data to be extracted
 */
unsigned long extract4byteuint(unsigned char *buffer, unsigned short *index)
{
  unsigned long data;
  
  data = (unsigned long)buffer[*index] << 0;
  *index += 1;
  data |= (unsigned long)buffer[*index] << 8;
  *index += 1;
  data |= (unsigned long)buffer[*index] << 16;
  *index += 1;
  data |= (unsigned long)buffer[*index] << 24;
  *index += 1;
  
  return(data);
}

/*
 *  extractdouble -  extracts a double(8 bytes) from the receive buffer
 *
 *  parameters:
 *     unsigned char *buffer -- is the receive buffer which holds the data
 *     unsigned short *index -- is the receive buffer index which will be
 *                              incremented to reflect the position of the
 *                              next piece of data to be extracted
 */
double extractdouble(unsigned char *buffer, unsigned short *index)
{
  double data;
  unsigned long long *tempp, temp;
  
  /* Assume that double is 64 bits and long long is 64 bits. */
  
  temp = (unsigned long long)buffer[*index] << 0;
  *index += 1;
  temp |= (unsigned long long)buffer[*index] << 8;
  *index += 1;
  temp |= (unsigned long long)buffer[*index] << 16;
  *index += 1;
  temp |= (unsigned long long)buffer[*index] << 24;
  *index += 1;
  temp |= (unsigned long long)buffer[*index] << 32;
  *index += 1;
  temp |= (unsigned long long)buffer[*index] << 40;
  *index += 1;
  temp |= (unsigned long long)buffer[*index] << 48;
  *index += 1;
  temp |= (unsigned long long)buffer[*index] << 56;
  *index += 1;
  
  tempp = (unsigned long long *)&data;
  *tempp = temp;
  
  return(data);
}

/************************************************
 *                                              *
 * Global variable access functions for Allegro * 
 * Common Lisp interface                        *
 *                                              *
 ************************************************/

int get_state(long state[NUM_STATE])
{
  int i;
  
  for (i=0;i<NUM_STATE;i++) 
    state[i] = State[i];
  return(TRUE);
}

int get_laser(int laser[2*NUM_LASER+1])
{
  int i;

  for (i=0;i<=Laser[0];i++) 
    laser[i] = Laser[i];
  return(TRUE);
}

int get_mask(int mask[NUM_MASK])
{
  int i;

  for (i=0;i<44;i++) 
    mask[i] = usedSmask[i];
  return(TRUE);
}

int set_mask(int mask[NUM_MASK])
{
  int i;

  for (i=0;i<NUM_MASK;i++) 
    Smask[i] = mask[i];
  return(TRUE);
}

int set_server_machine_name(char *sname)
{
  strcpy(SERVER_MACHINE_NAME, sname);
  strcpy(Host_name, "");
  return(TRUE);
}

int set_serv_tcp_port(int port)
{
  SERV_TCP_PORT = port;
  return(TRUE);
}


/*
 *
 * 
 *         PosData Attachment
 *         ===================
 *    
 *    Here all procudures are defined that deal with the 
 * 
 *    attachment of PosData to sensory readings.
 * 
 *
 */


/***************
 * FUNCTION:     posDataRequest
 * PURPOSE:      request position information for sensors
 * ARGUMENTS:    int posRequest : 
 *               The argument of this function specifies the sensors 
 *               for which the position information (PosData) should 
 *               be attached to the sensory reading.
 *               Its value is obtained by ORing the desired defines. 
 * EXAMPLE:      To attach PosData to sonars and laser:
 *               posDataRequest ( POS_SONAR | POS_LASER );
 * ALGORITHM:    currently sets the global variable Smask[0] and
 *               then calls ct() to transmit the change to the server
 * RETURN:       TRUE if the argument was correct, else FALSE
 * SIDE EFFECT:  Smask[ SMASK_POS_DATA ]
 * CALLS:        
 * CALLED BY:    
 ***************/
int posDataRequest ( int posRequest )
{
  /* check if the argument is okay */
  if ( posRequest & 
      !( POS_INFRARED | POS_SONAR | POS_BUMPER | POS_LASER | POS_COMPASS ) )
    return ( FALSE );

  /* The value in Smask[ SMASK_POS_DATA ] is passed through entire system */
  Smask[ SMASK_POS_DATA ] = posRequest;
  ct();

  return ( TRUE );
}

/***************
 * FUNCTION:     posDataCheck
 * PURPOSE:      return the sensors for which the PosData attachment
 *               is currently requested. 
 * ARGUMENTS:    None
 * ALGORITHM:    returns the mask that is not globally accessibe and 
 *               that is set by ct() to be the value of Smask[0]
 * RETURN:       int, see posDataRequest
 *               the macros POS_*_P can be used to examine the value
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
int posDataCheck ( void )
{
  return ( usedSmask[ SMASK_POS_DATA ] );
}

/***************
 * FUNCTION:     posInfraredRingGet
 * PURPOSE:      copy the PosData for all infrareds to accessible memory
 * ARGUMENTS:    PosData posData [INFRAREDS] :
 *               an array of PosData structures that is filled with 
 *               PosData. The position information for each infrared
 *               containts the configuration of the robot at the time 
 *               of the sensory reading and a timestamp for the 
 *               configuration and the senosry reading .
 * ALGORITHM:    copies blocks of memory
 * RETURN:       int, return always TRUE
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
int posInfraredRingGet ( PosData posData[INFRAREDS] )
{
  /* copy the whole thing in one block */
  memcpy ( posData, posDataAll.infrared, INFRAREDS * sizeof ( PosData ) );

  return ( TRUE );
}


/***************
 * FUNCTION:     posInfraredGet
 * PURPOSE:      copy the PosData for a specific infrared to accessible 
 *               memory
 * ARGUMENTS:    int infraredNumber : the number of the infrared
 *               PosData *posData : the memory location that the information
 *                                  will be copied to 
 * ALGORITHM:    copies block of memory
 * RETURN:       int, always returns TRUE
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
int posInfraredGet     ( PosData *posData , int infraredNumber)
{
  /* copy the whole thing in one block */
  memcpy ( posData, &posDataAll.infrared[infraredNumber], sizeof ( PosData ) );

  return ( TRUE );
}

/***************
 * FUNCTION:     posSonarRingGet
 * PURPOSE:      copy the PosData for all sonars to accessible memory
 * ARGUMENTS:    PosData posData [SONARS] :
 *               an array of PosData structures that is filled with 
 *               PosData. The position information for each sonar
 *               containts the configuration of the robot at the time 
 *               of the sensory reading and a timestamp for the 
 *               configuration and the senosry reading .
 * ALGORITHM:    copies blocks of memory
 * RETURN:       int, return always TRUE
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
int posSonarRingGet    ( PosData posData[SONARS] )
{
  /* copy the whole thing in one block */
  memcpy ( posData, posDataAll.sonar, SONARS * sizeof ( PosData ) );

  return ( TRUE );
}

/***************
 * FUNCTION:     posSonarGet
 * PURPOSE:      copy the PosData for a specific sonar to accessible memory
 * ARGUMENTS:    int infraredNumber : the number of the sonar
 *               PosData *posData : the memory location that the information
 *                                  will be copied to 
 * ALGORITHM:    copies block of memory
 * RETURN:       int, always returns TRUE
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
int posSonarGet        ( PosData *posData , int sonarNumber)
{
  /* copy the whole thing in one block */
  memcpy ( posData, &posDataAll.sonar[sonarNumber], sizeof ( PosData ) );

  return ( TRUE );
}

/***************
 * FUNCTION:     posBumperGet
 * PURPOSE:      copy PosData for the bumper to accessible memory
 * ARGUMENTS:    PosData *posData : where the data is copied to 
 * ALGORITHM:    copies a block of memory
 * RETURN:       int, always returns TRUE
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 * NOTE:         The bumper differs from other sensors in that the 
 *               posData is only updated after one of the bumper sensors 
 *               change its value from zero to one. This means that the 
 *               posData for the bumper always contains the position and 
 *               timeStamps of the latest hit, or undefined information 
 *               if the bumper was not hit yet.
 ***************/
int posBumperGet       ( PosData *posData )
{
  /* copy the whole thing in one block */
  memcpy ( posData, &posDataAll.bumper, sizeof ( PosData ) );

  return ( TRUE );
}

/***************
 * FUNCTION:     posLaserGet
 * PURPOSE:      copy PosData for the laser to accessible memory
 * ARGUMENTS:    PosData *posData : where the data is copied to 
 * ALGORITHM:    copies a block of memory
 * RETURN:       int, always returns TRUE
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 * NOTE:         The laser is updated at a frequency of 30Hz.
 ***************/
int posLaserGet        ( PosData *posData )
{
  /* copy the whole thing in one block */
  memcpy ( posData, &posDataAll.laser, sizeof ( PosData ) );

  return ( TRUE );
}

/***************
 * FUNCTION:     posCompassGet
 * PURPOSE:      copy PosData for the compass to accessible memory
 * ARGUMENTS:    PosData *posData : where the data is copied to 
 * ALGORITHM:    copies a block of memory
 * RETURN:       int, always returns TRUE
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 * NOTE:         The compass is updated ad a frequency of 10Hz.
 ***************/
int posCompassGet      ( PosData *posData )
{
  /* copy the whole thing in one block */
  memcpy ( posData, &posDataAll.compass, sizeof ( PosData ) );

  return ( TRUE );
}

/***************
 * FUNCTION:     posTimeGet
 * PURPOSE:      get the PosData time (Intellisys 100) in milliseconds
 * ARGUMENTS:    None
 * ALGORITHM:    ---
 * RETURN:       int 
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 * NOTE:         Use POS_TICKS_TO_MS and POS_MS_TO_TICKS to convert
 *               between ticks and milliseconds. Overflow after 49 days.
 ***************/
int posTimeGet         ( void )
{
  return ( (int) posDataTime );
}

/***************
 * FUNCTION:     voltCpuGet
 * PURPOSE:      get the voltage of the power supply for the CPU
 * ARGUMENTS:    None
 * ALGORITHM:    ---
 * RETURN:       float (the voltage in volt)
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
float voltCpuGet         ( void )
{
  return ( voltConvert ( voltageCPU , RANGE_CPU_VOLTAGE ) );
}

/***************
 * FUNCTION:     voltMotorGet
 * PURPOSE:      get the voltage of the power supply for the motors
 * ARGUMENTS:    None
 * ALGORITHM:    ---
 * RETURN:       float (the voltage in volt)
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
float voltMotorGet         ( void )
{
  return ( voltConvert ( voltageMotor , RANGE_MOTOR_VOLTAGE ) );
}

/***************
 * FUNCTION:     voltConvert
 * PURPOSE:      convert from the DA reading to the right voltage range
 * ARGUMENTS:    unsigned char reading: the reading of the da
 * ALGORITHM:    ---
 * RETURN:       float (the voltage in volt)
 * SIDE EFFECT:  
 * CALLS:        
 * CALLED BY:    
 ***************/
static float voltConvert ( unsigned char reading , float range )
{
  /* 
   * original reading is [0...255] and represents [2...5]volt.
   * the 5 volt value is converted to 12V by multiplying (range/5)
   */
  return ( ( 2.0 +  ( ( (float) (reading*3) ) / 255.0 ) ) * ( range / 5.0 ) );
}


/****************************************************************/



long arm_zr(short override)
{
  long result;

  unsigned short b_index, b_length;
  unsigned char serial_number;
  unsigned char packet_type;
  unsigned char user_send_buffer[256];
  unsigned char user_receive_buffer[256];

  init_send_buffer(&b_index);

  stuff2byteuint(override, user_send_buffer, &b_index);
  stuff_send_buffer_header(b_index, 0, ARM_ZR, user_send_buffer);
  
  special_request(user_send_buffer, user_receive_buffer);

  init_receive_buffer(&b_index);
  extract_receive_buffer_header(&b_length, &serial_number, &packet_type,
				user_receive_buffer);

  result=extract4byteuint(user_receive_buffer, &b_index);
  return result;
}

long arm_ws(short l, short g, long timeout, long *time_remain)
{
  long result;

  unsigned short b_index, b_length;
  unsigned char serial_number;
  unsigned char packet_type;
  unsigned char user_send_buffer[256];
  unsigned char user_receive_buffer[256];

  init_send_buffer(&b_index);

  stuff2byteuint(l, user_send_buffer, &b_index);
  stuff2byteuint(g, user_send_buffer, &b_index);
  stuff4byteuint(timeout, user_send_buffer, &b_index);
  stuff_send_buffer_header(b_index, 0, ARM_WS, user_send_buffer);
  
  special_request(user_send_buffer, user_receive_buffer);

  init_receive_buffer(&b_index);
  extract_receive_buffer_header(&b_length, &serial_number, &packet_type,
				user_receive_buffer);

  result=extract4byteuint(user_receive_buffer, &b_index);
  if (time_remain)
    *time_remain=extract4byteuint(user_receive_buffer, &b_index);

  return result;
}


long arm_mv(long l_mode, long l_v, long g_mode, long g_v)
{
  long result;

  unsigned short b_index, b_length;
  unsigned char serial_number;
  unsigned char packet_type;
  unsigned char user_send_buffer[256];
  unsigned char user_receive_buffer[256];

  init_send_buffer(&b_index);

  stuff4byteuint(l_mode, user_send_buffer, &b_index);
  stuff4byteuint(l_v, user_send_buffer, &b_index);
  stuff4byteuint(g_mode, user_send_buffer, &b_index);
  stuff4byteuint(g_v, user_send_buffer, &b_index);
  stuff_send_buffer_header(b_index, 0, ARM_MV, user_send_buffer);
  
  special_request(user_send_buffer, user_receive_buffer);

  init_receive_buffer(&b_index);
  extract_receive_buffer_header(&b_length, &serial_number, &packet_type,
				user_receive_buffer);

  result=extract4byteuint(user_receive_buffer, &b_index);

  return result;
}
