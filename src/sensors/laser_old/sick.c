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

#define CARMEN_LASER_USE_SELECT 1
#define CARMEN_LASER_LOW_LATENCY 1

#include <carmen/global.h>
#include <carmen/carmenserial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>

/*****  DIRK WAS HERE - START ******/
#include <sys/utsname.h>
#if !defined(CYGWIN) && !defined(__APPLE__)
#include <linux/serial.h>
#include <linux/version.h>
#endif
/*****  DIRK WAS HERE - END ******/

#include "laser_ipc.h"
#include "sick.h"

/*****  DIRK WAS HERE - START ******/
extern void shutdown_laser(int x);
/*****  DIRK WAS HERE - END ******/

/* fancy serial functions */

int iParity(parity_t par)
{
  if(par == N)
    return(IGNPAR);
  else
    return(INPCK);
}

int iSoftControl(int flowcontrol)
{
  if(flowcontrol)
    return(IXON);
  else
    return(IXOFF);
}

int cDataSize(int numbits)
{
  switch(numbits) {
  case 5:
    return(CS5);
    break;
  case 6:
    return(CS6);
    break;
  case 7:
    return(CS7);
    break;
  case 8:
    return(CS8);
    break;
  default:
    return(CS8);
    break;
  }
}

int cStopSize(int numbits)
{
  if(numbits == 2)
    return(CSTOPB);
  else
    return(0);
}

int cFlowControl(int flowcontrol)
{
  if(flowcontrol)
    return(CRTSCTS);
  else
    return(CLOCAL);
}

int cParity(parity_t par)
{
  if(par != N) {
    if(par == O)
      return(PARENB | PARODD);
    else
      return(PARENB);
  }
  else
    return(0);
}

int cBaudrate(int baudrate)
{
  switch(baudrate) {
  case 0:
    return(B0);
    break;
  case 300:
    return(B300);
    break;
  case 600:
    return(B600);
    break;
  case 1200:
    return(B1200);
    break;
  case 2400:
    return(B2400);
    break;
  case 4800:
    return(B4800);
    break;
  case 9600:
    return(B9600);
    break;
  case 19200:
    return(B19200);
    break;
  case 38400:
    return(B38400);
    break;
  case 57600:
    return(B57600);
    break;
  case 115200:
    return(B115200);
    break;
#if !defined(CYGWIN) && !defined(__APPLE__)
  case 500000:
    /* to use 500k you have to change the entry of B460800 in you kernel:
       /usr/src/linux/drivers/usb/serial/ftdi_sio.h:
       ftdi_8U232AM_48MHz_b460800 = 0x0006    */
    return(B460800);
    break;
#endif
  default:
    return(B9600);
    break;
  }
}

void sick_set_serial_params(sick_laser_p laser)
{
  struct termios  ctio;
  
  tcgetattr(laser->dev.fd, &ctio); /* save current port settings */
  ctio.c_iflag = iSoftControl(laser->dev.swf) | iParity(laser->dev.parity);
  ctio.c_oflag = 0;
  ctio.c_cflag = CREAD | cFlowControl(laser->dev.hwf || laser->dev.swf) |
    cParity(laser->dev.parity) | cDataSize(laser->dev.databits) | 
    cStopSize(laser->dev.stopbits);
  ctio.c_lflag = 0;
  ctio.c_cc[VTIME] = 0;     /* inter-character timer unused */
  ctio.c_cc[VMIN] = 0;      /* blocking read until 0 chars received */
  cfsetispeed(&ctio, (speed_t)cBaudrate(laser->dev.baudrate));
  cfsetospeed(&ctio, (speed_t)cBaudrate(laser->dev.baudrate));
  tcflush(laser->dev.fd, TCIFLUSH);
  tcsetattr(laser->dev.fd, TCSANOW, &ctio);
}

/*****  DIRK WAS HERE - START ******/

int
kernel_minimum_version( int a, int b, int c )
{
  struct utsname        uts;
  int                   ca, cb, cc; 
  uname(&uts);
  sscanf( uts.release, "%d.%d.%d", &ca, &cb, &cc );
  if (ca*65536+cb*256+cc>=a*65536+b*256+c) {
    return(TRUE);
  } else {
    return(FALSE);
  }
}

void sick_set_baudrate(sick_laser_p laser, int brate)
{
  struct termios  ctio;

#if !defined(CYGWIN) && !defined(__APPLE__)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,20)
  struct serial_struct  serinfo;
#endif
  
  tcgetattr(laser->dev.fd, &ctio); /* save current port settings */
  
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,20)
  if (brate==500000 && kernel_minimum_version(2,4,20)) {
    cfsetispeed ( &ctio, (speed_t) cBaudrate(38400) );
    cfsetospeed ( &ctio, (speed_t) cBaudrate(38400) );
    serinfo.reserved_char[0] = 0;
    if (ioctl(laser->dev.fd, TIOCGSERIAL, &serinfo) < 0) {
      fprintf(stderr," cannot get serial info\n");
      close(laser->dev.fd);
      shutdown_laser(1);
    }
    serinfo.flags =
      ( serinfo.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    serinfo.custom_divisor =  serinfo.baud_base / brate;
    if (ioctl(laser->dev.fd, TIOCSSERIAL, &serinfo) < 0) {
      fprintf(stderr," cannot set serial info\n");
      close(laser->dev.fd);
      shutdown_laser(1);
    }
  } else {
    cfsetispeed ( &ctio, (speed_t) cBaudrate(brate) );
    cfsetospeed ( &ctio, (speed_t) cBaudrate(brate) );
  }
#else 
  cfsetispeed(&ctio, (speed_t)cBaudrate(brate));
  cfsetospeed(&ctio, (speed_t)cBaudrate(brate));
#endif

#else

  cfsetispeed(&ctio, (speed_t)cBaudrate(brate));
  cfsetospeed(&ctio, (speed_t)cBaudrate(brate));

#endif

  tcflush(laser->dev.fd, TCIFLUSH);
  tcsetattr(laser->dev.fd, TCSANOW, &ctio);
}

/*****  DIRK WAS HERE - END ******/

int sick_serial_connect(sick_laser_p laser)
{
  if((laser->dev.fd = open(laser->dev.ttyport, O_RDWR | O_NOCTTY, 0)) < 0)
    return(-1);
  #ifdef CARMEN_LASER_LOW_LATENCY
  carmen_serial_set_low_latency(laser->dev.fd);
  #endif
  sick_set_serial_params(laser);
  return(laser->dev.fd);
}

/* sick_compute_checksum - Compute the CRC checksum of a segment of data. */

static int sick_compute_checksum(unsigned char *CommData, int uLen)
{
  unsigned char abData[2] = {0, 0}, uCrc16[2] = {0, 0};
  
  while(uLen--) {
    abData[0] = abData[1];
    abData[1] = *CommData++;
    if(uCrc16[0] & 0x80) {
      uCrc16[0] <<= 1;
      if(uCrc16[1] & 0x80)
        uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;
      uCrc16[0] ^= CRC16_GEN_POL0;
      uCrc16[1] ^= CRC16_GEN_POL1;
    } 
    else {
      uCrc16[0] <<= 1;
      if(uCrc16[1] & 0x80)
        uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;
    }
    uCrc16[0] ^= abData[0];
    uCrc16[1] ^= abData[1];
  }
  return (((int)uCrc16[0]) * 256 + ((int)uCrc16[1]));
}

int sick_read_data(sick_laser_p laser, unsigned char *data, double timeout)
{
  static int val, i, j, l, pos, chk1, chk2;
  double start_time;
#ifdef CARMEN_LASER_USE_SELECT
  fd_set read_set;
  struct timeval timer;
  timer.tv_sec=(long)(floor(timeout));
  timer.tv_usec=(long)((timeout-floor(timeout))*1000000);
  FD_ZERO(&read_set);
  FD_SET(laser->dev.fd, &read_set);
#endif

  l    = BUFFER_SIZE;
  pos  = 0;
  chk1 = FALSE;
  chk2 = FALSE;
#ifdef CARMEN_LASER_USE_SELECT
  while(select(laser->dev.fd+1, &read_set, NULL, NULL, &timer)) {
  start_time = carmen_get_time();
#else
  start_time = carmen_get_time();
  while(carmen_get_time() - start_time < timeout) {
#endif
    val = carmen_serial_numChars(laser->dev.fd);
    if(val > 0) {
      if(pos + val >= BUFFER_SIZE)
	return(0);
      if(pos + val >= l + 6)
	val = l + 6 - pos;
      read(laser->dev.fd, &(data[pos]), val);
      pos += val;
      if(!chk1 && pos > 2) {
	if(data[0] != STX || data[1] != LID) {
	  for(i = 1; i < pos - 1; i++) {
	    if(data[i] == STX && data[i+1] == LID) {
	      for(j = i; j < pos; j++) {
		data[j - i] = data[j];
	      }
	      pos -= i;
	      chk1 = TRUE;
	      break;
	    }
	  }
	  if(!chk1)
	    pos = 0;
	} 
	else
	  chk1 = TRUE;
      }
      if(!chk2 && pos > 4) {
	l = data[3] * 256 + data[2];
	chk2 = TRUE;
      }
    }
    if(pos == l + 6)
      return(l + 6);
#ifndef CARMEN_LASER_USE_SELECT
    usleep(1000);
#endif
  }
  return(0);
}

int sick_write_command(sick_laser_p laser, unsigned char command,
		       unsigned char *argument, int arg_length)
{
  unsigned char buffer[MAX_COMMAND_SIZE];
  int pos = 0, i, check, length, loop, answer = 0, counter = 0;
  int val = 0;
#ifdef CARMEN_LASER_USE_SELECT
  fd_set read_set;
  struct timeval timer;
  timer.tv_sec=1;
  timer.tv_usec=0;//1000*MAX_TIME_FOR_RESPONDING;
  FD_ZERO(&read_set);
  FD_SET(laser->dev.fd, &read_set);
#endif

  /* SYNC CHARS */
  buffer[pos++] = 0x02;
  /* ADDRESS    */
  buffer[pos++] = 0x00;
  /* MESSAGE LENGTH */
  length = 1 + arg_length;
  buffer[pos++] = length & 0x00ff;
  buffer[pos++] = length / 256;	/* I wonder if that works */
  /* COMMAND */
  buffer[pos++] = command;
  /* ARGUMENT */
  if(arg_length > 0)
    for(i=0; i < arg_length; i++)
      buffer[pos++] = argument[i];
  /* CHECKSUM */
  check = sick_compute_checksum(buffer, length + 4);
  buffer[pos++] = check & 0x00ff;
  buffer[pos++] = check / 256;
  carmen_serial_writen(laser->dev.fd, buffer, pos);

  /* wait for acknowledgement */
  loop = 1;
  answer = INI;
#ifdef CARMEN_LASER_USE_SELECT
  loop=select(laser->dev.fd+1, &read_set, NULL, NULL, &timer);
  if(loop) {
#else
  while(loop) {
#endif
    counter++;
    val = carmen_serial_numChars(laser->dev.fd);
    if(val > 0) {
      read(laser->dev.fd, &buffer, val);
      switch(buffer[0]) {
      case 0x02:
	answer = STX;
	break;
      case 0x06:
	answer = ACK;
	break;
      case 0x10:
	answer = DLE;
	break;
      case 0x15:
	answer = NAK;
	break;
      default:
	answer = UKN;
      }
      loop = 0;
    } 
    else if(counter > 5) {
      answer = TIO;
      loop = 0;
    }
#ifndef CARMEN_LASER_USE_SELECT
    usleep(1000); 
#endif
  }
  return answer;
}

void sick_request_status(sick_laser_p laser)
{
  sick_write_command(laser, 0x31, NULL, 0);
}

void sick_request_sensor(sick_laser_p laser)
{
  static unsigned char args[2] = {0x05, 0xb4};

  sick_write_command(laser, 0x30, args, 2);
}

int sick_set_laser_baudrate(sick_laser_p laser, int brate)
{
  unsigned char args[1];
  int result;

  if(brate == 500000)
    args[0] = 0x48;
  else if(brate == 38400)
    args[0] = 0x40;
  else if(brate == 19200)
    args[0] = 0x41;
  else
    args[0] = 0x42;
  result = sick_write_command(laser, 0x20, args, 1);
  return (result == ACK);
}

int sick_set_config_mode(sick_laser_p laser)
{
  unsigned char data[MAX_COMMAND_SIZE], args[MAX_COMMAND_SIZE];
  int i, result;

  for(i = 0; i < 8; i++)
    args[i + 1] = (unsigned char)laser->dev.passwd[i];
  args[0] = 0x00;
  carmen_serial_ClearInputBuffer(laser->dev.fd);
  result = sick_write_command(laser, 0x20, args, 9);
  if(result == ACK)
    return(sick_read_data(laser, data, MAX_TIME_FOR_CONFIG));
  else
    return(FALSE);
}

int sick_set_lms_resolution(sick_laser_p laser)
{
  unsigned char args[4];
  int result;

  if(laser->settings.angle_range == 100) {
    args[0] = 0x64; args[1] = 0x00;
    if(laser->settings.angle_resolution == RES_1_00_DEGREE) {
      args[2] = 0x64; args[3] = 0x00;
    } 
    else if(laser->settings.angle_resolution == RES_0_50_DEGREE) {
      args[2] = 0x32; args[3] = 0x00;
    } 
    else {
      args[2] = 0x19; args[3] = 0x00;
    }
  } 
  else {
    args[0] = 0xB4; args[1] = 0x00;
    if(laser->settings.angle_resolution == RES_1_00_DEGREE) {
      args[2] = 0x64; args[3] = 0x00;
    } else {
      args[2] = 0x32; args[3] = 0x00;
    }
  }
  result = sick_write_command(laser, 0x3B, args, 4);
  return(result == ACK);
}

int sick_request_lms_config(sick_laser_p laser)
{
  int result;
  
  result = sick_write_command(laser, 0x74, NULL, 0);
  return(result == ACK);
}

int sick_set_lms_config(sick_laser_p laser, unsigned char *data, int len)
{
  int result;

  if(len == 32) {
    result = sick_write_command(laser, 0x77, data, len);
    return(result == ACK);
  } 
  else
    return(FALSE);
}

int sick_parse_conf_data(sick_laser_p laser, unsigned char *buf, int length)
{
  int check, i;
  unsigned char data[32];
  
  if(length < 4)
    return(FALSE);
  else
    check = sick_compute_checksum(buf, length - 2);
  if((length != 42 && length != 40) || buf[4] != 0xF4 ||
     buf[length - 2] != (check & 0x00ff) || buf[length - 1] != (check / 256))
    return(FALSE);
  for(i = 0; i < 32; i++)
    data[i] = buf[i + 5];
  if((laser->settings.range_res == CM && data[6] != 0) ||
     (laser->settings.range_res == MM && data[6] != 1) ||
     (laser->settings.range_res == DM && data[6] != 2) ||
     (laser->settings.range_dist == SICK_RANGE80M  && data[5] != 1) ||
     (laser->settings.range_dist == SICK_RANGE160M && data[5] != 3) ||
     (laser->settings.range_dist == SICK_RANGE320M && data[5] != 5) ||
     (laser->settings.range_dist == SICK_REMISSION_NORM && data[5] != 13) ||
     (laser->settings.range_dist == SICK_REMISSION_DIRECT && data[5] != 14)) {

    //    fprintf(stderr, "ok\n");
    fprintf(stderr, "config-mode ... ");
    do {} while (!sick_set_config_mode(laser));
    fprintf(stderr, "ok ...");
  
    /* fix the laser sunlight problem */
    //    data[4] |= 1;

    switch(laser->settings.range_dist) {
    case SICK_RANGE80M:
      data[5] = 1; 
      break;
    case SICK_RANGE160M:
      data[5] = 3;
      break;
    case SICK_RANGE320M:
      data[5] = 5;
      break;
    case SICK_REMISSION_NORM:
      data[5] = 13;
      break;
    case SICK_REMISSION_DIRECT:
      data[5] = 14;
      break;
    default:
      data[5] = 1;
      break;
    }
    switch(laser->settings.range_res) {
    case CM:
      data[6] = 0;
      break;
    case MM:
      data[6] = 1;
      break;
    case DM:
      data[6] = 2;
      break;
    default:
      data[6] = 0;
      break;
    }
    fprintf(stderr, " set LMS config ... ");
    do {} while (!sick_set_lms_config(laser, data, 32));
    fprintf(stderr, "ok ...");
  }
  return(TRUE);
}

int sick_set_lms_range(sick_laser_p laser)
{
  int l = 0;
  unsigned char data[BUFFER_SIZE];

  if(sick_request_lms_config(laser)) {
    usleep(100000);
    l = sick_read_data(laser, data, MAX_TIME_FOR_GETTING_CONF);
    return(sick_parse_conf_data(laser, data, l));
  } 
  else
    return(FALSE);
}

void sick_start_continuous_mode(sick_laser_p laser)
{
  unsigned char lmsarg[1] = {0x24}, pls180arg[1] = {0x20};
  unsigned char pls360arg[1] = {0x24};
  int result = 0;
  
  do {
    if(laser->settings.type == LMS)
      result = sick_write_command(laser, 0x20, lmsarg, 1);
    else if(laser->settings.type == PLS &&
	    laser->settings.angle_resolution == RES_1_00_DEGREE)
      result = sick_write_command(laser, 0x20, pls180arg, 1);
    else if(laser->settings.type == PLS &&
	    laser->settings.angle_resolution == RES_0_50_DEGREE)
      result = sick_write_command(laser, 0x20, pls360arg, 1);
  } while(result != ACK);
}

void sick_stop_continuous_mode(sick_laser_p laser)
{
  unsigned char args[1] = {0x25};
  int result;

  do {
    result = sick_write_command(laser, 0x20, args, 1);
  } while(result != ACK);
}

// *** REI - START *** //
void sick_start_continuous_remission_part_mode(sick_laser_p laser)
{
  unsigned char lmsarg[7] = {0x2b, 1,0,0,0,0,0}, pls180arg[1] = {0x20};
  unsigned char pls360arg[1] = {0x24};
  int result = 0;
  
  if(laser->settings.type == LMS) {
    lmsarg[3] = 1;
    lmsarg[5] = laser->settings.rem_values;
  }
  else if(laser->settings.type == PLS) {
	laser->settings.use_remission = 0;
	fprintf(stderr, "WARNING: No remission-mode support for non LMS lasers. Continuing normal mode.\n"); 
  }

  do {
    if(laser->settings.type == LMS) 
      result = sick_write_command(laser, 0x20, lmsarg, 7);
    else if(laser->settings.type == PLS &&
	    laser->settings.angle_resolution == RES_1_00_DEGREE)
      result = sick_write_command(laser, 0x20, pls180arg, 1);
    else if(laser->settings.type == PLS &&
	    laser->settings.angle_resolution == RES_0_50_DEGREE)
      result = sick_write_command(laser, 0x20, pls360arg, 1);
  } while(result != ACK);
}
// *** REI - END *** //

/* sick_test_baudrate - Test a combination of baudrate and parity 
   of the laser. */

int sick_testBaudrate(sick_laser_p laser, int brate)
{
  unsigned char data[BUFFER_SIZE], ReqLaser[2] = {5, 180};
  int response;

  sick_set_baudrate(laser, brate);
  
  response = sick_write_command(laser, 0x30, ReqLaser, 2);
  if(response == NAK) {
    fprintf(stderr, "Error : Could not send command to laser.\n");
    return FALSE;
  }
  if(sick_read_data(laser, data, MAX_TIME_FOR_TESTING_BAUDRATE))
    return TRUE;
  return FALSE;
}

int sick_detect_baudrate(sick_laser_p laser)
{
  fprintf(stderr, "INFO: detect connected baudrate: ...... 9600");
  if(sick_testBaudrate(laser, 9600)) {
    fprintf(stderr, "\n");
    return(9600);
  } 
  else {
    fprintf(stderr, "\rINFO: detect connected baudrate: ...... 19200");
    if(sick_testBaudrate(laser, 19200)) {
      fprintf(stderr, "\n");
      return(19200);
    } 
    else {
      fprintf(stderr, "\rINFO: detect connected baudrate: ...... 38400");
      if(sick_testBaudrate(laser, 38400)) {
	fprintf(stderr, "\n");
	return(38400);
      } 
      else {
	if(laser->settings.use_highspeed) {
	  fprintf(stderr, "\rINFO: detect connected baudrate: ...... 500000");
	  if(sick_testBaudrate(laser, 500000)) {
	    fprintf(stderr, "\n");
	    return(500000);
	  } 
	  else {
	    fprintf(stderr, "\rINFO: detect connected baudrate: ...... failed\n");
	    return(0);
	  }
	} 
	else {
	  fprintf(stderr, "\rINFO: detect connected baudrate: ...... failed\n");	  
	  return(0);
	}
      }
    }
  }
}

int sick_check_baudrate(sick_laser_p laser, int brate)
{
  fprintf(stderr, "check baudrate:\n");
  fprintf(stderr, "    %d ... ", brate);
  if(sick_testBaudrate(laser, brate)) {
    fprintf(stderr, "yes\n");
    return(TRUE);
  } 
  else {
    fprintf(stderr, "no\n");
    return(FALSE);
  }
}

void sick_install_settings(sick_laser_p laser)
{
  laser->dev.type = laser->settings.type;
  laser->dev.baudrate = laser->settings.start_baudrate;
  laser->dev.parity = laser->settings.parity;
  laser->dev.fd = -1;
  laser->dev.databits = laser->settings.databits;
  laser->dev.stopbits = laser->settings.stopbits;
  laser->dev.hwf = laser->settings.hwf;
  laser->dev.swf = laser->settings.swf;
  strncpy((char *)laser->dev.passwd, (const char *)laser->settings.password, 8);
  laser->dev.ttyport =
    (char *)malloc((strlen(laser->settings.device_name) + 1) * sizeof(char));
  carmen_test_alloc(laser->dev.ttyport);
  strcpy(laser->dev.ttyport, laser->settings.device_name);
}

void sick_allocate_laser(sick_laser_p laser)
{
  int i;
  
  laser->numvalues = laser->settings.num_values;
  laser->range = (double *)malloc(laser->settings.num_values * sizeof(double));
  carmen_test_alloc(laser->range);
  laser->glare = (int *)malloc(laser->settings.num_values * sizeof(int));
  carmen_test_alloc(laser->glare);
  laser->wfv = (int *)malloc(laser->settings.num_values * sizeof(int));
  carmen_test_alloc(laser->wfv);
  laser->sfv = (int *)malloc(laser->settings.num_values * sizeof(int));
  carmen_test_alloc(laser->sfv);
  laser->buffer = (unsigned char *)malloc(LASER_BUFFER_SIZE);
  carmen_test_alloc(laser->buffer);
  laser->new_reading = 0;
  laser->buffer_position = 0;
  laser->processed_mark = 0;
  for(i = 0; i < laser->settings.num_values; i++) {
    laser->range[i] = 0.0;
    laser->glare[i] = 0;
    laser->wfv[i] = 0;
    laser->sfv[i] = 0;
  }
// *** REI - START *** //
  if(laser->settings.use_remission) {
    laser->remvalues = laser->settings.rem_values;
	laser->remission = (double *)malloc(laser->settings.rem_values * sizeof(double));
	carmen_test_alloc(laser->remission);
	for(i = 0; i< laser->settings.rem_values; i++) 
	  laser->remission[i] = 0;
  }
// *** REI - END *** //
}

void sick_connect_device(sick_laser_p laser)
{
  sick_install_settings(laser);
  sick_allocate_laser(laser);
  fprintf(stderr, "INFO: connect TTY %-14s ...... ", laser->dev.ttyport);
  sick_serial_connect(laser);
  if(laser->dev.fd == -1) {
    fprintf(stderr, "failed\n");
    exit(1);
  }
  fprintf(stderr, "ok\n");
  fprintf(stderr, "INFO: set port param %6d:%d%c%d ....... ",
	  laser->dev.baudrate, laser->dev.databits,
	  (laser->dev.parity == N ? 'N' : laser->dev.parity == E ? 'E' : 'O'),
	  laser->dev.stopbits);
  sick_set_serial_params(laser);
  fprintf(stderr, "ok\n");
}

void sick_start_laser(sick_laser_p laser)
{
  int brate = 0;

  fprintf(stderr, "###########################################\n");
  fprintf(stderr, "INFO: select mode ..................... ");
#ifdef CARMEN_LASER_USE_SELECT
  fprintf(stderr, "on\n");
#else
  fprintf(stderr, "off\n");
#endif
  fprintf(stderr, "INFO: LASER type ...................... ");
  fprintf(stderr, "%s\n", (laser->settings.type == PLS) ? "PLS" : "LMS");
  
  /* open the serial port */
  sick_connect_device(laser);
  
  /* make sure the baudrate is set correctly, change it if necessary */
  if(laser->settings.detect_baudrate) {
    brate = sick_detect_baudrate(laser);
    if(!brate)
      exit(1);
  } 
  else if(!sick_check_baudrate(laser, laser->settings.start_baudrate)) {
    fprintf(stderr, "ERROR: communication does not work!\n");
    exit(1);
  }
  if(brate != laser->settings.set_baudrate) {
    fprintf(stderr, "INFO: set LASER in config-mode ........ ");
    while(!sick_set_config_mode(laser));
    fprintf(stderr, "ok\n");
    fprintf(stderr, "INFO: set LASER baudrate to %6d .... ",
	    laser->settings.set_baudrate);
    while(!sick_set_laser_baudrate(laser, laser->settings.set_baudrate));
    sick_set_baudrate(laser, laser->settings.set_baudrate);
    fprintf(stderr, "ok\n");
  }
  
  /* set the resolution of the blue lasers */
  if(laser->settings.type == LMS) {
    fprintf(stderr, "INFO: angle range ..................... %3d\n",
	    laser->settings.angle_range);
    fprintf(stderr, "INFO: angle resolution ................ %1.2f\n",
	    (laser->settings.angle_resolution == RES_1_00_DEGREE ? 1.0 :
	     laser->settings.angle_resolution == RES_0_50_DEGREE ? 0.5 : 
	     0.25));
    fprintf(stderr, "INFO: set LASER mode .................. ");
    while(!sick_set_lms_resolution(laser));
    fprintf(stderr, "ok\n");
    usleep(100000);
    fprintf(stderr, "INFO: get LMS configuration ........... ");
    while(!sick_set_lms_range(laser));
    fprintf(stderr, "ok\n");
  }
  
  /* set the laser to continuous mode */
// *** REI - START *** //
  if(laser->settings.use_remission == 1) {

    fprintf(stderr, "INFO: using remission mode ............ ");
    if (laser->settings.range_dist == SICK_REMISSION_NORM) {
      fprintf(stderr, "normalized\n");
    }
    else  if (laser->settings.range_dist == SICK_REMISSION_DIRECT) {
      fprintf(stderr, "direct\n");
    }
    else fprintf(stderr, "unknwon\n");

    fprintf(stderr, "INFO: start LASER continuous remission mode ..... ");
	sick_start_continuous_remission_part_mode(laser);
//	sick_start_remission_part_mode(laser);
// *** REI - END *** //
  } else {
    fprintf(stderr, "INFO: using remission mode ............ none\n");
    fprintf(stderr, "INFO: start LASER continuous mode ..... ");
	sick_start_continuous_mode(laser);
  }
  laser->packet_timestamp=-1;
  fprintf(stderr, "ok\n");
  fprintf(stderr, "###########################################\n");
}

/* sick_valid_packet - This function returns 1 if a valid packet is
   detected in a chunk of data.  An offset and packet length are 
   returned. */

int sick_valid_packet(unsigned char *data, long size,
		      long *offset, long *len)
{
  int i, check, packet_size = 0, theo_size = 0;

  for(i = 0; i < size; i++) {
    if(packet_size == 0 && *data == 0x02)
      packet_size++;
    else if(packet_size == 1 && *data == 0x80)
      packet_size++;
    else if(packet_size == 1)
      packet_size = 0;
    else if(packet_size == 2) {
      theo_size = data[0];
      packet_size++;
    } 
    else if(packet_size == 3) {
      theo_size += (data[0] << 8) + 6;
      if(size >= theo_size + (i - packet_size)) {	// check the crc
	check = data[theo_size - 3 - 2];
	check += data[theo_size - 3 - 1] << 8;                                
	if(check != sick_compute_checksum(data - 3, theo_size - 2)) {
	  i -= 2;
	  data -= 2;
	  packet_size = 0;
	}
	else {
	  *offset = i - packet_size;
	  *len = theo_size;
	  return 1;
	}
      } 
      else
	packet_size = 0;
    }
    data++;
  }
  return 0;
}

/* sick_process_packet - Interpret packets received from the laser.  If
   the packets contain laser data, expand the data into a useful form. */

static void sick_process_packet_distance(sick_laser_p laser, unsigned char *packet)
{
  int i = 0, LoB = 0, HiB = 0, bit14, bit15, numMeasurements;
  float conversion = 1.0;

  if(packet[0] == 0xb0) {

	 
    /* Check the number of measurements */
    numMeasurements = ((packet[2] << 8) + packet[1]) & 0x3FFF;

    /* Extract the distance conversion factor */
    bit14 = packet[2] & 0x40;
    bit15 = packet[2] & 0x80;
    if(laser->settings.type == LMS) {
      if(!bit15)
	if(!bit14)
	  conversion = 1.0;
	else
	  conversion = 0.1;
      else
	conversion = 10.0;
    }
    
    /* Compute range values */
    laser->new_reading = 1;
    for (i = 0; i < numMeasurements; i++) {
      LoB = packet[i * 2 + 3]; 
      HiB = packet[i * 2 + 4];
      laser->range[i] = ((HiB & 0x1f) * 256 + LoB) * conversion;
      laser->glare[i] = (HiB & 0x20) / 32; 
      laser->wfv[i] = (HiB & 0x40) / 64;  
      laser->sfv[i] = (HiB & 0x80) / 128;
    }
  }
}

// *** REI - START *** //
static void sick_process_packet_remission(sick_laser_p laser, unsigned char *packet)
{
  int i = 0, LoB = 0, HiB = 0, bit14, bit15, numMeasurements;
  int parts, mstart, mend;
  int offs;
  float conversion = 1.0;

  if(packet[0] == 0xf5) {

    /* Extract number of scan parts (should be 1) */
	parts = packet[1] & 0x7;
	 
    offs = 0;

    mstart = ((packet[offs + 4] << 8) + packet[offs + 3]);
	mend   = ((packet[offs + 6] << 8) + packet[offs + 5]);
//  fprintf(stderr, "mstart, mend = %d, %d\n", mstart, mend);
   
	/* Check the number of measurements */
    numMeasurements = ((packet[offs + 8] << 8) + packet[offs + 7]) & 0x3FFF;
//  fprintf(stderr, "num_measurem. = %d\n",numMeasurements);

    /* Extract the distance conversion factor */
    bit14 = packet[offs + 8] & 0x40;
    bit15 = packet[offs + 8] & 0x80;
    if(laser->settings.type == LMS) {
      if(!bit15)
	if(!bit14)
	  conversion = 1.0;
	else
	  conversion = 0.1;
      else
	conversion = 10.0;
    }
    
    /* Compute range values */
    laser->new_reading = 1;
    for (i = 0; i < numMeasurements; i++) {
      LoB = packet[i * 4 + offs + 9]; 
      HiB = packet[i * 4 + offs + 10];
      laser->range[i] = (HiB * 256 + LoB) * conversion;
	  laser->remission[i] = packet[i * 4 + offs + 12] * 256 + packet[i * 4 + offs + 11];
    }
  }
}

static void sick_process_packet(sick_laser_p laser, unsigned char *packet)
{
  if(laser->settings.use_remission == 1)
	sick_process_packet_remission(laser, packet);
  else sick_process_packet_distance(laser, packet);
}
// *** REI - END *** //

/* sick_handle_laser - Process any data that is available from the 
   laser. Attempt to detect valid packets in the data. */

void sick_handle_laser(sick_laser_p laser)
{
  int bytes_available, bytes_read;
  int leftover;
  
  laser->new_reading = 0;
#ifdef CARMEN_LASER_USE_SELECT
  double timeout=0.1;
  fd_set read_set;
  struct timeval timer;
  timer.tv_sec=(long)(floor(timeout));
  timer.tv_usec=(long)((timeout-floor(timeout))*1000000);
  FD_ZERO(&read_set);
  FD_SET(laser->dev.fd, &read_set);
  select(laser->dev.fd+1, &read_set, NULL, NULL, &timer);
#endif
  /* read what is available in the buffer */
  bytes_available = carmen_serial_numChars(laser->dev.fd);
  if(bytes_available > LASER_BUFFER_SIZE - laser->buffer_position){
    bytes_available = LASER_BUFFER_SIZE - laser->buffer_position;
  }
  bytes_read = carmen_serial_readn(laser->dev.fd, laser->buffer +
				   laser->buffer_position, bytes_available);

  /* process at most one laser reading */
  if(bytes_read > 0) {
      if (laser->packet_timestamp<0.){
	  laser->packet_timestamp=carmen_get_time();
      }
      laser->buffer_position += bytes_read;
      if(sick_valid_packet(laser->buffer + laser->processed_mark,
			   laser->buffer_position - laser->processed_mark,
			   &(laser->packet_offset), &(laser->packet_length))) {
	  laser->timestamp=laser->packet_timestamp;
	  sick_process_packet(laser, laser->buffer + laser->processed_mark +
			  laser->packet_offset + 4);
	  laser->packet_timestamp=-1.;

	  leftover = laser->buffer_position - laser->processed_mark - 
	      laser->packet_length;
	  laser->processed_mark += laser->packet_offset + laser->packet_length;
	  //PACKET_DROPPING
	  while (leftover>laser->packet_length) {
	      laser->processed_mark +=laser->packet_length;
	      leftover-=laser->packet_length;
	      fprintf(stderr,"D");
	  }
	  if(leftover == 0) {
	      laser->buffer_position = 0;
	      laser->processed_mark = 0;
	  }
    }
  }
  
  
  /* shift everything forward in the buffer, if necessary */
  if(laser->buffer_position > LASER_BUFFER_SIZE / 2) {
    memmove(laser->buffer, laser->buffer + laser->processed_mark,
	    laser->buffer_position - laser->processed_mark);
    laser->buffer_position = laser->buffer_position - laser->processed_mark;
    laser->processed_mark = 0;
  } 
}

void sick_stop_laser(sick_laser_p laser)
{
  fprintf(stderr, "\nINFO: stop LASER continuous mode ....... ");
  sick_stop_continuous_mode(laser);
  fprintf(stderr, "ok\n");
  close(laser->dev.fd);
}
