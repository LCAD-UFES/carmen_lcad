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
#include <limits.h>

#define        PIONEER_SERIAL_TIMEOUT            5.0
#define        METRES_PER_INCH          0.0254
#define        WHEELBASE                (16.5 * METRES_PER_INCH) // need to set this
#define        ROT_VEL_FACT_RAD         (WHEELBASE)

static int dev_fd;

static double max_t_vel = 1.0;

#define PIONEER_PACKET_HEADER  "\xFA\xFB"

#define PIONEER_SYNC0           0
#define PIONEER_SYNC1           1
#define PIONEER_SYNC2           2

#define PIONEER_PULSE           0
#define PIONEER_OPEN            1
#define PIONEER_CLOSE           2
#define PIONEER_ENABLE          4
#define PIONEER_SETA            5
#define PIONEER_STOP            29
#define PIONEER_VEL2            32

#define PIONEER_NEGATIVE_INT    0x1B
#define PIONEER_POSITIVE_INT    0x3B

#define PIONEER_ON              1
#define PIONEER_OFF             0

#define PIONEER2_VEL2_CONVERSION_FACTOR             50.0              // m/s => 2cm/sec
#define PIONEER_MAX_NUM_SONARS  16

struct pioneer_raw_information_packet {
  unsigned char motor_status;
  unsigned char x[2];
  unsigned char y[2];
  unsigned char orientation[2];
  unsigned char vl[2];
  unsigned char vr[2];
  unsigned char battery;
  unsigned char bumper[2];
  unsigned char servo_position[2];
  unsigned char pulse_width[2];
  unsigned char compass_heading;
  unsigned char num_sonar_readings;
  struct {
    unsigned char n;
    unsigned char range[2];
  } sonar[3*PIONEER_MAX_NUM_SONARS];
};

static int 
pioneer_buf_to_checksum(unsigned char *buf) 
{
  return 256*buf[0] + buf[1];
}

static int 
pioneer_calculate_checksum(unsigned char *ptr, int length) 
{
  unsigned int c = 0;
  
  while(length > 1) 
    {
      c += (*(ptr)<<8) | *(ptr+1);
      c = c & 0xffff;
      length -= 2;
      ptr += 2;
    }

  if(length > 0) 
    c = c ^ (int)*(ptr++);

  return(c);
}


static void 
pioneer_int_to_buf(unsigned char *buf, int n) 
{
  if (n >= 0)
    buf[0] = PIONEER_POSITIVE_INT;
  else
    buf[0] = PIONEER_NEGATIVE_INT;

  n = abs(n);

  buf[1] = n & 0x00FF;
  buf[2] = (n & 0xFF00) >> 8;
}
 
static int 
pioneer_read_string(unsigned char *buf, double read_timeout) 
{
  int c = 0;
  int header_length = strlen(PIONEER_PACKET_HEADER);
  unsigned char header[header_length+1];
  unsigned char length_char[1];
  int i;
  double start_time, current_time;

  for (i = 0; i <= header_length; ++i)
    header[i] = '\0';

  start_time = carmen_get_time();
  do 
    {                                  /* busy waits until we read the   */
      for (i = 1; i < header_length; ++i) /* command header                 */
	header[i-1] = header[i];
      carmen_serial_readn(dev_fd, header + header_length - 1, 1);

      if (read_timeout > 0) {
	current_time = carmen_get_time();
	if (current_time - start_time > read_timeout)
	  // timeout
	  return -1;
      }
    } 
  while (strcmp((char *)header, (const char *)PIONEER_PACKET_HEADER) != 0);
  
  carmen_serial_readn(dev_fd, length_char, 1);
  c = (int) length_char[0];             /* the length, including chksum   */

  carmen_serial_readn(dev_fd, buf, c);
  c -= 2;                               /* removes checksum from length,  */
                                        /* checksum is now at buf+c       */
  if (pioneer_calculate_checksum(buf, c) != pioneer_buf_to_checksum(buf+c)) 
    {
      fprintf(stderr, "Checksum error on packet ");
      for (i = 0; i < c; ++i)
	fprintf(stderr, "%c", buf[i]);
      fprintf(stderr, "\n");
      return pioneer_read_string(buf, read_timeout);
    }

  return c;
}

static int 
pioneer_send_string(unsigned char *ptr, int length) 
{
  int send_length = length + 5;         /* adds a 2-byte packet header,   */
                                        /* a 1-byte length (inc. chksum), */
                                        /* and a 2-byte checksum          */
  unsigned char cmd[256] = PIONEER_PACKET_HEADER;
  int checksum;
  int i;

  if (length <= 0)
    return 0;

  cmd[2] = (unsigned char) length+2;

  for (i = 0; i < length; ++i)
    cmd[i+3] = ptr[i];

  checksum = pioneer_calculate_checksum(ptr, length);
  cmd[send_length-2] = (unsigned char) ((checksum & 0xFF00) >> 8);
  cmd[send_length-1] = (unsigned char) (checksum & 0x00FF);

  return carmen_serial_writen(dev_fd, cmd, send_length);
}

static int 
pioneer_send_command0(char cmd) 
{
  char buf[256];

  buf[0] = cmd;
  return pioneer_send_string((unsigned char *)buf, 1);
}

static int 
pioneer_send_command1(char cmd, int arg1) 
{
  char buf[256];
  
  buf[0] = cmd;
  pioneer_int_to_buf((unsigned char *)(buf+1), arg1);
  return pioneer_send_string((unsigned char *)buf, 4);
}

static void 
pioneer_seta(int accel, int decel) 
{
  accel = abs(accel);
  decel = abs(decel);
  accel *= 10;
  decel *= 10;
  pioneer_send_command1(PIONEER_SETA, accel);
  pioneer_send_command1(PIONEER_SETA, -decel);
}


static int 
pioneer_sync0(void) 
{
  unsigned char buf[256];
  buf[0] = !PIONEER_SYNC0;

  pioneer_send_command0(PIONEER_SYNC0);
  pioneer_read_string(buf, 0.2);

  if (buf[0] != PIONEER_SYNC0) {
    carmen_warn("Could not SYNC0\n");
    return -1;
  }

  return 0;
}


static int 
pioneer_sync1(void) 
{
  unsigned char buf[256];

  pioneer_send_command0(PIONEER_SYNC1);
  pioneer_read_string(buf, PIONEER_SERIAL_TIMEOUT);
  if (buf[0] != PIONEER_SYNC1) {
    fprintf(stderr, "Could not SYNC1\n");
    return -1;
  }

  return 0;
}


static int 
pioneer_sync2(void) 
{
  unsigned char buf[256];

  pioneer_send_command0(PIONEER_SYNC2);
  pioneer_read_string(buf, PIONEER_SERIAL_TIMEOUT);

  return 0;
}


void 
pioneer_vel2(int vl, int vr) 
{
  int arg;

  arg = 256*vl + vr;
  pioneer_send_command1(PIONEER_VEL2, arg);
}

static void 
set_wheel_velocities(double vl, double vr)
{
  if(vl > max_t_vel)
    vl = max_t_vel;
  else if(vl < -max_t_vel)
    vl = -max_t_vel;
  if(vr > max_t_vel)
    vr = max_t_vel;
  else if(vr < -max_t_vel)
    vr = -max_t_vel;

  vl *= PIONEER2_VEL2_CONVERSION_FACTOR;
  vr *= PIONEER2_VEL2_CONVERSION_FACTOR;

  if (vl > 127)
    vl = 127;
  else if  (vl < -127)
    vl = -127;
  if (vr > 127)
    vr = 127;
  else if (vr < -127)
    vr = -127;

  pioneer_vel2((int)vl, (int)vr);
}


void 
shutdown_pioneer(int signo __attribute__ ((unused))) 
{
  fprintf(stderr, "\nShutting down robot...");
  sleep(1);
  pioneer_send_command0(PIONEER_CLOSE);
  fprintf(stderr, "done.\n");
  carmen_terminal_restore();
  exit(0);
}

int 
main(int argc __attribute__ ((unused)), 
     char **argv __attribute__ ((unused))) 
{
  double wheel_diameter;
  char dev_name[100];
  double vl, vr;
  struct pioneer_raw_information_packet r;
  int k;
  double tv, rv;

  signal(SIGINT, shutdown_pioneer);
  signal(SIGTERM, shutdown_pioneer);

  carmen_terminal_cbreak(0);

  strcpy(dev_name, "/dev/ttyS24");
  wheel_diameter = 0.165;

  if (carmen_serial_connect(&dev_fd, dev_name) < 0) 
    return -1;

  if (pioneer_sync0() < 0)
    return -1;

  if (pioneer_sync1() < 0)
    return -1;

  if (pioneer_sync2() < 0)
    return -1;

  pioneer_send_command0(PIONEER_OPEN);
  pioneer_send_command0(PIONEER_PULSE);  
  pioneer_send_command1(PIONEER_ENABLE, PIONEER_ON);

  pioneer_seta(10, 3.0);

  while(1) {
    fprintf(stderr, ".");
    pioneer_send_command0(PIONEER_PULSE);  
    if (pioneer_read_string((unsigned char*) &r, PIONEER_SERIAL_TIMEOUT) < 0)
      carmen_die("Serial read timed out\n");

    k = getchar();

    if (k != EOF) {
      switch(k) {
      case 'u': tv = 0.5; rv = 0.2; break;
      case 'i': tv = 0.5; rv = 0.0; break;
      case 'o': tv = 0.5; rv = -0.2; break;
      case 'j': tv = 0.0; rv = 0.2; break;
      case 'k': tv = 0.0; rv = 0; break;
      case 'l': tv = 0.0; rv = -0.2; break;
      case 'm': tv = -0.5; rv = 0.2; break;
      case ',': tv = -0.5; rv = 0; break;
      case '.': tv = -0.5; rv = -0.2; break;
      default:  tv = 0.0; rv = 0; break;
      }
      if (tv == 0.0 && rv == 0.0) {
	pioneer_send_command0(PIONEER_STOP);
      } else {
	vl = tv;
	vr = tv;
	vl -= 0.5 * rv * ROT_VEL_FACT_RAD*3;
	vr += 0.5 * rv * ROT_VEL_FACT_RAD*3;
	set_wheel_velocities(vl, vr);
      }
    }
  }
  return 0;
}
