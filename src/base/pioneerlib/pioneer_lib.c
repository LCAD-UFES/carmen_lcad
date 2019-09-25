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
#include <limits.h>

#include "pioneer_params.h"

#define        PIONEER_SERIAL_TIMEOUT            5

static int dev_fd;
static int sonar_is_on = 0;
static int pioneer_version = 0;

static double angle_conv_factor, dist_conv_factor, vel_conv_factor;
static double range_conv_factor, diff_conv_factor, vel2_divisor;

/*** Some pioneer internal constants.  I have them defined here since the ***/
/*** outside world doesn't need to know about them.                       ***/

#define PIONEER_PACKET_HEADER  "\xFA\xFB"

#define PIONEER_SYNC0           0
#define PIONEER_SYNC1           1
#define PIONEER_SYNC2           2

#define PIONEER_PULSE           0
#define PIONEER_OPEN            1
#define PIONEER_CLOSE           2
#define PIONEER_POLLING         3
#define PIONEER_ENABLE          4
#define PIONEER_SETA            5
#define PIONEER_VEL             11
#define PIONEER_RVEL            21
#define PIONEER_SETRA           23
#define PIONEER_SONAR           28
#define PIONEER_STOP            29
#define PIONEER_VEL2            32

#define PIONEER_NEGATIVE_INT    0x1B
#define PIONEER_STRING          0x2B
#define PIONEER_POSITIVE_INT    0x3B

#define PIONEER_ON              1
#define PIONEER_OFF             0
#define PIONEER_MOTOR_NO_POWER  0x31
#define PIONEER_MOTOR_STOPPED   0x32
#define PIONEER_MOTOR_MOVING    0x33

#define PIONEER_BATTERY_CONVERSION_FACTOR           0.1
#define PIONEER_COMPASS_CONVERSION_FACTOR           2.0

#define PIONEER_MAX_NUM_SONARS   32
#define PIONEER_MAX_NUM_BUMPERS  14


/* Cyrill, 14.01.2004
   according to the ActivMedia manual the discretization
   is 4mm/sec, not 5mm/sec. So the factor should be 250.0
   and not 200.0 */
//#define PIONEER1_VEL2_CONVERSION_FACTOR             200.0             // m/s => 5mm/sec
#define PIONEER1_VEL2_CONVERSION_FACTOR             250.0             // m/s => 4mm/sec
#define PIONEER2_VEL2_CONVERSION_FACTOR             50.0              // m/s => 2cm/sec


/*** When we receive an information packet back from the robot, it is     ***/
/*** initially read into a raw information packet as if to a big char     ***/
/*** buffer.  It is then converted to a regular information packet, which ***/
/*** contains more useful data types.                                     ***/
struct pioneer_raw_information_packet 
{
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
  struct 
  {
    unsigned char n;
    unsigned char range[2];
  } sonar[3*PIONEER_MAX_NUM_SONARS];
};

typedef enum {PIONEER_NO_POWER, PIONEER_STOPPED, PIONEER_MOVING} 
motor_status_t;

static struct pioneer_raw_information_packet raw_state;

int pioneer_vel2(int vl, int vr) ;


/*** TABLE OF CONTENTS ***/

/*** Higher-level robot functions ***/

static int 
pioneer_buf_to_checksum(unsigned char *buf) 
{
  return 256*buf[0] + buf[1];
}


static int 
pioneer_buf_to_int(unsigned char *buf) 
{
  int val = buf[0] + 256*buf[1];
  if (val > SHRT_MAX)
    val -= USHRT_MAX;
  return val;
}


static int 
pioneer_buf_to_unsigned_int(unsigned char *buf) 
{
  int val = buf[0] + 256 * (0x7F & buf[1]);  
  return val;
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

/*** Return value:  the length of the string read                       ***/
static int 
pioneer_read_string(unsigned char *buf, double read_timeout, double * packet_timestamp) 
{
  int c = 0;
  int header_length = strlen(PIONEER_PACKET_HEADER);
  unsigned char header[header_length+1];
  unsigned char length_char[1];
  int i;
  double start_time, current_time;
  double header_time;

  for (i = 0; i <= header_length; ++i)
    header[i] = '\0';

  start_time = carmen_get_time();
  header_time = start_time;
  do 
    {                                  /* busy waits until we read the   */
      for (i = 1; i < header_length; ++i) /* command header                 */
	header[i-1] = header[i];
      carmen_serial_readn(dev_fd, header + header_length - 1, 1);
      header_time=carmen_get_time();
      if (read_timeout > 0) {
	current_time = carmen_get_time();
	if (current_time - start_time > read_timeout)
	  // timeout
	  return -1;
      }
    } 
  while (strcmp((char *)header, PIONEER_PACKET_HEADER) != 0);
 
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
      return pioneer_read_string(buf, read_timeout, NULL);
    }
  if (packet_timestamp)
	*packet_timestamp=header_time;
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

static int
pioneer_sonar(int b) 
{
  if (pioneer_version == 1 && b == 0) {
    int result = pioneer_send_command1(PIONEER_POLLING, 0);
    if (result >=0)
      sonar_is_on=0;
    return result;
  }
  else {
    int result = pioneer_send_command1(PIONEER_SONAR, b);

    if (result >=0) {
      if (b == 0)
	sonar_is_on=0;
      else
	sonar_is_on=1;
    }
    return result;
  }
}


static int 
pioneer_sync0(void) 
{
  unsigned char buf[256];
  buf[0] = !PIONEER_SYNC0;

  pioneer_send_command0(PIONEER_SYNC0);
  if ((pioneer_read_string(buf, 0.2, NULL) == -1) && (pioneer_version == 1)) 
    {
      /* sometimes the Pioneer I's want two syncs */
      pioneer_send_command0(PIONEER_SYNC0);
      if (pioneer_read_string(buf, 0.2, NULL) == -1) 
	{
	  carmen_warn("Could not SYNC0\n");
	  return -1;
	}
    }

  if (buf[0] != PIONEER_SYNC0) 
    {
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
  pioneer_read_string(buf, PIONEER_SERIAL_TIMEOUT, NULL);
  if (buf[0] != PIONEER_SYNC1) 
    {
      carmen_warn("Could not SYNC1\n");
      return -1;
    }

  return 0;
}

static int 
pioneer_sync2(void) 
{
  unsigned char buf[256];

  pioneer_send_command0(PIONEER_SYNC2);
  pioneer_read_string(buf, PIONEER_SERIAL_TIMEOUT, NULL);

  return 0;
}

int abs_max(int a, int b) {
  a = abs(a);
  b = abs(b);
  return (a>b)?a:b;
}

int
pioneer_vel2(int vl, int vr) 
{
  int arg;
  int absmax;

  absmax = abs_max(vl, vr);
  if (absmax > 127) {
    float scale_factor;
    scale_factor = 127.0 / ((float)absmax);
    vl = (int) ( scale_factor * ((float)vl));
    vr = (int) ( scale_factor * ((float)vr));
  }

  if (1)
    fprintf(stderr,"(%d %d)\n", vl, vr);

  arg = 256*vl + vr;

  return pioneer_send_command1(PIONEER_VEL2, arg);
}

int
carmen_base_direct_sonar_on(void)
{
  int err = 0;
  fprintf(stderr, "Switching sonars on\n");
    //  if (!sonar_is_on){ 
    err = pioneer_sonar(1);
    //  }
  if (err < 0)
    return -1;

  return PIONEER_MAX_NUM_SONARS;
}

int
carmen_base_direct_sonar_off(void)
{

  fprintf(stderr, "Switching sonars off\n");
  //  if (sonar_is_on) 
  return pioneer_sonar(0);

  //  return 0;
}

int 
carmen_base_direct_reset(void)
{
  return 0;
}

int 
carmen_base_direct_initialize_robot(char *model, char *dev) 
{
  int i;
  int pioneer_model;
  int result;
  int count;

  pioneer_model = -1;
  for (i = 0; carmen_pioneer_models[i][0] != 0; i++) 
  {
    if (strlen(model) == strlen(carmen_pioneer_models[i]) &&
        carmen_strncasecmp(model, carmen_pioneer_models[i], 
          strlen(carmen_pioneer_models[i])) == 0)
    {
      pioneer_model = i;
      break;
    }
  }

  if (pioneer_model == -1) 
    {
      carmen_warn("Unknown pioneer model %s\nKnown models: ", model);
      for (i = 0; carmen_pioneer_models[i][0] != 0; i++) 
	{
	  carmen_warn("%s ", carmen_pioneer_models[i]);
	  if (i % 6 == 5)
	    carmen_warn("\n");
	  carmen_warn("To determine your pioneer model, read \n"
		      "carmen/src/pioneer/README\n");
	  return -1;
	}
    }

  // if needed to make gcc 4.3.1 happy
  if (pioneer_model != -1) {
    angle_conv_factor = carmen_pioneer_params[pioneer_model][0];
    dist_conv_factor = carmen_pioneer_params[pioneer_model][1];
    vel_conv_factor = carmen_pioneer_params[pioneer_model][2];
    range_conv_factor = carmen_pioneer_params[pioneer_model][3];
    diff_conv_factor = carmen_pioneer_params[pioneer_model][4];
    vel2_divisor = carmen_pioneer_params[pioneer_model][5];
  }

  if (pioneer_model < 17) 
    pioneer_version = 2;
  else
    pioneer_version = 1;

  if (carmen_serial_connect(&dev_fd, dev) < 0) 
    return -1;

  count = 0;
  do 
    {
      result = pioneer_sync0();
      if (result < 0 && count == 4)
	return -1;
      count++;
    }
  while (result < 0);

  if (pioneer_sync1() < 0)
    return -1;

  if (pioneer_sync2() < 0)
    return -1;

  pioneer_send_command0(PIONEER_OPEN);
  pioneer_sonar(0);
  pioneer_send_command0(PIONEER_PULSE);

  if (pioneer_version == 2) 
    pioneer_send_command1(PIONEER_ENABLE, PIONEER_ON);

  return 0;
}

int 
carmen_base_direct_shutdown_robot(void) 
{
  if (sonar_is_on) {
    carmen_base_direct_sonar_off();
    usleep(1000000);
  }

  pioneer_send_command0(PIONEER_CLOSE);
  usleep(1000000);
  close(dev_fd);
  fprintf( stderr, "\nClosed\n" );
  return(0);
}

int 
carmen_base_direct_set_acceleration(double acceleration)
{
  acceleration = fabs(acceleration) * 1000;

  return pioneer_send_command1(PIONEER_SETA, acceleration);
}

int 
carmen_base_direct_set_deceleration(double deceleration)
{
  deceleration = -fabs(deceleration) * 1000;

  return pioneer_send_command1(PIONEER_SETA, deceleration);
}

int 
carmen_base_direct_set_velocity(double tv, double rv)
{
  /** Cyrill, 13.Jan.2004
      Pioneer drives much nicer when using VEL2 instead
      of VEL and RVEL commands.
 
      ActiveMedia's strage dist_conf_factor comes from:
      distBetweemLeftAndRightWheel = 0.002 / dist_conf_factor
  */
  int result = 0;
  double vl = tv  - 0.001 * rv / diff_conv_factor;
  double vr = tv  + 0.001 * rv / diff_conv_factor;

  if (pioneer_version == 1)   {
    vl *= PIONEER1_VEL2_CONVERSION_FACTOR;
    vr *= PIONEER1_VEL2_CONVERSION_FACTOR;
  }
  else   {
    vl *= PIONEER2_VEL2_CONVERSION_FACTOR;
    vr *= PIONEER2_VEL2_CONVERSION_FACTOR;
  }

  result =  pioneer_vel2((int) rint(vl), (int) rint(vr));
  
  if (result < 0)
    return -1;
  return 0;

  /* The old method to send the velocities to the robot.
     Pioneer Robots (can) drive quite poor if using
     VEL and RVEL command, that's why I changed the
     code to the  VEL2 command */
  
/*   int err;  */
/*   int arg; */
/*   arg = tv * 1000; */
/*   err = pioneer_send_command1(PIONEER_VEL, arg); */
/*   if (err < 0) */
/*     return -1; */
  
/*   arg = carmen_radians_to_degrees(rv); */
/*   err = pioneer_send_command1(PIONEER_RVEL, arg); */
/*   if (err < 0) */
/*     return -1; */
/*   return 0; */
}


int 
carmen_base_direct_update_status(double* update_timestamp) 
{
  int read = 0;
  double packet_timestamp=0.;
  pioneer_send_command0(PIONEER_PULSE);
  memset(&raw_state, 0, sizeof(struct pioneer_raw_information_packet));
  raw_state.motor_status = 0;
  do 
    {
      read = pioneer_read_string((unsigned char*) &raw_state, 
				 PIONEER_SERIAL_TIMEOUT, &packet_timestamp);
    }
  while (carmen_serial_numChars(dev_fd) > 0);

  if (raw_state.motor_status != PIONEER_MOTOR_NO_POWER &&
      raw_state.motor_status != PIONEER_MOTOR_STOPPED &&
      raw_state.motor_status != PIONEER_MOTOR_MOVING) 
    {
      carmen_warn("\nTried to convert non-information packet.\n");
      return -1;
    }

  if (update_timestamp){
    *update_timestamp=packet_timestamp;
  }
  return 0;
  //  return read;
}

int
carmen_base_direct_get_state(double *displacement, double *rotation,
			     double *tv, double *rv)
{
  double delta_x, delta_y, delta_theta, vl, vr;

  static int initialised = 0;

  static int    pioneer_prev_x, pioneer_prev_y;

  int           pioneer_x, pioneer_y;
  int           pioneer_delta_x, pioneer_delta_y;  

  static double prev_theta;
  double theta;

  if (raw_state.motor_status != PIONEER_MOTOR_NO_POWER &&
      raw_state.motor_status != PIONEER_MOTOR_STOPPED &&
      raw_state.motor_status != PIONEER_MOTOR_MOVING) 
    {
      carmen_warn("\nTried to convert non-information packet.\n");
      return -1;
    }

  pioneer_x = pioneer_buf_to_unsigned_int(raw_state.x);
  pioneer_y = pioneer_buf_to_unsigned_int(raw_state.y);

  theta =
    pioneer_buf_to_unsigned_int(raw_state.orientation) * angle_conv_factor;
  theta = carmen_normalize_theta(theta);

  if (initialised) 
    {      
      pioneer_delta_x = pioneer_x - pioneer_prev_x;
      pioneer_delta_y = pioneer_y - pioneer_prev_y;

      // x and y roll-over by 0x7fff
 
      if ( pioneer_delta_x > 0x3fff ) {
	pioneer_delta_x -= 0x7fff;
      } else if ( pioneer_delta_x < -0x3fff ) {
	pioneer_delta_x += 0x7fff;
      }

      if ( pioneer_delta_y > 0x3fff ) {
	pioneer_delta_y -= 0x7fff;
      } else if ( pioneer_delta_y < -0x3fff ) {
	pioneer_delta_y += 0x7fff;
      }

      delta_x = pioneer_delta_x * dist_conv_factor / 1000.0;
      delta_y = pioneer_delta_y * dist_conv_factor / 1000.0;
      delta_theta = theta - prev_theta;

      if (theta < prev_theta && prev_theta - theta > M_PI)
	delta_theta = (theta + M_PI) - (prev_theta - M_PI);
      else if (theta > prev_theta && theta - prev_theta > M_PI)
	delta_theta = (theta - M_PI) - (prev_theta + M_PI);


      if (displacement) {
	*displacement = hypot(delta_x, delta_y);
      }

      if (rotation)
	*rotation = delta_theta;
    }
  else 
    {
      initialised = 1;
    }

  pioneer_prev_x     = pioneer_x;
  pioneer_prev_y     = pioneer_y;
  prev_theta         = theta;

  vl = pioneer_buf_to_int(raw_state.vl) * vel_conv_factor / 1000;
  vr = pioneer_buf_to_int(raw_state.vr) * vel_conv_factor / 1000;

  // We do this because the stupid ActivMedia controller returns non-zero
  // values for wheel velocity when the robot is not moving.

  if (fabs(vl) < .05)
    vl = 0;
  if (fabs(vr) < .05)
    vr = 0;

  if (tv)
    *tv = (vl+vr)/2;

  if (*tv<0.0 && *displacement )
    *displacement *= -1;
  

/*   if (rv) */
/*     *rv = (vr-vl) * diff_conv_factor; */

  /** Cyrill, 13.Jan.2004
      ActivMedia uses this strange diff_conv_factor to compute the 
      rotational velocity from the individual wheel speeds.

      Mathematically it should be:
      omega = (vRightWheel - vLeftWheel) / DistanceBetweenRightAndLeftWheel

      That means ActivMedia's diff_conv_factor is computed by
      diff_conv_factor = 0.002/DistanceBetweenRightAndLeftWheelInM

      If we want to use ActivMedia's parameter, we must compute:
      omega = (vRightWheel - vLeftWheel) * 500.0 * diff_conv_factor;
  */
  if (rv)
    *rv = (vr-vl) * 500.0 * diff_conv_factor;
  
  return 0;
}

int
carmen_base_direct_get_integrated_state(double *x, double *y, double *theta,
					double *tv, double *rv)
{
  double vl, vr;

  if (raw_state.motor_status != PIONEER_MOTOR_NO_POWER &&
      raw_state.motor_status != PIONEER_MOTOR_STOPPED &&
      raw_state.motor_status != PIONEER_MOTOR_MOVING) 
    {
      carmen_warn("\nTried to convert non-information packet.\n");
      return -1;
    }

  if (x)
    *x = pioneer_buf_to_unsigned_int(raw_state.x) * dist_conv_factor/1000;
  if (y)
    *y = pioneer_buf_to_unsigned_int(raw_state.y) * dist_conv_factor/1000;
  if (theta)
    {
      *theta = pioneer_buf_to_int(raw_state.orientation) * 
	angle_conv_factor;
      *theta = carmen_normalize_theta(*theta);
    }

  vl = pioneer_buf_to_int(raw_state.vl) * vel_conv_factor / 1000;
  vr = pioneer_buf_to_int(raw_state.vr) * vel_conv_factor / 1000;

  // We do this because the stupid ActivMedia controller returns non-zero
  // values for wheel velocity when the robot is not moving.

  if (fabs(vl) < .05)
    vl = 0;
  if (fabs(vr) < .05)
    vr = 0;

  if (tv)
    *tv = (vl+vr)/2;
  if (rv)
    *rv = (vr-vl) * diff_conv_factor;

  return 0;
}

int 
carmen_base_direct_send_binary_data(unsigned char *data, int size)
{
  return pioneer_send_string(data, size);
}

int 
carmen_base_direct_get_binary_data(unsigned char **data, int *size)
{
  if (data == NULL || size == NULL)
    return 0;

  (*data) = calloc(1, sizeof(struct pioneer_raw_information_packet));  
  carmen_test_alloc(*data);
  *size = sizeof(struct pioneer_raw_information_packet);
  memcpy(*data, &raw_state, *size);
  return 0;
}

int carmen_base_direct_get_bumpers(unsigned char *state, int num_bumpers)
{
  int i;
  if (state == NULL || num_bumpers == 0)
    return PIONEER_MAX_NUM_BUMPERS;
  for (i=0; i<7 && i<num_bumpers; i++){
    state[i]=((raw_state.bumper[1])&(1<<(i+1)))!=0?1:0;
  }
  int j;
  for (j=0; j<7 && i<num_bumpers; j++){
    state[i++]=((raw_state.bumper[0])&(1<<(j+1))) != 0 ? 1 : 0;
  }
  return PIONEER_MAX_NUM_BUMPERS;
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
  double range;

  if (ranges != NULL && sonar_is_on) {
    //fprintf(stderr, "sonars %d, readings %d:", num_sonars, raw_state.num_sonar_readings);
    for (i = 0; i < num_sonars; i++){
      ranges[i]=-1.;
    }
    for (i = 0; i < raw_state.num_sonar_readings; i++) {
      range = pioneer_buf_to_unsigned_int(raw_state.sonar[i].range) *
	range_conv_factor / 1000.0;
      if (raw_state.sonar[i].n < num_sonars)
        ranges[raw_state.sonar[i].n] = range;
    } 
/*     for (i = 0; i < num_sonars; i++){ */
/*       fprintf(stderr, "%f " ,ranges[i]); */
/*     } */
/*     fprintf(stderr, "\n"); */
 }

  return 0;
}
