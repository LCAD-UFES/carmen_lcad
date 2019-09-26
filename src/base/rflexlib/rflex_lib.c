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
#include <netinet/in.h>
#include <carmen/drive_low_level.h>
#include "rflex.h"
#include "rflex-io.h"
#include "rflex_params.h"

typedef struct {
  int current_displacement_odometry;
  int current_bearing_odometry;

  int last_displacement_odometry;
  int last_bearing_odometry;

  int t_vel;
  int r_vel;
  int num_sonars;
  int ranges[MAX_NUM_SONARS];
  int num_bumpers;
  char *bumpers;
} rflex_status_t;

static rflex_status_t status;
static int dev_fd;
static int sonar_is_on = 0;

static double acceleration_state = 0;
static double deceleration_state = 0;

static int rflex_model = 0;

static double angle_conversion;
static double distance_conversion;

static carmen_inline int
sgn(long val)
{
  if (val < 0) 
    return(0);
  else 
    return(1);
}

/* COMPUTE CRC CODE */

static int
computeCRC( unsigned char *buf, int nChars )
{ 
  int i, crc;
  if (nChars==0) {
    crc = 0;
  } else {
    crc = buf[0];
    for (i=1; i<nChars; i++) {
      crc ^= buf[i];
    }
  }
  return(crc);
}

/* CONVERSION BYTES -> NUM */

static unsigned int
convertBytes2UInt16( unsigned char *bytes )
{
  unsigned int i;
  memcpy( &i, bytes, 2 );
  return(htons(i));
}


static unsigned long
convertBytes2UInt32( unsigned char *bytes )
{
  unsigned long i;
  memcpy( &i, bytes, 4 );
  return(htonl(i));
}

/* CONVERSION NUM -> BYTES */

static void
convertUInt8( unsigned int i, unsigned char *bytes )
{
  memcpy( bytes, &i, 1 );
}

static void
convertUInt32( unsigned long l, unsigned char *bytes )
{
  uint32_t conv;
  conv = htonl( l );
  memcpy( bytes, &conv, 4 );
}

static void
cmdSend( int port, int id, int opcode, int len, unsigned char *data )
{
  int i;
  static unsigned char cmd[MAX_COMMAND_LENGTH];
  /* START CODE */
  cmd[0] = 0x1b;
  cmd[1] = 0x02;
  /* PORT */
  cmd[2] = (unsigned char) port;
  /* ID */
  cmd[3] = (unsigned char) id;
  /* OPCODE */
  cmd[4] = (unsigned char) opcode;
  /* LENGTH */
  cmd[5] = (unsigned char) len;
  /* DATA */
  for (i=0; i<len; i++) {
    cmd[6+i] = data[i];
  }
  /* CRC */
  cmd[6+len] = computeCRC( &(cmd[2]), len+4 );    /* END CODE */
  cmd[6+len+1] = 0x1b;
  cmd[6+len+2] = 0x03;

  writeData( dev_fd, cmd, 9+len );
}

static void
parseMotReport( unsigned char *buffer )
{
  int rv, timeStamp, acc, trq;
  unsigned char axis, opcode;
    
  opcode = buffer[4];
  switch(opcode) {
  case MOT_SYSTEM_REPORT:
    rv        = convertBytes2UInt32(&(buffer[6]));
    timeStamp = convertBytes2UInt32(&(buffer[10]));
    axis      = buffer[14];
    if (axis == 0) {
      status.current_displacement_odometry=convertBytes2UInt32(&(buffer[15]));
      status.t_vel = convertBytes2UInt32(&(buffer[19]));
    } else if (axis == 1) {
      status.current_bearing_odometry = convertBytes2UInt32(&(buffer[15]));
      status.r_vel = convertBytes2UInt32(&(buffer[19]));
    }
    acc       = convertBytes2UInt32(&(buffer[23]));
    trq       = convertBytes2UInt32(&(buffer[27]));
    break;
  default:
    break;
  }
}

static void
parseSonarReport( unsigned char *buffer )
{
  unsigned int sid;
 
  int count, retval, timeStamp;
  unsigned char opcode, dlen;
    
  opcode = buffer[4];
  dlen   = buffer[5];

  status.num_sonars=MAX_NUM_SONARS;
  switch(opcode) {
  case SONAR_REPORT:
    retval    = convertBytes2UInt32(&(buffer[6]));
    timeStamp = convertBytes2UInt32(&(buffer[10]));
    count = 0;
    while ((8+count*3<dlen) && (count<256)) {
      sid   = buffer[14+count*3];
      status.ranges[sid] = convertBytes2UInt16( &(buffer[14+count*3]) );
      count++;
    }
    //	    fprintf( stderr, "(s:%d)", count );
    break;
  default:
    break;
  }
}

static int
parseBuffer( unsigned char *buffer, unsigned int len )
{
  unsigned int port, dlen, crc;

  port   = buffer[2];
  dlen   = buffer[5];

  if (dlen+8>len) {
    return(0);
  } else {
    crc    = computeCRC( &(buffer[2]), dlen+4 );
    if (crc != buffer[len-3])
      return(0);
    switch(port) {
    case SYS_PORT:
      fprintf( stderr, "(sys)" );
      break;
    case MOT_PORT:
      parseMotReport( buffer );
      break;
    case JSTK_PORT:
      break;
    case SONAR_PORT:
      parseSonarReport( buffer );
      break;
    case DIO_PORT:
      fprintf( stderr, "(dio)" );
      break;
    case IR_PORT:
      fprintf( stderr, "(ir)" );
      break;
    default:
      break;
    }
  }
  return(1);
}

static void
clear_incoming_data(void)
{
  unsigned char buffer[4096];
  int len;
  int bytes;

  // 32 bytes here because the motion packet is 34. No sense in waiting for a
  // complete packet -- we can starve because the last 2 bytes might always
  // arrive with the next packet also, and we never leave the loop. 
  
  bytes = bytesWaiting(dev_fd);
  while (bytes > 32) {
    waitForAnswer(dev_fd, buffer, &len);
    parseBuffer(buffer, len);
    bytes = bytesWaiting(dev_fd); 
  }
}

void
carmen_rflex_digital_io_on(int period)
{
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) period, &(data[0]) );
  cmdSend(DIO_PORT, 0, DIO_REPORTS_REQ, 4, data);
}

void
carmen_rflex_digital_io_off(void)
{
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 0, &(data[0]) );
  cmdSend(SONAR_PORT, 4, DIO_REPORTS_REQ, 4, data );
}

void
carmen_rflex_brake_on(void)
{
  cmdSend(MOT_PORT, 0, MOT_BRAKE_SET, 0, NULL);
}

void
carmen_rflex_brake_off(void)
{
  cmdSend(MOT_PORT, 0, MOT_BRAKE_RELEASE, 0, NULL);
}

void
carmen_rflex_motion_set_defaults(void)
{
  cmdSend(MOT_PORT, 0, MOT_SET_DEFAULTS, 0, NULL);
}

void
carmen_rflex_odometry_on(long period)
{ 
  unsigned char data[MAX_COMMAND_LENGTH];
  int num_read;
  
  convertUInt32( period, &(data[0]) );         /* period in ms */
  convertUInt32( (long) 3, &(data[4]) );       /* mask */
  cmdSend(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data);

  num_read = read(dev_fd, data, MAX_COMMAND_LENGTH);   
}

void
carmen_rflex_odometry_off(void)
{ 
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 0, &(data[0]) );       /* period in ms */
  convertUInt32( (long) 0, &(data[4]) );       /* mask */
  cmdSend(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data);
}

void
carmen_rflex_stop_robot(void)
{
  carmen_base_direct_set_velocity(0, 0);
}

void
carmen_rflex_update_bumpers(int num_bumpers, char *values)
{
  clear_incoming_data();

  if (num_bumpers > status.num_bumpers) 
    {
      carmen_warn("Requested more bumpers than available.\n");
      num_bumpers = status.num_bumpers;
    }

  memcpy(values, status.bumpers, num_bumpers*sizeof(char));
}

void 
carmen_rflex_initialize(int trans_acceleration,
			int rot_acceleration,
			int trans_pos __attribute__ ((unused)),
			int rot_pos __attribute__ ((unused)))
{
  unsigned char data[MAX_COMMAND_LENGTH];
  int i;

  // Translation 

  data[0] = 0;
  convertUInt32((long) 0, &(data[1]) );                 /* velocity */ 
  convertUInt32((long) trans_acceleration, &(data[5])); /* acceleration */ 
  convertUInt32((long) 0, &(data[9]) );                 /* torque */ 
  data[13] = 0;

  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data);

  // Rotation

  data[0] = 1;
  convertUInt32((long) 0, &(data[1]) );                  /* velocity */ 
  convertUInt32((long) rot_acceleration, &(data[5]) );   /* acceleration */ 
  convertUInt32((long) 0, &(data[9]) );                  /* torque */ 
  data[13] = 0;

  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data);

  //mark all non-existant (or no-data) sonar as such
  //note - this varies from MAX_INT set when sonar fail to get a valid echo.

  for (i=0; i < MAX_NUM_SONARS; i++)
     status.ranges[i] = -1;
}

int carmen_base_direct_sonar_on(void)
{
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 30000, &(data[0]) );
  convertUInt32( (long) 0, &(data[4]) );
  convertUInt32( (long) 0, &(data[8]) );
  convertUInt8(  (int) 2, &(data[12]) );
  cmdSend(SONAR_PORT, 4, SONAR_RUN, 13, data);

  sonar_is_on = 1;

  return 0;
}

int carmen_base_direct_sonar_off(void)
{
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 0, &(data[0]) );
  convertUInt32( (long) 0, &(data[4]) );
  convertUInt32( (long) 0, &(data[8]) );
  convertUInt8(  (int) 0, &(data[12]) );
  cmdSend(SONAR_PORT, 4, SONAR_RUN, 13, data);

  sonar_is_on = 0;
  return 0;
}

int
carmen_base_direct_initialize_robot(char *model, char *dev)
{
  Device rdev;
  int i;

  rflex_model = -1;
  for (i = 0; carmen_rflex_models[i] != 0; i++) 
    {
      if (strlen(model) == strlen(carmen_rflex_models[i]) &&
	  carmen_strncasecmp(model, carmen_rflex_models[i], 
			     strlen(carmen_rflex_models[i])) == 0)
	{
	  rflex_model = i;
	  break;
	}
  }

  if (rflex_model == -1) 
    {
      carmen_warn("Unknown rflex model %s\nKnown models: ", model);
      for (i = 0; carmen_rflex_models[i] != 0; i++) 
	{
	  carmen_warn("%s ", carmen_rflex_models[i]);
	  if (i % 6 == 5)
	    carmen_warn("\n");
	}
      carmen_warn("To determine your rflex model, read \n"
	      "carmen/src/rflex/README\n");
      return -1;
    }

  distance_conversion = carmen_rflex_params[rflex_model][0];
  angle_conversion = carmen_rflex_params[rflex_model][1];  

  strncpy(rdev.ttyport, dev, MAX_NAME_LENGTH);
  rdev.baud           = 115200;
  rdev.databits       = 8;
  rdev.parity         = N;
  rdev.stopbits       = 1;
  rdev.hwf            = 0;
  rdev.swf            = 0;
  
  if (DEVICE_connect_port(&rdev) < 0) {
    carmen_warn("Can't open device %s\n", rdev.ttyport);
    return -1;
  } 

  dev_fd = rdev.fd;

  carmen_rflex_odometry_on(100000);
  carmen_rflex_digital_io_on(100000);
  carmen_rflex_motion_set_defaults();

  carmen_rflex_brake_off();

  carmen_rflex_initialize(1.0*distance_conversion, 
			  M_PI/4 * angle_conversion, 0, 0);

  return 0;
}

int carmen_base_direct_reset(void)
{
  unsigned char data[MAX_COMMAND_LENGTH];

  convertUInt8((long) 0, &(data[0]));                   /* forward motion */
  convertUInt32((long) 0, &(data[1]));                  /* trans velocity */
  convertUInt32((long) acceleration_state, &(data[5])); /* trans acc */
  convertUInt32((long) STD_TRANS_TORQUE, &(data[9]));   /* trans torque */
  convertUInt32((long) 0, &(data[13]));                 /* trans position */

  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_POS, 17, data);

  convertUInt8((long) 1, &(data[0]));                  /* rotational motion */
  convertUInt32((long) 0, &(data[1]));                 /* rot velocity  */
  convertUInt32((long) STD_ROT_ACC, &(data[5]));       /* rot acc */
  convertUInt32((long) STD_ROT_TORQUE, &(data[9]));    /* rot torque */
  convertUInt32((long) 0, &(data[13]));                /* rot position */

  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_POS, 17, data);

  return 0;
}

int 
carmen_base_direct_shutdown_robot(void)
{
  if (sonar_is_on)
    carmen_base_direct_sonar_off();

  carmen_rflex_odometry_off();
  carmen_rflex_digital_io_off();
  carmen_rflex_brake_on();
  carmen_rflex_motion_set_defaults();

  return 0;
}

int 
carmen_base_direct_set_acceleration(double acceleration)
{
  acceleration_state = acceleration * distance_conversion;

  return 0;
}

int 
carmen_base_direct_set_deceleration(double deceleration)
{
  deceleration_state = deceleration * distance_conversion;

  return 0;
}

int 
carmen_base_direct_set_velocity(double tv, double rv)
{
  unsigned char data[MAX_COMMAND_LENGTH];

  tv = tv * distance_conversion;
  rv = rv * angle_conversion;
  
  convertUInt8( (long) 0, &(data[0]));                   /* forward motion */
  convertUInt32( (long) abs(tv), &(data[1]));        /* abs trans velocity */

  if ((tv > 0 && tv < status.t_vel) || (tv < 0 && tv > status.t_vel))
    convertUInt32( (long) deceleration_state, &(data[5])); /* trans acc */
  else
    convertUInt32( (long) acceleration_state, &(data[5])); /* trans acc */

  convertUInt32( (long) STD_TRANS_TORQUE, &(data[9]));   /* trans torque */
  convertUInt8((long) sgn(tv), &(data[13]));           /* trans direction */

  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data);

  convertUInt8((long) 1, &(data[0]));                  /* rotational motion */
  convertUInt32((long) abs(rv), &(data[1]));         /* abs rot velocity  */
  /* 0.275 rad/sec * 10000 */ 
  convertUInt32((long) STD_ROT_ACC, &(data[5]));       /* rot acc */
  convertUInt32((long) STD_ROT_TORQUE, &(data[9]));    /* rot torque */
  convertUInt8((long) sgn(rv), &(data[13]));         /* rot direction */

  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data);

  return 0;
}

int 
carmen_base_direct_update_status(double* packet_timestamp __attribute__ ((unused)))
{
  clear_incoming_data();

  return 0;
}

int 
carmen_base_direct_get_state(double *displacement, double *rotation,
			     double *tv, double *rv)
{
  if (displacement)
    *displacement = (status.current_displacement_odometry - 
		     status.last_displacement_odometry) / distance_conversion;
  
  if (rotation)
    *rotation = (status.current_bearing_odometry - 
		 status.last_bearing_odometry) / angle_conversion;
  if (tv)
    *tv = status.t_vel / distance_conversion;
  if (rv)
    *rv = status.r_vel / angle_conversion;

  status.last_displacement_odometry = status.current_displacement_odometry;
  status.last_bearing_odometry = status.current_bearing_odometry;

  return 0;
}

int 
carmen_base_direct_get_integrated_state(double *x __attribute__ ((unused)), 
					double *y __attribute__ ((unused)), 
					double *theta __attribute__ ((unused)),
					double *tv __attribute__ ((unused)), 
					double *rv __attribute__ ((unused)))
{
  carmen_warn("Hardware integration not supported by RFlex.\n");

  return 0;
}

int 
carmen_base_direct_send_binary_data(unsigned char *data 
				    __attribute__ ((unused)), 
				    int size __attribute__ ((unused)))
{

  return 0;
}

int 
carmen_base_direct_get_binary_data(unsigned char **data
				   __attribute__ ((unused)), 
				   int *size __attribute__ ((unused)))
{

  return 0;
}

int carmen_base_direct_get_bumpers(unsigned char *state
				   __attribute__ ((unused)), 
				   int num_bumpers __attribute__ ((unused)))
{

  return 0;
}

void carmen_base_direct_arm_get(double servos[] __attribute__ ((unused)), 
				int num_servos __attribute__ ((unused)), 
				double *currents __attribute__ ((unused)), 
				int *gripper __attribute__ ((unused)))
{
}

void carmen_base_direct_arm_set(double servos[] __attribute__ ((unused)), 
				int num_servos __attribute__ ((unused)))
{
}

int carmen_base_direct_get_sonars(double *ranges, 
				  carmen_point_t *positions 
				  __attribute__ ((unused)),
				  int num_sonars)
{
  int i;

  for(i=0; i < num_sonars && i < status.num_sonars; i++) {
    if (status.ranges[i] < 0)
      continue;
    else if(status.ranges[i]==32766)
      ranges[i]=0;
    else
      ranges[i]=status.ranges[i];
  }
  
  if ( i < num_sonars)
    carmen_warn("Requested more sonars (%d) than available (%d).\n",
		num_sonars, i);      
  
  return 0;
}
