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
#include "arm_low_level.h"
#include <carmen/serial.h>
#include <sys/ioctl.h>
#include <limits.h>

#define ORC_MASTER 0
#define ORC_SLAVE 1
#define ORC_PAD 2

#define ORC_STATUS 0x2A

#define ORC_LEFT_SONAR_PING 4
#define ORC_RIGHT_SONAR_PING 6

#define ORC_LEFT_SONAR_ECHO 49
#define ORC_RIGHT_SONAR_ECHO 51

#define ORC_SONAR_PING 6
#define ORC_SONAR_ECHO 7

#define ORC_LEFT_MOTOR 0
#define ORC_RIGHT_MOTOR 2

#define ORC_LEFT_MOTOR_ACTUAL_PWM 14
#define ORC_RIGHT_MOTOR_ACTUAL_PWM 19

#define ORC_LEFT_PINMODE 4
#define ORC_RIGHT_PINMODE 5

#define ORC_LEFT_MOTOR_QUAD_PORT 16
#define ORC_RIGHT_MOTOR_QUAD_PORT 18

#define ORC_QUAD_PHASE_FAST 14

#define ORC_LEFT_MOTOR_DIR 25
#define ORC_RIGHT_MOTOR_DIR 26
#define ORC_FORWARD 1
#define ORC_BACKWARD 2

#define ORC_MAX_ANGULAR_VEL 8.0 // Radians / seconds

#define ORC_MAX_PWM 255
#define ORC_FF_GAIN ((ORC_MAX_PWM / ORC_MAX_ANGULAR_VEL) * 0.9)
#define ORC_P_GAIN 8
#define ORC_I_GAIN 3
#define ORC_D_GAIN 15

#define ORC_VEL_ACCEL_TEMP 0.9
#define ORC_VEL_DECEL_TEMP 0.4

#define ORC_LEFT_MOTOR_ENCODER 7
#define ORC_RIGHT_MOTOR_ENCODER 10

#define ORC_MASTER_TIME 4
#define ORC_SLAVE_TIME 48

#define ORC_WHEEL_DIAMETER .125
#define ORC_WHEEL_BASE .43
#define ORC_ENCODER_RESOLUTION 500
#define ORC_GEAR_RATIO 65.5

#define ORC_DIGITAL_IN_PULL_UP 1
#define ORC_DIGITAL_IN 6

#define ORC_SERVO_CURRENT 35
#define ORC_SERVO_PWM_STATE 8
#define ORC_SERVO_PIN 5

// velezj: For RssII Arm
#define ORC_SHOULDER_MOTOR 1
#define ORC_SHOULDER_MOTOR_ACTUAL_PWM 17
#define ORC_SHOULDER_MOTOR_DIR 25
#define ORC_SHOULDER_MOTOR_QUAD_PORT 16
#define ORC_SHOULDER_MOTOR_ENCODER 7
#define ORC_SHOULDER_MOTOR_DIRECTION_SIGN 1
#define ORC_ELBOW_MOTOR 3
#define ORC_ELBOW_MOTOR_ACTUAL_PWM 23
#define ORC_ELBOW_MOTOR_DIR 26
#define ORC_ELBOW_MOTOR_QUAD_PORT 18
#define ORC_ELBOW_MOTOR_ENCODER 10
#define ORC_ELBOW_MOTOR_DIRECTION_SIGN -1
#define ORC_ARM_GEAR_REDUCTION ( 65.5 * 5.0 )
#define ORC_ARM_TICKS_PER_RADIAN ( ORC_ENCODER_RESOLUTION * ORC_ARM_GEAR_REDUCTION / ( 2.0 * M_PI ) )
#define ORC_ELBOW_MAX_PWM 40 // 27
#define ORC_ELBOW_MIN_PWM 35 // 22
#define ORC_SHOULDER_MAX_PWM 50 // 37
#define ORC_SHOULDER_MIN_PWM 42 // 30
#define ORC_ARM_P_GAIN 8
#define ORC_ARM_D_GAIN 15
#define ORC_ARM_I_GAIN 0 // 3
#define ORC_ARM_MAX_ANGULAR_VEL 0.0035 // Radians / second
#define ORC_ARM_MIN_ANGULAR_VEL 0.00125 // Radians / second 
//#define ORC_ARM_FF_GAIN ((ORC_ARM_MAX_PWM / ORC_ARM_MAX_ANGULAR_VEL) * 0.9)
#define ORC_ARM_FF_GAIN ( (90 / ORC_ARM_MAX_ANGULAR_VEL ) * 0.9 );
#define ORC_ARM_THETA_P_GAIN 0.008
#define ORC_ARM_THETA_D_GAIN 0.028 // 0.018
#define ORC_ARM_THETA_I_GAIN 0.000 // 0.003
#define ORC_ARM_GRIPPER_SERVO 0 // 0
#define ORC_ARM_GRIPPER_MIN_PWM 8600
#define ORC_ARM_GRIPPER_PWM_PER_RADIAN 2214.2 

static double acceleration;
static double deceleration;

static double x, y, theta;
static double displacement, rotation;
static double left_velocity, right_velocity;
static double left_iTerm = 0.0, right_iTerm = 0.0;
static double left_vel_ramp = 0.0, right_vel_ramp = 0.0;
static double left_error_prev = 0.0, right_error_prev = 0.0;
static double left_desired_velocity = 0, right_desired_velocity = 0;
static double left_range, right_range;
static int initialized = 0;
static int sonar_on = 1;

static int left_pwm = 0, right_pwm = 0;
static short last_master_ticks;
static short last_slave_ticks;
static int left_last_tick, right_last_tick;
static double time_since_last_command;
static double last_command_time;

static double servo_state[4];
static double servo_current[2];

static char bumpers[4];
static int gripper_state = 0;
static int serial_fd = -1;

// velezj: rssII Arm
static double gripper_theta = 0.0, gripper_desired_theta = 0.0;
static double shoulder_theta = 0.0, elbow_theta = 0.0;
static double shoulder_desired_theta = 0.0, elbow_desired_theta = 0.0;
static double shoulder_theta_iTerm = 0.0, elbow_theta_iTerm = 0.0;
static double shoulder_theta_error_prev = 0.0, elbow_theta_error_prev = 0.0;
static double shoulder_iTerm = 0.0, elbow_iTerm = 0.0;
static double shoulder_error_prev = 0.0, elbow_error_prev = 0.0;
static double shoulder_desired_angular_velocity = 0.0, elbow_desired_angular_velocity = 0.0;
static double shoulder_angular_velocity, elbow_angular_velocity;
static int shoulder_pwm, elbow_pwm;
static int shoulder_last_tick = 0, elbow_last_tick = 0;

static void recover_failure(void)
{
  printf("Recovering from Serial Failure\n");
  initialized = 0;
  if (serial_fd >= 0) {
    close(serial_fd);
    serial_fd = -1;
  }
  while (serial_fd < 0) {
    sleep(1);
    carmen_warn("Trying to reconnect to arm...\n");
    carmen_arm_direct_initialize(NULL,  NULL);
  }
}

unsigned char create_checksum(unsigned char *buffer, int size)
{
  unsigned char checksum = 0;
  int i;

  for (i = 0; i < size; i++)
    checksum = (checksum << 1) + buffer[i] + (checksum & 0x80 ? 1 : 0);

  return checksum;
}

void send_packet(unsigned char *byte, int length,  unsigned char where)
{
  static unsigned char *buffer;
  static unsigned char size;
  static int buffer_size = 0;
  int i;
  int num_written;
  
  static unsigned char count = 0;

  //  carmen_warn("Sent packet %x to %d : %d\n", byte[0], where, count);
  count++;
  if (count == 0x40)
    count = 0;

  if (buffer_size == 0) {
    buffer = (unsigned char *)calloc(sizeof(unsigned char), length+4);
    carmen_test_alloc(buffer);
    buffer_size = length+4;
  } else if (buffer_size < length+4) {
    buffer = (unsigned char *)realloc
      (buffer, sizeof(unsigned char)*(length+4));
    carmen_test_alloc(buffer);
    buffer_size = length+4;
  }

  size = length+4;

  buffer[0] = 0xED;
  buffer[1] = size;
  buffer[2] = (where << 6) | count; // was | 0xF;
  for (i = 0; i < length; i++)
    buffer[i+3] = byte[i];
  buffer[length+3] = create_checksum(buffer, length+3);

  num_written = carmen_serial_writen(serial_fd, buffer, length+4);
  if (num_written < 0)
    recover_failure();
}

static unsigned char *check_packet_length(int packet_length)
{
  unsigned char *buffer = (unsigned char *)calloc(sizeof(unsigned char), packet_length);
  carmen_test_alloc(buffer);

  return buffer;
}

static unsigned char *read_packet(void)
{
  unsigned char byte, routing;
  unsigned char *buffer = NULL;
  unsigned char *data_ptr;
  unsigned char checksum;
  fd_set rfds;

  int num_ready_chars;
  int packet_length;
  struct timeval timeout;
  int num_ready;

  int count = 0;
  int packet_count = 0;

  do {
    do {
      num_ready_chars = carmen_serial_numChars(serial_fd);
      
      if (num_ready_chars == -1) 
	return NULL;
      
      count = 0;
      while (count < 50 && num_ready_chars == 0) {
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;
	FD_ZERO(&rfds);
	FD_SET(0, &rfds);
        num_ready = select(1, &rfds, NULL, NULL, &timeout);
        num_ready_chars = carmen_serial_numChars(serial_fd);
        count++;
      }

      if (num_ready_chars < 1) {
	return NULL;
      }
      //      carmen_warn("Ready: %d\n", num_ready_chars);
      
      if (carmen_serial_readn(serial_fd, &byte, 1) < 0)
	return NULL;
    } while (byte != 0xED);
    
    if (carmen_serial_readn(serial_fd, &byte, 1) < 0) 
      return NULL;
    
    packet_length = byte;
    
    buffer = check_packet_length(packet_length);
    buffer[0] = 0xED;
    buffer[1] = byte;
    if (carmen_serial_readn(serial_fd, &byte, 1) < 0) 
      return NULL;
    
    buffer[2] = byte; 
    data_ptr = buffer+3;
    routing = byte >> 6;

    if (carmen_serial_readn(serial_fd, data_ptr, packet_length-3) < 0) 
      return NULL;
  
    checksum = create_checksum(buffer, packet_length-1);
    
    if (checksum != buffer[packet_length-1]) {
      carmen_warn("Corrupted data from serial line. "
		  "Dropping packet.\n");
      free(buffer);
      buffer = NULL;
    }
    
    if (routing == ORC_PAD) {
      //      carmen_warn("Tossing orc_pad packet\n");
      free(buffer);
      buffer = NULL;
    } 
    //    else
    //      carmen_warn("Got packet %x from %d : %d\n", buffer[3],
    //		  routing, byte & 0x3f);    
      
    packet_count++;
  } while (!buffer && packet_count < 3);

  //  carmen_warn("Returning buffer of size %d\n", packet_length);

  return buffer;
}

static int wait_for_ack(void)
{
  unsigned char *buffer;
  unsigned char response;
  
  buffer = read_packet();
  if (buffer == NULL)
    return -1;

  response = buffer[3];
  free(buffer);

  if (response != 0)
    return -1;

  return 0;
}

static int send_packet_and_ack(unsigned char *byte, int length,  
			       unsigned char where)
{
  int count;
  int err;

  count = 0;
  do {
    send_packet(byte, length, where);
    err = wait_for_ack();
    if (err == 0)
      return 0;
    count++;
  } while (count < 3);

  return -1;
}

static void command_velocity(double desired_velocity, double current_velocity,
           int current_pwm, unsigned char WHICH_MOTOR)
{
  double desired_angular_velocity, current_angular_velocity;
  double desired_velocity_ramp = 0;
  double ffTerm = 0.0, pTerm = 0.0, dTerm = 0.0, velError = 0.0;
  double * iTermPtr;
  double * velErrorPrevPtr;
  double * velRampPtr;
  unsigned char command_pwm;
  unsigned char dir;
  unsigned char buffer[4];
  char which;

  if (WHICH_MOTOR == ORC_LEFT_MOTOR) {
    which = 'L';
    iTermPtr = &left_iTerm;
    velRampPtr = &left_vel_ramp;
    velErrorPrevPtr = &left_error_prev;
  }
  else {
    which = 'R';
    iTermPtr = &right_iTerm;
    velRampPtr = &right_vel_ramp;
    velErrorPrevPtr = &right_error_prev;
  }

  if (desired_velocity == 0) {
    *velRampPtr = 0;
  }
  else if (desired_velocity > *velRampPtr) {
    (*velRampPtr) += acceleration * time_since_last_command;
    printf("accel %f %f %f\n", acceleration * time_since_last_command,
	   acceleration, time_since_last_command);
    //    (*velRampPtr) += ORC_VEL_ACCEL_TEMP * time_since_last_command;
    //    printf("accel %f\n", ORC_VEL_ACCEL_TEMP * time_since_last_command);
    if (*velRampPtr > desired_velocity) {
      *velRampPtr = desired_velocity;
    }
  }
  else if (desired_velocity < *velRampPtr) {
    (*velRampPtr) -= deceleration * time_since_last_command;
    printf("decel %f %f %f\n", deceleration * time_since_last_command,
    	   deceleration, time_since_last_command);
    //    (*velRampPtr) -= ORC_VEL_DECEL_TEMP * time_since_last_command;
    //    printf("decel %f\n", ORC_VEL_DECEL_TEMP * time_since_last_command);
    if (*velRampPtr < desired_velocity) {
      *velRampPtr = desired_velocity;
    }
  }

  desired_velocity_ramp = *velRampPtr;

  desired_angular_velocity = desired_velocity_ramp / (ORC_WHEEL_DIAMETER/2.0);
  current_angular_velocity = current_velocity / (ORC_WHEEL_DIAMETER/2.0);


  if (fabs(desired_angular_velocity) > .001) {
      /* Nick, what did you mean to do here?  I don't understand these
         comparisons.  Do you mean < 3 and &&? */
    if (desired_angular_velocity > ORC_MAX_ANGULAR_VEL)
      desired_angular_velocity = ORC_MAX_ANGULAR_VEL;
    if (desired_angular_velocity < -ORC_MAX_ANGULAR_VEL)
      desired_angular_velocity = -ORC_MAX_ANGULAR_VEL;

    velError = (desired_angular_velocity - current_angular_velocity);
    ffTerm = desired_angular_velocity * ORC_FF_GAIN;
    pTerm = velError * ORC_P_GAIN;
    *iTermPtr += velError * ORC_I_GAIN;
    dTerm = (velError - *velErrorPrevPtr) * ORC_D_GAIN;

    current_pwm = (int)(ffTerm + pTerm + *iTermPtr + dTerm);

    *velErrorPrevPtr = velError;

    if (abs(current_pwm) > ORC_MAX_PWM)
      current_pwm = (current_pwm > 0 ? ORC_MAX_PWM : -ORC_MAX_PWM);
    if (WHICH_MOTOR == ORC_LEFT_MOTOR)
      current_pwm = -current_pwm;

    if (current_pwm < 0) {
      command_pwm = -current_pwm;
      dir = ORC_BACKWARD;
    } else {
      command_pwm = current_pwm;
      dir = ORC_FORWARD;
    }
  } else {
    dir = ORC_FORWARD;
    command_pwm = 0;
    *iTermPtr = 0.0;
  }

  buffer[0] = 0x4D;
  buffer[1] = WHICH_MOTOR;
  buffer[2] = dir;
  buffer[3] = command_pwm;
  send_packet_and_ack(buffer, 4, ORC_SLAVE);
}


// velezj: rssII Arm
unsigned short gripper_theta_to_pwm( double theta ) {
  if( theta < 0 ) {
    carmen_warn( "gripper angles can ONLY be positive <= PI radians " );
    return 0;
  }
  
  return ORC_ARM_GRIPPER_MIN_PWM + theta * ORC_ARM_GRIPPER_PWM_PER_RADIAN;
}


// velezj: rssII Arm
double gripper_pwm_to_theta( unsigned short pwm ) {
  return (double)( pwm - ORC_ARM_GRIPPER_MIN_PWM ) / ORC_ARM_GRIPPER_PWM_PER_RADIAN;
}


// velezj: rssII Arm
void carmen_arm_reset() {
  shoulder_iTerm = 0.0;
  elbow_iTerm = 0.0;
}


// velezj: rssII Arm
void carmen_arm_direct_set( double shoulder_desired_angle, double elbow_desired_angle, double gripper_desired_angle ) {

  // reset the iterms of the vel first
  carmen_arm_reset();

  shoulder_desired_theta = shoulder_desired_angle;
  elbow_desired_theta = elbow_desired_angle;
  gripper_desired_theta = gripper_desired_angle;

  // send the gripper angle as a servo command to orc
  unsigned char buffer[4];
  unsigned short pwm = gripper_theta_to_pwm( gripper_desired_angle );
  buffer[0] = 0x53;
  buffer[1] = ORC_ARM_GRIPPER_SERVO;
  buffer[2] = pwm >> 8;
  buffer[3] = pwm & 0x00ff;
  send_packet_and_ack(buffer, 4, ORC_MASTER);
  
  printf( "\ncarmen_arm_direct_set: shoulder=%f  elbow=%f  gripper=%f \n", shoulder_desired_angle, elbow_desired_angle, gripper_desired_angle );
}


void carmen_arm_get_theta_state( double *shoulder_angle, double *elbow_angle, double *gripper_angle ) {
  *shoulder_angle = shoulder_theta;
  *elbow_angle = elbow_theta;
  *gripper_angle = gripper_theta;
}


void carmen_arm_get_state( double *shoulder_avel, double *elbow_avel ) {
  *shoulder_avel = shoulder_angular_velocity;
  *elbow_avel = elbow_angular_velocity;
}


// velezj: rssII Arm
static void carmen_arm_command_angular_velocity( double desired_angular_velocity, double current_angular_velocity, int current_pwm, unsigned char WHICH_MOTOR ) {
  
  double ffTerm = 0.0, pTerm = 0.0, dTerm = 0.0, velError = 0.0;
  double * iTermPtr;
  double * velErrorPrevPtr;;
  unsigned char command_pwm;
  unsigned char dir;
  unsigned char buffer[4];
  char which;
  int motor_direction_sign = 1;
  int max_pwm, min_pwm;

  if (WHICH_MOTOR == ORC_SHOULDER_MOTOR) {
    which = 'S';
    iTermPtr = &shoulder_iTerm;
    velErrorPrevPtr = &shoulder_error_prev;
    motor_direction_sign = ORC_SHOULDER_MOTOR_DIRECTION_SIGN;
    max_pwm = ORC_SHOULDER_MAX_PWM;
    min_pwm = ORC_SHOULDER_MIN_PWM;
  }
  else {
    which = 'E';
    iTermPtr = &elbow_iTerm;
    velErrorPrevPtr = &elbow_error_prev;
    motor_direction_sign = ORC_ELBOW_MOTOR_DIRECTION_SIGN;
    max_pwm = ORC_ELBOW_MAX_PWM;
    min_pwm = ORC_ELBOW_MIN_PWM;
  }

  if (fabs(desired_angular_velocity) > .0005) {
    if (desired_angular_velocity > ORC_ARM_MAX_ANGULAR_VEL)
      desired_angular_velocity = ORC_ARM_MAX_ANGULAR_VEL;
    if (desired_angular_velocity < -ORC_ARM_MAX_ANGULAR_VEL)
      desired_angular_velocity = -ORC_ARM_MAX_ANGULAR_VEL;

    velError = (desired_angular_velocity - current_angular_velocity);
    ffTerm = desired_angular_velocity * ORC_ARM_FF_GAIN;
    pTerm = velError * ORC_ARM_P_GAIN;
    *iTermPtr += velError * ORC_ARM_I_GAIN;
    dTerm = (velError - *velErrorPrevPtr) * ORC_ARM_D_GAIN;

    current_pwm = (int)(ffTerm + pTerm + *iTermPtr + dTerm);

    *velErrorPrevPtr = velError;

    if (abs(current_pwm) > max_pwm)
      current_pwm = (current_pwm > 0 ? max_pwm : -max_pwm);
    if( abs( current_pwm) < min_pwm )
      current_pwm = ( current_pwm > 0 ? min_pwm : -min_pwm );
    if (WHICH_MOTOR == ORC_SHOULDER_MOTOR)
      current_pwm = -current_pwm;

    current_pwm *= motor_direction_sign;

    if (current_pwm < 0) {
      command_pwm = -current_pwm;
      dir = ORC_BACKWARD;
    } else {
      command_pwm = current_pwm;
      dir = ORC_FORWARD;
    }
  } else {
    dir = ORC_FORWARD;
    command_pwm = 0;
    *iTermPtr = 0.0;
  }

  // debug
  if( command_pwm != 0 ) {
    printf( "[ %c ] p: %f  i: %f  d: %f   ve: %f\n", which, pTerm, *iTermPtr, dTerm, velError );  
    printf( "[ %c ] sending PWM: %d\n", which, ( dir == ORC_BACKWARD ? -command_pwm : command_pwm ) );
  }

  buffer[0] = 0x4D;
  buffer[1] = WHICH_MOTOR;
  buffer[2] = dir;
  buffer[3] = command_pwm;
  send_packet_and_ack(buffer, 4, ORC_SLAVE);
}


// velezj: rssII Arm
void bound_value( double *vPtr, double min, double max ) {
  if( *vPtr < min )
    *vPtr = min;
  if( *vPtr > max )
    *vPtr = max;
}


// velezj: rssII Arm
double sign( double v ) {
  if( v < 0.0 )
    return -1.0;
  return 1.0;
}


// velezj: rssII Arm
void carmen_arm_control() {

  double shoulder_theta_delta = shoulder_desired_theta - shoulder_theta;
  double elbow_theta_delta = elbow_desired_theta - elbow_theta;
  
  double pTerm = 0.0, dTerm = 0.0;
  double *iTermPtr;

  // shoulder
  iTermPtr = &shoulder_theta_iTerm;
  //printf( "control: shoulder diff: %f    ( %f  -  %f )\n", shoulder_theta_delta, shoulder_desired_theta, shoulder_theta );
  if( fabs( shoulder_theta_delta ) > 0.01 ) {
    pTerm = shoulder_theta_delta * ORC_ARM_THETA_P_GAIN;
    *iTermPtr += ( shoulder_theta_delta * ORC_ARM_THETA_I_GAIN );
    dTerm = ( shoulder_theta_delta - shoulder_theta_error_prev ) * ORC_ARM_THETA_D_GAIN;
    shoulder_desired_angular_velocity = ( pTerm + *iTermPtr * dTerm );
    bound_value( &shoulder_desired_angular_velocity, -ORC_ARM_MAX_ANGULAR_VEL, ORC_ARM_MAX_ANGULAR_VEL );
    if( fabs( shoulder_desired_angular_velocity ) < ORC_ARM_MIN_ANGULAR_VEL ) {
      shoulder_desired_angular_velocity = sign( shoulder_desired_angular_velocity ) * ORC_ARM_MIN_ANGULAR_VEL;
    }

    printf( "contorl: shoulder: p: %f   i: %f   d: %f    av: %f\n", pTerm, *iTermPtr, dTerm, shoulder_desired_angular_velocity );
  } else {
    shoulder_desired_angular_velocity = 0.0;
    *iTermPtr = 0.0;
    //printf( "control: shoulder: done!\n" );
  }

  //elbow
  iTermPtr = &elbow_theta_iTerm;
  //printf( "control: elbow diff: %f    ( %f  -  %f )\n", elbow_theta_delta, elbow_desired_theta, elbow_theta );
  if( fabs( elbow_theta_delta ) > 0.01 ) {
    pTerm = elbow_theta_delta * ORC_ARM_THETA_P_GAIN;
    *iTermPtr += ( elbow_theta_delta * ORC_ARM_THETA_I_GAIN );
    dTerm = ( elbow_theta_delta - elbow_theta_error_prev ) * ORC_ARM_THETA_D_GAIN;
    elbow_desired_angular_velocity = ( pTerm + *iTermPtr * dTerm );
    bound_value( &elbow_desired_angular_velocity, -ORC_ARM_MAX_ANGULAR_VEL, ORC_ARM_MAX_ANGULAR_VEL );
    if( fabs( elbow_desired_angular_velocity ) < ORC_ARM_MIN_ANGULAR_VEL ) {
      elbow_desired_angular_velocity = sign( elbow_desired_angular_velocity ) * ORC_ARM_MIN_ANGULAR_VEL;
    }

    printf( "contorl: elbow: p: %f   i: %f   d: %f    av: %f\n", pTerm, *iTermPtr, dTerm, elbow_desired_angular_velocity );
  } else {
    elbow_desired_angular_velocity = 0.0;
    *iTermPtr = 0.0;
    //printf( "control: elbow: done!\n" );
  }

  carmen_arm_command_angular_velocity( shoulder_desired_angular_velocity, shoulder_angular_velocity, shoulder_pwm, ORC_SHOULDER_MOTOR );
  carmen_arm_command_angular_velocity( elbow_desired_angular_velocity, elbow_angular_velocity, elbow_pwm, ORC_ELBOW_MOTOR );
  //command_velocity( shoulder_desired_angular_velocity * ( ORC_WHEEL_DIAMETER / 2.0 ), shoulder_angular_velocity * ( ORC_WHEEL_DIAMETER / 2.0 ), shoulder_pwm, ORC_LEFT_MOTOR );
} 


static double delta_tick_to_metres(int delta_tick)
{
  double revolutions = (double)delta_tick/
    (double)(ORC_ENCODER_RESOLUTION*ORC_GEAR_RATIO);
  double radians = revolutions*2*M_PI;
  double metres = radians*(ORC_WHEEL_DIAMETER/2.0);

  return metres;
}


// velezj: for RssII Arm
static double delta_ticks_to_angle( int delta_ticks ) 
{
  double radians = (double)delta_ticks / (double)ORC_ARM_TICKS_PER_RADIAN;
  return radians;
}


static double voltage_to_current(unsigned short voltage)
{
  double current;
  current = voltage*5.0/65536.0;
  // V=IR, R=0.18 ohm
  current = current/0.18;
  return current;
}

static int unpack_short (unsigned char *buffer, int offset)
{
  return ((buffer[offset]&0x00ff)<<8)+(buffer[offset+1]&0x00ff);
}

static void unpack_master_packet(unsigned char *buffer)
{
  int range;
  short time_ticks, delta_master_ticks;
  char command_buffer[2];
  short bumper_state;
  unsigned short servo_pwm;
  int i;
  static double last_ping_time = 0;
  static int last_ping = ORC_RIGHT_SONAR_PING;

  //carmen_warn("Got master packet\n");
 //printf("unpack_master_packet\n");

  range = unpack_short(buffer, ORC_LEFT_SONAR_ECHO);
  if (range == 0xffff)
    left_range = 0;
  else
    left_range = range/1000000.0*331.46/2.0;

  range = unpack_short(buffer, ORC_RIGHT_SONAR_ECHO);
  if (range == 0xffff)
    right_range = 0;
  else
    right_range = range/1000000.0*331.46/2.0;

  time_ticks = buffer[ORC_MASTER_TIME];
  delta_master_ticks = time_ticks - last_master_ticks;
  if (delta_master_ticks > SHRT_MAX/2)
    delta_master_ticks -= 2*SHRT_MAX;
  if (delta_master_ticks < -SHRT_MAX/2)
    delta_master_ticks += 2*SHRT_MAX;

  last_master_ticks = time_ticks;

  if (sonar_on && carmen_get_time() - last_ping_time > .05) {
    command_buffer[0] = 'R';
    if (last_ping == ORC_LEFT_SONAR_PING)
      command_buffer[1] = ORC_RIGHT_SONAR_PING;
    else
      command_buffer[1] = ORC_LEFT_SONAR_PING;
    send_packet_and_ack(command_buffer, 2, ORC_MASTER);
    last_ping = command_buffer[1];
    last_ping_time = carmen_get_time();
  }

  bumper_state = unpack_short(buffer, ORC_DIGITAL_IN);

  bumpers[0] = bumper_state >> 8 & 1;
  bumpers[1] = bumper_state >> 9 & 1;
  bumpers[2] = bumper_state >> 10 & 1;
  bumpers[3] = bumper_state >> 11 & 1;
  gripper_state = bumper_state >> 12 & 1;

 
  for (i = 0; i < 4; i++) {
    servo_pwm = unpack_short(buffer, ORC_SERVO_PWM_STATE+i*2);
    servo_state[i] = servo_pwm;
  }

  // velezj: rssII Arm
  gripper_theta = gripper_pwm_to_theta( unpack_short( buffer, ORC_SERVO_PWM_STATE + ORC_ARM_GRIPPER_SERVO * 2 ) );

}

static void unpack_slave_packet(unsigned char *buffer)
{
  short left_tick, right_tick, time_ticks, delta_slave_ticks;
  double left_displacement, right_displacement;
  int left_delta_tick, right_delta_tick;
  double delta_slave_time;
  int left_dir, right_dir;
  unsigned char left_pinmode, right_pinmode;
  unsigned short voltage;
  static double start;

  // velezj: RssII Arm
  short shoulder_tick, elbow_tick;
  int shoulder_delta_tick, elbow_delta_tick;
  int shoulder_dir, elbow_dir;
  double shoulder_angle_change, elbow_angle_change;

  //carmen_warn("Got slave packet\n");
  //printf("unpack_slave_packet\n");

  if (!initialized) {
    start = carmen_get_time();
    initialized = 1;
    x = 0;
    y = 0;
    theta = 0;

    left_last_tick = unpack_short(buffer, ORC_LEFT_MOTOR_ENCODER);
    right_last_tick = ((buffer[ORC_RIGHT_MOTOR_ENCODER]&0x00ff)<<8)+
      (buffer[ORC_RIGHT_MOTOR_ENCODER+1]&0x00ff);

    last_slave_ticks = ((buffer[ORC_SLAVE_TIME]&0x00ff)<<8)+
      (buffer[ORC_SLAVE_TIME+1]&0x00ff);
    left_pwm = buffer[ORC_LEFT_MOTOR_ACTUAL_PWM];
    right_pwm = buffer[ORC_RIGHT_MOTOR_ACTUAL_PWM];

    left_dir = ((buffer[ORC_LEFT_MOTOR_DIR] & 0x0f) >> 2) & 0x03;
    right_dir = ((buffer[ORC_RIGHT_MOTOR_DIR] & 0x0f) >> 2) & 0x03;

    if (left_dir == ORC_FORWARD)
      left_pwm = -left_pwm;
    if (right_dir == ORC_BACKWARD)
      right_pwm = -right_pwm;

    // velezj: RssII Arm
    shoulder_theta = 0.0;
    elbow_theta = 0.0;
    shoulder_last_tick = unpack_short( buffer, ORC_SHOULDER_MOTOR_ENCODER );
    elbow_last_tick = unpack_short( buffer, ORC_ELBOW_MOTOR_ENCODER );
    shoulder_pwm = buffer[ ORC_SHOULDER_MOTOR_ACTUAL_PWM ];
    elbow_pwm = buffer[ ORC_ELBOW_MOTOR_ACTUAL_PWM ];
    shoulder_dir = ( ( buffer[ ORC_SHOULDER_MOTOR_DIR ] & 0x0f ) ) & 0x03;
    elbow_dir = ( ( buffer[ ORC_ELBOW_MOTOR_DIR ] & 0x0f ) ) & 0x03;
    if( shoulder_dir == ORC_BACKWARD )
      shoulder_pwm = -shoulder_pwm;
    if( elbow_dir == ORC_BACKWARD )
      elbow_pwm = -elbow_pwm;

    return;
  }

  voltage = unpack_short(buffer, ORC_SERVO_CURRENT);
  servo_current[0] = voltage_to_current(voltage);
  voltage = unpack_short(buffer, ORC_SERVO_CURRENT+2);
  servo_current[1] = voltage_to_current(voltage);

  left_pinmode = buffer[ORC_LEFT_PINMODE];
  right_pinmode = buffer[ORC_RIGHT_PINMODE];

  left_pwm = buffer[ORC_LEFT_MOTOR_ACTUAL_PWM];
  right_pwm = buffer[ORC_RIGHT_MOTOR_ACTUAL_PWM];

  left_dir = ((buffer[ORC_LEFT_MOTOR_DIR] & 0x0f) >> 2) & 0x03;
  right_dir = ((buffer[ORC_RIGHT_MOTOR_DIR] & 0x0f) >> 2) & 0x03;

  if (left_dir == ORC_FORWARD)
    left_pwm = -left_pwm;
  if (right_dir == ORC_BACKWARD)
    right_pwm = -right_pwm;

  left_tick = ((buffer[ORC_LEFT_MOTOR_ENCODER]&0x00ff)<<8)+
    (buffer[ORC_LEFT_MOTOR_ENCODER+1]&0x00ff);

  left_delta_tick = left_tick - left_last_tick;
  if (left_delta_tick > SHRT_MAX/2)
    left_delta_tick = left_delta_tick - 2*SHRT_MAX;
  if (left_delta_tick < -SHRT_MAX/2)
    left_delta_tick = left_delta_tick + 2*SHRT_MAX;

  right_tick = ((buffer[ORC_RIGHT_MOTOR_ENCODER]&0x00ff)<<8)+
    (buffer[ORC_RIGHT_MOTOR_ENCODER+1]&0x00ff);
  right_delta_tick = right_tick - right_last_tick;
  if (right_delta_tick > SHRT_MAX/2)
    right_delta_tick = right_delta_tick - 2*SHRT_MAX;
  if (right_delta_tick < -SHRT_MAX/2)
    right_delta_tick = right_delta_tick + 2*SHRT_MAX;

  left_last_tick = left_tick;
  right_last_tick = right_tick;

  right_delta_tick = -right_delta_tick;

  left_displacement = delta_tick_to_metres(left_delta_tick);
  right_displacement = delta_tick_to_metres(right_delta_tick);


  // velezj: For RssII Arm
  shoulder_pwm = buffer[ORC_SHOULDER_MOTOR_ACTUAL_PWM];
  elbow_pwm = buffer[ORC_ELBOW_MOTOR_ACTUAL_PWM];

  shoulder_dir = ((buffer[ORC_SHOULDER_MOTOR_DIR] & 0x0f) ) & 0x03;
  elbow_dir = ((buffer[ORC_ELBOW_MOTOR_DIR] & 0x0f) ) & 0x03;

  if (shoulder_dir == ORC_BACKWARD)
    shoulder_pwm = -shoulder_pwm;
  if (elbow_dir == ORC_BACKWARD)
    elbow_pwm = -elbow_pwm;

  shoulder_tick = ((buffer[ORC_SHOULDER_MOTOR_ENCODER]&0x00ff)<<8)+
    (buffer[ORC_SHOULDER_MOTOR_ENCODER+1]&0x00ff);

  shoulder_delta_tick = shoulder_tick - shoulder_last_tick;
  if (shoulder_delta_tick > SHRT_MAX/2)
    shoulder_delta_tick = shoulder_delta_tick - 2*SHRT_MAX;
  if (shoulder_delta_tick < -SHRT_MAX/2)
    shoulder_delta_tick = shoulder_delta_tick + 2*SHRT_MAX;

  elbow_tick = ((buffer[ORC_ELBOW_MOTOR_ENCODER]&0x00ff)<<8)+
    (buffer[ORC_ELBOW_MOTOR_ENCODER+1]&0x00ff);
  elbow_delta_tick = elbow_tick - elbow_last_tick;
  if (elbow_delta_tick > SHRT_MAX/2)
    elbow_delta_tick = elbow_delta_tick - 2*SHRT_MAX;
  if (elbow_delta_tick < -SHRT_MAX/2)
    elbow_delta_tick = elbow_delta_tick + 2*SHRT_MAX;

  shoulder_last_tick = shoulder_tick;
  elbow_last_tick = elbow_tick;

  //elbow_delta_tick = -elbow_delta_tick;

  shoulder_angle_change = delta_ticks_to_angle(shoulder_delta_tick);
  elbow_angle_change = delta_ticks_to_angle(elbow_delta_tick);
  if( fabs( shoulder_desired_theta - shoulder_theta ) > 0.01 ) {
    printf( "shoulder_delta_theta: %f  (%d)\n", shoulder_angle_change, shoulder_delta_tick );
  }
  if( fabs( elbow_desired_theta - elbow_theta ) > 0.01 ) {
    printf( "elbow_delta_theta: %f  (%d)\n", elbow_angle_change, elbow_delta_tick );
  }



  time_ticks = ((buffer[ORC_SLAVE_TIME]&0x00ff)<<8)+
    (buffer[ORC_SLAVE_TIME+1]&0x00ff);

  delta_slave_ticks = time_ticks - last_slave_ticks;
  last_slave_ticks = time_ticks;

  if (delta_slave_ticks > SHRT_MAX/2)
    delta_slave_ticks -= 2*SHRT_MAX;
  if (delta_slave_ticks < -SHRT_MAX/2)
    delta_slave_ticks += 2*SHRT_MAX;

  delta_slave_time = delta_slave_ticks/4000.0;

  displacement = (left_displacement+right_displacement)/2;
  rotation = atan2(right_displacement-left_displacement, ORC_WHEEL_BASE);

  left_velocity = left_displacement/delta_slave_time;
  right_velocity = right_displacement/delta_slave_time;

  x = x+cos(theta);
  y = y+sin(theta);
  theta = carmen_normalize_theta(theta+rotation);

  // velezj: For RssII Arm
  shoulder_theta = shoulder_theta + shoulder_angle_change;
  elbow_theta = elbow_theta + elbow_angle_change;
  // since elbow relative to shoulder and when the shoulder moves to elbow angle
  // in fact does NOT change with it, so we must update our elbow angle even when
  // only the shoulder moves
  //elbow_theta -= shoulder_angle_change; 
  shoulder_angular_velocity = shoulder_angle_change / delta_slave_time;
  elbow_angular_velocity = elbow_angle_change / delta_slave_time;
  if( fabs( shoulder_desired_theta - shoulder_theta ) > 0.01 || fabs( elbow_desired_theta - elbow_theta ) > 0.01 ) {
    printf( "shoulder_theta: %f  (av = %f)\n", shoulder_theta, shoulder_angular_velocity );
    printf( "elbow_theta:    %f  (av = %f)\n", elbow_theta, elbow_angular_velocity  );
  }

  time_since_last_command = carmen_get_time() - last_command_time;
  last_command_time = carmen_get_time();
  command_velocity(left_desired_velocity, left_velocity, left_pwm,
       ORC_LEFT_MOTOR);
  command_velocity(right_desired_velocity, right_velocity, right_pwm,
       ORC_RIGHT_MOTOR);
}

int carmen_arm_direct_reset(void)
{
  printf("carmen_arm_direct_reset\n");
  initialized = 0;
  return 0;
}


int carmen_arm_direct_initialize(char *model __attribute__ ((unused)), char *dev)
{
  int result;
  unsigned char buffer[5];

  result = carmen_serial_connect(&serial_fd, dev);
  if(result == -1) {
    serial_fd = -1;
    return -1;
  }

  if (strstr(dev, "USB") == NULL) {
    carmen_warn("This doesn't look like a USB port. Setting linespeed to 115200\n");
    carmen_serial_configure(serial_fd, 115200, "8N1");
  } else {
    carmen_warn("This looks like a USB port. Setting linespeed to 500000\n");
    carmen_serial_configure(serial_fd, 500000, "8N1");
  }

  fprintf(stderr, "Clearing input buffer...");
  carmen_serial_ClearInputBuffer(serial_fd);
  carmen_arm_direct_update_status();
  carmen_arm_direct_update_status();
  carmen_serial_ClearInputBuffer(serial_fd);
  fprintf(stderr, " ok.\n");

  buffer[0] = 'C';
  buffer[1] = 0; // Motor Encoder 0
  buffer[2] = 14; // Quad Phase Fast

  fprintf(stderr, "Setting motor encoder 0 into quad phase fast mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok.\n");

  buffer[1] = 1; // Motor Encoder 0
  fprintf(stderr, "Setting motor encoder 1 into quad phase fast mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 2; // Motor Encoder 1
  fprintf(stderr, "Setting motor encoder 2 into quad phase fast mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 3; // Motor Encoder 1
  fprintf(stderr, "Setting motor encoder 3 into quad phase fast mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[0] = 'W';
  buffer[1] = ORC_LEFT_MOTOR; 
  buffer[2] = 10;             // Quad Phase Fast
  fprintf(stderr, "Setting left motor slew to %d... ", buffer[2]);
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = ORC_RIGHT_MOTOR; 
  fprintf(stderr, "Setting right motor slew to %d... ", buffer[2]);
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");  



  //Edsinger: added servo 
  buffer[0] = 'C';
  buffer[1] = 0; // Servo0
  buffer[2] = ORC_SERVO_PIN; //Servo
  fprintf(stderr, "Setting servo pin 0 into servo mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 1; // Servo1
  buffer[2] = ORC_SERVO_PIN; //Servo
  fprintf(stderr, "Setting servo pin 1 into servo mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 2; // Servo2
  buffer[2] = ORC_SERVO_PIN; //Servo
  fprintf(stderr, "Setting servo pin 2 into servo mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 3; // Servo3
  buffer[2] = ORC_SERVO_PIN; //Servo
  fprintf(stderr, "Setting servo pin 3 into servo mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 10; // Bumper
  buffer[2] = ORC_DIGITAL_IN_PULL_UP;
  fprintf(stderr, "Setting bumper 0 into digital pull-up mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 11; // Bumper
  buffer[2] = ORC_DIGITAL_IN_PULL_UP;
  fprintf(stderr, "Setting bumper 1 into digital pull-up mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 12; // Bumper
  buffer[2] = ORC_DIGITAL_IN_PULL_UP;
  fprintf(stderr, "Setting bumper 2 into digital pull-up mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = 13; // Bumper
  buffer[2] = ORC_DIGITAL_IN_PULL_UP;
  fprintf(stderr, "Setting bumper 3 into digital pull-up mode... ");
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  fprintf(stderr, "Setting gripper into digital pull-up mode... ");
  buffer[1] = 12; // Gripper
  buffer[2] = ORC_DIGITAL_IN_PULL_UP;
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[0] = 0x4D;
  buffer[1] = ORC_LEFT_MOTOR;
  buffer[2] = 0;
  buffer[3] = 0;
  fprintf(stderr, "Zeroing left motor velocity ... ");
  if (send_packet_and_ack(buffer, 4, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  buffer[1] = ORC_RIGHT_MOTOR;
  fprintf(stderr, "Zeroing right motor velocity... ");
  if (send_packet_and_ack(buffer, 4, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  fprintf(stderr, "Checking board status... ");
  if (carmen_arm_direct_update_status() < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok\n");

  return 0;
}

int carmen_arm_direct_shutdown(void)
{
  unsigned char buffer[5];

  buffer[0] = 0x4D;
  buffer[1] = ORC_LEFT_MOTOR;
  buffer[2] = ORC_FORWARD;
  buffer[3] = 0;
  send_packet_and_ack(buffer, 4, ORC_SLAVE);  buffer[0] = 0x4D;

  buffer[1] = ORC_RIGHT_MOTOR;
  buffer[2] = ORC_FORWARD;
  buffer[3] = 0;
  send_packet_and_ack(buffer, 4, ORC_SLAVE);

  close(serial_fd);

  return 0;
}

static unsigned char *get_status_packet(unsigned char where)
{
  unsigned char byte;
  unsigned char *buffer = NULL;
  unsigned char routing = 0;
  int count;

  byte = ORC_STATUS;

  count = 0;
  do {
    send_packet(&byte, 1, where);
    buffer = read_packet();
    if (buffer != NULL) {
      byte = buffer[2];
      routing = byte >> 6;
      if (buffer[3] == ORC_STATUS && routing == where) {
	return buffer;
      } else {
	carmen_warn("Out of order packet %c %d %d %d\n", buffer[3],
		    buffer[3], routing, where);
	free(buffer);
	buffer = NULL;
      }
    }
    if (!buffer || buffer[0] != 0xED || buffer[3] != 0)
      count++;
    //    carmen_warn("Count %d\n", count);
  } while (count < 10);
  
  return buffer;

}

int carmen_arm_direct_update_status(void)
{
  unsigned char *buffer;
  double start_time = carmen_get_time();

  buffer = get_status_packet(ORC_MASTER);
  if (buffer == NULL)
    return -1;
  unpack_master_packet(buffer);
  free(buffer);

  if(0)
    carmen_warn("time to update: %.2f\n", carmen_get_time() -
		start_time);

  buffer = get_status_packet(ORC_SLAVE);
  if (buffer == NULL)
    return -1;
  unpack_slave_packet(buffer);
  free(buffer);

  if(0)
  carmen_warn("time to update: %.2f\n", carmen_get_time() -
	      start_time);
  return 0;
}

/* void carmen_arm_direct_set(double servos[], int num_servos) */
/* { */
/*   unsigned char buffer[4]; */
/*   unsigned short pwm; */
/*   int i; */

  
/* if (num_servos > 4) { */
/*     carmen_warn("orc_lib.c only supports 4 servos (%d sent)\n" */
/*                 "Returning only 4\n", num_servos); */
/*     num_servos = 4; */
/*   } */

/*  for (i=0;i<num_servos;i++) */
/*    {  */
/*      int nservo=(int)servos[i]; */
/*     pwm = nservo; */
/*     buffer[0] = 0x53; */
/*     buffer[1] = i; */
/*     buffer[2] = pwm >> 8; */
/*     buffer[3] = pwm & 0x00ff; */
/*     send_packet(buffer, 4, ORC_MASTER); */
/*   } */
/* } */



/* void carmen_arm_direct_get(double servos[], int num_servos, */
/* 			   double *currents, int *gripper) */
/* { */
/*   int i; */

  
/*   if (num_servos > 4) { */
/*     carmen_warn("orc_lib.c only supports 4 servos (%d requested)\n" */
/*                 "Returning only 4\n", num_servos); */
/*     num_servos = 4; */
/*   } */
 
/*   for (i = 0; i < num_servos; i++) { */
/*     servos[i] = servo_state[i]; */
/*   } */
/*   if (currents && num_servos > 0) { */
/*     currents[0] = servo_current[0]; */
/*     if (num_servos > 1) */
/*       currents[1] = servo_current[1]; */
/*   } */
/*   if (gripper) */
/*     *gripper = gripper_state; */
/* } */
