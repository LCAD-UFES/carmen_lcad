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
#include "../base_low_level.h"
#include <carmen/carmenserial.h>
#include <sys/ioctl.h>
#include <limits.h>

#define ORC_MASTER 0
#define ORC_SLAVE 1
#define ORC_PAD 2

#define ORC_STATUS 0x2A

// Sonar defines

#define ORC_LEFT_SONAR_PING_PIN 4
#define ORC_LEFT_SONAR_ECHO_PIN 5
#define ORC_RIGHT_SONAR_PING_PIN 6
#define ORC_RIGHT_SONAR_ECHO_PIN 7
#define ORC_SONAR_PING_MODE 6
#define ORC_SONAR_ECHO_MODE 7

#define ORC_LEFT_SONAR_PING 0
#define ORC_RIGHT_SONAR_PING 1
#define ORC_LEFT_SONAR_RANGE 49
#define ORC_RIGHT_SONAR_RANGE 51

// Arm defines

#define ORC_SERVO_PIN_0 0
#define ORC_SERVO_PIN_1 1
#define ORC_SERVO_PIN_2 2
#define ORC_SERVO_PIN_3 3

#define ORC_GRIPPER_PIN 12

#define ORC_SERVO_CURRENT 35
#define ORC_SERVO_PWM_STATE 8
#define ORC_SERVO_MODE 5

// Bumper defines


#define ORC_BUMPER_PIN_0 8
#define ORC_BUMPER_PIN_1 9
#define ORC_BUMPER_PIN_2 10
#define ORC_BUMPER_PIN_3 11

#define ORC_DIGITAL_IN_PULL_UP 1
#define ORC_DIGITAL_IN 6

// Configuring IR sensor to use Sonar ports
#define ORC_LEFT_IR_PING 5
#define ORC_RIGHT_IR_PING 7

// Configure pins to digital
#define ORC_LEFT_IR_ECHO 4
#define ORC_RIGHT_IR_ECHO 6

// Sets modes for pins  
#define ORC_IR_PING 3 // Digital Out; 
#define ORC_IR_ECHO 1 // Digital In (Pull-Up)

#define ORC_LEFT_MOTOR 0
#define ORC_RIGHT_MOTOR 2

#define ORC_LEFT_MOTOR_ACTUAL_PWM 14
#define ORC_RIGHT_MOTOR_ACTUAL_PWM 19

#define ORC_LEFT_MOTOR_SLEW 15
#define ORC_RIGHT_MOTOR_SLEW 21

#define ORC_LEFT_PINMODE 4
#define ORC_RIGHT_PINMODE 5

#define ORC_LEFT_MOTOR_QUAD_PORT 16
#define ORC_RIGHT_MOTOR_QUAD_PORT 18

#define ORC_LEFT_ENCODER_STATE 4
#define ORC_RIGHT_ENCODER_STATE 5

#define ORC_QUAD_PHASE_FAST 14

#define ORC_LEFT_MOTOR_DIR 25
#define ORC_RIGHT_MOTOR_DIR 26
#define ORC_FORWARD 1
#define ORC_BACKWARD 2

#define ORC_MAX_ANGULAR_VEL 8.0 // Radians / seconds

#define ORC_MAX_PWM 250
#define ORC_FF_GAIN ((ORC_MAX_PWM / ORC_MAX_ANGULAR_VEL) * 0.9)
#define ORC_P_GAIN 20
#define ORC_D_GAIN 5

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

static double acceleration;
static double deceleration;

static double x, y, theta;
static double displacement, rotation;
static double left_velocity, right_velocity;
static double left_error_prev = 0.0, right_error_prev = 0.0;
static double left_desired_velocity = 0, right_desired_velocity = 0;
static double left_range, right_range;
static int initialized = 0;
static int sonar_on = 1;


// We ignore the d-term the first time we enter the control loop
// after receiving a new velocity command. This is because the
// d-term captures the derivative of the error, which is not 
// meaningful if the error is caused by the command that moved
// the desired velocity set point.
static int ignore_left_d_term = 1;
static int ignore_right_d_term = 1;

static int left_pwm = 0, right_pwm = 0;
static short last_master_ticks;
static short last_slave_ticks;
static int left_last_tick, right_last_tick;
static double time_since_last_command;
static double last_command_time;

static double servo_state[4];
static double servo_current[2];

static unsigned char irs[4];
static unsigned char bumpers[4];
static int gripper_state = 0;
static int serial_fd = -1;

static double left_displacement, right_displacement;
static double delta_slave_time;

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
    carmen_warn("Trying to reconnect to base...\n");
    carmen_base_direct_initialize_robot(NULL,  NULL);
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
  buffer[2] = (where << 6) | 0xF;
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
  //  fd_set rfds;

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
	//	FD_ZERO(&rfds);
	//	FD_SET(0, &rfds);
	//        num_ready = select(1, &rfds, NULL, NULL, &timeout);
        num_ready = select(0, NULL, NULL, NULL, &timeout);
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
    //  } while (!buffer && packet_count < 3);
  } while (!buffer);

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
    //    carmen_warn("Resending: %d\n", count);
    count++;
  } while (count < 3);

  return -1;
}
void carmen_base_command_velocity(double desired_velocity, 
				  double current_velocity,
				  unsigned char WHICH_MOTOR)
{
  double desired_angular_velocity, current_angular_velocity;
  double pTerm = 0.0, dTerm = 0.0, velError = 0.0;
  double *velErrorPrevPtr;
  double pGain = ORC_P_GAIN;

  int current_pwm;
  int new_pwm;

  unsigned char command_pwm;
  unsigned char dir;
  unsigned char buffer[4];
  int *ignore_d_term;

  if (WHICH_MOTOR == ORC_LEFT_MOTOR) {
    velErrorPrevPtr = &left_error_prev;
    current_pwm = left_pwm;
    ignore_d_term = &ignore_left_d_term;
    pGain = pGain * 1.2;
  } else {
    velErrorPrevPtr = &right_error_prev;
    current_pwm = right_pwm;
    ignore_d_term = &ignore_right_d_term;
  }

  desired_angular_velocity = desired_velocity / (ORC_WHEEL_DIAMETER/2.0);
  current_angular_velocity = current_velocity / (ORC_WHEEL_DIAMETER/2.0);

  if (fabs(desired_angular_velocity) > .001) {
    if (desired_angular_velocity > ORC_MAX_ANGULAR_VEL)
      desired_angular_velocity = ORC_MAX_ANGULAR_VEL;
    if (desired_angular_velocity < -ORC_MAX_ANGULAR_VEL)
      desired_angular_velocity = -ORC_MAX_ANGULAR_VEL;

    velError = (desired_angular_velocity - current_angular_velocity);
    pTerm = velError * pGain;

    if (!*ignore_d_term)
      dTerm = (velError - *velErrorPrevPtr) * ORC_D_GAIN;
    else {
      dTerm = 0;
      *ignore_d_term = 0;
    }

    new_pwm = current_pwm + carmen_round(pTerm + dTerm);

    if(0)
    carmen_warn("%s %f %f : %f %f : %d %d %d\n", 
		(WHICH_MOTOR == ORC_LEFT_MOTOR ? "left" : "right"),
		desired_angular_velocity,
		current_angular_velocity, pTerm, dTerm,
		current_pwm, carmen_round(pTerm+dTerm), new_pwm);

    *velErrorPrevPtr = velError;

    if (abs(new_pwm) > ORC_MAX_PWM)
      new_pwm = (new_pwm > 0 ? ORC_MAX_PWM : -ORC_MAX_PWM);

    if (WHICH_MOTOR == ORC_RIGHT_MOTOR) {
      if (new_pwm < 0) 
	dir = ORC_FORWARD;
      else 
	dir = ORC_BACKWARD;
      //      carmen_warn("left %s\n", (dir == 1) ? "forward" : "backward");
    } else {
      if (new_pwm < 0) 
	dir = ORC_BACKWARD;
      else 
	dir = ORC_FORWARD;
      //      carmen_warn("right %s\n", (dir == 1) ? "forward" : "backward");
    }
    command_pwm = abs(new_pwm);
  } else {
    dir = ORC_FORWARD;
    command_pwm = 0;
  }

  if (0)
  carmen_warn("Tried to send %d %d %f %f %d\n", command_pwm, new_pwm,
	      desired_velocity, current_velocity, WHICH_MOTOR);

  buffer[0] = 0x4D;
  buffer[1] = WHICH_MOTOR;
  buffer[2] = dir;
  buffer[3] = command_pwm;
  send_packet_and_ack(buffer, 4, ORC_SLAVE);
}

static double delta_tick_to_metres(int delta_tick)
{
  double revolutions = (double)delta_tick/
    (double)(ORC_ENCODER_RESOLUTION*ORC_GEAR_RATIO);
  double radians = revolutions*2*M_PI;
  double metres = radians*(ORC_WHEEL_DIAMETER/2.0);

  return metres;
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
  unsigned char command_buffer[2];
  short digital_io_state;
  unsigned short servo_pwm;
  int i;
  static double last_ping_time = 0;
  static int last_ping = ORC_RIGHT_SONAR_PING;

  //carmen_warn("Got master packet\n");
  //printf("unpack_master_packet\n");

  range = unpack_short(buffer, ORC_LEFT_SONAR_RANGE);
  if (range == 0xffff)
    left_range = 0;
  else
    left_range = range/1000000.0*331.46/2.0;

  range = unpack_short(buffer, ORC_RIGHT_SONAR_RANGE);
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

  digital_io_state = unpack_short(buffer, ORC_DIGITAL_IN);

  irs[0] = digital_io_state >> ORC_LEFT_IR_ECHO & 1;
  irs[1] = digital_io_state >> ORC_RIGHT_IR_ECHO & 1; // 1 means there's a curb (beyond range of IR)

  bumpers[0] = digital_io_state >> 8 & 1;
  bumpers[1] = digital_io_state >> 9 & 1;
  bumpers[2] = digital_io_state >> 10 & 1;
  bumpers[3] = digital_io_state >> 11 & 1;
  gripper_state = digital_io_state >> 12 & 1;
 
  for (i = 0; i < 4; i++) {
    servo_pwm = unpack_short(buffer, ORC_SERVO_PWM_STATE+i*2);
    servo_state[i] = servo_pwm;
  }

}

static void unpack_slave_packet(unsigned char *buffer)
{
  short left_tick, right_tick, time_ticks, delta_slave_ticks;
  int left_delta_tick, right_delta_tick;
  int left_dir, right_dir;
  unsigned char left_pinmode, right_pinmode;
  unsigned short voltage;
  static double start;
  int left_orc_encoder, right_orc_encoder;

  unsigned char left_slew, right_slew;

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

    if (left_dir == ORC_BACKWARD)
      left_pwm = -left_pwm;
    if (right_dir == ORC_FORWARD)
      right_pwm = -right_pwm;

    return;
  }

  left_orc_encoder = buffer[ORC_LEFT_ENCODER_STATE];
  if (left_orc_encoder != ((0x0e << 4) + 0x0e))
    carmen_warn("left encoder is in a bad state: %d %d\n", 
		(left_orc_encoder >> 4) & 0xf, (left_orc_encoder & 0xf));
  //  else
  //    carmen_warn("left encoder is in a good state: %d %d\n", 
  //		(left_orc_encoder >> 4) & 0xf, (left_orc_encoder & 0xf));
  right_orc_encoder = buffer[ORC_RIGHT_ENCODER_STATE];
  if (right_orc_encoder != ((0x0e << 4) + 0x0e))
    carmen_warn("right encoder is in a bad state: %d %d\n", 
		(right_orc_encoder >> 4) & 0xf, right_orc_encoder & 0xf);
  //  else
  //    carmen_warn("right encoder is in a good state: %d %d\n", 
  //		(right_orc_encoder >> 4) & 0xf, right_orc_encoder & 0xf);

  voltage = unpack_short(buffer, ORC_SERVO_CURRENT);
  servo_current[0] = voltage_to_current(voltage);
  voltage = unpack_short(buffer, ORC_SERVO_CURRENT+2);
  servo_current[1] = voltage_to_current(voltage);

  left_pinmode = buffer[ORC_LEFT_PINMODE];
  right_pinmode = buffer[ORC_RIGHT_PINMODE];

  left_pwm = buffer[ORC_LEFT_MOTOR_ACTUAL_PWM];
  right_pwm = buffer[ORC_RIGHT_MOTOR_ACTUAL_PWM];

  left_slew = buffer[ORC_LEFT_MOTOR_SLEW];
  right_slew = buffer[ORC_RIGHT_MOTOR_SLEW];

  if (0)
  carmen_warn("left slew: %d right slew: %d\n", left_slew,
	      right_slew);

  left_dir = ((buffer[ORC_LEFT_MOTOR_DIR] & 0x0f) >> 2) & 0x03;
  right_dir = ((buffer[ORC_RIGHT_MOTOR_DIR] & 0x0f) >> 2) & 0x03;

  if (left_dir == ORC_BACKWARD)
    left_pwm = -left_pwm;
  if (right_dir == ORC_FORWARD)
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

  left_delta_tick = -left_delta_tick;

  left_displacement = delta_tick_to_metres(left_delta_tick);
  right_displacement = delta_tick_to_metres(right_delta_tick);

  if (0)
  carmen_warn("left %d %f right %d %f\n", left_delta_tick, left_displacement,
	      right_delta_tick, right_displacement);

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

  if (0) {
    carmen_warn("Displacement: %f rotation: %f\n", displacement,
		carmen_radians_to_degrees(rotation));
    carmen_warn("before %f %f %f\n", x, y, carmen_radians_to_degrees(theta));
  }

  x = x+displacement*cos(theta);
  y = y+displacement*sin(theta);
  theta = carmen_normalize_theta(theta+rotation);

  if(0)
  carmen_warn("after %f %f %f (%f %f)\n", x, y, carmen_radians_to_degrees(theta),
	      displacement, carmen_radians_to_degrees(rotation));

  left_velocity = left_displacement/delta_slave_time;
  right_velocity = right_displacement/delta_slave_time;

  carmen_base_command_velocity(left_desired_velocity, left_velocity,
       ORC_LEFT_MOTOR);
  carmen_base_command_velocity(right_desired_velocity, right_velocity,
       ORC_RIGHT_MOTOR);

  time_since_last_command = carmen_get_time() - last_command_time;
  last_command_time = carmen_get_time();
}

int carmen_base_direct_sonar_on(void)
{
  printf("carmen_base_direct_sonar_on\n");
  sonar_on = 1;

  return 2;
}

int carmen_base_direct_sonar_off(void)
{
  sonar_on = 0;
  return 2;
}


int carmen_base_direct_reset(void)
{
  printf("carmen_base_direct_reset\n");
  x = 0;
  y = 0;
  theta = 0;

  initialized = 0;
  return 0;
}

static int orc_set_encoder_quad_phase_fast(int motor) 
{
  unsigned char buffer[5];

  buffer[0] = 'C';
  buffer[1] = motor; // Motor Encoder 0
  buffer[2] = ORC_QUAD_PHASE_FAST; // Quad Phase Fast

  fprintf(stderr, "Setting motor channel %d into quad phase fast mode... ",
	  motor);
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok.\n");

  return 0;
}

static int orc_set_motor_slew(int motor, int slew) 
{
  unsigned char buffer[5];

  buffer[0] = 'W';
  buffer[1] = motor;
  buffer[2] = slew;
  fprintf(stderr, "Setting %s motor slew to %d... ", 
	  (motor == ORC_LEFT_MOTOR ? "left" : "right") , slew);
  if (send_packet_and_ack(buffer, 3, ORC_SLAVE) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok.\n");

  return 0;
}

int orc_set_pin(int pin, char *description, int pin_mode) 
{
  unsigned char buffer[5];
  char *modes[15] = {"digital in", "digital in (pull-up)",
		    "digital in (pull-down)", "digital out",
		    "digital out (slow)", "servo", "sonar ping",
		    "sonar echo", "analog in", "analog out",
		    "clock generator out", "quadrature phase",
		    "mono phase", "unknown", "quad-phase fast"};
  buffer[0] = 'C';
  buffer[1] = pin; // Servo0
  buffer[2] = pin_mode; //Servo
  fprintf(stderr, "Setting %s pin %d into %s mode... ", description, pin,
	  modes[pin_mode]);
  if (send_packet_and_ack(buffer, 3, ORC_MASTER) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok.\n");

  return 0;
}

int orc_set_pwm(int motor, int dir, int pwm)
{
  unsigned char buffer[5];

  buffer[0] = 0x4D;
  buffer[1] = motor;
  buffer[2] = dir;
  buffer[3] = pwm;
  if (send_packet_and_ack(buffer, 4, ORC_SLAVE) < 0) {
    return -1;
  }

  return 0;
}


int carmen_base_direct_initialize_robot(char *model __attribute__ ((unused)),
          char *dev)
{
  int result;

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

  carmen_base_direct_update_status(NULL);
  carmen_base_direct_update_status(NULL);
  carmen_serial_ClearInputBuffer(serial_fd);
  fprintf(stderr, " ok.\n");

  // Set pins for Motor Encoder 0 into fast mode

  if (orc_set_encoder_quad_phase_fast(ORC_LEFT_MOTOR) < 0)
    return -1;
  if (orc_set_encoder_quad_phase_fast(ORC_LEFT_MOTOR+1) < 0)
    return -1;

  // Set pins for Motor Encoder 1 into fast mode

  if (orc_set_encoder_quad_phase_fast(ORC_RIGHT_MOTOR) < 0)
    return -1;
  if (orc_set_encoder_quad_phase_fast(ORC_RIGHT_MOTOR+1) < 0)
    return -1;

  // Set motor slew rates 

  if (orc_set_motor_slew(ORC_LEFT_MOTOR, 10) < 0)
    return -1;
  if (orc_set_motor_slew(ORC_RIGHT_MOTOR, 10) < 0)
    return -1;

  // Set servo pins into servo mode
  
  if (orc_set_pin(ORC_SERVO_PIN_0, "servo", ORC_SERVO_MODE) < 0)
    return -1;
  if (orc_set_pin(ORC_SERVO_PIN_1, "servo", ORC_SERVO_MODE) < 0)
    return -1;
  if (orc_set_pin(ORC_SERVO_PIN_2, "servo", ORC_SERVO_MODE) < 0)
    return -1;
  if (orc_set_pin(ORC_SERVO_PIN_3, "servo", ORC_SERVO_MODE) < 0)
    return -1;

  if (orc_set_pin(ORC_LEFT_SONAR_PING_PIN, "left sonar ping",
		  ORC_SONAR_PING_MODE) < 0)
    return -1;
  if (orc_set_pin(ORC_LEFT_SONAR_ECHO_PIN, "left sonar echo",
		  ORC_SONAR_ECHO_MODE) < 0)
    return -1;
  if (orc_set_pin(ORC_RIGHT_SONAR_PING_PIN, "right sonar ping",
		  ORC_SONAR_PING_MODE) < 0)
    return -1;
  if (orc_set_pin(ORC_RIGHT_SONAR_ECHO_PIN, "right sonar echo",
		  ORC_SONAR_ECHO_MODE) < 0)
    return -1;

  if (orc_set_pin(ORC_BUMPER_PIN_0, "bumper", ORC_DIGITAL_IN_PULL_UP) < 0)
    return -1;
  if (orc_set_pin(ORC_BUMPER_PIN_1, "bumper", ORC_DIGITAL_IN_PULL_UP) < 0)
    return -1;
  if (orc_set_pin(ORC_BUMPER_PIN_2, "bumper", ORC_DIGITAL_IN_PULL_UP) < 0)
    return -1;
  if (orc_set_pin(ORC_BUMPER_PIN_3, "bumper", ORC_DIGITAL_IN_PULL_UP) < 0)
    return -1;
  if (orc_set_pin(ORC_GRIPPER_PIN, "gripper sensor", 
		  ORC_DIGITAL_IN_PULL_UP) < 0)
    return -1;

  fprintf(stderr, "Zeroing left motor velocity ... ");
  if (orc_set_pwm(ORC_LEFT_MOTOR, 0, 0) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok.\n");
  fprintf(stderr, "Zeroing right motor velocity ... ");
  if (orc_set_pwm(ORC_RIGHT_MOTOR, 0, 0) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }
  fprintf(stderr, "ok.\n");

  fprintf(stderr, "Checking board status... ");
  if (carmen_base_direct_update_status(NULL) < 0) {
    fprintf(stderr, "%sfailed%s.\n", carmen_red_code, carmen_normal_code);
    return -1;
  }

  fprintf(stderr, "Orc Board initialization succeeded.\n");

  return 0;
}

int carmen_base_direct_shutdown_robot(void)
{
  carmen_base_direct_set_velocity(0.0, 0.0);

  close(serial_fd);

  return 0;
}

int carmen_base_direct_set_acceleration(double new_acceleration)
{
  acceleration = new_acceleration;
  //  printf("carmen_base_direct_set_acceleration: accel=%f\n", new_acceleration);

  return 0;
}

int carmen_base_direct_set_deceleration(double new_deceleration)
{
  deceleration = new_deceleration;
  //  printf("carmen_base_direct_set_deceleration: decel=%f\n", new_deceleration);

  return 0;
}

int carmen_base_direct_set_velocity(double new_tv, double new_rv)
{
  left_desired_velocity = new_tv;
  right_desired_velocity = new_tv;

  right_desired_velocity += new_rv/2;
  left_desired_velocity -= new_rv/2;

  carmen_base_command_velocity(left_desired_velocity, left_velocity,
       ORC_LEFT_MOTOR);
  carmen_base_command_velocity(right_desired_velocity, right_velocity,
       ORC_RIGHT_MOTOR);

  time_since_last_command = carmen_get_time() - last_command_time;
  last_command_time = carmen_get_time();

  printf("carmen_base_direct_set_velocity: tv=%.2f, rv=%.2f\n", 
	 new_tv, new_rv);
  carmen_base_direct_update_status(NULL);

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
    count++;
    if (0)
    carmen_warn("Count %d\n", count);
  } while (count < 5);
  
  return buffer;

}

int carmen_base_direct_update_status(double* packet_timestamp __attribute__((unused)))
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

int carmen_base_direct_get_state(double *disp_p __attribute__ ((unused)), 
				 double *rot_p __attribute__ ((unused)),
				 double *tv_p __attribute__ ((unused)), 
				 double *rv_p __attribute__ ((unused)))
{

  carmen_die("Turn base_hardware_integrator on\n");

  return 0;
}

int carmen_base_query_encoders(double *disp_p __attribute__ ((unused)),
			       double *rot_p __attribute__ ((unused)),
			       double *tv_p __attribute__ ((unused)),
			       double *rv_p __attribute__ ((unused)))
{
  carmen_die("Turn base_hardware_integrator on\n");

  return 0;
}

int carmen_base_query_low_level(double *left_disp, double *right_disp,
				double *delta_time)
{
  int err;

  err = carmen_base_direct_update_status(NULL);

  if (err < 0)
    return -1;

  *left_disp = left_displacement;
  *right_disp = right_displacement;
  *delta_time = delta_slave_time;

  return 0;
}

int carmen_base_direct_get_integrated_state(double *x_p, double *y_p,
					    double *theta_p, double *tv_p,
					    double *rv_p)
{
  int err;

  err = carmen_base_direct_update_status(NULL);
  if (err < 0)
    return -1;

  if(0)
  carmen_warn("before %f %f %f\n", x, y, theta);

  *x_p = x;
  *y_p = y;
  *theta_p = theta;
  *tv_p = (left_velocity+right_velocity)/2;
  *rv_p = atan2(right_velocity-left_velocity, ORC_WHEEL_BASE);  

  return 0;
}

int carmen_base_direct_get_sonars(double *ranges, carmen_point_t *positions,
          int num_sonars)
{
  if (num_sonars >= 1) {
    ranges[0] = left_range;
    positions[0].x = .05;
    positions[0].y = -.10;
    positions[0].theta = -M_PI/2;
  }
  if (num_sonars >= 2) {
    ranges[1] = right_range;
    positions[1].x = .05;
    positions[1].y = .10;
    positions[1].theta = M_PI/2;
  }

  return 0;
}

int carmen_base_direct_get_bumpers(unsigned char *bumpers_p, int num_bumpers)
{
  if (num_bumpers >= 1)
    bumpers_p[0] = bumpers[0];
  if (num_bumpers >= 2)
    bumpers_p[1] = bumpers[1];
  if (num_bumpers >= 3)
    bumpers_p[2] = bumpers[2];
  if (num_bumpers >= 4)
    bumpers_p[3] = bumpers[3];
  return 4;
}

int carmen_base_direct_send_binary_data(unsigned char *data, int size)
{
  return carmen_serial_writen(serial_fd, data, size);
}

int  carmen_base_direct_get_binary_data(unsigned char **data
					__attribute__ ((unused)), 
					int *size __attribute__ ((unused)))
{

  return 0;
}

int carmen_arm_direct_num_velocities(int num_joints __attribute__ ((unused)))
{
  return 0;
}

int carmen_arm_direct_num_currents(int num_joints)
{
  if (num_joints > 2)
    return 2;
  return num_joints;
}

void carmen_arm_direct_set(double *joint_angles, int num_joints)
{
  unsigned char buffer[4];
  unsigned short pwm;
  int i;
  
  if (num_joints > 4) {
    carmen_warn("orc_lib.c only supports 4 joints (%d sent)\n"
                "Returning only 4\n", num_joints);
    num_joints = 4;
  }
  
  for (i=0;i<num_joints;i++) { 
    int nservo=(int)joint_angles[i];
    pwm = nservo;
    buffer[0] = 0x53;
    buffer[1] = i;
    buffer[2] = pwm >> 8;
    buffer[3] = pwm & 0x00ff;
    send_packet(buffer, 4, ORC_MASTER);
  }
}

void carmen_arm_direct_get_state(double *joint_angles, double *joint_currents,
				 double *joint_angular_vels
				 __attribute__ ((unused)), 
				 int *gripper_closed,
				 int num_joints)
{
  int i;
  
  if (num_joints > 4) {
    carmen_warn("orc_lib.c only supports 4 joints (%d requested)\n"
                "Returning only 4\n", num_joints);
    num_joints = 4;
  }
 
  for (i = 0; i < num_joints; i++) {
    joint_angles[i] = servo_state[i];
  }
  if (joint_currents && num_joints > 0) {
    joint_currents[0] = servo_current[0];
    if (num_joints > 1)
      joint_currents[1] = servo_current[1];
  }
  if (gripper_closed)
    *gripper_closed = gripper_state;
}

