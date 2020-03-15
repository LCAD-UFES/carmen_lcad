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

#define ORC_LEFT_MOTOR 0
#define ORC_RIGHT_MOTOR 2

#define ORC_LEFT_ENCODER_PORT 0
#define ORC_RIGHT_ENCODER_PORT 2

#define ORC_LEFT_REVERSE_ENCODER 1
#define ORC_RIGHT_REVERSE_ENCODER -1


#define ORC_MAX_ANGULAR_VEL 8.0 // Radians / seconds

#define ORC_MAX_PWM 80
// #define ORC_FF_GAIN ((ORC_MAX_PWM / ORC_MAX_ANGULAR_VEL) * 0.9)
#define ORC_P_GAIN 20
#define ORC_D_GAIN 10 //3 //5
#define ORC_I_GAIN 0 //0

#define ORC_WHEEL_DIAMETER .2525
#define ORC_WHEEL_BASE .37
#define ORC_TICKS_PER_REVOLUTION 20000.0
#define ORC_ENCODER_RESOLUTION 4550
#define ORC_GEAR_RATIO 1 // 12.5 ??? 
//#define ORC_TICKS_PER_REVOLUTION = ORC_ENCODER_RESOLUTION*ORC_GEAR_RATIO


// --------- of questionable use -----------

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

// Configuring IR sensor to use Sonar ports
#define ORC_LEFT_IR_PING 5
#define ORC_RIGHT_IR_PING 7

// Configure pins to digital
#define ORC_LEFT_IR_ECHO 4
#define ORC_RIGHT_IR_ECHO 6

// Sets modes for pins  
#define ORC_IR_PING 3 // Digital Out; 
#define ORC_IR_ECHO 1 // Digital In (Pull-Up)

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

#define ORC_BUMPER_PORT0 10
#define ORC_BUMPER_PORT1 11
#define ORC_BUMPER_PORT2 12
#define ORC_BUMPER_PORT3 13

#define ORC_VEL_ACCEL_TEMP 0.9
#define ORC_VEL_DECEL_TEMP 0.4

#define ORC_MASTER_TIME 4
#define ORC_SLAVE_TIME 48

#define ORC_DIGITAL_IN_PULL_UP 1
#define ORC_DIGITAL_IN 6

#define ORC_SERVO_CURRENT 35
#define ORC_SERVO_PWM_STATE 8
#define ORC_SERVO_PIN 5
