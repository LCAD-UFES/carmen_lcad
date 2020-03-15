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

#ifndef _ORCCONSTANTS_H
#define _ORCCONSTANTS_H

// return values
#define CMD_SUCCESS 0x00
#define CMD_UNKNOWN 0x01
#define CMD_WRONGARGCOUNT  0x02
#define CMD_BADARG 0x03
#define CMD_FAILED 0x05

// commands <0x80 are for orcboard

#define CMD_STDIN 0x00
#define CMD_MOTOR_SET 0x01
#define CMD_QUADPHASE_READ 0x02
#define CMD_I2C_TRANSACTION 0x3
#define CMD_ANALOG_READ 0x04
#define CMD_GYRO_SETUP 0x05
#define CMD_GYRO_CALIBRATE 0x06
#define CMD_GYRO_READ 0x07
#define CMD_PINMODE_SET 0x08
#define CMD_DIGOUT_SET 0x09
#define CMD_DIGIN_READ 0x0A
#define CMD_PADSENSE_READ 0x0B
#define CMD_SERVO_SET 0x0C
#define CMD_INTEGRATOR_READ 0x0D
#define CMD_INTEGRATOR_MUX_SET 0x0E
#define CMD_TIME_READ 0x0F
#define CMD_ANALOG_FILTER_SET 0x10
#define CMD_ORC_NULL 0x11
#define CMD_MOTOR_ACCEL 0x12
#define CMD_GYRO_GET_CALIBRATION 0x13
#define CMD_PWMCLK_SET 0x14

// commands >=0x80 are for orcpad

#define CMD_CONSOLE_WRITECHARS 0x80
#define CMD_LCD_CLEAR 0x81
#define CMD_LCD_FILL 0x82
#define CMD_LCD_DRAWCHARS 0x83
#define CMD_CONSOLE_HOME 0x84
#define CMD_CONSOLE_GOTO 0x85
#define CMD_LCD_WRITE 0x86
#define CMD_LCD_READ 0x87
#define CMD_ORCSTATUS_WRITE 0x88
#define CMD_PAD_NULL 0x89

// packet offsets
#define PACKET_ID      1
#define PACKET_DATALEN 2
#define PACKET_DATA    3

#define PACKET_MARKER 0xED

#define PACKET_ID_ORCBOARD_BROADCAST 0xFF
#define PACKET_ID_ORCPAD_BROADCAST   0xFE
#define PACKET_ID_ORCBOARD_TO_ORCPAD 0xFD
#define PACKET_ID_ORCD               0xF7
#define PACKET_ID_NO_ACK             0xF0

#define MSG_ASYNC_ODOMETRY 0x0010
#define MSG_ASYNC_ORCRESET 0x0020
#define MSG_ASYNC_HEARTBEAT 0x0030

#define PACKET_MAX_LEN 48
#define PACKET_MAX_DATALEN (PACKET_MAX_LEN - 4)

#endif
