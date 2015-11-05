/**
   \file  Robots/LoBot/irccm/LoCmdMain.c
   \brief Robolocust control program for the iRobot Create's Command
   Module.

   This file defines the main function for a control program meant to be
   run on the iRobot Create's Command Module. This program is designed to
   listen to the Command Module's USB port for incoming motor commands
   from the higher (C++) layers of the lobot controller and convert them
   into the equivalent sequence of Open Interface opcode and operand
   bytes to be sent to the iRobot Create.
*/

/*
 ************************************************************************
 * The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   *
 * by the University of Southern California (USC) and the iLab at USC.  *
 * See http://iLab.usc.edu for information about this project.          *
 *                                                                      *
 * Major portions of the iLab Neuromorphic Vision Toolkit are protected *
 * under the U.S. patent ``Computation of Intrinsic Perceptual Saliency *
 * in Visual Environments, and Applications'' by Christof Koch and      *
 * Laurent Itti, California Institute of Technology, 2001 (patent       *
 * pending; application number 09/912,225 filed July 23, 2001; see      *
 * http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     *
 ************************************************************************
 * This file is part of the iLab Neuromorphic Vision C++ Toolkit.       *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is free software; you can   *
 * redistribute it and/or modify it under the terms of the GNU General  *
 * Public License as published by the Free Software Foundation; either  *
 * version 2 of the License, or (at your option) any later version.     *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  *
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the   *
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      *
 * PURPOSE.  See the GNU General Public License for more details.       *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   *
 * to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   *
 * Boston, MA 02111-1307 USA.                                           *
 ************************************************************************
*/

/*
   Primary maintainer for this file: mviswana usc edu
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoCmdMain.c $
   $Id: LoCmdMain.c 13769 2010-08-08 01:34:02Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoRemote.h"
#include "LoBumps.h"
#include "LoCliffs.h"
#include "LoWheelDrops.h"
#include "LoDrive.h"
#include "LoSensors.h"
#include "LoBeep.h"
#include "LoUtils.h"
#include "LoIO.h"
#include "LoTimer.h"
#include "LoCMInterface.h"
#include "LoOpenInterface.h"

// AVR headers
#include <avr/io.h>
#include <avr/interrupt.h>

/*------------------------------- TYPES -------------------------------*/

// Rather than use a large switch-case construct to respond to high-level
// commands, we use a dispatch table containing function pointers. This
// type defines a command handler, i.e., a function that implements the
// code required to handle a high-level command. Such a function will
// take one parameter: a 16-bit word. If a particular command doesn't
// need any operands, its handler is free to ignore this input parameter.
typedef void (*CommandHandler)(int op) ;

// Each high-level command is identified by a single character. We use the
// following type to map these command IDs to their corresponding
// handlers.
typedef struct {
   char cmd ;
   CommandHandler function ;
} CommandInfo ;

/*------------------------------ GLOBALS ------------------------------*/

// As mentioned earlier, we use a dispatch table rather than a
// switch-case construct to handle the different high-level commands
// supported by the Robolocust controller.
//
// The following array is that dispatch table.
//
// DEVNOTE: To add new commands to the system, implement an appropriate
// handler function and then add its function pointer and other relevant
// info to this table.
static const CommandInfo g_command_map[] = {
   {LOBOT_CMD_NOP,      &lo_nop},
   {LOBOT_CMD_FORWARD,  &lo_forward},
   {LOBOT_CMD_REVERSE,  &lo_reverse},
   {LOBOT_CMD_STOP,     &lo_stop},
   {LOBOT_CMD_LEFT,     &lo_left},
   {LOBOT_CMD_RIGHT,    &lo_right},
   {LOBOT_CMD_STRAIGHT, &lo_straight},
   {LOBOT_CMD_SPIN,     &lo_cmd_spin},
   {LOBOT_CMD_ENABLE_REAR_BUMPS,  &lo_enable_rear_bumps},
   {LOBOT_CMD_DISABLE_REAR_BUMPS, &lo_disable_rear_bumps},
} ;

/*----------------------------- CONSTANTS -----------------------------*/

// How many different commands can this control program handle?
static const char LOBOT_NUM_COMMANDS =
   sizeof(g_command_map)/sizeof(g_command_map[0]) ;

// Number of milliseconds to wait for iRobot Create to boot
#ifndef LOBOT_CREATE_BOOT_DELAY
   #define LOBOT_CREATE_BOOT_DELAY 3500
#endif

// This control program continuously listens to the Command Module's USB
// port for the higher layers of the Robolocust controller to send it
// motor (and other commands).
//
// If it doesn't hear from the higher layers for more than some
// predefined amount of time, it will assume that the higher layers
// aren't functioning and will shut off the robot's motors to ensure that
// nothing catastrophic happens.
//
// The following constant specifies the number of milliseconds that can
// elapse before this control program will consider the higher layers of
// the system non-functional. That is, we will shut off the robot's
// motors if we don't receive any high-level commands within these many
// milliseconds since the previous command came in.
#ifndef LOBOT_COMPUTER_TIMEOUT
   #define LOBOT_COMPUTER_TIMEOUT 250
#endif

// When the control program starts up, it plays a little diddy after
// initialization to indicate that it is up and ready to start processing
// high-level commands. This value specifies the amount of time (in ms)
// to wait while the startup diddy plays.
#ifndef LOBOT_STARTUP_BEEP_DELAY
   #define LOBOT_STARTUP_BEEP_DELAY 5000
#endif

// Number of milliseconds to wait between consecutive iterations of
// this control program's main loop.
#ifndef LOBOT_UPDATE_DELAY
   #define LOBOT_UPDATE_DELAY 50
#endif

// Another useful auditory feedback: at the end of each iteration of the
// main loop play a little beep to let users know that the control
// program is still alive.
//
// NOTE: Playing the heartbeat beep in every iteration of the main loop
// will result in a one very long and annoying beep (because the update
// delay is usually very small). Therefore we emit the heartbeat sound
// every couple of iterations.
//
// The following number specifies the number of iterations between
// heartbeat beeps.
//
// DEVNOTE: To get the (approximate) number of milliseconds between
// heartbeats, multiply the following number by the main loop's update
// delay specified above.
#ifndef LOBOT_HEARTBEAT_INTERVAL
   #define LOBOT_HEARTBEAT_INTERVAL 25
#endif

/*-------------------------- INITIALIZATION ---------------------------*/

/*
   Function: init_command_module
    Purpose: Initialize Command Module's ATmega168 microcontroller

   Mostly, the initialization is simply a matter of configuring various
   pins. The ATmega168 has 3 I/O ports, viz., B, C and D. Pins B0-B3 and
   C0-C5 are exposed for end-user applications. The remaining pins are
   for internal use by the Command Module.

   For lobot, we don't use any of the command module's I/O pins. So we
   simply set them all to zero, i.e., to act as inputs and enable their
   pull-ups.

   B4 is used to redirect the internal UART towards either the USB port
   or the Create's serial port. This pin *must* be an output. Since this
   control program needs to listen to the USB port for incoming motor
   commands from the higher (C++) layers of the lobot controller, we
   initialize it to the high state. Whenever required, this pin will be
   cleared to redirect I/O to the Create's serial port and then set high
   again to continue listening for incoming commands.

   B5 is used to detect whether or not the Create is powered up; it
   *must* be an input.

   B6, B7, C6 and C7 are hard-wired and cannot be changed in software.

   Port D is also for internal use by the Command Module. However, it is
   available to end-user applications. Nonetheless, the port D pins are
   required to be configured in a specific way.
*/
static void init_command_module(void)
{
   // Configure the I/O pins
   DDRB  = 0x10 ; // B0-3 = input,   B4 = output, B5 = input
   PORTB = 0xCF ; // B0-3 = pull-up, B4 = Create, B5 = floating

   DDRC  = 0x00 ; // C0-5 = input
   PORTC = 0xFF ; // C0-5 = pull-up

   DDRD  = 0xE6 ; // reserved by Command Module; must be set like this
   PORTD = 0x7D ; // reserved by Command Module; must be set like this
}

// Power on the robot if required
static void init_robot(void)
{
   if (PINB & 0x20) // B5 = 1 ==> robot powered up
      return ;      // no need to do anything then

   while (! (PINB & 0x20)) // continue till B5 (Create power detect) is high
   {
      PORTD &= ~0x80 ; // clear D7 (the power control pin)
      lo_wait(500) ;   // delay to ensure Create sees new state

      PORTD |= 0x80 ;  // set D7 to turn power on
      lo_wait(500) ;   // delay to ensure Create sees new state

      PORTD &= ~0x80 ; // clear D7
   }
   lo_wait(LOBOT_CREATE_BOOT_DELAY) ;
}

// Initialize the Open Interface protocol
static void init_open_interface(void)
{
   lo_tx(LOBOT_OI_CMD_START) ;
   lo_reset_baud(LOBOT_OI_BAUD_57600) ;
   lo_tx(LOBOT_OI_CMD_FULL) ;
   lo_wait(100) ;
}

// Light the Play and Power LEDs on the Create as visual
// feedback/confirmation that the robot is up and the control program
// ready for action.
static void init_leds(void)
{
   lo_tx(LOBOT_OI_CMD_LEDS)  ;
   lo_tx(LOBOT_OI_LED_PLAY)  ; // turn Play LED on and Advance LED off
   lo_tx(0x00) ; lo_tx(0xFF) ; // Power LED: green at full intensity
   lo_wait(50) ;
}

/*-------------- LOW-LEVEL ACTIONS AND ACKNOWLEDGEMENTS ---------------*/

// Take care of wheel drops, cliffs, bumps and remote control commands in
// that order of preference.
static void react_to_sensors(void)
{
   lo_wheel_drops() ;
   if (lo_is_wheel_dropped()) // don't bother with the other sensors
      ;
   else // no wheel drops ==> look at cliffs, bumps and the remote control
   {
      lo_cliffs() ;
      if (lo_is_cliff_detected()) // don't bother with bumps
         ;
      else // no wheel drops and no cliffs ==> check bumps and remote
      {
         lo_bumps() ;
         if (lo_bumps_pending()) // don't bother with remote
            ;
         else
            lo_remote() ;
      }
   }
}

// If we reacted to low-level/built-in Roomba/Create sensors here, let
// the high-level controller know.
//
// NOTE: Order of preference ==> wheel drops, cliffs, bumps.
static void send_pending_acks(void)
{
   if (lo_wheel_drops_pending())
      lo_send_wheel_drops() ;
   else if (lo_cliffs_pending())
      lo_send_cliffs() ;
   else if (lo_bumps_pending())
      lo_send_bumps() ;
   else if (lo_remote_pending())
      lo_send_remote() ;

   if (lo_sensors_pending())
      lo_send_sensors() ;
}

/*------------------ COMMAND VALIDATION AND DESPATCH ------------------*/

// Check the parity of the high-level 4-byte command sequence
static char bad_parity(char cmd[LOBOT_CMD_SIZE])
{
   char xor = 0 ;
   for (unsigned char i = 0; i < LOBOT_CMD_SIZE; ++i)
      xor ^= cmd[i] ;
   return xor ; // should be all zeros; otherwise, bad parity
}

// Execute the high-level command sent in by lobot controller by looking
// up its handler in the command map and then invoking that function with
// the supplied parameter.
static void execute(char cmd, int param)
{
   for (unsigned char i = 0; i < LOBOT_NUM_COMMANDS; ++i)
      if (cmd == g_command_map[i].cmd) {
         (*g_command_map[i].function)(param) ;
         return ;
      }

   // Bogus command! High-level has gone bonkers ==> stop the robot
   lo_stop(0) ;
}

/*----------------------- MISCELLANEOUS HELPERS -----------------------*/

// Quick helper to check if any unsafe conditions have been detected or
// if the user is remote controlling the robot. If this is true, then the
// low-level controller will not accept any high-level commands.
#define ignore_high_level() \
   (lo_is_wheel_dropped() || lo_is_cliff_detected() || lo_is_remote_control())

// Quick helper to check if the Play button has been pressed
#define play_quit() \
   ((lo_get_sensor(LOBOT_SENSORS_BUTTONS) & LOBOT_OI_BUTTON_PLAY) == \
     LOBOT_OI_BUTTON_PLAY)

/*------------------------------- MAIN --------------------------------*/

int main(void)
{
   // Initialization steps
   cli() ; // don't want to be interrupted during initialization
   init_command_module() ;
   lo_init_timer() ;
   lo_init_comm() ;
   lo_init_sensors() ;
   lo_init_drive() ;
   sei() ; // okay (in fact, need) to start responding to interrupts now
   init_robot() ;
   init_open_interface() ;
   init_leds() ;
   lo_init_beeps() ;
   lo_stop(0) ; // just as a precaution

   // Play the startup signal to indicate that the program is ready to go
   lo_beep(LOBOT_BEEP_STARTUP) ;
   lo_wait(LOBOT_STARTUP_BEEP_DELAY) ;

   // Main loop: listen for high-level commands and execute them
   int heartbeat = LOBOT_HEARTBEAT_INTERVAL ;
   for(;;)
   {
      // Retrieve sensor data and react as required, sending
      // acknowledgements to the high level if necessary. When the user
      // presses the 'P' button on the remote or the Play button on the
      // robot, quit this low-level control program.
      lo_sensors() ;
      if (lo_sensors_pending()) {
         react_to_sensors() ;
         if (lo_remote_quit() || play_quit())
            break ;
      }

      // Send any pending acknowledgements and sensor data to high level.
      // But before we switch the USART to talk to the high level, we
      // need to ensure that the drive module's acceleration/decelartion
      // functionality doesn't try to issue drive commands while the
      // USART is switched. Otherwise, it will send drive commands to the
      // wrong destination and the high level will receive what it will
      // construe as bogus acknowledgement messages.
      lo_clear_buffers() ;
      lo_suspend_ramping() ;
      lo_io_to_usb() ;
      send_pending_acks() ;

      // If conditions are safe (e.g., no wheels dropped) and the user
      // isn't remote controlling the robot, then retrieve the next
      // high-level command and execute it. If no command is available,
      // stop the robot.
      char cmd[LOBOT_CMD_SIZE] = {0} ;
      if (ignore_high_level()) // unsafe conditions or remote control
         ; // don't send ACK_READY
      else // no unsafe conditions detected ==> okay to execute high-level cmds
      {
         lo_tx(LOBOT_ACK_READY) ;

         lo_rx(cmd, LOBOT_CMD_SIZE, LOBOT_COMPUTER_TIMEOUT) ;
         if (lo_io_error() || bad_parity(cmd))
            cmd[0] = LOBOT_CMD_STOP ;
      }

      lo_clear_buffers() ;
      lo_io_to_create() ;
      lo_resume_ramping() ;
      if (cmd[0])
         execute(cmd[0], lo_make_word(cmd[1], cmd[2])) ;

      // Short wait before the next iteration
      lo_wait(LOBOT_UPDATE_DELAY) ;

      // Emit heartbeat sound as auditory feedback to users that the
      // low-level control program is still alive and kicking...
      if (heartbeat-- == 0) {
         lo_beep(LOBOT_BEEP_HEARTBEAT) ;
         heartbeat = LOBOT_HEARTBEAT_INTERVAL ;
      }
   }

   // User pressed the 'P' button on the Roomba Remote or the Play
   // button on the robot to quit the Robolocust program and
   // automatically dock the robot with the home base.
   //
   // NOTE: Skip automatic docking. With the mini-ITX system + Li-Ion
   // batteries, the robot is quite heavy. Auto-docking can damage the
   // robot, the Roomba Home Base, or both. Best to lift robot and dock
   // it manually. Therefore, only put the robot back in passive mode.
   lo_beep(LOBOT_BEEP_QUITTING) ;
   lo_wait(LOBOT_STARTUP_BEEP_DELAY) ;
   //lo_tx(LOBOT_OI_CMD_DEMO) ;
   //lo_tx(LOBOT_OI_DEMO_COVER_AND_DOCK) ;
   lo_tx(LOBOT_OI_CMD_PASSIVE) ;

   return 0 ;
}
