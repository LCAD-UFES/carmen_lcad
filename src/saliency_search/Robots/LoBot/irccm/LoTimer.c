/**
   \file  Robots/LoBot/irccm/LoTimer.c
   \brief Timer API for Robolocust's iRobot Create Command Module control
   program.

   This file defines the functions and variables used to implement the
   API for "generalized" one and ten millisecond timers for the iRobot
   Create Command Module control program used by Robolocust. This API
   provides a means for other parts of the control program to setup "stop
   clocks" and perform whatever operations they need while this stop
   clock ticks away. Additionally, these timers are "generalized" in the
   sense that client modules may register arbitrary callback functions
   that will be triggered each time the timer ISR kicks in.
*/

/*
 ************************************************************************
 * The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   *
 * by the University of Southern California (USC) and the iLab at USC.  *
 * See http:  iLab.usc.edu for information about this project.          *
 *                                                                      *
 * Major portions of the iLab Neuromorphic Vision Toolkit are protected *
 * under the U.S. patent ``Computation of Intrinsic Perceptual Saliency *
 * in Visual Environments, and Applications'' by Christof Koch and      *
 * Laurent Itti, California Institute of Technology, 2001 (patent       *
 * pending; application number 09/912,225 filed July 23, 2001; see      *
 * http:  pair.uspto.gov/cgi-bin/final/home.pl for current status).     *
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoTimer.c $
   $Id: LoTimer.c 13781 2010-08-12 17:37:02Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoTimer.h"

// AVR headers
#include <avr/interrupt.h>
#include <avr/io.h>

/*----------------------------- CONSTANTS -----------------------------*/

// Both the 1ms and 10ms timers are associated with a list of arbitrary
// callback functions that facilitate asynchronous operations across the
// entire low-level controller. This enum specifies the maximum number of
// callbacks supported for each "generalized timer."
enum {
   LOBOT_MAX_TIMER_CALLBACKS = 5
} ;

/*-------------------------- DATA STRUCTURES --------------------------*/

// This structure is used to hold each generalized timer's list of
// callbacks.
typedef struct {
   int n ; // total number of callbacks in the list
   TimerCallback cb[LOBOT_MAX_TIMER_CALLBACKS] ; // callback list entries
} cb_list ;

/*------------------------------ GLOBALS ------------------------------*/

// This module uses the following variable to keep track of the total
// number of milliseconds that have elapsed since the 1ms timer was
// setup.
static volatile unsigned long g_total_ticks ;

/*
   The stop watch implemented by this module works as follows: client
   modules setup a timer for some number of milliseconds and then test to
   see if the stop watch is running. For example:

           lo_setup_timer(1000) ;
           while (lo_timer_is_running())
              do_something() ;

   These two variables are used internally by this module to implement
   the above functionality.

   When clients setup a timer, g_stop_watch_ticks will be set to the
   number of milliseconds the stop watch should last and the
   g_stop_watch_running flag will be set to indicate that the stop watch
   is on.

   The one millisecond timer's interrupt handler will then count down
   g_stop_watch_ticks. When the ticks reach zero, the interrupt handler
   will clear the g_stop_watch_running flag to indicate that the stop
   watch is done.

   DEVNOTE: These two variables are modified by an interrupt service
   routine. Therefore, they *must* be declared volatile to prevent the
   compiler from optimizing them away.
*/
static volatile unsigned int g_stop_watch_ticks ;
static volatile char g_stop_watch_running ;

// These two variables hold the callbacks for the two "generalized
// timers" implemented by this module.
static volatile cb_list g_timer1_callbacks  ;
static volatile cb_list g_timer10_callbacks ;

/*-------------------------- INITIALIZATION ---------------------------*/

/*
   The following function sets up the 8-bit Timer0 to generate an
   interrupt every 1 ms (thus its designation within the low-level
   controller as timer1).

   To do this, we need to set the WGM01 (Waveform Generation Mode) bit in
   Timer0's Timer/Counter Control Register (TCCR0A). This will enable
   Clear Timer on Compare (CTC) mode so that the hardware automatically
   clears the timer's count register after the timer fires (which means
   that we don't have to do it in software).

   As per the ATmega168 datasheet, the WGM01 bit is bit #1. Since we
   don't care about the other bits in TCCR0A, we can set those to zero.
   Therefore, the value for TCCR0A will have to 0x02 in order to setup
   the timer in the mode we want.

   Next, we need to select a clock source for the timer. To do that we
   have to set the CS bits in TCCR0B. These are bits 0, 1 and 2. Again,
   we don't care about the other bits in this register.

   Since we want a timer with one millisecond resolution and the main
   clock signal is running at 18.432MHz, we will need a prescaler to
   ensure that the counter value we are counting up to can fit in the
   8-bit TCNT0 register.

   The timer's counter value for a given target frequency is computed
   using the following formula:

                            C = F/(p * f) - 1

   where F = the main clock frequency (18.432MHz)
         p = the selected prescaler
         f = the target frequency

   For the ATmega168, Timer0 supports prescaler values of 1 (i.e., no
   prescale, which means that the timer runs at the same speed as the
   core CPU), 16, 64, 256 and 1024.

   Since we want a 1ms timer, our target frequency, f, is 1000Hz.

   Thus, the possible counter values for the available prescalers are:

                               p        C
                             ----    ------
                                1    18,431
                               16     1,151
                               64       287
                              256        71
                             1024        17

   Clearly, the first three options are unviable because we will need a
   16-bit counter to store those C values. Therefore, we go with one of
   the last two in the above table. Quite arbitrarily, we choose p = 1024
   and C = 17.

   To use a prescaler of 1024, we will have to set bits CS02 and CS00 in
   TCCR0B. In hex, the value we're talking about is 0x05.

   And, as shown above, the Output Compare Register (OCR0A) will have to
   be initialized with a value of 17 (or 0x11 in hexadecimal).

   Finally, to actually generate the desired interrupt when OCR0A reaches
   a value of 17, we have to setup Timer0's interrupt mask by fiddling
   with the right set of bits in the TIMSK0 register. The bit in question
   is OCIE0A (Timer/Counter0 Output Compare Match A Interrupt Enable).

   As the per the ATmega168 datasheet, OCIE0A is bit #1 in TIMSK0. Thus,
   setting TIMSK0 to 0x02 results in the timer interrupt being fired as
   configured by TCCR0A, TCCR0B and OCR0A.
*/
static void timer1_init(void)
{
   TCCR0A = 0x02 ; // CTC mode
   TCCR0B = 0x05 ; // prescaler = 1024
   OCR0A  = 0x11 ; // count from 0 to 17 for 1ms
   TIMSK0 = 0x02 ; // trigger interrupt when OCR0A reaches above value
}

/*
   The following function sets up the 8-bit Timer2 to generate an
   interrupt every 10 ms (thus its designation within this low-level
   control program as timer10).

   To setup Timer2 to fire every 10ms, we first need to set the WGM01
   (Waveform Generation Mode) bit in Timer2's Timer/Counter Control
   Register (TCCR2A). This will enable Clear Timer on Compare (CTC) mode
   so that the hardware automatically clears the timer's count register
   after the timer fires (which means that we don't have to do it in
   software).

   As per the ATmega168 datasheet, the WGM01 bit is bit #1. Since we
   don't care about the other bits in TCCR2A, we can set those to zero.
   Therefore, the value for TCCR2A will have to be 0x02 in order to setup
   the timer in the mode we want.

   Next, we need to select a clock source for the timer. To do that we
   have to set the CS bits in TCCR2B. These are bits 0, 1 and 2. Again,
   we don't care about the other bits in this register.

   Since we want a timer with 10 ms resolution and the main clock signal
   is running at 18.432MHz, we will need a prescaler to ensure that the
   counter value we are counting up to can fit in the 8-bit TCNT2
   register.

   The timer's counter value for a given target frequency is computed
   using the following formula:

                            C = F/(p * f) - 1

   where F = the main clock frequency (18.432MHz)
         p = the selected prescaler
         f = the target frequency

   For the ATmega168, Timer2 supports prescaler values of 1 (i.e., no
   prescale, which means that the timer runs at the same speed as the
   core CPU), 8, 32, 64, 128, 256 and 1024.

   Since we want a 10ms timer, our target frequency, f, is 100Hz.

   Thus, the possible counter values for the available prescalers are:

                               p       C
                             ----   -------
                                1   184,319
                               32     5,759
                               64     2,879
                              128     1,439
                              256       719
                             1024       179

   Clearly, the last option is the only viable one because its C value is
   the only one that will fit in an 8-bit register.

   To use a prescaler of 1024, we will have to set bits CS22, CS21 and
   CS20 in TCCR2B. In hex, the value we're talking about is 0x07.

   And, as shown above, the Output Compare Register (OCR2A) will have to
   be initialized with a value of 179 (B3 in hexadecimal).

   Finally, to actually generate the desired interrupt when OCR2A reaches
   a value of 17, we have to setup Timer0's interrupt mask by fiddling
   with the right set of bits in the TIMSK2 register. The bit in question
   is OCIE2A (Timer/Counter2 Output Compare Match A Interrupt Enable).

   As the per the ATmega168 datasheet, OCIE2A is bit #1 in TIMSK2. Thus,
   setting TIMSK2 to 0x02 results in the timer interrupt being fired as
   configured by TCCR2A, TCCR2B and OCR2A.
*/
static void timer10_init(void)
{
   TCCR2A = 0x02 ; // CTC mode
   TCCR2B = 0x07 ; // prescaler = 1024
   OCR2A  = 0xB3 ; // count from 0 to 179 for 10ms
   TIMSK2 = 0x02 ; // trigger interrupt when OCR2A reaches above value
}

// This function initializes the two "generalized timers" supported by
// this module.
void lo_init_timer(void)
{
   timer1_init() ;
   timer10_init() ;
}

// Add a 1ms timer callback
void lo_add_timer1_cb(TimerCallback t)
{
   if (g_timer1_callbacks.n < LOBOT_MAX_TIMER_CALLBACKS)
      g_timer1_callbacks.cb[g_timer1_callbacks.n++] = t ;
}

// Add a 10ms timer callback
void lo_add_timer10_cb(TimerCallback t)
{
   if (g_timer10_callbacks.n < LOBOT_MAX_TIMER_CALLBACKS)
      g_timer10_callbacks.cb[g_timer10_callbacks.n++] = t ;
}

/*---------------------- TIMER INTERRUPT HANDLERS ---------------------*/

// Helper function to invoke each of the callback functions stored in the
// supplied callback list.
static void trigger_callbacks(volatile cb_list* callbacks)
{
   for (int i = 0; i < callbacks->n; ++i)
      (*(callbacks->cb[i]))() ;
}

// Timer0 interrupt handler for the 1ms timer: this "generalized timer"
// is responsible for maintaining the "global clock" (viz., the total
// 1ms ticks since the timer was setup) and for implementing the 1ms
// resolution timer delays.
//
// Additionally, it triggers the 1ms timer callbacks registered by client
// modules.
ISR(TIMER0_COMPA_vect)
{
   ++g_total_ticks ;

   if (g_stop_watch_ticks)
      --g_stop_watch_ticks ;
   else
      g_stop_watch_running = 0 ;

   trigger_callbacks(&g_timer1_callbacks) ;
}

// Timer2 interrupt handler for the 10ms timer: this "generalized timer"
// only triggers the callback functions registered by client modules; it
// has no other task other than helping facilitate application wide
// support for asynchronous operation in 10ms increments.
ISR(TIMER2_COMPA_vect)
{
   trigger_callbacks(&g_timer10_callbacks) ;
}

/*----------------------------- TIMER API -----------------------------*/

// Return the current total number of milliseconds that have elapsed
// since Timer0 was setup.
unsigned long lo_ticks(void)
{
   return g_total_ticks ;
}

// Setup a stop watch for the specified number of milliseconds
void lo_setup_timer(unsigned int ms)
{
   g_stop_watch_ticks   = ms ;
   g_stop_watch_running = 1 ;
}

// Check if a stop watch is running
char lo_timer_is_running()
{
   return g_stop_watch_running ;
}

// Busy wait for the specified number of milliseconds
void lo_wait(unsigned int ms)
{
   g_stop_watch_ticks   = ms ;
   g_stop_watch_running = 1 ;
   while (g_stop_watch_running)
      ;
}
