/**
   \file  Robots/LoBot/irccm/LoTimer.h
   \brief Generalized timer API for Robolocust's iRobot Create Command
   Module control program.

   This file defines an API for "generalized" one and ten millisecond
   timers for the iRobot Create Command Module control program used by
   Robolocust. The API provides a means for other parts of the control
   program to setup "stop clocks" and perform whatever operations they
   need while this stop clock ticks away. Additionally, these timers are
   "generalized" in the sense that client modules may register arbitrary
   callback functions that will be triggered each time the timer ISR
   kicks in.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoTimer.h $
   $Id: LoTimer.h 13689 2010-07-23 03:38:35Z mviswana $
*/

#ifndef LOBOT_IRCCM_TIMER_DOT_H
#define LOBOT_IRCCM_TIMER_DOT_H

/*------------------------------- TYPES -------------------------------*/

/// The ATmega128 provides three timers that tick away in parallel with
/// the main clock. These three timers provide a means for asynchronous
/// operations w.r.t. the main "thread" of execution. However, as there
/// are only three such timers, if each module that required some form of
/// asynchrony were to "capture" a timer for itself, only three modules
/// could possibly take advantage of this feature.
///
/// To work around this limitation, the Robolocust low-level controller
/// uses the ATmega128's two 8-bit timers to implement a 1ms timer and a
/// 10ms timer. This timer module then associates a list of arbitrary
/// callbacks with each of these timers and allows client modules to
/// register their timer callbacks here, thus, enabling all modules to
/// take advantage of asynchrony if required. Hence the term "generalized
/// timer."
///
/// The following type defines the signature for timer callback
/// functions.
///
/// NOTE: Timer callbacks should not be computationally involved. Rather
/// they should perform a quick couple of assignments, if statements and
/// other simple operations. Otherwise, the timer callbacks run the risk
/// of not being triggered every one and ten milliseconds.
///
/// NOTE 2: This timer module supports a maximum of ten callbacks per
/// generalized timer.
typedef void (*TimerCallback)(void) ;

/*-------------------------- INITIALIZATION ---------------------------*/

/// This function sets up the ATmega's two 8-bit timers, one to fire
/// every millisecond and the other to fire every ten milliseconds.
/// Client modules can then register their timer callbacks to take
/// advantage of the asynchrony supported by the ATmega128.
void lo_init_timer(void) ;

/// This function adds a timer callback to the 1ms timer's callback list.
///
/// NOTE: A maximum of 10 such callbacks are supported.
void lo_add_timer1_cb(TimerCallback) ;

/// This function adds a timer callback to the 10ms timer's callback
/// list.
///
/// NOTE: A maximum of 10 such callbacks are supported.
void lo_add_timer10_cb(TimerCallback) ;

/*----------------------------- TIMER API -----------------------------*/

/// Return the current total number of milliseconds that have elapsed
/// since the low-level controller's 1ms timer was setup.
unsigned long lo_ticks(void) ;

/// Setup a stop watch for the specified number of milliseconds.
void lo_setup_timer(unsigned int ms) ;

/// Check if a stop watch is running.
char lo_timer_is_running(void) ;

/// Busy wait for the specified number of milliseconds.
void lo_wait(unsigned int ms) ;

/// This function can be used to temporarily disable the 10ms timer's
/// (i.e., the ATmega Timer2) interrupts.
#define lo_suspend_timer10() TIMSK2 &= ~0x02

/// This function can be used to reenable temporarily disabled 10ms timer
/// (i.e., the ATmega Timer2) interrupts.
#define lo_resume_timer10() TIMSK2 |= 0x02

/*---------------------------------------------------------------------*/

#endif
