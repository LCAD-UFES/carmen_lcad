/**
   \file  Robots/LoBot/irccm/LoIO.h
   \brief Serial I/O API for Robolocust control program running on iRobot
   Create Command Module.

   This file defines an API for serial port I/O to be used by the various
   modules of the Robolocust control program for the iRobot Create
   Command Module.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoIO.h $
   $Id: LoIO.h 13744 2010-08-02 02:55:15Z mviswana $
*/

#ifndef LOBOT_IRCCM_IO_DOT_H
#define LOBOT_IRCCM_IO_DOT_H

/*----------------------------- CONSTANTS -----------------------------*/

// The different I/O error states/codes
enum {
   LOBOT_IO_OK,
   LOBOT_IO_TIMEOUT,
} ;

/*-------------------------- INITIALIZATION ---------------------------*/

/// Set up the serial port
void lo_init_comm(void) ;

/// Reset communication speed on both Create robot and Command Module
void lo_reset_baud(int baud_rate) ;

/// Flush the RX and TX buffers
void lo_clear_buffers(void) ;

/*------------------------- SEND/RECEIVE API --------------------------*/

/// Transmit a byte over the serial port
void lo_tx(char byte) ;

/// Receive the specified number of bytes over the serial port, waiting a
/// maximum number of milliseconds before giving up. If the timeout is
/// zero, wait forever for the data to come in.
///
/// The function returns the number of bytes successfully read.
///
/// Clients should setup a large enough buffer to accommodate the amount
/// of data they require and check the return value before using the data
/// in the buffer. If the number of bytes read is less than the number
/// requested, clients can query the I/O error to see what went wrong.
///
/// NOTE: Since we're using 8-bit integers, only a maximum of 255 bytes
/// can be retrieved at one time. For the purposes of this control
/// program, that number is more than enough.
unsigned char lo_rx(char* buf, unsigned char n, unsigned int timeout) ;

/// Receive a single byte over the serial port, waiting a maximum number
/// of milliseconds before giving up. If the timeout is zero, wait
/// forever for the data byte to come in.
///
/// Clients should check the I/O error flag before using the byte
/// returned by this function.
char lo_rx1(unsigned int timeout) ;

/*------------------------- USART REDIRECTION -------------------------*/

/**
   Since the Command Module has only one USART, it needs to be switched
   between talking to the USB port and the iRobot Create's serial port.

   Usually, the low-level Robolocust control program that runs on the
   command module will listen for incoming high-level motor commands over
   the USB port. But every time it needs to actually execute one of those
   commands, it will have to switch the USART to talk to the Create's
   serial port and then back again to continue listening to the higher
   layers of the Robolocust controller.

   These functions switch the USART between the Command Module's USB port
   and the Create's serial port.
*/
//@{
void lo_io_to_usb(void) ;
void lo_io_to_create(void) ;
//@}

/*-------------------------- ERROR CHECKING ---------------------------*/

/// Retrieve current I/O status. After each I/O operation, clients should
/// use this function to check for errors.
char lo_io_error(void) ;

/*---------------------------------------------------------------------*/

#endif
