/**
   \file  Robots/LoBot/irccm/LoIO.c
   \brief Serial I/O API for Robolocust control program running on iRobot
   Create Command Module.

   This file defines the functions for serial port I/O needed by the
   Robolocust iRobot Create Command Module control program.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoIO.c $
   $Id: LoIO.c 13744 2010-08-02 02:55:15Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoIO.h"
#include "LoTimer.h"
#include "LoOpenInterface.h"

// AVR headers
#include <avr/io.h>
#include <avr/interrupt.h>

/*------------------------------ GLOBALS ------------------------------*/

// When I/O operations fail, we set a flag to let clients know
static char g_io_error ;

/*-------------------------- INITIALIZATION ---------------------------*/

// Set up the serial port
void lo_init_comm()
{
   UBRR0  = LOBOT_OI_UBRR_57600 ;
   UCSR0B = (_BV(TXEN0)  | _BV(RXEN0))  ; // enable transmit & receive
   UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01)) ; // 8-bit data
}

// Reset communication speed on both Create robot and Command Module
void lo_reset_baud(int baud_rate)
{
   if (baud_rate < LOBOT_OI_BAUD_300 || baud_rate > LOBOT_OI_BAUD_115200)
      return ;

   // Send baud command and new baud rate
   lo_tx(LOBOT_OI_CMD_BAUD) ;
   UCSR0A |= _BV(TXC0) ;
   lo_tx(baud_rate) ;

   // Busy wait until transmit is complete
   while (! (UCSR0A & _BV(TXC0)))
      ;

   // Inhibit interrupts while we update the baud rate register
   cli() ;

   // Update baud rate register
   switch (baud_rate)
   {
      case LOBOT_OI_BAUD_115200:
         UBRR0 = LOBOT_OI_UBRR_115200 ;
         break ;
      case LOBOT_OI_BAUD_57600:
         UBRR0 = LOBOT_OI_UBRR_57600 ;
         break ;
      case LOBOT_OI_BAUD_38400:
         UBRR0 = LOBOT_OI_UBRR_38400 ;
         break ;
      case LOBOT_OI_BAUD_28800:
         UBRR0 = LOBOT_OI_UBRR_28800 ;
         break ;
      case LOBOT_OI_BAUD_19200:
         UBRR0 = LOBOT_OI_UBRR_19200 ;
         break ;
      case LOBOT_OI_BAUD_14400:
         UBRR0 = LOBOT_OI_UBRR_14400 ;
         break ;
      case LOBOT_OI_BAUD_9600:
         UBRR0 = LOBOT_OI_UBRR_9600 ;
         break ;
      case LOBOT_OI_BAUD_4800:
         UBRR0 = LOBOT_OI_UBRR_4800 ;
         break ;
      case LOBOT_OI_BAUD_2400:
         UBRR0 = LOBOT_OI_UBRR_2400 ;
         break ;
      case LOBOT_OI_BAUD_1200:
         UBRR0 = LOBOT_OI_UBRR_1200 ;
         break ;
      case LOBOT_OI_BAUD_600:
         UBRR0 = LOBOT_OI_UBRR_600 ;
         break ;
      case LOBOT_OI_BAUD_300:
         UBRR0 = LOBOT_OI_UBRR_300 ;
         break ;
      default:
         UBRR0 = LOBOT_OI_UBRR_57600 ;
         break ;
   }

   // Okay to receive interrupts again
   sei();

   // Small delay to let things stabilize
   lo_wait(100) ;
}

// Clear TX and RX buffers
void lo_clear_buffers(void)
{
   // Wait for TX buffer to empty
   while (! (UCSR0A & _BV(UDRE0)))
      ;

   // Empty the RX buffer
   char c ;
   while (UCSR0A & 0x80)
      c = UDR0 ;
}

/*------------------------- SEND/RECEIVE API --------------------------*/

// Transmit a byte over the serial port
void lo_tx(char byte)
{
   // Clear the communications error flag
   g_io_error = LOBOT_IO_OK ;

   // Busy wait for transmit buffer to become empty
   while (! (UCSR0A & _BV(UDRE0)))
      ;

   // Transmit buffer now empty ==> place byte in USART data register to send
   UDR0 = byte ;
}

// Receive the specified number of bytes over the serial port, waiting a
// maximum number of milliseconds before giving up. If the timeout is
// zero, wait forever for the data to come in.
//
// The function returns the number of bytes successfully read.
//
// Clients should setup a large enough buffer to accommodate the amount
// of data they require and check the return value before using the data
// in the buffer. If the number of bytes read is less than the number
// requested, clients can query the I/O error to see what went wrong.
unsigned char lo_rx(char* buf, unsigned char n, unsigned int timeout)
{
   // Clear the communications error flag
   g_io_error = LOBOT_IO_OK ;

   unsigned char i = 0 ;
   if (timeout == 0) // wait forever
   {
      for (; i < n; ++i) {
         while (! (UCSR0A & 0x80)) // serial data not yet available
            ;
         buf[i] = UDR0 ;
      }
   }
   else // wait for specified amount of time and then give up
   {
      lo_setup_timer(timeout) ;
      while (lo_timer_is_running() && i < n)
         if (UCSR0A & 0x80) // serial byte is available
            buf[i++] = UDR0 ;

      if (i < n)
         g_io_error = LOBOT_IO_TIMEOUT ;
   }
   return i ;
}

// Receive a single byte over the serial port, waiting a maximum number
// of milliseconds before giving up. If the timeout is zero, wait forever
// for the data byte to come in.
//
// Clients should check the I/O error flag before using the byte returned
// by this function.
char lo_rx1(unsigned int timeout)
{
   char c = 0 ;
   lo_rx(&c, 1, timeout) ;
   return c ;
}

/*------------------------- USART REDIRECTION -------------------------*/

void lo_io_to_usb()
{
   lo_wait(10) ; // just in case there are some pending bytes to/from Create
   PORTB |= 0x10 ;
   lo_wait(10) ; // need to wait at least 10 bit times after switching ports
}

void lo_io_to_create()
{
   lo_wait(10) ; // just in case there are some pending bytes to/from USB
   PORTB &= ~0x10 ;
   lo_wait(10) ; // need to wait at least 10 bit times after switching ports
}

/*-------------------------- ERROR CHECKING ---------------------------*/

// Retrieve current I/O status
char lo_io_error()
{
   return g_io_error ;
}
