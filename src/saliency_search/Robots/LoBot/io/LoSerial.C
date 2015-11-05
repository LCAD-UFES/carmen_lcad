/**
   \file  Robots/LoBot/io/LoSerial.C
   \brief This file defines the non-inline member functions of the
   lobot::Serial class.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoSerial.C $
// $Id: LoSerial.C 12858 2010-02-17 20:35:10Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoSerial.H"
#include "Robots/LoBot/misc/LoExcept.H"

// Standard C headers
#include <errno.h>

//------------------------------ MACROS ---------------------------------

// Sometimes, during development, it may not be convenient to have
// something actually connected to the serial port. We may simply want to
// fake the Propeller being attached. For these situations, we can
// uncomment the following line.
//#define LOBOT_SERIAL_DEVMODE 1

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- LIBSERIAL VERSION ---------------------------

#ifdef HAVE_LIBSERIAL

// Quick helper to convert integral baud rates to the correct enum
// expected by libserial
static SerialPort::BaudRate baud_rate_enum(int baud_rate)
{
   if (baud_rate < 63) // halfway between 50 and 75
      return SerialPort::BAUD_50 ;
   if (baud_rate < 93) // halfway between 75 and 110
      return SerialPort::BAUD_75 ;
   if (baud_rate < 122) // halfway between 110 and 134
      return SerialPort::BAUD_110 ;
   if (baud_rate < 142) // halfway between 134 and 150
      return SerialPort::BAUD_134 ;
   if (baud_rate < 175) // halfway between 150 and 200
      return SerialPort::BAUD_150 ;
   if (baud_rate < 250) // halfway between 200 and 300
      return SerialPort::BAUD_200 ;
   if (baud_rate < 450) // halfway between 300 and 600
      return SerialPort::BAUD_300 ;
   if (baud_rate < 900) // halfway between 600 and 1200
      return SerialPort::BAUD_600 ;
   if (baud_rate < 1500) // halfway between 1200 and 1800
      return SerialPort::BAUD_1200 ;
   if (baud_rate < 2100) // halfway between 1800 and 2400
      return SerialPort::BAUD_1800 ;
   if (baud_rate < 3600) // halfway between 2400 and 4800
      return SerialPort::BAUD_2400 ;
   if (baud_rate < 7200) // halfway between 4800 and 9600
      return SerialPort::BAUD_4800 ;
   if (baud_rate < 14400) // halfway between 9600 and 19200
      return SerialPort::BAUD_9600 ;
   if (baud_rate < 28800) // halfway between 19200 and 38400
      return SerialPort::BAUD_19200 ;
   if (baud_rate < 48000) // halfway between 38400 and 57600
      return SerialPort::BAUD_38400 ;
   if (baud_rate < 86400) // halfway between 57600 and 115200
      return SerialPort::BAUD_57600 ;
   if (baud_rate < 172800) // halfway between 115200 and 230400
      return SerialPort::BAUD_115200 ;
   return SerialPort::BAUD_230400 ;
}

// Constructor
Serial::Serial(const ModelManager&, const std::string& device, int baud_rate)
   : m_serial(device)
{
#ifdef LOBOT_SERIAL_DEVMODE
   ; // don't init serial port when working with development mode dummy
#else
   try
   {
      m_serial.Open(baud_rate_enum(baud_rate)) ;
   }
   catch (std::exception&)
   {
      throw io_error(SERIAL_PORT_INIT_ERROR) ;
   }
#endif
}

// See if the serial port has input data pending
bool Serial::ready()
{
   return m_serial.IsDataAvailable() ;
}

// Receive data from serial port
int Serial::read(char buf[], int n)
{
   int i = 0 ;
   try
   {
      for (; i < n; ++i)
         buf[i] = m_serial.ReadByte() ;
   }
   catch (std::exception&)
   {
   }
   return i ;
}

// Send data via serial port
void Serial::write(char buf[], int n)
{
   try
   {
      for (int i = 0; i < n; ++i)
         m_serial.WriteByte(buf[i]) ;
   }
   catch (std::exception&)
   {
      throw io_error(SERIAL_PORT_WRITE_ERROR) ;
   }
}

#else // don't HAVE_LIBSERIAL ==> fall back to INVT serial port support

//--------------------------- INVT VERSION ------------------------------

// Constructor
//
// DEVNOTE: INVT's Serial class expects a non-const ModelManager
// reference in its constructor. However, since the lobot::factory
// mechanism only works with const references to constructor parameters,
// the lobot::MotorBase constructor will pass a const ModelManager& to
// this function here. To ensure that INVT's Serial class construction
// works properly, we have to cast away the constness of the ModelManager
// passed in.
Serial::
Serial(const ModelManager& mgr, const std::string& device, int baud_rate)
   : m_serial(new ::Serial(const_cast<ModelManager&>(mgr),
                           "LobotSerialPort", "LobotSerialPort")),
     m_peek_flag(false), m_peek_char(0)
{
#ifdef LOBOT_SERIAL_DEVMODE
   ; // don't init serial port when working with development mode dummy
#else
   ModelManager& M = const_cast<ModelManager&>(mgr) ;

   bool i_stopped_the_model_manager = false ;
   if (M.started()) {
      M.stop() ;
      i_stopped_the_model_manager = true ;
   }

   M.addSubComponent(m_serial) ;
   m_serial->configure(device.c_str(), baud_rate, "8N1", false, false, 0) ;

   M.exportOptions(MC_RECURSE) ;
   if (i_stopped_the_model_manager)
      M.start() ;
#endif
}

// Check if the serial port has pending data that can be read
bool Serial::ready()
{
   if (m_peek_flag)
      return true ;

   m_serial->setBlocking(false) ;

   bool is_ready = false ;
   switch (m_serial->read(& m_peek_char, 1))
   {
      case -1:
         if (errno == EAGAIN)
            is_ready = false ;
         else
            throw io_error(SERIAL_PORT_READ_ERROR) ;
         break ;

      case 0: // EOF
         //is_ready = true ;
         is_ready = false ;
         break ;

      case 1:
         if (m_peek_char != '\0') { // often get bogus null chars; ignore 'em
            is_ready    = true ;
            m_peek_flag = true ;
         }
         break ;

      default: // hunh!?! this should never happen; but if it does, then
         errno = 0 ;
         throw io_error(SERIAL_PORT_READ_ERROR) ;
         break ;
   }

   m_serial->setBlocking(true) ;
   return is_ready ;
}

// Receive data from serial port
int Serial::read(char buf[], int n)
{
   if (n <= 0)
      throw io_error(SERIAL_PORT_BAD_ARG) ;

   if (m_peek_flag) {
      buf[0] = m_peek_char ;
      m_peek_flag = false ;
      m_peek_char = 0 ;
      return 1 + m_serial->read(buf + 1, n - 1) ;
   }
   return m_serial->read(buf, n) ;
}

// Send data via serial port
void Serial::write(char buf[], int n)
{
   if (n <= 0)
      throw io_error(SERIAL_PORT_BAD_ARG) ;
   m_serial->write(buf, n) ;
}

#endif // HAVE_LIBSERIAL

//------------- FUNCTIONS COMMON IN BOTH IMPLEMENTATIONS ----------------

// Read the specified number of bytes and don't return until they've all
// been received.
void Serial::read_full(char buf[], int n)
{
   for (int m = n; m > 0;)
      m -= read(buf + n - m, m) ;
}

// Read and discard specified number of bytes
void Serial::eat(int n)
{
   char buf[n] ;
   read_full(buf, n) ;
}

// Clean-up
Serial::~Serial(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
