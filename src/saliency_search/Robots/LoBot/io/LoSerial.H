/**
   \file  Robots/LoBot/io/LoSerial.H
   \brief Interface for talking to the robot's motor driver.

   lobot's computer sends motor commands to a Sabertooth motor driver via
   a Propeller board that is connected to the computer over a USB serial
   port. This file defines a class that encapsulates the serial
   communications interface using either libserial (if available) or
   INVT's serial communications support (if libserial is not available).
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
// Primary maintainer for this file: Manu Viswanathan <mviswana@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoSerial.H $
// $Id: LoSerial.H 12645 2010-01-25 03:28:29Z mviswana $
//

#ifndef LOBOT_SERIAL_DOT_H
#define LOBOT_SERIAL_DOT_H

//------------------------------ HEADERS --------------------------------

// Headers for serial port communications
#ifdef HAVE_LIBSERIAL

#include <SerialPort.h>

#else // fallback to using INVT serial port support

#include "Devices/Serial.H"
#include "nub/ref.h"

#endif

// INVT model manager stuff
#include "Component/ModelManager.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Serial
   \brief Serial communications API.

   The robot's motors are controlled by a motor driver that is in turn
   controlled by a Propeller-based I/O board. This board connects to the
   Robolocust computer (the mini-ITX Debian box) over a USB port. The
   lobot::Serial class provides an API for interfacing with the Propeller
   using a serial port. It uses libserial for the serial communications
   if that library is available. Otherwise, it falls back to using INVT's
   support for serial communications.
*/
class Serial {
   /// The low-level serial device interface.
#ifdef HAVE_LIBSERIAL
   SerialPort m_serial ;
#else
   nub::ref< ::Serial> m_serial ;
   bool m_peek_flag ;
   char m_peek_char ;
#endif

public:
   /// Initialization.
   Serial(const ModelManager&, const std::string& device, int baud_rate) ;

   /// Check if the serial port has data available for reading.
   bool ready() ;

   /// Receive data from the serial port. Returns the number of bytes
   /// successfully read. Client should check that this is equal to the
   /// expected number.
   int read(char buf[], int n) ;

   /// Send data to the serial port.
   void write(char buf[], int n) ;

   /// Read the specified number of bytes and don't return until they're
   /// all read in.
   void read_full(char buf[], int n) ;

   /// Read and discard the specified number of bytes.
   void eat(int n) ;

   /// Clean-up.
  ~Serial() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
