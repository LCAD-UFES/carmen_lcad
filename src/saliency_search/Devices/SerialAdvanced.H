/*!@file Devices/SerialAdvanced.H  class for interfacing with a serial port */

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
// Primary maintainer for this file: Nitin Dhavale <dhavale@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/SerialAdvanced.H $
// $Id: SerialAdvanced.H 11656 2009-09-04 01:25:53Z rand $
//

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

//! enum for different serial errors that could occur
enum serialError
  {
    serialErrSuccess = 0,
    serialErrOpenNoTty,
    serialErrOpenFailed,
    serialErrSpeedInvalid,
    serialErrFlowInvalid,
    serialErrParityInvalid,
    serialErrCharsizeInvalid,
    serialErrStopbitsInvalid,
    serialErrOptionInvalid,
    serialErrResourceFailure,
    serialErrOutput,
    serialErrInput,
    serialErrTimeout,
    serialErrExtended,
    serialErrReadFailed,
    serialErrReadTimedOut,
    serialErrWriteFailed,
    serialErrFcntlFailed,
    serialErrTcGetAttrFailed,
    serialErrTcSetAttrFailed
  };

//! Interface to a serial port
/*! The port will be open at start() time; see ModelComponent.H for
  details. Because typically one may use several serial ports with
  different configs, this ModelComponent does not export any
  command-line option (there would be too many otherwise if many ports
  are used). Typically, thus, the port can be configured either via
  config files or using the various access functions provided in this
  class. */
class SerialAdvanced : public ModelComponent {
public:
  //! Constructor
  SerialAdvanced(OptionManager& mgr, const std::string& descrName = "SerialAdvanced Port",
         const std::string& tagName = "SerialAdvancedPort");

  //! destructor
  ~SerialAdvanced(void);

  //! Configure the port before it is started.
  /*! This will update our internal ModelParam values to the specified
      ones; thus this should be called before start(). In contrast,
      the setXXX functions below are for reconfiguration at runtime,
      once the port is already open and running. This function is
      provided as a shortcut to set a bunch of ModelParam values in
      one simple call.
    @param dev device name (e.g., "/dev/ttyS0")
    @param speed speed in bauds. See setSpeed() for valid values
    @param format a 3-char string for charbits, parity, and stop bits; for
           example "8N1"; parity should be "N", "E" or "O".
    @param flowSoft use software flow control if true
    @param flowHard use hardware flow control if true
    @param tout read timeout in 1/10000s or 0 for no timeout */
  void configure(const char *dev = "/dev/ttyS0", const int speed = 9600,
                 const char *format = "8N1",
                 const bool flowSoft = false, const bool flowHard = true,
                 const int = 0);


  //! Set the DTR mode off momentarily.
  /*! @param millisec number of milliseconds. */
  void toggleDTR(const time_t millisec);

  //! transmit continuous stream of zero-valued bits for specific duration.
  void sendBreak(void);

  //! attempt to read up to nbytes from serial port into the buffer
  /*! @param buffer holds bytes after read
      @nbytes number of bytes to attempt to read
      @return 0 on a timeout, -1 on error, number of bytes actually read
      on a success */
  int read(void* buffer, const int nbytes);

  //! attempts to get the next character from the serial port
  /*!  for comptability with dirk's code  */
  char read(void);

  //! write bytes to the port
  /*! @param buffer begin writing from the location buffer.
      @param nbytes number of bytes to write
      @return -1 on failure, number of bytes written on success */
  int write(const void* buffer, const int nbytes);

  //! print a verbal message indicating the last error.
  void perror(void);

protected:
  //! open the port and get started
  void start2();

  //! close the port and get stopped
  void stop2();

private:
  int dev; // descriptor associated with the device file
  serialError serialErrno; // error id of the last error
  termios savedState; // saved state to restore in the destructor

  // set our errno
  serialError error(const serialError serialErrorNum);

  // open the serial port. This is called from the constructor, so there is
  //    no need to explicitly call this
  // @return -1 on error
  int openPort();

  //! Set serial port speed for both input and output.
  /*! This function is to change the speed while already started.
      @param speed to select. 0 signifies modem "hang up" and possible
             values are 0, 110, 300, 600, 1200, 2400, 4800, 9600,
             19200, 38400, 57600, 115200.
      @return 0 or serialErrSuccess on success. The perror() method
             can be used to print a longer description of the error. */
  serialError setSpeed(const int speed);

  //! Set the serial port flow control on Input as well as Ouput
  /*! This function is to change the flow control while already started.
    @param useHard use hardware flow control if true
    @param useSoft use software flow control if true
    @return 0 or serialErrSuccess on success */
  serialError setFlowControl(const bool useHard, const bool useSoft);

  //! Set character size.
  /*! This function is to change the char size while already started.
    @return 0 on success.
    @param bits character size to use; valid values are 5, 6, 7, 8. */
  serialError setCharBits(const int bits);

  //! Set parity mode.
  /*! This function is to change the parity while already started.
    @param useParity use some parity if true
    @param oddParity parity is odd if true, otherwise even
    @return 0 on success. */
  serialError setParity(const bool useParity, const bool oddParity);

  //! Set number of stop bits.
  /*! This function is to change the stop bits while started.
    @return 0 on success.
    @param bits stop bits (only values 1 and 2 are valid). */
  serialError setStopBits(const int bits);

  //! Set the access to blocking or not
  /*! If blocking, a write will block until all data has actually been
    sent through; otherwise it will return immediately while the data
    is being transmitted. */
  serialError setBlocking(const bool blocking);

  OModelParam<std::string> itsDevName;   //! device name
  OModelParam<int>         itsBaud;      //!< Baudrate
  OModelParam<int>         itsCharBits;  //!< number of useful bits per char
  OModelParam<int>         itsStopBits;  //!< number of stop bits
  OModelParam<bool>        itsUseParity; //!< use parity if true
  OModelParam<bool>        itsParityOdd; //!< parity is odd if true, otherwise even
  OModelParam<bool>        itsFlowHard;  //!< use hardware flow control if true
  OModelParam<bool>        itsFlowSoft;  //!< use software flow control if true
  OModelParam<int>         itsRdTout;    //!< read timeout in 1/10000s
  OModelParam<bool>        itsBlocking;  //!< use blocking mode

  OModelParam<std::string> itsDevSearchDescriptor;   //!< A string to search for on all serial ports

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
