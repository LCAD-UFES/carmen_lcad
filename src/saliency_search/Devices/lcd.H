/*!@file Devices/lcd.H Interface to an LCD screen via serial port */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/lcd.H $
// $Id: lcd.H 6522 2006-04-25 16:41:24Z rjpeters $
//

#ifndef LCD_H_DEFINED
#define LCD_H_DEFINED

#include "Component/ModelComponent.H"
#include "Devices/Serial.H"
#include "Util/Types.H"

#include <stdarg.h>

//! lcd.H Interface to an LCD screen via serial port
/*! LCD Class. Contains interfaces to display to a LCD connected via a
 serial port. If you need more than a serial port to connect to an
 LCD, that LCD must be very special.  Note that the code is made for
 Backpack LCDs by SEETRON. */

class lcd : public ModelComponent
{
public:
  //! Default constructor. See ModelComponent.H
  lcd(OptionManager& mgr,
      const std::string& descrName = "Alphanumeric LCD Display",
      const std::string& tagName = "lcd");

  //! Destructor
  ~lcd();

  //! print some text starting at given cursor position
  /*! syntax is the same as printf(), plus the x,y
    coordinates. Returns true on success. */
  bool printf(const int x, const int y, const char *fmt, ...)
    // NOTE: this __attribute__ tells gcc that it should issue
    // printf-style warnings when compiling calls to lcd::printf(),
    // treating the 3rd argument (fmt) as the format string, and the
    // 4th and subsequent arguments as the printf-style parameters
    // (SUBNOTE: because this is a member function, there is a hidden
    // 'this' parameter that counts as arg 1, so the listed arguments
    // are counted starting from 2)
    __attribute__((format(__printf__, 4, 5)));
    ;

  //! Takes a string and outputs to LCD
  bool writeString(const char *s);

  //! Clears LCD screen
  bool clear();

  //! Scrolls display left by i positions
  bool scrollLeft(const int i);

  //! Scrolls display right by i positions
  bool scrollRight(const int i);

  //! Moves cursor to start of screen
  bool home();

  //! Moves cursor left by i positions
  bool moveCursorLeft(const int i);

  //! Moves cursor right by i positions
  bool moveCursorRight(const int i);

  //! Makes cursor a blinking block
  bool blinkBlock();

  //! Makes cursor a blinking underline
  bool blinkUnderline();

  //! Makes cursor invisible (not recommended)
  bool invisibleCursor();

  //! Sets cursor position to i
  bool setCursorPosition(const int i);

protected:
  nub::soft_ref<Serial> itsPort; //!< our serial port
  NModelParam<int> itsRows;   //!< number of rows (lines of text)
  NModelParam<int> itsCols;   //!< number of columns

private:
  //! send a command
  bool writeCommand(const byte cmd);
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
