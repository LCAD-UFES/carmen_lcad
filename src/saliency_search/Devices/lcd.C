/*!@file Devices/lcd.C Interface to an LCD screen via serial port */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/lcd.C $
// $Id: lcd.C 6100 2006-01-17 01:59:41Z rjpeters $
//

#include "Devices/lcd.H"

#include "rutz/compat_snprintf.h"

// character byte sequences for non-printing functions
//
// Clear screen              : <254> <1>
// Scroll left               : <254> <24>
// Scroll right              : <254> <28>
// Move cursor to top left   : <254> <2>
// Move cursor left             : <254> <16>
// Move cursor right             : <254> <20>
// Make cursor underline     : <254> <14>
// Make blink-block cursor   : <254> <13>
// Make cursor invisible     : <254> <12>
// Set cursor position             : <254> <128 + position>

//! Char to be sent to gain access to special functions:
#define LCD_FUNCTION 254

// ######################################################################
lcd::lcd(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr)),
  itsRows("LCDrows", this, 4),
  itsCols("LCDcols", this, 20)
{
  // set a default config for our serial port:
  itsPort->configure("/dev/ttyS0", 9600, "8N1", false, false, 1);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);
}

// ######################################################################
lcd::~lcd()
{ }

// ######################################################################
bool lcd::printf(const int x, const int y, const char *fmt, ...)
{
  if (itsRows.getVal() != 4)
    LFATAL("FIXME: need some code to support various numbers of rows!");

  // figure out our starting offset:
  int offset = x;
  switch(y)
    {
    case 0: break;
    case 1: offset += 64; break;
    case 2: offset += itsCols.getVal(); break;
    case 3: offset += 64 + itsCols.getVal(); break;
    default: LFATAL("Row number %d out of range 0..3", y);
    }

  // get to that position:
  if (setCursorPosition(offset) == false) return false;

  // format and write out our message, truncating at our number of columns:
  char txt[itsCols.getVal() + 1];
  va_list a; va_start(a, fmt);
  vsnprintf(txt, itsCols.getVal()+1, fmt, a);
  va_end(a);
  return writeString(txt);
}

// ######################################################################
bool lcd::writeString(const char *s)
{ return (itsPort->write(s, strlen(s)) == int(strlen(s))); }

// ######################################################################
bool lcd::clear()
{ return writeCommand(1); }

// ######################################################################
bool lcd::scrollLeft(const int i)
{
  // uses for loop to scroll sequentially:
  for (int j = 0; j < i; j ++)
    if (writeCommand(24) == false) return false;
  return true;
}

// ######################################################################
bool lcd::scrollRight(const int i)
{
  // uses for loop to scroll sequentially:
  for (int j = 0; j < i; j ++)
    if (writeCommand(28) == false) return false;
  return true;
}

// ######################################################################
bool lcd::home()
{ return writeCommand(2); }

// ######################################################################
bool lcd::moveCursorLeft(const int i)
{
  // uses for loop to move sequentially:
  for (int j = 0; j < i; j ++)
    if (writeCommand(16) == false) return false;
  return true;
}

// ######################################################################
bool lcd::moveCursorRight(const int i)
{
  // uses for loop to move sequentially:
  for (int j = 0; j < i; j ++)
    if (writeCommand(20) == false) return false;
  return true;
}

// ######################################################################
bool lcd::blinkBlock()
{ return writeCommand(13); }

// ######################################################################
bool lcd::blinkUnderline()
{ return writeCommand(14); }

// ######################################################################
bool lcd::invisibleCursor()
{ return writeCommand(12); }

// ######################################################################
bool lcd::setCursorPosition(const int i)
{
  if (i < 0 || i > 127) { LERROR("Invalid position %d", i); return false; }
  return writeCommand(128 + i);
}

// ######################################################################
bool lcd::writeCommand(const byte cmd)
{
  byte s[2]; s[0] = LCD_FUNCTION; s[1] = cmd;
  if (itsPort->write((char *)s, 2) != 2) return false;
  return true;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
