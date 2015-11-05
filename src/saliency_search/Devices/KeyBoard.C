/*!@file Devices/KeyBoard.C a simple keyboard interface */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Naila Rizvi <nrizvi@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/KeyBoard.C $
// $Id: KeyBoard.C 13695 2010-07-25 18:19:38Z dberg $
//

#include "Devices/KeyBoard.H"

#include "Util/log.H"
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// ######################################################################
KeyBoard::KeyBoard()
{
  // switch keyboard to non-canonical mode:
  struct termios new_settings;
  if (tcgetattr(0, &stored_settings) == -1)
    PLERROR("Cannot tcgetattr");
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  new_settings.c_cc[VMIN] = 1;
  if (tcsetattr(0, TCSANOW, &new_settings) == -1)
    PLERROR("Cannot tcsetattr");

  // by default, assume keyboard is blocking:
  blocking = true;
}

// ######################################################################
KeyBoard:: ~KeyBoard()//Destructor
{
  // restore blocking mode:
  setBlocking(true);

  // restore previous keyboard attributes:
  if (tcsetattr(0, TCSANOW, &stored_settings) == -1)
    PLERROR("Cannot tcsetattr");
}

// ######################################################################
void KeyBoard::setBlocking(const bool block)
{
  int flags = fcntl(0, F_GETFL, 0);
  if (flags == -1) PLERROR("Cannot get flags");
  if (block) flags &= (~O_NONBLOCK); else flags |= O_NONBLOCK;
  if (fcntl(0, F_SETFL, flags) == -1) PLERROR("Cannot set flags");
  else blocking = block; // remember setting
}

// ######################################################################
KeyBoardKey KeyBoard::getKey(const bool block)
{
  // make sure we are in the correct blocking/non-blocking mode:
  if (blocking != block) setBlocking(block);
  
  // get the key & return:
  int ch = getc(stdin);
  if (ch == EOF) return KBD_NONE;
  return fromChar(getc(stdin));
}

// ######################################################################
int KeyBoard::getKeyAsChar(const bool block)
{
  // make sure we are in the correct blocking/non-blocking mode:
  if (blocking != block) setBlocking(block);
  
  // get the key & return:
  int ch = getc(stdin);
  return ch;
}

// ######################################################################
KeyBoardKey KeyBoard::fromChar(const char c) const
{
  switch(c)
    {
    case 'a': return KBD_KEY1;
    case 'b': return KBD_KEY2;
    case 'c': return KBD_KEY3;
    case 'd': return KBD_KEY4;
    case 'e': return KBD_KEY5;
    default: return KBD_OTHER;
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
