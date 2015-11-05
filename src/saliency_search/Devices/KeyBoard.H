/*!@file Devices/KeyBoard.H a simple keyboard interface */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/KeyBoard.H $
// $Id: KeyBoard.H 13695 2010-07-25 18:19:38Z dberg $
//

#ifndef KEYBOARD_H_DEFINED
#define KEYBOARD_H_DEFINED

#include <termios.h>

//! Possible keys returned by our access methods
enum KeyBoardKey {
  KBD_NONE, KBD_KEY1, KBD_KEY2, KBD_KEY3, KBD_KEY4, KBD_KEY5, KBD_OTHER
};


//! Simple interface to a keyboard
/*! This class provides basic methods to check whether a key was
  pressed on the keyboard. It is used by the Beobot, which has a small
  keypad with just 5 keys. */
class KeyBoard
{
public:
  //! Constructor
  KeyBoard();

  //! Destructor
  ~KeyBoard();

  //! Get a key from the keyboard
  /*! @param block if true, this function will block until a key is
    pressed. Otherwise, it will return the key pressed if any, or
    KBD_NONE if no key was pressed. */
  KeyBoardKey getKey(const bool block);
  
  //! get the key from the keybard and return as an int
  /*! @param block if true, this function will block until a key is
    pressed. */
  int getKeyAsChar(const bool block);
  
private:
  // Enable/disables blocking mode
  void setBlocking(const bool block);

  // convert from char to KeyBoardKey
  KeyBoardKey fromChar(const char c) const;

  struct termios stored_settings;
  bool blocking;  // keep track of whether we are blocking or not
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
