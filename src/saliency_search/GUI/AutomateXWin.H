/*!@file GUI/AutomateXWin.H Automate X windows by sending keys and getting images */


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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/AutomateXWin.H $
// $Id: AutomateXWin.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef AUTOMATEXWIN_H_DEFINED
#define AUTOMATEXWIN_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xmu/Xmu.h>
#include <unistd.h>
#include <X11/extensions/XTest.h>

#define USE_SHM

#ifdef USE_SHM
#include <sys/ipc.h>
#include <sys/shm.h>
#include <X11/extensions/XShm.h>
#endif


class AutomateXWin
{
public:

  //!connect to the x window display and select the given window name
  AutomateXWin(const char* win_name);

  //! destroy the window and stop the event handling
  virtual ~AutomateXWin();


  //!set the focus to this window
  void setFocus();

  //!Get the image of the window
  Image<PixRGB<byte> > getImage();

  //!Delete the image buffer
  void DeleteImage(void);

  //!send the given key to the window
  void sendKey(const int key);

  int getHighBitIndex (unsigned int x);
  void GetWindowGeometry();
  Window XWindowByName(Display *display, const Window rootwin, const char *name);
  void XListWindows(Display *display, const Window rootwin);

private:
  Display *itsDisplay;
  int itsScreen;
  Window itsRootWin, itsWindow;
  XVisualInfo itsVInfo;

  XImage *itsImage;

#ifdef USE_SHM
  XShmSegmentInfo itsShminfo;
#endif

  unsigned int itsWidth;
  unsigned int itsHeight;
  unsigned int itsDepth;

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
