/*!@file Util/Pause.H simple global pause toggle */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Pause.H $
// $Id: Pause.H 5832 2005-11-01 19:20:52Z rjpeters $
//

#ifndef UTIL_PAUSE_H_DEFINED
#define UTIL_PAUSE_H_DEFINED

//! Change in to or out of the paused state
/*! Note that this actually does nothing itself; all it does is change
    the result of isPaused(). Individual applications can check
    isPaused() in their main loops and act accordingly; or, they
    equally well just ignore isPaused() if they don't care to pause. */
void setPause(bool on);

//! Check if we are in the paused state
bool isPaused();

//! Simple class to help main loops handle pause/unpause events
class PauseWaiter
{
public:
  //! Constructor
  /*! @param usecs how many microseconds to usleep() in each wait cycle */
  PauseWaiter(const int usecs = 50000);

  //! Check if we are in the paused state
  /*! Applications can use the return value to modify the behavior of
      their main loops accordingly. */
  bool checkPause();

private:
  int itsUsecsWait;
  bool itsInPause;
};

//! Check if we are in the paused state, and if so, loop until we become unpaused
/*!
    @return how many wait cycles were spent waiting to become unpaused
 */
int waitForPause(const int usecs = 50000);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_PAUSE_H_DEFINED
