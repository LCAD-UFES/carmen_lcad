/*!@file Util/fpe.H floating-point exceptions */

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
// Primary maintainer for this file: Rob Peters <rjpeters@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/fpe.H $
// $Id: fpe.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef FPE_H_DEFINED
#define FPE_H_DEFINED


/*
  Here's a state diagram for turning floating-point exceptions on/off
  and locked/unlocked. In this diagram "fpe" is an abbreviation for
  "fpExceptions" in the functions declared below.

  *
  *                _____ fpeOff()  fpeOff() _____
  *               /no-op\ fpeOn()  fpeOn() /no-op\
  *              /       \                /       \
  * .===============.    /                \    .================.
  * | FPE-LOCKED-ON | <-'                  `-> | FPE-LOCKED-OFF |
  * '==============='                          '================'
  *           ^  |                                        ^  |
  * fpeLock() |  |                              fpeLock() |  |
  *           |  |                                        |  |
  *           |  | fpeUnlock()                            |  | fpeUnlock()
  *           |  v                fpeOn()                 |  v
  * .===============.  <<-------------------   .================.
  * |    FPE-ON     |                          |    FPE-OFF     |
  * '==============='   ------------------->>  '================'
  *                         fpeOff()

*/

//! Turn floating-point exceptions on
void fpExceptionsOn();

//! Turn floating-point exceptions off
void fpExceptionsOff();

//! Lock floating-point exceptions in their current state.
/*! That is, if FPEs are on now, they will stay locked on even if
  someone calls fpExceptionsOff(); likewise, if FPEs are off now,
  they will stay locked off even if someone calls
  fpExceptionsOn(). */
void fpExceptionsLock();

//! Unlock floating-point exceptions.
/*! This does not turn FPEs on or off, but does make it possible for
  the state to be changed again via fpExceptionsOn() or
  fpExceptionsOff(). */
void fpExceptionsUnlock();

//! Check whether floating-point exceptions are on
bool fpIsEnabled();

//! Check whether floating-point exceptions are locked in their current state
bool fpIsLocked();


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !FPE_H_DEFINED
