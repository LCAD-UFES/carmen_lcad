/*!@file Util/TransientStatus.H Helper for transient boolean status */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/TransientStatus.H $
// $Id: TransientStatus.H 7392 2006-11-02 21:14:09Z rjpeters $
//

#ifndef UTIL_TRANSIENTSTATUS_H_DEFINED
#define UTIL_TRANSIENTSTATUS_H_DEFINED

#include "Util/log.H"

//! Indicate the on/off/begin/end of some transient
enum TransientStatus
  {
    TSTATUS_OFF,     //!< we are off
    TSTATUS_ON,      //!< we are on
    TSTATUS_BEGIN,   //!< we just switched from off to on
    TSTATUS_END      //!< we just switched from on to off
  };

//! Compute a transient from the steady states oldstate and newstate
inline TransientStatus transientStatus(const bool oldstate,
                                       const bool newstate)
{
  if (oldstate)
    { if (newstate) return TSTATUS_ON; else return TSTATUS_END; }
  else
    { if (newstate) return TSTATUS_BEGIN; else return TSTATUS_OFF; }
}

//! Compute a transient from oldstate and newstate and-masked by mask m
inline TransientStatus transientStatus(const int oldstate,
                                       const int newstate, const int m)
{
  return transientStatus( (oldstate & m) != 0, (newstate & m) != 0 );
}

//! Display a TransientStatus as a 3-char C string
inline const char* transientStatusToStr(const TransientStatus& ts)
{
  switch(ts)
    {
    case TSTATUS_OFF:   return "Off"; break;
    case TSTATUS_ON:    return "On "; break;
    case TSTATUS_BEGIN: return "Beg"; break;
    case TSTATUS_END:   return "End"; break;
    default: LFATAL("Inconsistency in Util/TransientStatus.H");
    }
  return 0; // keep some versions of g++ happy...
}

//! Check whether status is on right now (could be ON or BEGIN)
/*! A return value of true means that it's on right now (either
  because it has been on for a while, or it just began). A return
  value of false DOES NOT mean that it's off, rather it means that it
  is not on. */
inline bool transientStatusIsOn(const TransientStatus ts)
{ return (ts == TSTATUS_ON || ts == TSTATUS_BEGIN); }

//! Check whether status is off right now (could be OFF or END)
/*! A return value of true means that it's off right now (either
  because it has been off for a while, or it just ended). A return
  value of false DOES NOT mean that it's on, rather it means that it
  is not off. */
inline bool transientStatusIsOff(const TransientStatus ts)
{ return (ts == TSTATUS_OFF || ts == TSTATUS_END); }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
