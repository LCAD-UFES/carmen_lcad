/*!@file Util/fpe.C floating-point exceptions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/fpe.C $
// $Id: fpe.C 6132 2006-01-20 22:59:25Z rjpeters $
//

#include "Util/fpe.H"

#ifdef HAVE_FEENABLEEXCEPT
#include <fenv.h>
#endif

namespace
{
  bool isEnabled = false;
  bool isLocked = false;
}

// ######################################################################
void fpExceptionsOn()
{
  if (!isLocked)
    {
      // Removed: FE_UNDERFLOW as it crashes several classes incl. WinnerTakeAll
#ifdef HAVE_FEENABLEEXCEPT
      feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
      isEnabled = true;
    }
}

// ######################################################################
void fpExceptionsOff()
{
  if (!isLocked)
    {
#ifdef HAVE_FEENABLEEXCEPT
      feclearexcept(FE_ALL_EXCEPT);
      fedisableexcept(FE_ALL_EXCEPT);
#endif
      isEnabled = false;
    }
}

// ######################################################################
void fpExceptionsLock()
{
  isLocked = true;
}

// ######################################################################
void fpExceptionsUnlock()
{
  isLocked = false;
}

// ######################################################################
bool fpIsEnabled()
{
  return isEnabled;
}

// ######################################################################
bool fpIsLocked()
{
  return isLocked;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
