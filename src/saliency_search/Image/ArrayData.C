/*!@file Image/ArrayData.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ArrayData.C $
// $Id: ArrayData.C 5277 2005-08-10 01:38:57Z rjpeters $
//

#ifndef IMAGE_ARRAYDATA_C_DEFINED
#define IMAGE_ARRAYDATA_C_DEFINED

#include "Image/ArrayData.H"

#include "Util/Assert.H" // for ASSERT()
#include "Util/log.H"

void check_acquisition(StoragePolicy s, int count) throw()
{
  // see comment at Image<T>::deepcopy() for a discussion of why we
  // forbid shared objects with a StoragePolicy of WRITE_THRU
  if (s == WRITE_THRU && count > 1)
    {
      LERROR("attach()'ed arrays cannot be shared\n"
             "\tNOTE: consider using Image<T>::deepcopy()");
      // note that we do a hard abort here with ASSERT(0) rather than
      // doing an LFATAL(), because this function is declared as
      // no-throw, whereas LFATAL() would throw an exception and break
      // our no-throw spec
      ASSERT(0);
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_ARRAYDATA_C_DEFINED
