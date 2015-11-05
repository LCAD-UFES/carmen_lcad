/*!@file Image/CheckedIterator.C A range-checked iterator */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/CheckedIterator.C $
// $Id: CheckedIterator.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Image/CheckedIterator.H"

#include "Util/Assert.H"
#include "Util/log.H"

#include <cstdio>

void CheckedIteratorAux::ck_range_helper(const void* vptr, size_t voffset,
                                         const void* vstart, const void* vstop,
                                         size_t size)
{
  const char*  ptr    = static_cast<const char*>(vptr);
  const char*  start  = static_cast<const char*>(vstart);
  const char*  stop   = static_cast<const char*>(vstop);
  const size_t offset = voffset * size;

  if (!(ptr+offset >= start) ||
      !(ptr+offset < stop))
    {
      LINFO("[checked iterator violation] ptr:        %p", ptr);
      LINFO("[checked iterator violation] size:       %" ZU , size);
      LINFO("[checked iterator violation] offset:     %" ZU , offset);
      LINFO("[checked iterator violation] ptr+offset: %p", ptr+offset);
      LINFO("[checked iterator violation] start:      %p", start);
      LINFO("[checked iterator violation] stop:       %p", stop);
    }

  ASSERT(ptr+offset >= start); ASSERT(ptr+offset < stop);
}
