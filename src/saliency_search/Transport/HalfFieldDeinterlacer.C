/*!@file Transport/HalfFieldDeinterlacer.C "Deinterlace" frames by always returning the top half-field */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/HalfFieldDeinterlacer.C $
// $Id: HalfFieldDeinterlacer.C 14290 2010-12-01 21:44:03Z itti $
//

#ifndef TRANSPORT_HALFFIELDDEINTERLACER_C_DEFINED
#define TRANSPORT_HALFFIELDDEINTERLACER_C_DEFINED

#include "Transport/HalfFieldDeinterlacer.H"

#include "Raster/GenericFrame.H"
#include "Video/VideoFrame.H"
#include "rutz/trace.h"

// ######################################################################
template <bool BottomField>
HalfFieldDeinterlacer<BottomField>::HalfFieldDeinterlacer(OptionManager& mgr)
  :
  Deinterlacer(mgr)
{}

// ######################################################################
template <bool BottomField>
HalfFieldDeinterlacer<BottomField>::~HalfFieldDeinterlacer()
{}

// ######################################################################
template <bool BottomField>
GenericFrameSpec HalfFieldDeinterlacer<BottomField>::peekFrameSpec()
{
  GenericFrameSpec result = getDelegate().peekFrameSpec();

  switch (result.nativeType)
    {
    case GenericFrame::NONE:
      ASSERT(0);
      break;

    case GenericFrame::RGB_U8:
    case GenericFrame::RGBD:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_RGB24;
      result.videoByteSwap = false;
      break;

    case GenericFrame::RGB_F32:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_RGB24;
      result.videoByteSwap = false;
      break;

    case GenericFrame::GRAY_U8:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_GREY;
      result.videoByteSwap = false;
      break;

    case GenericFrame::GRAY_F32:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_GREY;
      result.videoByteSwap = false;
      break;

    case GenericFrame::RGB_U16:
      break;

    case GenericFrame::GRAY_U16:
      break;

    case GenericFrame::VIDEO:
      // no changes
      break;
    }

  return result;
}

// ######################################################################
template <bool BottomField>
bool HalfFieldDeinterlacer<BottomField>::setFrameNumber(int n)
{
  return getDelegate().setFrameNumber(n);
}

// ######################################################################
template <bool BottomField>
void HalfFieldDeinterlacer<BottomField>::startStream()
{
  getDelegate().startStream();
}

// ######################################################################
template <bool BottomField>
GenericFrame HalfFieldDeinterlacer<BottomField>::readFrame()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const VideoFrame raw = getDelegate().readFrame().asVideo();
  if (!raw.initialized())
    // ok, we got an empty frame from our delegate (probably means the
    // stream is exhausted), so just return an empty frame
    return GenericFrame();
  return GenericFrame(raw.makeBobDeinterlaced(BottomField));
}

template class HalfFieldDeinterlacer<true>;
template class HalfFieldDeinterlacer<false>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_HALFFIELDDEINTERLACER_C_DEFINED
