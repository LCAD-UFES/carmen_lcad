/*!@file Transport/CoerceVideoFormatOfilt.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/CoerceVideoFormatOfilt.C $
// $Id: CoerceVideoFormatOfilt.C 9248 2008-02-09 00:29:28Z rjpeters $
//

#ifndef TRANSPORT_COERCEVIDEOFORMATOFILT_C_DEFINED
#define TRANSPORT_COERCEVIDEOFORMATOFILT_C_DEFINED

#include "Transport/CoerceVideoFormatOfilt.H"

#include "Raster/GenericFrame.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"
#include "Video/VideoFormatCoercion.H"
#include "Video/VideoFrame.H"

// ######################################################################
CoerceVideoFormatOfilt::CoerceVideoFormatOfilt(OptionManager& mgr,
                                               VideoFormat mode)
  :
  FrameOfilt(mgr,
             sformat("%s-coercion output filter",
                     convertToString(mode).c_str()),
             sformat("CoerceVideoFormatOfilt(%s)",
                     convertToString(mode).c_str())),
  itsDestFormat(mode)
{}

// ######################################################################
CoerceVideoFormatOfilt::~CoerceVideoFormatOfilt()
{}

// ######################################################################
void CoerceVideoFormatOfilt::start1()
{
  printCoercionTable();
}

// ######################################################################
void CoerceVideoFormatOfilt::filterFrame(FrameOstream& dest,
                                         const GenericFrame& frame,
                                         const std::string& shortname,
                                         const FrameInfo& auxinfo)
{
  const VideoFrame src = frame.asVideo();

  if (src.getMode() == itsDestFormat)
    dest.writeFrame(GenericFrame(src), shortname, auxinfo);
  else
    {
      const VideoFormatCoercion& c =
        findConverter(src.getMode(), itsDestFormat);

      LINFO("converting %s with %s",
            shortname.c_str(), c.describe().c_str());

      const VideoFrame conv = c.apply(src);

      dest.writeFrame(GenericFrame(conv), shortname, auxinfo);
    }
}

// ######################################################################
template <VideoFormat VF>
TCoerceVideoFormatOfilt<VF>::TCoerceVideoFormatOfilt(OptionManager& mgr)
  :
  CoerceVideoFormatOfilt(mgr, VF)
{}

// ######################################################################
template <VideoFormat VF>
TCoerceVideoFormatOfilt<VF>::~TCoerceVideoFormatOfilt()
{}

template class TCoerceVideoFormatOfilt<VIDFMT_GREY>;
template class TCoerceVideoFormatOfilt<VIDFMT_RGB555>;
template class TCoerceVideoFormatOfilt<VIDFMT_RGB565>;
template class TCoerceVideoFormatOfilt<VIDFMT_RGB24>;
template class TCoerceVideoFormatOfilt<VIDFMT_RGB32>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV24>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUYV>;
template class TCoerceVideoFormatOfilt<VIDFMT_UYVY>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV444>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV422>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV411>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV444P>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV422P>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV411P>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV420P>;
template class TCoerceVideoFormatOfilt<VIDFMT_YUV410P>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_COERCEVIDEOFORMATOFILT_C_DEFINED
