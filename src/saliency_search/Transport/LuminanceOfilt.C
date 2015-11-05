/*!@file Transport/LuminanceOfilt.C Output filter that converts rgb images to their grayscale luminance */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/LuminanceOfilt.C $
// $Id: LuminanceOfilt.C 8594 2007-07-19 23:28:46Z rjpeters $
//

#ifndef TRANSPORT_LUMINANCEOFILT_C_DEFINED
#define TRANSPORT_LUMINANCEOFILT_C_DEFINED

#include "Transport/LuminanceOfilt.H"

#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"

// ######################################################################
LuminanceOfilt::LuminanceOfilt(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName)
  :
  FrameOfilt(mgr, descrName, tagName)
{}

// ######################################################################
LuminanceOfilt::~LuminanceOfilt()
{}

// ######################################################################
void LuminanceOfilt::filterFrame(FrameOstream& dest,
                                 const GenericFrame& frame,
                                 const std::string& shortname,
                                 const FrameInfo& auxinfo)
{
  const GenericFrame::NativeType t = frame.nativeType();

  if (t == GenericFrame::RGB_U8 ||
      (t == GenericFrame::VIDEO
       && frame.asVideo().getMode() != VIDFMT_GREY))
    {
      const Image<byte> lum = luminance(frame.asRgb());
      dest.writeFrame(GenericFrame(lum),
                      shortname + "-lum",
                      FrameInfo(auxinfo.description + " (luminance)",
                                auxinfo.srcpos));
    }
  else if (t == GenericFrame::RGB_F32)
    {
      const Image<float> lum = luminance(frame.asRgbF32());
      dest.writeFrame(GenericFrame(lum, frame.floatFlags()),
                      shortname + "-lum",
                      FrameInfo(auxinfo.description + " (luminance)",
                                auxinfo.srcpos));
    }
  else if (t == GenericFrame::NONE
           || t == GenericFrame::GRAY_U8
           || t == GenericFrame::GRAY_F32
           || (t == GenericFrame::VIDEO
               && frame.asVideo().getMode() == VIDFMT_GREY))
    {
      // it's already greyscale, so just pass the frame through as-is
      dest.writeFrame(frame, shortname, auxinfo);
    }
  else
    LFATAL("Oops! Don't know how to generate luminance of %s image (%s)",
           frame.frameSpec().getDescription().c_str(),
           shortname.c_str());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_LUMINANCEOFILT_C_DEFINED
