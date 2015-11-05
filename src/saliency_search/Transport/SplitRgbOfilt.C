/*!@file Transport/SplitRgbOfilt.C Output filter that splits rgb images into separate r/g/b components */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/SplitRgbOfilt.C $
// $Id: SplitRgbOfilt.C 8053 2007-03-06 23:23:20Z rjpeters $
//

#ifndef TRANSPORT_SPLITRGBOFILT_C_DEFINED
#define TRANSPORT_SPLITRGBOFILT_C_DEFINED

#include "Transport/SplitRgbOfilt.H"

#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"

// ######################################################################
SplitRgbOfilt::SplitRgbOfilt(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName)
  :
  FrameOfilt(mgr, descrName, tagName)
{}

// ######################################################################
SplitRgbOfilt::~SplitRgbOfilt()
{}

// ######################################################################
void SplitRgbOfilt::filterFrame(FrameOstream& dest,
                                const GenericFrame& frame,
                                const std::string& shortname,
                                const FrameInfo& auxinfo)
{
  const Image<PixRGB<byte> > rgb = frame.asRgb();

  Image<byte> r, g, b;
  getComponents(rgb, r, g, b);

  dest.writeFrame(GenericFrame(r),
                  shortname + "-r",
                  FrameInfo(auxinfo.description + " (red component)",
                            auxinfo.srcpos));

  dest.writeFrame(GenericFrame(g),
                  shortname + "-g",
                  FrameInfo(auxinfo.description + " (green component)",
                            auxinfo.srcpos));

  dest.writeFrame(GenericFrame(b),
                  shortname + "-b",
                  FrameInfo(auxinfo.description + " (blue component)",
                            auxinfo.srcpos));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_SPLITRGBOFILT_C_DEFINED
