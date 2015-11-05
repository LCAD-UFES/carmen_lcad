/*!@file Transport/LuminanceOfilt.H Output filter that converts rgb images to their grayscale luminance */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/LuminanceOfilt.H $
// $Id: LuminanceOfilt.H 8594 2007-07-19 23:28:46Z rjpeters $
//

#ifndef TRANSPORT_LUMINANCEOFILT_H_DEFINED
#define TRANSPORT_LUMINANCEOFILT_H_DEFINED

#include "Transport/FrameOfilt.H"

/// Output filter that converts rgb images to their grayscale luminance
class LuminanceOfilt : public FrameOfilt
{
public:
  /// Constructor
  LuminanceOfilt(OptionManager& mgr,
                 const std::string& descrName = "RGB-to-luminance output filter",
                 const std::string& tagName = "LuminanceOfilt");

  /// Destructor
  virtual ~LuminanceOfilt();

protected:
  /// Convert frame to grayscale luminance and pass on to dest
  virtual void filterFrame(FrameOstream& dest,
                           const GenericFrame& frame,
                           const std::string& shortname,
                           const FrameInfo& auxinfo);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_LUMINANCEOFILT_H_DEFINED
