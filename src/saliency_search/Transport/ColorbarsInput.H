/*!@file Transport/ColorbarsInput.H FrameIstream class that generates a static "colorbars" test pattern */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/ColorbarsInput.H $
// $Id: ColorbarsInput.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_COLORBARSINPUT_H_DEFINED
#define TRANSPORT_COLORBARSINPUT_H_DEFINED

#include "Transport/FrameIstream.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/SimTime.H"
#include "rutz/time.h"

/// FrameIstream class that generates a static "colorbars" test pattern
class ColorbarsInput : public FrameIstream
{
public:
  /// Constructor
  ColorbarsInput(OptionManager& mgr);

  /// Virtual destructor
  virtual ~ColorbarsInput();

  //! Set the inter-frame interval (or framerate)
  virtual void setConfigInfo(const std::string& frametime);

  //! Advise the FrameIstream object of the current frame number
  virtual bool setFrameNumber(int n);

  /// Peek at the specifications of the next frame to be read
  virtual GenericFrameSpec peekFrameSpec();

  /// Generate a random color image
  virtual GenericFrame readFrame();

private:
  Image<PixRGB<byte> > itsImage;
  SimTime itsFramePeriod;
  int itsFrameNumber;
  rutz::time itsStartTime;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_COLORBARSINPUT_H_DEFINED
