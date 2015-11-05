/*!@file Transport/ShiftedImage.H A FrameIstream subclass for
  an image shifted in time to display a clean FOE or planar motion */

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
// Primary maintainer for this file: Christian Siagian <siagian at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/ShiftedImage.H $
// $Id: $
//

#ifndef TRANSPORT_SHIFTEDIMAGE_H_DEFINED
#define TRANSPORT_SHIFTEDIMAGE_H_DEFINED

#include "Image/Dims.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"

#include "Transport/FrameIstream.H"
#include "rutz/rand.h"

//! FrameIstream subclass that generates random images
/*! This can be a handy way to quickly test an executable, 
  with an image that will be doubled and shifted to emulate egomotion.
  For example: focus of expansion:
  --in=shiftedImage:path/image:foe on the command-line to configure your
  InputFrameSeries to use a ShiftedImage source. */
class ShiftedImage : public FrameIstream
{
public:
  //! Constructor
  ShiftedImage(OptionManager& mgr);

  //! Virtual destructor
  virtual ~ShiftedImage();

  //! Override from FrameIstream; just calls setDims()
  virtual void setConfigInfo(const std::string& dimsstring);

  //! Peek at the specifications of the next frame to be read
  virtual GenericFrameSpec peekFrameSpec();

  //! Generate a random color image
  virtual GenericFrame readFrame();

private:
  void setConfigPlanar(const std::string& params);
  void setConfigFoe(const std::string& params);
  void setConfigRotation(const std::string& params);

  Image<PixRGB<byte> > getPlanarMotionStimuli(uint step);
  Image<PixRGB<byte> > getFoeStimuli(uint step);
  Image<PixRGB<byte> > getRotationMotionStimuli(uint step);

  bool itsIsPlanar;
  float itsDx;
  float itsDy;

  bool itsIsFoe;
  Point2D<int> itsFoe;

  bool itsIsRotation;
  Point2D<int> itsRotationCenter;

  uint                 itsTotalSteps;
  uint                 itsStep;
  Image<PixRGB<byte> > itsImage;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RANDOMINPUT_H_DEFINED
