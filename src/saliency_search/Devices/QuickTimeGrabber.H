/*!@file Devices/QuickTimeGrabber.H Grab frames (e.g. from a camera) using QuickTime's SequenceGrabber APIs */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/QuickTimeGrabber.H $
// $Id: QuickTimeGrabber.H 8000 2007-02-27 07:57:37Z rjpeters $
//

#ifndef DEVICES_QUICKTIMEGRABBER_H_DEFINED
#define DEVICES_QUICKTIMEGRABBER_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Transport/FrameIstream.H"

/// Grab frames (e.g. from a camera) using QuickTime's SequenceGrabber APIs
class QuickTimeGrabber : public FrameIstream
{
public:
  /// Constructor
  QuickTimeGrabber(OptionManager& mgr,
                   const std::string& descrName = "QuickTime Frame Grabber",
                   const std::string& tagName = "QuickTimeGrabber");

  /// Virtual destructor
  virtual ~QuickTimeGrabber();

  /// Prepare to start grabbing streams with SGPrepare() and SGStartRecord()
  virtual void startStream();

  /// Return the specifications of the next frame to be returned
  virtual GenericFrameSpec peekFrameSpec();

  /// Grab the next frame from the camera
  virtual GenericFrame readFrame();

private:
  virtual void start1();

  virtual void stop2();

  OModelParam<Dims> itsDims;

  struct Impl;
  Impl* rep;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_QUICKTIMEGRABBER_H_DEFINED
