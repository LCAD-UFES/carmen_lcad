/*!@file Neuro/BeoHeadBrain.H A brain for the beo head */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/BeoHeadBrain.H $
// $Id: BeoHeadBrain.H 9974 2008-07-25 01:23:47Z icore $
//

#ifndef BEOHEADBRAIN_H_DEFINED
#define BEOHEADBRAIN_H_DEFINED

#include "Media/FrameSeries.H"
#include "Neuro/NeoBrain.H"
#include "Util/WorkThreadServer.H"

class BeoHeadBrain : public NeoBrain
{
public:
  //! Constructor
  BeoHeadBrain(OptionManager& mgr,
               const std::string& descrName = "BeoHeadBrain",
               const std::string& tagName = "BeoHeadBrain");

  virtual ~BeoHeadBrain();

  void initHead(); //start the input frame and tracking thread

  void updateEyesImage(); //read from frame input and set the eyets image

  Image<PixRGB<byte> > getLeftEyeImg();
  Image<PixRGB<byte> > getRightEyeImg();

  // pull in other overloads of setTarget() from NeoBrain
  using NeoBrain::setTarget;

  void setTarget(Point2D<int> loc);

  void trackObjects();
  Point2D<int> getTargetLoc();

private:
  nub::soft_ref<InputFrameSeries> itsLeftEye;

  Image<PixRGB<byte> > itsCurrentLeftEyeImg;
  Image<PixRGB<byte> > itsCurrentRightEyeImg;

  Point2D<int> itsCurrentTargetLoc;

  rutz::shared_ptr<WorkThreadServer> itsThreadServer;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
