/*!@file Neuro/BeoHeadBrain.H A brain for the beo head */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/BeoHeadBrain.C $
// $Id: BeoHeadBrain.C 12782 2010-02-05 22:14:30Z irock $
//

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include "Neuro/BeoHeadBrain.H"

#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Util/JobWithSemaphore.H"

//preformace simple rotines like getting image from eyes and tracking
namespace
{
  class preBrainLoop : public JobWithSemaphore
  {
    public:
      preBrainLoop(BeoHeadBrain* neoBrain)
        :
          itsBeoHeadBrain(neoBrain),
          itsPriority(1),
          itsJobType("PreBrainLoop")
    {}

      virtual ~preBrainLoop() {}

      virtual void run()
      {
        ASSERT(itsBeoHeadBrain);
        while(1)
        {
          itsBeoHeadBrain->updateEyesImage();
          itsBeoHeadBrain->trackObjects(); //should be one thread for left and right eyes
          usleep(1000);
        }
      }

      virtual const char* jobType() const
      { return itsJobType.c_str(); }

      virtual int priority() const
      { return itsPriority; }

    private:
      BeoHeadBrain* itsBeoHeadBrain;
      const int itsPriority;
      const std::string itsJobType;
  };
}

// ######################################################################
BeoHeadBrain::BeoHeadBrain(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName)
  :
  NeoBrain(mgr, descrName, tagName)
{
  itsLeftEye = nub::ref<InputFrameSeries>(new InputFrameSeries(mgr));
  addSubComponent(itsLeftEye);
}

// ######################################################################
BeoHeadBrain::~BeoHeadBrain()
{}

// ######################################################################
void BeoHeadBrain::initHead()
{
  //setup pid loop thread
  itsThreadServer.reset(new WorkThreadServer("preBrainLoop",1)); //start a single worker thread
  itsThreadServer->setFlushBeforeStopping(false);
  rutz::shared_ptr<preBrainLoop> j(new preBrainLoop(this));
  itsThreadServer->enqueueJob(j);

  Dims imageDims = itsLeftEye->peekDims();
  init(imageDims);
}

void BeoHeadBrain::updateEyesImage()
{
  const FrameState is = itsLeftEye->updateNext();
  if (is == FRAME_COMPLETE)
    return;

  //grab the images
  GenericFrame input = itsLeftEye->readFrame();
  if (!input.initialized())
    return;

  itsCurrentLeftEyeImg = input.asRgb();
}

Image<PixRGB<byte> >  BeoHeadBrain::getLeftEyeImg()
{
  return itsCurrentLeftEyeImg;
}

Image<PixRGB<byte> >  BeoHeadBrain::getRightEyeImg()
{
  return itsCurrentRightEyeImg;
}

// ######################################################################
void BeoHeadBrain::setTarget(const Point2D<int> loc)
{
  this->NeoBrain::setTarget(loc, luminance(itsCurrentLeftEyeImg), -1, false);
}

// ######################################################################
void BeoHeadBrain::trackObjects()
{
  itsCurrentTargetLoc = this->trackObject(luminance(itsCurrentLeftEyeImg));
}

// ######################################################################
Point2D<int> BeoHeadBrain::getTargetLoc()
{
  return itsCurrentTargetLoc;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
