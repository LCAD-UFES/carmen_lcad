/*!@file Image/VisualTracker.H Interface to VisualTracker */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/VisualTracker.H $
// $Id: VisualTracker.H 13551 2010-06-10 21:56:32Z itti $
//

#ifndef VisualTracker_H_DEFINED
#define VisualTracker_H_DEFINED

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Simulation/SimEvents.H"
#include "Simulation/SimModule.H"
#include "Image/Image.H"
#include "Media/MediaSimEvents.H"
#include "Channels/InputFrame.H"

class  SimEventSetVisualTracker;

class VisualTracker : public SimModule
{
public:

  //Constructor
  VisualTracker(OptionManager& mgr,
          const std::string& descrName = "VisualTracker",
          const std::string& tagName = "VisualTracker");

  ~VisualTracker();

  void start2();

  //! initalize the tracker
  void initTracker(Dims imageDims);

  //! Set the target point to track
  void setTargets(const Image<byte>& grey, const Point2D<int> loc);

  //! Set the target as a region to track
  void setTargets(const Image<byte>& grey, const Image<byte>& target);

  //! Get the location of track points
  Point2D<int> trackObjects(const Image<byte>& grey);

  //! Get the location of track objects
  Point2D<int> trackTemplObject(const Image<byte>& grey, double &err);

protected:
  //! Callback for when a new input frame is available
  SIMCALLBACK_DECLARE(VisualTracker, SimEventInputFrame);

  SIMCALLBACK_DECLARE(VisualTracker, SimEventSetVisualTracker);

private:

#ifdef HAVE_OPENCV
    int itsMaxNumPoints;
    int itsCurrentNumPoints;
    CvPoint2D32f* itsCurrentPoints;
    CvPoint2D32f* itsPreviousPoints;
    IplImage *itsCurrentPyramid;
    IplImage *itsPreviousPyramid;
    Image<byte> itsPreviousGreyImg;
    Image<byte> itsCurrentGreyImg;
    char* itsTrackStatus;
    float* itsTrackError;
    int itsTrackFlags;

    CvHistogram *itsObjectHist; //the histogram of the object
    IplImage* itsBackproject;
    CvRect itsTrackWindow;

    Point2D<int> itsCurrentTargetLoc;


    //For kalman filtering
    bool itsUseKalman;
    CvKalman* itsKalman;
#endif

    Image<byte> itsTargetTempl;

    bool itsInitTracker;
    bool itsTracking;

    OModelParam<int> itsTrackWindowSize;
    OModelParam<int> itsInitTrackWindowSize;

};



/* ############################### Ganglion sim events ######################## */
class SimEventVisualTracker : public SimEvent
{
public:
  SimEventVisualTracker(SimModule* src, Point2D<int>& targetLoc, bool tracking) :
    SimEvent(src), itsTargetLoc(targetLoc),
    itsTracking(tracking)
  {}

  virtual ~SimEventVisualTracker(){}

  const Point2D<int> getTargetLoc() { return itsTargetLoc; }
  const bool isTracking() { return itsTracking; }

private:
  const Point2D<int>& itsTargetLoc;
  bool itsTracking;
};

/* ############################### Ganglion sim events ######################## */
class SimEventSetVisualTracker : public SimEvent
{
public:
  SimEventSetVisualTracker(SimModule* src, Point2D<int>& targetLoc) :
    SimEvent(src), itsTargetLoc(targetLoc)
  {}

  virtual ~SimEventSetVisualTracker(){}

  const Point2D<int> getTargetLoc() { return itsTargetLoc; }

private:
  const Point2D<int>& itsTargetLoc;
};


#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
