/*!@file Robots2/Beobot2/Navigation/GistSal_Navigation/GistSal_Navigation.H Ice Module to Log data    */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/GistSal_Navigation/GistSal_Navigation.H
// $Id: GistSal_Navigation.H 15277 2012-05-07 22:37:01Z kai $
//

#ifndef GISTSAL_NAVIGATIONI_H
#define GISTSAL_NAVIGATIONI_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/Point3D.H"
#include "GUI/SimpleMeter.H"

#include "Beobot/Environment.H"
#include "Beobot/GSlocalizer.H"

#include <vector>
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"
#include "SIFT/VisualObjectMatch.H"
#include "Transport/FrameInfo.H"

#include "Beobot/SalientRegionTracker.H"

// ######################################################################

struct LandmarkDBMatch
{
  LandmarkDBMatch() { };

  LandmarkDBMatch
  (rutz::shared_ptr<VisualObject> inVo,
   Point2D<int> inVoOffset,
   GSlocJobData inDbi,
   Point2D<int> inDbOffset,
   rutz::shared_ptr<VisualObjectMatch> inMatchRes):
    vo(inVo),
    voOffset(inVoOffset),
    dbi(inDbi),
    dbOffset(inDbOffset),
    matchRes(inMatchRes)
  { }

  //! visual object that is being matched
  rutz::shared_ptr<VisualObject> vo;
  Point2D<int> voOffset;

  //! index of the matched database salient region
  GSlocJobData dbi;
  Point2D<int> dbOffset;

  //! matching statistics
  rutz::shared_ptr<VisualObjectMatch> matchRes;
};

struct MatchCoordinate
{
  MatchCoordinate() { };

  MatchCoordinate
  (Point2D<double> targetCoord,
   Point2D<double> dbCoord):
    itsTargetCoord(targetCoord),
    itsDbCoord(dbCoord)
  { }

  //! coordinate of the matches
  Point2D<double> itsTargetCoord;
  Point2D<double> itsDbCoord;
};

// ######################################################################
//! Vision Navigation using Gist and Salient Regions
class GistSal_Navigation : public RobotBrainComponent
{
public:

  GistSal_Navigation(OptionManager& mgr,
                const std::string& descrName = "GistSal_Navigation",
                const std::string& tagName = "GistSal_Navigation");

  //! set the environment - has to be done
  void setEnvironment(rutz::shared_ptr<Environment> env);

  ~GistSal_Navigation();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  void start1();

private:

  uint getKeypointMatches();
  uint projectForward
  (std::vector<rutz::shared_ptr<LandmarkDBMatch> > &landmarkDBMatches,
   nub::soft_ref<SalientRegionTracker> tracker);

  void computeErrorDifference();
  bool estimateProgressToDestination();
  void recover(bool foundSomething);
  void navigation();

  void updateMotorPID(double tran, double rot,double error);
  void updateMotor(double tran,double rot);

  void drawState();

  void updatePosition(double turn);
  Image<PixRGB<byte> > drawInfoImg();

  Image<PixRGB<byte> > itsCurrImg;
  Image<PixRGB<byte> > itsProcImg;

  IceUtil::Mutex its_input_mutex;           //!< locking current input info
  IceUtil::Mutex its_Curr_Mtr_mutex;        //!< locking motor related vars
  IceUtil::Mutex its_Curr_Dbr_mutex;        //!< locking landmark DB results
  IceUtil::Mutex its_Curr_Loc_mutex;        //!< locking current location
  IceUtil::Mutex its_tracker_mutex;         //!< locking tracker states
  IceUtil::Mutex its_Curr_CornerMtr_mutex;  //!< locking corner motor command

  nub::soft_ref<OutputFrameSeries> itsOfs;

  //! Overall Display Image
  Image<PixRGB<byte> > itsDispImg;

  //! all the environment information
  rutz::shared_ptr<Environment> itsEnvironment;

  //! from its environment: topological map
  rutz::shared_ptr<TopologicalMap> itsTopologicalMap;

  //! from its environment: visual landmark database
  rutz::shared_ptr<LandmarkDB> itsLandmarkDB;

  Timer  itsTimer;
  int    itsCurrImgID;
  int    itsPrevProcImgID;

  //! Motor related states
  double itsRcTransSpeed;
  double itsRcRotSpeed;
  int    itsRemoteMode;

  double itsTransSpeed;
  double itsRotSpeed;
  std::vector<MatchCoordinate> itsMatchCoordinates;
  std::vector<MatchCoordinate> itsCornerMatchCoordinates;

  double itsLastSetTransSpeed;
  double itsLastSetRotSpeed;

  int    itsDir;               // current direction

  float itsTurnAngle;
  float itsJunctionDist;

  double itsCornerTransSpeed;
  double itsCornerRotSpeed;
  int itsCornerProcStatus;

  double itsXError;
  double itsYError;
  double itsError;

  double itsIState;
  double itsDState;

  double itsPGain;
  double itsIGain;
  double itsDGain;

  uint itsReverseCount;
  bool itsReverseDir;

  double itsDistToGoal;

  //! tracker related modules and states
  Timer  itsLastFoundTimer;
  nub::soft_ref<SalientRegionTracker> itsSearchTracker;
  nub::soft_ref<SalientRegionTracker> itsNewMovementTracker;
  nub::soft_ref<SalientRegionTracker> itsCurrMovementTracker;
  bool itsResetSearchTracker;

  std::vector<Point2D<int> > itsCurrSalPoints;
  std::vector<Rectangle >    itsCurrSalRects;
  ImageSet<float> itsCurrCmap;

  //! incoming matched salient regions
  std::vector<rutz::shared_ptr<LandmarkDBMatch> > itsCurrLandmarkDBMatches;
  std::vector<rutz::shared_ptr<LandmarkDBMatch> > itsNewLandmarkDBMatches;

  //! path planning
  uint  itsDesiredSegmentLocation;
  float itsDesiredSegmentLengthTraveled;

  uint  itsCurrentSegmentLocation;
  float itsCurrentSegmentLengthTraveled;

  std::vector<int> itsMoves;
  bool itsInGoal;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
