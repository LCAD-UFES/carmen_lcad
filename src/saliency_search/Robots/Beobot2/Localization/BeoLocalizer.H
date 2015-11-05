/*!@file Robots/Beobot2/Localization/BeoLocalizer.H integrates
  vision based landmark recognition, and odometry data to localize
  using a topological map. It also sends out commands if there is a
  goal location sent by the task manager. We use A* algorithm to find
  the shortest path.                                                    */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the Ux.S. patent ``Computation of Intrinsic Perceptual Saliency //
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Localization/BeoLocalizer.H
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Beobot/TopologicalMap.H"
#include "Beobot/LocParticle.H"

#include "Util/Timer.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Robots/Beobot2/BeoCommon.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>
#include "Util/Timer.H"

#include "Image/Image.H"
#include "GUI/Button.H"
#include "GUI/ImageDisplayStream.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H" //for ImageDisplayStream


#ifndef BEOBOT2_LOCALIZATION_BEOLOCALIZERI_H
#define BEOBOT2_LOCALIZATION_BEOLOCALIZERI_H

// ######################################################################


// ######################################################################
//! Localization by integrating various sensory
class BeoLocalizer : public RobotBrainComponent
{
public:

  BeoLocalizer(OptionManager& mgr,
               const std::string& descrName = "BeoLocalizer",
               const std::string& tagName   = "BeoLocalizer");

  ~BeoLocalizer();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  //! register the 
  virtual void registerTopics();

  void start1();

  //! get display image of the current localization state
  Image<PixRGB<byte> > getDisplayImage
  (std::vector<Point2D<double> > particleLocs, 
   Point2D<double> currLoc, Point2D<double> currGoal, Point2D<double> vlLoc,
   Point2D<double> accOdometry,
   bool inTurnProcedure, double distanceToJunction);
  
private:

  //! initialize the localization particles
  void initParticles(LocParticle init_loc);

  //! update belief using the input just processed
  //! update our likely location
  void updateBelief();

  //! move the particles by the inputted motion
  void actionUpdateBelief(double diffHeading, Point2D<double> diffPosition);

  //! update belief using the segment prediction
  void segmentUpdateBelief();

  //! update belief using the location of the recognized landmark
  void locationUpdateBelief();

  //! set the most likely location
  void setLocation();

  //! check whether the robot needs to turn or stop
  //! given a path to a goal location
  void trackPathTraversalProgress();

  //! get the turn goal location given the angle
  Point2D<int> getTurnGoalLocation(double angle);

  //! explore the environment
  void explorationProcedure();

  //! send to beo localizer the updated goal location
  void sendGoalLocation(Point2D<int> goalLoc, uint snum, float ltrav);

  //! send the screenshot image to BeoVisualizer
  void publishScreenShot(Image<PixRGB<byte> > img);

  //! handle user mouse and keyboard event
  void handleUserEvent();

  //! convert from screen coordinates to topological map coordinates
  Point2D<int> screenToTopogical(Point2D<int> pt);



  //! its topological map
  rutz::shared_ptr<TopologicalMap> itsTopologicalMap;

  //! Google map image
  Image<PixRGB<byte> > itsGoogleMap;

  //! Map view offset for pan and zoom
  Point2D<int> itsMapDisplayOffset;
  Point2D<int> itsTopologicalMapOffset;
  double itsTopologicalMapZoom;
  double itsMapDisplayZoom;

  //! Last user mouse click location on the image
  Point2D<int> itsLastMouseClick;
  //! Last mouse click on map area
  Point2D<int> itsLastMouseClickOnMap;
	std::vector<Point2D<int> >itsLastMouseClicks;
	std::vector<Point2D<int> >itsTopologicalMapPoints;
	bool itsMeasureDisplayed;
	bool itsSimulationMode;

  //! Last mouse stay location
  Point2D<int> itsLastMouseStay;
  int itsLastMouseStayCount;




	//! Last user click location on the topological map
	LocParticle itsLastClickLocation;
  //! localization particles
  std::vector<LocParticle> itsLocalizationParticles;

  //! FIXXX: in the future a grid occupancy map

  IceUtil::Mutex its_Particle_mutex;    //!< locking localization particles
  IceUtil::Mutex its_Odometry_mutex;    //!< locking odometry information
  //! the current location particle
  //! this is done so that the weight (or confidence level)
  //! is easily accessible
  LocParticle itsCurrentLocationParticle;
  LocParticle itsInitialLocationParticle;
  
  LocParticle itsVLCurrentLocationParticle;

  //! the goal location particle
  LocParticle itsGoalLocationParticle;
  
  //! current localization standard error 
  double itsCurrentStandardError;

  //! we have a specific dead reckoning procedure
  bool itsIsDeadReckoning;
  Point2D<double> itsAccOdometry;
  std::vector<Point2D<double> > itsOdometryValues;



  //! the current path to the goal
  std::vector<int> itsMoves;
  bool itsInTurnProcedure;
  double itsDistanceToTargetLocation;

  //! display related variables
  nub::soft_ref<OutputFrameSeries> itsOfs;
  Timer itsTimer;  
  Image<PixRGB<byte> > itsDispImg;
  DebugWin itsDebugWin;
  int itsCurrentMessageID;
  int itsLastMessageID;
};
#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
