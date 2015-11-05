/*!@file Robots/Beobot2/Localization/BeoLocalizer.C integrates
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Localization/BeoLocalizer.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Localization/BeoLocalizer.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Image/DrawOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"   // for rescale()

#include "Util/Timer.H"
#include "Util/Geometry2DFunctions.H"
#include "Transport/FrameInfo.H" //for ImageDisplayStream
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"


// number of particles used for topological map localization
#define NUM_PARTICLES                  100
#define BEOLOCALIZER_DISPLAY_INTERVAL  50.0
#define MAX_BEOBOT2_LOC_ERROR          10.0

#define JUNCTION_DISTANCE_THRESHOLD    0.05 // <m>
//#define MAPOFFSET_X 935 
//#define MAPOFFSET_Y 290
//#define MAP_ZOOM 1.55 
//#define MAP_WIDTH 640
//#define MAP_HEIGHT 480

//Full t-map
#define MAPOFFSET_X 237
#define MAPOFFSET_Y 133
#define MAP_ZOOM 1.55 

//HD
//#define MAP_DISPLAY_X 176
//#define MAP_DISPLAY_Y 25 
//#define MAP_DISPLAY_Z  0.8F
//#define MAP_WIDTH 1280 
//#define MAP_HEIGHT 640 

#define MAP_DISPLAY_X 375 
#define MAP_DISPLAY_Y 48 
#define MAP_DISPLAY_Z  0.65F
#define MAP_WIDTH 640 
#define MAP_HEIGHT 480 
// ######################################################################
BeoLocalizer::BeoLocalizer
(OptionManager& mgr, const std::string& descrName, const std::string& tagName)
  :
  RobotBrainComponent(mgr, descrName, tagName),
  itsTopologicalMap(new TopologicalMap()),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsCurrentMessageID(0),
  itsLastMessageID(-1)
{
  //itsOfs->addFrameDest("display");//Add default --out=display
  addSubComponent(itsOfs);

  // initialize current location and goal location

  // FIXXX make this inputted from beoTaskManager or something reasonable
  std::string tmapFilename("src/Robots/Beobot2/Localization/maps/USC.tmap");
  std::string tmapFilename2("src/Robots/Beobot2/Localization/maps/USC.json");
  std::string gmapFilename("src/Robots/Beobot2/Localization/maps/USC_library/USC_library2.png");
  itsGoogleMap = Raster::ReadRGB(gmapFilename.c_str());

  LocParticle init_loc(0, 0.0F, 460.0F, 336.0F); init_loc.weight = 1.0F;
  LocParticle goal_loc(3, 1.0F, 761.0F, 115.0F); goal_loc.weight = 1.0F;

  //LocParticle init_loc(uint(5), 0.0F); init_loc.weight = 1.0F;
  //LocParticle goal_loc(uint(8), 1.0F); goal_loc.weight = 1.0F;

  // NOTE: Map should be in metric

  //std::string tmapFilename("../data/HNB/HNBbasement.tmap");
  //LocParticle init_loc(1, 0.0F, 14.0F,  2.0F); init_loc.weight = 1.0F;
  //LocParticle goal_loc(4, 1.0F,  6.0F,  6.0F); goal_loc.weight = 1.0F;

  itsIsDeadReckoning = true;
  itsOdometryValues.clear();
  itsAccOdometry = Point2D<double>(0.0F, 0.0F);
  itsMapDisplayOffset = Point2D<int>(MAP_DISPLAY_X, MAP_DISPLAY_Y);
  itsTopologicalMapOffset = Point2D<int>(MAPOFFSET_X, MAPOFFSET_Y);
  itsTopologicalMapZoom = MAP_ZOOM;
  itsMapDisplayZoom = MAP_DISPLAY_Z;

  itsLastMouseClick = Point2D<int>(-1,-1);
  itsLastMouseClickOnMap = Point2D<int>(-1,-1);

  itsLastMouseStay = Point2D<int>(-1,-1);
  itsLastMouseStayCount = 0;
	//the scale zgreen/itsMapDisplayOffset = Point2D<int>(677, 210);
  //itsMapDisplayZoom = 0.65F;


  // set the topological map
  //itsTopologicalMap->read(tmapFilename);
  itsTopologicalMap->readJSON(tmapFilename2);

  // initialize particles
  initParticles(init_loc);

  // set current location
  itsCurrentLocationParticle   = init_loc;
  itsInitialLocationParticle   = init_loc;
  itsVLCurrentLocationParticle = init_loc;

  // set goal location
  itsGoalLocationParticle = goal_loc;

  // command procedure
  itsInTurnProcedure = false;
  itsDistanceToTargetLocation = -1.0F;
  itsMoves.clear(); 
	itsLastMouseClicks.clear();
	itsTopologicalMapPoints.clear();
	itsMeasureDisplayed = false;
	itsSimulationMode = false;
}

// ######################################################################
BeoLocalizer::~BeoLocalizer()
{ }

// ######################################################################
void BeoLocalizer::start1()
{ }

// ######################################################################
void BeoLocalizer::registerTopics()
{
  // subscribe to odometry, IMU, Laser
  this->registerSubscription("MotorMessageTopic");
  this->registerSubscription("GPSMessageTopic");
  this->registerSubscription("VisionLocalizationMessageTopic");


  // sends out goal location command
  this->registerPublisher("GoalLocationRequestTopic");
  this->registerPublisher("CurrentLocationResetMessageTopic");
  this->registerPublisher("AccumulatedOdometryMessageTopic");

  // sends out the screen shot to BeoVisualizer
  this->registerPublisher("VisualizerMessageTopic");
}

// ######################################################################
void BeoLocalizer::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  Timer timer(1000000);

  // Odometry message
  if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
    {
      timer.reset();

      BeobotEvents::MotorMessagePtr mtrMsg =
        BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);

      // store the information
      double diffHeading    = -mtrMsg->encoderOri;
      Point2D<double> diffPosition(mtrMsg->encoderX, mtrMsg->encoderY);


      itsLastMessageID = itsCurrentMessageID;
      itsCurrentMessageID = mtrMsg->RequestID;
      
      // printf("MM[%6d]:(%9.6f %9.6f)head: %f trajLen: %f imu: %f last %3d diff %d\n", 
      //       mtrMsg->RequestID, 
      //       mtrMsg->encoderX,mtrMsg->encoderY,
      //      -mtrMsg->encoderOri, 
      //       mtrMsg->trajLen, mtrMsg->imuHeading,itsLastMessageID,itsCurrentMessageID-itsLastMessageID);

      its_Particle_mutex.lock();

      // update the belief
      actionUpdateBelief(diffHeading, diffPosition);

      // check progress of path
      // whether it needs to issue a command to change direction
      trackPathTraversalProgress();

      its_Particle_mutex.unlock();

      // FIXXXXXXXXXXXXXXXXXXXXXX THIS IS A COMPLETE HACK!!!!! 
      // should just be a handshaking message
      // check whether the turn procedure is done
      its_Odometry_mutex.lock();
      double dx = itsAccOdometry.i;
      double dy = itsAccOdometry.j;
      double distance = sqrt(dx*dx + dy*dy);
      if(distance > 2.0) itsInTurnProcedure = false;
      its_Odometry_mutex.unlock();
      ////LINFO("Distance after turning: %f", distance);
      // FIXXXXXXXXXXXXXXXXXXXXXX THIS IS A COMPLETE HACK!!!!! 
    }

  // GPS message                    
  else if(eMsg->ice_isA("::BeobotEvents::GPSMessage"))
    {
      timer.reset();

      // we got GPS data                                                   
      BeobotEvents::GPSMessagePtr gpsMsg =
        BeobotEvents::GPSMessagePtr::dynamicCast(eMsg);
      
      int currRequestID = gpsMsg->RequestID;
      LINFO("GPSMessage[%6d]", currRequestID);
      
      //double lat = gpsMsg->latitude;
      //double lon = gpsMsg->longitude;
      //double xin = gpsMsg->x;                          
      //double yin = gpsMsg->y;      

      // FIXXX: update the belief using localization observation
      //        if map has GPS tag 
      its_Particle_mutex.lock();
      //gpsObservationUpdateBelief();
      its_Particle_mutex.unlock();
    }

  // vision localization results
  // already be projected forward 
  // NOTE: should send in landmark recognition message 
  else if(eMsg->ice_isA("::BeobotEvents::VisionLocalizationMessage"))
    {
      timer.reset();

      // we got GPS data                                   
      BeobotEvents::VisionLocalizationMessagePtr vlMsg =
        BeobotEvents::VisionLocalizationMessagePtr::dynamicCast(eMsg);
      
      int currRequestID = vlMsg->RequestID;
      uint  segnum  = vlMsg->segnum;
      float lentrav = vlMsg->lentrav;
      float x       = vlMsg->x;                       
      float y       = vlMsg->y;      

      its_Particle_mutex.lock();
      itsVLCurrentLocationParticle.segnum  = segnum;
      itsVLCurrentLocationParticle.lentrav = lentrav;
      itsVLCurrentLocationParticle.x       = x;
      itsVLCurrentLocationParticle.y       = y;

      //VLObservationUpdateBelief();
      its_Particle_mutex.unlock();

      LINFO("VLMessage[%6d]: (%d %f): (%f %f)", 
            currRequestID, segnum, lentrav, x,y);     
    }



  // // update the segment belief
  // its_particles_mutex.lock();
  // segmentUpdateBelief();      
  // its_particles_mutex.unlock();
  
  // maybe pass in the variance as well

  
  // manual set location for robot
  // from the taskManager
  // reset and call the initParticles again

  // goal/turn complete message
  //   halt the robot? --> set all the 
  //  reset the dead reckoning (if on) to the new start of segment 
}

// ######################################################################
void BeoLocalizer::evolve()
{
  // draw the map
  Timer timer(1000000);

  // reset debug window
  itsDebugWin.reset();



  // its_Particle_mutex.lock();
  if(itsSimulationMode)
    {
      //simulate robot moveing
      bool inGoal = false;
      LocParticle gl = itsGoalLocationParticle;
      LocParticle cl = itsCurrentLocationParticle;
      if(gl.segnum == cl.segnum && fabs(gl.lentrav - cl.lentrav) < 0.01)
        inGoal = true;

      if(!inGoal)
        {
          if(!itsInTurnProcedure)
            actionUpdateBelief(0.0, Point2D<double>(2.0,0));//forward
          else
            actionUpdateBelief(1.0/180.0*M_PI, Point2D<double>(2.0,0));//turn
        }
    }
  trackPathTraversalProgress();

  // // check progress of path
  // // whether it needs to issue a command to change direction

  // its_Particle_mutex.unlock();

  // // FIXXXXXXXXXXXXXXXXXXXXXX THIS IS A COMPLETE HACK!!!!! 
  // // check whether the turn procedure is done
  // its_Odometry_mutex.lock();
  // double dx = itsAccOdometry.i;
  // double dy = itsAccOdometry.j;
  // double distance = sqrt(dx*dx + dy*dy);
  // if(distance > 2.0) itsInTurnProcedure = false;
  // its_Odometry_mutex.unlock();

  // //LINFO("Distance: %f", distance);
  // // FIXXXXXXXXXXXXXXXXXXXXXX THIS IS A COMPLETE HACK!!!!! 
 















  its_Particle_mutex.lock();
  std::vector<LocParticle> loc_particles = itsLocalizationParticles;
  LocParticle curr_loc = itsCurrentLocationParticle;
  LocParticle goal_loc = itsGoalLocationParticle;
  LocParticle vl_loc   = itsVLCurrentLocationParticle;
  bool inTurnProcedure = itsInTurnProcedure;
  double distanceToTargetLocation = itsDistanceToTargetLocation;
  its_Particle_mutex.unlock();

  its_Odometry_mutex.lock();
  std::vector<Point2D<double> > odometryValues = itsOdometryValues;
  Point2D<double> accOdometry = itsAccOdometry;
  its_Odometry_mutex.unlock();

  std::vector<Point2D<double> > particleLocs;
  for(uint i = 0; i < loc_particles.size(); i++)
    {
      uint  snum  = loc_particles[i].segnum;
      double ltrav = loc_particles[i].lentrav;

      Point2D<double> temp_pt
        (itsTopologicalMap->getLocationFloat(snum, ltrav));
      particleLocs.push_back(temp_pt);
      //LINFO("[%5.2f %5.2f]: %d %f", 
      //      particleLocs[0].i, particleLocs[0].j, snum, ltrav);
    }
  Point2D<double> currLoc
    (itsTopologicalMap->getLocationFloat
     (curr_loc.segnum, curr_loc.lentrav));    

  Point2D<double> goalLoc
    (itsTopologicalMap->getLocationFloat
     (goal_loc.segnum, goal_loc.lentrav));    

  Point2D<double> vlLoc
    (itsTopologicalMap->getLocationFloat
     (vl_loc.segnum, vl_loc.lentrav));    

  LDEBUG("goal: [%f %f]:[%d %f]  cl: [%f %f]:[%d %f] vl: [%f %f]:[%d %f]", 
         goalLoc.i, goalLoc.j, curr_loc.segnum, curr_loc.lentrav,
         currLoc.i, currLoc.j, curr_loc.segnum, curr_loc.lentrav,
         vlLoc.i, vlLoc.j, vl_loc.segnum, vl_loc.lentrav);

  itsDispImg = getDisplayImage
    (particleLocs, currLoc, goalLoc, vlLoc, accOdometry, 
     inTurnProcedure, distanceToTargetLocation);

  itsOfs->writeRGB(itsDispImg, "BeoLocalizer::evolve",FrameInfo("BLocal",SRC_POS));
  //itsOfs->updateNext();  
  handleUserEvent();
  double time = timer.get()/1000.0;
  //LINFO("time: %f", time);
  //Raster::waitForKey();

  if(time < BEOLOCALIZER_DISPLAY_INTERVAL)
    {
      double diff = BEOLOCALIZER_DISPLAY_INTERVAL - time;
      sleep(diff/1000.0f);
    }
}

// ######################################################################
void BeoLocalizer::initParticles(LocParticle init_loc)
{
  uint nsegment = itsTopologicalMap->getSegmentNum();

  itsLocalizationParticles.clear();

  // if we have an invalid initial location
  if(init_loc.lentrav == -1.0F)
    {
      LINFO("create random particles");

      // create initial random particles
      for(uint i = 0; i < NUM_PARTICLES; i++)
        {
          double t  = rand()/(RAND_MAX + 1.0);
          double t2 = rand()/(RAND_MAX + 1.0);

          uint  snum  = uint ((0)    + ((nsegment) * t ));
          double ltrav = double((0.0F) + ((1.0F    ) * t2));

          // no need to compute the cartesian coordinate for now
          itsLocalizationParticles.push_back(LocParticle(snum, ltrav));
          itsLocalizationParticles[i].weight = 0.5F;
        }
    }
  else
    {
      uint   snum  = init_loc.segnum;
      double ltrav = init_loc.lentrav;
      LINFO("Use DeadReckoning seg:%d @ %f",snum,ltrav);
      itsIsDeadReckoning = true;

      // get the particles
      for(uint i = 0; i < NUM_PARTICLES; i++)
        {
          itsLocalizationParticles.push_back(LocParticle(snum, ltrav));
          itsLocalizationParticles[i].weight = 1.0F;
        }
      // reset the odometry data to new location
      double altrav = itsTopologicalMap->getSegmentLength(snum)*ltrav;
      itsAccOdometry = Point2D<double>(altrav, 0.0F);

      // send out current location reset
      BeobotEvents::CurrentLocationResetMessagePtr clrMsg =
        new BeobotEvents::CurrentLocationResetMessage;
      
      clrMsg->RequestID = itsCurrentMessageID;
      clrMsg->snum      = snum; 
      clrMsg->ltrav     = ltrav; 
      LINFO("CLR[%d]: %d %f", itsCurrentMessageID, snum, ltrav);
      publish("CurrentLocationResetMessageTopic", clrMsg);
    }

  // FIXXX: should we fill in the cartesian information as well??
  
  setLocation();
} 

// ######################################################################
void BeoLocalizer::updateBelief()
{
  //! update belief using the input just processed
  //! update our likely location
}

// ######################################################################
void BeoLocalizer::actionUpdateBelief
(double diffHeading, Point2D<double> diffPosition)
{
  // move the object from the previous location

  // if is dead reckoning
  double dx = 0.0F;
  //double toFt = 100.0/2.54/12;

  if(itsIsDeadReckoning)
    {
      // we can more accurately estimate the current movement

      // store the movement and heading to estimate road heading
      // get front movement with respect to that information

      its_Odometry_mutex.lock();
      itsOdometryValues.push_back(diffPosition);
      itsAccOdometry += diffPosition;
      Point2D<double> accOdometry = itsAccOdometry;
      its_Odometry_mutex.unlock();

      double di = accOdometry.i;
      double dj = accOdometry.j;
      dx = sqrt(di*di + dj*dj);

      itsCurrentStandardError = dx/20.0F; 
      if(itsCurrentStandardError == 0.0F) 
        itsCurrentStandardError = .05F;
    }
  // if IMU is not on
  else
    {
      // FIXXX for now just use DX and acccumulate
      dx = diffPosition.i;
    }
  
  // convert to map scale
  double mscale = itsTopologicalMap->getMapScale();
  dx /= mscale;

  // apply the motor movement + noise to each particles
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      uint  snum  = itsLocalizationParticles[i].segnum;
      double ltrav = itsLocalizationParticles[i].lentrav;
      LDEBUG("particle[%d]: %d, %f", i, snum, ltrav);
      
      LDEBUG("len[%d]: %f", i, itsTopologicalMap->getSegmentLength(snum));

      // convert dx to current segment length travel percentage
      double init_ltrav = 0.0F;
      if(itsInitialLocationParticle.segnum == snum) 
        init_ltrav = itsInitialLocationParticle.lentrav;

      double curr_dx = 
        init_ltrav + dx/itsTopologicalMap->getSegmentLength(snum);
                                                  
      // apply the motor command
      double nltrav = 0.0;
      if(itsIsDeadReckoning)
        {
          nltrav = curr_dx;
        }
      else
        {
          nltrav = ltrav + curr_dx;

          // assume std odometry error of .02ft
          double err    = BEOBOT2_STDEV_ODOMETRY_ERROR/mscale;
          
          // add noise from a Gaussian distribution (Box-Muller method)
          double r1 = double(rand()/(RAND_MAX + 1.0));
          double r2 = double(rand()/(RAND_MAX + 1.0));
          double r = err * sqrt( -2.0 * log(r1));
          double phi = 2.0 * M_PI * r2;
          nltrav += (r * cos(phi));
        }

      // if nltrav is now improper
      // FIXXX: NEED THE SPILL OVER EFFECT
      if(nltrav < 0.0F) nltrav = 0.0F;
      if(nltrav > 1.0F) nltrav = 1.0F;
      itsLocalizationParticles[i].lentrav = nltrav;
    }
  
  if(itsIsDeadReckoning)
    {
      // send out accumulated odometry since turning
      BeobotEvents::AccumulatedOdometryMessagePtr aomsg =
          new BeobotEvents::AccumulatedOdometryMessage;
      aomsg->RequestID = itsCurrentMessageID;
      aomsg->AccOdo    = dx;
      LINFO("AccOdo[%d]: %f", itsCurrentMessageID, dx);
      publish("AccumulatedOdometryMessageTopic", aomsg);
    }

  setLocation();
}

// ######################################################################
void BeoLocalizer::segmentUpdateBelief()
{
  // update belief using the segment prediction

  itsCurrentStandardError = MAX_BEOBOT2_LOC_ERROR;
  //  
  setLocation();
}

// ######################################################################
void BeoLocalizer::locationUpdateBelief()
{
  // store the current location if deadreckoning is in effect
  
  //! update belief using the location of the recognized landmark

  // if best location is not close to deadreckoning 
  // itsIsDeadReckoning = false;

  itsCurrentStandardError = MAX_BEOBOT2_LOC_ERROR;
  
  setLocation();
}

// ######################################################################
void BeoLocalizer::setLocation()
{
  //! set the most likely location
  double max_weight = itsLocalizationParticles[0].weight; 
  uint max_i = 0;
  for(uint i = 1; i < NUM_PARTICLES; i++)
    {
      double weight = itsLocalizationParticles[i].weight;
      if(max_weight < weight) { max_weight = weight; max_i = i; }
    }
  itsCurrentLocationParticle = itsLocalizationParticles[max_i];  
}

// ######################################################################
void BeoLocalizer::trackPathTraversalProgress()
{
  // check if there is a specific goal location
  // if no direction use exploration
  if(itsGoalLocationParticle.x == -1.0F) explorationProcedure();
    
  LocParticle cl = itsCurrentLocationParticle;      
  LocParticle gl = itsGoalLocationParticle;      

  // get local map dimensions
  int lmw = LOCAL_MAP_NUM_HORZ_GRIDS;
  int lmh = LOCAL_MAP_NUM_VERT_GRIDS;

  // grid height in metric
  double gh  = LOCAL_MAP_GRID_HEIGHT/1000.0F;

  // LINFO("from [%d %f] to [%d %f]", 
  //       cl.segnum, cl.lentrav, gl.segnum, gl.lentrav);

  // get path
  std::vector<int> moves;
  double dist =
    itsTopologicalMap->getPath
    (cl.segnum, cl.lentrav, gl.segnum, gl.lentrav, moves);
  //if(dist == -1.0F) 
  //  LINFO("no path from (%d %f) to (%d %f)",
  //        cl.segnum, cl.lentrav, gl.segnum, gl.lentrav);
  uint msize = moves.size();
  //for(uint i = 0; i < msize; i++) LDEBUG("Move[%d]: %d", i, moves[i]);

  // real metric distance 
  dist *= itsTopologicalMap->getMapScale();

  // default goal of staying in place
  Point2D<int> nextGoal(-2,-2);
  LDEBUG("moves: %"ZU , moves.size());

  // check if the robot is on the goal
  if(msize == 0)
    { 
      itsDistanceToTargetLocation = 0.0F;
      nextGoal = 
        Point2D<int>(ROBOT_CENTER_RATIO_I*lmw, 
                     ROBOT_CENTER_RATIO_J*lmh); return; 
    }

  // check if robot is on the same segment as the goal
  if(msize == 1)
    {
      // goal is straight ahead
      itsDistanceToTargetLocation = dist;
      nextGoal = 
        Point2D<int>(ROBOT_CENTER_RATIO_I*lmw, -1);

      // threshold for the edge of the local map
      double threshToGoal = 
        ROBOT_CENTER_RATIO_J*lmh*gh;
      LINFO("dist: %f < thresh: %f", dist, threshToGoal);
      if(dist <= threshToGoal)
        {          
          LINFO("1 Move: NEAR GOAL"); 
          nextGoal.j = int((threshToGoal - dist)/gh);

          // FIXXX: stop if it's close enough
          if(dist < 3*gh) nextGoal.j = ROBOT_CENTER_RATIO_J*lmh;
          sendGoalLocation(nextGoal, 0, -1.0F); return;
        }
    }
  
  // check whether the robot needs to turn around
  // if the goal is behind it
  else if(moves[0] == TOPOMAP_TURN_AROUND)
    {
      LINFO("1 Move: TURN AROUND");
      nextGoal = Point2D<int>
        (ROBOT_CENTER_RATIO_I*lmw, lmh);

      //LINFO("from [%d %f] to [%d %f]", 
      //      cl.segnum, cl.lentrav, gl.segnum, gl.lentrav);
      //for(uint i = 0; i < msize; i++) 
      //  LINFO("Move[%d]: %d", i, moves[i]);
    }

  // else if moving forward and there is at least one more turn

  // HACK TO ALLOW IT TO SKIP FIRST SEGMENT
  else if(moves[0] == TOPOMAP_MOVE_FORWARD && msize > 1) 
    {
      // get the current edge
      double ds, de;
      rutz::shared_ptr<Edge> e1 =
        itsTopologicalMap->getEdge(cl.segnum, cl.lentrav, ds, de);

      // distance to next junction in meters
      double jdist = de * itsTopologicalMap->getMapScale();
      itsDistanceToTargetLocation = jdist;
      //LINFO("JDIST: %f", jdist);

      // if we are near/on the junction
      // FIXXX NOTE: may need to deal with overshoot at junction 
      //             such as setting actual goal distance inside the local map
      //             so navigator can lower the speed
      if(jdist < JUNCTION_DISTANCE_THRESHOLD)
        //  if(jdist <= 0.0F)
        {
          // get the next edge
          rutz::shared_ptr<Edge> e2 =
            itsTopologicalMap->getEdge(moves[1]);

          // calculate the angle of turn
          // Note: negate to use robot convention
          double angle = -itsTopologicalMap->getAngle(e1,e2);
          nextGoal     =  getTurnGoalLocation(angle);         
          
          // get the start location of the next edge
          uint   snum  = itsLocalizationParticles[0].segnum + 1;
          double ltrav = 0.0;
          itsTopologicalMap->getEdgeStartLocation(moves[1], snum, ltrav);

          LINFO("Turning to: %d %d", nextGoal.i, nextGoal.j);

          Point2D<double> p(itsTopologicalMap->getLocationFloat(snum, ltrav)); 
          LocParticle loc(snum, ltrav, p.i, p.j); loc.weight = 1.0F;

          // advance the particles
          initParticles(loc);

          // zero out the accumulated odometry
          itsOdometryValues.clear();
          itsAccOdometry = Point2D<double>(0.0F, 0.0F);
        }
    }
  else itsDistanceToTargetLocation = -1.0;

  // turn on turn procedure indicator
  if(!itsInTurnProcedure)
    {
      int gi = nextGoal.i; int gj = nextGoal.j;
      if(( gi == -1 || gi == lmw || gj == -1 || gj == lmh) && 
         !(gi == int(ROBOT_CENTER_RATIO_I*lmw) && gj == -1)    )
       {
         itsInTurnProcedure = true;
         
         // send stop command just once
         sendGoalLocation(nextGoal, 
                          itsCurrentLocationParticle.segnum,
                          itsCurrentLocationParticle.lentrav);

         LINFO("TURN procedure is ON");
       }
    }
}

// ######################################################################
Point2D<int> BeoLocalizer::getTurnGoalLocation(double angle)
{
  LINFO("ANGLE: %f", angle*180.0/M_PI);

  int lmw    = LOCAL_MAP_NUM_HORZ_GRIDS;
  int lmh    = LOCAL_MAP_NUM_VERT_GRIDS;

  
  Point2D<double> p1
    (ROBOT_CENTER_RATIO_I*lmw, 
     ROBOT_CENTER_RATIO_J*lmh );
  double di = double(-sin(angle)); double dj = double(-cos(angle));
  Point2D<double> p2(p1.i+di, p1.j+dj);

  Point2D<double> goalf;

  // top left quadrant
  if(di <= 0.0F && dj <= 0.0F)
    {
      Point2D<double> p3_1(   -.5,  -.5);
      Point2D<double> p4_1(lmw+.5,  -.5);      
      Point2D<double> r1 = intersectPoint(p1,p2,p3_1,p4_1);

      Point2D<double> p3_2( -.5,    -.5);
      Point2D<double> p4_2( -.5, lmh+.5);
      Point2D<double> r2 = intersectPoint(p1,p2,p3_2,p4_2);

      double dist1 = r1.distance(p1);
      double dist2 = r2.distance(p1);      
      if(dist1 <= dist2) goalf = r1 + Point2D<double>(  0,-.5); 
      else               goalf = r2 + Point2D<double>(-.5,  0);
      LINFO("Q1: %f %f", dist1,dist2);
    }

  // top right quadrant
  else if(di > 0.0F && dj < 0.0F)
    {
      Point2D<double> p3_1(   -.5,  -.5);
      Point2D<double> p4_1(lmw+.5,  -.5);      
      Point2D<double> r1 = intersectPoint(p1,p2,p3_1,p4_1);

      Point2D<double> p3_2(lmw+.5,    -.5);
      Point2D<double> p4_2(lmw+.5, lmh+.5);
      Point2D<double> r2 = intersectPoint(p1,p2,p3_2,p4_2);

      double dist1 = r1.distance(p1);
      double dist2 = r2.distance(p1);
      
      if(dist1 <= dist2) goalf = r1 + Point2D<double>( 0,-.5); 
      else goalf = r2;
      LINFO("Q2: %f %f", dist1,dist2);
    }

  // bottom left quadrant
  else if(di < 0.0F && dj > 0.0F)
    {
      Point2D<double> p3_1(   -.5, lmh+.5);
      Point2D<double> p4_1(lmw+.5, lmh+.5);      
      Point2D<double> r1 = intersectPoint(p1,p2,p3_1,p4_1);

      Point2D<double> p3_2( -.5,    -.5);
      Point2D<double> p4_2( -.5, lmh+.5);
      Point2D<double> r2 = intersectPoint(p1,p2,p3_2,p4_2);

      double dist1 = r1.distance(p1);
      double dist2 = r2.distance(p1);
      
      if(dist1 <= dist2) goalf = r1; 
      else               goalf = r2 + Point2D<double>(-.5,  0);
      LINFO("Q3: %f %f", dist1,dist2);
    }

  // bottom right quadrant
  else if(di >= 0.0F && dj >= 0.0F)
    {
      Point2D<double> p3_1(   -.5, lmh+.5);
      Point2D<double> p4_1(lmw+.5, lmh+.5);      
      Point2D<double> r1 = intersectPoint(p1,p2,p3_1,p4_1);

      Point2D<double> p3_2(lmw,  -1);
      Point2D<double> p4_2(lmw, lmh);
      Point2D<double> r2 = intersectPoint(p1,p2,p3_2,p4_2);

      double dist1 = r1.distance(p1);
      double dist2 = r2.distance(p1);
      
      if(dist1 <= dist2) goalf = r1; else goalf = r2;
      LINFO("Q4: %f %f", dist1,dist2);
    }

  Point2D<int> goal(goalf.i, goalf.j); 
  LINFO("di: %f dj: %f|p2(%f,%f)| %f %f | %d %d",
        di, dj, p2.i,p2.j, goalf.i, goalf.j, goal.i, goal.j);

  return goal;
}

// ######################################################################
void BeoLocalizer::sendGoalLocation
(Point2D<int> goalLoc, uint snum, float ltrav)
{
  BeobotEvents::GoalLocationRequestPtr msg = 
    new BeobotEvents::GoalLocationRequest;
  msg->goalLoc.i = goalLoc.i;
  msg->goalLoc.j = goalLoc.j;
  if(ltrav != -1.0F)
    msg->goalType  = BEONAVIGATOR_TURN_GOAL;
  else
    msg->goalType  = BEONAVIGATOR_STATIONARY_TARGET_GOAL;
  msg->snum      = snum;
  msg->ltrav     = ltrav; 
  this->publish("GoalLocationRequestTopic", msg);
  LINFO("Publishing goal location (%d %d)",goalLoc.i, goalLoc.j);
}

// ######################################################################
void BeoLocalizer::explorationProcedure()
{  
  // may need SLAM at some point
  
  // Saliency probably will take over as well
}

// ######################################################################
void BeoLocalizer::publishScreenShot(Image<PixRGB<byte> > img)
{
//  BeobotEvents::VisualizerMessagePtr msg = 
//    new BeobotEvents::VisualizerMessage;
//  msg->image     = Image2Ice(img); 
//  msg->BeoAppID  = BEO_LOCALIZER;
//  msg->RequestID = itsCurrentMessageID++;
//  this->publish("VisualizerMessageTopic", msg);


  std::string saveTName(sformat("./beoLoc_%05d.png",itsCurrentMessageID++));

  //save screen shot to file
  Raster::WriteRGB(img, saveTName);
  LINFO("Save %s ",saveTName.c_str());
}

// ######################################################################
Image<PixRGB<byte> > BeoLocalizer::getDisplayImage
(std::vector<Point2D<double> > particleLocs, 
 Point2D<double> currLoc, Point2D<double> goalLoc, Point2D<double> vlLoc,
 Point2D<double> accOdometry,
 bool inTurnProcedure, double distanceToTargetLocation)
{
  // define drawing colors
  PixRGB<byte> black (  0,  0,  0);
  PixRGB<byte> white (255,255,255);
  PixRGB<byte> red   (255,  0,  0);
  PixRGB<byte> blue  (  0,  0,255);
  PixRGB<byte> green (  0,255,  0);
  PixRGB<byte> yellow(255,255,  0); 
  PixRGB<byte> orange(255,128,  0);
  PixRGB<byte> cyan  (  0,255,255);
  PixRGB<byte> darkgreen (  0,100,  0);
 

  int w  = MAP_WIDTH;
  int h  = MAP_HEIGHT;
  int h2 = MAP_HEIGHT/3;
  Image< PixRGB<byte> > res(w,h+h2,ZEROS);

/*
   itsGoogleMap(G) 1969 x 1481
  +--------------------------------------------------------
  |        TopologicalDisplayMap(T) 640 x 480 :itsTopologicalMapOffset 
  |		G = T + gOff | T = G - gOff = (M + mOff)/Z - gOff 
  |       +-------------------
  |       |     +------Actuall Topological Map(A) -> A = (T - 0.5) / scale, scale = T.width / A.width 
  |       |     |
  |       |  Screen DisplayMap(M) 640x480 itsMapDisplayOffset(mOff),itsMapDisplayZoom(Z)  
  |    +--+----------+ 			G = (M + mOff)/ Z | M = (G*Z) - mOff
  |    |  |          | M is also mouse click coord
  |    |  |  +----+  |
  |    |  |  |  R | RobotCloseMap(R) 240 x 240: R = G + robotLoc - 120
  |    |  |  +----+  |
*/

  Image< PixRGB<byte> > ggmap = itsGoogleMap;//rescale(itsGoogleMap,itsGoogleMap.getDims()*itsTopologicalMapZoom);
  //check gmapoffset is valid
  //if(!ggmap.coordsOk(Point2D<int>(gmapOffset.i+w,gmapOffset.j+h)))
  //  gmapOffset = Point2D<int>(0,0);


  //crop gmap to display size,fill black when outside the gmap
  //Image< PixRGB<byte> > ggmapCroped = crop(ggmap,gmapOffset,Dims(w,h),true);

  // get the map from the topolagical map class
  Image< PixRGB<byte> > mapImg = itsTopologicalMap->getMapImage(ggmap,itsTopologicalMapZoom,itsTopologicalMapOffset);
  //Dims d = itsTopologicalMap->getMapDims();
  //LINFO("dims: %d %d", d.w(), d.h());

  // add the particles on the map
  //int scale = int(mapImg.getWidth()/d.w());
  for(uint i = 0; i < particleLocs.size(); i++)
    {
      // get the point
      Point2D<double> loc = particleLocs[i] * itsTopologicalMapZoom + itsTopologicalMapOffset;
      //LINFO("point: %5.2f %5.2f", loc.i, loc.j);
      //LINFO("point: %5.2f %5.2f", particleLocs[i].i, particleLocs[i].j);
      drawDisk(mapImg, Point2D<int>(loc.i+.5,loc.j+.5), 2, 
               PixRGB<byte>(0,255,255));
    }

  // Find closest point on the TopologicalMap segment from mouse click
  uint seg = 0; double ltrav = -1.0F;
  if(itsLastMouseClick.i >= 0 && itsLastMouseClick.i < w && 
     itsLastMouseClick.j >= 0 && itsLastMouseClick.j < h)
    {
      itsLastMouseClickOnMap = itsLastMouseClick;
      //shift and zoom to TopologicalMap drawing coord (Pt/Z)+M-G
      Point2D<double> mouseClickInMapCoord = 
        (Point2D<double>(itsLastMouseClick+itsMapDisplayOffset)/itsMapDisplayZoom);

      //convert again to topological map coord
      Point2D<int> mouseClickInTMapCoord = 
        Point2D<int>((mouseClickInMapCoord.i-itsTopologicalMapOffset.i-0.5)/itsTopologicalMapZoom, 
                     (mouseClickInMapCoord.j-itsTopologicalMapOffset.j-0.5)/itsTopologicalMapZoom);

      itsTopologicalMap->getLocation(mouseClickInTMapCoord,seg,ltrav);
      //LINFO("close location of (%d,%d) is seg %d ltrav %f",
      //		  mouseClickInTMapCoord.i,mouseClickInTMapCoord.j,seg,ltrav);

      itsLastClickLocation = LocParticle(seg,ltrav);
      //LINFO("You click seg:%d lenv:%f",itsLastClickLocation.segnum,itsLastClickLocation.lentrav);


      // add click points to vector, do not add it if its duplicated point
      if(itsLastMouseClicks.size()== 0 || itsLastMouseClicks[itsLastMouseClicks.size()-1] != itsLastMouseClick)
        {
          itsLastMouseClicks.push_back(itsLastMouseClick);
          itsTopologicalMapPoints.push_back(mouseClickInTMapCoord);
        }
    }else{
		
		
  }

  //LINFO("Gmap (%d x %d) MapImg (%d x %d) GmapOffset (%d,%d)",
  //    ggmap.getWidth(),ggmap.getHeight(),
  //    mapImg.getWidth(),mapImg.getHeight(),
  //    itsTopologicalMapOffset.i,itsTopologicalMapOffset.j);
  // paste annotated crop map back to full map

  if(seg != 0 && ltrav != -1.0F)
    {
      Point2D<double> clickPtOnSeg
        (itsTopologicalMap->getLocationFloat 
         (itsLastClickLocation.segnum, itsLastClickLocation.lentrav));    
      Point2D<int> clickPtOnMap = 
        Point2D<int>(clickPtOnSeg.i*itsTopologicalMapZoom+0.5+itsTopologicalMapOffset.i,
                     clickPtOnSeg.j*itsTopologicalMapZoom+itsTopologicalMapOffset.j+0.5);
      drawDisk(mapImg, clickPtOnMap, 5.0/itsMapDisplayZoom, PixRGB<byte>(255,255,0));
    }


  //inplacePaste(ggmap, mapImg, itsTopologicalMapOffset);

  //// draw the most likely robot location
  //double mscale = itsTopologicalMapZoom;//itsTopologicalMap->getMapScale();
  // radius of error
  int rad = 2;//int(itsCurrentStandardError/mscale*scale +.5); 
  if(rad < 2) rad = 2;
  //LINFO("scale: %d mscale: %f rad: %d", scale, mscale, rad);
  Point2D<int> cl(currLoc.i*itsTopologicalMapZoom+.5, currLoc.j*itsTopologicalMapZoom+.5);
  Point2D<int> clg = cl + itsTopologicalMapOffset;

  Point2D<int> gl(goalLoc.i*itsTopologicalMapZoom+.5, goalLoc.j*itsTopologicalMapZoom+.5);
  Point2D<int> glg = gl + itsTopologicalMapOffset;

  Point2D<int> vll(vlLoc.i*itsTopologicalMapZoom+.5, vlLoc.j*itsTopologicalMapZoom+.5);
  Point2D<int> vllg = vll + itsTopologicalMapOffset;


  //crop a zoom in map of current robot location,size of h2 x h2
  Image< PixRGB<byte> > robotCloseMap = 
    crop(ggmap,clg-Point2D<int>(h2/2,h2/2),Dims(h2-2,h2-2),true);
  drawStar2(robotCloseMap, Point2D<int>(h2/2,h2/2), PixRGB<byte>(0,0,255),10,1);
  inplacePaste(res, robotCloseMap, Point2D<int>(w-h2+1,h+1));


  // draw current and goal location in last scale map, 
  // so the mark size is independent from Map scale
  Image< PixRGB<byte> > zoomMap = rescale(mapImg,mapImg.getDims()*itsMapDisplayZoom);



  Point2D<int> disp_vl(vllg*itsMapDisplayZoom);
  drawStar2(zoomMap, disp_vl, red,10,1);

  Point2D<int> disp_cl(clg*itsMapDisplayZoom);
  drawStar2(zoomMap, disp_cl, blue,15,1);
  drawStar2(zoomMap, disp_cl, blue,10,2);
  if(rad > 0) drawCircle(zoomMap, disp_cl, rad, green, 1);

  Point2D<int> disp_goal(glg*itsMapDisplayZoom);
  if(goalLoc.isValid()) drawDisk(zoomMap, disp_goal, 10, blue);

  LDEBUG("goal: %d %d cl: %d %d vl: %d %d", 
         disp_goal.i, disp_goal.j, disp_cl.i, disp_cl.j, disp_vl.i, disp_vl.j);


  int vw = 640;//for BeoVisualizer
  int vh = 440;//for BeoVisualizer
  // crop a map to send to BeoVisualizer, 
  // same scale and offset of display map but different dims 
  Image< PixRGB<byte> > visualizerMap = 
    crop(zoomMap,itsMapDisplayOffset+Point2D<int>(0,70),Dims(vw,vh),true);
  //itsDebugWin.show(visualizerMap,"Visualizer Map");
  

  //publishScreenShot(visualizerMap);



  Image< PixRGB<byte> > finalMap = crop(zoomMap, itsMapDisplayOffset,Dims(w,h),true);

  drawDisk(finalMap, itsLastMouseClick, 5, PixRGB<byte>(0,255,255));

  // paste the map
  inplacePaste(res, finalMap, Point2D<int>(0,0));

  // simulate mode info
  if(itsSimulationMode)
    {
      writeText(res, Point2D<int>(0,0), 
		sformat("Simulation Mode").c_str(),
		red, white, SimpleFont::FIXED(12));
    }

  // odometry information
  double dx = accOdometry.i;
  double dy = accOdometry.j;
  double distance = sqrt(dx*dx + dy*dy);
  double traveled = itsTopologicalMap->getSegmentLength
    (itsCurrentLocationParticle.segnum)*(itsCurrentLocationParticle.lentrav);
  std::string text = 
    sformat("[DX:%7.2f DY:%7.2f]: D:%7.2f", accOdometry.i, accOdometry.j, distance);
  writeText(res, Point2D<int>(0,h), text.c_str(),white,
            black, SimpleFont::FIXED(12));

  Rectangle r = Rectangle::tlbrO(30+h,5,70+h,45);
  if(inTurnProcedure) drawFilledRect(res, r, green);
  else                drawFilledRect(res, r, red  );
  std::string text2 = sformat(": %7.2fm %7.2fm", distanceToTargetLocation,traveled );
  writeText
    (res, Point2D<int>(50,30+h), text2.c_str(),
     white, black, SimpleFont::FIXED(20));


  //draw last stay location
  if(itsLastMouseStayCount > 3 && 
     itsLastMouseStay.i >= 0 && itsLastMouseStay.i < w && 
     itsLastMouseStay.j >= 0 && itsLastMouseStay.j < h)
    {
      Point2D<int> stayPt = screenToTopogical(itsLastMouseStay);
      std::string label = itsTopologicalMap->getNodeLabel(stayPt);
      writeText (res, itsLastMouseStay, label.c_str(), 
                 red, black, SimpleFont::FIXED(12),true);
    }

  double len = 
    itsTopologicalMap->getSegmentLength(itsLastClickLocation.segnum)*
    (1.0-itsLastClickLocation.lentrav);
  std::string segText= 
    sformat(" Click :%-2d@ %7.5f %-5.2fm", 
            itsLastClickLocation.segnum, itsLastClickLocation.lentrav,len);
  writeText(res, Point2D<int>(2,h+h2-75), segText.c_str(),white,
            black, SimpleFont::FIXED(12));



  //Button startButton =  
  //  Button(Point2D<int>(0,h+110),Dims(70,30));
  //startButton.setLabel(std::string("Start"));
  //startButton.setButtonBgColor(red);
  //startButton.setFrontSize(12);
  //startButton.show(res);


  writeText(res, Point2D<int>(95,h+h2-50), 
            sformat("%-2d@ %6.4f%%",
                    itsCurrentLocationParticle.segnum,
                    itsCurrentLocationParticle.lentrav).c_str(),
            blue, black, SimpleFont::FIXED(12));
	
  writeText(res, Point2D<int>(95,h+h2-25), 
            sformat("%-2d@ %6.4f%%",
                    itsGoalLocationParticle.segnum,
                    itsGoalLocationParticle.lentrav).c_str(),
            green, black, SimpleFont::FIXED(12));

  Button currentButton =  
    Button(Point2D<int>(0,h+h2-50),Dims(90,25));
  currentButton.setLabel(std::string("Current"));
  currentButton.setButtonBgColor(blue);
  currentButton.setFrontSize(12);
  currentButton.show(res);

  Button goalButton =  
    Button(Point2D<int>(0,h+h2-25),Dims(90,25));
  goalButton.setLabel(std::string("Goal"));
  goalButton.setButtonBgColor(green);
  goalButton.setFrontSize(12);
  goalButton.show(res);

  Button measureButton =  
    Button(Point2D<int>(w/2,h+h2-85),Dims(120,25));
  if(itsMeasureDisplayed)
    measureButton.setLabel(std::string("MeasureOn"));
  else
    measureButton.setLabel(std::string("MeasureOff"));
  measureButton.setButtonBgColor(yellow);
  measureButton.setLabelColor(black);
  measureButton.setFrontSize(12);
  measureButton.show(res);

  if(measureButton.handle_events(itsLastMouseClick))
    {
      LINFO("You click Measure");
      itsMeasureDisplayed = !itsMeasureDisplayed;
      itsLastMouseClick = Point2D<int>(-1,-1);
    }

  //center the map to mouse click
  Button centerButton =  
    Button(Point2D<int>(w/2,h+h2-25),Dims(90,25));
  centerButton.setLabel(std::string("Center"));
  centerButton.setButtonBgColor(orange);
  centerButton.setFrontSize(12);
  centerButton.show(res);

  if(itsMeasureDisplayed){
    uint ptnum = itsLastMouseClicks.size();
    double totalLength = 0.0;// in meters
    for(uint i = 0;i < ptnum;i++)
      {		
        // draw current point and coordinate in actuall topological map space 
        Point2D<int> tcpt = itsTopologicalMapPoints[i];
        Point2D<int> cpt = Point2D<int>(((tcpt*itsTopologicalMapZoom+0.5) + itsTopologicalMapOffset)*itsMapDisplayZoom - itsMapDisplayOffset);
        drawDisk(res, cpt, 2, PixRGB<byte>(0,255,255));		
        if(i > 0)
          {
            Point2D<int> tlpt = itsTopologicalMapPoints[i-1];
            //convert from actual topological map coord to current screen coord
            Point2D<int> lpt = Point2D<int>(((tlpt*itsTopologicalMapZoom+0.5) + itsTopologicalMapOffset)*itsMapDisplayZoom - itsMapDisplayOffset);
            Point2D<int> mpt = (cpt+lpt) /2;
            drawLine(res, cpt,  lpt, white, 4,0.1);//thick 4, transparent 10%
            drawLine(res, cpt,  lpt, blue , 2,0.15);//thick 2, transparent 15%
            drawLine(res, cpt,  lpt, black, 1);//thick 1 

            double dist = lineDist(tcpt,tlpt);	
            writeText(res, mpt, sformat("%4.2fm",dist).c_str(), red, white, SimpleFont::FIXED(8),true);

            totalLength += dist;
          }
        writeText(res, cpt, sformat("(%d,%d)",tcpt.i,tcpt.j).c_str(), darkgreen, white, SimpleFont::FIXED(5));
      }
    writeText(res, Point2D<int>(w/2,h+h2-60), 
              sformat("Total: %5.2f m",totalLength).c_str(), white, black, SimpleFont::FIXED(8));
  }else{
    itsLastMouseClicks.clear();
    itsTopologicalMapPoints.clear();
  } 

  if(currentButton.handle_events(itsLastMouseClick))
    {
	
      LINFO("You click current seg:%d lenv:%f",
            itsLastClickLocation.segnum,itsLastClickLocation.lentrav);
      initParticles(itsLastClickLocation);
      itsCurrentLocationParticle = itsLastClickLocation;
      itsLastMouseClick = Point2D<int>(-1,-1);
    }
  else if(goalButton.handle_events(itsLastMouseClick))
    {	
      LINFO("You click goal seg:%d lenv:%f",
            itsLastClickLocation.segnum,itsLastClickLocation.lentrav);
      itsGoalLocationParticle = itsLastClickLocation;
      itsLastMouseClick = Point2D<int>(-1,-1);
    }else if(centerButton.handle_events(itsLastMouseClick))
    {
      LINFO("You Click Center Button");
      int dx = itsLastMouseClickOnMap.i - w/2;
      int dy = itsLastMouseClickOnMap.j - h/2;
      Point2D<int>shift(dx,dy);
      itsMapDisplayOffset += shift;
      itsLastMouseClick = Point2D<int>(-1,-1);
     
    }

  return res;
}
// ######################################################################
void BeoLocalizer::handleUserEvent()
{
  //handle clicks
  const nub::soft_ref<ImageDisplayStream> ids =
    itsOfs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("BeoLocalizer::evolve")
    : rutz::shared_ptr<XWinManaged>();

  if (uiwin.is_valid())
    {
      int key = uiwin->getLastKeyPress();
      //compute the offset for zoom shift
      double shiftX = (double)(itsGoogleMap.getWidth()) *0.05/itsTopologicalMapZoom;
      double shiftY = (double)(itsGoogleMap.getHeight())*0.05/itsTopologicalMapZoom/2.0;
      if(key != -1)
        LINFO("key %i", key);
      switch(key)
        {
        case -1:
          break;
        case 13: //g (MAC)
        case 42: //g 
          break;
        case 11: //f (mac)
        case 41: //f 
          break;
        case 10: //d (mac) 
        case 40: //d 
          break;
        case 9:  //s (mac) 
        case 39: //s 
          itsSimulationMode = !itsSimulationMode;
          break;
        case  8: //a (mac) 
        case 38: //a 
          break;

        //left
        case 12: //h (mac) 
        case 43: //h 
          itsTopologicalMapOffset.i --;
          LINFO("GmapOffset(%d,%d)",itsTopologicalMapOffset.i,itsTopologicalMapOffset.j);
          break;        
        //down
        //case 12: //j (mac) 
        case 44: //j 
        //case 46: //j (mac) 
          itsTopologicalMapOffset.j++;
          LINFO("GmapOffset(%d,%d)",itsTopologicalMapOffset.i,itsTopologicalMapOffset.j);
          break;
        //up
        case 45://k
          itsTopologicalMapOffset.j--;
          LINFO("GmapOffset(%d,%d)",itsTopologicalMapOffset.i,itsTopologicalMapOffset.j);
          break;
        //right
        case 46://L (J on MAC)
          itsTopologicalMapOffset.i++;
          LINFO("GmapOffset(%d,%d)",itsTopologicalMapOffset.i,itsTopologicalMapOffset.j);
          break;

        //zoom out topomap
        case 47://; 
          itsTopologicalMapZoom += 0.05;
          LINFO("tzoom %f",itsTopologicalMapZoom);
          break;
        //zoom in topomap
        case 48://'
          itsTopologicalMapZoom -= 0.05;
          LINFO("tzoom %f",itsTopologicalMapZoom);
          break;


        case 57: //space (mac)(n on PC) 
        case 65: //space pause/resume
          break;
        case 14: //z (mac)
        case 52: //z 
	  itsMapDisplayZoom-=0.05;
	  if(itsMapDisplayZoom <= 0.0)itsMapDisplayZoom = 0.01;
	  //shift map to stay in the center
	  itsMapDisplayOffset.i -= shiftX;
	  itsMapDisplayOffset.j -= shiftY;
          break;
        case 15: //x (mac)
        case 53: //x 
	  itsMapDisplayZoom+=0.05;
	  //shift map to stay in the center
	  itsMapDisplayOffset.i += shiftX;
	  itsMapDisplayOffset.j += shiftY;
          break;
        case 16: //c (mac) 
        case 54: //c 
          // reset map to default
          itsMapDisplayZoom = MAP_DISPLAY_Z;
          itsMapDisplayOffset = Point2D<int>(MAP_DISPLAY_X,MAP_DISPLAY_Y);
          break;
        case 17: //v (mac)
        case 55: //v 
          break;
        case 19: //b (mac) 
        case 56: //b 
          break;
        case 20: //q (mac) 
        case 24: //q 
          break;
        case 21: //w (mac) 
        case 25: //w 
          break;
        case 133: //arrow down(mac)
        case 116: //arrow down
        case 104: //arrow down(beokeyboard) 
	  itsMapDisplayOffset.j-=3;
	  if(itsMapDisplayOffset.j < 0) itsMapDisplayOffset.j = 0;
	  LINFO("pan(%d,%d) zoom %f",
                itsMapDisplayOffset.i,itsMapDisplayOffset.j,itsMapDisplayZoom);
          break;
        case 134: //arrow up(mac) 
        case 111: //arrow up 
        case 98: //arrow up(beokeyboard) 
	  itsMapDisplayOffset.j+=3;
	  LINFO("pan(%d,%d) zoom %f",
                itsMapDisplayOffset.i,itsMapDisplayOffset.j,itsMapDisplayZoom);
          break;
        case 131: //arrow left(mac) 
        case 100: //arrow left,BeoKeyboard
        case 113: //arrow left
	  itsMapDisplayOffset.i-=3;
	  if(itsMapDisplayOffset.i < 0) itsMapDisplayOffset.i = 0;
	  LINFO("pan(%d,%d) zoom %f",
                itsMapDisplayOffset.i,itsMapDisplayOffset.j,itsMapDisplayZoom);
          break;
        case 132: //arrow right(mac) 
        case 102: //arrow right,BeoKeyboard
        case 114: //arrow right 
	  itsMapDisplayOffset.i+=3;
	  LINFO("pan(%d,%d) zoom %f",
                itsMapDisplayOffset.i,itsMapDisplayOffset.j,itsMapDisplayZoom);
          break;
          //case 39: //o (mac),conflict with Linux 's' key
        case 32: //o 

          break;
        default:		
          LINFO("key %i", key);
          break;
        }

      Point2D<int> motionPos = uiwin->getLastMouseMotion();
      if(motionPos.isValid())
      {
        itsLastMouseStayCount = 0;
        itsLastMouseStay = motionPos;
        //LINFO("mouse drag (%d,%d)",motionPos.i,motionPos.j);
      }else{
        if(itsLastMouseStayCount < 100) itsLastMouseStayCount++;
        //LINFO("Last mouse loc (%d,%d) count: %3d",
        //    itsLastMouseStay.i,itsLastMouseStay.j,itsLastMouseStayCount);
      }



      Point2D<int> pos = uiwin->getLastMouseClick();
      if (pos.isValid())
        {
	  itsLastMouseClick = pos;
          PixRGB<byte> pixColor = itsDispImg.getVal(pos);
          LINFO("Mouse Click (%d %d) Color (%d,%d,%d)", 
                pos.i,pos.j,pixColor.red(),pixColor.green(),pixColor.blue());

          //if(itsResetGoalButton.handle_events(pos))
          //  LINFO("You Click Reset Goal Button ");
			
        } 
    }
}	

// ######################################################################
Point2D<int> BeoLocalizer::screenToTopogical(Point2D<int> pt)
{
      //shift and zoom to TopologicalMap drawing coord (Pt/Z)+M-G
      Point2D<double> mapCoordPt = 
        (Point2D<double>(pt+itsMapDisplayOffset)/itsMapDisplayZoom);

      //convert again to topological map coord
      Point2D<int> topoPt= 
        Point2D<int>((mapCoordPt.i-itsTopologicalMapOffset.i-0.5)/itsTopologicalMapZoom, 
                     (mapCoordPt.j-itsTopologicalMapOffset.j-0.5)/itsTopologicalMapZoom);
      return topoPt;

}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
