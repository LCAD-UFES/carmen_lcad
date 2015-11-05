/*!@file Robots/Beobot2/Navigation/BeoNavigator.C integrates various
 vision, LRF, and encoder data to create and navigate using a single
 local grid occupancy map. We use A* shortest path algorithm and takes
 into account the closest obstacle to generate the motor command        */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/BeoNavigator.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/BeoNavigator.H"
#include "Robots/Beobot2/BeoCommon.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"   // for luminance()
#include "Image/ShapeOps.H"   // for rescale()
#include "Image/MathOps.H"    // for stdev()
#include "Image/MatrixOps.H"  // for matrixMult()
#include "Image/CutPaste.H"   // for inplacePaste()
#include "Transport/FrameInfo.H" //for ImageDisplayStream
#include "GUI/ImageDisplayStream.H"

#include "Util/Timer.H"

#include "Ice/IceImageUtils.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
 
// ######################################################################
BeoNavigator::BeoNavigator(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName)
  :
  RobotBrainComponent(mgr, descrName, tagName),
  itsLocalMap
  (new LocalMap(LOCAL_MAP_NUM_HORZ_GRIDS, LOCAL_MAP_NUM_VERT_GRIDS, 
                LOCAL_MAP_GRID_WIDTH, LOCAL_MAP_GRID_HEIGHT)),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsCurrentMessageID(0)
{
  // odometry input
  itsCurrentGlobalHeading = 0.0;
  itsDiffPosition         = Point2D<double>(0.0, 0.0);

  // display related variables
  // we will display the first frame
  itsLastUpdateIndex  = -1;
  itsLastDisplayIndex = -2;
  itsTimer.reset();

  //itsOfs->addFrameDest("display");//Add default --out=display
  addSubComponent(itsOfs);
}

// ######################################################################
BeoNavigator::~BeoNavigator()
{ }

// ######################################################################
void BeoNavigator::start1()
{ }

// ######################################################################
void BeoNavigator::registerTopics()
{
  // subscribe to odometry, IMU, Laser
  this->registerSubscription("MotorMessageTopic");
  this->registerSubscription("GPSMessageTopic");
  //this->registerSubscription("IMUMessageTopic");
  this->registerSubscription("LRFMessageTopic");
  this->registerSubscription("TraversalMapMessageTopic");
  this->registerSubscription("GoalLocationRequestTopic");
  this->registerSubscription("ResetRequestTopic");

  // later vision:
  // this->registerSubscription("LandmarkTrackMessageTopic");
  // this->registerSubscription("LandmarkDBSearchResultMessageTopic");
  // this->registerSubscription("CurrentLocationMessageTopic");
  // this->registerSubscription("CornerMotorRequestTopic");


  // sends out the screen shot to BeoVisualizer
  //this->registerPublisher("VisualizerMessageTopic");


  // sends out motor command
  this->registerPublisher("MotorRequestTopic");

  this->registerPublisher("ResetRequestTopic");

}

// ######################################################################
void BeoNavigator::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  Timer timer(1000000);

  // motor Encoder message
  // NOTE: measured about 15fps.
  if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
    {
      BeobotEvents::MotorMessagePtr mtrMsg =
        BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);
 
      // store the information
      its_Curr_Mtr_mutex.lock();
      itsCurrentIMUHeading    = mtrMsg->imuHeading;
      itsCurrentGlobalHeading = mtrMsg->encoderOri;
      itsDiffPosition.i       = mtrMsg->encoderX;
      itsDiffPosition.j       = mtrMsg->encoderY;

      // RC related caps
      double transCap         = mtrMsg->rcTransCap;
      double rotCap           = mtrMsg->rcRotCap;
      double currentTransVel  = mtrMsg->robotTransVel;
      double currentRotVel    = mtrMsg->robotRotVel;

      its_Curr_Mtr_mutex.unlock();


      double heading = mtrMsg->imuHeading; //FIXXX PUT BACK
      if(heading <= -2.0*M_PI) heading = mtrMsg->encoderOri; //FIXXX PUT BACK

      LINFO("imu: %f h: %f rh: %f", mtrMsg->imuHeading, mtrMsg->encoderOri, mtrMsg->rawEncoderOri );

      //LINFO("MotorMsg[%6d]: %5.2f %5.2f IMU: %5.2fd enc head: %5.2fd", 
      //      mtrMsg->RequestID,  mtrMsg->imuHeading, mtrMsg->encoderOri,
      //      mtrMsg->imuHeading/M_PI*180.0F, mtrMsg->encoderOri/M_PI*180.0F);

      // update map using odometry
      its_Local_Map_mutex.lock();
      itsLocalMap->updateLocalMap
        (heading, itsDiffPosition,
         transCap, rotCap, currentTransVel, currentRotVel);
      Beobot2::MotorCommand cmd = itsLocalMap->getNavigationCommand();
      its_Local_Map_mutex.unlock();
      // LINFO("MOT time: t1: %6.2f", timer/1000.0F);

      updateMotor(cmd.translation, cmd.rotation);

      its_Update_Index_mutex.lock();
      itsLastUpdateIndex++;
      its_Update_Index_mutex.unlock();
    }

  // GPS message                    
  else if(eMsg->ice_isA("::BeobotEvents::GPSMessage"))
    {
      // we got GPS data                                                   
      BeobotEvents::GPSMessagePtr gpsMsg =
        BeobotEvents::GPSMessagePtr::dynamicCast(eMsg);
      
      int currRequestID = gpsMsg->RequestID;
      LINFO("GPSMessage[%6d]", currRequestID);
      
      //double lat = gpsMsg->latitude;
      //double lon = gpsMsg->longitude;
      //double xin = gpsMsg->x;                          
      //double yin = gpsMsg->y;      

      // FIXXX: indicate whether it is close to an intersection

      // FIXXX: need to update the local map as well

      // probably can be done by a localizer
      // who has a shape of the environment

      its_Update_Index_mutex.lock();
      itsLastUpdateIndex++;
      its_Update_Index_mutex.unlock();
    }

  // LRF message                                                  
  else if(eMsg->ice_isA("::BeobotEvents::LRFMessage"))
  {
    timer.reset();

    // we got LRF data                                                   
    BeobotEvents::LRFMessagePtr lrfMsg =
      BeobotEvents::LRFMessagePtr::dynamicCast(eMsg);

    //int currRequestID = lrfMsg->RequestID;
    //LINFO("LRFMessage[%6d]", currRequestID);      

    std::vector<double> distances = lrfMsg->distances;
    std::vector<double> angles    = lrfMsg->angles;

    its_Curr_Lrf_mutex.lock();
    itsDistances = distances;
    itsAngles    = angles;
    its_Curr_Lrf_mutex.unlock();

    // update the localmap and compute navigation 
    timer.reset();
    its_Local_Map_mutex.lock();
    itsLocalMap->updateLocalMap(distances, angles);
    Beobot2::MotorCommand cmd = itsLocalMap->getNavigationCommand(); 
    its_Local_Map_mutex.unlock();
    //LINFO("LRF time: %f", timer/1000.0F);
  
    updateMotor(cmd.translation, cmd.rotation);

    its_Update_Index_mutex.lock();
    itsLastUpdateIndex++;
    its_Update_Index_mutex.unlock();

    // FIXXX need person detector from laser
  }

  // Traversal Map message
  else if(eMsg->ice_isA("::BeobotEvents::TraversalMapMessage"))
  {
    // skip if currently in turning mode
    its_Local_Map_mutex.lock();
    bool inTurnProcedure = itsLocalMap->isInTurnProcedure();
    its_Local_Map_mutex.unlock();
    if(inTurnProcedure) return;

    timer.reset();

    // we got a traversal map data
    // NOTE: Calibration should already be done on the sender side
    BeobotEvents::TraversalMapMessagePtr tmapMsg =
      BeobotEvents::TraversalMapMessagePtr::dynamicCast(eMsg);

    Image<float> tmap = Ice2Image<float>(tmapMsg->tmap);

    its_Traversal_Map_mutex.lock();
    itsTraversalMapIndex     = tmapMsg->RequestID;
    itsTraversalMap          = tmap;
    itsEstimatedRobotHeading = tmapMsg->heading; 
    itsLateralDeviation      = tmapMsg->lateralDeviation;
    its_Traversal_Map_mutex.unlock();
    
    // update the localmap and compute navigation 
    its_Local_Map_mutex.lock();
    itsLocalMap->updateLocalMap(tmap, tmapMsg->heading, tmapMsg->lateralDeviation);
    //Beobot2::MotorCommand cmd = itsLocalMap->getNavigationCommand();
    its_Local_Map_mutex.unlock();
  
    //updateMotor(cmd.translation, cmd.rotation);

    its_Update_Index_mutex.lock();
    itsLastUpdateIndex++;
    its_Update_Index_mutex.unlock();
    //LINFO("TMA time: %f", timer.get()/1000.0F);
  }

  // change goal location
  else if(eMsg->ice_isA("::BeobotEvents::GoalLocationRequest"))
    {
      timer.reset();
      
      // we got new goal location request                     
      BeobotEvents::GoalLocationRequestPtr glrMsg =
        BeobotEvents::GoalLocationRequestPtr::dynamicCast(eMsg);      

      Point2D<int> goalNode(glrMsg->goalLoc.i, glrMsg->goalLoc.j);          
      its_Local_Map_mutex.lock();
      itsLocalMap->setGoalNode(goalNode);
      its_Local_Map_mutex.unlock();

      if(glrMsg->goalType == BEONAVIGATOR_TURN_GOAL)
        {
          LINFO("Turn[%4d %4d] to (%d, %f)", 
                goalNode.i, goalNode.j, glrMsg->snum, glrMsg->ltrav);
        }
      else if(glrMsg->goalType == BEONAVIGATOR_STATIONARY_TARGET_GOAL)
        {
          LINFO("Stationary goalNode[%4d %4d] of ??? shape", 
                goalNode.i, goalNode.j);
        }
      else if(glrMsg->goalType == BEONAVIGATOR_MOVING_TARGET_GOAL)
        {
          LINFO("Moving     goalNode[%4d %4d] of ??? shape moving ???", 
                goalNode.i, goalNode.j);
        }

      its_Update_Index_mutex.lock();
      itsLastUpdateIndex++;
      its_Update_Index_mutex.unlock();
    }

  // reset request
  else if(eMsg->ice_isA("::BeobotEvents::ResetRequest"))
    {
      BeobotEvents::ResetRequestPtr resetMsg =
        BeobotEvents::ResetRequestPtr::dynamicCast(eMsg);
      LINFO("Got Reset Request %d",resetMsg->ResetID);

      // if message not sent by itself
      if(resetMsg->RequestAppID != BEO_NAVIGATOR)
        {
          if(resetMsg->ResetID == BEO_ALL||resetMsg->ResetID == BEO_NAVIGATOR)
            {
              reset();
            }
        }
    }



  // NOTE: FIXXX: goal location should be in metric for ease of use 
  // caller is responsible in updating this information properly 
  // as the robot is moving
  // we can do smart laser tracking of figuring 
  // where this person is going to

  // but we will also move the goal as the robot is moving
  // if it's within the local map bounds

  // FIXXX: global heading needs to be accounted in the desired 
  // have to think about who gives the command to BeoNavigator to here
  // (should be localizer, after a directive from app-BeoTaskManager)  
  // think about how to stop at goal destination and when to turn 
  // after turn need to reset to zero again


  // this is probably a blocking open-loop operation
  
  // but also need to be careful about
  // not overcommitting too quickly

  // IMPORTANT: need more visual feedback during turns
  // check with the inputs that are matched
  // see if they are from the next segment
  
  // have to consider approach to moving things


  // landmark (GSNav detector)
  // FIXXX this should be the difference in pix locs for all matches
  // it has a heading information 
  // but mostly it is a lateral difference information
  // to take out the steady state information
  // ==> think about also tracking the found landmarks
  

  // FIXXX: vanishing point detector: just a heading indicator
  //  separate than TraversalMapMessage

  // 3D representation ???
}

// ######################################################################
void BeoNavigator::evolve()
{
  // draw the current local and immediate map
  its_Update_Index_mutex.lock();
  int lastUpdateIndex     = itsLastUpdateIndex;
  its_Update_Index_mutex.unlock();

  // add check to not draw when there is no update
  if(lastUpdateIndex != itsLastDisplayIndex)
    {
      Timer timer(1000000);
      
      // get all the necessary information
      its_Local_Map_mutex.lock();

      double gridWidth  = itsLocalMap->getGridWidth();
      double gridHeight = itsLocalMap->getGridHeight();

      Image<double> gridOccupancyMap = 
        itsLocalMap->getGridOccupancyMap();
      Image<double> immediateGridOccupancyMap =
        itsLocalMap->getImmediateGridOccupancyMap();

      std::vector<double> distances = itsLocalMap->getDistances();
      std::vector<double> angles    = itsLocalMap->getAngles();

      std::vector<double> immediateDistances =
        itsLocalMap->getImmediateDistances();
      std::vector<double> immediateAngles =
        itsLocalMap->getImmediateAngles();

      std::vector<Point3D<double> >predictedNextLocations =
        itsLocalMap->getPredictedNextLocations();

      Point2D<double> currAccTrajectory =
        itsLocalMap->getCurrAccTrajectory();

      std::vector<Point2D<float> > path =
        itsLocalMap->getPath();

      std::vector<Point2D<float> > smoothedPath =
        itsLocalMap->getSmoothedPath();

      double desiredHeading =
        itsLocalMap->getDesiredHeading();
      
      double selfLocalHeading =
        itsLocalMap->getSelfLocalHeading();

      Beobot2::MotorCommand currentMotorCommand =
        itsLocalMap->getCurrentMotorCommand();

      double currentTransVelCommand =
        itsLocalMap->getCurrentTransVelCommand();

      double currentRotVelCommand =
        itsLocalMap->getCurrentRotVelCommand();

      std::vector<Point2D<int> > goalNodes =
        itsLocalMap->getGoalNodes();

      Point2D<int> goalNode =
        itsLocalMap->getGoalNode();

      std::vector<Point2D<double> >centers = itsLocalMap->getDWcenters();

      int dead_end_indicator = itsLocalMap->getDeadEndIndicator();

      Image<float> histogram = itsLocalMap->getHeadingHistogram();

      double curr_global_heading = itsLocalMap->getCurrentGlobalHeading();

      double road_global_heading = itsLocalMap->getRoadGlobalHeading();

      double road_confidence = itsLocalMap->getRoadGlobalHeadingConfidence();

      float lateral_deviation = itsLocalMap->getLateralDeviation();

      float lateral_deviation_angle = itsLocalMap->getLateralDeviationAngle();

      its_Local_Map_mutex.unlock();

      //uint64 time1 = timer.get();

      //LINFO("curr gheading: %f road gheading: %f", curr_global_heading, road_global_heading);

      std::vector<float> obstacleDist = itsLocalMap->getClosestObjDistOnPath(); 

      Image<PixRGB<byte> > localMapImage = 
        getLocalMapImage
        (gridWidth, gridHeight, 
         gridOccupancyMap, immediateGridOccupancyMap,
         distances, angles, immediateDistances, immediateAngles,
         predictedNextLocations,
         currAccTrajectory,
         path, smoothedPath,
         desiredHeading, selfLocalHeading,
         currentMotorCommand, currentTransVelCommand, currentRotVelCommand,
         goalNodes, goalNode, centers, 
         dead_end_indicator, histogram,
         curr_global_heading, road_global_heading, road_confidence,
         lateral_deviation, lateral_deviation_angle,obstacleDist);

      //LINFO("display[%d]: %f", lastUpdateIndex, time1/1000.0F);
      std::string text = sformat("[%6d]", lastUpdateIndex);
      writeText(localMapImage, Point2D<int>(0,0),
                text.c_str(),PixRGB<byte>(255,0,0),PixRGB<byte>(0,0,0),
                SimpleFont::FIXED(12), true);

      itsOfs->writeRGB
        (localMapImage, "BeoNavigator::evolve", FrameInfo("BNav",SRC_POS));

      
      itsDispImg = localMapImage; 
      handleUserEvent();

      //uint64 time2 = timer.get();
      //LINFO("time2: %f", (time2 -time1)/1000.0F);      
      //Raster::waitForKey();

      itsLastDisplayIndex = lastUpdateIndex;
      itsDebugWin.reset();
    }
}

// ######################################################################
Image<PixRGB<byte> > BeoNavigator::getLocalMapImage
( double gridWidth, double gridHeight, 
  Image<double> gridOccupancyMap, Image<double> immediateGridOccupancyMap,
  std::vector<double> distances, std::vector<double> angles,
  std::vector<double> immediateDistances,
  std::vector<double> immediateAngles,
  std::vector<Point3D<double> >predictedNextLocations,
  Point2D<double> currAccTrajectory,
  std::vector<Point2D<float> > path, 
  std::vector<Point2D<float> > smoothedPath,
  double desiredHeading, double selfLocalHeading,
  Beobot2::MotorCommand currentMotorCommand,
  double currentTransVelCommand, double currentRotVelCommand,
  std::vector<Point2D<int> > goalNodes,
  Point2D<int> goalNode,
  std::vector<Point2D<double> >centers, 
  int dead_end_indicator, Image<float> histogram,
  double curr_global_heading, double road_global_heading, 
  double road_confidence,
  float lateral_deviation, float lateral_deviation_angle,
  std::vector<float> obstacleDist 
) 
{
  // find the proper display image ratio
  float ratioW = float(gridWidth)  / float(LOCAL_MAP_DISPLAY_GRID_SIZE);
  float ratioH = float(gridHeight) / float(LOCAL_MAP_DISPLAY_GRID_SIZE);
  float ratio  = (ratioW > ratioH ? ratioW : ratioH);
  
  uint w = gridOccupancyMap.getWidth();
  uint h = gridOccupancyMap.getHeight();

  Image<PixRGB<byte> > disp(int(w * gridWidth /ratio), 
                            int(h * gridHeight/ratio), ZEROS);
  uint dispw = disp.getWidth();
  uint disph = disp.getHeight();
  LDEBUG("ratioW: %f: gw: %f || ratioH: %f gh: %f|| image size: %d %d",
         ratioW, gridWidth, ratioH, gridHeight, dispw,disph);

  uint wim = immediateGridOccupancyMap.getWidth();
  uint him = immediateGridOccupancyMap.getHeight();
  int disp2w = int(wim * IMMEDIATE_MAP_DISPLAY_GRID_SIZE);
  int disp2h = int(him * IMMEDIATE_MAP_DISPLAY_GRID_SIZE);
  Image<PixRGB<byte> > disp2(disp2w, disp2h, ZEROS);

  // define drawing colors
  PixRGB<byte> black (  0,  0,  0);
  PixRGB<byte> red   (255,  0,  0);
  PixRGB<byte> blue  (  0,  0,255);
  PixRGB<byte> green (  0,255,  0);
  PixRGB<byte> yellow(255,255,  0); 
  PixRGB<byte> orange(255,128,  0);
  PixRGB<byte> cyan  (  0,255,255);

  PixRGB<byte> unknown_display_color(127,64,64);
 
  // draw the grid occupancy belief
  for(uint i = 0; i < w; i++)
    for(uint j = 0; j < h; j++)
      {
        // get the rectangle
        uint le = uint(float(i  )/float(w) * dispw);
        uint ri = uint(float(i+1)/float(w) * dispw);
        uint to = uint(float(j  )/float(h) * disph);
        uint bo = uint(float(j+1)/float(h) * disph);
        LDEBUG("lrtb: %d %d %d %d", le, ri, to, bo);
        Rectangle r = Rectangle::tlbrO(to,le,bo,ri);

        double val  = gridOccupancyMap.getVal(i,j);        
        byte   ival = byte(val * 255.0);
        drawFilledRect(disp, r, PixRGB<byte>(ival,ival,ival));
        if(val == LOCAL_MAP_DEFAULT_GRID_VALUE)
          drawFilledRect(disp, r, unknown_display_color);
      }

  // draw the grid occupancy lines in the coarse map
  for(uint i = 0; i < w; i++)
    {
      uint ii = uint(float(i)/float(w)*dispw);
      drawLine
        (disp, Point2D<int>(ii, 0), Point2D<int>(ii, disph-1), black);
    }
  drawLine
    (disp, Point2D<int>(dispw-1,0), Point2D<int>(dispw-1,disph-1), black);
  for(uint j = 0; j < h; j++)
    {
      uint jj = uint(float(j)/float(h)*dispw);
      drawLine
        (disp, Point2D<int>(0,jj), Point2D<int>(dispw-1,jj), black);
    }
  drawLine
    (disp, Point2D<int>(0,disph-1), Point2D<int>(dispw-1,disph-1), black);

  // draw the immediate grid occupancy belief
  for(uint i = 0; i < wim; i++)
    for(uint j = 0; j < him; j++)
      {
        // get the rectangle
        uint le = uint(float(i  )/float(wim) * disp2w);
        uint ri = uint(float(i+1)/float(wim) * disp2w);
        uint to = uint(float(j  )/float(him) * disp2h);
        uint bo = uint(float(j+1)/float(him) * disp2h);
        LDEBUG("lrtb: %d %d %d %d", le, ri, to, bo);
        Rectangle r = Rectangle::tlbrO(to,le,bo,ri);

        double val  = immediateGridOccupancyMap.getVal(i,j);        
        byte   ival = byte(val * 255.0);
        drawFilledRect(disp2, r, PixRGB<byte>(ival,ival,ival));

        // if(Point2D<int>(i,j) == closestObstacle)
        //   drawFilledRect(disp2, r, red);

        if(val == LOCAL_MAP_DEFAULT_GRID_VALUE)
          drawFilledRect(disp2, r, unknown_display_color);
      }  

  // // draw the immediate grid occupancy lines
  // for(uint i = 0; i < wim; i++)
  //   {
  //     //uint ii = uint(float(i)/float(wim)*disp2w);
  //     //drawLine
  //     //  (disp2, Point2D<int>(ii, 0), Point2D<int>(ii, disp2h-1), black);
  //   }
  // //  drawLine
  // //  (disp2, Point2D<int>(disp2w-1,0), Point2D<int>(dispw-1,disp2h-1), black);
  // for(uint j = 0; j < him; j++)
  //   {
  //     //uint jj = uint(float(j)/float(him)*disp2w);
  //     //  drawLine
  //     //  (disp2, Point2D<int>(0,jj), Point2D<int>(disp2w-1,jj), black);
  //   }
  // //  drawLine
  // // (disp2, Point2D<int>(0,disp2h-1), Point2D<int>(disp2w-1,disp2h-1), black);

  // current location
  Point2D<float> currentLocation(wim/2.0, him*0.75);//in pixel 
  float cx2 = currentLocation.i/float(wim)*disp2w;
  float cy2 = currentLocation.j/float(wim)*disp2h;
  Point2D<int> robotMapCLoc(cx2 +.5, cy2 +.5);
  drawDisk(disp2, robotMapCLoc, 6, blue);

  float c_to_i_w_ratio = float(LOCAL_MAP_GRID_WIDTH )/float(IMMEDIATE_MAP_GRID_WIDTH );
  float c_to_i_h_ratio = float(LOCAL_MAP_GRID_HEIGHT)/float(IMMEDIATE_MAP_GRID_HEIGHT);    

  float gridW2 = float(disp2w)/float(wim);
  float gridH2 = float(disp2h)/float(wim);

  double imw = IMMEDIATE_MAP_WIDTH/1000.0;
  double imh = IMMEDIATE_MAP_HEIGHT/1000.0;

  // draw predicted next locations
  for(uint p = 0;p < predictedNextLocations.size();p++)
    {
      // next location in immediate map pixel coord
      Point3D<double> tloc = predictedNextLocations[p];
      Point2D<int> next_loc = 
        Point2D<int>( wim/2.00 - tloc.y/imh*wim +.5, 
                      him*0.75 - tloc.x/imw*him +.5 );		

      Point2D<int> robotMapPNLoc
        (next_loc.i/float(wim)*disp2w +.5,
         next_loc.j/float(wim)*disp2h +.5 );
      drawDisk(disp2, robotMapPNLoc, 8, green);
    }

  // draw the motion trajectory
  for(uint c = 0;c < centers.size();c++)
    {
      // radius in grid units
      Point2D<double> center = centers[c];
      double radius         = fabs(center.j/.03);

      // radius in pixels
      int display_radius = int(radius/float(wim)*disp2w);
      
      center.i = currentLocation.i - center.j/.03;
      center.j = currentLocation.j;

      drawDisk(disp2, Point2D<int>(center.i*gridW2+.5, center.j*gridH2+.5), 8, red);

      if(display_radius > 5000)     
        {
          drawLine(disp2, 
                   Point2D<int>(robotMapCLoc.i, 0), 
                   Point2D<int>(robotMapCLoc.i, disp2.getHeight()-1), orange,2);
        }
      else
        {
          if((center.i*gridW2+0.5) < robotMapCLoc.i)
            {
              drawArc
                (disp2, Point2D<int>(center.i*gridW2+.5,center.j*gridH2+.5), 
                 radius*gridW2, orange,M_PI/2.0,M_PI,2);
            }
          else
            {
              drawArc
                (disp2, Point2D<int>(center.i*gridW2+.5,center.j*gridH2+.5), 
                 radius*gridW2, orange,0.0,M_PI/2.0,2);
            }
        }
    }

  int gridW = int(LOCAL_MAP_GRID_WIDTH/ratio);
  int gridH = int(LOCAL_MAP_GRID_HEIGHT/ratio);

  // center of display map
  float cmapx = dispw/2.0F;
  float cmapy = disph/2.0F+disph/4.0F;

  // current robot location
  //float cx = cmapx - currAccTrajectory.j/ratio + gridW/2.0;
  //float cy = cmapy - currAccTrajectory.i/ratio + gridH/2.0;
  float cx = cmapx + gridW/2.0;
  float cy = cmapy + gridH/2.0;

  // LINFO("cmapx: %f cmapy: %f ctraj: %f %f -->  %f %f --> cx: %f cy: %f",
  //        cmapx,cmapy, currAccTrajectory.i, currAccTrajectory.j,
  //        currAccTrajectory.i/ratio, currAccTrajectory.j/ratio,cx,cy);


  // draw the robot location
  drawDisk  (disp, Point2D<int>(cx,cy), 4, red);
  drawCircle(disp, Point2D<int>(cx,cy), 6, red);

  // draw the laser points
  std::vector<double>::iterator dptr = distances.begin();
  std::vector<double>::iterator stop = distances.end();
  int dang = int(selfLocalHeading /M_PI* 180);
  int angle = -141+dang;
  while(dptr != stop)
    {
      double dist = *dptr++;

      // only draw front laser
      if(angle >= -90 && angle <= 90)
        {
          float  rad  = dist/ratio; if (rad < 0) rad = 1.0;
          
          Point2D<int> pt;
          pt.i = int(cx - rad*sin((double)angle*M_PI/180.0)+.5); 
          pt.j = int(cy - rad*cos((double)angle*M_PI/180.0)+.5);
          
          //drawLine  (disp, cpt, pt, blue);0
          drawCircle(disp, pt,  2,  yellow);      
        }
      angle++;
    }

  // // draw the immediate laser points
  // std::vector<double>::iterator idptr = immediateDistances.begin();
  // std::vector<double>::iterator iaptr = immediateAngles.begin();
  // std::vector<double>::iterator istop = immediateDistances.end();
  // while(idptr != istop)
  //   {
  //     double dist = *idptr++;
  //     double ang  = *iaptr++; ang+= dang;
  //     float  rad  = dist/ratio; if (rad < 0) rad = 1.0;

  //     Point2D<int> pt;
  //     pt.i = cx - int(rad*sin((double)ang*M_PI/180.0)); 
  //     pt.j = cy - int(rad*cos((double)ang*M_PI/180.0));
 
  //     drawCircle(disp, pt,  2,  red);      
  //     if(closestLRFObstacle == ang-dang) drawCircle(disp, pt,  4,  green);
  //   }

  // draw robot body and its immediate region in the coarse map
  // rotate about front center of the robot body
  int probotW = int(BEOBOT2_ROBOT_WIDTH/ratio);
  int immMapW = int(IMMEDIATE_MAP_WIDTH/ratio); //3m width
  Rectangle body = 
    Rectangle(Point2D<int>(cx-probotW/2,cy), Dims(probotW, probotW));
  Point2D<float> pivot(body.left()+body.width()/2.0F, body.top());
  drawAngledRectangle(body, pivot, selfLocalHeading, disp, red);
  Rectangle immRect = 
    Rectangle(Point2D<int>(cx-immMapW/2,cy-3.0*immMapW/4.0), 
              Dims(immMapW, immMapW));
  drawAngledRectangle(immRect, pivot, selfLocalHeading, disp, red);

  // setup the full display image
  int maxHeight = int((h+2)* gridHeight/ratio);
  if(maxHeight < disp2h) maxHeight = disp2h;


  int whist = histogram.getWidth();
  int hhist = histogram.getHeight();
  int hscale = (maxHeight/hhist);

  if(hhist > maxHeight) { hscale = 1; maxHeight = hhist; }
  Image<PixRGB<byte> > displayImage    
    (int((w+2)*gridWidth /ratio) + disp2w + whist*hscale, maxHeight, ZEROS);

  Image<PixRGB<byte> > thistogram(histogram);
  inplacePaste
    (displayImage, disp, Point2D<int>(gridWidth/ratio, gridHeight/ratio));

  inplacePaste
    (displayImage, zoomXY(thistogram,hscale), 
     Point2D<int>(int((w+2)*gridWidth /ratio) + disp2w, 0));

  // draw the goal locations
  int rad = uint(1.0F/float(w) * disp2w);
  for(uint i = 0; i < goalNodes.size(); i++)
    {
      int gi = goalNodes[i].i+1;
      int gj = goalNodes[i].j+1;

      uint dgi = uint(float(gi+.5)/float(w) * dispw);
      uint dgj = uint(float(gj+.5)/float(h) * disph);

      drawDisk(displayImage, Point2D<int>(dgi,dgj), rad, yellow);
    }

  // draw the combined goal nodes
  int ggi = goalNode.i+1;
  int ggj = goalNode.j+1;
  uint dggi = uint(float(ggi+.5)/float(w) * dispw);
  uint dggj = uint(float(ggj+.5)/float(h) * disph);
  drawDisk(displayImage, Point2D<int>(dggi,dggj), rad, green);

  // add display offset to account for 1 pixel slacks all around
  Point2D<int> dOffset(gridW,gridH);
 
  // get the path to the goal (from A*) to the coarse map
  Point2D<float>  curr_loc(cx + gridWidth /ratio, cy + gridHeight/ratio);
  float   cdi  = 0   , cdj  = 0;
  if(path.size() > 0)
    {
      for(uint i = 0; i < path.size()-1; i++)
        {
          Point2D<float> step = path[i+1] - path[i];
          float di = step.i; float dj = step.j;                   
          Point2D<float> next_loc(curr_loc.i+gridW*di, curr_loc.j+gridH*dj);
          
          Point2D<int> p1(curr_loc.i+.5, curr_loc.j+.5);
          Point2D<int> p2(next_loc.i+.5, next_loc.j+.5);
          drawLine(displayImage, p1, p2, green, 2);
          cdi += di; cdj += dj;
          
          curr_loc = next_loc;
        }
    }

  //itsDebugWin.show(displayImage,"A*");
  // draw the smoothed path to the coarse map
  Point2D<float> scurr_loc(cx + gridWidth /ratio, cy + gridHeight/ratio);
  float scdi = 0.0F, scdj = 0.0F;
  if(smoothedPath.size() > 0)
    {
      int ct = 0;
      for(uint i = 0; i < smoothedPath.size()-1; i++)
        {
          Point2D<float> sstep = smoothedPath[i+1] - smoothedPath[i];
          float sdi = sstep.i; float sdj = sstep.j;
          //LINFO("[%2d]here: sdij: %f %f, gridwh: %d %d", i,sdi, sdj, gridW, gridH);
          Point2D<float> snext_loc
            (scurr_loc.i + gridW*sdi, scurr_loc.j + gridH*sdj); 

          Point2D<int> p1(scurr_loc.i+.5, scurr_loc.j+.5);
          Point2D<int> p2(snext_loc.i+.5, snext_loc.j+.5);
          drawLine(displayImage, p1, p2, blue,2);



          //draw circle to indicate the bubble size
          if(ct%5 == 0)
          {
            float dist_to_obstacle = obstacleDist[i];
            float colorRatio = 1.0 - (float)i/(float)smoothedPath.size()*0.5;
            //LINFO("radius to obstacle is %f",dist_to_obstacle);
            if(dist_to_obstacle > 0.0 && dist_to_obstacle < double(IMMEDIATE_MAP_WIDTH)/double(LOCAL_MAP_GRID_WIDTH)){
              drawCircle(displayImage,p1,(int)dist_to_obstacle*gridW,PixRGB<byte>(blue*colorRatio),1);
            }else{
              ct--;
            }
          }
          ct++;
          scdi += sdi; scdj += sdj;

          scurr_loc = snext_loc;
        }
    }

  // draw robot body in the immediate map
  int probotW2 = int(gridW2*BEOBOT2_ROBOT_WIDTH /IMMEDIATE_MAP_GRID_WIDTH);
  int probotH2 = int(gridH2*BEOBOT2_ROBOT_HEIGHT/IMMEDIATE_MAP_GRID_HEIGHT);
  Rectangle rImmMap = 
    Rectangle(Point2D<int>(cx2-probotW2/2,cy2), Dims(probotW2, probotH2));
  if(disp2.rectangleOk(rImmMap)) drawRect(disp2, rImmMap, red);

  // get the path to the goal (from A*) to the immediate map
  if(path.size() > 0)
    {
      float cAng = cos(-selfLocalHeading);
      float sAng = sin(-selfLocalHeading);

      Point2D<float>  curr_loc2(cx2,cy2);
      float   cdi2  = 0   , cdj2  = 0;
      for(uint i = 0; i < path.size()-1; i++)
        {
          Point2D<float> step = path[i+1] - path[i];
          float di = step.i; float dj = step.j;                   

          Point2D<float> next_loc2
            (curr_loc2.i+c_to_i_w_ratio*gridW2*di, 
             curr_loc2.j+c_to_i_h_ratio*gridH2*dj);
          
          // rotate the path
          Point2D<int> rcl2
            (cx2 + (curr_loc2.i - cx2)*cAng + (curr_loc2.j - cy2)*sAng,
             cy2 - (curr_loc2.i - cx2)*sAng + (curr_loc2.j - cy2)*cAng );

          Point2D<int> rnl2
            (cx2 + (next_loc2.i - cx2)*cAng + (next_loc2.j - cy2)*sAng,
             cy2 - (next_loc2.i - cx2)*sAng + (next_loc2.j - cy2)*cAng );
          
          Point2D<int> p1(rcl2.i+.5, rcl2.j+.5);
          Point2D<int> p2(rnl2.i+.5, rnl2.j+.5);
          drawLine(disp2, p1, p2, green, 2);
          cdi2 += di; cdj2 += dj;
          
          curr_loc2 = next_loc2;
        }
    }

  // draw the smoothed path to the immediate map
  if(smoothedPath.size() > 0)
    {
      float cAng = cos(-selfLocalHeading);
      float sAng = sin(-selfLocalHeading);

      Point2D<float> scurr_loc2(cx2,cy2);
      float scdi2 = 0.0F, scdj2 = 0.0F;
      for(uint i = 0; i < smoothedPath.size()-1; i++)
        {
          Point2D<float> sstep = smoothedPath[i+1] - smoothedPath[i];
          float sdi = sstep.i; float sdj = sstep.j;
          //LINFO("here: sdij: %f %f, gridwh: %d %d", sdi, sdj, gridW, gridH);
          Point2D<float> snext_loc2
            (scurr_loc2.i + c_to_i_w_ratio*gridW2*sdi, 
             scurr_loc2.j + c_to_i_h_ratio*gridH2*sdj); 

          // rotate the path
          Point2D<int> rscl2
            (cx2 + (scurr_loc2.i - cx2)*cAng + (scurr_loc2.j - cy2)*sAng,
             cy2 - (scurr_loc2.i - cx2)*sAng + (scurr_loc2.j - cy2)*cAng );

          Point2D<int> rsnl2
            (cx2 + (snext_loc2.i - cx2)*cAng + (snext_loc2.j - cy2)*sAng,
             cy2 - (snext_loc2.i - cx2)*sAng + (snext_loc2.j - cy2)*cAng );
          
          Point2D<int> p1(rscl2.i+.5, rscl2.j+.5);
          Point2D<int> p2(rsnl2.i+.5, rsnl2.j+.5);
          drawLine(disp2, p1, p2, blue,2);
          scdi2 += sdi; scdj2 += sdj;

          scurr_loc2 = snext_loc2;
        }

      // emphasize the first smoothed path segment
      Point2D<float> sstep = smoothedPath[1] - smoothedPath[0];
      float sdi = sstep.i; float sdj = sstep.j;
      Point2D<int> next_loc_sp
        (robotMapCLoc.i+c_to_i_w_ratio*gridW2*sdi+ .5, 
         robotMapCLoc.j+c_to_i_h_ratio*gridH2*sdj+ .5); 

      Point2D<int> rnext_loc_sp
        (cx2 + (next_loc_sp.i - cx2)*cAng + (next_loc_sp.j - cy2)*sAng,
         cy2 - (next_loc_sp.i - cx2)*sAng + (next_loc_sp.j - cy2)*cAng );
          
      drawLine(disp2, robotMapCLoc, rnext_loc_sp, red,2);

    }

  int disp2di = dispw + 2*int(gridWidth/ratio);
  inplacePaste(displayImage, disp2, Point2D<int>(disp2di, 0));


  int cwh = 528;//corse map width height
  Image< PixRGB<byte> > visualizeImg= crop(displayImage,Point2D<int>(0,0),Dims(cwh,cwh),true); 
  //itsDebugWin.show(visualizeImg,"visualizer");
  //publishScreenShot(visualizeImg);

  // draw the robot and desired heading 
  //uint dhLen = gridWidth; 
  //Point2D<int> zoomCenter(dispw/2,disph*.75);
  //drawLine(displayImage, zoomCenter, selfLocalHeading+M_PI/2.0, dhLen, red,  2);
  //drawLine(displayImage, zoomCenter, desiredHeading  +M_PI/2.0, dhLen, blue, 2);

  // updates
  std::string text = 
    sformat("T: %4.2f R: %5.3f [curr: %5.3fdeg | des: %5.3fdeg]", 
            currentMotorCommand.translation, 
            currentMotorCommand.rotation,
            selfLocalHeading/M_PI*180.0F, desiredHeading/M_PI*180.0F);
  Point2D<int> tLoc(0,int(.9*disph));
  writeText(displayImage, tLoc, text.c_str(),                         
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(10));

  text = 
    sformat("optimal_v: %f optimal_w: %f", 
            currentTransVelCommand, currentRotVelCommand);
  tLoc = Point2D<int>(0,int(.95*disph));
  writeText(displayImage, tLoc, text.c_str(),                         
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(10));

  tLoc = Point2D<int>(int(dispw+2*gridWidth/ratio),int(.85*disph));
  if(dead_end_indicator == 1)
    {
      writeText(displayImage, tLoc, 
                std::string("NEAR DEAD END").c_str(),
                orange, black, SimpleFont::FIXED(20));
    }
  else if(dead_end_indicator == 2)
    {
      writeText(displayImage, tLoc, std::string("DEAD END").c_str(),
                red, black, SimpleFont::FIXED(20));
    }

  // draw a reset button	
  itsResetGoalButton = 
    Button(Point2D<int>(dispw+disp2w*0.68,disp2h*0.90),Dims(140,40));
  itsResetGoalButton.setLabel(std::string(" Reset "));
  itsResetGoalButton.setButtonBgColor(blue);
  itsResetGoalButton.show(displayImage);

  tLoc = Point2D<int>(int((w+2)*gridWidth/ratio), disp2h);
  text = sformat("IMU : %7.2fdeg", curr_global_heading/M_PI*180.0);
  writeText(displayImage, tLoc, text.c_str(),
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(16)); 

  tLoc = Point2D<int>(int((w+2)*gridWidth/ratio), disp2h + 30);
  text = sformat("road: %7.2fdeg [%4.2f]", 
                 road_global_heading/M_PI*180.0, road_confidence);
  writeText(displayImage, tLoc, text.c_str(),
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(16)); 

  tLoc = Point2D<int>(int((w+2)*gridWidth/ratio), disp2h + 60);
  text = sformat("traj: %f %f", 
                 currAccTrajectory.i, currAccTrajectory.j);
  writeText(displayImage, tLoc, text.c_str(),
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(16)); 

  tLoc = Point2D<int>(int((w+2)*gridWidth/ratio), disp2h + 90);
  text = sformat("ldev: %5.2fmm A: %5.2fdeg", 
                 lateral_deviation, lateral_deviation_angle/M_PI*180.0F);
  writeText(displayImage, tLoc, text.c_str(),
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(16)); 

  return displayImage;
}
// ######################################################################
void BeoNavigator::publishScreenShot(Image<PixRGB<byte> > img)
{
  BeobotEvents::VisualizerMessagePtr msg = 
    new BeobotEvents::VisualizerMessage;
  msg->image     = Image2Ice(img); 
  msg->BeoAppID  = BEO_NAVIGATOR;
  msg->RequestID = itsCurrentMessageID++;
  this->publish("VisualizerMessageTopic", msg);
}
// ######################################################################
std::vector<Point2D<int> > BeoNavigator::drawAngledRectangle
(Rectangle body, Point2D<float> pivot, double robotAngle,  
 Image<PixRGB<byte> > &disp, PixRGB<byte> color)
{
  // get the four corners
  Point2D<int> tl = body.topLeft();
  Point2D<int> tr = body.topRight();
  Point2D<int> bl = body.bottomLeft();
  Point2D<int> br = body.bottomRight();

  float cAng = cos(robotAngle);
  float sAng = sin(robotAngle);

  // rotate the 4 corners properly
  Point2D<int> rtl
    (pivot.i + (tl.i - pivot.i)*cAng + (tl.j - pivot.j)*sAng,
     pivot.j - (tl.i - pivot.i)*sAng + (tl.j - pivot.j)*cAng );

  Point2D<int> rtr
    (pivot.i + (tr.i - pivot.i)*cAng + (tr.j - pivot.j)*sAng,
     pivot.j - (tr.i - pivot.i)*sAng + (tr.j - pivot.j)*cAng );

  Point2D<int> rbl
    (pivot.i + (bl.i - pivot.i)*cAng + (bl.j - pivot.j)*sAng,
     pivot.j - (bl.i - pivot.i)*sAng + (bl.j - pivot.j)*cAng );

  Point2D<int> rbr
    (pivot.i + (br.i - pivot.i)*cAng + (br.j - pivot.j)*sAng,
     pivot.j - (br.i - pivot.i)*sAng + (br.j - pivot.j)*cAng );

  // draw each line
  drawLine(disp, rtl, rtr, color,1);
  drawLine(disp, rtr, rbr, color,1);
  drawLine(disp, rbr, rbl, color,1);
  drawLine(disp, rbl, rtl, color,1);

  std::vector<Point2D<int> > points;
  points.push_back(rtl);
  points.push_back(rtr);
  points.push_back(rbr);
  points.push_back(rbl);
  points.push_back(rtl);
  return points;
}

// ######################################################################
void BeoNavigator::updateMotor(double tran, double rot)
{
  BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
  msg->transVel = tran;
  msg->rotVel   = rot;
  this->publish("MotorRequestTopic", msg);
  // LINFO("[%d] Publish motor request Trans %f Rotation %f",
  //       itsPrevProcImgID,tran,rot);
}

// ######################################################################
void BeoNavigator::reset()
{
  BeobotEvents::ResetRequestPtr msg = new BeobotEvents::ResetRequest;
  msg->RequestAppID = BEO_NAVIGATOR;
  msg->ResetID      = BEO_ALL;
  this->publish("ResetRequestTopic", msg);

  its_Local_Map_mutex.lock();
  itsLocalMap.reset(new LocalMap(LOCAL_MAP_NUM_HORZ_GRIDS, 
                                 LOCAL_MAP_NUM_VERT_GRIDS, 
                                 LOCAL_MAP_GRID_WIDTH, 
                                 LOCAL_MAP_GRID_HEIGHT));
  its_Local_Map_mutex.unlock();
  itsCurrentGlobalHeading = 0.0;
  itsCurrentIMUHeading    = 0.0;
  itsDiffPosition         = Point2D<double>(0.0, 0.0);	
}

// ######################################################################
void BeoNavigator::handleUserEvent()
{
  //handle clicks
  const nub::soft_ref<ImageDisplayStream> ids =
    itsOfs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("BeoNavigator::evolve")
    : rutz::shared_ptr<XWinManaged>();

  if (uiwin.is_valid())
    {
      int key = uiwin->getLastKeyPress();

      if(key != -1)
        LINFO("key %i", key);
      switch(key)
        {
        case -1:
          break;
        case 23: //r (MAC)
        case 27: //r reset
          reset();
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
          break;
        case  8: //a (mac) 
        case 38: //a 
          break;
        case 12: //h (mac) 
        case 43: //h 
          break;
          //case 12: //j (mac) 
        case 44: //j 
        case 46: //j (mac) 
          break;
        case 57: //space (mac)(n on PC) 
        case 65: //space pause/resume
          break;
        case 14: //z (mac)
        case 52: //z 
          break;
        case 15: //x (mac)
        case 53: //x 
          break;
        case 16: //c (mac) 
        case 54: //c 
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
          break;
        case 134: //arrow up(mac) 
        case 111: //arrow up 
        case 98: //arrow up(beokeyboard) 
          break;
        case 100: //arrow left,BeoKeyboard
        case 113: //arrow left
          break;
        case 102: //arrow right,BeoKeyboard
        case 114: //arrow right 
          break;
          //case 39: //o (mac),conflict with Linux 's' key
        case 32: //o 

          break;
        default:		
          LINFO("key %i", key);
          break;
        }

      Point2D<int> pos = uiwin->getLastMouseClick();
      if (pos.isValid())
        {
          PixRGB<byte> pixColor = itsDispImg.getVal(pos);
          LINFO("Mouse Click (%d %d) Color (%d,%d,%d)", 
                pos.i,pos.j,pixColor.red(),pixColor.green(),pixColor.blue());

          if(itsResetGoalButton.handle_events(pos))
            {
              LINFO("You Click Reset Goal Button ");
              reset();
            }
        } 
    }
}	

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
