/*!@file Robots2/Beobot2/Navigation/GPS_Navigation/GPS_Navigation.C Ice Module to navigate using GPS    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/GPS_Navigation/GPS_Navigation.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/GPS_Navigation/GPS_Navigation.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#define  NUM_PREV_COORDS          10
#define  MAX_DIST                 5.0    // m
#define  PROXIMITY_DIST           0.0002 // GPS unit 
#define  RECOVERY_BUFFER_TIME     10000.0 // ms  
#define  ROBOT_WIDTH              0.6    // m

// ######################################################################
GPS_Navigation::GPS_Navigation(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  //itsOfs(new OutputFrameSeries(mgr)),
  itsDispImg(512, 512, ZEROS),
  itsTimer(1000000),
  itsNDNavigationAlgorithm(new ND_Navigation_Algorithm()),
  itsAvoidanceTimer(1000000)
{
  //addSubComponent(itsOfs);

  // set the start and goal location
  itsGoalWayPoints.push_back
    (Point2D<double>(34.0209944, -118.2875500)); // 34 1 15.58, -118 17 15.18
  itsGoalWayPoints.push_back
    (Point2D<double>(34.0208306, -118.2879556)); // 34 1 14.99, -118 17 16.64
  itsGoalWayPoints.push_back
    (Point2D<double>(34.0211944, -118.2872750)); // 34 1 16.30, -118 17 14.19
  itsGoalWayPoints.push_back
    (Point2D<double>(34.0203417, -118.2851694)); // 34 1 13.23  -118 17 06.61

  // set the heading of each pathway
  itsGoalHeadings.push_back(0.0);

  itsCurrentStartLocation = itsGoalWayPoints[0];
  itsCurrentGoalLocation  = itsGoalWayPoints[1];

  // Point2D<double> p1 = itsCurrentStartLocation;
  // Point2D<double> p2 = itsCurrentGoalLocation;
  // double heading = atan2(p2.j - p1.j, p2.i - p1.i);

  itsCurrentGoalHeading   = -100.0; //heading //itsGoalHeadings[0];
  itsAtGoal               = false;

  itsCurrNumGPScoords = 0;
  itsPrevGPScoords.resize(NUM_PREV_COORDS);

  //itsNDNavigationAlgorithm->setGoalSector(itsGoalSector);
  itsIsSafe       = true;
  itsOffPathState = 0;

  // TAKE THIS OUT
  //itsAvoidanceTrajectory = Point2D<double>(10.0, 12.0);
  //Beobot2::MotorCommand cmd = recover
  //  (M_PI/4.0, M_PI/6.0);
  //Raster::waitForKey();
  
}

// // ######################################################################
// GPS_Navigation::getLatLon
// (int lat1, int lat2 float lat3,int lon1, int lon2, int lon3)
// {

// }

// ######################################################################
GPS_Navigation::~GPS_Navigation()
{ }

// ######################################################################
void GPS_Navigation::start1()
{
}

// ######################################################################
void GPS_Navigation::registerTopics()
{
  // subscribe to GPS, LRF, IMU, and odometry value
  this->registerSubscription("GPSMessageTopic");
  this->registerSubscription("LRFMessageTopic");
  //this->registerSubscription("IMUSMessageTopic");
  this->registerSubscription("MotorMessageTopic");

  this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void GPS_Navigation::evolve()
{ 
  LINFO("itsOffPathState: %d", itsOffPathState);

  if(itsAtGoal) updateMotor(0.0, 0.0);
  else
    {
      Beobot2::MotorCommand cmd;
      cmd.translation = 0.0;
      cmd. rotation   = 0.0;
      if(itsIsSafe)
        {     
          // if we are in the middle of avoiding obstacle
          if(itsOffPathState > 0)
            {
              // accumulate the avoidance trajectory
              itsAvoidanceTrajectory += itsCurrentMovement;

              if(itsOffPathState == 1)
                {
                  itsAvoidanceTimer.reset();
                  itsOffPathState = 2;
                }
              else
                {
                  float time = itsAvoidanceTimer.get()/1000.0F;
                  
                  if(time > RECOVERY_BUFFER_TIME)
                    {
                      LINFO("recovering");

                      // time to move back to the path
                      cmd = recover
                        (itsCurrentHeading, itsCurrentGoalHeading);

                      // if we are close enough
                      if(itsDistanceToPath < ROBOT_WIDTH)
                        {
                          itsAvoidanceTrajectory = Point2D<double>(0.0, 0.0);
                          itsDistanceToPath      = 0.0;
                          itsOffPathState        = 0;
                        }
                    }
                }
            }
          // else we just use the odometry-based
          else cmd = itsHeadingCommand;
        }

      // not safe: go to LRF manuever
      else
        {
          itsOffPathState  = 1;
 
          // accumulate the avoidance trajectory
          itsAvoidanceTrajectory += itsCurrentMovement;
  
          cmd = itsNDcommand;          
        }

      // send to BeoPilot
      updateMotor(cmd.translation,cmd. rotation);
    } 
}

// ######################################################################
Beobot2::MotorCommand GPS_Navigation::recover
(double heading, double idealHeading)
{
  // compute corrective 
  double diff = heading - idealHeading;
  if(diff >  M_PI) diff = diff - 2*M_PI;
  if(diff < -M_PI) diff = diff + 2*M_PI;

  // get the ideal heading component
  double cosH = cos(idealHeading);
  double sinH = sin(idealHeading);

  double x2   = itsAvoidanceTrajectory.i;
  double y2   = itsAvoidanceTrajectory.j;
  double dist = y2*cosH - x2*sinH;

  double rotVal  = 2.0 * diff/(2*M_PI);

  double Ax = 0.0;
  double Ay = 0.0;
  double Bx = x2 + dist*sinH;
  double By = y2 - dist*cosH;

  double side = float((Bx-Ax)*(y2-Ay)-(By-Ay)*(x2-Ax) > 0.0);
  if(side == 0.0) side = -1.0;

  LINFO("side: %f", side);

  double distVal = 1.0;
  if(dist < MAX_DIST) distVal = dist/MAX_DIST;
  distVal = -side * distVal;

  LINFO("heading diff: %f --> rotVal = %f", diff, rotVal);
  LINFO("2[%f %f]: dist: %f --> distVal: %f", x2, y2, dist, distVal);

  double rot = 0.5*rotVal + 0.5*distVal + .07;

  LINFO("raw rot: %f", rot);

  if(rot >  0.5) rot =  0.5;
  if(rot < -0.5) rot = -0.5;

  LINFO("clipped rot: %f", rot);

  // command to be executed by the BeoPilot
  Beobot2::MotorCommand cmd;

  cmd.translation = 0.7F; 
  cmd.rotation    = rot;

  LINFO("h[%7.4f] i[%7.4f]d: %7.4f r: %7.4f", 
        heading, idealHeading, diff, rot);

  itsDistanceToPath = dist;

  return cmd;
}

// ######################################################################
Beobot2::MotorCommand GPS_Navigation::computeHeading
(double heading, double idealHeading)
{
  // compute corrective 
  double diff = heading - idealHeading;
  if(diff >  M_PI) diff = diff - 2*M_PI;
  if(diff < -M_PI) diff = diff + 2*M_PI;

  double rot  = 3.0*diff/(2*M_PI) + .07;
  if(rot >  0.5) rot =  0.5;
  if(rot < -0.5) rot = -0.5;

  // command to be executed by the BeoPilot
  Beobot2::MotorCommand cmd;

  cmd.translation = 0.7F; 
  cmd.rotation    = rot;

  //LINFO("h[%7.4f] i[%7.4f]d: %7.4f r: %7.4f", 
  //      heading, idealHeading, diff, rot);

  return cmd;
}

// ######################################################################
Beobot2::MotorCommand GPS_Navigation::computeGPS_Navigation
(double lat, double lon, double xin, double yin)
{
  double Ax = itsCurrentStartLocation.i;
  double Ay = itsCurrentStartLocation.j;
  double Bx = itsCurrentGoalLocation.i;
  double By = itsCurrentGoalLocation.j;

  double x = lat;
  double y = lon;

  LINFO("lat: %f lon: %f x: %f y: %f", lat, lon, xin, yin);

  LINFO("Ax: %f Ay: %f Bx: %f By: %f", Ax, Ay, Bx, By);

  double dgx = Bx - Ax;
  double dgy = By - Ay;
  double ang = atan2(dgy, dgx);

  //LINFO("dgx: %f dgy: %f ang %f", dgx, dgy, ang);

  // set the line get distance, etc
  double diffA = (Bx - Ax)*(x - Ax);
  double diffB = (By - Ay)*(y - Ay); 
  double diff  =  diffA + diffB; 
  double tdistSq = dgx*dgx + dgy*dgy;
  double u     = diff/tdistSq;

  //LINFO("diffA: %f diffB: %f diff %f tdistSq: %f u: %f", 
  //      diffA, diffB, diff, tdistSq, u);

  double intx = Ax + u*dgx;
  double inty = Ay + u*dgy;

  double dx =  x - intx;
  double dy =  y - inty;
  double dist = pow((dx*dx) + (dy*dy), 0.5);

  double side = float((Bx-Ax)*(y-Ay)-(By-Ay)*(x-Ax) > 0.0);
  if(side == 0.0) side = -1.0;

  double distVal = 1.0;
  if(dist < MAX_DIST) distVal = dist/MAX_DIST;
  distVal = -side * distVal;

  //LINFO("intx: %f inty: %f", intx, inty);
  //LINFO("dx: %f dy: %f, dist: %f side: %f ==> distVal: %f", 
  //      dx, dy, dist, side, distVal);

  Point2D<double> heading = Point2D<double>(90.0,90.0);//getCurrentHeading(lat, lon, x, y);
  double aheading = atan2(heading.j, heading.i); 
  double dang  = ang - aheading;
  if(dang > M_PI)        dang = dang - 2*M_PI;
  else if (dang < -M_PI) dang = dang + 2*M_PI;

  double angVal = dang/M_PI;

  //LINFO("aheading: %f ang: %f dang: %f ==> angVal: %f ", 
  //      aheading, ang, dang, angVal);

  // command to be executed by the BeoPilot
  Beobot2::MotorCommand cmd;

  cmd.translation = 0.5F; 
  cmd.rotation    = 0.5 * distVal + 0.5 * angVal;

  //LINFO("rot: %f", cmd.rotation);
  //Raster::waitForKey();

  return cmd;
}

// ######################################################################
Point2D<double> GPS_Navigation::getCurrentHeading
(double lat, double lon, double x, double y)
{
  if(itsCurrNumGPScoords == 1)
    return Point2D<double>
      (itsCurrentGoalLocation.i - itsCurrentStartLocation.i,
       itsCurrentGoalLocation.j - itsCurrentStartLocation.j );

  int prevNum = (itsCurrNumGPScoords - 2)%NUM_PREV_COORDS;
  int currNum = (itsCurrNumGPScoords - 1)%NUM_PREV_COORDS;

  double dx = itsPrevGPScoords[currNum].i - itsPrevGPScoords[prevNum].i;
  double dy = itsPrevGPScoords[currNum].j - itsPrevGPScoords[prevNum].j;
  return Point2D<double>(dx,dy);
}

// ######################################################################
void GPS_Navigation::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // GPS message
  if(eMsg->ice_isA("::BeobotEvents::GPSMessage"))
  {
    // we got GPS data
    BeobotEvents::GPSMessagePtr gpsMsg =
      BeobotEvents::GPSMessagePtr::dynamicCast(eMsg);

    int currRequestID = gpsMsg->RequestID;
    LINFO("Got a GPSMessage with Request ID = %d", currRequestID);

    double lat = gpsMsg->latitude;
    double lon = gpsMsg->longitude;
    //double xin = gpsMsg->x;
    //double yin = gpsMsg->y;

    double x = lat;
    double y = lon;

    itsPrevGPScoords[itsCurrNumGPScoords%NUM_PREV_COORDS] = 
      Point2D<double>(x,y);
    itsCurrNumGPScoords++;

    LINFO("[%5d] GPS lat:%15.6f lon:%15.6f (%15.6f, %15.6f)", 
          itsCurrNumGPScoords, lat, lon, x, y);

    // find the closest way point
    int closestWP = -1; double minDiff = -1.0;
    for(uint i = 0; i < itsGoalWayPoints.size(); i++)
      {
        Point2D<double> pt = itsGoalWayPoints[i];
        double dx = pt.i - x ;
        double dy = pt.j - y;
        double diff = pow((dx*dx) + (dy*dy), 0.5);
        LINFO("(%f %f) --> (%f %f): diff: %f", pt.i, pt.j, x, y, diff);


        if(closestWP == -1)
          {
            closestWP = 0; minDiff = diff;
          }
        else if(diff < minDiff)
          {
            closestWP = i; minDiff = diff;
          }
      }

    // set the next waypoint to go to
    if(closestWP != 0)
      {
        if(minDiff > PROXIMITY_DIST)
          {
            itsCurrentStartLocation = itsGoalWayPoints[closestWP-1];
            itsCurrentGoalLocation  = itsGoalWayPoints[closestWP  ]; 
            itsCurrentGoalHeading   = itsGoalHeadings[closestWP-1];
          }
        else
          {
            if(closestWP != int(itsGoalWayPoints.size() - 1))
              {
                itsCurrentStartLocation = itsGoalWayPoints[closestWP  ];
                itsCurrentGoalLocation  = itsGoalWayPoints[closestWP+1];                 
                itsCurrentGoalHeading   = itsGoalHeadings[closestWP-1];
              }
            // else we're done
          }
      }
    else 
      {
        itsCurrentStartLocation = itsGoalWayPoints[0];
        itsCurrentGoalLocation  = itsGoalWayPoints[1]; 
        itsCurrentGoalHeading   = itsGoalHeadings[closestWP-1];
      }

    // stop the robot at goal
    if(closestWP == int(itsGoalWayPoints.size() - 1) && 
       minDiff < PROXIMITY_DIST)
      {        
        LINFO("We're Done!!!!");
        itsAtGoal = true;
      }
  }

  // can move around
  else if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
  {
    if(itsAtGoal) updateMotor(0.0,0.0);
    else
      {
        // we got motor data
        BeobotEvents::MotorMessagePtr motorMsg =
          BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);

        //int currRequestID = motorMsg->RequestID;
        double heading    = motorMsg->imuHeading;
        double x          = motorMsg->encoderX;
        double y          = motorMsg->encoderY;
        itsCurrentMovement = Point2D<double>(x,y);

        //LINFO("Got a MotorMessage with Req ID = %d:h %f", 
        //      currRequestID, heading);

         if(itsCurrentGoalHeading == -100.0)   
           itsCurrentGoalHeading = heading;

        // compute navigation
        //cmd = computeGPS_Navigation(lat, lon, x, y);
        Beobot2::MotorCommand cmd = 
          computeHeading(heading, itsCurrentGoalHeading);
        itsCurrentHeading = heading;

        itsHeadingCommand = cmd;
      }
  }

  // LRF message
  else if(eMsg->ice_isA("::BeobotEvents::LRFMessage"))
  {
    // we got LRF data
    BeobotEvents::LRFMessagePtr lrfMsg =
      BeobotEvents::LRFMessagePtr::dynamicCast(eMsg);

    itsDistances = lrfMsg->distances;
    itsAngles    = lrfMsg->angles;

    // compute navigation
    Beobot2::MotorCommand cmd = 
      itsNDNavigationAlgorithm->computeNavigationCommand
      (itsDistances, itsAngles);
    itsIsSafe = itsNDNavigationAlgorithm->isSafe();

    itsNDcommand = cmd;
  }
}

// ######################################################################
void GPS_Navigation::updateMotor(double tran,double rot)
{
  BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
  msg->transVel = tran;
  msg->rotVel   = rot;
  this->publish("MotorRequestTopic", msg);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
