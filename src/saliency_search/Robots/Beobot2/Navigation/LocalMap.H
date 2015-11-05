/*!@file Robots/Beobot2/Navigation/LocalMap.H representation of robot's 
surrounding */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/LocalMap.H $
// $Id: $
//

// ######################################################################
/*! Local Map implementation                                           */

#ifndef BEOBOT2_LOCALMAP_H_DEFINED
#define BEOBOT2_LOCALMAP_H_DEFINED


#define NUM_STORED_HEADINGS                      500 

#include "Image/OpenCVUtil.H"

#include "Robots/Beobot2/BeoCommon.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/Point3D.H"

#include "SIFT/Histogram.H"

#include "Beobot/Graph.H"
#include "Beobot/GridMap.H"

#include "GUI/XWinManaged.H"

#include "Util/Timer.H"

//! Local map representation that maps robot surroundings
//! and generate motion command to go to the goal target
//! target can be a location or a person
class LocalMap 
{
public:

  //! Constructor: generate a blank map
  LocalMap(uint num_horz_grid, uint num_vert_grid, 
           double grid_width,  double grid_height);

  //! Destructor
  ~LocalMap();

  //! returns an image representation of the map
  //! NOTE that this representation is slow 
  //! and is not recommended for real time usage
  Image<PixRGB<byte> > getLocalMapImage();

  //! update local map using Laser Range Finder (LRF) data
  void updateLocalMap
  (std::vector<double> distances, std::vector<double> angles);

  //! update local map using odometry
  void updateLocalMap
  (double heading, Point2D<double> dPosition,
   double transCap        = -1.0, double rotCap        = -1.0, 
   double currentTransVel =  0.0, double currentRotVel =  0.0);

  //! update local map with a traversable map
  //! NOTE: caller is responsible for spatially aligning
  //!       the map pose with the current robot pose
  void updateLocalMap(Image<float> gmap);

  //! update local map with a traversable map
  //! and a local heading (usually with respect to the road)
  void updateLocalMap
  (Image<float> gmap, double heading, float lateral_deviation = 0.0f);

  //! set the goal node
  bool setGoalNode(Point2D<int> goalNode);

  //! return the steering command
  //! we use A* for path planning 
  //! then smooth the result to avoid obstacle as much as possible
  //! the account for robot geometry and dynamics 
  //! to generate motor command
  Beobot2::MotorCommand getNavigationCommand();

  //! compute the motor command 
  //! using Dynamic Window Approach [Fox 1997]
  Beobot2::MotorCommand getDWAnavigationCommand();

  //! return map dimension
  void getMapDimension(double &gridWidth,   double &gridHeight, 
                       uint   &numHorzGrid, uint   &numVertGrid);

  //! execute turn procedure if currently in one
  void turnProcedure();

  //! check if robot is currently turning
  //! by having the goal not straight ahead
  bool isInTurnProcedure();

  //! compute the immediate grid occupancy map
  //! usually for display purposes
  Image<double> getImmediateGridOccupancyMap();

  //! create the heading histogram display
  //! usually for display purposes  
  Image<float> getHeadingHistogram();

  //! individual variable access function
  inline const double                        getGridWidth() const;
  inline const double                        getGridHeight() const;
  inline const Image<double>                 getGridOccupancyMap() const;
  inline const Point2D<double>               getGridRobotCenterRatio() const;
  inline const std::vector<double>           getDistances() const;
  inline const std::vector<double>           getAngles() const;
  inline const std::vector<double>           getImmediateDistances() const;
  inline const std::vector<double>           getImmediateAngles() const;
  inline const std::vector<Point3D<double> > getPredictedNextLocations() const;
  inline const Point2D<double>               getCurrAccTrajectory() const;
  inline const std::vector<Point2D<float> >  getPath() const;
  inline const std::vector<Point2D<float> >  getSmoothedPath() const;
  inline const double                        getDesiredHeading() const;
  inline const double                        getSelfLocalHeading() const;
  inline const Beobot2::MotorCommand         getCurrentMotorCommand() const;
  inline const std::vector<Point2D<int> >    getGoalNodes() const;
  inline const Point2D<int>                  getGoalNode() const;
  inline const std::vector<Point2D<double> > getDWcenters() const;
  inline const double                        getCurrentTransVelCommand() const;
  inline const double                        getCurrentRotVelCommand() const;
  inline const int                           getDeadEndIndicator() const;
  inline const double                        getCurrentGlobalHeading() const;
  inline const double                        getRoadGlobalHeading() const;
  inline const double                        getRoadGlobalHeadingConfidence() const;
  inline const float                         getLateralDeviation() const;
  inline const float                         getLateralDeviationAngle() const;
  inline const std::vector<float>            getClosestObjDistOnPath() const;

private:

  //! FIXXX: make sure it's needed
  std::vector<Point2D<int> > getLine
  (Point2D<int> p1, Point2D<int> p2, const int w, const int h);

  std::vector<Point2D<int> > getLine
  (Point2D<float> p1, Point2D<float> p2, const int w, const int h);
  
  //! apply the accumulated translational shift to the locap map
  void shiftLocalMap();

  //! rotate local map about the location (x,y) for ang angle
  Image<double> rotateMap
  (Image<double> srcImg, const int x, const int y, const float ang);

  //!
  Image<double> combineMap
  (Image<double> currMap, Image<double> inMap);

  //! get motor command given the desired angle
  Beobot2::MotorCommand getMotorCommand(double diff);

  //! update the coarse local map
  Image<double> computeCoarseLocalMap();

  //! update immediate local map
  Image<double> computeImmediateLocalMap();

  //!
  Image<double> computeImmediateLocalMap(Image<float> map);

  //! 
  void estimateRoadGlobalHeading(double dheading);

  //!
  bool checkForTurnCompletion(float cHeading, float gHeading);

  //! filter lateral deviation estimation
  void filterLateralDeviation(float lateral_deviation);

  //! setup grid map weights to search the trajectory
  void setupGridMap();
  void setupGridMapEdgeWeights();

  //! compute trajectory
  void computeTrajectory();

  //! modify the path according to task
  std::vector<Point2D<int> > modifyPathForCurrentTask
  (std::vector<Point2D<int> > steps, float &pathCost);

  //! smooth the path
  std::vector<Point2D<float> > smoothPath(std::vector<Point2D<float> > path);

  //! modify path to avoid obstacle as much as possible 
  std::vector<Point2D<float> > modifyPathForObstacleAvoidance();

  //! generate desired heading 
  //! for the next step
  void generateDesiredHeading(); 

  //! given a robot direction check if it will hit 
  //! any segment in the smoothed path
  float getDistanceToTrajectory
  (Point2D<float> curr_m_pt, float ang);

  //! given a vector (p1, p2) check if it will hit 
  //! segment (p3,p4) in the smoothed path
  float distanceToSegment
  (Point2D<float> p1, Point2D<float> p2, 
   Point2D<float> p3, Point2D<float> p4);

  //! find closest obstacle to the robot
  //! with the robot modeled by a rectangle
  float distanceToClosestObstacle
  (std::vector<Point2D<float> > m_body);

  //! Dynamic Window Approach related utility functions
  double getMaxTransAcceleration();
  double getMaxRotAcceleration();
  double getMaxRotVelocity();
  double getMaxTransVelocity();
  double getAcceleration(double motorPower);
  double getVelocity(double motorPower);

  //!
  Point3D<double> getOptimalVW
  (Point3D<double> currentPos,
   double currentV,double currentW, double dV,double dW,
   double maxV,double maxW, double stepV,double stepW,
   double accel, double goalHeading, double desireV, double dt);
  
  //! find next pos x,y,theta given v,w
  Point3D<double> 
  getNextPos(double v,double w, double dt,
             Point3D<double> currentPos = Point3D<double>(0.0,0.0,0.0));
  
  //! find distance to the closest obstacle 
  //! on the arc trajectory
  Beobot2::distAngle 
  getNearestObstacle(double v, double w, 
                     Point3D<double> currentPos = 
                     Point3D<double>(0.0,0.0,0.0),
                     bool debug = false);

  //! find distance to the closest obstacle 
  //! on the straightline trajectory
  double getNearestObstacleStraight(double v);

  //! mode the robot geometry using a square 
  //! For beobot the width is 60cm (very close to actual size) 
  std::vector<Point2D<float> > getRobotBody
  (Point2D<float> front_center, double angle);

  //! non-linear transformation to account for low motor response
  //! in heading command close to 0
  Beobot2::MotorCommand compensateLowVelocitySaturation
  (Beobot2::MotorCommand cmd);

  //! the grid occupancy map
  Image<double> itsGridOccupancyMap;
  Image<double> itsImmediateGridOccupancyMap;
  double itsGridWidth;
  double itsGridHeight;
  uint   itsNumHorzGrid;
  uint   itsNumVertGrid;
  Point2D<double> itsGridRobotCenterRatio;

  std::vector<Point3D<double> > itsPredictedNextLocations;

  std::vector<Point2D<float> > itsPath;
  std::vector<Point2D<int  > > itsPathSteps;
  std::vector<Point2D<float> > itsSmoothedPath;
  //the closest obstacle distance on each path node
  std::vector<float> itsClosestObjDistOnPath;
  
  float itsPathCost;

  //! the heading that path planner produced
  double itsDesiredHeading;

  //! initial BeoPilot (IMU) heading
  double itsInitialGlobalHeading;

  //! store the previous global heading
  double itsPreviousGlobalHeading;

  double itsCurrentGlobalHeading;
  double itsRoadGlobalHeading;
  double itsRoadGlobalHeadingConfidence;


  //! histogram storage for heading
  std::vector<std::vector<double> > itsHeadingHistogram;
  std::vector<double> itsHeadingValues;


  //! indicates state of dead-end detector:
  //! can go to goal, near dead end, dead end
  int itsDeadEndIndicator;

  //! dynamic window approach related variables
  double itsTransCap; 
  double itsRotCap;

  double itsCurrentTransVel;
  double itsCurrentRotVel; 

  double itsCurrentTransVelCommand;
  double itsCurrentRotVelCommand; 

  std::vector<Point2D<double> >itsDWcenters;

  //! current motor command sent to BeoPilot
  Beobot2::MotorCommand itsCurrentMotorCommand;

  //! the grid map representation of the map
  rutz::shared_ptr<GridMap> itsGridMap;

  //! the robot's self local heading
  double itsSelfLocalHeading;
  Point2D<double> itsCurrAccTrajectory;

  //! coordinate of goal heading
  //! it can be in the outer part (beyond the map)
  //! or within the local map (to cue for object/person) 
  //!   --> goal should be in front of the target
  //! also have vector of targets for multiple target tasks  
  //! e.g. following the road while following a person
  Point2D<int>   itsGoalNode;
  Point2D<float> itsGoalLocation;
  std::vector<Point2D<int> > itsGoalNodes;

  //! lateral deviation related information
  float itsLateralDeviation;
  float itsLateralDeviationAngle;

  //! LRF related information
  std::vector<double> itsDistances;
  std::vector<double> itsAngles;
  std::vector<Point2D<float> > itsLRFmetricSelfCoordinates;
  std::vector<Point2D<float> > itsLRFgridMapCoordinates;
  
  std::vector<double> itsImmediateDistances;
  std::vector<double> itsImmediateAngles;

  Image<std::vector<Point2D<float> > > itsLRFmap;

  //! display (debugging)
  Image<PixRGB<byte> > itsDisplayImage;
  rutz::shared_ptr<XWinManaged> itsWin;
  Timer itsTimer;


  int   itsComputedNavCommandCount;
  int   itsTotalNavCommandCount;
  Timer itsNavCommandTimer;

};

// ######################################################################
// Implementation for LocalMap inline functions
// ######################################################################
inline const double LocalMap::getGridWidth() const
{ return itsGridWidth; }

inline const double LocalMap::getGridHeight() const
{ return itsGridHeight; }
      
inline const Image<double> LocalMap::getGridOccupancyMap() const
{ return itsGridOccupancyMap; }

inline const Point2D<double> LocalMap::getGridRobotCenterRatio() const
{ return itsGridRobotCenterRatio; }

inline Image<double> LocalMap::getImmediateGridOccupancyMap()
{ 
  // update Immediate Local Map
  // Image<double> currImap = computeImmediateLocalMap(); 
  // itsImmediateGridOccupancyMap = 
  //   combineMap(itsImmediateGridOccupancyMap, currImap);

  itsImmediateGridOccupancyMap = computeImmediateLocalMap(); 
  return itsImmediateGridOccupancyMap; 
}

inline const std::vector<double> LocalMap::getDistances() const
{ return itsDistances; }

inline const std::vector<double> LocalMap::getAngles() const
{ return itsAngles; }

inline const std::vector<double> LocalMap::getImmediateDistances() const
{ return itsImmediateDistances; }

inline const std::vector<double> LocalMap::getImmediateAngles() const
{ return itsImmediateAngles; }

inline const std::vector<Point3D<double> > LocalMap::getPredictedNextLocations() const
{ return itsPredictedNextLocations; }

inline const Point2D<double> LocalMap::getCurrAccTrajectory() const
{ return itsCurrAccTrajectory; }

inline const std::vector<Point2D<float> > LocalMap::getPath() const
{ return itsPath; }

inline const std::vector<Point2D<float> > LocalMap::getSmoothedPath() const
{ return itsSmoothedPath; }

inline const double LocalMap::getDesiredHeading() const
{ return itsDesiredHeading; }

inline const double LocalMap::getSelfLocalHeading() const
{ return itsSelfLocalHeading; }

inline const Beobot2::MotorCommand LocalMap::getCurrentMotorCommand() const
{ return itsCurrentMotorCommand; }


inline const double LocalMap::getCurrentTransVelCommand() const
{ return itsCurrentTransVelCommand; }

inline const double LocalMap::getCurrentRotVelCommand() const
{ return itsCurrentRotVelCommand; }

inline const std::vector<Point2D<int> > LocalMap::getGoalNodes() const
{ return itsGoalNodes; }

inline const Point2D<int> LocalMap::getGoalNode() const
{ return itsGoalNode; }

inline const std::vector<Point2D<double> >LocalMap::getDWcenters() const
{ return itsDWcenters; }

inline const int LocalMap::getDeadEndIndicator() const
{ return  itsDeadEndIndicator; }

inline Image<float> LocalMap::getHeadingHistogram()
{
  uint w = itsHeadingHistogram.size();
  Image<float> histogram(1,w, ZEROS);
  float bscale = 255.0F/float(NUM_STORED_HEADINGS );
  for(uint i = 0; i < itsHeadingHistogram.size(); i++)
    {
      float value = float(itsHeadingHistogram[i].size());
      if(value > 0) histogram.setVal(0,i, value*bscale);
    }

  // uint scale = 8;
  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(w*scale,h*scale), 0, 0, "hist"));
  // // else itsWin->setDims(Dims(w*scale,h*scale));
  // itsWin->drawImage(zoomXY(histogram,scale),0,0);

  return histogram;
}

inline const double LocalMap::getCurrentGlobalHeading() const
{ return itsCurrentGlobalHeading;}

inline const double LocalMap::getRoadGlobalHeading() const
{ return itsRoadGlobalHeading; }

inline const double LocalMap::getRoadGlobalHeadingConfidence() const
{ return itsRoadGlobalHeadingConfidence; }

inline const float LocalMap::getLateralDeviation() const
{ return itsLateralDeviation; }

inline const float LocalMap::getLateralDeviationAngle() const
{ return itsLateralDeviationAngle; }

inline const std::vector<float> LocalMap::getClosestObjDistOnPath() const
{ return itsClosestObjDistOnPath; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
