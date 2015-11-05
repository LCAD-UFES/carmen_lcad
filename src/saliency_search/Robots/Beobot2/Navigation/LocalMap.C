/*!@file Beobot/GridMap.C represent robot's surrounding using 
 grid occupancy map, self (local) heading and heading above the map     */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/LocalMap.C $
// $Id:GridMap.C 6891 2006-05-25 06:46:56Z siagian $
//

// the number of most recent headings stored
// to estimate global road heading 
// count of 500 adds up to interval of last 16.67s (30fps)  
#define HEADING_BIN_SPACING                      5   // degrees in 1 bin  

// Number of headings outputted by BeoRoadFinder
// 16 within image, 4 on each side
#define NUM_ROAD_FINDER_HEADINGS                 25 

// dead end indicator constants
#define CAN_REACH_GOAL               0
#define NEAR_DEAD_END                1
#define DEAD_END                     2

// minimum distance for legal LRF reading 
// highly unlikely that a lower reading is valid
#define MIN_LRF_DIST                 30.0F   // in mm

// minimum distance to obstacle, any closer would hit it
// this is to speed up the A* path planning stage
#define MINIMUM_DISTANCE_TO_OBSTACLE 1.0F // in coarse grid unit

// the extent of influence for a grid value
// in the A* path planning stage
#define GRID_INFLUENCE_EXTENT        4 // in coarse grid unit

// path smoothing data weight constant
#define PATH_SMOOTHING_DATA_WEIGHT   0.3F

// path smoothing smoothness weight constant
#define PATH_SMOOTHING_SMOOTHNESS_WEIGHT 0.2F; //.1F also work

// path smoothing convergence tolerance
#define PATH_SMOOTHING_TOLERANCE     .00001F;

// the maximum distance of LRF points to store in itsLRFmap
// 5 coarse grid unit constitutes  +/- 1.25 robot distance away
#define  LRF_MAP_NEIGHBORHOOD_SIZE   5  // in coarse grid unit 

// maximum distance of obstacle to consider 
// in Dynamic Window Approach [Fox 1997] 
#define MAXIMUM_OBSTACLE_DISTANCE    float(LOCAL_MAP_GRID_WIDTH)*float(LOCAL_MAP_NUM_HORZ_GRIDS)/1000.0F

// field of view range of regular lens (60 degree)
// wide angle lens: 120 degrees, omni camera: 360 degrees
#define MAXIMUM_FOV_ANGLE_DIFFERENCE 27.0/180.0 * M_PI

// in Dynamic Window Approach [Fox 1997] 
#define NORMAL_SPEED_GAIN            .8F       
#define NEAR_DEAD_END_SPEED_GAIN     .6F
#define DEAD_END_SPEED_GAIN          .4F

#include "Robots/Beobot2/Navigation/LocalMap.H"

#include "Robots/Beobot2/BeoCommon.H"

#include "Raster/Raster.H"
#include "Image/DrawOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"

#include "Util/FastMathFunctions.H"
#include "Util/Geometry2DFunctions.H"
#include "Util/Timer.H"

#include <cstdio>


// ######################################################################
LocalMap::LocalMap(uint num_horz_grid, uint num_vert_grid,
                   double grid_width,  double grid_height)
:
  itsTimer(1000000),
  itsNavCommandTimer(1000000)
{ 
  itsWin.reset();
  itsGridRobotCenterRatio  = Point2D<double>(ROBOT_CENTER_RATIO_I,
                                             ROBOT_CENTER_RATIO_J);

  itsGridOccupancyMap = 
    Image<double>(num_horz_grid, num_vert_grid, ZEROS) + 
    LOCAL_MAP_DEFAULT_GRID_VALUE;

  uint nImmGrid = uint(IMMEDIATE_MAP_WIDTH/IMMEDIATE_MAP_GRID_WIDTH);
  itsImmediateGridOccupancyMap =
    Image<double>(nImmGrid, nImmGrid, ZEROS) + 
    LOCAL_MAP_DEFAULT_GRID_VALUE;

  itsNumHorzGrid = num_horz_grid;
  itsNumVertGrid = num_vert_grid;

  itsGridWidth  = grid_width;
  itsGridHeight = grid_height;
  
  // default goal node of going straight
  double gi = double(num_horz_grid)*itsGridRobotCenterRatio.i;
  itsGoalNode     = Point2D<int>(gi, -1);
  itsGoalLocation.i = itsGoalNode.i + .5F;
  itsGoalLocation.j = itsGoalNode.j + .5F;
  itsGoalNodes.clear();
  itsGoalNodes.push_back(itsGoalNode);

  // the grid map
  setupGridMap();

  // set the local position and heading to looking straight forward
  itsCurrAccTrajectory    = Point2D<double>(0.0,0.0);
  itsSelfLocalHeading = 0.0;

  // clear path to goal
  itsPath.clear();
  itsPathSteps.clear();
  itsSmoothedPath.clear();
  itsDWcenters.clear();
  itsPathCost = -1.0F;
  itsDeadEndIndicator = 0; // can go to the goal

  itsCurrentMotorCommand.translation = 0.0;
  itsCurrentMotorCommand.rotation    = 0.0;
  itsCurrentTransVelCommand          = 0.0;
  itsCurrentRotVelCommand            = 0.0; 

  itsDesiredHeading               = 0.0;
  itsCurrentGlobalHeading         = 0.0;
  itsRoadGlobalHeading            = 0.0;
  itsRoadGlobalHeadingConfidence  = 0.0;

  itsInitialGlobalHeading         = -2*M_PI;
  itsPreviousGlobalHeading        = -2*M_PI;

  // road heading estimators
  itsHeadingHistogram.clear();
  int hsize = 360/HEADING_BIN_SPACING;
  for(int i = 0; i < hsize; i++)
    itsHeadingHistogram.push_back(std::vector<double>());
  itsHeadingValues.clear();

  itsTransCap        = 0.0; 
  itsRotCap          = 0.0;
  itsCurrentTransVel = 0.0;
  itsCurrentRotVel   = 0.0; 
  
  // current robot location in image coordinate
  itsPredictedNextLocations.clear();

  itsLRFmetricSelfCoordinates.clear();
  itsLRFgridMapCoordinates.clear();

  itsComputedNavCommandCount = 0;
  itsTotalNavCommandCount    = 0;

  itsLateralDeviation      = 0.0F;
  itsLateralDeviationAngle = 0.0F;
}

// ######################################################################
LocalMap::~LocalMap() { }

// ######################################################################
void LocalMap::getMapDimension
(double &gridWidth  , double &gridHeight, 
 uint   &numHorzGrid, uint   &numVertGrid)
{ 
  gridWidth   = itsGridWidth;
  gridHeight  = itsGridHeight;
  numHorzGrid = itsNumHorzGrid;
  numVertGrid = itsNumVertGrid;
}

// ######################################################################
bool LocalMap::setGoalNode(Point2D<int> goalNode)
{ 
  int gi = goalNode.i;   int gj = goalNode.j;

  // reject out of bounds goal
  if(gi < -1 && gi > int(itsNumHorzGrid) && 
     gj < -1 && gj < int(itsNumVertGrid)   ) return false;
  
  itsGoalNodes.clear();
  itsGoalNodes.push_back(goalNode);
  itsGoalNode = goalNode;

  // offset the road heading before turning procedure starts
  // add the turn offset to the heading history
  double goalHeading = 
    atan2(int(itsGridRobotCenterRatio.i*itsNumHorzGrid) - gi, 
          int(itsGridRobotCenterRatio.j*itsNumVertGrid) - gj);
  for(uint i = 0; i < itsHeadingHistogram.size(); i++)
    itsHeadingHistogram[i].clear();
  for(uint i = 0; i < itsHeadingValues.size(); i++)
    {
      double ang = itsHeadingValues[i];
      ang += goalHeading;
      if     (ang >  M_PI) ang -= 2*M_PI;
      else if(ang < -M_PI) ang += 2*M_PI;
      itsHeadingValues[i] = ang;
      
      // compute the histogram index and push it in
      double deghead = (ang + M_PI)/M_PI*180.0;
      if(deghead >= 360.0F) deghead = 359.0F;
      int index = int(deghead)/HEADING_BIN_SPACING;
      itsHeadingHistogram[index].push_back(ang);
    } 

  itsRoadGlobalHeading+= goalHeading;
  if     (itsRoadGlobalHeading >  M_PI) itsRoadGlobalHeading -= 2*M_PI;
  else if(itsRoadGlobalHeading < -M_PI) itsRoadGlobalHeading += 2*M_PI;

  // reset the lateral deviation
  itsLateralDeviation      = 0.0F;
  itsLateralDeviationAngle = 0.0F;
  return true;
}

// ######################################################################
bool LocalMap::isInTurnProcedure()
{
  int gi = itsGoalNode.i;
  int gj = itsGoalNode.j;

  // turns when goal to the left, right, or behind the robot
  int right  = int(itsNumHorzGrid);
  int bottom = int(itsNumVertGrid);
  return((gi == -1    && gj >= 0) || 
         (gi == right && gj >= 0) ||
         (gj == bottom));
}

// ######################################################################
void LocalMap::updateLocalMap
(double heading, Point2D<double> dposition,
 double transCap, double rotCap, 
 double currentTransVel,  double currentRotVel)
{
  // Odometry updateLocalMap

  itsTransCap        = transCap/100.0; 
  itsRotCap          = rotCap/100.0;
  itsCurrentTransVel = currentTransVel;
  itsCurrentRotVel   = currentRotVel;

  itsCurrentGlobalHeading = heading;

  double dheading = 0.0;
  // check whether it's an initial step
  if(itsPreviousGlobalHeading != -2*M_PI)
    {
      dheading = heading - itsPreviousGlobalHeading;
      if(dheading > M_PI)       dheading -= 2*M_PI;
      else if(dheading < -M_PI) dheading += 2*M_PI;
    }
  // in initial step set current heading to the road global heading 
  else 
    {
      itsRoadGlobalHeading    = itsCurrentGlobalHeading;
      itsInitialGlobalHeading = itsCurrentGlobalHeading;
    }

  itsPreviousGlobalHeading = heading;

  // update the robot heading using encoders or IMU
  itsSelfLocalHeading += dheading;
  if(itsSelfLocalHeading > M_PI)       itsSelfLocalHeading -= 2*M_PI;
  else if(itsSelfLocalHeading < -M_PI) itsSelfLocalHeading += 2*M_PI;

  // update the accumulated position change 
  // after converting to mm (from m)

  // use the initial global heading 
  // to transform the change in position
  double trans_heading = itsCurrentGlobalHeading - itsInitialGlobalHeading;

  double c_heading = cos(trans_heading);
  double s_heading = sin(trans_heading); 

  double d_i = dposition.i;
  double d_j = dposition.j;

  double td_i =  d_i*c_heading + d_j*s_heading;
  double td_j = -d_i*s_heading + d_j*c_heading;
  Point2D<double> trans_dposition(td_i,td_j);
  //Point2D<double> trans_dposition = dposition;

  //LINFO("trans_h = %f (c: %f s: %f): d(%f %f) --> td(%f %f)", 
  //      trans_heading, c_heading, s_heading, d_i, d_j, td_i, td_j);

  trans_dposition *= 1000.0;
  
  // filter out drifts (just keep the horizontal component

  itsCurrAccTrajectory += (trans_dposition);
  itsGoalLocation.i    += trans_dposition.j/float(LOCAL_MAP_GRID_WIDTH);
 
  // apply the accumulated translational shift to the local map
  shiftLocalMap();

  // apply turn procedure if needed
  turnProcedure();
}

// ######################################################################
void LocalMap::turnProcedure()
{
  // if the heading is beyond the goal and not straight ahead
  // we are in turn procedure
  if(!isInTurnProcedure()) return;

  int gi = itsGoalNode.i;
  int gj = itsGoalNode.j;
  double goalHeading = 
    atan2(int(itsGridRobotCenterRatio.i*itsNumHorzGrid) - gi, 
          int(itsGridRobotCenterRatio.j*itsNumVertGrid) - gj);

  LINFO("[%3d %3d] goal head: %7.2f deg self: %7.2f deg", 
        gi, gj, goalHeading/M_PI*180.0, itsSelfLocalHeading/M_PI*180.0);

  // if the difference is close
  if(checkForTurnCompletion(itsSelfLocalHeading, goalHeading))
    {
      // rotate the map
      int w = itsGridOccupancyMap.getWidth();
      int h = itsGridOccupancyMap.getHeight();
          
      // negative d_angle because of 
      // the robot-centric angle convention
      itsGridOccupancyMap = 
        rotateMap(itsGridOccupancyMap, w, h, -goalHeading);

      // rotate self local heading as well
      itsSelfLocalHeading -= goalHeading;
      if     (itsSelfLocalHeading >  M_PI) itsSelfLocalHeading -= 2*M_PI;
      else if(itsSelfLocalHeading < -M_PI) itsSelfLocalHeading += 2*M_PI;
      LINFO("new self local heading: %f", itsSelfLocalHeading/M_PI*180.0);

      // change the goal heading back to forward center
      itsGoalNode = Point2D<int>
        (int(itsGridRobotCenterRatio.i*itsNumHorzGrid),-1);
      itsGoalNodes.clear();
      itsGoalNodes.push_back(itsGoalNode);

      LINFO("\n\nDONE TURNING\n\n");
    }
}

// ######################################################################
bool LocalMap::checkForTurnCompletion(float cHeading, float gHeading)
{
  double diff = fabs(cHeading - gHeading) /M_PI*180.0F;
  
  LINFO("gheading: %7.2f vs: %7.2f --> diff: %7.2f", 
        gHeading/M_PI*180.0F, cHeading/M_PI*180.0F, diff);
  
  // if goal is to the left
  if(gHeading > 0 && (diff < 5.0F || cHeading > gHeading)) return true;

  // if the goal is to the right
  if(gHeading < 0 && (diff < 5.0F || cHeading < gHeading)) return true;

  // if the goal it straight forward
  if(gHeading == 0.0F && diff < 10.0F) return true;

  // if the goal it right behind
  if((gHeading ==  M_PI && (cHeading < -.875*M_PI)) ||
     (gHeading == -M_PI && (cHeading >  .875*M_PI))   ) return true;

  return false;
}

// ######################################################################
void LocalMap::estimateRoadGlobalHeading(double curr_road_heading)
{
  // filter out input heading that 
  uint   size = itsHeadingValues.size();
  double diff_threshold = MAXIMUM_FOV_ANGLE_DIFFERENCE;
  double diff = fabs(itsRoadGlobalHeading -  curr_road_heading);
  if(diff > 2*M_PI) diff -= M_PI;
  if(size > NUM_STORED_HEADINGS/2 && 
     diff > diff_threshold &&
     itsRoadGlobalHeadingConfidence > .75) return;

  // pop older value if needed
  if(size >= NUM_STORED_HEADINGS)
    {
      // pop it out of the least recent heading values
      double phead = itsHeadingValues[0];
      *itsHeadingValues.erase(itsHeadingValues.begin());

      // pop it from the appropriate histogram slot
      double p_deghead = (phead + M_PI)/M_PI*180.0;
      if(p_deghead >= 360.0F) p_deghead = 359.0F;
      int pindex    = int(p_deghead)/HEADING_BIN_SPACING;
      //LINFO("phead: %f --> %d", phead/M_PI*180.0, pindex);

      if(itsHeadingHistogram[pindex].size() > 0)
        {
          //double val = itsHeadingHistogram[pindex][0];
          *itsHeadingHistogram[pindex].
            erase(itsHeadingHistogram[pindex].begin());
          // LINFO("in hist: %f vs. pop: %f", 
          //       val/M_PI*180.0, phead/M_PI*180.0);
        }
    }
  
  // push the new one in
  itsHeadingValues.push_back(curr_road_heading);

  // input the current heading to the appropriate histogram
  double c_deghead = (curr_road_heading + M_PI)/M_PI*180.0;
  if(c_deghead >= 360.0F) c_deghead = 359.0F;
  int cindex = int(c_deghead)/HEADING_BIN_SPACING;
  // LINFO("road_heading: %f --> index: %d/%d", 
  //       curr_road_heading/M_PI*180.0, cindex, int(itsHeadingHistogram.size()));
  itsHeadingHistogram[cindex].push_back(curr_road_heading);
  size = itsHeadingValues.size();
  // LINFO("heading size: %d", size);

  // for(uint i = 0; i < itsHeadingValues.size(); i++)
  //   LINFO("[%3d]: %f", i, itsHeadingValues[i]/M_PI*180.0F);

  // for(uint i = 0; i < itsHeadingHistogram.size(); i++)
  //   {
  //     uint num = itsHeadingHistogram[i].size();
  //     if(num > 0) LINFO("[%3d]: %3d", i, num);
  //   }

  // estimate heading using histogram  
  int total = size;
  int range = 3; int hsize = int(itsHeadingHistogram.size());
  int max = 0; int max_i = 0;
  for(int i = 0; i <= hsize-range; i++)	
    {
      int sum = 0;
      for(int j = 0; j < range; j++)
        {
          sum += itsHeadingHistogram[i+j].size(); 
        }
      if(max < sum) { max = sum; max_i = i; }
    }
  // if the histogram is consistent 
  // estimate heading for the full interval
  float conf_val = float(max)/float(total);
  if(conf_val >= 0.75F)
    {
      uint num = 0; double theading = 0.0F;
      for(int i = max_i; i < max_i+range; i++)
        {
          int csize = itsHeadingHistogram[i].size(); 
          num += csize;
          for(int j = 0; j < csize; j++)
            theading += itsHeadingHistogram[i][j];
        }

      itsRoadGlobalHeading = theading/double(num);
    }
  itsRoadGlobalHeadingConfidence = conf_val;

  // if not consistent enough 
  // either only use heading from the last second
  // or not change (seems to be the better idea) 
  //else itsRoadGlobalHeading = curr_road_heading;

  //LINFO("max_i[%d]: %d/%d conf: %f ==> estimated heading: %f", 
  //      max_i, size, max, conf_val, itsRoadGlobalHeading/M_PI*180.0F);
}

// ######################################################################
void LocalMap::updateLocalMap
(std::vector<double> distances, std::vector<double> angles)
{
  // LRF updateLocalMap

  // store the distances and angles
  itsDistances.clear();
  itsAngles.clear();
  for(uint i = 0; i < distances.size(); i++)
    {
      itsDistances.push_back(distances[i]);      
      itsAngles.push_back(angles[i]);
    }

  // update Coarse Local Map 
  Image<double> currCLmap = computeCoarseLocalMap();
  itsGridOccupancyMap = combineMap(itsGridOccupancyMap, currCLmap);
}

// ######################################################################
void LocalMap::updateLocalMap(Image<float> gmap)
{
  // NOTE: This function assume that the gmap is taken in the same pose
  //       thus precluding the need for realignment.
  //       In general incoming map assumed to already projected forward

  // update coarse local map
  itsGridOccupancyMap = combineMap(itsGridOccupancyMap, gmap);
}

// ######################################################################
void LocalMap::updateLocalMap
(Image<float> gmap, double dheading, float lateral_deviation)
{
  // traversal map updateLocalMap

  Timer t(1000000);

  // NOTE: This function assumes that the gmap heading is the correct one
  //       we rotate the local map and set the heading with the input

  // FIXXX time this -- if can just skip!!! FIXXX the rotation
  // Rotate an image about (x,y) ang(in Radians)

  // double dheading = 0.0;

  // // check whether it's an initial step
  // if(itsPreviousGlobalHeading != -2*M_PI)
  //   {
  //     dheading = heading - itsPreviousGlobalHeading;
  //     if(dheading > M_PI)       dheading -= 2*M_PI;
  //     else if(dheading < -M_PI) dheading += 2*M_PI;
  //   }
  // itsPreviousGlobalHeading = heading;

  //float d_angle = dheading - itsSelfLocalHeading; 
  //int w = itsGridOccupancyMap.getWidth();
  //int h = itsGridOccupancyMap.getHeight();
  //LINFO("dheading: %f self: %f angle difference: %f w %d h %d", 
  //      dheading, itsSelfLocalHeading, d_angle, w,h);

  // negative d_angle because of the robot-centric angle convention
  //itsGridOccupancyMap = rotateMap(itsGridOccupancyMap, w, h, -d_angle);

  // update coarse local map
  //itsGridOccupancyMap = combineMap(itsGridOccupancyMap, gmap);

  //LINFO("time: %f", t.get()/1000.0F);

  double curr_road_heading = itsCurrentGlobalHeading - dheading;
  if     (curr_road_heading >  M_PI) curr_road_heading -= 2*M_PI;
  else if(curr_road_heading < -M_PI) curr_road_heading += 2*M_PI;  

  // estimate road heading
  estimateRoadGlobalHeading(curr_road_heading);
  
  //LINFO("estimate road heading: %f",t.get()/1000.0F);

  // if road global heading estimation confidence is high
  if(itsRoadGlobalHeadingConfidence > .8)
    {
      // reset self local heading
      itsSelfLocalHeading = 
        itsCurrentGlobalHeading - itsRoadGlobalHeading;
      if     (itsSelfLocalHeading >  M_PI) itsSelfLocalHeading -= 2*M_PI;
      else if(itsSelfLocalHeading < -M_PI) itsSelfLocalHeading += 2*M_PI;

      // LINFO("self_l_head: %f (cg: %f - rg: %f): conf: %f",
      //       itsSelfLocalHeading    /M_PI*180.0F,
      //       itsCurrentGlobalHeading/M_PI*180.0F, 
      //       itsRoadGlobalHeading   /M_PI*180.0F,
      //       itsRoadGlobalHeadingConfidence);
    }
  // otherwise we keep it as is in hope it will catch on
  //else LINFO("not reestimating self local heading");

  // shift the goal using the lateral deviation
  if(!isInTurnProcedure())
    {  
      filterLateralDeviation(lateral_deviation);
    }
}

// ######################################################################
void LocalMap::filterLateralDeviation(float lateral_deviation)
{
  // compute difference with previous lateral deviation
  float input_lat_dev = lateral_deviation;
  float prev_lat_dev  = itsLateralDeviation;
  float diff          = input_lat_dev - prev_lat_dev;

  //LINFO("input: %f - prev: %f = diff: %f", 
  //      input_lat_dev, prev_lat_dev, diff);

  // filter the goal location to suppress spikes
  // can only move 5mm at a time
  float LAT_DEV_STEP = 5.0F;
  float f_lat_dev = input_lat_dev;
  if     (diff >  LAT_DEV_STEP) f_lat_dev = prev_lat_dev + LAT_DEV_STEP;
  else if(diff < -LAT_DEV_STEP) f_lat_dev = prev_lat_dev - LAT_DEV_STEP;

  // cap for the border cases
  // if it's too far out, put to the end
  float w          = float(itsGridOccupancyMap.getWidth());
  float grid_w     = float(LOCAL_MAP_GRID_WIDTH);
  float border = grid_w*(w/2+1);
  if     (f_lat_dev < -border) f_lat_dev = -border;
  else if(f_lat_dev >  border) f_lat_dev =  border;

  // lateral deviation in mm
  itsLateralDeviation = f_lat_dev;
  //LINFO("f_lat_dev %f", itsLateralDeviation);
}

// ######################################################################
void LocalMap::shiftLocalMap()
{
  // FIXXXX if trajectory is small: indication of drift --> just kill it???

  double horz1 = itsCurrAccTrajectory.j;
  double vert1 = itsCurrAccTrajectory.i;

  // check if movement shifts the coarse local map
  double nhorz1 = horz1/double(itsGridWidth );
  double nvert1 = vert1/double(itsGridHeight);
  //LINFO("1[%f %f] --> %f %f", horz1, vert1, nhorz1, nvert1);
  if(fabs(nhorz1) < 1.0 && fabs(nvert1) < 1.0)    
    { LDEBUG("No Shift"); }
  else
    {
      int hshift = int(nhorz1 + 0.5);
      if(horz1 < 0.0) hshift = int(nhorz1 - 0.5);
      int vshift = int(nvert1 + 0.5);
      if(vert1 < 0.0) vshift = int(nvert1 - 0.5);
      //LINFO("shifting by: horz: %f --> %d | vert: %f --> %d", 
      //      nhorz1, hshift, nvert1, vshift);

      Image<double> gmap = 
        shiftClean(itsGridOccupancyMap, hshift, vshift, 
                   LOCAL_MAP_DEFAULT_GRID_VALUE);
      itsGridOccupancyMap = gmap;

      if(fabs(nhorz1) >= 1.0) itsCurrAccTrajectory.j = 0.0;      
      if(fabs(nvert1) >= 1.0) itsCurrAccTrajectory.i = 0.0;

      // do not shift goal node based on odometry
      //itsGoalNode.i     += hshift;
      //if(horz1 < 0) itsGoalLocation.i  = itsGoalNode.i+1.0F; 
      //else          itsGoalLocation.i  = itsGoalNode.i; 
    }
}

// ######################################################################
Image<double> LocalMap::rotateMap
(Image<double> srcImg, const int x, const int y, const float ang)
{
  // make sure the source image is valid
  ASSERT(srcImg.initialized());

  // create and clear the return image
  int w = srcImg.getWidth(), h = srcImg.getHeight();
  Image<double> retImg(w, h, ZEROS);

  // create temporary working image, put srcImg in the middle
  // put some padding (values are 0) around the image
  // so we won't need to check out of bounds when rotating
  int pad = int(ceil(sqrt(w * w + h * h)));
  int wR  = 2 * pad + w;
  int hR  = 2 * pad + h;
  Image<double> tempImg(wR, hR, ZEROS); 
  tempImg += LOCAL_MAP_DEFAULT_GRID_VALUE;
  
  inplacePaste(tempImg, srcImg, Point2D<int>(pad,pad));

  Image<double>::iterator rBPtr = retImg.beginw();
  float cAng = cos(ang), sAng = sin(ang);

  // fill in the return image with the appropriate values
  float xR = 0.0; float yR = 0.0;
  int dx = pad + x, dy = pad + y;
  for(int j = -y; j < h-y; j++)
    for(int i = -x; i < w-x; i++)
      {
        xR = dx + i*cAng + j*sAng;
        yR = dy - i*sAng + j*cAng;

        //LINFO("%d %d }}}}",int(xR+.5),int(yR+.5));
        *rBPtr++ = tempImg.getVal(int(xR+.5),int(yR+.5));
          //T(tempImg.getValInterp(xR,yR));
      }

  return retImg;
}

// ######################################################################
Image<double> LocalMap::combineMap
(Image<double> currMap, Image<double> inMap)
{
  uint w = currMap.getWidth();
  uint h = currMap.getHeight();
  Image<double> resMap(w,h, ZEROS);

  Image<double>::const_iterator cptr = currMap.begin();
  Image<double>::const_iterator iptr = inMap.begin();
  Image<double>::iterator rptr = resMap.beginw();

  for(uint j = 0; j < h; j++)
    for(uint i = 0; i < w; i++)
      {
        double val1 = *cptr++;
        double val2 = *iptr++;
        
        // compute new value
        double rval = val1;

        // always update on absolute value
        if     (val2 == 1.0) rval = 1.0;
        else if(val2 == 0.0) rval = 0.0;

        // can only update if there is a new value
        else if(val2 != LOCAL_MAP_DEFAULT_GRID_VALUE) 
          {
            double c_weight = .95, in_weight = .05;
            if(val1 == LOCAL_MAP_DEFAULT_GRID_VALUE) rval = val2;
            else 
              {
                rval = c_weight * val1 + in_weight * val2;
                // NOTE: can be more sophisticated
              }
          }
        *rptr++ = rval;
      }

  return resMap;
}

// ######################################################################
Image<double> LocalMap::computeCoarseLocalMap()
{
  double angle  = -141.0 + itsSelfLocalHeading/M_PI*180.0;
  double langle = -141.0;

  uint w = itsGridOccupancyMap.getWidth();
  uint h = itsGridOccupancyMap.getHeight();
  Image<double> res(w,h,ZEROS); res += LOCAL_MAP_DEFAULT_GRID_VALUE; 
  Image<double> count(w,h, ZEROS);

  float gridW = float(itsGridWidth);
  float gridH = float(itsGridHeight);
  
  // get the robot point location and heading
  // drawing after the laser drawing
  float cmapx = w*itsGridRobotCenterRatio.i + .5F;
  float cmapy = h*itsGridRobotCenterRatio.j + .5F;

  float cx = cmapx + itsCurrAccTrajectory.i/gridW;
  float cy = cmapy + itsCurrAccTrajectory.j/gridH;
  Point2D<int> cpt(cx, cy);
  //LINFO("cx: %f cy: %f", cx,cy);

  // also get the laser points within the immediate map
  itsImmediateDistances.clear();
  itsImmediateAngles.clear();

  itsLRFmetricSelfCoordinates.clear();
  itsLRFgridMapCoordinates.clear();

  // draw gaussian likelihood through each line
  for(uint i = 0; i < itsDistances.size(); i++)
    {
      double dist  = itsDistances[i];

      // change invalid distances and set as maximum (10m)
      if(dist == -1.0F) 
        { dist = 10000; } //itsDistances[i] = dist; }

      // disregard laser points that hits robot
      if(dist > MIN_LRF_DIST && (langle > -90.0 && langle < 90.0))
        {

          // FIXXX NOTES: this is to debug sun interference
          //if(dist < 1000.0) LINFO("[%3d]: %f", i, dist);

          // accounting self heading in the local map
          // FIXXX: can be stored to speed up computation
          float dx = dist*sin((double)angle*M_PI/180.0);
          float dy = dist*cos((double)angle*M_PI/180.0);
          
          // without self heading accounted for
          float o_dx = -dist*sin((double)langle*M_PI/180.0);
          float o_dy = -dist*cos((double)langle*M_PI/180.0);
          itsLRFmetricSelfCoordinates.push_back
            (Point2D<float>(o_dx/1000.0,o_dy/1000.0));

          if(fabs(dx) < IMMEDIATE_MAP_WIDTH *itsGridRobotCenterRatio.i && 
             fabs(dy) < IMMEDIATE_MAP_HEIGHT*itsGridRobotCenterRatio.j   )
            {
              itsImmediateDistances.push_back(dist);
              itsImmediateAngles.push_back(langle);
            }

          float pdx = dx/gridW;
          float pdy = dy/gridH;

          float ptx = cx - pdx; 
          float pty = cy - pdy;
          itsLRFgridMapCoordinates.push_back(Point2D<float>(ptx,pty));
          
          //LINFO("angle: %f langle: %f distance: %f: LRFmetricSelf: %f %f LRFgrid: %f %f",
          //      angle, langle, dist, o_dx/1000.0,o_dy/1000.0, ptx, pty);


          // not adding (.5,.5) offset 
          // because it's not about closest top left grid point 
          // but which grid the point(ptx,pty) actually falls to
          Point2D<int> pt (ptx, pty);

          // draw the ray: get the hit coordinate
          //std::vector<Point2D<int> > pts  = getLine(cpt, pt, res.getDims()); 

          // draw the ray: get the hit coordinate
          std::vector<Point2D<int> > pts  = 
            getLine(Point2D<float>(cx, cy), Point2D<float>(ptx, pty), w,h); 
          uint sj = 0;
          for(uint j = 0; j < pts.size(); j++)
            {
              if(res.coordsOk(pts[j]))
                {
                  // set grids to empty until the end of the laser ray
                  // or it hits filled grid
                  if(res.getVal(pts[j]) != 0.0)
                    {
                      res.setVal(pts[j], 1.0); // for empty grid
                    }
                  else j = pts.size();                    
                  sj++;
                }
            }

          if(res.coordsOk(pt)) res.setVal(pt, 0.0);

          // get the surroundings as well
      
          // add the count 
      
          // get the probability
      
          // add the probability
        }

      angle  += 1.0;
      langle += 1.0;
    }
  //LINFO(" ");
  
  return res;
}

// ######################################################################
Image<double> LocalMap::computeImmediateLocalMap()
{
  uint w = itsImmediateGridOccupancyMap.getWidth();
  uint h = itsImmediateGridOccupancyMap.getHeight();
  Image<double> res(w,h,ZEROS); res += LOCAL_MAP_DEFAULT_GRID_VALUE;
  Image<double> count(w,h, ZEROS);

  float imm_gridW = float(IMMEDIATE_MAP_GRID_WIDTH );
  float imm_gridH = float(IMMEDIATE_MAP_GRID_HEIGHT);
  
  // get the robot point location and heading
  // drawing after the laser drawing
  float cmapx = w*itsGridRobotCenterRatio.i;
  float cmapy = h*itsGridRobotCenterRatio.j; 

  float cx = cmapx + itsCurrAccTrajectory.i/imm_gridW;
  float cy = cmapy + itsCurrAccTrajectory.j/imm_gridH;
  Point2D<int> cpt(cx,cy);

  // draw gaussian likelihood through each line  
  double langle = -141.0;
  for(uint i = 0; i < itsDistances.size(); i++)
    {
      // skip invalid distances
      double dist  = itsDistances[i];
      double angle = langle + double(i);
      // double langle = itsAngles[i];

      // change invalid distances and set as maximum (10m)
      if(dist == -1.0){ dist = 10000; }

      // skip distances outside of +/-90 degrees
      // skip distances less than MIN_LRF_DIST: 5mm
      //   most likely garbage
      if(!(dist > MIN_LRF_DIST && (angle > -90.0 && angle < 90.0)))
        continue;

      float dx = dist*sin((double)angle*M_PI/180.0);
      float dy = dist*cos((double)angle*M_PI/180.0);
      
      float pdx = dx/imm_gridW;
      float pdy = dy/imm_gridH;
      
      float ptx = cx - pdx; 
      float pty = cy - pdy;                    
      
      Point2D<int> pt (ptx, pty);

      // draw the ray: get the hit coordinate
      std::vector<Point2D<int> > pts  = getLine(cpt, pt, w,h); 

      uint sj = 0;
      for(uint j = 0; j < pts.size(); j++)
        {
          if(res.coordsOk(pts[j]))
            {
              if(res.getVal(pts[j]) != 0.0)
                {
                  res.setVal(pts[j], 1.0);
                }
              else j = pts.size();
              sj++;
            }
        }

      if(res.coordsOk(pt)) res.setVal(pt, 0.0);      
    }
  return res;
}

// ######################################################################
Image<double> LocalMap::computeImmediateLocalMap(Image<float> map)
{
  uint w = itsImmediateGridOccupancyMap.getWidth();
  uint h = itsImmediateGridOccupancyMap.getHeight();
  Image<double> res(w,h,ZEROS); res += LOCAL_MAP_DEFAULT_GRID_VALUE;

  Image<double>::iterator aptr = res.beginw();

  // offset location
  float oi1 = LOCAL_MAP_HORZ_ORIGIN*LOCAL_MAP_NUM_HORZ_GRIDS;
  float oi2 = IMMEDIATE_MAP_HORZ_ORIGIN*IMMEDIATE_MAP_WIDTH/ 
    float(LOCAL_MAP_GRID_WIDTH);
  float oi =  oi1 - oi2;

  float oj1 = LOCAL_MAP_VERT_ORIGIN*LOCAL_MAP_NUM_VERT_GRIDS;
  float oj2 = IMMEDIATE_MAP_VERT_ORIGIN*IMMEDIATE_MAP_HEIGHT/ 
    float(LOCAL_MAP_GRID_HEIGHT);
  float oj = oj1 - oj2;
 
  float i_ratio = IMMEDIATE_MAP_GRID_WIDTH/float(LOCAL_MAP_GRID_WIDTH);
  float j_ratio = IMMEDIATE_MAP_GRID_HEIGHT/float(LOCAL_MAP_GRID_HEIGHT);
  
  // LINFO("oij[%f - %f = %f || %f - %f = %f]: %f %f", 
  //       oi1, oi2, oi, oj1, oj2, oj, i_ratio, j_ratio);
  
  // copy the coarse map values properly
  for(uint j = 0; j < h; j++)
    for(uint i = 0; i < w; i++)
      {
        float ii = oi + i*i_ratio;
        float jj = oj + j*j_ratio;

        // NOTE: can try to do interpolation
        *aptr++ = map.getVal(int(ii), int(jj));
        //res.setVal(i,j,map.getVal(int(ii), int(jj)));
      }

  return res;
}

// ######################################################################
std::vector<Point2D<int> > LocalMap::getLine
(Point2D<float> p1, Point2D<float> p2, const int w, const int h)
{
  std::vector<Point2D<int> > points;
  Point2D<int> pp1(p1.i, p1.j);
  Point2D<int> pp2(p2.i, p2.j);
  if(pp1 == pp2){ points.push_back(pp1); return points; }

  // ray tracing algorithm
  float dx = p2.i - p1.i, ax = fabs(dx); int sx = signOf(int(dx));
  float dy = p2.j - p1.j, ay = fabs(dy); int sy = signOf(int(dy));

  float x1 = p1.i, y1 = p1.j;         
  float x2 = p2.i, y2 = p2.j;         

  int   x  = x1, y  = y1;

  float xx = int(x1) + .5 - p1.i;
  float yy = int(y1) + .5 - p1.j;

  // LINFO("[%f %f] [%f %f] dy: %f dx: %f sx: %d sy %d ==> xx: %f yy: %f", 
  //       p1.i, p1.j, p2.i, p2.j, dy, dx, sx, sy, xx,yy); 
  if (ax > ay)
    {
      for (;;)
        {
          // assignment
          // if (x == int(p2.i)) 
          //   { points.push_back(pp2); return points; }

          if ((x >= x2) && sx ==  1) return points; 


          if (x >= 0 && x < w && y >= 0 && y < h)
            points.push_back(Point2D<int>(x,y));
          // LINFO("[[%3d]]: %3d %3d", int(points.size())-1, int(x), int(y));

          if ((x <= x2) && sx == -1) return points; 

          // if (x == int(p2.i))
          //   { 
          //     if(y != int(p2.j)) points.push_back(pp2); 
          //     return points; 
          //   }
           
          x += sx; xx+=sx;
          yy = p1.j + dy/dx * xx;  y = int(yy); 
          // LINFO("x: %d xx = %f yy = %f", x, xx,yy);

        }
    }
  else
    {
      for (;;)
        {
          if ((y >= y2) && sy ==  1) return points; 

          // assignment
          if (x >= 0 && x < w && y >= 0 && y < h)
            points.push_back(Point2D<int>(x,y));
          // LINFO("[[%3d]]: %3d %3d", int(points.size())-1, int(x), int(y));

          if ((y <= y2) && sy == -1) return points; 

          y += sy; yy+=sy;
          xx = p1.i + dx/dy * yy;  x = int(xx);
          // LINFO("y: %d yy = %f xx = %f", y, yy,xx);
        }
    }

  return points;
}

// ######################################################################
std::vector<Point2D<int> > LocalMap::getLine
(Point2D<int> p1, Point2D<int> p2, const int w, const int h)
{
  std::vector<Point2D<int> > points;
  
  // ray tracing algorithm
  // from Graphics Gems / Paul Heckbert
  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;
   
  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      for (;;)
        {
          // assignment
          if (x >= 0 && x < w && y >= 0 && y < h)
            points.push_back(Point2D<int>(x,y));
           
          if (x == p2.i) return points;
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      for (;;)
        {
          // assignment
          if (x >= 0 && x < w && y >= 0 && y < h)
            points.push_back(Point2D<int>(x,y));

          if (y == p2.j) return points;
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
    }

  return points;
}

// ######################################################################
Beobot2::MotorCommand LocalMap::getNavigationCommand()
{
  // compute command only every 40ms
  float ellapsed_time = itsTimer.get()/1000.0F;
  if(ellapsed_time < 50.0F) 
    {
      itsTotalNavCommandCount++;
      return itsCurrentMotorCommand;
    }

  Beobot2::MotorCommand cmd;

  // compute the trajectory and set the goal heading
  computeTrajectory();

  // if we are at goal, we stop
  if(itsPathCost == 0.0F)  
    { cmd.translation = 0.0F; cmd.rotation = 0.0F; return cmd; }

  // if there is no path to the goal
  if(itsPathCost < 0.0F)
    { 
      // go to a recovery mode -- turn in place
      cmd.translation = 0.0F; cmd.rotation = -0.8F; 
      itsSelfLocalHeading = 0.0;
      itsDesiredHeading   = 0.0;
      return cmd; 
    }

  // Dynamic Window Approach [Fox 1997] to compute motion command
  cmd = getDWAnavigationCommand();

  // non-linear transformation 
  // to account for Beobot 2.0 low motor response
  // in heading command close to 0  
  Beobot2::MotorCommand final_cmd =
    compensateLowVelocitySaturation(cmd);
  itsCurrentMotorCommand = final_cmd;

  itsTimer.reset();

  if(itsComputedNavCommandCount == 0) itsNavCommandTimer.reset();
  itsComputedNavCommandCount++;
  itsTotalNavCommandCount++;

  //float time = itsNavCommandTimer.get()/1000.0F;
  // LINFO("num cmd:(%d/%d) rate:(%fms/fr = %6.3f fps)/(%fms/fr = %6.3f fps)", 
  //       itsComputedNavCommandCount, itsTotalNavCommandCount, 
  //       time/itsComputedNavCommandCount, itsComputedNavCommandCount/(time/1000.0F),
  //       time/itsTotalNavCommandCount   , itsTotalNavCommandCount/(time/1000.0F));

  return final_cmd;
}

// ######################################################################
Beobot2::MotorCommand LocalMap::compensateLowVelocitySaturation
(Beobot2::MotorCommand cmd)
{
  // non-linear transformation to account for low motor response
  // in heading command close to 0

  double f_tra = cmd.translation;
  double rot   = cmd.rotation;
  double f_rot = cmd.rotation;

  // -1 to -.2
  if(rot >= -1.0 && rot <= -0.2)
    f_rot = (-1.0 - rot)/0.8 *-0.6 + -1.0;

  // -.2 to -.1
  else if(rot > -0.2 && rot <= -0.1)
    f_rot = (-0.2 - rot)/0.1 *-0.15 + -0.4;

  // -.1 to -.05
  else if(rot > -0.1 && rot <= -0.05)
    f_rot = (-0.1 - rot)/0.05 *-0.1 + -.25;

  // -.05 to 0
  else if(rot > -0.05 && rot <= 0.0)
    f_rot = (-0.05 - rot)/0.05 *-0.15 + -0.15;

  // 0 to 0.05 --> 0 to .15
  else if(rot > 0.00 && rot <= 0.05)
    f_rot = (rot)/0.05 * 0.15;

  // .05 to .1 --> .15 to .25 
  else if(rot > 0.05 && rot <= 0.10)
    f_rot = (rot - 0.05)/0.05 *0.1 + 0.15;

  // .1 to .2 --> .25 to .4
  else if(rot > 0.10 && rot <= 0.20)
    f_rot = (rot - 0.1)/0.1 *0.15 + 0.25;

  // .2 to 1.0 --> .4 to 1.0
  else if(rot > 0.20 && rot <= 1.0)
    f_rot = (rot - 0.2)/0.8*0.6 + 0.4;

  //LINFO(" (%5.2f %5.2f) --> (%5.2f %5.2f)", f_tra, rot, f_tra, f_rot);
  Beobot2::MotorCommand f_cmd;
  f_cmd.translation = f_tra; 
  f_cmd.rotation    = f_rot;
  return f_cmd;
}

// ######################################################################
void LocalMap::computeTrajectory()
{
  Timer tim(1000000);

  // setup the grid map edge weights
  setupGridMapEdgeWeights();
  //uint64 t1 = tim.get();

  uint w = itsGridOccupancyMap.getWidth();
  uint h = itsGridOccupancyMap.getHeight();

  // set the current goal internal node
  // current internal node ALWAYS in the middle of the map
  // need a (1,1) offset because the grid map for shortest path 
  // includes 1 pixel slacks all around
  Point2D<int> current_internal_node
    (w*itsGridRobotCenterRatio.i+1, h*itsGridRobotCenterRatio.j+1);
  Point2D<int> goal_internal_node    = itsGoalNode + Point2D<int>(1,1);

  // compute the shortest path
  std::vector<Point2D<int> > steps; 
  float pathCost = itsGridMap->getShortestPath
    (current_internal_node, goal_internal_node, steps);
  uint org_steps = steps.size();

  //uint64 t2 = tim.get();

  // modify output path with respect to current task
  steps = modifyPathForCurrentTask(steps, pathCost);

  //uint64 t3 = tim.get();

  itsPathCost  = pathCost;
  itsPathSteps.clear();
  for(uint i = 0; i < steps.size(); i++) 
    itsPathSteps.push_back(steps[i]);
  itsPath.clear(); 

  if(steps.size() == 0)
    { LINFO("NO PATH TO GOAL: %d org steps", org_steps); return; }
  
  // NOTE: add (-1,-1) offset to the current internal node 
  //       because the shortest path internal grid map 
  //       includes 1 pixel slacks all around
  //       also add (.5,.5) offset 
  //       because the path is through the middle of the grid square
  Point2D<float> offset(-1+.5,-1+.5);
  itsPath.push_back(current_internal_node + offset);
  for(uint i = 0; i < steps.size(); i++)
    itsPath.push_back(itsPath[i] + steps[i]);

  // change the last location 
  // to the more accurate float goal location
  //itsPath[itsPath.size()-1] = itsGoalLocation;

  // smooth the path
  itsSmoothedPath = smoothPath(itsPath);

  //uint64 t4 = tim.get();

  // account for robot geometry
  itsSmoothedPath = modifyPathForObstacleAvoidance();

  //uint64 t5 = tim.get();

  // set the goal heading for the next step
  generateDesiredHeading();

  //uint64 t6 = tim.get();

  // LINFO("set: %7.2fms + getpath: %7.2fms + modTask: %7.2fms + smooth: %7.2f +"
  //       "modObs: %7.2f + getDes: %7.2f = %7.2f",
  //       t1/1000.0F, (t2-t1)/1000.0F, (t3-t2)/1000.0F, (t4-t3)/1000.0F, 
  //       (t5-t4)/1000.0F, (t6-t5)/1000.0F, t6/1000.0F);
}

// ######################################################################
std::vector<Point2D<int> > LocalMap::modifyPathForCurrentTask
(std::vector<Point2D<int> > steps, float &pathCost)
{  
  std::vector<Point2D<int> > modified_steps;
  uint w = itsGridOccupancyMap.getWidth();
  uint h = itsGridOccupancyMap.getHeight();
  Point2D<int> curr_node
    (w*itsGridRobotCenterRatio.i,h*itsGridRobotCenterRatio.j);
  Point2D<int> goal_node = itsGoalNode;

  // check if robot is in turning procedure
  // this is most important command 
  // because we want to execute it as quickly as possible
  bool isTurning = isInTurnProcedure();
  if(isTurning)
    {
      // turning must have a path or not yet in goal
      if(steps.size() > 0 || pathCost == 0.0F) return steps;

      // get a straightline path to the goal
      std::vector<Point2D<int> > tpath = 
        getLine(curr_node, goal_node,w,h);
      if(tpath.size() <= 1) 
        { pathCost = 0.0F; return modified_steps; }

      // set path to be as long as possible 
      // without hitting an obstacle 
      // but at least 3 STEPS or to the goal
      
      // just add three steps to that direction
      int max_steps = int(tpath.size())-1; 
      int num_steps = 0;
      int min_steps = 3; 
      if(min_steps > max_steps) min_steps = max_steps;       
      for(int i = 0; i < min_steps; i++)
        {
          modified_steps.push_back(tpath[i+1]-tpath[i]);
          num_steps++;
        }

      if(num_steps >= max_steps) 
        { pathCost = float(num_steps); return modified_steps; }
      
      // now add path as much as possible
      Point2D<int> curr_node = tpath[num_steps];         
      bool keep_going = 
        (itsGridOccupancyMap.coordsOk(curr_node) &&
         itsGridOccupancyMap.getVal(curr_node) > 0.2);
      while(keep_going && num_steps < max_steps)
        {
          Point2D<int> next_node = tpath[num_steps+1];
          keep_going = (itsGridOccupancyMap.coordsOk(next_node) &&
                        itsGridOccupancyMap.getVal(next_node) > 0.2); 

          if(keep_going)
            { 
              modified_steps.push_back(next_node - curr_node); 
              num_steps++; 
            }
          curr_node = next_node;
        }

      // FIXXX what to do with itsDeadEndIndicator      
      pathCost = float(num_steps);
      return modified_steps;
    }

  // in normal non-turning or going to the goal mode
  // FIXXX NEEDS DEBUGGING
  if(itsGridOccupancyMap.coordsOk(itsGoalNode))
    {
      // if can't get to the goal
      if(steps.size() == 0)
        {
          // FIXXX: draw a line to the goal 
          // getLine(itsCurrentNode, itsGoalNode);
          // move to that direction as much as possible
          // that is that it does not hit an obstacle
          
          // if minimum moves of 3 is not possible
          // stop and send a message up 
          // (to localizer or task manager)
          // that robot cannot go to goal
        }

      // FIXXX what to do with itsDeadEndIndicator      
    }

  // in normal mode still can be moving to a dead-end

  // check if robot is stuck and needs to move backwards
  bool move_back = false; int curr_j = 0;  
  for(uint i = 0; i < steps.size(); i++)
    {   
      curr_j += steps[i].j;
      if(curr_j > 0){ move_back = true; i = steps.size(); }
    }

  // keep going until robot is within 2 robot size to obstacle
  if(move_back)
    {
      // check how far forward we can go
      Point2D<int> forward_move(0,-1);
      Point2D<int> curr_node = 
        Point2D<int>(int(itsGridRobotCenterRatio.i*w),
                     int(itsGridRobotCenterRatio.j*h) ) + forward_move;
      bool keep_going = true;
      uint num_forward_move = 0;
      while(keep_going)
        {
          keep_going = itsGridOccupancyMap.coordsOk(curr_node);
          if(keep_going) 
            {
              float val = itsGridOccupancyMap.getVal(curr_node);
              //LINFO("[%3d %3d]: %f", curr_node.i, curr_node.j, val);
              if(val > 0.2) num_forward_move++;
              else           keep_going = false;
              curr_node += forward_move;
            }
        }

      // check if there is still enough space to move forward
      steps.clear();
      if(num_forward_move >= 10 && itsDeadEndIndicator != DEAD_END) 
        {
          itsDeadEndIndicator = NEAR_DEAD_END;
          pathCost = float(num_forward_move);
          for(uint i = 0; i < num_forward_move; i++)
            steps.push_back(forward_move);
        }
      else 
        { itsDeadEndIndicator = DEAD_END; pathCost = -1.0F; }
      //LINFO("MOVING FORWARD: %d steps size(%d) :: ddi %d", 
      //      num_forward_move, int(steps.size()), itsDeadEndIndicator);
    }
  else itsDeadEndIndicator = CAN_REACH_GOAL;
  //LINFO("ddi: %d m_back: %d in turn proc: %d steps: %d",
  //      itsDeadEndIndicator, move_back, isInTurnProcedure(), 
  //      int(steps.size())); 

  return steps;
}

// ######################################################################
void LocalMap::setupGridMap()
{  
  int nhorz = itsNumHorzGrid;
  int nvert = itsNumVertGrid;

  Timer tim(1000000);

  // adding outer layer 
  // to enable goal setting beyond the range of local map
  itsGridMap.reset(new GridMap(nhorz+2, nvert+2));
  LINFO("time: %f",tim.get()/1000.0F);
}

// ######################################################################
void LocalMap::setupGridMapEdgeWeights()
{  
  int nhorz    = itsNumHorzGrid;
  int nvert    = itsNumVertGrid;
  float diag   = sqrt(nhorz*nhorz + nvert*nvert); 
  float diagSq = diag*diag; 

  Timer tim(1000000);

  // setup the neighborhood weights
  int n_size = GRID_INFLUENCE_EXTENT;   

  float maxDist = sqrt(2*(n_size+1)*(n_size+1));
  std::vector<std::pair<Point2D<int>,float> > neighbor;
  for (int jj = -n_size ; jj <= n_size; jj++) 
    for (int ii = -n_size ; ii <= n_size; ii++) 
      {
        float dist = sqrt(ii*ii + jj*jj);
        float dweight = 1.0 - dist/maxDist;
        neighbor.push_back
          (std::pair<Point2D<int>,float>(Point2D<int>(ii,jj),dweight));
        //LINFO("[%3d %3d] dist: %f dweight: %f", ii,jj, dist,dweight);
      }

  tim.reset();  

  // setup weight map to account for previous path
  Image<float> prevGridMap(nhorz, nvert, ZEROS);
  if(itsPath.size() > 0)
    {
      Image<float>::iterator pptr = prevGridMap.beginw();

      // for each point in the map
      for(int j = 0; j < nvert; j++)
        for(int i = 0; i < nhorz; i++)
          {
            // compute the closest location to path
            // approximated by just abs(di) + abs(dj)
            // not: sqrt(di*di + dj*dj);
            // int di = abs(itsPath[0].i - i);
            // int dj = abs(itsPath[0].j - j);
            int di = int(itsPath[0].i) - i;
            int dj = int(itsPath[0].j) - j;
            float min_dist = di*di+dj*dj;//sqrt(di*di + dj*dj);
            for(uint k = 1; k < itsPath.size(); k++)
              {
                if(!prevGridMap.coordsOk(itsPath[k])) continue;

                // di = abs(itsPath[k].i - i);
                // dj = abs(itsPath[k].j - j);
                di = int(itsPath[k].i) - i;
                dj = int(itsPath[k].j) - j;
                float dist = di*di+dj*dj;//sqrt(di*di + dj*dj);
                if(min_dist > dist) min_dist = dist;
              }

            // weight it with the distance to last path
            // set to zero if within 2 grid length
            if(min_dist < 2.0) min_dist = 0.0F;
            float value = min_dist/diagSq*4;
            //if(value > .25F) value = .25F;
            *pptr++ =  value;
          }
    }

  //LINFO("time: %f", tim.get()/1000.0F);
  tim.reset();

  // blurred weight grid occupancy map
  Image<float>::iterator pptr = prevGridMap.beginw();
  Image<float> wGridMap(nhorz, nvert, ZEROS);
  Image<float>::iterator aptr = wGridMap.beginw();


  // threshold for minimum distance to an obstacle
  // any closer would hit the obstacle
  float rs_thresh = 1.0 - MINIMUM_DISTANCE_TO_OBSTACLE/maxDist;

  //int gi = itsGoalNode.i;
  //int gj = itsGoalNode.j;

  // go through each grid location
  // to compute the occupancy value
  for(int j = 0; j < nvert; j++)
    for(int i = 0; i < nhorz; i++)
      {
        // obstacle weight value
        float maxVal = 0.0F; 
        for(uint dij = 0; dij < neighbor.size(); dij++)
          {
            Point2D<int> dp = neighbor[dij].first;
            double dweight  = neighbor[dij].second;

            int di = i+dp.i; int dj = j+dp.j;
            if(di < 0 || di >= nhorz || dj < 0 || dj >= nvert)
              continue;

            float val = itsGridOccupancyMap.getVal(di,dj);
            
            // NOTE: in itsGridOccupancyMap 0.0 means filled with obstacle
            //       but in itsGridMap innerworking 0.0 means free
            //       that is, it costs less to go through that grid
            if(val == LOCAL_MAP_DEFAULT_GRID_VALUE) 
              {
                if(dp.i == 0 && dp.j == 0) val = 0.99F; //1.0 - .01;
                else val = 0.00F;
              }
            else val = 1.0F - val;

            float wval = val*dweight;

            // all grid next to a hit grid is automatically filled
            if(val == 1.0F && dweight >= rs_thresh) wval = 1.0F;             

            if(wval > maxVal) maxVal = wval;
          }

        // also add weights to incorporate previous path
        // so that path used is similar to the previous path
        if(maxVal < 0.99F)
          {
            // cannot make a grid untraversable 
            // because of adding history
            float total_val = maxVal + *pptr++;

            // add value to everything else but straight path to goal
            // for faster lateral deviation correction
            //if(gj == -1 && gi != i) total_val += 0.001F;

            if(total_val > .99F) total_val = .99F;
            *aptr++ = total_val;
          }
        else{ *aptr++ = maxVal; pptr++; }
      }

  //uint tBlur = tim.get();
  //LINFO("Time 1   : %f  |neighbor size: %d", tBlur/1000.0F, int(neighbor.size()));

  // setup the internal grid map for the shortest path run
  // which includes 1 pixel slacks all around
  Image<float> internal_grid_map(nhorz+2,nvert+2,NO_INIT);
  Image<float>::iterator wptr = wGridMap.beginw();
  for(int j = 0; j < nvert; j++)
    for(int i = 0; i < nhorz; i++)
      {  
        float val = *wptr++;
        internal_grid_map.setVal(i+1,j+1, val);
      }

  // outer goal areas are unknown
  for(int i = 0; i < nhorz; i++)
    {
      internal_grid_map.setVal(i, 0      , 0.99F);
      internal_grid_map.setVal(i, nvert+1, 0.99F);
    }

  for(int j = 0; j < nvert; j++)
    {
      internal_grid_map.setVal(0      , j, 0.99F);
      internal_grid_map.setVal(nhorz+1, j, 0.99F);
    }

  // uint t2 = tim.get();
  // LINFO("Time 2   : %f  || %f", (t2-tBlur)/1000.0F, t2/1000.0F);

  // update the gridmap
  itsGridMap->updateGridMap(internal_grid_map);


  // bias the edges on the current robot location node
  // according to the robot current self heading
  Point2D<int> robot_loc
    (nhorz*itsGridRobotCenterRatio.i+1, nvert*itsGridRobotCenterRatio.j+1);
  std::vector<float> weights = itsGridMap->getDirectedEdgeWeights(robot_loc);
  uint index = 0;
  for (int ii = -1; ii <= 1; ii++) 
    for (int jj = -1; jj <= 1; jj++) 
      {
        if(ii == 0 && jj == 0) continue;

        // FIXXX maybe need fixing

        Point2D<int> v(-jj, ii);  // FIXXX BUG should it be -ii
        float dist = sqrt(v.i*v.i+v.j*v.j);

        double ang = atan2(v.j, v.i);
      
        double head = itsSelfLocalHeading;
      
        double rot = ang;// + M_PI/2;
        //if(rot > M_PI) rot = 2*M_PI - rot;
      
        double diff = rot - head;
        if(diff > M_PI)       diff = diff - 2*M_PI;
        else if(diff < -M_PI) diff = diff + 2*M_PI;
        
        float rotVal = float(fabs(diff/M_PI))/2.0F; 
      
        float weight = weights[index];        
        weight += rotVal*dist;
        weights[index] = weight;
        
        //LINFO(">>%4d %4d]a:%5.2f r::%5.2f h:%5.2f d: %5.2f v: %5.2f --> %f --> %f", 
        //      v.i, v.j, ang, rot, head, diff, rotVal, dist, weight);
        index++;
      }
  itsGridMap->setDirectedEdgeWeights(robot_loc, weights); 


  // uint t3 = tim.get();
  // LINFO("Time 3   : %f  || %f", (t3-t2)/1000.0F, t3/1000.0F);
}

// ######################################################################
std::vector<Point2D<float> > LocalMap::smoothPath
(std::vector<Point2D<float> > path)
{
  std::vector<Point2D<float> > smoothedPath;
  Timer tim(1000000);

  // path smoothing constants
  float weight_data   = PATH_SMOOTHING_DATA_WEIGHT;
  float weight_smooth = PATH_SMOOTHING_SMOOTHNESS_WEIGHT;
  float tolerance     = PATH_SMOOTHING_TOLERANCE;

  for(uint i = 0; i < path.size(); i++) 
    smoothedPath.push_back(Point2D<float>(path[i].i, path[i].j));
  if(path.size() <= 2) return smoothedPath;

  // iterate to smooth path
  float change = tolerance; uint num_iteration = 0;
  float previous_change = 0.0F;
  while(change >= tolerance && 
        !(num_iteration > 0 && fabs(change - previous_change) < tolerance))
    {
      previous_change = change;
      change = 0.0F;

      // skip the first and last points in the path
      // because they are the starting and ending points
      float spi_m1 = smoothedPath[0].i;
      float spj_m1 = smoothedPath[0].j;

      for(uint i = 1; i < smoothedPath.size()-1; i++) 
        {          
          float pi = path[i].i;
          float pj = path[i].j;

          float spi = smoothedPath[i].i;
          float spj = smoothedPath[i].j;

          float spi_p1 = smoothedPath[i+1].i;
          float spj_p1 = smoothedPath[i+1].j;

          smoothedPath[i].i += weight_data *(pi - spi);
          smoothedPath[i].j += weight_data *(pj - spj);

          smoothedPath[i].i += weight_smooth * (spi_p1 + spi_m1 - 2.0F*spi);
          smoothedPath[i].j += weight_smooth * (spj_p1 + spj_m1 - 2.0F*spj);

          spi_m1 = spi;
          spj_m1 = spj;

          float di = (spi - smoothedPath[i].i);
          float dj = (spj - smoothedPath[i].j);

          change += sqrt(di*di + dj*dj);
        }
      num_iteration++;
    }
 
  return smoothedPath;
}

// ######################################################################
std::vector<Point2D<float> > LocalMap::modifyPathForObstacleAvoidance()
{
  // Elastic Band algorithm:
  // http://adrianboeing.blogspot.com/2012/03/elastic-band-realtime-pathfinding.html
  //   iteratively add influence of the closest obstacle 
  //   to deform the straightline A* path 
  //   also add an attractive force to want to straighten out the path

  // NOTE: all calculations in this function is in gridmap unit
  //       1 grid is = LOCAL_MAP_GRID_WIDTH x LOCAL_MAP_GRID_HEIGHT 
  //       specified in src/Robots/Beobot2/BeoCommon.H

  Timer tim(1000000);

  std::vector<Point2D<float> > path  = itsSmoothedPath;
  uint path_size = path.size(); 
  if(path_size <= 2) return path;

  std::vector<Point2D<float> > modified_path;
  for(uint i = 0; i < path_size; i++)
    modified_path.push_back(path[i]);

  uint w = itsGridOccupancyMap.getWidth();
  uint h = itsGridOccupancyMap.getHeight();

  Image<std::vector<Point2D<float> > > lrfMap(w,h, ZEROS);
  int n_size = LRF_MAP_NEIGHBORHOOD_SIZE; 

  float max_dist = n_size;
  std::vector<Point2D<int> > neighbor;
  for (int jj = -n_size; jj <= n_size; jj++) 
    for (int ii = -n_size; ii <= n_size; ii++)     
      neighbor.push_back(Point2D<int>(ii,jj));

  // for each grid location
  for(uint i = 0; i < itsLRFgridMapCoordinates.size(); i++)
    {
      // store LRF point within n_size
      Point2D<float> lrf_pt = itsLRFgridMapCoordinates[i]; 
      Point2D<int> lrf_loc(lrf_pt.i, lrf_pt.j);
      for(uint j = 0; j <= neighbor.size(); j++)
        {
          Point2D<int> loc = lrf_loc + neighbor[j];
          if(lrfMap.coordsOk(loc)) lrfMap[loc].push_back(lrf_pt);
        }      
    }
  itsLRFmap = lrfMap;

  //LINFO("lrf time: %f", tim.get()/1000.0F);

  int m_path_size = modified_path.size();

  // iterate through the LRF points for each segment
  int num_iteration = 0; int max_iteration = 20;
  itsClosestObjDistOnPath.clear();
  itsClosestObjDistOnPath.resize(m_path_size);
  itsClosestObjDistOnPath[0] = 0.0;
  while(num_iteration < max_iteration)
    {
      // copy the path first
      std::vector<Point2D<float> > tm_path(m_path_size); 
      //for(int i = 0; i < m_path_si0ze; i++)
      //  tm_path[i] = modified_path[i];
            
      // go through each point
      for(int i = 1; i < m_path_size - 1; i++)
        {
          Point2D<float> loc = modified_path[i];
          //LINFO("loc[%3d]: %f %f", i, loc.i, loc.j);

          Point2D<int> pt(loc.i, loc.j);
          if(!lrfMap.coordsOk(pt)) continue;
          std::vector<Point2D<float> > lrf_points = lrfMap.getVal(pt);
          
          // get the repulsive forces: avoid obstacles
          float min_dist = -1.0F;
          Point2D<float> rep_force(0,0);
          Point2D<float> obstacle(0,0);
          for(uint j = 0; j < lrf_points.size(); j++)
            {
              float dist = lrf_points[j].distance(loc);

              // check for closest lrf point
              if(min_dist == -1.0F || min_dist >= dist)
                {
                  min_dist = dist;
                  obstacle = lrf_points[j];
                }
            }
          if(min_dist != -1.0F)
            {
              //LINFO("    obs: %f %f", rep_force.i, rep_force.j);
              rep_force = loc - obstacle;
              
              float r_i = rep_force.i;
              float r_j = rep_force.j;
              float r_mag = sqrt(r_i*r_i + r_j*r_j);
              //LINFO("r_i: %f r_j: %f: mag: %f", r_i, r_j, r_mag);
              
              float n_mag = 2.0F;
              if(r_mag != 0) n_mag = (max_dist - r_mag)/r_mag;
              if(n_mag < 0) 
                { n_mag = 0; rep_force = Point2D<float>(0,0); }
              else 
                { 
                  rep_force.i = r_i/r_mag*n_mag;  
                  rep_force.j = r_j/r_mag*n_mag;
                }
              //LINFO("n_mag: %f", n_mag);
              itsClosestObjDistOnPath[i] = min_dist;
            }
          // get the attractive force
          Point2D<float> prev_pt = modified_path[i-1];
          Point2D<float> next_pt = modified_path[i+1];
          Point2D<float> mid_pt  = (prev_pt+next_pt)/2.0F;
          Point2D<float> att_force = mid_pt - loc;
          // printf("[%5.2f %5.2f] p: %5.2f %5.2f -- n: %5.2f %5.2f --> "
          //        "mid: %5.2f %5.2f --> att: %5.2f %5.2f\n",
          //        loc.i, loc.j,
          //        prev_pt.i, prev_pt.j, next_pt.i, next_pt.j,
          //        mid_pt.i, mid_pt.j, att_force.i, att_force.j);

          // dampen velocity to ensure convergence
          // there is no explicit convergence because 
          // the obstacle pushes the robot 1 at a time 
          //float weight = .2 - .01*num_iteration;
          float weight = 
            0.2F*(1.0F - float(num_iteration)/float(max_iteration));

          // integrate both with velocity
          Point2D<float> comb_force = 
            (att_force + rep_force*2.0F)/3.0F * weight;

          //if(att_force.i != 0.0)            

          // printf("[%3d][%3d] rep(%5.2f %5.2f): %5.2f %5.2f -- "
          //        "att: %5.2f %5.2f w: %5.2f: comb: %5.2f %5.2f \n", 
          //        num_iteration, i, rep_force.i, rep_force.j, 
          //        obstacle.i, obstacle.j,
          //        att_force.i, att_force.j,
          //        weight, comb_force.i, comb_force.j);
          
          tm_path[i] = loc + comb_force;

          // check if prev to current node is too far 
          // if it is, has to split the segment          
        }

      // FIXXX: should we not update the second to last one 
      //        in case an obstacle is next to goal 
      for(int i = 1; i < m_path_size-1; i++)
        modified_path[i] = tm_path[i];

      num_iteration++;
    }

  return modified_path;
}

// ######################################################################
void LocalMap::generateDesiredHeading()
{
  // given smoothed path that already accounted for robot geometry
  // generate the optimal first step
  // by taking into account: 
  //    how close to the smoothed path heading (.4 weight)
  //    how close to obstacle near the robot (.4 weight)
  //    whether the direction will hit the path (.2 weight)

  if(itsSmoothedPath.size() <= 1) 
    {
      itsDesiredHeading = 0.0;
      LINFO("NOT MOVING: heading = 0.0 degrees"); 
      return;
    }

  Timer tim(1000000);

  // get angle already accounting for path to goal and obstacle
  Point2D<float> step    = itsSmoothedPath[1] - itsSmoothedPath[0];
  float step_length      = step.distance(Point2D<float>(0,0));
  float step_ang         = atan2(-step.i, -step.j);
  Point2D<float> org_pt  = itsSmoothedPath[0];
  Point2D<float> next_pt = itsSmoothedPath[1];
  float d_ang            = 5.0F/180.0F*M_PI;  
  float org_ang          = step_ang;
  // LINFO("[%5.2f %5.2f] [%5.2f %5.2f] --> [%5.2f %5.2f]: %f degrees", 
  //       org_pt.i, org_pt.j, next_pt.i, next_pt.j, 
  //       step.i, step.j, step_ang/M_PI*180.0F);

  // for loop of +/- 45 degrees of angle
  float range_ang = d_ang*9.0F;
  float r_ang   = step_ang - range_ang;
  float l_ang   = step_ang + range_ang;

  float max_val = -1.0;
  float max_ang = org_ang;
  Point2D<float> max_m_pt =  next_pt;
  float max_traj_dist = sqrt(2.0)*LOCAL_MAP_NUM_HORZ_GRIDS;

  for(float ang = r_ang; ang <= l_ang;  ang+= d_ang)
    {
      // compute the new point
      float di = -step_length * sin(ang);
      float dj = -step_length * cos(ang);
      Point2D<float> curr_m_pt(org_pt.i + di, org_pt.j + dj);

      // LINFO("ang: %f ==> %f %f ==> curr_m_pt: %f %f", 
      //       ang/M_PI*180.0F, di, dj, curr_m_pt.i, curr_m_pt.j);
      
      // compute distance to closest obstacle
      std::vector<Point2D<float> > m_body = 
        getRobotBody(curr_m_pt, ang);      
      float dist = distanceToClosestObstacle(m_body);

      // take out angle that will hit obstacle
      if(dist < 0.0F) continue;

      // calculate difference to ideal angle
      float diff_ang = ang - org_ang;

      // evaluate angle based on: 

      // closer to original angle
      float ang_val = 1.0F - fabs(diff_ang)/range_ang;

      // angle that optimize distance from obstacle
      // --> up to 1 pixel width, after that saturates	  
      // quadratic weight, could be linear or just all or nothing
      float tdist = dist; if(tdist > 1.0F) tdist = 1.0F;
      float dist_val = 1.0F - (1.0F - tdist)*(1.0F - tdist);

      // get unobstructive distance to trajectory
      float traj_dist = -1.0F;

      // zero angle 0 is skipped automatic 0.0
      if(diff_ang == 0.0F) traj_dist = 0.0F;
      else traj_dist = getDistanceToTrajectory(curr_m_pt, ang);

      // if the angle hits the trajectory of path 
      // the value is positive, otherwise negative
      // the magnitude of the distance is the closest obstacle it hits
      float traj_val = 0.0F;
      if(traj_dist > 0 && traj_dist < max_traj_dist) { traj_val = 1.0F; }

      // the closer intersection the better ??? ALL OR NOTHING (LIKE A BONUS): .1weight
      // within 3 robot length (linear or quadratic???)
      // not hit = 0.0 --> closest angle will deal with that issue
        
      // distance before it hits an obstacle

      // update optimal angle
      float val = .4F*ang_val + .4F*dist_val + .2F*traj_val;
      // LINFO("%f = .4F*%f + .4F*%f + .2F*%f --> dist: %f, diff: %f traj_dist: %f", 
      //       val, ang_val, dist_val, traj_val, dist, diff_ang, traj_dist); 

      if(val > max_val) 
        { max_val = val; max_ang = ang; max_m_pt = curr_m_pt; }
    }

  // range to +/- M_PI
  if(max_ang >  M_PI)      max_ang = max_ang - 2*M_PI;
  else if(max_ang < -M_PI) max_ang = max_ang + 2*M_PI;
    
  // set the desired heading
  itsSmoothedPath[1]      = max_m_pt;
  itsDesiredHeading       = max_ang;
  // LINFO("final first step: %f degrees: %f %f time: %f", 
  //       max_ang/M_PI*180.0F, max_m_pt.i, max_m_pt.j, tim.get()/1000.0F);
}

// ######################################################################
float LocalMap::distanceToClosestObstacle
(std::vector<Point2D<float> > m_body)
{
  float min_dist = LRF_MAP_NEIGHBORHOOD_SIZE;
  
  Point2D<float> center;
  for(uint i = 0; i < m_body.size()-1; i++) center += m_body[i];
  center /= 4.0F;
  // LINFO("[%f %f][%f %f][%f %f][%f %f] --> %f %f", 
  //       m_body[0].i, m_body[0].j, m_body[1].i, m_body[1].j,
  //       m_body[2].i, m_body[2].j, m_body[3].i, m_body[3].j,
  //       center.i, center.j);

  Point2D<int> c_coord(center.i, center.j);

  std::vector<Point2D<float> > lrfCoords;
  if(itsLRFmap.coordsOk(c_coord)) lrfCoords = itsLRFmap.getVal(c_coord);

  // go through all the obstacles 
  // find closest obstacle in this configuration
  bool any_hit = false;
  for(uint i = 0; i < lrfCoords.size() && !any_hit; i++)
    {
      Point2D<float> hit_pt = lrfCoords[i];
      bool hit = pnpoly(m_body, hit_pt);
      if(hit){ any_hit = true;  min_dist = -1.0F; }
      else
        {
          float dist = distanceFromPoly(hit_pt, m_body);
          if(dist < min_dist) min_dist = dist;
        }
    }
     
  return min_dist;
}

// ######################################################################
float LocalMap::getDistanceToTrajectory
(Point2D<float> curr_m_pt, float ang)
{
  float distance = sqrt(2.0)*LOCAL_MAP_NUM_HORZ_GRIDS;

  Point2D<float> e_pt(curr_m_pt.i - sin(ang), curr_m_pt.j - cos(ang));

  //LINFO("[%f %f] [%f %f]", curr_m_pt.i, curr_m_pt.j, e_pt.i, e_pt.j);

  for(uint i = 0; i < itsSmoothedPath.size()-1; i++)
    {
      // get distance to segment
      Point2D<float> p3 = itsSmoothedPath[i  ];
      Point2D<float> p4 = itsSmoothedPath[i+1];
      float int_dist = distanceToSegment(curr_m_pt, e_pt,p3,p4);

      //LINFO("  with [%f %f]-[%f %f]: %f", p3.i, p3.j, p4.i, p4.j, int_dist);

      // skip if it does not intersect the segment
      if(int_dist < 0.0F) continue;
        
      // do geometry check until it hits an obstacle 
      // or finish to the intersect point
      bool hit = false;
      float dist = 0.0F; 
      while(dist < int_dist && !hit)
        {
          dist+= 4.0F; // 1 robot length in pixel unit
          if(dist > int_dist) dist = int_dist;

          Point2D<float> pt
            (curr_m_pt.i - dist*sin(ang), curr_m_pt.j - dist*cos(ang));
          std::vector<Point2D<float> > c_body = getRobotBody(pt, ang);      

          Point2D<float> center;
          for(uint i = 0; i < c_body.size()-1; i++) center += c_body[i];
          center /= 4.0F;

          Point2D<int> c_coord(center.i, center.j);
          
          std::vector<Point2D<float> > lrfCoords;
          if(itsLRFmap.coordsOk(c_coord)) 
            lrfCoords = itsLRFmap.getVal(c_coord);

          // go through all the obstacles 
          // find closest obstacle in this configuration
          for(uint i = 0; i < lrfCoords.size() && !hit; i++)
            {
              hit = pnpoly(c_body, lrfCoords[i]);
              //if(hit) LINFO("lrf: %f %f", lrfCoords[i].i, lrfCoords[i].j);
            }

          //LINFO("  move: %f [%f %f]: HIT???: %d", dist, pt.i, pt.j, hit);
        }

      distance = dist;

      // negative means we hit an obstacle first
      if(hit) distance *= -1.0F;
      break;
    }

  return distance;
}

// ######################################################################
float LocalMap::distanceToSegment
(Point2D<float> p1, Point2D<float> p2, 
 Point2D<float> p3, Point2D<float> p4)
{
  float int_dist = -1.0F;
  
  Point2D<float> int_pt = lineIntersectPoint(p1,p2, p3,p4);
  // LINFO("[%f %f] [%f %f] -- [%f %f] [%f %f] --> %f %f", 
  //       p1.i, p1.j, p2.i, p2.j, p3.i, p3.j, p4.i, p4.j,
  //       int_pt.i, int_pt.j);

  Point2D<float> lpt = p3; Point2D<float> rpt = p4;
  if (p4.i < p3.i) { lpt = p4; rpt = p3; }

  Point2D<float> tpt = p3; Point2D<float> bpt = p4;
  if (p4.j < p3.j) { tpt = p4; bpt = p3; }

  float di = p2.i - p1.i;
  float dj = p2.j - p1.j;

  // if it's in the segment and 
  if(int_pt.i >= lpt.i && int_pt.i <= rpt.i && 
     int_pt.j >= tpt.j && int_pt.j <= bpt.j && 
     ((di >= 0 &&  p1.i <= int_pt.i) || (di < 0 &&  p1.i > int_pt.i)) &&
     ((dj >= 0 &&  p1.j <= int_pt.j) || (dj < 0 &&  p1.j > int_pt.j))    )
    {
      int_dist = p1.distance(int_pt); 
      //LINFO("HIT    intersect: %f %f: dist: %f", int_pt.i,int_pt.j, int_dist);
    }
  //else LINFO("NO HIT intersect: %f %f: dist: %f", int_pt.i,int_pt.j, int_dist);

  return int_dist;
}

// ######################################################################
std::vector<Point2D<float> > LocalMap::getRobotBody
(Point2D<float> front_center, double angle)
{
  // get the front center of the body
  // Point2D<float> center
  //   (body.left()+body.width()/2.0F, body.top());

  Point2D<float> fc = front_center;

  float robot_width  = 
    float(BEOBOT2_ROBOT_WIDTH) /float(LOCAL_MAP_GRID_WIDTH);
  float robot_height = 
    float(BEOBOT2_ROBOT_HEIGHT)/float(LOCAL_MAP_GRID_HEIGHT);

  // get the four corners
  Point2D<float> tl = 
    front_center + Point2D<float>(-robot_width/2, 0);
  Point2D<float> tr = 
    front_center + Point2D<float>( robot_width/2, 0);
  Point2D<float> bl = 
    front_center + Point2D<float>(-robot_width/2, robot_height);
  Point2D<float> br = 
    front_center + Point2D<float>( robot_width/2, robot_height);

  float cAng = cos(angle);
  float sAng = sin(angle);

  // rotate the 4 corners properly
  Point2D<float> rtl
    (fc.i + (tl.i - fc.i)*cAng + (tl.j - fc.j)*sAng,
     fc.j - (tl.i - fc.i)*sAng + (tl.j - fc.j)*cAng );

  Point2D<float> rtr
    (fc.i + (tr.i - fc.i)*cAng + (tr.j - fc.j)*sAng,
     fc.j - (tr.i - fc.i)*sAng + (tr.j - fc.j)*cAng );

  Point2D<float> rbl
    (fc.i + (bl.i - fc.i)*cAng + (bl.j - fc.j)*sAng,
     fc.j - (bl.i - fc.i)*sAng + (bl.j - fc.j)*cAng );

  Point2D<float> rbr
    (fc.i + (br.i - fc.i)*cAng + (br.j - fc.j)*sAng,
     fc.j - (br.i - fc.i)*sAng + (br.j - fc.j)*cAng );

  //  LINFO("center: %f %f tl: %f %f tr: %f %f bl: %f %f br: %f %f", 
  //     fc.i, fc.j, tl.i, tl.j, tr.i, tr.j, bl.i, bl.j, br.i, br.j);
  //LINFO("rtl: %f %f rtr: %f %f rbl: %f %f rbr: %f %f", 
  //      rtl.i, rtl.j, rtr.i, rtr.j, rbl.i, rbl.j, rbr.i, rbr.j);
  
  std::vector<Point2D<float> > points;
  points.push_back(rtl);
  points.push_back(rtr);
  points.push_back(rbr);
  points.push_back(rbl);
  points.push_back(rtl);
  return points;
}

// ######################################################################
Beobot2::MotorCommand LocalMap::getDWAnavigationCommand()
{  
  Beobot2::MotorCommand cmd;
  cmd.translation = 0.0;  cmd.rotation    = 0.0; 
  Timer tim(1000000);

  // inputs:
  // trajectory current robot 
  // grid occupancy map
  // robot state

  // NOTE: all units in metric:
  //       m     for distances
  //       m/s   for trans velocity
  //       rad/s for rot   velocity
  double diff = itsDesiredHeading - itsSelfLocalHeading;

  // add a small bias to correct for lateral deviation
  uint path_size = itsSmoothedPath.size(); 
  float grid_w  = float(LOCAL_MAP_GRID_WIDTH);
  float ld_x    = itsSmoothedPath[0].j - itsSmoothedPath[path_size-1].j;
  float ld_y    = -itsLateralDeviation/grid_w;
  float ld_ang  = atan2(ld_y,ld_x);
  itsLateralDeviationAngle = ld_ang;

  //float temp_ang = diff;
  diff += 2.5*ld_ang; 

  // LINFO("ldev: %fmm %fdeg + 2.5*%fdeg = %fdeg ", 
  //       itsLateralDeviation, temp_ang/M_PI*180.0F, ld_ang/M_PI*180.0F, 
  //       diff/M_PI*180.0F);

  if(diff > M_PI)       diff = diff - 2*M_PI;
  else if(diff < -M_PI) diff = diff + 2*M_PI;

  double goalHeading = diff;
  // LINFO("Goal heading %f degrees", goalHeading/M_PI*180.0F);

  // get the maximum translation and rotation velocity
  double maxV = getMaxTransVelocity();
  double maxW = getMaxRotVelocity  ();//make allowable turn very small

  double speed_gain = NORMAL_SPEED_GAIN; 
  if(itsDeadEndIndicator == NEAR_DEAD_END) 
    speed_gain = NEAR_DEAD_END_SPEED_GAIN;       
  else if(itsDeadEndIndicator == DEAD_END) 
    speed_gain = DEAD_END_SPEED_GAIN;    

  double desiredV = maxV*speed_gain; // 80% of max speed
  //LINFO("maxV: %f maxW: %f", maxV, maxW/M_PI*180.0);

  // current trans and rot
  double currentV = itsCurrentTransVel;
  double currentW = itsCurrentRotVel;

  // FIXXX: figure out the time interval
  //double ld =  itsTimer.get()/1000000.0;
  double dt = 0.8; //itsTimer.get()/1000000.0;
  //LINFO("dt: %f (%f), %f %f", dt, ld, itsTransCap, itsRotCap);

  // compute the max speed given current speed and current max acceleration
  double dV = getMaxTransAcceleration()* dt;
  double dW = getMaxRotAcceleration()  * dt;

  double stepV = 0.10; // search from 0, 0.05, 0.1, 0.15....to allowableV
  double stepW = 0.10; // search from 0, 0.05, 0.1, 0.15....to allowableW


  double accel = getMaxTransAcceleration();
  Point3D<double> currentPos(0.0,0.0,0.0);

  itsDWcenters.clear();
  itsPredictedNextLocations.clear();

  // Search forward steps
  for(int step = 0; step < 1; step++)
    {
      Point3D<double> optimalVW = 
        getOptimalVW(currentPos,currentV,currentW, dV,dW,
                     maxV,maxW, stepV,stepW,
                     accel, goalHeading, desiredV,dt);

      double optimal_v     = optimalVW.x;
      double optimal_w     = optimalVW.y;
      // double optimal_theta = optimalVW.z;

      // get optimal next location
      Point3D<double> optNextPos = 
        getNextPos(optimal_v, optimal_w, dt, currentPos);
		
      // if robot have turning velocity, set center offset from current pos. 
      // Otherwise, set center as current pos
      Point2D<double> center = Point2D<double>(currentPos.x,currentPos.y);
      if(optimal_w) center += Point2D<double>(0.0, optimal_v/optimal_w);
      //LINFO("Angle to obstacle %f",optimal_theta/M_PI*180.0);

      itsDWcenters.push_back(center);
      itsPredictedNextLocations.push_back(optNextPos);

      if(step == 0)
        {
          // compute the resulting motor command
          // check both cap to prevent divide by zero
          if(itsTransCap > 0.0)
            cmd.translation = (optimal_v + 0.02964)/2.8/itsTransCap; 
          double rw       = BEOBOT2_ROBOT_WHEEL_DIST/1000.0; //0.51m
          if(itsRotCap > 0.0)
            cmd.rotation    = (optimal_w*rw/2.0 + 0.02964)/2.8/itsRotCap;
          
          itsCurrentTransVelCommand = optimal_v;
          itsCurrentRotVelCommand   = optimal_w;

          // LINFO("Time: %f Optimal vw is %f %f|cmd %f %f| next pos: %f %f | radius: %f | cost %f",
          //       tim.get()/1000.0F, optimal_v,optimal_w,cmd.translation, cmd.rotation,
          //       optNextPos.x, optNextPos.y,
          //       itsDWcenters[0].j, optimal_cost);

          //Raster::waitForKey();
        }
      currentPos = optNextPos;
      currentV = optimal_v;
      currentW = optimal_w;
    }

  return cmd;
}

// ######################################################################
Point3D<double> LocalMap::getOptimalVW
(
 Point3D<double> currentPos,
 double currentV,double currentW,
 double dV,double dW,
 double maxV,double maxW,
 double stepV,double stepW,
 double accel,
 double goalHeading,
 double desiredV,
 double dt)
{
  // find optimal cost 
  double optimal_cost = 0.0;
  double optimal_v    = 0.0;
  double optimal_w    = 0.0;
  double optimal_theta= 0.0;

  double allowableV = currentV + dV; // v = v0 + at
  double allowableW = currentW + dW; // w = w0 + at
  if(allowableV > maxV) allowableV = maxV;
  if(allowableW > maxW) allowableW = maxW;

  // the slowest speed we can decelerate in next frame
  double allowableV2 = currentV - dV; // v = v0 - at
  double allowableW2 = currentW - dW; // w = w0 - at
  if(allowableV2 < stepV*3.0) allowableV2 = stepV*3.0;
  if(allowableW2 < -maxW) allowableW2 = -maxW;

  // maximum distance of obstacle considered
  double dist_max  = MAXIMUM_OBSTACLE_DISTANCE;

   // LINFO("currentV %f plus/minus %f -->  %4.2f:%4.2f m/s ,maxV %f "
   //       "currentR %f plus/minus %f --> :%4.2f:%4.2f deg/s, maxW %f",
   //       currentV, dV, allowableV2, allowableV, maxV,
   //       currentW/M_PI*180.0, dW/M_PI*180.0, 
   //       allowableW2/M_PI*180.0, allowableW/M_PI*180.0,maxW/M_PI*180.0);  
  //LINFO("timeBL: %f", tim.get()/1000.0F);

  //double optimal_heading = -1.0;
  for(double v = allowableV2; v < allowableV ; v+= stepV)
    {
      for(double w = allowableW2; w < allowableW ; w+= stepW)
        {
          // find distance to the closest obstacle on the trajectory
          Beobot2::distAngle da  = getNearestObstacle(v,w, currentPos);
          double distToObstacle  = da.dist;
          double angleToObstacle = da.angle;

          // compute break distance given max deacceleration		
          double breakDist = v*v/(2*accel);	

          // LINFO("[v: %6.2f w: %6.2f] distToObstacle : %7.3f breakDist: %7.3f angleToObstacle:%7.3f", 
          //       v,w, distToObstacle, breakDist, angleToObstacle/M_PI*180.0);

          //LINFO("v %f w %f",v,w);
          // find next pos x,y,theta given v,w
          Point3D<double> nextPos = getNextPos(v,w,dt,currentPos);

          if(distToObstacle > breakDist )
            {
              // fitness of path depends on:
              //   difference in heading to goal
              //   clearance to closest obstacle
              //   desired velocity
              double headDiff  = 
                fabs(goalHeading - nextPos.z);
								
              double clearance = 
                (distToObstacle - breakDist) / (dist_max - breakDist);
              double desiredVdiff = fabs(desiredV - v);

              double headDiffCost     = headDiff/M_PI;
              double clearanceCost    = 1.0 - clearance;
              if(clearanceCost < 0.0) clearanceCost = 0.0;
              double desiredVdiffCost = desiredVdiff /maxV;
              
               // LINFO("gh:(%f - %f) = %f cl: (%f - %f)/(%f - %f) = %f "
               //       "vel: fabs(%f - %f) = %f",
               //       goalHeading,nextPos.z,headDiff,
               //       distToObstacle, breakDist, dist_max, breakDist,
               //       (distToObstacle - breakDist) / (dist_max - breakDist),
               //       desiredV, v, desiredVdiff);

              double cost = 
                (0.97*headDiffCost)  + 
                (0.02*clearanceCost) + 
                (0.01*desiredVdiffCost);

              //printf("vw(%3.2f,%3.2f)Cost %7.3f = "
              //       "hd 0.8*%7.3f + cl 0.1*%7.3f + dv 0.1*%7.3f "
              //       "gh:(%f - %f) = %f cl: (%f - %f)/(%f - %f) = %f "
              //       "vel: fabs(%f - %f) = %f \n", 
              //       v,w, cost, headDiffCost, clearanceCost, desiredVdiffCost,
              //       goalHeading,nextPos.z,headDiff,
              //       distToObstacle, breakDist, dist_max, breakDist,
              //       (distToObstacle - breakDist) / (dist_max - breakDist),
              //       desiredV, v, desiredVdiff);

              //if(cost < 0.05)
              //  LINFO("vw(%4.2f,%5.2f)Cost %f = hd %f +cl %f + dv %f", 
              //        v,w, cost, headDiffCost,clearanceCost,desiredVdiffCost);
              if(cost < optimal_cost || optimal_cost == 0.0) 
                {
                  optimal_cost = cost;
                  optimal_v = v;
                  optimal_w = w;         
                  optimal_theta = angleToObstacle;
                  //countOpt = count-1;


                  //LINFO("clearance: (%f - %f)/(%f - %f) = %f  vel: fabs(%f - %f) = %f",
                  //		distToObstacle, breakDist, dist_max, breakDist,
                  //		(distToObstacle - breakDist) / (dist_max - breakDist),
                  //		desiredV, v, desiredVdiff);
                  // LINFO("<<<<<<[%4d]: v: %f w: %f", count++, v,w/M_PI*180.0);                  
                  // LINFO("<<<<<<[%f, %f]      next pos: (%f %f); heading: %f vs. %f(from modA*)", 
                  //       v,w/M_PI*180.0, nextPos.x, nextPos.y, nextPos.z/M_PI*180.0F, 
                  //       goalHeading/M_PI*180.0F);


                  // LINFO("[%8.2f, %8.2f]      next pos: (%f %f); heading: %f vs. %f(from modA*):"
                  //       "%f ==> %f = 0.8*%f + 0.1*%f + 0.1*%f", 
                  //       v,w/M_PI*180.0, nextPos.x, nextPos.y, nextPos.z/M_PI*180.0F, 
                  //       goalHeading/M_PI*180.0F, headDiff/M_PI*180.0F,
                  //       cost, headDiffCost, clearanceCost, desiredVdiffCost);


                }


              // if(headDiff < optimal_heading || optimal_heading == -1.0) 
              //   {
              //     optimal_heading = headDiff;

              //     //LINFO("\n<<<<<%4d>>>>: v: %f w: %f", count++, v,w/M_PI*180.0);
              //     // LINFO("<<      next pos: (%f %f); heading: %f vs. %f(from modA*): headDiff: %f", 
              //     //       nextPos.x, nextPos.y, nextPos.z/M_PI*180.0F, 
              //     //       goalHeading/M_PI*180.0F, headDiff/M_PI*180.0F);
              //     // LINFO("<<      %f = 0.8*%f + 0.1*%f + 0.1*%f\n", 
              //     //       cost, headDiff, clearance, desiredVdiff);
              //   }




            }
          // else there would not be time to avoid the obstacle
          //else LINFO("vw(%f,%f) OUTTTTTTTTTTTTTTTTTTTT: obs dist > break dist: %f > %f",v,w, distToObstacle, breakDist);

        }		
    }

  //LINFO("timeAL: %f", tim.get()/1000.0F);


  //getNearestObstacle(optimal_v,optimal_w,currentPos,true);
  //LINFO("\n\nopt Cost %5.4f V %5.2f, W %5.2f, Theta %5.2f",optimal_cost,optimal_v,optimal_w,optimal_theta);
  //if(optimal_w < -0.1)
  //Raster::waitForKey();
  return Point3D<double>(optimal_v,optimal_w,optimal_theta);	
}

// ######################################################################
// given motor power from -1.0 to 1.0 
// return velocity in m/s
double LocalMap::getVelocity(double motorPower)
{
  return motorPower * 2.8 - 0.02964;
}

// ######################################################################
// given motor power from -1.0 to 1.0 
// return acceleration in m/s^2
double LocalMap::getAcceleration(double motorPower)
{
  return motorPower * 8.9197 + 2.375;
}

// ######################################################################
// The max speed is limited by RC capped, so that's the max speed robot can go
double LocalMap::getMaxTransVelocity()
{
  return getVelocity(itsTransCap);
}
// ######################################################################
//in rad/sec
double LocalMap::getMaxRotVelocity()
{
  double w = BEOBOT2_ROBOT_WHEEL_DIST/1000.0;//0.51m
  return getVelocity(itsRotCap)*4/w;
	
}
// ######################################################################
double LocalMap::getMaxTransAcceleration()
{
  return getAcceleration(itsTransCap);
}
// ######################################################################
double LocalMap::getMaxRotAcceleration()
{
  double w = BEOBOT2_ROBOT_WHEEL_DIST/1000.0;//0.51m
  return getAcceleration(itsRotCap)*4/w;
}

// ######################################################################
Point3D<double> LocalMap::getNextPos(double v, double w, double dt,Point3D<double> currentPos)
{
  // if robot motion is direction straight
  if(w == 0) return Point3D<double>(v*dt, 0, 0.0);
  
  double radius = v/w;
  double theta  = w*dt;   
  double x      = radius*sin(theta);
  double y      = radius - radius*cos(theta);

  return Point3D<double>(x,y,theta);
}

// ######################################################################
Beobot2::distAngle LocalMap::getNearestObstacle
(double v, double w, Point3D<double> currentPos,bool debug)
{
  if(itsLRFmetricSelfCoordinates.size() == 0) return Beobot2::distAngle(100.0,0.0);
  if(w == 0.0) return Beobot2::distAngle(getNearestObstacleStraight(v),0.0);

  // closest arc distance
  double min_a_distance = MAXIMUM_OBSTACLE_DISTANCE;
  double min_theta = M_PI*2.0;

  // find distance to the closest obstacle on the trajectory

  double radius = v/w;
  Point2D<double> center = Point2D<double>(currentPos.x - radius, currentPos.y);
  
  // half robot distance in <meter>
  double robot_width      = BEOBOT2_ROBOT_WIDTH/1000.0; 
  double half_robot_width = robot_width/2.0; 

  bool target_angle = true;//(w < 0.0);//(v == .4 && w > 0 && w < .1);
  if(debug && target_angle)
    LINFO("v:%f w:%4.2f radius:%4.2f center(%4.2f,%4.2f) rw: %3.1f Range:%3.1f~%3.1f LRF size:%3d",
          v,w,radius,center.i,center.j,half_robot_width, 
          abs(radius) - half_robot_width, abs(radius) + half_robot_width,
          (int)itsLRFmetricSelfCoordinates.size());
	
  // for each laser
  //LINFO("lrf size %d",(int)itsLRFmetricSelfCoordinates.size());
  //LINFO("center (%f,%f) vw(%f,%f) radius %f",center.i,center.j,v,w,radius);
  for(uint i = 0; i < itsLRFmetricSelfCoordinates.size(); i++)
    {
      double lx = itsLRFmetricSelfCoordinates[i].i;
      double ly = itsLRFmetricSelfCoordinates[i].j; 

      // compute distance from center to this LRF point
      double dx = lx - center.i;
      double dy = ly - center.j;
      double c_dist = sqrt(dx*dx+dy*dy); 

      //if(v>  2.19 && v < 2.21 && ((w < -0.11 && w > -0.13) ||(w < -0.01 && w> -0.021)))

      // if(debug && target_angle)
      //   {
      //     LINFO("laser[%3d]: %f %f", i, lx, ly);
      //     LINFO("dist to center to LRF[%d] dx: %f dy: %f c_dist: %f vs. %f (+/- %f)",
      //           (int)i, dx, dy, c_dist, fabs(radius), half_robot_width);
      //   }

      // check if the robot hit the LRF point
      double slack = .05;
      if(c_dist < (fabs(radius) + half_robot_width - slack) && 
         c_dist > (fabs(radius) - half_robot_width + slack)    )
        { 
          // parameterized equation from center to obstacle
          double six = dx/c_dist;
          double siy = dy/c_dist;
          
          // double ix = center.i + six*radius;
          // double iy = center.j + siy*radius;
         
          double theta =  atan2(-siy, -six);
          if(w > 0.0) theta =  atan2(-siy, six);
          if(theta < 0.0) theta += 2*M_PI;

          // compute the arc
          double arc_distance = 0.0;

          //if it's straight line, arc_distance should be line
          if(six == 0)
            {
              arc_distance = c_dist;
            }else{
            if(w < 0.0) arc_distance = -radius*theta;
            else        arc_distance =  radius*theta;
          }

          // if(debug && target_angle){
          //   LINFO("laser[%3d]: %f %f", i, lx, ly);
          //   LINFO("dist to center to LRF[%d] dx: %f dy: %f c_dist: %f",(int)i, dx, dy, c_dist);
            
          //   LINFO("six: %f siy: %f theta: %f arc_distance: %f", 
          //         six, siy, theta/M_PI*180.0, arc_distance);
          // }

          // if it's less than minimum arc
          if(min_a_distance > arc_distance)
            {
              min_a_distance = arc_distance;
              min_theta = theta;

              if(debug && target_angle){
                LINFO("laser[%3d]: %f %f", i, lx, ly);
                LINFO("dist to center to LRF[%d] dx: %f dy: %f c_dist: %f",(int)i, dx, dy, c_dist);
            
                LINFO("six: %f siy: %f theta: %f arc_distance: %f", 
                      six, siy, theta/M_PI*180.0, arc_distance);
              }

              if(debug && target_angle)
                LINFO("=========>         min arc theta %f min arc length %f",min_theta/M_PI*180.0,min_a_distance);
            }
        }
      else 
        {
          //if(debug && target_angle) LINFO("NOT HITTING");
        }
    }
  if(debug && target_angle)
    LINFO("Final v%f w%f min_theta %f min_dist %f",v,w,min_theta/M_PI*180.0,min_a_distance);
  return Beobot2::distAngle (min_a_distance,min_theta);
}

// ######################################################################
double LocalMap::getNearestObstacleStraight(double v)
{
  if(itsLRFmetricSelfCoordinates.size() == 0) return 100.0;

  // closest distance
  double min_distance = 100.0;

  // find distance to the closest obstacle on the straight trajectory
  
  // half robot distance in <meter>
  double half_robot_width = BEOBOT2_ROBOT_WIDTH/2000.0; 

  // for each laser
  for(uint i = 0; i < itsLRFmetricSelfCoordinates.size(); i++)
    {
      double lx = itsLRFmetricSelfCoordinates[i].i;
      double ly = itsLRFmetricSelfCoordinates[i].j; 

      // check if the robot hit the LRF point
      if(ly < half_robot_width && ly > -half_robot_width)
        { 
          // compute the distance
          double distance = lx;

          // if it's less than minimum distance
          if((lx >= 0.0 && v >= 0.0) || (lx < 0.0 && v < 0.0)) 
            if(min_distance > distance)
              {
                min_distance = distance;
              }
        }
    }

  //  LINFO("here s: %f", min_distance);

  return min_distance;
}  

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
