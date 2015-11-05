/*!@file
 Robots2/Beobot2/Navigation/ND_Navigation/ND_Navigation_Algorithm.C
 Nearness Difference algorithm to navigate indoors using LRF  */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/ND_Navigation.C
// $ $Id: ND_Navigation.C 12962 2010-03-06 02:13:53Z irock $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/ND_Navigation/ND_Navigation_Algorithm.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#define  SecurityNearnessRad 1000.0

// ######################################################################
ND_Navigation_Algorithm::ND_Navigation_Algorithm() :
  itsDispImg(512, 512, ZEROS),
  itsTimer(1000000)
{
  itsWin.reset();

  // goal is always in front of the robot FOR NOW
  // this should be set by the Localizer at some point
  itsGoalSector = 141;

  itsPrevDirection = 141;
  itsBigJumpCount = 0;
}

// ######################################################################
ND_Navigation_Algorithm::~ND_Navigation_Algorithm()
{ }

// ######################################################################
void ND_Navigation_Algorithm::setGoalSector(uint goal)
{ 
  itsGoalSector = goal;
}

// ######################################################################
bool ND_Navigation_Algorithm::isSafe()
{ 
  return itsIsSafe;
}

// ######################################################################
Beobot2::MotorCommand ND_Navigation_Algorithm::computeNavigationCommand
(std::vector<double> distances, std::vector<double> angles)
{
  itsDistances = distances;
  itsAngles    = angles;

  // find gaps and regions in the input Laser Range Finder
  findGapsAndRegions();

  // find all movement information
  findGoalRegion();

  biasForSafety();

  // can't go too far from previous direction
  LDEBUG("Jump detected: %d %d", itsPrevDirection, itsNextDirection);

  if((abs(int(itsPrevDirection) - int(itsNextDirection)) > 90)
     && itsBigJumpCount < 2)
    {
      LDEBUG("BIG JUMP\n\n\n");
      itsNextDirection = itsPrevDirection;
      itsBigJumpCount++;
    }
  else itsBigJumpCount = 0;

  if(abs(int(itsPrevDirection) - int(itsNextDirection)) > 15)
    {
      LDEBUG("JUMP\n\n\n");
      if(itsPrevDirection > itsNextDirection)
        itsNextDirection = itsPrevDirection - 15;
      else itsNextDirection = itsPrevDirection + 15;
    }
  LDEBUG("Jump detected: %d %d", itsPrevDirection, itsNextDirection);

  // formulate the motor commands
  float frontIndex = 141.0; float maxAngle = 141.0;
  float rot = (itsNextDirection - frontIndex)/maxAngle * 2.0;
  if(rot >  1.0) rot =  1.0;
  if(rot < -1.0) rot = -1.0;
  if(itsNextDirection < frontIndex)
    {
      LDEBUG("Go RIGHT: %f", rot);
    }
  else
    {
      LDEBUG("Go LEFT : %f", rot);
    }

  // draw the situation
  drawSituation();

  //Raster::waitForKey();

  // command to be executed by the BeoPilot
  Beobot2::MotorCommand cmd;

  cmd.rotation = rot;

  // safety precaution
  float dist = itsDistances[120];
  for(uint i = 101; i < 180; i++)
    {
      if(itsDistances[i] > 0.0F && (dist < 0.0F || dist > itsDistances[i])) 

        dist = itsDistances[i];
      //LINFO("dist[%d]: %f --> %f", i, itsDistances[i], dist);
    }
  LDEBUG("DIST: %f", dist);
  //Raster::waitForKey();

  if(dist < 400.0)
    cmd.translation = 0.0;
  else
    {
      float trans = 1.0 - 1.5 * abs(rot);
      if(trans < 0.4) trans = 0.4;
      //trans += 0.1;

      float trans2 = 0.9;
      if(trans2 < 1500.0) trans2 = (dist-400.0)/1000.0*.5 + .4;

      if(trans2 < trans) trans = trans2;

      if(trans > 1.0) trans = 1.0;
      //if(trans < -1.0) trans = -1.0;

      cmd.translation  = trans;
    }

  std::string ntext(sformat("T: %10.6f R: %10.6f",
                            cmd.translation, cmd.rotation));
  writeText(itsDispImg, Point2D<int>(0,400), ntext.c_str());

  uint w = itsDispImg.getWidth();
  uint h = itsDispImg.getHeight();
  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(w,h), w+10, 0, "Contours"));
  else itsWin->setDims(Dims(w,h));
  itsWin->drawImage(itsDispImg,0,0);

  itsPrevDirection = itsNextDirection;
  return cmd;
}

// ######################################################################
void ND_Navigation_Algorithm::findGapsAndRegions()
{
  itsGaps.clear();
  itsRegions.clear();

  uint nSectors = itsDistances.size();
  uint prevEnd = 0;

  // threshold for discontinuity
  double discThres = 1000.0;

  // find discontinuities (gaps)
  for(uint i = 1; i < nSectors; i++)
  {
    // ignore invalid -1.0 distances
    if(itsDistances[i-1] == -1.0 || itsDistances[i] == -1.0)
      continue;

    // ignore regions occupied by the killswitches
    if(i >=  25 && i <=  36) continue;
    if(i >= 243 && i <= 253) continue;

    if(abs(itsDistances[i-1] - itsDistances[i]) > discThres)
      {
        itsGaps.push_back(i);
        RegionInformation r;
        r.start = prevEnd;
        r.end   = i-1;
        itsRegions.push_back(r);
        prevEnd = i;
      }
  }
  RegionInformation r;
  r.start = prevEnd;
  r.end   = nSectors-1;
  itsRegions.push_back(r);

  // get the minimum distances
  setMinimumDistances();
}

// ######################################################################
void ND_Navigation_Algorithm::drawSituation()
{
  // draw regions
  float max = itsDistances[0]; float min = itsDistances[0];
  for(uint i = 1; i < itsDistances.size(); i++)
    {
      if(min > itsDistances[i]) min = itsDistances[i];
      if(max < itsDistances[i]) max = itsDistances[i];
    }
  if (min == max) max += 1.0;
  itsMinDistance = min;
  itsMaxDistance = max;

  itsDispImg.clear(PixRGB<byte>(0, 0, 0));
  for(uint i = 0; i < itsRegions.size(); i++)
    {
      PixRGB<byte> c((byte)rand()/2, (byte)rand()/2, (byte)rand()/2);

      for(uint j = itsRegions[i].start; j <= itsRegions[i].end; j++)
        {
          float dist = itsDistances[j];
          int angle  = itsAngles[j];

          // note drawing takes up a lot of time
          float rad = dist; rad = ((rad - min)/(max-min))*256;
          if (rad < 0) rad = 1.0;

          Point2D<int> pt;
          pt.i = 256 - int(rad*sin((double)angle*M_PI/180.0));
          pt.j = 256 - int(rad*cos((double)angle*M_PI/180.0));

          drawCircle(itsDispImg, pt, 2, PixRGB<byte>(255,0,0));
          drawLine(itsDispImg, Point2D<int>(256,256), pt, c);

          LDEBUG("[%4d][%4d] <%4d>: %10.1f mm", i, j, angle, dist);
        }
    }

  for(uint i = 0; i < itsRegions.size(); i++)
    {
      float dist = itsDistances[itsRegions[i].minIndex];
      int angle  = itsAngles[itsRegions[i].minIndex];
      //drawRangeLine(dist, angle, PixRGB<byte>(255,0,0));

      PixRGB<byte> c((byte)rand()/2, (byte)rand()/2, (byte)rand()/2);
      dist  = itsDistances[itsRegions[i].start];
      angle = itsAngles[itsRegions[i].start];
      //drawRangeLine(dist, angle, c);

      dist  = itsDistances[itsRegions[i].end];
      angle = itsAngles[itsRegions[i].end];
      //drawRangeLine(dist, angle, c);

      uint midInd = (itsRegions[i].start+ itsRegions[i].end) /2;
      dist  = itsDistances[midInd] + 1000.0;
      angle = itsAngles[midInd];

      // note drawing takes up a lot of time
      float rad = dist; rad = ((rad - min)/(max-min))*256;
      if (rad < 0) rad = 1.0;
      Point2D<int> pt;
      pt.i = 256 - int(rad*sin((double)angle*M_PI/180.0));
      pt.j = 256 - int(rad*cos((double)angle*M_PI/180.0));

      std::string ntext(sformat("%d", i));
      writeText(itsDispImg, pt, ntext.c_str());
    }

  // draw the next direction and navigable region boundaries
  int angle  = itsAngles[itsNextDirection];
  drawRangeLine(itsMaxDistance, angle, PixRGB<byte>(0,0,255));
  angle  = itsAngles[itsRegions[itsGoalRegion].start];
  drawRangeLine(itsMaxDistance, angle, PixRGB<byte>(255, 255, 0));
  angle  = itsAngles[itsRegions[itsGoalRegion].end];
  drawRangeLine(itsMaxDistance, angle, PixRGB<byte>(255, 255, 0));

  // draw the security zone
  float rad = ((SecurityNearnessRad - itsMinDistance)/
               (itsMaxDistance - itsMinDistance))*256;
  drawCircle(itsDispImg, Point2D<int>(256, 256) , rad,
             PixRGB<byte>(255,255,0));
  //LINFO("radius: %f", rad);

  // draw the hirzontal white line denoting -90 to 90 range
  drawLine(itsDispImg, Point2D<int>(256,0), Point2D<int>(256,512),
           PixRGB<byte>(255,255,255));
  drawLine(itsDispImg, Point2D<int>(0,256), Point2D<int>(512,256),
           PixRGB<byte>(255,255,255));

  drawRangeLine(itsMaxDistance, itsAngles[itsLeftEnd], PixRGB<byte>(255,0,0));
  drawRangeLine(itsMaxDistance, itsAngles[itsRightEnd], PixRGB<byte>(255,0,0));
}

// ######################################################################
void ND_Navigation_Algorithm::drawRangeLine
(float dist, int angle, PixRGB<byte> c)
{
  float min = itsMinDistance;
  float max = itsMaxDistance;
  // note drawing takes up a lot of time
  float rad = ((dist - min)/(max-min))*256;
  if (rad < 0) rad = 1.0;

  Point2D<int> pt;
  pt.i = 256 - int(rad*sin((double)angle*M_PI/180.0));
  pt.j = 256 - int(rad*cos((double)angle*M_PI/180.0));

  drawLine(itsDispImg, Point2D<int>(256,256), pt, c);
}

// ######################################################################
void ND_Navigation_Algorithm::setMinimumDistances()
{
  for(uint i = 0; i < itsRegions.size(); i++)
    {
      float minDist = itsDistances[itsRegions[i].start];
      uint  minIndex = itsRegions[i].start;
      for(uint j = itsRegions[i].start +1; j <= itsRegions[i].end; j++)
        {
          // ignore invalid -1.0 distances
          if(itsDistances[j] == -1.0) continue;

          // ignore regions occupied by the killswitches
          if(j >=  25 && j <=  36) continue;
          if(j >= 243 && j <= 253) continue;


          if(minDist > itsDistances[j])
            {
              minDist  = itsDistances[j];
              minIndex = j;
            }
        }

      itsRegions[i].minDistance = minDist;
      itsRegions[i].minIndex    = minIndex;
      LDEBUG("[%4d]{%4d, %4d}  minDist: %f",
             itsRegions[i].minIndex,
             itsRegions[i].start, itsRegions[i].end,
             itsRegions[i].minDistance);
    }
}

// ######################################################################
void ND_Navigation_Algorithm::findGoalRegion()
{
  // find the Free Walking Area:
  //   the closest region to the goal location

  // check the goal region first
  uint goalRegion = 0;
  for(uint i = 0; i < itsRegions.size(); i++)
    {
      LDEBUG("[%d][%d %d]",
            i, itsRegions[i].start,itsRegions[i].end);
      if(itsRegions[i].start <= itsGoalSector &&
         itsRegions[i].end   >= itsGoalSector)
        {
          goalRegion = i;
          LDEBUG("goal: %d", i);
        }
    }

  LDEBUG("initial goal region: %d", goalRegion);
  if(isNavigableRegion(goalRegion))
    LDEBUG("just go straight: %d", goalRegion);
  else
    {
      // if we can't go straight: it's not safe
      itsIsSafe = false;

      int cRegion = goalRegion + 1;
      while(cRegion < int(itsRegions.size()) &&
            !isNavigableRegion(cRegion))
        {
          cRegion++;
        }

      // trouble no space in left
      if(cRegion == int(itsRegions.size()))
        {
          cRegion = goalRegion - 1;
          while(cRegion >= 0  &&
                !isNavigableRegion(cRegion))
            {
              cRegion--;
            }

          if(cRegion == -1)
            {
              // use goal region until something good happen
              LDEBUG("get lucky");
            }
          else
            {
              goalRegion = cRegion;
              LDEBUG("to the right: %d", goalRegion);
            }
        }
      else
        {
          goalRegion = cRegion;
          LDEBUG("to the left: %d", goalRegion);
        }
    }

  itsGoalRegion = goalRegion;
  uint start = itsRegions[goalRegion].start;
  uint end   = itsRegions[goalRegion].end;

  float total = 0.0;
  for(uint i = start; i <= end; i++)
    total += itsDistances[i];
  total /= 2.0;

  uint index = start;
  while(index <= end && total > 0.0)
    {
      total -= itsDistances[index];
      index++;
    }
  itsNextDirection = index;


  //itsNextDirection =
  //  (itsRegions[goalRegion].start + itsRegions[goalRegion].end)/2;

  LDEBUG("Next dir: %d ", itsNextDirection);
}

// ######################################################################
void ND_Navigation_Algorithm::biasForSafety()
{
  uint biasRange = 60;
  // check for danger to the Right of itsNextDirection
  if(itsNextDirection > biasRange)
    itsRightEnd = itsNextDirection - biasRange;
  else itsRightEnd = 0;

  // check for danger to the Left of itsNextDirection
  if(itsNextDirection < itsAngles.size() - biasRange)
    itsLeftEnd = itsNextDirection + biasRange;
  else itsLeftEnd = itsAngles.size() - 1;

  // cut off everythings behind the robot
  // only consider front 180 degrees
  if(itsRightEnd < 50) itsRightEnd =  50;
  if(itsLeftEnd > 230) itsLeftEnd  = 230;

  if(itsNextDirection <  50) itsNextDirection =  50;
  if(itsNextDirection > 230) itsNextDirection = 230;

  if(itsRightEnd > itsLeftEnd) return;

  float rightDist = SecurityNearnessRad;
  int rightIndex = -1;
  for(uint i = itsRightEnd; i < itsNextDirection; i++)
    {
      // ignore invalid -1.0 distances
      if(itsDistances[i] == -1.0) continue;

      // ignore regions occupied by the killswitches
      if(i >=  25 && i <=  36) continue;
      if(i >= 243 && i <= 253) continue;

      if(itsDistances[i] < SecurityNearnessRad)
        {
          rightDist  = itsDistances[i];
          rightIndex = i;
        }
    }

  float leftDist = SecurityNearnessRad;
  int leftIndex = -1;
  for(uint i = itsNextDirection; i < itsLeftEnd; i++)
    {
      // ignore invalid -1.0 distances
      if(itsDistances[i] == -1.0) continue;

      // ignore regions occupied by the killswitches
      if(i >=  25 && i <=  36) continue;
      if(i >= 243 && i <= 253) continue;

      if(itsDistances[i] < SecurityNearnessRad)
        {
          leftDist  = itsDistances[i];
          leftIndex = i;
        }
    }

  LDEBUG("safety [%f %f]",
        itsAngles[itsRightEnd], itsAngles[itsLeftEnd]);

  // no obstacle
  if(rightIndex == -1 && leftIndex == -1)
    {
      LDEBUG("good to go");
      itsIsSafe = true;
      return;
    }

  // there is obstacle nearby that we need to avoid
  itsIsSafe = false;

  // right obstacle, bias left
  if(rightIndex != -1 && leftIndex == -1)
    {
      itsNextDirection = itsNextDirection + 30;
      if(itsNextDirection >= itsAngles.size())
        itsNextDirection = itsAngles.size() - 1;

      LDEBUG("right obstacle: [%d] %f: -> %d",
            rightIndex, rightDist, itsNextDirection);
      return;
    }

  // left obstacle, bias right
  if(rightIndex == -1 && leftIndex != -1)
    {
      if(itsNextDirection < 30)
        itsNextDirection = 0;
      else
        itsNextDirection -= 30;

      LDEBUG("left obstacle: [%d] %f -> %d",
            leftIndex, leftDist, itsNextDirection);
      return;
    }

  // obstacles on both
  if(rightIndex != -1 && leftIndex != -1)
    {
      int bias = int((leftDist - rightDist)/SecurityNearnessRad * 10.0);
      if (bias > 10.0)  bias = 10.0;
      if (bias < -10.0) bias = -10.0;

      int temp = itsNextDirection + bias;
      if(temp < 0) temp = 0;
      if(temp >= int(itsAngles.size())) temp = itsAngles.size() - 1;

      LDEBUG("hopefully nothing bad happens: %d -> %d", itsNextDirection, temp);
      itsNextDirection = temp;

      return;
    }
  return;
}

// ######################################################################
bool ND_Navigation_Algorithm::isNavigableRegion(uint index)
{
  bool extreme = false;
  uint size = itsRegions[index].end - itsRegions[index].start;
  uint count = 0;
  for(uint i = itsRegions[index].start; i <= itsRegions[index].end; i++)
    {
      if(i > 230 || i < 50) count++;
    }
  if(count > size/2) extreme = true;

  uint nSector  = itsRegions[index].end - itsRegions[index].start;
  float minDist = itsRegions[index].minDistance;

  bool isNav = (((nSector >= 6 && minDist > 1250.0) ||
                ((nSector >= 4 && nSector <= 5) && minDist >= 2000.0))
                && !extreme);

  if(!isNav) LDEBUG("not Nav: %d %d   %f extreme: %d", index,
                   nSector, minDist, extreme);

  return isNav;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
