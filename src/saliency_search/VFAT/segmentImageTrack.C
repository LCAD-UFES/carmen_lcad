/*!@file VFAT/segmentImageTrack.C Basic image segmenter blob finder using color */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageTrack.C $
// $Id: segmentImageTrack.C 4663 2005-06-23 17:47:28Z rjpeters $
//

#include "VFAT/segmentImageTrack.H"

#define LOTBOUND 150
#define BOUND 10
#define LEVBOUND 140
#define MINBOUND 5
#define TRAJ 4
#define SAMPLESTART 30
#define MAXTRAJ 50
#define MAXSIZE 100
#define MAXMASS 25000
#define MINMASS 25
#define RATIOMIN 0.3F
#define LEVRATIOMIN -0.3F
#define RATIOMAX 1.7F
#define LEVRATIOMAX 3.3F

void segmentImageTrack::applyHardConst()
{
  if(LOT == true)
  {
    xBound = LOTBOUND;
    yBound = LOTBOUND;
  }
  else
  {
    xBound = (int)(BOUND + (levity*LEVBOUND));
    yBound = (int)(BOUND + (levity*LEVBOUND));
  }
  for(int i = 0; i < image->numberBlobs(); i++)
  {
    softCandidateBlob[i] = true;
    candidateBlob[i] = true;
    killedByTrack[i] = false;
    // check if a blobs mass is within cosnstraints
    if((image->getMass(i) < minMass) || (image->getMass(i) > maxMass))
    {
      candidateBlob[i] = false;
      softCandidateBlob[i] = false;
      killedByTrack[i] = true;
    }

    // check that a blob is within our X frame
    if((image->getCenterX(i) < (centerX - xBound)) ||
        (image->getCenterX(i) > (centerX + xBound)))
    {
      candidateBlob[i] = false;
      killedByTrack[i] = true;
    }

    // check that a blob is within our Y frame
    if((image->getCenterY(i) < (centerY - yBound)) ||
       (image->getCenterY(i) > (centerY + yBound)))
    {
      candidateBlob[i] = false;
      killedByTrack[i] = true;
    }

    // check that blob is within size ratios
    float foo = (image->getXmax(i) - image->getXmin(i));
    if(foo != 0)
    {
      float temp = (image->getYmax(i) - image->getYmin(i))/foo;
      if((temp < (RATIOMIN + (levity*LEVRATIOMIN)))
         || (temp > (RATIOMAX + (levity*LEVRATIOMAX))))
      {
        candidateBlob[i] = false;
        killedByTrack[i] = true;
      }
    }
  }
}

// INSERT SIZE TO MASS RATIO
// INSERT DUAL TRACK

void segmentImageTrack::fluidTrackCalc(float *thisMean, float *thisStd,
                                       float *thisCounter,
                                       std::vector<float> &thisList)
{
  if((LOT == false) && (doTraj == true))
  {
    if(*thisCounter == TRAJ)
      *thisCounter = 0;
    for(int i = 0; i < TRAJ; i++)
    {
      *thisMean += thisList[i];
      *thisStd += pow(thisList[i],2)/TRAJ;
    }
    *thisMean = *thisMean/TRAJ;
    *thisStd = sqrt(*thisStd - pow(*thisMean,2));
  }
}

void segmentImageTrack::mergeBlobs()
{
  mass = 0;
  double meanX = 0;
  double meanY = 0;
  minX = 640;
  minY = 480;
  maxX = 0;
  maxY = 0;

  // calculate the center of a combined blob from the average
  // mass center of all remaining blobs

  for(int i = 0; i < image->numberBlobs(); i++)
  {
    if(candidateBlob[i] == true)
    {
      mass += image->getMass(i);
      meanX += image->getMass(i)*image->getCenterX(i);
      meanY += image->getMass(i)*image->getCenterY(i);
      if(image->getXmax(i) > maxX) maxX = image->getXmax(i);
      if(image->getYmax(i) > maxY) maxY = image->getYmax(i);
      if(image->getXmin(i) < minX) minX = image->getXmin(i);
      if(image->getYmin(i) < minY) minY = image->getYmin(i);
    }
  }

  if(mass > 0)
  {
    centerX = (int)(meanX/mass);
    centerY = (int)(meanY/mass);
    //LOT = false;
    if(((maxX-minX)*(maxY-minY)) >
       ((image->getImageSizeX()*image->getImageSizeY())/2))
      LOT = true;
    else
      LOT = false;
  }
  else
  {
    LOT = true;
  }
}

/*=============================================================*/
/*        PUBLIC methods                                       */
/*=============================================================*/

segmentImageTrack::segmentImageTrack()
{}

segmentImageTrack::segmentImageTrack(int initSize)
{
  setUpVars(initSize);
}

segmentImageTrack::segmentImageTrack(int initSize,segmentImage *seg)
{
  image = seg;
  setUpVars(initSize);
}

void segmentImageTrack::setImage(segmentImage *seg)
{
  image = seg;
}

void segmentImageTrack::setUpVars(int initSize)
{
   // after the first iteration, do this
  candidateBlob.resize(initSize,true);
  softCandidateBlob.resize(initSize,true);
  killedByTrack.resize(initSize,true);
  Tsize.resize(TRAJ,0);
  Tmass.resize(TRAJ,0);
  TMS.resize(TRAJ,0);
  TsizeStd.resize(TRAJ,0);
  TmassStd.resize(TRAJ,0);
  TMSStd.resize(TRAJ,0);
  pVergance.resize(initSize,0);
  minMass = MINMASS;
  maxMass = MAXMASS;
  xBound = BOUND;
  yBound = BOUND;
  centerX = 0;
  centerY = 0;
  iter = false;
  LOT = false;
  trajCounterM = 0;
  trajCounterS = 0;
  trajCounterMS = 0;
  trajStart = 0;
  doTraj = false;
}

segmentImageTrack::~segmentImageTrack()
{}

void segmentImageTrack::track(float _levity)
{
  levity = _levity;
  if(iter == true)
  {
    // apply hard constraints to blob
    applyHardConst();
  }
  else
  {
    iter = true;
  }
  // merge all remaining blobs into a super blob
  mergeBlobs();
}

int segmentImageTrack::getObjectX()
{
  return centerX;
}

int segmentImageTrack::getObjectY()
{
  return centerY;
}

int segmentImageTrack::getXmin()
{
  return minX;
}

int segmentImageTrack::getXmax()
{
  return maxX;
}

int segmentImageTrack::getYmin()
{
  return minY;
}

int segmentImageTrack::getYmax()
{
  return maxY;
}

int segmentImageTrack::getMass()
{
  return mass;
}

bool segmentImageTrack::isCandidate(int blob)
{
  return candidateBlob[blob];
}

bool segmentImageTrack::isSoftCandidate(int blob)
{
  return softCandidateBlob[blob];
}

bool segmentImageTrack::wasKilledByTrack(int blob)
{
  return killedByTrack[blob];
}

void segmentImageTrack::setCandidate(int blob, bool setThis)
{
  candidateBlob[blob] = setThis;
}

bool segmentImageTrack::returnLOT()
{
  return LOT;
}

void segmentImageTrack::reset()
{
  trajStart = 0;
  doTraj = false;

}
