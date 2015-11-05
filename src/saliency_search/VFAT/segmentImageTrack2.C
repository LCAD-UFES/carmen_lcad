/*!@file VFAT/segmentImageTrack2.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageTrack2.C $
// $Id: segmentImageTrack2.C 6003 2005-11-29 17:22:45Z rjpeters $
//

#include "VFAT/segmentImageTrack2.H"

#include "Util/Assert.H"

#include <iostream>
/*
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
*/

void segmentImageTrack2::SITapplyHardConst()
{
  ASSERT(SIT_blobSet && "Must set blob properties in SITsetBlobProp");
  if(SIT_LOT == true)
  {
    //std::cerr << "SET new LOT\n";
    SIT_xBound = SIT_blobProp->BP_LOTbound;
    SIT_yBound = SIT_blobProp->BP_LOTbound;
  }
  else
  {
    //std::cerr << "SET old LOT\n";
    SIT_xBound = (int)(SIT_blobProp->BP_bound +
                       (SIT_levity*SIT_blobProp->BP_softBound));
    SIT_yBound = (int)(SIT_blobProp->BP_bound +
                       (SIT_levity*SIT_blobProp->BP_softBound));
  }

  for(int i = 0; i < SIT_image->SInumberBlobs(); i++)
  {
    SIT_softCandidateBlob[i] = true;
    SIT_candidateBlob[i] = true;
    SIT_killedByTrack[i] = false;
    // check if a blobs mass is within cosnstraints
    if((SIT_image->SIgetMass(i) < SIT_blobProp->BP_minMass) ||
       (SIT_image->SIgetMass(i) >SIT_blobProp->BP_maxMass))
    {
      SIT_candidateBlob[i] = false;
      SIT_softCandidateBlob[i] = false;
      SIT_killedByTrack[i] = true;
      /*std::cerr << "KILLED " << i << " MASS\n";
      std::cerr << "\t" << SIT_blobProp->BP_minMass
                << " < " <<  SIT_image->SIgetMass(i)
                << " < " << SIT_blobProp->BP_maxMass
                << "\n";*/
    }

    // check that a blob is within our X frame
    if((SIT_image->SIgetCenterX(i) < (SIT_centerX - SIT_xBound)) ||
        (SIT_image->SIgetCenterX(i) > (SIT_centerX + SIT_xBound)))
    {
      SIT_candidateBlob[i] = false;
      SIT_killedByTrack[i] = true;
      /*std::cerr << "KILLED " << i << " Xframe\n";
      std::cerr << "\t" << (SIT_centerX - SIT_xBound)
                << " < " << SIT_image->SIgetCenterX(i)
                << " < " << (SIT_centerX + SIT_xBound)
                << "\n";*/

    }

    // check that a blob is within our Y frame
    if((SIT_image->SIgetCenterY(i) < (SIT_centerY - SIT_yBound)) ||
       (SIT_image->SIgetCenterY(i) > (SIT_centerY + SIT_yBound)))
    {
      SIT_candidateBlob[i] = false;
      SIT_killedByTrack[i] = true;
      /*std::cerr << "KILLED " << i << " Yframe\n";
      std::cerr << "\t" << (SIT_centerY - SIT_yBound)
                << " < " << SIT_image->SIgetCenterY(i)
                << " < " << (SIT_centerY + SIT_yBound)
                << "\n";*/
    }

    // check that blob is within size ratios

    float foo = (SIT_image->SIgetXmax(i) - SIT_image->SIgetXmin(i));
    if(foo != 0)
    {
      float temp = (SIT_image->SIgetYmax(i) - SIT_image->SIgetYmin(i))/foo;
      if((temp < (SIT_blobProp->BP_ratioMin
                  + (SIT_levity*SIT_blobProp->BP_softRatioMin)))
         || (temp > (SIT_blobProp->BP_ratioMax
                     + (SIT_levity*SIT_blobProp->BP_softRatioMin))))
      {
        SIT_candidateBlob[i] = false;
        SIT_killedByTrack[i] = true;
        /*std::cerr << "KILLED " << i << " RATIO\n";*/
      }

    }
  }
}

// INSERT SIZE TO MASS RATIO
// INSERT DUAL TRACK

void segmentImageTrack2::SITfluidTrackCalc(float *thisMean, float *thisStd,
                                       float *thisCounter,
                                       std::vector<float> &thisList)
{
  if((SIT_LOT == false) && (SIT_doTraj == true))
  {
    if(*thisCounter == SIT_blobProp->BP_traj)
      *thisCounter = 0;
    for(int i = 0; i < SIT_blobProp->BP_traj; i++)
    {
      *thisMean += thisList[i];
      *thisStd += pow(thisList[i],2)/SIT_blobProp->BP_traj;
    }
    *thisMean = *thisMean/SIT_blobProp->BP_traj;
    *thisStd = sqrt(*thisStd - pow(*thisMean,2));
  }
}

void segmentImageTrack2::SITmergeBlobs()
{
  SIT_mass = 0;
  double meanX = 0;
  double meanY = 0;
  SIT_minX = 640;
  SIT_minY = 480;
  SIT_maxX = 0;
  SIT_maxY = 0;

  // calculate the center of a combined blob from the average
  // mass center of all remaining blobs

  for(int i = 0; i < SIT_image->SInumberBlobs(); i++)
  {
    //std::cerr << "BLOB " << i << "\n";
    if(SIT_candidateBlob[i] == true)
    {
      //std::cerr << "OK\n";
      SIT_mass += SIT_image->SIgetMass(i);
      meanX += SIT_image->SIgetMass(i)*SIT_image->SIgetCenterX(i);
      meanY += SIT_image->SIgetMass(i)*SIT_image->SIgetCenterY(i);
      if(SIT_image->SIgetXmax(i) > SIT_maxX) SIT_maxX =
                                               SIT_image->SIgetXmax(i);
      if(SIT_image->SIgetYmax(i) > SIT_maxY) SIT_maxY =
                                               SIT_image->SIgetYmax(i);
      if(SIT_image->SIgetXmin(i) < SIT_minX) SIT_minX =
                                               SIT_image->SIgetXmin(i);
      if(SIT_image->SIgetYmin(i) < SIT_minY) SIT_minY =
                                               SIT_image->SIgetYmin(i);
    }
  }

  if(SIT_mass > 0)
  {
    SIT_centerX = (int)(meanX/SIT_mass);
    SIT_centerY = (int)(meanY/SIT_mass);
    //LOT = false;

    if(((SIT_maxX-SIT_minX)*(SIT_maxY-SIT_minY)) >
       ((SIT_image->SIgetImageSizeX()*SIT_image->SIgetImageSizeY())
        /SIT_blobProp->BP_maxFrameSize))
    {
      SIT_LOT = true;
      std::cerr << "LOT: " << ((SIT_maxX-SIT_minX)*(SIT_maxY-SIT_minY))
                << " > " <<
        ((SIT_image->SIgetImageSizeX()*SIT_image->SIgetImageSizeY())
         /SIT_blobProp->BP_maxFrameSize)
                << "\n";
    }
    else
    {
      SIT_LOT = false;
    }
  }
  else
  {
    SIT_LOT = true;
    //std::cerr << "LOT: mass < 0" << "\n";
  }
}

/*=============================================================*/
/*        PUBLIC methods                                       */
/*=============================================================*/

segmentImageTrack2::segmentImageTrack2()
{
  SIT_blobSet = false;
}

segmentImageTrack2::segmentImageTrack2(int initSize)
{
  SITsetUpVars(initSize);
  SIT_blobSet = false;
}

segmentImageTrack2::segmentImageTrack2(int initSize,segmentImage2 *seg)
{
  SIT_image = seg;
  SITsetUpVars(initSize);
  SIT_blobSet = false;
}

void segmentImageTrack2::SITsetImage(segmentImage2 *seg)
{
  SIT_image = seg;
}

void segmentImageTrack2::SITsetBlobProp(blobProp *bp)
{
  SIT_blobSet = true;
  SIT_blobProp = bp;
}

void segmentImageTrack2::SITsetUpVars(int initSize)
{
   // after the first iteration, do this
  SIT_candidateBlob.resize(initSize,true);
  SIT_softCandidateBlob.resize(initSize,true);
  SIT_killedByTrack.resize(initSize,true);
  SIT_Tsize.resize(SIT_blobProp->BP_traj,0);
  SIT_Tmass.resize(SIT_blobProp->BP_traj,0);
  SIT_TMS.resize(SIT_blobProp->BP_traj,0);
  SIT_TsizeStd.resize(SIT_blobProp->BP_traj,0);

  SIT_TmassStd.resize(SIT_blobProp->BP_traj,0);
  SIT_TMSStd.resize(SIT_blobProp->BP_traj,0);
  SIT_pVergance.resize(initSize,0);
  SIT_centerX = 0;
  SIT_centerY = 0;
  SIT_iter = false;
  SIT_LOT = false;
  SIT_trajCounterM = 0;
  SIT_trajCounterS = 0;
  SIT_trajCounterMS = 0;
  SIT_trajStart = 0;
  SIT_doTraj = false;
}

segmentImageTrack2::~segmentImageTrack2()
{}

void segmentImageTrack2::SITtrack(float _levity)
{
  SIT_levity = _levity;
  if(SIT_iter == true)
  {
    // apply hard constraints to blob
    SITapplyHardConst();
  }
  else
  {
    SIT_iter = true;
  }
  // merge all remaining blobs into a super blob
  SITmergeBlobs();
}

int segmentImageTrack2::SITgetObjectX()
{
  return SIT_centerX;
}

int segmentImageTrack2::SITgetObjectY()
{
  return SIT_centerY;
}

int segmentImageTrack2::SITgetXmin()
{
  return SIT_minX;
}

int segmentImageTrack2::SITgetXmax()
{
  return SIT_maxX;
}

int segmentImageTrack2::SITgetYmin()
{
  return SIT_minY;
}

int segmentImageTrack2::SITgetYmax()
{
  return SIT_maxY;
}

int segmentImageTrack2::SITgetMass()
{
  return SIT_mass;
}

bool segmentImageTrack2::SITisCandidate(int blob)
{
  return SIT_candidateBlob[blob];
}

bool segmentImageTrack2::SITisSoftCandidate(int blob)
{
  return SIT_softCandidateBlob[blob];
}

bool segmentImageTrack2::SITwasKilledByTrack(int blob)
{
  return SIT_killedByTrack[blob];
}

void segmentImageTrack2::SITsetCandidate(int blob, bool setThis)
{
  SIT_candidateBlob[blob] = setThis;
}

bool segmentImageTrack2::SITreturnLOT()
{
  return SIT_LOT;
}

void segmentImageTrack2::SITsetLOT(bool LOT)
{
  SIT_LOT = LOT;
}

void segmentImageTrack2::SITreset()
{
  SIT_trajStart = 0;
  SIT_doTraj = false;

}
