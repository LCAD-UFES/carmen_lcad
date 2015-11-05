/*!@file VFAT/segmentImageTrack.H .H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageTrack.H $
// $Id: segmentImageTrack.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#include "Image/Image.H"
#include "Util/log.H"
#include "VFAT/segmentImage.H"
#include <cstdio>
#include <cstdlib>

class segmentImageTrack
{
private:
  int minX,maxX,minY,maxY,centerX,centerY,mass;
  int minMass, maxMass;
  int xBound, yBound;
  int trajCounterM,trajCounterS,trajCounterR,trajCounterMS;
  bool iter,LOT,doTraj,trajStart;
  float levity;
  float meanTsize;
  float meanTmass;
  float meanTratio;
  float meanTMS;
  float meanTsizeStd;
  float meanTmassStd;
  float meanTratioStd;
  float meanTMSStd;
  segmentImage *image;
  //! stores blobs that are candidates after filtering
  std::vector<bool> candidateBlob;
  std::vector<bool> softCandidateBlob;
  //! this tells if candidicy was removed be me and not someone else
  std::vector<bool> killedByTrack;
  //! holds size info
  std::vector<float> Tsize;
  std::vector<float> TsizeStd;
  //! holds mass info
  std::vector<float> Tmass;
  std::vector<float> TmassStd;
  //! holds Ratio info
  std::vector<float> Tratio;
  std::vector<float> TratioStd;
  //! holds size verses mass info
  std::vector<float> TMS;
  std::vector<float> TMSStd;
  //! holds votes
  std::vector<int> Twinner;
  std::vector<float> TwinnerScore;

  //! apply hard Constraints to blobs
  void applyHardConst();
  //! apply fluid Mass constraints
  void fluidTrackCalc(float *thisMean, float *thisStd,
                      float *thisCounter,
                      std::vector<float> &thisList);
  //! marry winning blobs
  void mergeBlobs();
public:
  //! P that a blob will bring this instance into vergance
  std::vector<float> pVergance;
  //! set up tracker one time, input max blobs and a linked image segmenter
  segmentImageTrack();
  segmentImageTrack(int initSize);
  segmentImageTrack(int initSize,segmentImage *seg);
  ~segmentImageTrack();
  //! insert image into object
  void setImage(segmentImage *seg);
  //! set up variables
  void setUpVars(int initSize);
  //! track this object
  /*! Call this method to apply single camera tracking constraints.
    @param _levity This is how much levity to allow in tracking constraints \
    0 = none while 1 equals total levity and almost disables this feature
  */
  void track(float _levity = 0);
  //! get centroid of object in X
  int getObjectX();
  //! get centroid of object in Y
  int getObjectY();
  //! get boundry of object minimum X
  int getXmin();
  //! get boundry of object maximum X
  int getXmax();
  //! get boundry of object minimum Y
  int getYmin();
  //! get boundry of object maximum Y
  int getYmax();
  //! get mass of object
  int getMass();
  //! tell if this blob is still a candidate
  bool isCandidate(int blob);
  //! tell if this blob is a soft candidate
  bool isSoftCandidate(int blob);
  //! tells if candidicy was removed by this class and not externally
  bool wasKilledByTrack(int blob);
  //! let an external class set this blobs candidicy
  void setCandidate(int blob, bool setThis);
  //! Returns if it has a loss of track
  bool returnLOT();
  //! resets some parameters if needed
  void reset();
};


