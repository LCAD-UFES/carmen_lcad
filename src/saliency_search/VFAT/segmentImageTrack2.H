/*!@file VFAT/segmentImageTrack2.H .H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageTrack2.H $
// $Id: segmentImageTrack2.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#include "Image/Image.H"
#include "Util/log.H"
#include "VFAT/blobProp.H"
#include "VFAT/segmentImage2.H"
#include <cstdio>
#include <cstdlib>

class segmentImageTrack2
{
private:
  int SIT_minX,SIT_maxX,SIT_minY,SIT_maxY,SIT_centerX,SIT_centerY,SIT_mass;
  //int SIT_minMass, SIT_maxMass;
  int SIT_xBound, SIT_yBound;
  int SIT_trajCounterM,SIT_trajCounterS,SIT_trajCounterR,SIT_trajCounterMS;
  bool SIT_iter,SIT_LOT,SIT_doTraj,SIT_trajStart;
  bool SIT_blobSet;
  float SIT_levity;
  float SIT_meanTsize;
  float SIT_meanTmass;
  float SIT_meanTratio;
  float SIT_meanTMS;
  float SIT_meanTsizeStd;
  float SIT_meanTmassStd;
  float SIT_meanTratioStd;
  float SIT_meanTMSStd;
  segmentImage2 *SIT_image;
  blobProp *SIT_blobProp;
  //! stores blobs that are candidates after filtering
  std::vector<bool> SIT_candidateBlob;
  std::vector<bool> SIT_softCandidateBlob;
  //! this tells if candidicy was removed be me and not someone else
  std::vector<bool> SIT_killedByTrack;
  //! holds size info
  std::vector<float> SIT_Tsize;
  std::vector<float> SIT_TsizeStd;
  //! holds mass info
  std::vector<float> SIT_Tmass;
  std::vector<float> SIT_TmassStd;
  //! holds Ratio info
  std::vector<float> SIT_Tratio;
  std::vector<float> SIT_TratioStd;
  //! holds size verses mass info
  std::vector<float> SIT_TMS;
  std::vector<float> SIT_TMSStd;
  //! holds votes
  std::vector<int> SIT_Twinner;
  std::vector<float> SIT_TwinnerScore;

  //! apply hard Constraints to blobs
  void SITapplyHardConst();
  //! apply fluid Mass constraints
  void SITfluidTrackCalc(float *thisMean, float *thisStd,
                      float *thisCounter,
                      std::vector<float> &thisList);
  //! marry winning blobs
  void SITmergeBlobs();
public:
  //! P that a blob will bring this instance into vergance
  std::vector<float> SIT_pVergance;
  //! set up tracker one time, input max blobs and a linked image segmenter
  segmentImageTrack2();
  segmentImageTrack2(int initSize);
  segmentImageTrack2(int initSize,segmentImage2 *seg);
  ~segmentImageTrack2();
  //! insert image into object
  void SITsetImage(segmentImage2 *seg);
  //! insert blobProp pointer
  void SITsetBlobProp(blobProp *bp);
  //! set up variables
  void SITsetUpVars(int initSize);
  //! track this object
  /*! Call this method to apply single camera tracking constraints.
    @param _levity This is how much levity to allow in tracking constraints \
    0 = none while 1 equals total levity and almost disables this feature
  */
  void SITtrack(float _levity = 0);
  //! get centroid of object in X
  int SITgetObjectX();
  //! get centroid of object in Y
  int SITgetObjectY();
  //! get boundry of object minimum X
  int SITgetXmin();
  //! get boundry of object maximum X
  int SITgetXmax();
  //! get boundry of object minimum Y
  int SITgetYmin();
  //! get boundry of object maximum Y
  int SITgetYmax();
  //! get mass of object
  int SITgetMass();
  //! tell if this blob is still a candidate
  bool SITisCandidate(int blob);
  //! tell if this blob is a soft candidate
  bool SITisSoftCandidate(int blob);
  //! tells if candidicy was removed by this class and not externally
  bool SITwasKilledByTrack(int blob);
  //! let an external class set this blobs candidicy
  void SITsetCandidate(int blob, bool setThis);
  //! Returns if it has a loss of track
  bool SITreturnLOT();
  //! allows you to manually set the LOT bool
  void SITsetLOT(bool LOT);
  //! resets some parameters if needed
  void SITreset();
};


