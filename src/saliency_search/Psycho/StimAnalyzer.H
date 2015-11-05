/*!@file Psycho/StimAnalyzer.H make different kind of visual test stimuli
*/

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimAnalyzer.H $
// $Id: StimAnalyzer.H 6795 2006-06-29 20:45:32Z rjpeters $


#ifndef STIM_ANALYZER_H_DEFINED
#define STIM_ANALYZER_H_DEFINED

#include <string>
#include <vector>
#include "Image/Image.H"
#include "Image/Pixels.H"

using namespace std;

//! analize stats from stim maker and surprise
class StimAnalyzer
{
public:
  StimAnalyzer(const int frames, const ushort conditions);
  ~StimAnalyzer();
  //! initalize SA with number of frames and conditions
  void SAinit(const int frames, const ushort conditions);
  //! upload a frame, salmap and gound truth with condition
  void SAinputImages(const Image<double> salmap,
                const Image<PixRGB<double> > groundTruth,
                const uint frame,
                const ushort condition);
  //! called from SAinputImages, runs stats per frame
  void SAcompImages();
  //! after all frames are run, find final stats
  void SAfinalStats();
  //! dump the stats "per frame"
  void SAdumpFrameStats(string fileName, string sample,
                        bool printHeader);
  //! dunp the stats "per condition"
  void SAdumpConditionStats(string fileName, string sample,
                            bool printHeader);

  //! is this item "on" durring this frame?
  std::vector<bool>   itsTargetFrameOn;
  //! is this item "on" durring this frame?
  std::vector<bool>   itsDistFrameOn;
  //! is this item "on" durring this frame?
  std::vector<bool>   itsOtherFrameOn;

  //! Pixel count of items in frame
  std::vector<uint>   itsTargetFrameNumber;
  //! Pixel count of items in frame
  std::vector<uint>   itsDistFrameNumber;
  //! Pixel count of items in frame
  std::vector<uint>   itsOtherFrameNumber;

  //! Pixel count of items when they are "on" per condition
  std::vector<uint>   itsTargetFrameTotalONNumber;
  //! Pixel count of items when they are "on" per condition
  std::vector<uint>   itsDistFrameTotalONNumber;
  //! Pixel count of items when they are "on" per condition
  std::vector<uint>   itsOtherFrameTotalONNumber;

  //! Sum saliency value per frame over each item
  std::vector<double> itsTargetFrameSum;
  //! Sum saliency value per frame over each item
  std::vector<double> itsDistFrameSum;
  //! Sum saliency value per frame over each item
  std::vector<double> itsOtherFrameSum;

  //! Sum of Squares saliency value per frame over each item
  std::vector<double> itsTargetFrameSS;
  //! Sum of Squares saliency value per frame over each item
  std::vector<double> itsDistFrameSS;
  //! Sum of Squares saliency value per frame over each item
  std::vector<double> itsOtherFrameSS;

  //! Min value of item per frame
  std::vector<double> itsTargetFrameMin;
  //! Min value of item per frame
  std::vector<double> itsDistFrameMin;
  //! Min value of item per frame
  std::vector<double> itsOtherFrameMin;

  //! Max value of item per frame
  std::vector<double> itsTargetFrameMax;
  //! Max value of item per frame
  std::vector<double> itsDistFrameMax;
  //! Max value of item per frame
  std::vector<double> itsOtherFrameMax;

  //! Mean value of item per frame
  std::vector<double> itsTargetFrameMean;
  //! Mean value of item per frame
  std::vector<double> itsDistFrameMean;
  //! Mean value of item per frame
  std::vector<double> itsOtherFrameMean;

  //! Std value of item per frame
  std::vector<double> itsTargetFrameStd;
  //! Std value of item per frame
  std::vector<double> itsDistFrameStd;
  //! Std value of item per frame
  std::vector<double> itsOtherFrameStd;

  //! Sum saliency value over each item per condition
  std::vector<double> itsTargetFrameTotalONSum;
  //! Sum saliency value over each item per condition
  std::vector<double> itsDistFrameTotalONSum;
  //! Sum saliency value over each item per condition
  std::vector<double> itsOtherFrameTotalONSum;

  //! Sum of Squares saliency value over each item per condition
  std::vector<double> itsTargetFrameTotalONSS;
  //! Sum of Squares saliency value over each item per condition
  std::vector<double> itsDistFrameTotalONSS;
  //! Sum of Squares saliency value over each item per condition
  std::vector<double> itsOtherFrameTotalONSS;

  //! Min pixel saliency value over each item per condition
  std::vector<double> itsTargetFrameTotalONMin;
  //! Min pixel saliency value over each item per condition
  std::vector<double> itsDistFrameTotalONMin;
  //! Min pixel saliency value over each item per condition
  std::vector<double> itsOtherFrameTotalONMin;

  //! Max pixel saliency value over each item per condition
  std::vector<double> itsTargetFrameTotalONMax;
  //! Max pixel saliency value over each item per condition
  std::vector<double> itsDistFrameTotalONMax;
  //! Max pixel saliency value over each item per condition
  std::vector<double> itsOtherFrameTotalONMax;

  //! Mean pixel saliency value over each item per condition
  std::vector<double> itsTargetFrameTotalONMean;
  //! Mean pixel saliency value over each item per condition
  std::vector<double> itsDistFrameTotalONMean;
  //! Mean pixel saliency value over each item per condition
  std::vector<double> itsOtherFrameTotalONMean;

  //! Std pixel saliency value over each item per condition
  std::vector<double> itsTargetFrameTotalONStd;
  //! Std pixel saliency value over each item per condition
  std::vector<double> itsDistFrameTotalONStd;
  //! Std pixel saliency value over each item per condition
  std::vector<double> itsOtherFrameTotalONStd;

private:
  //! this frames "ground truth" image
  Image<PixRGB<double> > itsGroundTruth;
  //! this frames Saliency Map
  Image<double>            itsSalMap;

  //! Ground truth Target Color
  PixRGB<double> itsGTtargetColor;
  //! Ground truth Target Color for patch
  PixRGB<double> itsGTtargetColorPatch1;
  //! Ground truth Target Color for large patch
  PixRGB<double> itsGTtargetColorPatch2;
  //! Ground truth Target Color for patch if target is off
  PixRGB<double> itsGTtargetColorPatch1off;
  //! Ground truth Target Color for patch if target is off
  PixRGB<double> itsGTtargetColorPatch2off;
  //! Ground truth Distractor Color
  PixRGB<double> itsGTdistColor;

  //! Total number of frames
  uint   itsFrames;
  //! The current frame
  uint   itsFrame;
  //! Total number of conditions
  ushort itsConditions;
  //! The current condition for this frame
  ushort itsCondition;
};

#endif // STIM_ANALYZER_H_DEFINED
