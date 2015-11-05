/*!@file Psycho/StimAnalyzer.C make different kind of visual test stimuli
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimAnalyzer.C $
// $Id: StimAnalyzer.C 6795 2006-06-29 20:45:32Z rjpeters $


#ifndef STIM_ANALYZER_C_DEFINED
#define STIM_ANALYZER_C_DEFINED

#include "Psycho/StimAnalyzer.H"
#include <cfloat>
#include <fstream>
#include <iostream>

using namespace std;

StimAnalyzer::StimAnalyzer(const int frames, const ushort conditions)
{
  itsGTtargetColor.set(255.0F,255.0F,255.0F);
  itsGTtargetColorPatch1.set(0.0F,128.0F,128.0F);
  itsGTtargetColorPatch2.set(0.0F,0.0F,255.0F);
  itsGTtargetColorPatch1off.set(128.0F,128.0F,0.0F);
  itsGTtargetColorPatch2off.set(255.0F,0.0F,0.0F);
  itsGTdistColor.set(128.0F,128.0F,128.0F);
  SAinit(frames,conditions);
}

/*****************************************************************************/

StimAnalyzer::~StimAnalyzer()
{}

/*****************************************************************************/

void StimAnalyzer::SAinit(const int frames, const ushort conditions)
{
  itsFrames     = frames;
  itsConditions = conditions;

  itsTargetFrameOn.resize(frames,false);
  itsDistFrameOn.resize(frames,false);
  itsOtherFrameOn.resize(frames,false);

  itsTargetFrameNumber.resize(frames,0);
  itsDistFrameNumber.resize(frames,0);
  itsOtherFrameNumber.resize(frames,0);

  itsTargetFrameTotalONNumber.resize(itsConditions,0);
  itsDistFrameTotalONNumber.resize(itsConditions,0);
  itsOtherFrameTotalONNumber.resize(itsConditions,0);

  itsTargetFrameSum.resize(frames,0.0F);
  itsDistFrameSum.resize(frames,0.0F);
  itsOtherFrameSum.resize(frames,0.0F);

  itsTargetFrameSS.resize(frames,0.0F);
  itsDistFrameSS.resize(frames,0.0F);
  itsOtherFrameSS.resize(frames,0.0F);

  itsTargetFrameMin.resize(frames,DBL_MAX);
  itsDistFrameMin.resize(frames,DBL_MAX);
  itsOtherFrameMin.resize(frames,DBL_MAX);

  itsTargetFrameMax.resize(frames,0.0F);
  itsDistFrameMax.resize(frames,0.0F);
  itsOtherFrameMax.resize(frames,0.0F);

  itsTargetFrameMean.resize(frames,0.0F);
  itsDistFrameMean.resize(frames,0.0F);
  itsOtherFrameMean.resize(frames,0.0F);

  itsTargetFrameStd.resize(frames,0.0F);
  itsDistFrameStd.resize(frames,0.0F);
  itsOtherFrameStd.resize(frames,0.0F);

  itsTargetFrameTotalONSum.resize(itsConditions,0.0F);
  itsDistFrameTotalONSum.resize(itsConditions,0.0F);
  itsOtherFrameTotalONSum.resize(itsConditions,0.0F);

  itsTargetFrameTotalONSS.resize(itsConditions,0.0F);
  itsDistFrameTotalONSS.resize(itsConditions,0.0F);
  itsOtherFrameTotalONSS.resize(itsConditions,0.0F);

  itsTargetFrameTotalONMin.resize(itsConditions,DBL_MAX);
  itsDistFrameTotalONMin.resize(itsConditions,DBL_MAX);
  itsOtherFrameTotalONMin.resize(itsConditions,DBL_MAX);

  itsTargetFrameTotalONMax.resize(itsConditions,0.0F);
  itsDistFrameTotalONMax.resize(itsConditions,0.0F);
  itsOtherFrameTotalONMax.resize(itsConditions,0.0F);

  itsTargetFrameTotalONMean.resize(itsConditions,0.0F);
  itsDistFrameTotalONMean.resize(itsConditions,0.0F);
  itsOtherFrameTotalONMean.resize(itsConditions,0.0F);

  itsTargetFrameTotalONStd.resize(itsConditions,0.0F);
  itsDistFrameTotalONStd.resize(itsConditions,0.0F);
  itsOtherFrameTotalONStd.resize(itsConditions,0.0F);
}

/*****************************************************************************/

void StimAnalyzer::SAinputImages(const Image<double>          salmap,
                                 const Image<PixRGB<double> > groundTruth,
                                 const uint   frame,
                                 const ushort condition)
{
  itsSalMap      = salmap;
  itsGroundTruth = groundTruth;
  itsFrame       = frame;
  itsCondition   = condition;
  SAcompImages();
}

/*****************************************************************************/

void StimAnalyzer::SAcompImages()
{
  double targetMax = 0.0F; double targetMin = DBL_MAX;
  double distMax   = 0.0F; double distMin   = DBL_MAX;
  double otherMax  = 0.0F; double otherMin  = DBL_MAX;

  int targetN = 0; double sumTarget = 0.0F; double ssTarget = 0.0F;
  int distN   = 0; double sumDist   = 0.0F; double ssDist   = 0.0F;
  int otherN  = 0; double sumOther  = 0.0F; double ssOther  = 0.0F;

  Image<double>::iterator salMapItr               = itsSalMap.beginw();
  Image<PixRGB<double> >::iterator groundTruthItr = itsGroundTruth.beginw();

  // Get the saliency values over Targets, distractors and other pixels

  while(salMapItr != itsSalMap.endw())
  {
    if((*groundTruthItr == itsGTtargetColor))//       ||
      ///      (*groundTruthItr == itsGTtargetColorPatch1) ||
      //     (*groundTruthItr == itsGTtargetColorPatch2))
    {
      if(*salMapItr > targetMax)
        targetMax = *salMapItr;
      else if(*salMapItr < targetMin)
        targetMin = *salMapItr;

      sumTarget += *salMapItr;
      ssTarget  += pow(*salMapItr,2.0);
      targetN++;
    }
    else if(*groundTruthItr == itsGTdistColor)
    {
      if(*salMapItr > distMax)
        distMax = *salMapItr;
      else if(*salMapItr < distMin)
        distMin = *salMapItr;

      sumDist += *salMapItr;
      ssDist  += pow(*salMapItr,2.0);
      distN++;
    }
    else
    {
      if(*salMapItr > otherMax)
        otherMax = *salMapItr;
      else if(*salMapItr < otherMin)
        otherMin = *salMapItr;

      sumOther += *salMapItr;
      ssOther  += pow(*salMapItr,2.0);
      otherN++;
    }
    ++salMapItr; ++groundTruthItr;
  }

  // Set whether targets and distractors are On or Off in this frame

  if(targetN != 0)
    itsTargetFrameOn[itsFrame]   = true;
  if(distN != 0)
    itsDistFrameOn[itsFrame]     = true;
  if(otherN != 0)
    itsOtherFrameOn[itsFrame]    = true;

  // set the final stat values for this frame

  itsTargetFrameNumber[itsFrame] = targetN;
  itsDistFrameNumber[itsFrame]   = distN;
  itsOtherFrameNumber[itsFrame]  = otherN;

  itsTargetFrameSum[itsFrame]    = sumTarget;
  itsDistFrameSum[itsFrame]      = sumDist;
  itsOtherFrameSum[itsFrame]     = sumOther;

  itsTargetFrameSS[itsFrame]     = ssTarget;
  itsDistFrameSS[itsFrame]       = ssDist;
  itsOtherFrameSS[itsFrame]      = ssOther;

  itsTargetFrameMin[itsFrame]    = targetMin;
  itsDistFrameMin[itsFrame]      = distMin;
  itsOtherFrameMin[itsFrame]     = otherMin;

  itsTargetFrameMax[itsFrame]    = targetMax;
  itsDistFrameMax[itsFrame]      = distMax;
  itsOtherFrameMax[itsFrame]     = otherMax;

  // compute mean and std for this frame

  itsTargetFrameMean[itsFrame]   = sumTarget/targetN;
  itsDistFrameMean[itsFrame]     = sumDist/distN;
  itsOtherFrameMean[itsFrame]    = sumOther/otherN;

  itsTargetFrameStd[itsFrame]    = sqrt((ssTarget/targetN)
    - pow(itsTargetFrameMean[itsFrame],2.0));
  itsDistFrameStd[itsFrame]      = sqrt((ssDist/distN)
    - pow(itsDistFrameMean[itsFrame],2.0));
  itsOtherFrameStd[itsFrame]     = sqrt((ssOther/otherN)
    - pow(itsOtherFrameMean[itsFrame],2.0));

  // gather values for this "condition"
  // additionally, keep track of whether the item is On or Off
  // in this frame

  if(itsTargetFrameOn[itsFrame])
  {
    itsTargetFrameTotalONNumber[itsCondition] += targetN;
    itsTargetFrameTotalONSum[itsCondition]    += sumTarget;
    itsTargetFrameTotalONSS[itsCondition]     += pow(sumTarget,2.0);
    if(targetMin < itsTargetFrameTotalONMin[itsCondition])
      itsTargetFrameTotalONMin[itsCondition]   = targetMin;
    else if (targetMax > itsTargetFrameTotalONMax[itsCondition])
      itsTargetFrameTotalONMax[itsCondition]   = targetMax;
  }

  if(itsDistFrameOn[itsFrame])
  {
    itsDistFrameTotalONNumber[itsCondition] += distN;
    itsDistFrameTotalONSum[itsCondition]    += sumDist;
    itsDistFrameTotalONSS[itsCondition]     += pow(sumDist,2.0);
    if(distMin < itsDistFrameTotalONMin[itsCondition])
      itsDistFrameTotalONMin[itsCondition]   = distMin;
    else if (distMax > itsDistFrameTotalONMax[itsCondition])
      itsDistFrameTotalONMax[itsCondition]   = distMax;
  }

  if(itsOtherFrameOn[itsFrame])
  {
    itsOtherFrameTotalONNumber[itsCondition] += otherN;
    itsOtherFrameTotalONSum[itsCondition]    += sumOther;
    itsOtherFrameTotalONSS[itsCondition]     += pow(sumOther,2.0);
    if(otherMin < itsOtherFrameTotalONMin[itsCondition])
      itsOtherFrameTotalONMin[itsCondition]   = otherMin;
    else if (otherMax > itsOtherFrameTotalONMax[itsCondition])
      itsOtherFrameTotalONMax[itsCondition]   = otherMax;
  }
}

/*****************************************************************************/

void StimAnalyzer::SAfinalStats()
{

  for(ushort i = 0; i < itsConditions; i++)
  {
    // compute means over conditions
    itsTargetFrameTotalONMean[i]   =
      itsTargetFrameTotalONSum[i]/itsTargetFrameTotalONNumber[i];
    itsDistFrameTotalONMean[i]     =
      itsDistFrameTotalONSum[i]/itsDistFrameTotalONNumber[i];
    itsOtherFrameTotalONMean[i]    =
      itsOtherFrameTotalONSum[i]/itsOtherFrameTotalONNumber[i];

    // computer std over conditions
    itsTargetFrameTotalONStd[i] =
      sqrt((itsTargetFrameTotalONSS[i]/itsTargetFrameTotalONNumber[i]) -
           pow(itsTargetFrameTotalONMean[i],2.0));
    itsDistFrameTotalONStd[i] =
      sqrt((itsDistFrameTotalONSS[i]/itsDistFrameTotalONNumber[i]) -
           pow(itsDistFrameTotalONMean[i],2.0));
    itsOtherFrameTotalONStd[i] =
      sqrt((itsOtherFrameTotalONSS[i]/itsOtherFrameTotalONNumber[i]) -
           pow(itsOtherFrameTotalONMean[i],2.0));
  }
}

/*****************************************************************************/

void StimAnalyzer::SAdumpFrameStats(string fileName, string sample,
                                    bool printHeader)
{


  if(printHeader)
  {
    ofstream out(fileName.c_str(),ios::out);

    out << "Sample\tFrame\t";

    out << "Target_On\tTarget_N\tTarget_Sum\tTarget_SS\tTarget_Min\t"
        << "Target_Max\tTarget_Mean\tTarget_Std\t";

    out << "Dist_On\tDist_N\tDist_Sum\tDist_SS\tDist_Min\tDist_Max\t"
        << "Dist_Mean\tDist_Std\t";

    out << "Other_On\tOther_N\tOther_Sum\tOther_SS\tOther_Min\tOther_Max\t"
        << "Other_Mean\tOther_Std\t";

    out << "\n";

    out.close();
  }

  ofstream out(fileName.c_str(),ios::app);

  for(uint i = 0; i < itsFrames; i++)
  {
    out << sample << "\t" << i << "\t";

    out << itsTargetFrameOn[i]      << "\t"
        << itsTargetFrameNumber[i]  << "\t"
        << itsTargetFrameSum[i]     << "\t"
        << itsTargetFrameSS[i]      << "\t"
        << itsTargetFrameMin[i]     << "\t"
        << itsTargetFrameMax[i]     << "\t"
        << itsTargetFrameMean[i]    << "\t"
        << itsTargetFrameStd[i]     << "\t";

    out << itsDistFrameOn[i]        << "\t"
        << itsDistFrameNumber[i]    << "\t"
        << itsDistFrameSum[i]       << "\t"
        << itsDistFrameSS[i]        << "\t"
        << itsDistFrameMin[i]       << "\t"
        << itsDistFrameMax[i]       << "\t"
        << itsDistFrameMean[i]      << "\t"
        << itsDistFrameStd[i]       << "\t";

    out << itsOtherFrameOn[i]       << "\t"
        << itsOtherFrameNumber[i]   << "\t"
        << itsOtherFrameSum[i]      << "\t"
        << itsOtherFrameSS[i]       << "\t"
        << itsOtherFrameMin[i]      << "\t"
        << itsOtherFrameMax[i]      << "\t"
        << itsOtherFrameMean[i]     << "\t"
        << itsOtherFrameStd[i]      << "\t";

    out << "\n";
  }
  out.close();
}
/*****************************************************************************/

void StimAnalyzer::SAdumpConditionStats(string fileName, string sample,
                                        bool printHeader)
{


  if(printHeader)
  {
    ofstream out(fileName.c_str(),ios::out);

    out << "Sample\tCondition\t";

    out << "Target_N\tTarget_Sum\tTarget_SS\tTarget_Min\tTarget_Max\t"
        << "Target_Mean\tTarget_Std\t";

    out << "Dist_N\tDist_Sum\tDist_SS\tDist_Min\tDist_Max\t"
        << "Dist_Mean\tDist_Std\t";

    out << "Other_N\tOther_Sum\tOther_SS\tOther_Min\tOther_Max\t"
        << "Other_Mean\tOther_Std\t";

    out << "\n";

    out.close();
  }

  ofstream out(fileName.c_str(),ios::app);

  for(ushort i = 0; i < itsConditions; i++)
  {
    out << sample << "\t" << i << "\t";

    out << itsTargetFrameTotalONNumber[i] << "\t"
        << itsTargetFrameTotalONSum[i]    << "\t"
        << itsTargetFrameTotalONSS[i]     << "\t"
        << itsTargetFrameTotalONMin[i]    << "\t"
        << itsTargetFrameTotalONMax[i]    << "\t"
        << itsTargetFrameTotalONMean[i]   << "\t"
        << itsTargetFrameTotalONStd[i]    << "\t";

    out << itsDistFrameTotalONNumber[i]   << "\t"
        << itsDistFrameTotalONSum[i]      << "\t"
        << itsDistFrameTotalONSS[i]       << "\t"
        << itsDistFrameTotalONMin[i]      << "\t"
        << itsDistFrameTotalONMax[i]      << "\t"
        << itsDistFrameTotalONMean[i]     << "\t"
        << itsDistFrameTotalONStd[i]      << "\t";

    out << itsOtherFrameTotalONNumber[i]  << "\t"
        << itsOtherFrameTotalONSum[i]     << "\t"
        << itsOtherFrameTotalONSS[i]      << "\t"
        << itsOtherFrameTotalONMin[i]     << "\t"
        << itsOtherFrameTotalONMax[i]     << "\t"
        << itsOtherFrameTotalONMean[i]    << "\t"
        << itsOtherFrameTotalONStd[i]     << "\t";

    out << "\n";
  }
  out.close();
}




#endif // STIM_ANALYZER_C_DEFINED
