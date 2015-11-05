/*!@file VFAT/segmentImageTrackMC.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageTrackMC.C $
// $Id: segmentImageTrackMC.C 9412 2008-03-10 23:10:15Z farhan $
//

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itti itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#ifndef SEGMENTIMAGETRACKMC_C_DEFINED
#define SEGMENTIMAGETRACKMC_C_DEFINED

#include "Util/Assert.H"
#include "VFAT/segmentImageTrackMC.H"
#include "Image/DrawOps.H"
#include "Raster/Raster.H"

#include <climits>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <time.h>


//! maximum lose of tracks before color is reset
#define LOTMAX            5

//! How many iteration to calculate over for movement statistics
#define ERRINTERVAL       5

//! decimation size reduction factor
#define DEC               4

//! Width of channel bars
#define BARWIDTH          10

//! spacing between channel histo bars
#define BARSPACE          5

//! histogram height
#define HISTOHEIGHT       450.0F

/*********************************************************************/
// compute color adaptation

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITcolorAdaptation()
{
  if(SIT_resetColor == false)
  {
    FLOAT mass = 0;
    typename std::vector<FLOAT>::iterator imean1 = SIT_chMean1.begin();
    typename std::vector<FLOAT>::iterator imean2;
    typename std::vector<FLOAT>::iterator istd1;
    typename std::vector<FLOAT>::iterator istd2;
    typename std::vector<FLOAT>::iterator ilb;
    typename std::vector<FLOAT>::iterator iub;

    while(imean1 != SIT_chMean1.end())
      *imean1++ = 0;

    // iterate over all blobs and asses new color thresholds from
    // candidate blobs
    SIT_blobListSize = 0;
    // (1) figure out which blobs we wish to extract information from
    for(INT i = 0; i < (unsigned)SIT_segment.SInumberBlobs(); i++)
    {
      if(SIT_candidateBlob[i] == true)
      {
        SIT_blobList[SIT_blobListSize] = i;
        SIT_blobListSize++;
      }
    }
    // (2) find mean color values for each blob if we have at least
    // one blob. This includes
    //        (a) mean color for each blob
    //        (b) variance (sum of squares) color for each blob
    //        (c) Mass of each blob (number of pixels)
    if(SIT_blobListSize > 0)
      SIT_segment.SIgetValueMean(&SIT_blobListSize,&SIT_blobList,
                                 &SIT_chMean1,&SIT_chStd1,&mass);

    // (3) If we have elected to, draw tracking targets for blobs
    if(SIT_drawTargetImage == true)
    {
      for(INT i = 0; i < SIT_segment.SInumberBlobs(); i++)
      {
        if(SIT_candidateBlob[i] == true)
        {
          SITdrawBlobTrack(i);
        }
        else
        {
          if(SIT_killedByTrack[i] == false)
          {
            SITdrawBlobBox(i);
          }
        }
      }
    }

    // (4) draw color bar graph if blobs have any mass
    if((SIT_LOT == false) && (SIT_drawTargetImage == true))
    {
      SITdrawBlobTrackMerged();
    }

    if(SIT_mass != 0)
    {
      istd1 = SIT_chStd1.begin();
      SIT_oldMean   = SIT_chMean1;
      SIT_oldStd    = SIT_chStd1;
      SIT_oldUB     = SIT_chUB;
      SIT_oldLB     = SIT_chLB;
      SIT_oldNorm   = SIT_chNorm;
      SIT_draw      = true;
    }

    // (5.a) If loss of track is registered 5 times (whatever LOTMAX is)
    // , reset color to orignal values from start up. throw away adaptive
    // values since they are no longer dependable
    if((SIT_LOT == true) && (SIT_useColorAdaptation == true))
    {
      //LINFO("LOT Number %d",SIT_LOTcount);
      if(SIT_drawColorAdaptImage == true)
        SITdrawHistoValues(&SIT_oldMean,&SIT_oldStd,
                           &SIT_oldUB,&SIT_oldLB,&SIT_oldNorm,true);
      if(SIT_LOTcount > LOTMAX)
      {
        LINFO("COLOR RESET due to LOT COUNT");
        SITresetColor(); // Will set SIT_resetColor = true
      }
      else
      {
        SIT_LOTcount++;
      }
    }
    else
    {
      SIT_LOTcount = 0;
    }

    //if((SIT_mass != 0) && (SIT_resetColor == false))
    if((SIT_LOT == false) && (SIT_useColorAdaptation == true))
    //if(SIT_mass != 0)
    {
      if(SIT_drawColorAdaptImage == true)
        SITdrawHistoValues(&SIT_chMean1,&SIT_chStd1,
                           &SIT_chUB,&SIT_chLB,&SIT_chNorm,false);
      // (5.b) if adaptive thresholding is turned on, adjust color
      // by standard deviation of color. As such each blobs color will
      // determin the new tracking color.
      istd1        = SIT_chStd1.begin();
      ilb          = SIT_chLB.begin();
      iub          = SIT_chUB.begin();

      typename std::vector<FLOAT>::iterator iadapt = SIT_chAdapt.begin();
      typename std::vector<FLOAT>::iterator instd = SIT_chNSTD.begin();

      for(imean1 = SIT_chMean1.begin(); imean1 != SIT_chMean1.end(); ++imean1,
            ++istd1, ++iub, ++ilb, ++instd)
      {

        *instd = *istd1*(*iadapt);
        /*
        if(*imean1 > *iub)
        {
          *imean1 = *iub;
        }
        else if(*imean1 < *ilb)
        {
          *imean1 = *ilb;
        }
        */

      }
      if(SIT_useLog == true)
      {
        std::ofstream outfile(SIT_LOG_FILE,std::ios::app);
        outfile << "FRAME " << SIT_frameNumber << " ADAPT " << "\t";
        typename std::vector<FLOAT>::iterator instd = SIT_chNSTD.begin();
        for(imean1 = SIT_chMean1.begin(); imean1 != SIT_chMean1.end();
            ++imean1,++instd)
        {
          outfile <<  *imean1 << " +/- (" << *instd << ")\t";
        }
        outfile << "\n";
        outfile.close();
      }
      SIT_segment.SIsetVal(SIT_chMean1,SIT_chNSTD,SIT_chSkew);
    }
  }
  else
  {
    LINFO("COLOR RESET TO USERS");
    SIT_resetColor = false;
  }
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITresetColor()
{
  SIT_segment.SIresetAvg();
  SIT_segment.SIsetVal(SIT_initMean,SIT_initStd,SIT_chSkew);

  if(SIT_useLog == true)
  {
    std::ofstream outfile(SIT_LOG_FILE,std::ios::app);
    outfile << "FRAME " << SIT_frameNumber << " RESET " << "\t";
    typename std::vector<FLOAT>::iterator imean = SIT_initMean.begin();
    typename std::vector<FLOAT>::iterator istd  = SIT_initStd.begin();
    while(imean != SIT_initMean.end())
    {
      outfile <<  *imean << " +/- (" << *istd << ")\t";
      ++imean; ++istd;
    }
    outfile << "\n";
    outfile.close();
  }

  SIT_segment.SIresetCandidates(true);
  SIT_LOTcount = 0;
  SIT_resetColor  = true;
  SIT_LOTandRESET = true;
}

/*********************************************************************/
// weed out blobs that suck

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITanalyzeBlobs()
{
  SIT_totalBlobs = SIT_segment.SInumberBlobs();
  SIT_killedBlobs = 0;

  // (1.a) We have a loss of track, as such we need to reset some of
  // the adaptive parameters. Here we reset the parameter where we
  // expect the target to be to a much larger area since we no longer
  // can assume its location do to loss of track.
  if(SIT_LOT == true)
  {
    SIT_xBound = SIT_blobProp.BP_LOTbound;
    SIT_yBound = SIT_blobProp.BP_LOTbound;
  }
  else
  {
    // (1.b) We expect the target to appear close to where we last saw it
    // we set a parameter around the last known location. Any blob outside
    // this parameter must be predjidist

    SIT_xBound = (int)(SIT_blobProp.BP_bound +
                       (SIT_thresh*SIT_blobProp.BP_softBound));
    SIT_yBound = (int)(SIT_blobProp.BP_bound +
                       (SIT_thresh*SIT_blobProp.BP_softBound));
  }

  std::vector<bool>::iterator softCandidateBlobItr
    = SIT_softCandidateBlob.begin();
  std::vector<bool>::iterator candidateBlobItr
    = SIT_candidateBlob.begin();
  std::vector<bool>::iterator killedByTrackItr
    = SIT_killedByTrack.begin();
  std::vector<std::string>::iterator reasonForKillItr
    = SIT_reasonForKill.begin();
  std::vector<unsigned short>::iterator reasonForKillCodeItr
    = SIT_reasonForKillCode.begin();

  std::string S1,S2,S3;
  char C1[48], C2[48], C3[48];
  // (2) For each blob that the tracker found as a potential target,
  // check its properties such as its location in the image vs.
  // where we expect the target to be. Also check to make sure that the
  // target is geometrically good. That is, make sure its mass, size and
  // basic shape conform to our expectations.
  for(INT i = 0; i < SIT_totalBlobs; i++,
        ++softCandidateBlobItr, ++candidateBlobItr,
        ++killedByTrackItr, ++reasonForKillItr, ++reasonForKillCodeItr)
  {
    *softCandidateBlobItr = true;
    *candidateBlobItr     = true;
    *killedByTrackItr     = false;
    *reasonForKillCodeItr = 0;
    *reasonForKillItr     = "(0) Blob OK ";

    LDEBUG("[%d] expected x min = %d",
           SIT_useExpectedLocation == true, SIT_expectedXmin);


    // (3) check if a blobs mass is within constraints. That is
    // Make sure the blob is neither too large or to small. If so, we
    // don't like it.
    if((SIT_segment.SIgetMass(i) < (INT)SIT_blobProp.BP_minMass)
       && (SIT_blobProp.BP_checkMass == true))
    {
      *candidateBlobItr     = false;
      *softCandidateBlobItr = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 1;
      SIT_killedBlobs++;

      S1 = " Blob Mass Contraints : Min : ";  S2 = " < ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetMass(i));
      sprintf(C2,"%d",(int)SIT_blobProp.BP_minMass);
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    else if((SIT_segment.SIgetMass(i) > (INT)SIT_blobProp.BP_maxMass)
            && (SIT_blobProp.BP_checkMass == true))
    {

      *candidateBlobItr     = false;
      *softCandidateBlobItr = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 2;
      SIT_killedBlobs++;

      S1 = " Blob Mass Contraints : Max : ";  S2 = " > ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetMass(i));
      sprintf(C2,"%d",(int)SIT_blobProp.BP_maxMass);
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    // (4) check that a blob is within our X frame. That is, does
    // a blob fall in the image in a place where we would expect it
    // for instance close to where we last saw it.
    else if((SIT_segment.SIgetCenterX(i) < (SIT_centerX - SIT_xBound))
            && (SIT_blobProp.BP_checkFrameX == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 3;
      SIT_killedBlobs++;

      S1 = " Blob Out of X Frame : Low : "; S2 = " < ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterX(i));
      sprintf(C2,"%d",(int)(SIT_centerX - SIT_xBound));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    else if((SIT_segment.SIgetCenterX(i) > (SIT_centerX + SIT_xBound))
            && (SIT_blobProp.BP_checkFrameX == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 4;
      SIT_killedBlobs++;

      S1 = " Blob Out of X Frame : High : "; S2 = " > ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterX(i));
      sprintf(C2,"%d",(int)(SIT_centerX + SIT_xBound));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    // (5) check that a blob is within our Y frame. That is, does
    // a blob fall in the image in a place where we would expect it
    // for instance close to where we last saw it.
    else if((SIT_segment.SIgetCenterY(i) < (SIT_centerY - SIT_yBound))
            && (SIT_blobProp.BP_checkFrameY == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 5;
      SIT_killedBlobs++;

      S1 = " Blob Out of Y Frame : Low : "; S2 = " < ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterY(i));
      sprintf(C2,"%d",(int)(SIT_centerY - SIT_yBound));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    else if((SIT_segment.SIgetCenterY(i) > (SIT_centerY + SIT_yBound))
            && (SIT_blobProp.BP_checkFrameY == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 6;
      SIT_killedBlobs++;

      S1 = " Blob Out of Y Frame : High : "; S2 = " > ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterY(i));
      sprintf(C2,"%d",(int)(SIT_centerY + SIT_yBound));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    // (6) check that a blob is within our EXPECTED X frame. That is, does
    // a blob fall in the image in a place where we would expect it
    // for instance close to where we last saw it.
    // This is used only if we apriori set this value
    else if((SIT_segment.SIgetCenterX(i) > SIT_expectedXmax) &&
            (SIT_useExpectedLocation == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 7;
      SIT_killedBlobs++;

      S1 = " Blob Out of Set Expected X Frame : High : "; S2 = " > ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterX(i));
      sprintf(C2,"%d",(int)(SIT_expectedXmax));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    else if((SIT_segment.SIgetCenterX(i) < SIT_expectedXmin) &&
            (SIT_useExpectedLocation == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 8;
      SIT_killedBlobs++;

      S1 = " Blob Out of Set Expected X Frame : Low : "; S2 = " < ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterX(i));
      sprintf(C2,"%d",(int)(SIT_expectedXmin));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    // (7) check that a blob is within our EXPECTED Y frame. That is, does
    // a blob fall in the image in a place where we would expect it
    // for instance close to where we last saw it.
    // This is used only if we apriori set this value
    else if((SIT_segment.SIgetCenterY(i) > SIT_expectedYmax) &&
            (SIT_useExpectedLocation == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 9;
      SIT_killedBlobs++;

      S1 = " Blob Out of Set Expected Y Frame : High : "; S2 = " > ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterY(i));
      sprintf(C2,"%d",(int)(SIT_expectedYmax));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    else if((SIT_segment.SIgetCenterY(i) < SIT_expectedYmin) &&
            (SIT_useExpectedLocation == true))
    {
      *candidateBlobItr     = false;
      *killedByTrackItr     = true;
      *reasonForKillCodeItr = 10;
      SIT_killedBlobs++;

      S1 = " Blob Out of Set Expected Y Frame : Low : "; S2 = " < ";
      sprintf(C1,"%d",(int)SIT_segment.SIgetCenterY(i));
      sprintf(C2,"%d",(int)(SIT_expectedYmin));
      sprintf(C3,"(%d)",*reasonForKillCodeItr);
      *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

      LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
    }
    // (8) check that blob is within size ratios. That is, does the
    // blob have a proper height and width for the target as we would
    // expect. For instance, as head should have a height and width that
    // are about the same.
    else if(SIT_blobProp.BP_checkSizeRatios == true)
    {
      FLOAT foo = (SIT_segment.SIgetXmax(i) - SIT_segment.SIgetXmin(i));
      if(foo != 0)
      {
        FLOAT temp = (SIT_segment.SIgetYmax(i)
                      - SIT_segment.SIgetYmin(i))/foo;

        if(temp < (SIT_blobProp.BP_ratioMin
                   + (SIT_thresh*SIT_blobProp.BP_softRatioMin)))
        {
          *candidateBlobItr     = false;
          *killedByTrackItr     = true;
          *reasonForKillCodeItr = 11;
          SIT_killedBlobs++;

          S1 = " Blob Size Ratios Contraint : Min :"; S2 = " < ";
          sprintf(C1,"%d",(int)temp);
          sprintf(C2,"%d",(int)(SIT_blobProp.BP_ratioMin
                                + (SIT_thresh*SIT_blobProp.BP_softRatioMin)));
          sprintf(C3,"(%d)",*reasonForKillCodeItr);
          *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

          LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
        }
        else if (temp > (SIT_blobProp.BP_ratioMax
                         + (SIT_thresh*SIT_blobProp.BP_softRatioMin)))
        {
          *candidateBlobItr     = false;
          *killedByTrackItr     = true;
          *reasonForKillCodeItr = 12;
          SIT_killedBlobs++;

          S1 = " Blob Size Ratios Contraint : Max : "; S2 = " > ";
          sprintf(C1,"%d",(int)temp);
          sprintf(C2,"%d",(int)(SIT_blobProp.BP_ratioMax
                                + (SIT_thresh*SIT_blobProp.BP_softRatioMin)));
          sprintf(C3,"(%d)",*reasonForKillCodeItr);
          *reasonForKillItr = C3 + S1 + C1 + S2 + C2;

          LDEBUG("%3d. reason to kill: %s", i, (*reasonForKillItr).c_str());
        }
        else
          LDEBUG("%3d. not killed", i);
      }
    }
  }
}


/*********************************************************************/
// merge all remaining blobs into a single new blob

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITmergeBlobs()
{
  SIT_mass    = 0;
  FLOAT meanX = 0;
  FLOAT meanY = 0;
  SIT_minX    = UINT_MAX;
  SIT_minY    = UINT_MAX;
  SIT_maxX    = 0;
  SIT_maxY    = 0;

  // calculate the center of a combined blob from the average
  // mass center of all remaining blobs
  std::vector<bool>::iterator candidateBlobItr = SIT_candidateBlob.begin();

  for(INT i = 0; i < SIT_segment.SInumberBlobs(); i++, ++candidateBlobItr)
  {
    if(*candidateBlobItr == true)
    {
      LDEBUG("*candidateBlobItr == true");
      SIT_mass += SIT_segment.SIgetMass(i);
      meanX    += SIT_segment.SIgetMass(i)*SIT_segment.SIgetCenterX(i);
      meanY    += SIT_segment.SIgetMass(i)*SIT_segment.SIgetCenterY(i);

      if((unsigned)SIT_segment.SIgetXmax(i) > SIT_maxX)
                                SIT_maxX = SIT_segment.SIgetXmax(i);

      if((unsigned)SIT_segment.SIgetYmax(i) > SIT_maxY)
                                SIT_maxY = SIT_segment.SIgetYmax(i);

      if((unsigned)SIT_segment.SIgetXmin(i) < SIT_minX)
                                SIT_minX = SIT_segment.SIgetXmin(i);

      if((unsigned)SIT_segment.SIgetYmin(i) < SIT_minY)
                                SIT_minY = SIT_segment.SIgetYmin(i);
    }

    LDEBUG("%3d. [%3f %3f] SIT_mass: %f ", i,
           float(SIT_segment.SIgetCenterX(i)),
           float(SIT_segment.SIgetCenterY(i)),
           float(SIT_segment.SIgetMass(i)));
  }

  // Compute SIT_LOT here and set
  if(SIT_mass != 0)
  {
    SIT_centerX = (int)(meanX/SIT_mass);
    SIT_centerY = (int)(meanY/SIT_mass);
    SIT_centerM = (int)(SIT_mass);

    if(((SIT_maxX - SIT_minX) * (SIT_maxY - SIT_minY)) >
       ((SIT_segment.SIgetImageSizeX() * SIT_segment.SIgetImageSizeY())/
        SIT_blobProp.BP_maxFrameSize))
    {
      SIT_LOT         = true;
      SIT_LOTtype     = 1;
      SIT_LOTtypeName = "(1) Positive Mass but Spacially too Large";
      LDEBUG("(1) Positive Mass but Spacially too Large");
    }
    else
    {
      SIT_LOT                 = false;
      SIT_LOTtypeName         = "(0) NO LOT, OK";
      SIT_useExpectedLocation = false;
      LDEBUG("(0) NO LOT, OK");
    }
  }
  else
  {
    SIT_LOTtypeName = "(2) Zero Mass in merged blobs";
    SIT_LOTtype = 2;
    SIT_LOT     = true;
    LDEBUG("(2) Zero Mass in merged blobs");

    // JACOB: changed this to fixed values instead of calling
    // a pointer that may or may not be assigned to something
    SIT_centerX = SIT_x_center;
    SIT_centerY = SIT_y_center;
  }
}

/*********************************************************************/
// PUBLIC ACCESS METHODS
/*********************************************************************/

// When called at the start, this will resize all the vectors we use
// We only call this once since it is expensive to call.
template SIT_TEMPLATE_CLASS
segmentImageTrackMC<SIT_TEMPLATE>::segmentImageTrackMC(INT maxBlobCount)
{
  SIT_LOTcount            = 0;
  SIT_frameNumber         = 0;
  SIT_useLog              = false;
  SIT_drawTargetImage     = true;
  SIT_drawColorAdaptImage = true;
  SIT_useColorAdaptation  = true;
  SIT_useExpectedLocation = false;

  time_t t       = time(0);
  std::string theTime = asctime(localtime(&t));

  std::ofstream outfile(SIT_LOG_FILE,std::ios::out);
  outfile << theTime << "\n";
  outfile.close();

  SIT_draw           = false;
  SIT_didCircleColor = false;
  SIT_didBoxColor    = false;
  SIT_didTrackColor  = false;

  // resize a bunch of std contaner vectors

  SIT_chMean1.resize( SIT_channels,0);
  SIT_chMean2.resize( SIT_channels,0);
  SIT_chStd1.resize(  SIT_channels,0);
  SIT_chStd2.resize(  SIT_channels,0);
  SIT_chLB.resize(    SIT_channels,0);
  SIT_chUB.resize(    SIT_channels,0);
  SIT_chNSTD.resize(  SIT_channels,0);
  SIT_chAdapt.resize( SIT_channels,0);
  SIT_chSkew.resize(  SIT_channels,0);
  SIT_initMean.resize(SIT_channels,0);
  SIT_initStd.resize( SIT_channels,0);

  SIT_softCandidateBlob.resize(maxBlobCount,false);
  SIT_candidateBlob.resize(    maxBlobCount,false);
  SIT_killedByTrack.resize(    maxBlobCount,false);
  SIT_blobList.resize(         maxBlobCount,0);
  SIT_reasonForKill.resize(    maxBlobCount,"no reason");
  SIT_reasonForKillCode.resize(maxBlobCount,0);

  SIT_barWidth    = BARWIDTH;
  SIT_barSpace    = BARSPACE;
  SIT_histoHeight = HISTOHEIGHT;


  // Load the config file and set a whole bunch of parameters from it

   blobConf.openFile("blob.conf",true);

  SIT_blobProp.BP_LOTbound     = (int)blobConf.getItemValueF("BP_LOTbound");
  SIT_blobProp.BP_bound        = (int)blobConf.getItemValueF("BP_bound");
  SIT_blobProp.BP_softBound    = (int)blobConf.getItemValueF("BP_softBound");
  SIT_blobProp.BP_lowBound     = (int)blobConf.getItemValueF("BP_lowBound");
  SIT_blobProp.BP_traj         = (int)blobConf.getItemValueF("BP_traj");
  SIT_blobProp.BP_sampleStart  = (int)blobConf.getItemValueF("BP_sampleStart");
  SIT_blobProp.BP_maxTraj      = (int)blobConf.getItemValueF("BP_maxTraj");
  SIT_blobProp.BP_maxSize      = (int)blobConf.getItemValueF("BP_maxSize");
  SIT_blobProp.BP_minSize      = (int)blobConf.getItemValueF("BP_minSize");
  SIT_blobProp.BP_maxFrameSize = blobConf.getItemValueF("BP_maxFrameSize");
  SIT_blobProp.BP_minMass      = (int)blobConf.getItemValueF("BP_minMass");
  SIT_blobProp.BP_maxMass      = (int)blobConf.getItemValueF("BP_maxMass");
  SIT_blobProp.BP_ratioMin     = blobConf.getItemValueF("BP_ratioMin");
  SIT_blobProp.BP_softRatioMin = blobConf.getItemValueF("BP_softRatioMin");
  SIT_blobProp.BP_ratioMax     = blobConf.getItemValueF("BP_ratioMax");
  SIT_blobProp.BP_softRatioMax = blobConf.getItemValueF("BP_softRatioMax");
  SIT_blobProp.BP_checkMass    = blobConf.getItemValueB("BP_checkMass");
  SIT_blobProp.BP_checkFrameX  = blobConf.getItemValueB("BP_checkFrameX");
  SIT_blobProp.BP_checkFrameY  = blobConf.getItemValueB("BP_checkFrameY");
  SIT_blobProp.BP_checkSizeRatios =
  blobConf.getItemValueB("BP_checkSizeRatios");

}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
segmentImageTrackMC<SIT_TEMPLATE>::~segmentImageTrackMC()
{}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetCircleColor(unsigned int r,
                                                       unsigned int g,
                                                       unsigned int b)
{
  SIT_circleRed      = r;
  SIT_circleBlue     = g;
  SIT_circleGreen    = b;
  SIT_didCircleColor = true;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetBoxColor(unsigned int r,
                                                    unsigned int g,
                                                    unsigned int b,
                                                    unsigned int bigr,
                                                    unsigned int bigg,
                                                    unsigned int bigb)
{
  SIT_boxRed      = r;
  SIT_boxGreen    = g;
  SIT_boxBlue     = b;
  SIT_bigBoxRed   = bigr;
  SIT_bigBoxGreen = bigg;
  SIT_bigBoxBlue  = bigb;
  SIT_didBoxColor = true;
}


/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetTrackColor(
                                      typename std::vector<FLOAT> *color,
                                      typename std::vector<FLOAT> *std,
                                      typename std::vector<FLOAT> *norm,
                                      typename std::vector<FLOAT> *adapt,
                                      typename std::vector<FLOAT> *upperBound,
                                      typename std::vector<FLOAT> *lowerBound,
                                      bool resetColor,
                                      bool resetCandidates)

{
  // make sure that all our contaners are the same size
  ASSERT(color->size()      == std->size());
  ASSERT(std->size()        == norm->size());
  ASSERT(norm->size()       == adapt->size());
  ASSERT(adapt->size()      == upperBound->size());
  ASSERT(upperBound->size() == lowerBound->size());

  // set initial tracking parameters
  SIT_thresh        = 1;
  SIT_initMean      = *color;
  SIT_initStd       = *std;
  SIT_chNorm        = *norm;
  SIT_chAdapt       = *adapt;
  SIT_chUB          = *upperBound;
  SIT_chLB          = *lowerBound;
  SIT_didTrackColor = true;
  SIT_LOTcount      = 0;
  typename std::vector<FLOAT> skew(color->size(),0);
  SIT_chSkew = skew;
  SIT_segment.SIsetAvg(ERRINTERVAL);
  SIT_segment.SIresetAvg();
  SIT_segment.SIsetVal(SIT_initMean,SIT_initStd,SIT_chSkew);

  if(SIT_useLog == true)
  {
    std::ofstream outfile(SIT_LOG_FILE,std::ios::app);
    outfile << "FRAME " << SIT_frameNumber << " INIT " << "\t";
    typename std::vector<FLOAT>::iterator imean = SIT_initMean.begin();
    typename std::vector<FLOAT>::iterator istd  = SIT_initStd.begin();
    while(imean != SIT_initMean.end())
    {
      outfile <<  *imean << " +/- (" << *istd << ")\t";
      ++imean; ++istd;
    }
    outfile << "\n";
    outfile.close();
  }


  SIT_resetColor = resetColor;
  if(resetCandidates == true)
    SIT_segment.SIresetCandidates(true);
}


/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetFrame(int *x, int *y)
{
  SIT_segment.SIsetFrame(x,y);
  SIT_x_center = *x/2;
  SIT_y_center = *y/2;
  SIT_centerX = SIT_x_center;
  SIT_centerY = SIT_y_center;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetUseSmoothing(bool smoothing, FLOAT alpha)
{
  SIT_useSmoothing   = smoothing;
  SIT_smoothingAlpha = alpha;
  SIT_didSmoothing   = false;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetExpectedTargetPosition(
                                              const unsigned int posX,
                                              const unsigned int posY,
                                              const unsigned int maxX,
                                              const unsigned int maxY,
                                              const unsigned int minX,
                                              const unsigned int minY)
{
  SIT_expectedX           = posX;
  SIT_expectedY           = posY;
  SIT_expectedXmax        = maxX;
  SIT_expectedYmax        = maxY;
  SIT_expectedXmin        = minX;
  SIT_expectedYmin        = minY;
  SIT_useExpectedLocation = true;

}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetExpectedTargetPosition(
                                              unsigned int *posX,
                                              unsigned int *posY,
                                              unsigned int *maxX,
                                              unsigned int *maxY,
                                              unsigned int *minX,
                                              unsigned int *minY,
                                              bool *isSet)
{
  *posX  = SIT_expectedX;
  *posY  = SIT_expectedY;
  *maxX  = SIT_expectedXmax;
  *maxY  = SIT_expectedYmax;
  *minX  = SIT_expectedXmin;
  *minY  = SIT_expectedYmin;
  *isSet = SIT_useExpectedLocation;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITunsetExpectedTargetPosition()
{
  SIT_useExpectedLocation = false;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITtoggleCandidateBandPass(bool toggle)
{
  SIT_segment.SItoggleCandidateBandPass(toggle);
}
/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITtoggleColorAdaptation(bool toggle)
{
  SIT_useColorAdaptation = toggle;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetFrameNumber(unsigned long frame)
{
  SIT_frameNumber = frame;
}

/*********************************************************************/
// MAIN METHOD
/*********************************************************************/

/* This is a basic tracker access method that tracks on one image at a time */
/* this one is designed to work with any Pixels pixel */

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITtrackImageAny
                               (Image<PixH2SV2<FLOAT> >& input,
                                Image<PixRGB<byte> > *image,
                                Image<PixRGB<byte> > *auxImage,
                                bool editBlobs)
{
  // Assert that parameters have been set up before starting
  ASSERT(SIT_didCircleColor == true);
  ASSERT(SIT_didBoxColor    == true);
  ASSERT(SIT_didTrackColor  == true);

  SIT_imageHold = image;
  SIT_auxHold   = auxImage;

  // decimate input image twice to speed things up
  input         = decXY(input);
  input         = decXY(input);

  Image<FLOAT> timage;
  timage.resize(input.getWidth(),input.getHeight(),true);
  SIT_chans.resize(SIT_channels,timage);
  for(unsigned int i = 0; i < SIT_channels; i++)
  {
    typename Image<FLOAT>::iterator iSIT_chans = SIT_chans[i].beginw();
    for(typename Image<PixH2SV2<FLOAT> >::iterator
        iinput = input.beginw();
        iinput != input.endw(); ++iinput, ++iSIT_chans)
    {
      *iSIT_chans = iinput->p[i];
    }
  }

  if(SIT_useSmoothing)
    SITsmoothImage(&SIT_chans);

  SITrunTrack(SIT_imageHold,&SIT_chans,editBlobs);
}

/*********************************************************************/

/* This is a basic tracker access method that tracks on one image at a time */
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITtrackImage
                                             (Image<PixRGB<byte> >& input,
                                              Image<PixRGB<byte> > *image,
                                              Image<PixRGB<byte> > *auxImage,
                                              bool editBlobs)
{

  // Assert that parameters have been set up before starting
  ASSERT(SIT_didCircleColor == true);
  ASSERT(SIT_didBoxColor    == true);
  ASSERT(SIT_didTrackColor  == true);

  SIT_imageHold = image;
  SIT_auxHold   = auxImage;
  Image< PixRGB<FLOAT> > fima;

  // decimate input image twice to speed things up
  fima         = decXY(input);
  fima         = decXY(fima);
  SIT_fimaHold = &fima;

  // break image into seperate channels
  Image<FLOAT> timage;
  timage.resize(fima.getWidth(),fima.getHeight(),true);

  //typename std::vector<Image<FLOAT> > chans(3,timage);
  SIT_chans.resize(3,timage);
  typename Image<FLOAT>::iterator iHimage = SIT_chans[0].beginw();
  typename Image<FLOAT>::iterator iSimage = SIT_chans[1].beginw();
  typename Image<FLOAT>::iterator iVimage = SIT_chans[2].beginw();
  for(typename Image<PixRGB<FLOAT> >::iterator iImage = fima.beginw();
      iImage != fima.endw(); ++iImage, ++iHimage, ++iSimage, ++iVimage)
  {
    FLOAT pixH,pixS,pixV;
    PixRGB<FLOAT> pix;
    pix = *iImage;
    PixHSV<FLOAT>(pix).getHSV(pixH,pixS,pixV);
    *iHimage = pixH; *iSimage = pixS; *iVimage = pixV;
  }

  if(SIT_useSmoothing)
    SITsmoothImage(&SIT_chans);

  SITrunTrack(SIT_imageHold,&SIT_chans,editBlobs);
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITtrackImage(typename
                                             std::vector<Image<FLOAT> >& input,
                                             Image<PixRGB<byte> > *image,
                                             Image<PixRGB<byte> > *auxImage,
                                             bool editBlobs)
{
  SIT_imageHold = image;
  SIT_auxHold   = auxImage;
  Image<FLOAT> told;
  typename std::vector<Image<FLOAT> > fima(input.size(),told);
  typename std::vector<Image<FLOAT> >::iterator ifima = fima.begin();
  for(typename std::vector<Image<FLOAT> >::iterator iinput = input.begin();
      iinput != input.end(); ++iinput, ++ifima)
  {
    *ifima = decXY(*iinput);
    *ifima = decXY(*ifima);
  }

  if(SIT_useSmoothing)
    SITsmoothImage(&fima);

  SITrunTrack(SIT_imageHold,&fima,editBlobs);
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsmoothImage(typename
                                             std::vector<Image<FLOAT> > *input)
{
  if(SIT_didSmoothing == false)
  {
    Image<FLOAT> timage;
    timage.resize(input->at(0).getWidth(),input->at(0).getHeight(),true);
    SIT_fimaLast.resize(input->size(),timage);
  }

  typename std::vector<Image<FLOAT> >::iterator iinput = input->begin();
  typename std::vector<Image<FLOAT> >::iterator ifima  = SIT_fimaLast.begin();
  while(iinput != input->end())
  {
    if(SIT_didSmoothing)
    {
      *ifima = (*iinput + ((*ifima) * SIT_smoothingAlpha))/
        (1+SIT_smoothingAlpha);
    }
    else
    {
      *ifima = *iinput;
      SIT_didSmoothing = true;
    }
    ++iinput; ++ifima;
  }
  input = &SIT_fimaLast;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITuseLog(bool useLog)
{
  SIT_useLog = useLog;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITtoggleDrawing(bool targetImage,
                                                         bool colorAdaptImage)
{
  SIT_drawTargetImage     = targetImage;
  SIT_drawColorAdaptImage = colorAdaptImage;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITrunTrack(
                                        Image<PixRGB<byte> > *image,
                                        typename
                                        std::vector<Image<FLOAT> > *input,
                                        bool editBlobs = true)
{
  SIT_LOTandRESET = false;
  // (1) segment the decimated image
  SIT_segment.SIsegment(image,input);
  LDEBUG("SEGMENTED");
  // (2) get center of mass for blobs
  SIT_segment.SIcalcMassCenter();
  LDEBUG("CALCULATED MASS CENTER");
  // (3) edit blobs, weed out all the non-hackers who are not
  // fit to carry a rifle
  if(editBlobs)
    SITanalyzeBlobs();
  LDEBUG("BLOBS ANALYZED");
  // (4) merge all remaining blobs
  SITmergeBlobs();
  LDEBUG("BLOBS MERGED");
  // (5) apply adaptive color thesholding
  SITcolorAdaptation();
  LDEBUG("COLORS ADAPTED");
}

/*********************************************************************/
// DATA RETURN METHODS
/*********************************************************************/

template SIT_TEMPLATE_CLASS
bool segmentImageTrackMC<SIT_TEMPLATE>::SITreturnLOT()
{
  return SIT_LOT;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
unsigned int segmentImageTrackMC<SIT_TEMPLATE>::SITreturnLOTtype()
{
  return SIT_LOTtype;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
std::string segmentImageTrackMC<SIT_TEMPLATE>::SITreturnLOTtypeName()
{
  return SIT_LOTtypeName;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
Image<long> segmentImageTrackMC<SIT_TEMPLATE>::SITreturnBlobMap()
{
  return SIT_segment.SIreturnBlobs();
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
Image<byte> segmentImageTrackMC<SIT_TEMPLATE>::SITreturnCandidateImage()
{
  return SIT_segment.SIreturnNormalizedCandidates();
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobPosition(int &x,
                                                           int &y)
{
      SIT_centerXmod = SIT_centerX*DEC;
      SIT_centerYmod = SIT_centerY*DEC;
  x = SIT_centerXmod;
  y = SIT_centerYmod;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobWeight(int &m)
{
  m = SIT_centerM*DEC;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetMinMaxBoundry(unsigned int *minX,
                                                            unsigned int *maxX,
                                                            unsigned int *minY,
                                                            unsigned int *maxY)
{
          SIT_minXmod = SIT_minX*DEC;
          SIT_maxXmod = SIT_maxX*DEC;
          SIT_minYmod = SIT_minY*DEC;
          SIT_maxYmod = SIT_maxY*DEC;
  *minX = SIT_minXmod;
  *maxX = SIT_maxXmod;
  *minY = SIT_minYmod;
  *maxY = SIT_maxYmod;
}

/********************************************************************/

/* commented this out because SIT_MASS is not defined (or even declared)
   anywhere, so we get an error with g++ 3.4.1 */
template SIT_TEMPLATE_CLASS
FLOAT segmentImageTrackMC<SIT_TEMPLATE>::SITgetMass()
{
  return SIT_mass;
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobAttrition(
                                                    INT *totalBlobs,
                                                    INT *killedBlobs)
{
  totalBlobs  = &SIT_totalBlobs;
  killedBlobs = &SIT_killedBlobs;
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetAdaptiveChannelVals(
                                        typename std::vector<FLOAT> *mean,
                                        typename std::vector<FLOAT> *std)
{
  mean = &SIT_chMean1;
  std  = &SIT_chStd1;
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
unsigned int segmentImageTrackMC<SIT_TEMPLATE>::SITgetLOTcount()
{
  return SIT_LOTcount;
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
INT segmentImageTrackMC<SIT_TEMPLATE>::SITnumberBlobs()
{
  return SIT_segment.SInumberBlobs();
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
unsigned short
segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobReasonForKillCode(INT blob)
{
  return SIT_reasonForKillCode[blob];
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
std::string segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobReasonForKill(INT blob)
{
  return SIT_reasonForKill[blob];
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
INT segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobMass(INT blob)
{
  return SIT_segment.SIgetMass(blob);
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
INT segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobPosX(INT blob)
{
  return (INT)floor(SIT_segment.SIgetCenterX(blob));
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
INT segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobPosY(INT blob)
{
  return (INT)floor(SIT_segment.SIgetCenterY(blob));
}


/*********************************************************************/
// DRAWING METHODS
/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITdrawBlobTrack(INT i)
{
  // find boundaries of this blob
  unsigned int tt = SIT_segment.SIgetYmin(i);
  unsigned int bb = SIT_segment.SIgetYmax(i);
  unsigned int ll = SIT_segment.SIgetXmin(i);
  unsigned int rr = SIT_segment.SIgetXmax(i);

  // draw bounding box for this blob
  // Note: box must be of height > 1 and width > 1
  if((bb != tt) && (ll != rr))
    drawRectEZ(*SIT_imageHold, Rectangle::tlbrI(tt*DEC,ll*DEC,bb*DEC,rr*DEC),
               PixRGB<byte>(SIT_boxRed,
                            SIT_boxGreen,
                            SIT_boxBlue),1);

  // draw target circle for this blob
  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_segment.SIgetCenterX(i)
                                     *DEC
                                     ,(int)SIT_segment.SIgetCenterY(i)*DEC)
             ,(int)sqrt((float)SIT_segment.SIgetMass(i)),
             PixRGB<byte>(SIT_circleRed,
                          SIT_circleGreen,
                          SIT_circleBlue),2);

  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_segment.SIgetCenterX(i)
                                     *DEC
                                     ,(int)SIT_segment.SIgetCenterY(i)*DEC)
             ,2,PixRGB<byte>(255,0,0),2);
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITdrawBlobTrackMerged()
{
  if((SIT_minY != SIT_maxY) && (SIT_minX != SIT_maxX))
    drawRectEZ(*SIT_imageHold, Rectangle::tlbrI(SIT_minY*DEC,SIT_minX*DEC,
                                         SIT_maxY*DEC,SIT_maxX*DEC),
               PixRGB<byte>(SIT_bigBoxRed,
                            SIT_bigBoxGreen,
                            SIT_bigBoxBlue),2);

  // draw target circle for this blob
  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_centerX*DEC,(int)SIT_centerY*DEC)
             ,(int)sqrt(SIT_mass),
             PixRGB<byte>(SIT_bigBoxRed,
                          SIT_bigBoxGreen,
                          SIT_bigBoxBlue),2);

  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_centerX*DEC,(int)SIT_centerY*DEC)
             ,2,PixRGB<byte>(255,0,0),2);
}



/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITdrawBlobBox(INT i)
{
  // find boundaries of this blob
  unsigned int tt = SIT_segment.SIgetYmin(i);
  unsigned int bb = SIT_segment.SIgetYmax(i);
  unsigned int ll = SIT_segment.SIgetXmin(i);
  unsigned int rr = SIT_segment.SIgetXmax(i);


  // draw bounding box for this blob
  // Note: box must be of height > 1 and width > 1
  if((bb != tt) && (ll != rr))
    drawRectEZ(*SIT_imageHold, Rectangle::tlbrI(tt*DEC,ll*DEC,bb*DEC,rr*DEC),
               PixRGB<byte>(SIT_circleRed,
                            SIT_circleGreen,
                            SIT_circleBlue),1);
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITdrawHistoValues(
                                           typename std::vector<FLOAT> *mean,
                                           typename std::vector<FLOAT> *std,
                                           typename std::vector<FLOAT> *lb,
                                           typename std::vector<FLOAT> *ub,
                                           typename std::vector<FLOAT> *norm,
                                           bool LOT)
{
  unsigned int minWidth = SIT_barWidth+SIT_barSpace*mean->size();
  if((unsigned)SIT_auxHold->getWidth() < minWidth)
    SIT_auxHold->resize(minWidth,(int)SIT_histoHeight,true);
  // draw background grid in HSV bar graph
  if(LOT == false)
    drawGrid(*SIT_auxHold, 25,25,1,1,PixRGB<byte>(100,100,100));
  else
    drawGrid(*SIT_auxHold, 25,25,1,1,PixRGB<byte>(200,100,100));

  typename std::vector<FLOAT>::iterator imean = mean->begin();
  typename std::vector<FLOAT>::iterator istd  = std->begin();
  typename std::vector<FLOAT>::iterator ilb   = lb->begin();
  typename std::vector<FLOAT>::iterator iub   = ub->begin();
  typename std::vector<FLOAT>::iterator inorm = norm->begin();

  unsigned int start = SIT_barSpace;
  unsigned int end   = SIT_barWidth + start;

  while(imean != mean->end())
  {
    FLOAT mean = (*imean/(*inorm));
    FLOAT std  = (*istd/(*inorm));
    FLOAT ub   = (*iub/(*inorm));
    FLOAT lb   = (*ilb/(*inorm));

    // draw HSV mean value bars as a bunch of rectangles
    drawRectEZ(*SIT_auxHold, Rectangle::tlbrI(0,start,(int)(mean*
                                                     SIT_histoHeight),end),
               PixRGB<byte>(0,0,255),1);

    // draw standard deviation bars

    if((((mean-std)*SIT_histoHeight) > 0)
       && (((mean+std)*SIT_histoHeight) < SIT_auxHold->getHeight()))
      drawRectEZ(*SIT_auxHold, Rectangle::tlbrI((int)((mean-std)*SIT_histoHeight)
                                         ,start+2,
                                         (int)((mean+std)*SIT_histoHeight)
                                         ,end-2),
                 PixRGB<byte>(255,255,0),1);


    drawRectEZ(*SIT_auxHold, Rectangle::tlbrI((int)(lb*(SIT_histoHeight-1))
                                       ,(signed)start-3,
                                       (int)(ub*(SIT_histoHeight-1))
                                       ,(signed)start-1),
               PixRGB<byte>(255,0,0),1);


    ++imean, ++istd, ++ilb, ++iub, ++inorm;
    start = end   + SIT_barSpace;
    end   = start + SIT_barWidth;
  }
}

#undef SIT_TEMPLATE_CLASS
#undef SIT_TEMPLATE

template class segmentImageTrackMC<float, unsigned int, 3>;
template class segmentImageTrackMC<float, unsigned int, 4>;

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
