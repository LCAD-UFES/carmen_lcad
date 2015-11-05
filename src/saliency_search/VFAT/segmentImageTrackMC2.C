/*!@file VFAT/segmentImageTrackMC2.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageTrackMC2.C $
// $Id: segmentImageTrackMC2.C 14376 2011-01-11 02:44:34Z pez $
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

#ifndef SEGMENTIMAGETRACKMC2_C_DEFINED
#define SEGMENTIMAGETRACKMC2_C_DEFINED

#include "Util/Assert.H"
#include "VFAT/segmentImageTrackMC2.H"
#include "Image/DrawOps.H"
#include "Raster/Raster.H"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <time.h>


//! maximum lose of tracks before color is reset
#define LOTMAX            5

//! How many iteration to calculate over for movement statistics
#define ERRINTERVAL       5

//! decimation size reduction factor
//#define DEC               2

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
  SIT_totalLifeSpan++;
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
      SIT_lifeSpan++;
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
        if(SIT_useHardBounds == true)
        {
          if(*imean1 > *iub)
          {
            *imean1 = *iub;
          }
          else if(*imean1 < *ilb)
          {
            *imean1 = *ilb;
          }
        }
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
      SIT_segment.SIsetVal(SIT_chMean1,SIT_chNSTD);
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
  SIT_segment.SIsetVal(SIT_initMean,SIT_initStd);

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
  SIT_LOTcount      = 0;
  SIT_totalLifeSpan = 0;
  SIT_lifeSpan      = 0;
  SIT_resetColor    = true;
  SIT_LOTandRESET   = true;
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

    // (3) check if a blobs mass is within cosnstraints. That is
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
        }
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
  // changed from UINT_MAX to std::numeric_limits<unsigned
  // short>::max() to avoid "warning: large integer implicitly
  // truncated to unsigned type" from g++ 4.1
  SIT_minX    = std::numeric_limits<unsigned short>::max();
  SIT_minY    = std::numeric_limits<unsigned short>::max();
  SIT_maxX    = 0;
  SIT_maxY    = 0;

  // calculate the center of a combined blob from the average
  // mass center of all remaining blobs
  std::vector<bool>::iterator candidateBlobItr = SIT_candidateBlob.begin();

  for(INT i = 0; i < SIT_segment.SInumberBlobs(); i++, ++candidateBlobItr)
  {
    if(*candidateBlobItr == true)
    {
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
  }

  // Compute SIT_LOT here and set
  if(SIT_mass != 0)
  {
    SIT_centerX = (int)(meanX/SIT_mass);
    SIT_centerY = (int)(meanY/SIT_mass);

    if(((SIT_maxX-SIT_minX)*(SIT_maxY-SIT_minY)) >
       ((SIT_segment.SIgetImageSizeX()*SIT_segment.SIgetImageSizeY())
        /SIT_blobProp.BP_maxFrameSize))
    {
      SIT_LOT         = true;
      SIT_LOTtype     = 1;
      SIT_LOTtypeName = "(1) Postive Mass but Spacially too Large";
    }
    else
    {
      SIT_LOT                 = false;
      SIT_LOTtypeName         = "(0) NO LOT, OK";
      SIT_useExpectedLocation = false;
    }
  }
  else
  {
    SIT_LOTtypeName = "(2) Zero Mass in merged blobs";
    SIT_LOTtype = 2;
    SIT_LOT     = true;

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
  SIT_editBlobs           = true;
  SIT_useHardBounds       = false;

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
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetCircleColor(unsigned char r,
                                                          unsigned char g,
                                                          unsigned char b)
{
  SIT_circleRed      = r;
  SIT_circleBlue     = g;
  SIT_circleGreen    = b;
  SIT_didCircleColor = true;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetBoxColor(unsigned char r,
                                                       unsigned char g,
                                                       unsigned char b,
                                                       unsigned char bigr,
                                                       unsigned char bigg,
                                                       unsigned char bigb)
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
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetTrackSignature(
                                    const covHolder<double> *cov,
                                    segHolder<FLOAT,INT,SIT_channels>  *seg)
{
  std::vector<unsigned short>::iterator channelMap = seg->channelMap.begin();
  typename std::vector<FLOAT>::iterator initMean = SIT_initMean.begin();
  typename std::vector<FLOAT>::iterator initStd  = SIT_initStd.begin();
  typename std::vector<FLOAT>::iterator chUB     = SIT_chUB.begin();
  typename std::vector<FLOAT>::iterator chLB     = SIT_chLB.begin();
  typename std::vector<FLOAT>::iterator norm     = SIT_chNorm.begin();
  typename std::vector<FLOAT>::iterator STDmod   = seg->STDmod.begin();
  typename std::vector<FLOAT>::iterator UBMod    = seg->upperBoundMod.begin();
  typename std::vector<FLOAT>::iterator LBMod    = seg->lowerBoundMod.begin();

  std::ofstream outfile1(SIT_LOG_FILE,std::ios::app);

  for(unsigned int i = 0; i < seg->dim; i++, ++channelMap,
        ++initMean, ++initStd, ++chUB, ++chLB, ++norm,
        ++STDmod, ++UBMod, ++LBMod)
  {
    const unsigned int cmap = *channelMap;
    const FLOAT covMean     = cov->mean[cmap];
    const FLOAT covBias     = cov->bias[cmap];

    // Set mean value on this channel from covHolder
    *initMean = covMean / covBias;
    // Set standard deviation on this channel from covHolder
    *initStd  = (cov->STD[cmap] * (1/covBias)) * (*STDmod);
    // Set upperBound on this channel from covHolder
    *chUB     = covMean / covBias + (*UBMod);
    // Set lowerBount on this channel from covHolder
    *chLB     = covMean / covBias - (*LBMod);
    // set the norm from this channel from covHolder (in feature.conf)
    *norm     = cov->norm[cmap];
    if(SIT_useLog == true)
    {
      outfile1 << "Setting channel segHolder " << seg->baseID
               << " via covHolder " << cov->baseID
               << " - Matching "    << seg->featureName[i]
               << " To "            << cov->featureName[cmap]
               << "\t- Value: "      << covMean
               << "  STD: "         << cov->STD[cmap]
               << "  Bias: "        << covBias
               << "\n";
    }
  }

  outfile1.close();


  SIT_chAdapt = seg->channelAdapt;
  // reset the frame to the center
  SITsetFrame(&seg->imageSizeX,&seg->imageSizeY);
  // where should the target appear?
  seg->expectedX    = cov->posX; seg->expectedY    = cov->posY;
  seg->expectedXmin = cov->minX; seg->expectedYmin = cov->minY;
  seg->expectedXmax = cov->maxX; seg->expectedYmax = cov->maxY;

  SITsetExpectedTargetPosition(cov->posX, cov->posY, cov->maxX,
                               cov->maxY, cov->minX, cov->minY);

  // reset some values we need to reset when uploading new sig.
  SIT_thresh         = 1;
  SIT_didTrackColor  = true;
  SIT_resetColor     = true;
  SIT_LOTcount       = 0;
  SIT_totalLifeSpan  = 0;
  SIT_lifeSpan       = 0;
  seg->LOTcount      = SIT_LOTcount;
  seg->totalLifeSpan = SIT_totalLifeSpan;
  seg->lifeSpan      = SIT_lifeSpan;

  SIT_segment.SIsetAvg(ERRINTERVAL);
  SIT_segment.SIresetAvg();
  SIT_segment.SIsetVal(SIT_initMean,SIT_initStd);
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
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetTrackSignature(
                                    segHolder<FLOAT,INT,SIT_channels>  *seg)
{
  SITgetBlobPosition(&(seg->posX),&(seg->posY));
  SITgetMinMaxBoundry(&(seg->minX),&(seg->maxX),&(seg->minY),&(seg->maxY));
  SITgetAdaptiveChannelVals(&(seg->mean),&(seg->STD));
  SITgetBlobAttrition(&(seg->blobNumber),&(seg->killedBlobs));
  SITgetLifeSpans(&(seg->totalLifeSpan),&(seg->lifeSpan));
  seg->mass        = (unsigned int)ceil(SITgetMass());
  seg->LOT         = SITreturnLOT();
  seg->LOTtype     = SITreturnLOTtype();
  seg->LOTtypeName = SITreturnLOTtypeName();
  seg->LOTcount    = SITgetLOTcount();
  seg->LOTandReset = SIT_LOTandRESET;
  seg->boundaryX   = SIT_xBound * SIT_GLOBAL_DEC;
  seg->boundaryY   = SIT_yBound * SIT_GLOBAL_DEC;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetTrackColor(
                                    typename std::vector<FLOAT> color,
                                    typename std::vector<FLOAT> std,
                                    typename std::vector<FLOAT> norm,
                                    typename std::vector<FLOAT> adapt,
                                    typename std::vector<FLOAT> upperBound,
                                    typename std::vector<FLOAT> lowerBound,
                                    bool resetColor,
                                    bool resetCandidates)
{
  // make sure that all our contaners are the same size
  ASSERT(color.size()      == std.size());
  ASSERT(std.size()        == norm.size());
  ASSERT(norm.size()       == adapt.size());
  ASSERT(adapt.size()      == upperBound.size());
  ASSERT(upperBound.size() == lowerBound.size());

  // set initial tracking parameters
  SIT_thresh        = 1;
  SIT_initMean      = color;
  SIT_initStd       = std;
  SIT_chNorm        = norm;
  SIT_chAdapt       = adapt;
  SIT_chUB          = upperBound;
  SIT_chLB          = lowerBound;
  SIT_didTrackColor = true;
  SIT_totalLifeSpan = 0;
  SIT_lifeSpan      = 0;
  SIT_LOTcount      = 0;
  SIT_segment.SIsetAvg(ERRINTERVAL);
  SIT_segment.SIresetAvg();
  SIT_segment.SIsetVal(SIT_initMean,SIT_initStd);

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
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetFrame(unsigned short *x,
                                                    unsigned short *y)
{
  int xx = *x / SIT_GLOBAL_DEC; int yy = *y / SIT_GLOBAL_DEC;
  SIT_segment.SIsetFrame(&xx,&yy);
  SIT_x_center = xx/2;
  SIT_y_center = yy/2;
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
                                              const unsigned short posX,
                                              const unsigned short posY,
                                              const unsigned short maxX,
                                              const unsigned short maxY,
                                              const unsigned short minX,
                                              const unsigned short minY)
{
  SIT_expectedX           = posX / SIT_GLOBAL_DEC;
  SIT_expectedY           = posY / SIT_GLOBAL_DEC;
  SIT_expectedXmax        = maxX / SIT_GLOBAL_DEC;
  SIT_expectedYmax        = maxY / SIT_GLOBAL_DEC;
  SIT_expectedXmin        = minX / SIT_GLOBAL_DEC;
  SIT_expectedYmin        = minY / SIT_GLOBAL_DEC;
  SIT_useExpectedLocation = true;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetExpectedTargetPosition(
                                              unsigned short *posX,
                                              unsigned short *posY,
                                              unsigned short *maxX,
                                              unsigned short *maxY,
                                              unsigned short *minX,
                                              unsigned short *minY,
                                              bool *isSet) const
{
  unsigned short pX  = SIT_expectedX    * SIT_GLOBAL_DEC;
  unsigned short pY  = SIT_expectedY    * SIT_GLOBAL_DEC;
  unsigned short XM  = SIT_expectedXmax * SIT_GLOBAL_DEC;
  unsigned short YM  = SIT_expectedYmax * SIT_GLOBAL_DEC;
  unsigned short xM  = SIT_expectedXmin * SIT_GLOBAL_DEC;
  unsigned short yM  = SIT_expectedYmin * SIT_GLOBAL_DEC;
  bool is            = SIT_useExpectedLocation;

  *posX  = pX; *posY  = pY;
  *maxX  = XM; *maxY  = YM;
  *minX  = xM; *minY  = yM;

  *isSet = is;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITunsetExpectedTargetPosition()
{
  SIT_useExpectedLocation = false;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
bool segmentImageTrackMC<SIT_TEMPLATE>::SITintersection(
                                              const covHolder<double> *cov,
                                              const int boundX,
                                              const int boundY) const
{
  int maxX,maxY,minX,minY;
  bool Xok = false;
  bool Yok = false;

  // what are the bounds of this object?
  // store and multiply by decimation reduction factor
  if(SIT_lifeSpan > 1)
  {
    minX = (signed)SIT_minX*SIT_GLOBAL_DEC;
    maxX = (signed)SIT_maxX*SIT_GLOBAL_DEC;
    minY = (signed)SIT_minY*SIT_GLOBAL_DEC;
    maxY = (signed)SIT_maxY*SIT_GLOBAL_DEC;
  }
  else
  {
    minX = (signed)SIT_expectedXmin*SIT_GLOBAL_DEC;
    maxX = (signed)SIT_expectedXmax*SIT_GLOBAL_DEC;
    minY = (signed)SIT_expectedYmin*SIT_GLOBAL_DEC;
    maxY = (signed)SIT_expectedYmax*SIT_GLOBAL_DEC;
  }

  // check for intersection with other objects
  if((((signed)cov->minX < minX-boundX) &&
      ((signed)cov->maxX < minX-boundX)) ||
     (((signed)cov->minX > maxX+boundX) &&
      ((signed)cov->maxX > maxX+boundX)))
  {
    Xok = true;
  }

  if((((signed)cov->minY < minY-boundY) &&
      ((signed)cov->maxY < minY-boundY)) ||
     (((signed)cov->minY > maxY+boundY) &&
      ((signed)cov->maxY > maxY+boundY)))
  {
    Yok = true;
  }

  if((Xok == false) && (Yok == false))
  {
    return false;
  }
  // else
  return true;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITpercentOverlap(
                           const segHolder<FLOAT,INT,SIT_channels>  *seg,
                           FLOAT *overlap,
                           FLOAT *thisArea,
                           FLOAT *otherArea) const
{
  const int minX = (signed)SIT_minX*SIT_GLOBAL_DEC;
  const int maxX = (signed)SIT_maxX*SIT_GLOBAL_DEC;
  const int minY = (signed)SIT_minY*SIT_GLOBAL_DEC;
  const int maxY = (signed)SIT_maxY*SIT_GLOBAL_DEC;

  // Find which parts overlap, if any
  bool olapX = false;
  bool olapY = false;
  bool sMinX = false;  bool sMaxX = false;
  bool sMinY = false;  bool sMaxY = false;

  // What are the intersections?
  if((seg->minX > minX) && (seg->minX < maxX))
    { sMinX = true; olapX = true;}
  if((seg->maxX > minX) && (seg->maxX < maxX))
    { sMaxX = true; olapX = true;}
  if((seg->minY > minY) && (seg->minY < maxY))
    { sMinY = true; olapY = true;}
  if((seg->maxY > minY) && (seg->maxY < maxY))
    { sMaxY = true; olapY = true;}

  // If intersection, computer percent overlap
  // else no intersection, return 0
  if(olapX && olapY)
  {
    // else find the area of the overlap, then the percentage

    const FLOAT slenX = seg->maxX - seg->minX;
    const FLOAT slenY = seg->maxY - seg->minY;
    *otherArea = slenX*slenY;

    const FLOAT lenX = maxX - minX;
    const FLOAT lenY = maxY - minY;
    *thisArea = lenX*lenY;

    // average the area. This forces the overlap to be a function
    // of the similarity if the two targets sizes.

    const FLOAT avgArea = ((*thisArea)+(*otherArea))/2;

    FLOAT diffX = 0.0F; FLOAT diffY = 0.0F;
    if(     (sMinX == true)  && (sMaxX == false))
      diffX = maxX      - seg->minX;
    else if((sMinX == false) && (sMaxX == true))
      diffX = seg->maxX - minX;
    else
      diffX = slenX;

    if(     (sMinY == true)  && (sMaxY == false))
      diffY = maxY      - seg->minY;
    else if((sMinY == false) && (sMaxY == true))
      diffY = seg->maxY - minY;
    else
      diffY = slenY;

    const FLOAT oarea = diffX*diffY;
    *overlap = oarea/avgArea;
    return;
  }

  *overlap = 0.0F;
  return;
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
void segmentImageTrackMC<SIT_TEMPLATE>::SITtoggleHardBounds(bool toggle)
{
  SIT_useHardBounds = toggle;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITsetFrameNumber(unsigned long frame)
{
  SIT_frameNumber = frame;
}

/*********************************************************************/
template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITeditBlobs(bool toggle)
{
  SIT_editBlobs = toggle;
}

/*********************************************************************/
// MAIN METHOD
/*********************************************************************/

/* This is a basic tracker access method that tracks on one image at a time */
/* this one is designed to work with any Pixels pixel */

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITtrackImage
                               (Image<PixH2SV2<FLOAT> > &input,
                                Image<PixRGB<byte> > *output)
{
  // Assert that parameters have been set up before starting
  ASSERT(SIT_didCircleColor == true);
  ASSERT(SIT_didBoxColor    == true);
  ASSERT(SIT_didTrackColor  == true);

  SIT_imageHold = output;

  // decimate input image twice to speed things up
  if(SIT_GLOBAL_DEC >= 2)
    input         = decXY(input);
  if(SIT_GLOBAL_DEC >= 4)
    input         = decXY(input);
  if(SIT_GLOBAL_DEC >= 8)
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

  SITrunTrack(SIT_imageHold,&SIT_chans);
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
                                        std::vector<Image<FLOAT> > *input)
{
  SIT_LOTandRESET  = false;
  SIT_didColorBars = false;
  // (1) segment the decimated image
  SIT_segment.SIsegment(image,input);
  LDEBUG("SEGMENTED");
  // (2) get center of mass for blobs
  SIT_segment.SIcalcMassCenter();
  LDEBUG("CALCULATED MASS CENTER");
  // (3) edit blobs, weed out all the non-hackers who are not
  // fit to carry a rifle
  if(SIT_editBlobs)
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
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobPosition(unsigned short *x,
                                                           unsigned short *y)
{
       SIT_centerXmod = SIT_centerX*SIT_GLOBAL_DEC;
       SIT_centerYmod = SIT_centerY*SIT_GLOBAL_DEC;
  *x = SIT_centerXmod;
  *y = SIT_centerYmod;
}
/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetMinMaxBoundry(
                                                        unsigned short *minX,
                                                        unsigned short *maxX,
                                                        unsigned short *minY,
                                                        unsigned short *maxY)
{
          SIT_minXmod = SIT_minX*SIT_GLOBAL_DEC;
          SIT_maxXmod = SIT_maxX*SIT_GLOBAL_DEC;
          SIT_minYmod = SIT_minY*SIT_GLOBAL_DEC;
          SIT_maxYmod = SIT_maxY*SIT_GLOBAL_DEC;
  *minX = SIT_minXmod;
  *maxX = SIT_maxXmod;
  *minY = SIT_minYmod;
  *maxY = SIT_maxYmod;
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
Image<PixRGB<byte> > segmentImageTrackMC<SIT_TEMPLATE>::SITgetColorBars()
{
  return SIT_auxHold;
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
  *totalBlobs  = SIT_totalBlobs;
  *killedBlobs = SIT_killedBlobs;
}
/********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetLifeSpans(
                                        unsigned long *totalLifeSpan,
                                        unsigned long *lifeSpan)
{
  *totalLifeSpan = SIT_totalLifeSpan;
  *lifeSpan      = SIT_lifeSpan;
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITgetAdaptiveChannelVals(
                                        typename std::vector<FLOAT> *mean,
                                        typename std::vector<FLOAT> *std)
{
  *mean = SIT_chMean1;
  *std  = SIT_chStd1;
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
  return (INT)floor(SIT_segment.SIgetCenterX(blob)) * SIT_GLOBAL_DEC;
}

/********************************************************************/

template SIT_TEMPLATE_CLASS
INT segmentImageTrackMC<SIT_TEMPLATE>::SITgetBlobPosY(INT blob)
{
  return (INT)floor(SIT_segment.SIgetCenterY(blob)) * SIT_GLOBAL_DEC;
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
    drawRectEZ(*SIT_imageHold, Rectangle::tlbrI(tt*SIT_GLOBAL_DEC,
                                         ll*SIT_GLOBAL_DEC,
                                         bb*SIT_GLOBAL_DEC,
                                         rr*SIT_GLOBAL_DEC),
               PixRGB<byte>(SIT_boxRed,
                            SIT_boxGreen,
                            SIT_boxBlue),1);

  // draw target circle for this blob
  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_segment.SIgetCenterX(i)
                                     *SIT_GLOBAL_DEC
                                     ,(int)SIT_segment.SIgetCenterY(i)
                                     *SIT_GLOBAL_DEC)
             ,(int)sqrt((float)SIT_segment.SIgetMass(i)),
             PixRGB<byte>(SIT_boxRed,
                          SIT_boxGreen,
                          SIT_boxBlue),2);

  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_segment.SIgetCenterX(i)
                                     *SIT_GLOBAL_DEC
                                     ,(int)SIT_segment.SIgetCenterY(i)
                                     *SIT_GLOBAL_DEC)
             ,2,PixRGB<byte>(255,0,0),2);
}

/*********************************************************************/

template SIT_TEMPLATE_CLASS
void segmentImageTrackMC<SIT_TEMPLATE>::SITdrawBlobTrackMerged()
{

  if((SIT_minY != SIT_maxY) && (SIT_minX != SIT_maxX))
    drawRectEZ(*SIT_imageHold, Rectangle::tlbrI(SIT_minY*SIT_GLOBAL_DEC,
                                         SIT_minX*SIT_GLOBAL_DEC,
                                         SIT_maxY*SIT_GLOBAL_DEC,
                                         SIT_maxX*SIT_GLOBAL_DEC),
               PixRGB<byte>(SIT_bigBoxRed,
                            SIT_bigBoxGreen,
                            SIT_bigBoxBlue),2);

  // draw target circle for this blob
  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_centerX*SIT_GLOBAL_DEC,
                                     (int)SIT_centerY*SIT_GLOBAL_DEC),
             (int)sqrt(SIT_mass),
             PixRGB<byte>(SIT_bigBoxRed,
                          SIT_bigBoxGreen,
                          SIT_bigBoxBlue),2);

  drawCircle(*SIT_imageHold, Point2D<int>((int)SIT_centerX*SIT_GLOBAL_DEC,(int)SIT_centerY*SIT_GLOBAL_DEC)
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
    drawRectEZ(*SIT_imageHold, Rectangle::tlbrI(tt*SIT_GLOBAL_DEC,
                                         ll*SIT_GLOBAL_DEC,
                                         bb*SIT_GLOBAL_DEC,
                                         rr*SIT_GLOBAL_DEC),
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
  SIT_didColorBars = true;

  unsigned int minWidth = SIT_barWidth+SIT_barSpace*mean->size();
  if((unsigned)SIT_auxHold.getWidth() < minWidth)
    SIT_auxHold.resize(minWidth,(int)SIT_histoHeight,true);
  // draw background grid in HSV bar graph
  if(LOT == false)
    drawGrid(SIT_auxHold, 25,25,1,1,PixRGB<byte>(100,100,100));
  else
    drawGrid(SIT_auxHold, 25,25,1,1,PixRGB<byte>(200,100,100));

/*  typename std::vector<FLOAT>::iterator imean = mean->begin();
  typename std::vector<FLOAT>::iterator istd  = std->begin();
  typename std::vector<FLOAT>::iterator ilb   = lb->begin();
  typename std::vector<FLOAT>::iterator iub   = ub->begin();
  typename std::vector<FLOAT>::iterator inorm = norm->begin();
*/
  //unsigned int start = SIT_barSpace;
  //unsigned int end   = SIT_barWidth + start;

  /*
  while(imean != mean->end())
  {
    if(*inorm != 0)
    {
      FLOAT mean = (*imean/(*inorm));
      FLOAT std  = (*istd/(*inorm));
      FLOAT ub   = (*iub/(*inorm));
      FLOAT lb   = (*ilb/(*inorm));

      // draw HSV mean value bars as a bunch of rectangles
      drawRectEZ(SIT_auxHold, Rectangle::tlbrI(0,start,(int)(mean*
                                                      SIT_histoHeight),end),
                 PixRGB<byte>(0,0,255),1);

      // draw standard deviation bars

      if((((mean-std)*SIT_histoHeight) > 0)
         && (((mean+std)*SIT_histoHeight) < SIT_auxHold.getHeight()))
        drawRectEZ(SIT_auxHold, Rectangle::tlbrI((int)((mean-std)*SIT_histoHeight)
                                        ,start+2,
                                          (int)((mean+std)*SIT_histoHeight)
                                          ,end-2),
                   PixRGB<byte>(255,255,0),1);


      drawRectEZ(SIT_auxHold, Rectangle::tlbrI((int)(lb*(SIT_histoHeight-1))
                                        ,(signed)start-3,
                                        (int)(ub*(SIT_histoHeight-1))
                                        ,(signed)start-1),
                 PixRGB<byte>(255,0,0),1);


      ++imean, ++istd, ++ilb, ++iub, ++inorm;
      start = end   + SIT_barSpace;
      end   = start + SIT_barWidth;
    }
  }
  */
}

#undef SIT_TEMPLATE_CLASS
#undef SIT_TEMPLATE

template class segmentImageTrackMC<float, unsigned int, 3>;
template class segmentImageTrackMC<float, unsigned int, 4>;

#endif
