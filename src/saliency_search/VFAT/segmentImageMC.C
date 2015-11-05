/*!@file VFAT/segmentImageMC.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageMC.C $
// $Id: segmentImageMC.C 6006 2005-11-29 22:34:22Z rjpeters $
//

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itt itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#ifndef SEGMENTIMAGEMC_C_DEFINED
#define SEGMENTIMAGEMC_C_DEFINED

#include "Util/Assert.H"
#include "VFAT/segmentImageMC.H"
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>


/*********************************************************************/

/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired feature values
   if so, set candidatePixels image pixel to true
*/
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIfindCandidates()
{
  // FIX
  std::vector<long>::iterator imasterVec = SI_masterVec.begin();

  if(SI_maxIDVal != 0)
    while(imasterVec != SI_masterVec.end())
      *imasterVec++ = -1;
  else
    for(INT i = 0; i < SI_maxIDVal; i++)
      *imasterVec++ = -1;

  Image<bool>::iterator
    candidatePixelsIter    = SI_candidatePixels.beginw();

  Image<bool>::iterator
    preCandidatePixelsIter = SI_preCandidatePixels.beginw();

  // set initial candidates for each pixel

  while(candidatePixelsIter != SI_candidatePixels.endw())
  {
    if((*preCandidatePixelsIter) == true)
    {
      *candidatePixelsIter = true;
    }
    else
    {
      *candidatePixelsIter = false;
    }
    *preCandidatePixelsIter = true;
    ++candidatePixelsIter; ++preCandidatePixelsIter;
  }

  typename std::vector<Image<FLOAT> >::iterator ifeatureMaps;
  typename std::vector<FLOAT>::iterator ilowThresh  = SI_lowThresh.begin();
  typename std::vector<FLOAT>::iterator ihighThresh = SI_highThresh.begin();
  // Run over each feature channel

  for(ifeatureMaps = SI_featureMaps->begin();
      ifeatureMaps !=  SI_featureMaps->end(); ++ifeatureMaps,
        ++ilowThresh, ++ihighThresh)
  {
    typename Image<FLOAT>::iterator iifeatureMaps = ifeatureMaps->beginw();
    preCandidatePixelsIter = SI_preCandidatePixels.beginw();
    candidatePixelsIter    = SI_candidatePixels.beginw();

    // Run over each pixel

    while(iifeatureMaps != ifeatureMaps->endw())
    {
      if((*iifeatureMaps > *ihighThresh) || (*iifeatureMaps < *ilowThresh))
      {
        *preCandidatePixelsIter = false;
        *candidatePixelsIter    = false;
      }
      //else
        //LINFO("VAL %f",*iifeatureMaps);
      ++preCandidatePixelsIter; ++candidatePixelsIter; ++iifeatureMaps;
    }
  }
}

/*********************************************************************/

/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired feature values
   if so, set candidatePixels image pixel to true
*/
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIfindCandidatesNoBandPass()
{
  // FIX
  std::vector<long>::iterator imasterVec = SI_masterVec.begin();

  if(SI_maxIDVal != 0)
    while(imasterVec != SI_masterVec.end())
      *imasterVec++ = -1;
  else
    for(INT i = 0; i < SI_maxIDVal; i++)
      *imasterVec++ = -1;

  Image<bool>::iterator
    candidatePixelsIter    = SI_candidatePixels.beginw();

  // set initial candidates for each pixel

  while(candidatePixelsIter != SI_candidatePixels.endw())
  {
    *candidatePixelsIter = true;
    ++candidatePixelsIter;
  }

  typename std::vector<Image<FLOAT> >::iterator ifeatureMaps;
  typename std::vector<FLOAT>::iterator ilowThresh  = SI_lowThresh.begin();
  typename std::vector<FLOAT>::iterator ihighThresh = SI_highThresh.begin();
  // Run over each feature channel

  for(ifeatureMaps = SI_featureMaps->begin();
      ifeatureMaps !=  SI_featureMaps->end(); ++ifeatureMaps,
        ++ilowThresh, ++ihighThresh)
  {
    typename Image<FLOAT>::iterator iifeatureMaps = ifeatureMaps->beginw();
    candidatePixelsIter = SI_candidatePixels.beginw();

    // Run over each pixel

    while(iifeatureMaps != ifeatureMaps->endw())
    {
      if((*iifeatureMaps > *ihighThresh) || (*iifeatureMaps < *ilowThresh))
      {
        *candidatePixelsIter = false;
      }
      ++candidatePixelsIter; ++iifeatureMaps;
    }
  }
}

/*********************************************************************/
/* if a pixel does not have an orthogonal neighbor then it is removed
   i.e. all pixels without 4 connectivity are removed as noise
*/
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIremoveSingles()
{
  Image<bool>::iterator icandidatePixels = SI_candidatePixels.beginw();

  const int width  = SI_candidatePixels.getWidth();
  const int height = SI_candidatePixels.getHeight();
  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      if(*icandidatePixels)
      {
        int kill = 0;
        const int XLeft   = x - 1;
        const int XRight  = x + 1;
        const int YTop    = y - 1;
        const int YBottom = y + 1;
        if((XLeft >= 0) && (SI_candidatePixels.getVal(XLeft,y)))
          kill++;
        if((XRight < width)
           && (SI_candidatePixels.getVal(XRight,y)))
          kill++;
        if((YTop >= 0) && (SI_candidatePixels.getVal(x,YTop)))
          kill++;
        if((YBottom < height)
           && (SI_candidatePixels.getVal(x,YBottom)))
          kill++;
        if(kill < 2)
          *icandidatePixels = false;
      }
      ++icandidatePixels;
    }
  }
}

/*********************************************************************/
/* if a pixel does not have an orthogonal neighbor then it is removed
   i.e. all pixels without 4 connectivity are removed as noise
*/
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIremoveSinglesItr()
{
  Image<bool>::iterator icandidatePixels = SI_candidatePixels.beginw();

  const unsigned int width  = SI_candidatePixels.getWidth();
  const unsigned int height = SI_candidatePixels.getHeight();

  // hypothetically, this could cause a segmentation fault, but
  // we don't access this until the memory is aligned so we are OK
  Image<bool>::iterator icandidatePixelsTop    =
    SI_candidatePixels.beginw() - width;
  Image<bool>::iterator icandidatePixelsBottom =
    SI_candidatePixels.beginw() + width;
  Image<bool>::iterator icandidatePixelsLeft   =
    SI_candidatePixels.beginw() - 1;
  Image<bool>::iterator icandidatePixelsRight  =
    SI_candidatePixels.beginw() + 1;

  for(unsigned int y = 0; y < height; y++)
  {
    for(unsigned int x = 0; x < width; x++)
    {
      if(*icandidatePixels)
      {
        unsigned int kill = 0;
        if((x != 0)     && (*icandidatePixelsLeft))
          kill++;
        if((x < width)  && (*icandidatePixelsRight))
          kill++;
        if((y != 0)     && (*icandidatePixelsTop))
          kill++;
        if((y < height) && (*icandidatePixelsBottom))
          kill++;
        if(kill < 2)
          *icandidatePixels = false;
      }
      ++icandidatePixels;    ++icandidatePixelsBottom; ++icandidatePixelsRight;
      ++icandidatePixelsTop; ++icandidatePixelsLeft;
    }
  }
}

/*********************************************************************/
/* scan pixels from top to bottom/left to right. Link all pixels in the
   same blob. Do this using recursive back tracking to make a blob linked
   as one object and not a collection of objects
*/
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIdiscreteLinking()
{
  const int width       = SI_candidatePixels.getWidth();
  const int height      = SI_candidatePixels.getHeight();
  bool trace            = false;
  INT pixID             = 0;
  SI_maxIDVal           = 0;
  SI_masters            = 0;
  SI_mastersCount       = 0;
  long lastNeighbor;

  for(int x = 0; x < width; x++)
  {
    trace        = false;
    lastNeighbor = -2;
    for(int y = 0; y < height; y++)
    {
      if(SI_candidatePixels.getVal(x,y))
      {
        if(!trace)
        {
          pixID++;
          trace        = true;
          lastNeighbor = -2;
        }
        SI_blobID.setVal(x,y,pixID);
        if(x > 0) // 1 or 0?
        {
          if(SI_candidatePixels.getVal(x-1,y))
          {
            // relink blob pixels if needed
            INT check =  SI_blobID.getVal((x-1),y);
            if((signed)check != lastNeighbor)
            {
              SIbackwardLink(&pixID,&check);
              lastNeighbor = check;
            }
          }
          else
          {
            lastNeighbor = -2;
          }
        }
        else
        {
          SIbackwardLink(&pixID,&pixID);
        }
      }
      else
      {
        trace = false;
        SI_blobID.setVal(x,y,0);
        lastNeighbor = -2;
      }
    }
  }
  //LINFO("Candidate pixels recognized %d",candi);
  SI_num = pixID;
}

/*********************************************************************/
/* scan pixels from top to bottom/left to right. Link all pixels in the
   same blob. Do this using recursive back tracking to make a blob linked
   as one object and not a collection of objects
*/
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIdiscreteLinkingOrtho()
{
  const unsigned int width       = (unsigned)SI_candidatePixels.getWidth();
  const unsigned int height      = (unsigned)SI_candidatePixels.getHeight();
  bool trace                     = false;
  INT pixID                      = 0;
  SI_maxIDVal                    = 0;
  SI_masters                     = 0;
  SI_mastersCount                = 0;
  long lastNeighbor;
  Image<bool>::iterator candidatePixelsItr = SI_candidatePixels.beginw();
  Image<long>::iterator blobIDItr          = SI_blobID.beginw();

  // Link through the first scan line
  trace        = false;
  lastNeighbor = -2;
  for(unsigned int x = 0; x < width; x++, ++candidatePixelsItr, ++blobIDItr)
  {
    if(*candidatePixelsItr)
    {
      if(!trace)
      {
        pixID++;
        trace        = true;
        lastNeighbor = -2;
      }
      *blobIDItr = pixID;
      SIbackwardLink(&pixID,&pixID);
    }
    else
    {
      trace = false;
      *blobIDItr = 0;
      lastNeighbor = -2;
    }
  }

  // link through all the rest of the scan lines with "side"
  // being the scan line to the side
  Image<bool>::iterator candidatePixelsSideItr = SI_candidatePixels.beginw();
  Image<long>::iterator blobIDSideItr          = SI_blobID.beginw();
  for(unsigned int y = 1; y < height; y++)
  {
    trace        = false;
    lastNeighbor = -2;
    for(unsigned int x = 0; x < width; x++, ++candidatePixelsItr, ++blobIDItr,
          ++candidatePixelsSideItr, ++blobIDSideItr)
    {
      if(*candidatePixelsItr)
      {
        if(!trace)
        {
          pixID++;
          trace        = true;
          lastNeighbor = -2;
        }
        *blobIDItr = pixID;
        if(*candidatePixelsSideItr)
        {
          // relink blob pixels if needed
          INT check = *blobIDSideItr;
          if((signed)check != lastNeighbor)
          {
            SIbackwardLink(&pixID,&check);
            lastNeighbor = check;
          }
          else
          {
            lastNeighbor = -2;
          }
        }
      }
      else
      {
        trace        = false;
        *blobIDItr   = 0;
        lastNeighbor = -2;
      }
    }
  }
  //LINFO("Candidate pixels recognized %d",candi);
  SI_num = pixID;
}

/*********************************************************************/
/* relink pixels with new val. This allows pixels that have a new value
   to take on the old value of the blob. Slaves take on masters and
   masters can take on a slaves master.

*/
template SI_TEMPLATE_CLASS inline
void segmentImageMC<SI_TEMPLATE>::SIbackwardLink(INT *slave, INT *master)
{
  long *masterVecMaster = &SI_masterVec[*master];
  long *masterVecSlave  = &SI_masterVec[*slave];

  if(*master > SI_maxIDVal)
    SI_maxIDVal = *master;
  if(*slave > SI_maxIDVal)
    SI_maxIDVal = *slave;

  // my master has no master
  if(*masterVecMaster == -1)
  {
    // I have no master
    if(*masterVecSlave == -1)
    {
      //LINFO("my master has no master/I have no master");
      *masterVecMaster = (long)SI_masters;
      *masterVecSlave  = (long)SI_masters;
      SI_masters++;
      SI_mastersCount++;
    }
    // I already have a master
    else
    {
      //LINFO("my master has no master/I already have a master");
      *masterVecMaster = *masterVecSlave;
    }
  }
  // my master has a master
  else
  {
    // I have no master
    if(*masterVecSlave == -1)
    {
      //LINFO("my master has a master/I have no master");
      *masterVecSlave = *masterVecMaster;
    }
    // I already have a master
    else
    {
      //LINFO("my master has a master/I already have a master");

      //for(long i = 0; i < *slave; i++)
      //for(long i = 0; i < SI_masters; i++)

      std::vector<long>::iterator masterVecItr = SI_masterVec.begin();
      for(INT i = 0; i <= SI_maxIDVal; i++, ++masterVecItr)
      {
        if(*masterVecItr == *masterVecSlave)
        {
          //LINFO("SET %d to %d",masterVec[i],masterVec[slave]);
          *masterVecItr = *masterVecMaster;
        }
      }
      *masterVecSlave = *masterVecMaster;
      SI_mastersCount--;
    }
  }
}

/*********************************************************************/
/* combine individual elements into a fully combined blob */
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIcombine()
{
  SI_totalBlobs = 0;
  for(int x = 0; x < SI_blobID.getWidth(); x++)
  {
    for(int y = 0; y < SI_blobID.getHeight(); y++)
    {
      if(SI_candidatePixels.getVal(x,y))
      {
        SI_blobID.setVal(x,y,SI_masterVec[SI_blobID.getVal(x,y)]);
      }
    }
  }

  std::vector<long>::iterator masterVecItr          = SI_masterVec.begin();
  typename std::vector<INT>::iterator reOrderVecItr = SI_reOrderVec.begin();
  std::vector<bool>::iterator resetItr              = SI_reset.begin();

  for(INT x = 0; x < SI_num; x++, ++masterVecItr)
  {
    bool add = true;
    //LINFO("Master Vec %d is %d",x,masterVec[x]);
    for(INT y = 0; y < SI_totalBlobs; y++)
    {
      if((long)SI_reOrderVec[y] == *masterVecItr)
        add = false;
    }

    if((add) && (*masterVecItr != -1))
    {
      //LINFO("DOING %d",masterVec[x]);
      *reOrderVecItr = *masterVecItr;
      SI_reverseOrderVec[*masterVecItr] = SI_totalBlobs;
      *resetItr = true;
      SI_totalBlobs++; ++reOrderVecItr; ++resetItr;
    }
  }
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIdoSegment()
{
  ASSERT(SI_set1); ASSERT(SI_set2); ASSERT(SI_set3); ASSERT(SI_set4);
  //LINFO("FINDING CANDIDATES");
  if(SI_useCandidateBandPass == true)
    SIfindCandidates();
  else
    SIfindCandidatesNoBandPass();
  //LINFO("REMOVING SINGLES");
  SIremoveSinglesItr();
  //LINFO("LINKING PIXELS");
  SIdiscreteLinking();
  //LINFO("RELABELING");
  SIcombine();
  //LINFO("DONE");
}

/*=============================================================*/
/*        PUBLIC methods                                       */
/*=============================================================*/

/*********************************************************************/

template SI_TEMPLATE_CLASS
segmentImageMC<SI_TEMPLATE>::segmentImageMC()
{
;
  SI_set1 = false; SI_set2 = false; SI_set3 = false; SI_set4 = false;
  LINFO("CREATED");
  SI_lowThresh.resize(SI_channels,0);
  SI_highThresh.resize(SI_channels,0);
  SI_maxIDVal = 0;
  Image<FLOAT> timage;
  SI_infeatureMaps.resize(SI_channels,timage);
  SI_useCandidateBandPass = true;
  SI_set4 = true;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
segmentImageMC<SI_TEMPLATE>::~segmentImageMC()
{}

/* set color segmentation values for color and
   threshold
*/

/*********************************************************************/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIsetVal(typename std::vector<FLOAT> &val,
                                 typename std::vector<FLOAT> &thresh,
                                 typename std::vector<FLOAT> &skew)
{
  typename std::vector<FLOAT>::iterator ilowThresh  = SI_lowThresh.begin();
  typename std::vector<FLOAT>::iterator ihighThresh = SI_highThresh.begin();
  typename std::vector<FLOAT>::iterator ival        = val.begin();
  typename std::vector<FLOAT>::iterator ithresh     = thresh.begin();
  typename std::vector<FLOAT>::iterator iskew       = skew.begin();

  for(INT i = 0; i < val.size(); i++, ++ilowThresh, ++ihighThresh,
        ++ival, ++ithresh, ++iskew)
  {
    if(*iskew <= 0)
    {
      *ilowThresh  = *ival - *ithresh + *iskew;
      *ihighThresh = *ival + *ithresh;
    }
    else
    {
      *ilowThresh  = *ival - *ithresh;
      *ihighThresh = *ival + *ithresh + *iskew;
    }
  }
  SI_set1 = true;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIresetCandidates(bool whichWay)
{
  Image<bool>::iterator
    candidatePixelsIter    = SI_candidatePixels.beginw();

  Image<bool>::iterator
    preCandidatePixelsIter = SI_preCandidatePixels.beginw();

  // set initial candidates for each pixel

  while(candidatePixelsIter != SI_candidatePixels.endw())
  {
    *candidatePixelsIter    = false;
    *preCandidatePixelsIter = whichWay;
    ++candidatePixelsIter; ++preCandidatePixelsIter;
  }
}

/*********************************************************************/
/* set size of window frame to inspect in image */

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIsetFrame(int *x, int *y)
{
  SI_masterVec.resize(*x*(*y),-1);
  SI_reOrderVec.resize(*x*(*y));
  SI_reverseOrderVec.resize(*x*(*y));
  SI_centerX.resize(*x*(*y));
  SI_centerY.resize(*x*(*y));
  SI_Xsum.resize(*x*(*y));
  SI_Ysum.resize(*x*(*y));
  SI_mass.resize(*x*(*y));
  SI_xmin.resize(*x*(*y));
  SI_xmax.resize(*x*(*y));
  SI_ymin.resize(*x*(*y));
  SI_ymax.resize(*x*(*y));
  SI_reset.resize(*x*(*y));
  LINFO("SETTING WIDTH %d, HEIGHT %d",*x,*y);
  SI_blobID.resize(*x,*y,-1);
  SI_candidatePixels.resize(*x,*y,false);
  SI_preCandidatePixels.resize(*x,*y,false);
  SI_set2 = true;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIsetAvg(INT doAvg)
{
  typename std::vector<FLOAT> temp(SI_channels,0);

  SI_avg.resize(doAvg,temp);
  SI_std.resize(doAvg,temp);
  SI_N.resize(doAvg,0);
  SI_tempAvg.resize(doAvg,0);
  SI_tempStd.resize(doAvg,0);
  SI_iter = doAvg;
  SI_count = 0;
  SI_set3 = true;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIresetAvg()
{
  typename std::vector<FLOAT> temp(SI_channels,0);

  for(typename std::vector<std::vector<FLOAT> >::iterator
        iavg = SI_avg.begin(); iavg != SI_avg.end(); ++iavg)
    *iavg = temp;
  for(typename std::vector<std::vector<FLOAT> >::iterator
        istd = SI_std.begin(); istd != SI_std.end(); ++istd)
    *istd = temp;

  typename std::vector<FLOAT>::iterator itempAvg = SI_tempAvg.begin();
  typename std::vector<FLOAT>::iterator itempStd = SI_tempStd.begin();
  typename std::vector<INT>::iterator   iN       = SI_N.begin();

  for(unsigned int i = 0; i < SI_N.size(); i++,
        ++itempAvg, ++itempStd, ++iN)
  {
    *itempAvg = 0.0F; *itempStd = 0.0F; *iN = 0;
  }
  SI_count = 0;

}
/*********************************************************************/
/* do image segmentation by calling private memebers to
   operate on image.
   1. Low pass image
   2. Find candidate pixels
   3. Eleminate single isolated candidates
   4. Link pixels in each blob
   5. Clean up
*/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIsegment(Image<PixRGB<byte> > *image,
                               typename std::vector<Image<FLOAT> > *featureMap,
                               bool lowPass)
{
  //double micros;
  struct timezone tz;
  struct timeval start, stop;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime     = 0;
  gettimeofday(&start, &tz);

  SI_workImage      = image;

  if(lowPass)
  {
    typename std::vector<Image<FLOAT> >::iterator
      iimage = featureMap->begin();
    typename std::vector<Image<FLOAT> >::iterator
      ifmap = SI_infeatureMaps.begin();

    while(iimage != featureMap->end())
    {
      *ifmap = lowPass5(*iimage);
      ++ifmap; ++iimage;
    }
    SI_featureMaps = &SI_infeatureMaps;
  }
  else
    SI_featureMaps = featureMap;


  SIdoSegment();

  gettimeofday(&stop,&tz);
  //micros = stop.tv_usec - start.tv_usec;
  //        LINFO("%.1f Microseconds to segment\n", micros);
  //std::cout << micros << " Microseconds to segment\n";

}

/*********************************************************************/
template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SItoggleCandidateBandPass(bool toggle)
{
  SI_useCandidateBandPass = toggle;
}

/*********************************************************************/
/* This method when called will take all remaning blobs from post
   processing and create a mother blob
*/
template SI_TEMPLATE_CLASS
Image<INT> segmentImageMC<SI_TEMPLATE>::SIcreateMother(Image<INT> &img)
{
  Image<INT> mother;
  mother.resize(img.getWidth(),img.getHeight(),ZEROS);
  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      if(img.getVal(x,y) != 0)
        mother.setVal(x,y,1);
      else
        mother.setVal(x,y,0);
    }
  }
  return mother;
}

/*********************************************************************/
/* return blob map
 */
template SI_TEMPLATE_CLASS
Image<long> segmentImageMC<SI_TEMPLATE>::SIreturnBlobs()
{
  return SI_blobID;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
Image<bool> segmentImageMC<SI_TEMPLATE>::SIreturnCandidates()
{
  return SI_candidatePixels;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
Image<FLOAT> segmentImageMC<SI_TEMPLATE>::SIreturnNormalizedCandidates()
{
  Image<FLOAT> NC;
  NC.resize(SI_candidatePixels.getWidth(),SI_candidatePixels.getHeight());
  for(int x = 0; x < SI_candidatePixels.getWidth(); x++)
  {
    for(int y = 0; y < SI_candidatePixels.getHeight(); y++)
    {
      if(SI_candidatePixels.getVal(x,y))
        NC.setVal(x,y,255);
      else
        NC.setVal(x,y,0);
    }
  }
  return NC;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
Image<PixRGB<FLOAT> > segmentImageMC<SI_TEMPLATE>::SIreturnWorkImage()
{
  ASSERT((SI_doType == 1) || (SI_doType == 2));
  return *SI_workImage;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
INT segmentImageMC<SI_TEMPLATE>::SInumberBlobs()
{
  return SI_totalBlobs;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
std::vector<INT> segmentImageMC<SI_TEMPLATE>::SIgetBlobMap()
{
  return SI_reOrderVec;
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIcalcMassCenter()
{

  Image<long>::iterator iblobID          = SI_blobID.beginw();
  Image<bool>::iterator icandidatePixels = SI_candidatePixels.beginw();

  const int width  = SI_candidatePixels.getWidth();
  const int height = SI_candidatePixels.getHeight();

  for(int y = 0 ; y < height; y++)
  {
    for(int x = 0 ; x < width; x++)
    {
      if((*icandidatePixels) && (*iblobID != -1))
      {
        //std::cerr << "foo " << reverseOrderVec[blobID.getVal(x,y)] << "\n";
        INT *indexBlob = &SI_reverseOrderVec[*iblobID];
        if(SI_reset[*indexBlob])
        {
          SI_reset[*indexBlob] = false;
          SI_Xsum[*indexBlob]  = x;
          SI_Ysum[*indexBlob]  = y;
          SI_mass[*indexBlob]  = 1;
          SI_xmin[*indexBlob]  = x;
          SI_ymin[*indexBlob]  = y;
          SI_xmax[*indexBlob]  = x;
          SI_ymax[*indexBlob]  = y;
        }
        else
        {
          SI_Xsum[*indexBlob] += x;
          SI_Ysum[*indexBlob] += y;
          SI_mass[*indexBlob]++;

          if(x <= SI_xmin[*indexBlob])
            SI_xmin[*indexBlob] = x;
          if(x >= SI_xmax[*indexBlob])
            SI_xmax[*indexBlob] = x;
          if(y <= SI_ymin[*indexBlob])
            SI_ymin[*indexBlob] = y;
          if(y >= SI_ymax[*indexBlob])
            SI_ymax[*indexBlob] = y;
        }
      }
      ++iblobID; ++icandidatePixels;
    }
  }

  for(INT b = 0; b < SI_totalBlobs; b++)
  {
    //std::cerr << "MASS " << mass[x] << "\n";
    if(SI_mass[b] > 0)
    {
      SI_centerX[b] = SI_Xsum[b]/SI_mass[b];
      SI_centerY[b] = SI_Ysum[b]/SI_mass[b];
    }
  }
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
FLOAT segmentImageMC<SI_TEMPLATE>::SIgetCenterX(INT blob)
{
  return SI_centerX[blob];
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
FLOAT segmentImageMC<SI_TEMPLATE>::SIgetCenterY(INT blob)
{
  return SI_centerY[blob];
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
INT segmentImageMC<SI_TEMPLATE>::SIgetMass(INT blob)
{
  return SI_mass[blob];
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
int segmentImageMC<SI_TEMPLATE>::SIgetXmin(INT blob)
{
  return SI_xmin[blob];
}

/*********************************************************************/
//! get X max for a blob
template SI_TEMPLATE_CLASS
int segmentImageMC<SI_TEMPLATE>::SIgetXmax(INT blob)
{
  return SI_xmax[blob];
}

/*********************************************************************/
//! get Y min for a blob
template SI_TEMPLATE_CLASS
int segmentImageMC<SI_TEMPLATE>::SIgetYmin(INT blob)
{
  return SI_ymin[blob];
}

/*********************************************************************/
//! get Y max for a blob
template SI_TEMPLATE_CLASS
int segmentImageMC<SI_TEMPLATE>::SIgetYmax(INT blob)
{
  return SI_ymax[blob];
}

/*********************************************************************/
//! get the working image size in X
template SI_TEMPLATE_CLASS
int segmentImageMC<SI_TEMPLATE>::SIgetImageSizeX()
{
  return SI_candidatePixels.getWidth();
}

/*********************************************************************/
//! get the working image size in Y
template SI_TEMPLATE_CLASS
int segmentImageMC<SI_TEMPLATE>::SIgetImageSizeY()
{
  return SI_candidatePixels.getHeight();
}

/*********************************************************************/
template SI_TEMPLATE_CLASS inline
void segmentImageMC<SI_TEMPLATE>::SIgetValue(INT *blob,
                                      typename std::vector<FLOAT> *mean,
                                      typename std::vector<FLOAT> *std,
                                      INT *in)
{
  typename std::vector<FLOAT>::iterator imean = mean->begin();
  typename std::vector<FLOAT>::iterator istd  = std->begin();
  typename std::vector<Image<FLOAT> >::iterator
    ifeatureMaps = SI_featureMaps->begin();

  bool dothis = true;
  *in = 0;
  for(INT i = 0; i < SI_featureMaps->size(); i++, ++imean, ++istd,
        ++ifeatureMaps)
  {
    Image<bool>::iterator icandidatePixels      = SI_candidatePixels.beginw();
    Image<long>::iterator iblobID               = SI_blobID.beginw();
    typename Image<FLOAT>::iterator iifeatureMaps = ifeatureMaps->beginw();

    FLOAT tot = 0;
    FLOAT ss = 0;

    while(icandidatePixels != SI_candidatePixels.endw())
    {
      if((*icandidatePixels) && (*iblobID != -1))
      {
        if(SI_reverseOrderVec[*iblobID] == *blob)
        {
          tot += *iifeatureMaps;
          ss += (pow(*iifeatureMaps,2))/SI_mass[*blob];
        }
      }
      ++icandidatePixels; ++iblobID; ++iifeatureMaps;
    }
    if(SI_mass[*blob] > 0)
    {
      *imean = tot/SI_mass[*blob];
      *istd = sqrt(fabs(ss - pow(*imean,2)));
      if(dothis == true)
        *in = SI_mass[*blob] + *in;
    }
    dothis = false;
  }
}

/*********************************************************************/

template SI_TEMPLATE_CLASS
void segmentImageMC<SI_TEMPLATE>::SIgetValueMean(INT *blobListSize,
                                          typename std::vector<INT> *blobList,
                                          typename std::vector<FLOAT> *mean,
                                          typename std::vector<FLOAT> *stdd,
                                          FLOAT *mass)
{
  // average feature channels over N iterations for all motherblobs
  // modulus on dataset size
  if(SI_count == SI_iter)
    SI_count = 0;

  INT SI_tempN;

  typename std::vector<INT>::iterator iN                   = SI_N.begin();
  typename std::vector<std::vector<FLOAT> >::iterator iavg = SI_avg.begin();
  typename std::vector<std::vector<FLOAT> >::iterator istd = SI_std.begin();

  typename std::vector<FLOAT> *pavg = &SI_avg[SI_count];
  typename std::vector<FLOAT> *pstd = &SI_std[SI_count];
  INT *pcount = &SI_N[SI_count];
  *pcount = 0;

  // get mean values etc. for all blobs in this iteration
  for(INT i = 0; i < *blobListSize; i++)
  {
    SIgetValue(&blobList->at(i), &SI_tempAvg, &SI_tempStd, &SI_tempN);
    *pcount = SI_tempN + (*pcount);
    for(INT f = 0; f < SI_featureMaps->size(); f++)
    {
      pavg->at(f) += SI_tempAvg[f]*SI_tempN;
      pstd->at(f) += SI_tempStd[f]*SI_tempN;
    }
  }

  for(INT f = 0; f < SI_featureMaps->size(); f++)
  {
    pavg->at(f) = pavg->at(f)/(*pcount);
    pstd->at(f) = pstd->at(f)/(*pcount);
  }
  *mass = *pcount;
  SI_count++;

  INT massSum = 0;

  // combine the results over the last N iterations for everything
  for(INT c = 0; c < SI_iter; c++, ++iavg, ++istd, ++iN)
  {
    massSum += *iN;
    typename std::vector<FLOAT>::iterator iiavg = iavg->begin();
    typename std::vector<FLOAT>::iterator iistd = istd->begin();
    typename std::vector<FLOAT>::iterator imean = mean->begin();
    typename std::vector<FLOAT>::iterator istdd = stdd->begin();

    for(INT i = 0; i < SI_featureMaps->size(); i++, ++iiavg, ++iistd,
          ++imean, ++istdd)
    {
      if(*iN != 0)
      {
        *imean += *iiavg*(*iN);
        *istdd += *iistd*(*iN);
      }
    }
  }

  // find the total mean and std over all blobs and iterations
  typename std::vector<FLOAT>::iterator imean = mean->begin();
  typename std::vector<FLOAT>::iterator istdd = stdd->begin();
  for(INT i = 0; i < SI_featureMaps->size(); i++, ++imean, ++istdd)
  {
    *imean = *imean/massSum;
    *istdd = *istdd/massSum;
  }
}

#undef SI_TEMPLATE_CLASS
#undef SI_TEMPLATE

template class segmentImageMC<float, unsigned int, 3>;
template class segmentImageMC<float, unsigned int, 4>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
