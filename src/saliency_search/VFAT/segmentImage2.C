/*!@file VFAT/segmentImage2.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImage2.C $
// $Id: segmentImage2.C 14376 2011-01-11 02:44:34Z pez $
//

#include "VFAT/segmentImage2.H"

#include "Util/Assert.H"
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>



/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired RGB values
   if so, set candidatePixels image pixel to true
*/
void segmentImage2::SIfindCandidatesRGB()
{
  PixRGB<float> pix;

  for(unsigned int x = 0; x < SI_masterVec.size(); x++)
    SI_masterVec[x] = -1;

  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      pix = SI_workImage.getVal(x,y);
      SI_candidatePixels.setVal(x,y,false);

      // first check if the values are within threshold
      if ((pix.red() < SI_redUT) &&
          (pix.green() < SI_greenUT) &&
          (pix.blue() < SI_blueUT) &&
          (pix.red() > SI_redLT) &&
          (pix.green() > SI_greenLT) &&
          (pix.blue() > SI_blueLT))
      {
          SI_candidatePixels.setVal(x,y,true);

      }
    }
  }
}

/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired HSV values
   if so, set candidatePixels image pixel to true
*/
void segmentImage2::SIfindCandidatesHSV()
{
  PixRGB<float> pix;

  for(unsigned int x = 0; x < SI_masterVec.size(); x++)
    SI_masterVec[x] = -1;

  bool modu = false;
  double HmoduL = SI_HLT;
  double HmoduU = SI_HUT;
  // modulus on the hue value, if lower bound is less than 0
  if(SI_HLT < 0)
  {
    modu = true;
    HmoduL = 360.0F + SI_HLT;
  }
  //else if used here because wrapping both upper and lower bounds will
  //cause undesirable behavior (pixels out of range for hue even though
  //whole range should be in range)
  // modulus on upper bound, if greater than 360
  else if(SI_HUT > 360)
  {
    modu = true;
    HmoduU = SI_HUT - 360.0F;
  }

  Image<PixRGB<float> >::iterator
    workImageIter = SI_workImage.beginw() + SI_frameX1 +
    (SI_frameY1*SI_workImage.getWidth());
  Image<bool>::iterator
    candidatePixelsIter = SI_candidatePixels.beginw() + SI_frameX1 +
    (SI_frameY1*SI_workImage.getWidth());
  Image<bool>::iterator
    preCandidatePixelsIter = SI_preCandidatePixels.beginw() + SI_frameX1 +
    (SI_frameY1*SI_workImage.getWidth());

  int jump = SI_frameX1 + (SI_workImage.getWidth()-SI_frameX2);

  for(int y = SI_frameY1; y < SI_frameY2; y++)
  {
    for(int x = SI_frameX1; x < SI_frameX2; x++)
    {
      float pixH,pixS,pixV;
      pix = (*workImageIter);
      PixHSV<float>(pix).getHSV(pixH,pixS,pixV);
      *candidatePixelsIter = false;

      //LINFO("%f,%f,%f,%f,%f,%f",pixH, SI_HUT, pixS, SI_SUT, pixV, SI_VUT);
      //LINFO("%f,%f,%f,%f,%f,%f",pixH, SI_HLT, pixS, SI_SLT, pixV, SI_VLT);
      // first check if the values are within threshold
      if ((pixS < SI_SUT) &&
          (pixV < SI_VUT) &&
          (pixS > SI_SLT) &&
          (pixV > SI_VLT))
      {
        //if((pixH < SI_HUT) && (pixH > SI_HLT))
        // changed to account for modulus properties in HSV for H
        if(((modu == false) && (pixH < SI_HUT) && (pixH > SI_HLT)) ||
           ((modu == true) && (!((pixH > HmoduU) && (pixH < HmoduL)))))
        {
         // LINFO("PASS: %f, %f, %f,%d,%d", pixH,pixS,pixV,x,y);
          if((*preCandidatePixelsIter) == true)
          {
            //LINFO("SET CANDIDATE: (%d,%d)",x,y);
            *candidatePixelsIter = true;
          }
          *preCandidatePixelsIter = true;
        }
        else
        {
          //LINFO("FAIL: %f, %f, %f,%d,%d", pixH,pixS,pixV,x,y);
          *preCandidatePixelsIter = false;
        }
      }
      ++workImageIter; ++candidatePixelsIter; ++preCandidatePixelsIter;
    }
    workImageIter = workImageIter + jump;
    candidatePixelsIter = candidatePixelsIter + jump;
    preCandidatePixelsIter = preCandidatePixelsIter + jump;
  }
}

/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired HSV values
   if so, set candidatePixels image pixel to true
*/

void segmentImage2::SIfindCandidatesGREY()
{
  float pix;

  for(unsigned int x = 0; x < SI_masterVec.size(); x++)
    SI_masterVec[x] = -1;

  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      pix = SI_workImageGREY.getVal(x,y);
      SI_candidatePixels.setVal(x,y,false);

      // first check if the values are within threshold
      if ((pix < SI_VUT) && (pix > SI_VLT))
      {
          SI_candidatePixels.setVal(x,y,true);
      }
    }
  }
}

/* if a pixel does not have an orthogonal neighbor then it is removed
   i.e. all pixels without 4 connectivity are removed as noise
*/

void segmentImage2::SIremoveSingles()
{
  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      if(SI_candidatePixels.getVal(x,y))
      {
        int kill = 0;
        int XLeft = x - 1;
        int XRight = x + 1;
        int YTop = y - 1;
        int YBottom = y + 1;
        if((XLeft >= 0) && (SI_candidatePixels.getVal(XLeft,y)))
          kill++;
        if((XRight < SI_candidatePixels.getWidth())
           && (SI_candidatePixels.getVal(XRight,y)))
          kill++;
        if((YTop >= 0) && (SI_candidatePixels.getVal(x,YTop)))
          kill++;
        if((YBottom < SI_candidatePixels.getHeight())
           && (SI_candidatePixels.getVal(x,YBottom)))
          kill++;
        if(kill < 2)
          SI_candidatePixels.setVal(x,y,false);
      }
    }
  }
}

/* scan pixels from top to bottom/left to right. Link all pixels in the
   same blob. Do this using recursive back tracking to make a blob linked
   as one object and not a collection of objects
*/

void segmentImage2::SIdiscreteLinking()
{

  bool trace = false;
  long pixID = 0;
  long lastNeighbor;
  SI_masters = 0;
  SI_mastersCount = 0;
  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    trace = false;
    lastNeighbor = -2;
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      if(SI_candidatePixels.getVal(x,y))
      {
        if(!trace)
        {
          pixID++;
          trace = true;
          lastNeighbor = -2;
        }
        SI_blobID.setVal(x,y,pixID);
        if((x-1) > SI_frameX1)
        {
          if(SI_candidatePixels.getVal(x-1,y))
          {
            // relink blob pixels if needed
            long check = SI_blobID.getVal((x-1),y);
            if(check != lastNeighbor)
            {
              SIbackwardLink(pixID,check);
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
          SIbackwardLink(pixID,pixID);
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
  //LINFO("Candidate Blobs scanned %d", pixID);
  //LINFO("Candidate pixels recognized %d",candi);
  SI_num = pixID;
}

/* relink pixels with new val. This allows pixels that have a new value
   to take on the old value of the blob. Slaves take on masters and
   masters can take on a slaves master.

*/

void segmentImage2::SIbackwardLink(long slave, long master)
{
  long *masterVecMaster = &SI_masterVec[master];
  long *masterVecSlave = &SI_masterVec[slave];
  // my master has no master
  if(*masterVecMaster == -1)
  {
    // I have no master
    if(*masterVecSlave == -1)
    {
      //LINFO("my master has no master/I have no master");
      *masterVecMaster = SI_masters;
      *masterVecSlave = SI_masters;
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
      SI_mastersCount--;
      for(int i = 0; i < slave; i++)
      {
        if(SI_masterVec[i] == *masterVecMaster)
        {
          //LINFO("SET %d to %d",masterVec[i],masterVec[slave]);
          SI_masterVec[i] = *masterVecSlave;
        }
      }
    }
  }
}

/* combine individual elements into a fully combined blob */

void segmentImage2::SIcombine()
{
  SI_totalBlobs = 0;
  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      if(SI_candidatePixels.getVal(x,y))
      {
        SI_blobID.setVal(x,y,SI_masterVec[SI_blobID.getVal(x,y)]);
      }
    }
  }
  for(int x = 0; x < SI_num; x++)
  {
    bool add = true;
    //LINFO("Master Vec %d is %d",x,masterVec[x]);
    for(int y = 0; y < SI_totalBlobs; y++)
    {
      if(SI_reOrderVec[y] == SI_masterVec[x])
        add = false;
    }

    if((add) && (SI_masterVec[x] != -1))
    {
      //LINFO("DOING %d",masterVec[x]);
      SI_reOrderVec[SI_totalBlobs] = SI_masterVec[x];
      SI_reverseOrderVec[SI_masterVec[x]] = SI_totalBlobs;
      SI_reset[SI_totalBlobs] = true;
      SI_totalBlobs++;
    }
  }
  //totalBlobs--;
}

void segmentImage2::SIdoSegment()
{
  //LINFO("FINDING CANDIDATES");
  if(SI_doType == 1)
  {
    ASSERT(SI_set1); ASSERT(SI_set2); ASSERT(SI_set3); ASSERT(SI_set4);
    SIfindCandidatesRGB();
  }
  if(SI_doType == 2)
  {
    ASSERT(SI_set1); ASSERT(SI_set2); ASSERT(SI_set3); ASSERT(SI_set4);
    SIfindCandidatesHSV();
  }
  if(SI_doType == 3)
  {
    ASSERT(SI_set3); ASSERT(SI_set4);
    SIfindCandidatesGREY();
  }
  //LINFO("REMOVING SINGLES");
  SIremoveSingles();
  //LINFO("LINKING PIXELS");
  SIdiscreteLinking();
  //LINFO("RELABELING");
  SIcombine();
  //LINFO("DONE");
}

/*=============================================================*/
/*        PUBLIC methods                                       */
/*=============================================================*/

segmentImage2::segmentImage2(int imageType)
{
  SI_doType = imageType;
  SI_set1 = false; SI_set2 = false; SI_set3 = false; SI_set4 = false;
  LINFO("CREATED");
}

segmentImage2::segmentImage2()
{
  SI_doType = HSV;
  SI_set1 = false; SI_set2 = false; SI_set3 = false; SI_set4 = false;
  LINFO("CREATED");
}

segmentImage2::~segmentImage2()
{}

/* set color segmentation values for color and
   threshold
*/

void segmentImage2::SIsetRed(int val, int thresh = 1, int skew = 0)
{
  ASSERT(SI_doType == 1);
  SI_red = val;
  if(skew <= 0)
  {
    SI_redLT = val - thresh + skew;
    SI_redUT = val + thresh;
  }
  else
  {
    SI_redLT = val - thresh;
    SI_redUT = val + thresh + skew;
  }
  SI_set1 = true;
}

void segmentImage2::SIsetGreen(int val, int thresh = 1, int skew = 0)
{
  ASSERT(SI_doType == 1);
  SI_green = val;
  if(skew <= 0)
  {
    SI_greenLT = val - thresh + skew;
    SI_greenUT = val + thresh;
  }
  else
  {
    SI_greenLT = val - thresh;
    SI_greenUT = val + thresh + skew;
  }
  SI_set2 = true;
}

void segmentImage2::SIsetBlue(int val, int thresh = 1, int skew = 0)
{
  ASSERT(SI_doType == 1);
  SI_blue = val;
  if(skew <= 0)
  {
    SI_blueLT = val - thresh + skew;
    SI_blueUT = val + thresh;
  }
  else
  {
    SI_blueLT = val - thresh;
    SI_blueUT = val + thresh + skew;
  }
  SI_set3 = true;
}

void segmentImage2::SIsetHue(double val, double thresh = 1, double skew = 0)
{
  ASSERT(SI_doType == 2);
  SI_H = val;
  if(skew <= 0)
  {
    SI_HLT = val - thresh + skew;
    SI_HUT = val + thresh;
  }
  else
  {
    SI_HLT = val - thresh;
    SI_HUT = val + thresh + skew;
  }
  SI_set1 = true;
}

void segmentImage2::SIsetSat(double val, double thresh = 1, double skew = 0)
{
  ASSERT(SI_doType == 2);
  SI_S = val;
  if(skew <= 0)
  {
    SI_SLT = val - thresh + skew;
    SI_SUT = val + thresh;
  }
  else
  {
    SI_SLT = val - thresh;
    SI_SUT = val + thresh + skew;
  }
  SI_set2 = true;
}

void segmentImage2::SIsetVal(double val, double thresh = 1, double skew = 0)
{
  ASSERT((SI_doType == 2) || (SI_doType == 3));
  SI_V = val;
  if(skew <= 0)
  {
    SI_VLT = val - thresh + skew;
    SI_VUT = val + thresh;
  }
  else
  {
    SI_VLT = val - thresh;
    SI_VUT = val + thresh + skew;
  }
  SI_set3 = true;
}

/* set size of window frame to inspect in image */

void segmentImage2::SIsetFrame(int x1, int y1, int x2, int y2,
                            int realX, int realY)
{
  SI_frameX1 = x1;
  SI_frameX2 = x2;
  SI_frameY1 = y1;
  SI_frameY2 = y2;
  SI_masterVec.resize(((x2-x1)*(y2-y1)),-1);
  SI_reOrderVec.resize(((x2-x1)*(y2-y1)));
  SI_reverseOrderVec.resize(((x2-x1)*(y2-y1)));
  SI_centerX.resize(((x2-x1)*(y2-y1)));
  SI_centerY.resize(((x2-x1)*(y2-y1)));
  SI_Xsum.resize(((x2-x1)*(y2-y1)));
  SI_Ysum.resize(((x2-x1)*(y2-y1)));
  SI_mass.resize(((x2-x1)*(y2-y1)));
  SI_xmin.resize(((x2-x1)*(y2-y1)));
  SI_xmax.resize(((x2-x1)*(y2-y1)));
  SI_ymin.resize(((x2-x1)*(y2-y1)));
  SI_ymax.resize(((x2-x1)*(y2-y1)));
  SI_reset.resize(((x2-x1)*(y2-y1)));
  LINFO("SETTING WIDTH %d, HEIGHT %d",realX,realY);
  SI_blobID.resize(realX,realY,-1);
  SI_candidatePixels.resize(realX,realY,false);
  SI_preCandidatePixels.resize(realX,realY,false);
  SI_set4 = true;
}

void segmentImage2::SIsetHSVavg(long doAvg)
{
  SI_Havg.resize(doAvg,0);
  SI_Savg.resize(doAvg,0);
  SI_Vavg.resize(doAvg,0);
  SI_Hstdd.resize(doAvg,0);
  SI_Sstdd.resize(doAvg,0);
  SI_Vstdd.resize(doAvg,0);
  SI_HSVN.resize(doAvg,0);
  SI_HSViter = doAvg;
  SI_HSVcount = 0;
}

/* do image segmentation by calling private memebers to
   operate on image.
   1. Low pass image
   2. Find candidate pixels
   3. Eleminate single isolated candidates
   4. Link pixels in each blob
   5. Clean up
*/

void segmentImage2::SIsegment(Image<PixRGB<float> > &image)
{
  double micros;
  struct timezone tz;
  struct timeval start, stop;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  gettimeofday(&start, &tz);

  SI_workImage.resize(1,1);
  SI_workImage = lowPass5(image);

  SIdoSegment();

  gettimeofday(&stop,&tz);
  micros = stop.tv_usec - start.tv_usec;
        LINFO("%.1f Microseconds to segment\n", micros);
  //std::cout << micros << " Microseconds to segment\n";

}

void segmentImage2::SIsegment(Image<float> &image)
{
  struct timezone tz;
  struct timeval start, stop;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  gettimeofday(&start, &tz);

  SI_workImageGREY.resize(1,1);
  SI_workImageGREY = lowPass5(image);

  SIdoSegment();

  gettimeofday(&stop,&tz);
        //LINFO("%.1f Microseconds to segment\n", micros);

}

/* This method when called will take all remaning blobs from post
   processing and create a mother blob
*/

Image<long> segmentImage2::SIcreateMother(Image<long> &img)
{
  Image<long> mother;
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

/* return blob map
 */

Image<long> segmentImage2::SIreturnBlobs()
{
  return SI_blobID;
}

Image<bool> segmentImage2::SIreturnCandidates()
{
  return SI_candidatePixels;
}

Image<float> segmentImage2::SIreturnNormalizedCandidates()
{
  Image<float> NC;
  NC.resize(SI_candidatePixels.getWidth(),SI_candidatePixels.getHeight());
  for(int x = 0; x < SI_candidatePixels.getWidth(); x++)
  {
    for(int y = 0; y < SI_candidatePixels.getHeight(); y++)
    {
      if(SI_candidatePixels.getVal(x,y))
        NC.setVal(x,y,255.0F);
      else
        NC.setVal(x,y,0.0F);
    }
  }
  return NC;
}

Image<PixRGB<float> > segmentImage2::SIreturnWorkImage()
{
  ASSERT((SI_doType == 1) || (SI_doType == 2));
  return SI_workImage;
}

Image<float> segmentImage2::SIreturnWorkImageGREY()
{
  ASSERT(SI_doType == 3);
  return SI_workImageGREY;
}


int segmentImage2::SInumberBlobs()
{
  return SI_totalBlobs;
}

std::vector<long> segmentImage2::SIgetBlobMap()
{
  return SI_reOrderVec;
}

void segmentImage2::SIcalcMassCenter()
{
  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      if((SI_candidatePixels.getVal(x,y)) && (SI_blobID.getVal(x,y) != -1))
      {
        //std::cerr << "foo " << reverseOrderVec[blobID.getVal(x,y)] << "\n";
        long *indexBlob = &SI_reverseOrderVec[SI_blobID.getVal(x,y)];
        if(SI_reset[*indexBlob])
        {
          SI_reset[*indexBlob] = false;
          SI_Xsum[*indexBlob] = x;
          SI_Ysum[*indexBlob] = y;
          SI_mass[*indexBlob] = 1;
          SI_xmin[*indexBlob] = x;
          SI_ymin[*indexBlob] = y;
          SI_xmax[*indexBlob] = x;
          SI_ymax[*indexBlob] = y;
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
    }
  }

  for(int x = 0; x < SI_totalBlobs; x++)
  {
    //std::cerr << "MASS " << mass[x] << "\n";
    if(SI_mass[x] > 0)
    {
      SI_centerX[x] = SI_Xsum[x]/SI_mass[x];
      SI_centerY[x] = SI_Ysum[x]/SI_mass[x];
    }
  }
}

float segmentImage2::SIgetCenterX(long blob)
{
  return SI_centerX[blob];
}

float segmentImage2::SIgetCenterY(long blob)
{
  return SI_centerY[blob];
}

long segmentImage2::SIgetMass(long blob)
{
  return SI_mass[blob];
}

int segmentImage2::SIgetXmin(long blob)
{
  return SI_xmin[blob];
}

//! get X max for a blob

int segmentImage2::SIgetXmax(long blob)
{
  return SI_xmax[blob];
}

//! get Y min for a blob

int segmentImage2::SIgetYmin(long blob)
{
  return SI_ymin[blob];
}

//! get Y max for a blob

int segmentImage2::SIgetYmax(long blob)
{
  return SI_ymax[blob];
}

 //! get the working image size in X
int segmentImage2::SIgetImageSizeX()
{
  return SI_candidatePixels.getWidth();
}
//! get the working image size in Y
int segmentImage2::SIgetImageSizeY()
{
  return SI_candidatePixels.getHeight();
}

void segmentImage2::SIgetHSVvalue(long blob, float *H, float *S, float *V,
                   float *Hstd, float *Sstd, float *Vstd)
{
  float totH = 0.0F, totS = 0.0F, totV = 0.0F;
  float ssH = 0.0F, ssS = 0.0F, ssV = 0.0F;
  PixRGB<float> pix;
  for(int x = SI_frameX1; x < SI_frameX2; x++)
  {
    for(int y = SI_frameY1; y < SI_frameY2; y++)
    {
      if((SI_candidatePixels.getVal(x,y)) && (SI_blobID.getVal(x,y) != -1))
      {
        if(SI_reverseOrderVec[SI_blobID.getVal(x,y)] == blob)
        {
          float pixH,pixS,pixV;
          pix = SI_workImage.getVal(x,y);
          PixHSV<float>(pix).getHSV(pixH,pixS,pixV);
          totH +=pixH; totS += pixS; totV += pixV;
          ssH += (pow(pixH,2))/SI_mass[blob];
          ssS += (pow(pixS,2))/SI_mass[blob];
          ssV += (pow(pixV,2))/SI_mass[blob];
        }
      }
    }
  }
  if(SI_mass[blob] > 0)
  {
    *H = totH/SI_mass[blob];
    *S = totS/SI_mass[blob];
    *V = totV/SI_mass[blob];
    *Hstd = sqrt(ssH - pow(*H,2));
    *Sstd = sqrt(ssS - pow(*S,2));
    *Vstd = sqrt(ssV - pow(*V,2));
  }
}

void segmentImage2::SIgetHSVvalueMean(long blob, float *H, float *S, float *V,
                        float *Hstd, float *Sstd, float *Vstd)
{
  double massSum = 0;
  if(SI_HSVcount == SI_HSViter)
    SI_HSVcount = 0;

  SIgetHSVvalue(blob, &SI_Havg[SI_HSVcount], &SI_Savg[SI_HSVcount],
              &SI_Vavg[SI_HSVcount], &SI_Hstdd[SI_HSVcount],
              &SI_Sstdd[SI_HSVcount], &SI_Vstdd[SI_HSVcount]);

  SI_HSVN[SI_HSVcount] = SI_mass[blob];
  SI_HSVcount++;
  for(int i = 0; i < SI_HSViter; i++)
  {
    massSum += SI_HSVN[i];
    *H += SI_Havg[i]*SI_HSVN[i];
    *S += SI_Savg[i]*SI_HSVN[i];
    *V += SI_Vavg[i]*SI_HSVN[i];
    *Hstd += SI_Hstdd[i]*SI_HSVN[i];
    *Sstd += SI_Sstdd[i]*SI_HSVN[i];
    *Vstd += SI_Vstdd[i]*SI_HSVN[i];
  }
  if(massSum > 0)
  {
    *H = *H/massSum;
    *S = *S/massSum;
    *V = *V/massSum;
    *Hstd = *Hstd/massSum;
    *Sstd = *Sstd/massSum;
    *Vstd = *Vstd/massSum;
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
