/*!@file VFAT/segmentImage.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImage.C $
// $Id: segmentImage.C 14376 2011-01-11 02:44:34Z pez $
//

#include "VFAT/segmentImage.H"

#include "Util/Assert.H"
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>



/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired RGB values
   if so, set candidatePixels image pixel to true
*/
void segmentImage::findCandidatesRGB()
{
  PixRGB<float> pix;

  for(unsigned int x = 0; x < masterVec.size(); x++)
    masterVec[x] = -1;

  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
    {
      pix = workImage.getVal(x,y);
      candidatePixels.setVal(x,y,false);

      // first check if the values are within threshold
      if ((pix.red() < redUT) &&
          (pix.green() < greenUT) &&
          (pix.blue() < blueUT) &&
          (pix.red() > redLT) &&
          (pix.green() > greenLT) &&
          (pix.blue() > blueLT))
      {
          candidatePixels.setVal(x,y,true);

      }
    }
  }
}

/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired HSV values
   if so, set candidatePixels image pixel to true
*/
void segmentImage::findCandidatesHSV()
{
  PixRGB<float> pix;

  for(unsigned int x = 0; x < masterVec.size(); x++)
    masterVec[x] = -1;

  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
    {
      float pixH,pixS,pixV;
      pix = workImage.getVal(x,y);
      PixHSV<float>(pix).getHSV(pixH,pixS,pixV);
      candidatePixels.setVal(x,y,false);

      //LINFO("%f,%f,%f,%f,%f,%f",pixH, HUT, pixS, SUT, pixV, VUT);
      //LINFO("%f,%f,%f,%f,%f,%f",pixH, HLT, pixS, SLT, pixV, VLT);
      // first check if the values are within threshold
      if ((pixS < SUT) &&
          (pixV < VUT) &&
          (pixS > SLT) &&
          (pixV > VLT))
      {
        // changed to account for modulus properties in HSV for H
        if((pixH < HUT) && (pixH > HLT))
          //if(((modu == false) && (pixH < HUT) && (pixH > HLT)) ||
          // ((modu == true) && (!((pixH > HUT) && (pixH < Hmodu)))))
        {
          if(preCandidatePixels.getVal(x,y) == true)
          {
            //LINFO("SET CANDIDATE");
            candidatePixels.setVal(x,y,true);
          }
          preCandidatePixels.setVal(x,y,true);
        }
        else
        {
          preCandidatePixels.setVal(x,y,false);
        }
      }
    }
  }
}

/* find candidate pixels by just checking image pixels to see if they
   are within threshold of desired HSV values
   if so, set candidatePixels image pixel to true
*/

void segmentImage::findCandidatesGREY()
{
  float pix;

  for(unsigned int x = 0; x < masterVec.size(); x++)
    masterVec[x] = -1;

  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
    {
      pix = workImageGREY.getVal(x,y);
      candidatePixels.setVal(x,y,false);

      // first check if the values are within threshold
      if ((pix < VUT) && (pix > VLT))
      {
          candidatePixels.setVal(x,y,true);
      }
    }
  }
}

/* if a pixel does not have an orthogonal neighbor then it is removed
   i.e. all pixels without 4 connectivity are removed as noise
*/

void segmentImage::removeSingles()
{
  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
    {
      if(candidatePixels.getVal(x,y))
      {
        int kill = 0;
        int XLeft = x - 1;
        int XRight = x + 1;
        int YTop = y - 1;
        int YBottom = y + 1;
        if((XLeft >= 0) && (candidatePixels.getVal(XLeft,y)))
          kill++;
        if((XRight < candidatePixels.getWidth())
           && (candidatePixels.getVal(XRight,y)))
          kill++;
        if((YTop >= 0) && (candidatePixels.getVal(x,YTop)))
          kill++;
        if((YBottom < candidatePixels.getHeight())
           && (candidatePixels.getVal(x,YBottom)))
          kill++;
        if(kill < 2)
          candidatePixels.setVal(x,y,false);
      }
    }
  }
}

/* scan pixels from top to bottom/left to right. Link all pixels in the
   same blob. Do this using recursive back tracking to make a blob linked
   as one object and not a collection of objects
*/

void segmentImage::discreteLinking()
{

  bool trace = false;
  long pixID = 0;
  long lastNeighbor;
  masters = 0;
  mastersCount = 0;
  for(int x = frameX1; x < frameX2; x++)
  {
    trace = false;
    lastNeighbor = -2;
    for(int y = frameY1; y < frameY2; y++)
    {
      if(candidatePixels.getVal(x,y))
      {
        if(!trace)
        {
          pixID++;
          trace = true;
          lastNeighbor = -2;
        }
        blobID.setVal(x,y,pixID);
        if((x-1) > frameX1)
        {
          if(candidatePixels.getVal(x-1,y))
          {
            // relink blob pixels if needed
            long check = blobID.getVal((x-1),y);
            if(check != lastNeighbor)
            {
              backwardLink(pixID,check);
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
          backwardLink(pixID,pixID);
        }
      }
      else
      {
        trace = false;
        blobID.setVal(x,y,0);
        lastNeighbor = -2;
      }
    }
  }
  //LINFO("Candidate Blobs scanned %d", pixID);
  //LINFO("Candidate pixels recognized %d",candi);
  num = pixID;
}

/* relink pixels with new val. This allows pixels that have a new value
   to take on the old value of the blob. Slaves take on masters and
   masters can take on a slaves master.

*/

void segmentImage::backwardLink(long slave, long master)
{
  long *masterVecMaster = &masterVec[master];
  long *masterVecSlave = &masterVec[slave];
  // my master has no master
  if(*masterVecMaster == -1)
  {
    // I have no master
    if(*masterVecSlave == -1)
    {
      //LINFO("my master has no master/I have no master");
      *masterVecMaster = masters;
      *masterVecSlave = masters;
      masters++;
      mastersCount++;
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
      mastersCount--;
      for(int i = 0; i < slave; i++)
      {
        if(masterVec[i] == *masterVecMaster)
        {
          //LINFO("SET %d to %d",masterVec[i],masterVec[slave]);
          masterVec[i] = *masterVecSlave;
        }
      }
    }
  }
}

/* combine individual elements into a fully combined blob */

void segmentImage::combine()
{
  totalBlobs = 0;
  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
    {
      if(candidatePixels.getVal(x,y))
      {
        blobID.setVal(x,y,masterVec[blobID.getVal(x,y)]);
      }
    }
  }
  for(int x = 0; x < num; x++)
  {
    bool add = true;
    //LINFO("Master Vec %d is %d",x,masterVec[x]);
    for(int y = 0; y < totalBlobs; y++)
    {
      if(reOrderVec[y] == masterVec[x])
        add = false;
    }

    if((add) && (masterVec[x] != -1))
    {
      //LINFO("DOING %d",masterVec[x]);
      reOrderVec[totalBlobs] = masterVec[x];
      reverseOrderVec[masterVec[x]] = totalBlobs;
      reset[totalBlobs] = true;
      totalBlobs++;
    }
  }
  //totalBlobs--;
}

void segmentImage::doSegment()
{
  //LINFO("FINDING CANDIDATES");
  if(doType == 1)
  {
    ASSERT(set1); ASSERT(set2); ASSERT(set3); ASSERT(set4);
    findCandidatesRGB();
  }
  if(doType == 2)
  {
    ASSERT(set1); ASSERT(set2); ASSERT(set3); ASSERT(set4);
    findCandidatesHSV();
  }
  if(doType == 3)
  {
    ASSERT(set3); ASSERT(set4);
    findCandidatesGREY();
  }
  //LINFO("REMOVING SINGLES");
  removeSingles();
  //LINFO("LINKING PIXELS");
  discreteLinking();
  //LINFO("RELABELING");
  combine();
  //LINFO("DONE");
}

/*=============================================================*/
/*        PUBLIC methods                                       */
/*=============================================================*/

segmentImage::segmentImage(int imageType)
{
  doType = imageType;
  set1 = false; set2 = false; set3 = false; set4 = false;
  LINFO("CREATED");
}

segmentImage::segmentImage()
{
  doType = HSV;
  set1 = false; set2 = false; set3 = false; set4 = false;
  LINFO("CREATED");
}

segmentImage::~segmentImage()
{}

/* set color segmentation values for color and
   threshold
*/

void segmentImage::setRed(int val, int thresh = 1, int skew = 0)
{
  ASSERT(doType == 1);
  red = val;
  if(skew <= 0)
  {
    redLT = val - thresh + skew;
    redUT = val + thresh;
  }
  else
  {
    redLT = val - thresh;
    redUT = val + thresh + skew;
  }
  set1 = true;
}

void segmentImage::setGreen(int val, int thresh = 1, int skew = 0)
{
  ASSERT(doType == 1);
  green = val;
  if(skew <= 0)
  {
    greenLT = val - thresh + skew;
    greenUT = val + thresh;
  }
  else
  {
    greenLT = val - thresh;
    greenUT = val + thresh + skew;
  }
  set2 = true;
}

void segmentImage::setBlue(int val, int thresh = 1, int skew = 0)
{
  ASSERT(doType == 1);
  blue = val;
  if(skew <= 0)
  {
    blueLT = val - thresh + skew;
    blueUT = val + thresh;
  }
  else
  {
    blueLT = val - thresh;
    blueUT = val + thresh + skew;
  }
  set3 = true;
}

void segmentImage::setHue(double val, double thresh = 1, double skew = 0)
{
  ASSERT(doType == 2);
  H = val;
  if(skew <= 0)
  {
    HLT = val - thresh + skew;
    HUT = val + thresh;
  }
  else
  {
    HLT = val - thresh;
    HUT = val + thresh + skew;
  }
  set1 = true;
}

void segmentImage::setSat(double val, double thresh = 1, double skew = 0)
{
  ASSERT(doType == 2);
  S = val;
  if(skew <= 0)
  {
    SLT = val - thresh + skew;
    SUT = val + thresh;
  }
  else
  {
    SLT = val - thresh;
    SUT = val + thresh + skew;
  }
  set2 = true;
}

void segmentImage::setVal(double val, double thresh = 1, double skew = 0)
{
  ASSERT((doType == 2) || (doType == 3));
  V = val;
  if(skew <= 0)
  {
    VLT = val - thresh + skew;
    VUT = val + thresh;
  }
  else
  {
    VLT = val - thresh;
    VUT = val + thresh + skew;
  }
  set3 = true;
}

/* set size of window frame to inspect in image */

void segmentImage::setFrame(int x1, int y1, int x2, int y2,
                            int realX, int realY)
{
  frameX1 = x1;
  frameX2 = x2;
  frameY1 = y1;
  frameY2 = y2;
  masterVec.resize(((x2-x1)*(y2-y1)),-1);
  reOrderVec.resize(((x2-x1)*(y2-y1)));
  reverseOrderVec.resize(((x2-x1)*(y2-y1)));
  centerX.resize(((x2-x1)*(y2-y1)));
  centerY.resize(((x2-x1)*(y2-y1)));
  Xsum.resize(((x2-x1)*(y2-y1)));
  Ysum.resize(((x2-x1)*(y2-y1)));
  mass.resize(((x2-x1)*(y2-y1)));
  xmin.resize(((x2-x1)*(y2-y1)));
  xmax.resize(((x2-x1)*(y2-y1)));
  ymin.resize(((x2-x1)*(y2-y1)));
  ymax.resize(((x2-x1)*(y2-y1)));
  reset.resize(((x2-x1)*(y2-y1)));
  LINFO("SETTING WIDTH %d, HEIGHT %d",realX,realY);
  blobID.resize(realX,realY,-1);
  candidatePixels.resize(realX,realY,false);
  preCandidatePixels.resize(realX,realY,false);
  set4 = true;
}

void segmentImage::setHSVavg(long doAvg)
{
  Havg.resize(doAvg,0);
  Savg.resize(doAvg,0);
  Vavg.resize(doAvg,0);
  Hstdd.resize(doAvg,0);
  Sstdd.resize(doAvg,0);
  Vstdd.resize(doAvg,0);
  HSVN.resize(doAvg,0);
  HSViter = doAvg;
  HSVcount = 0;
}

/* do image segmentation by calling private memebers to
   operate on image.
   1. Low pass image
   2. Find candidate pixels
   3. Eleminate single isolated candidates
   4. Link pixels in each blob
   5. Clean up
*/

void segmentImage::segment(Image<PixRGB<float> > &image)
{
  struct timezone tz;
  struct timeval start, stop;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  gettimeofday(&start, &tz);

  workImage.resize(1,1);
  workImage = lowPass5(image);

  doSegment();

  gettimeofday(&stop,&tz);
  //std::cout << micros << " Microseconds to segment\n";

}

void segmentImage::segment(Image<float> &image)
{
  struct timezone tz;
  struct timeval start, stop;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  gettimeofday(&start, &tz);

  workImageGREY.resize(1,1);
  workImageGREY = lowPass5(image);

  doSegment();

  gettimeofday(&stop,&tz);
  //std::cout << micros << " Microseconds to segment\n";

}

/* This method when called will take all remaning blobs from post
   processing and create a mother blob
*/

Image<long> segmentImage::createMother(Image<long> &img)
{
  Image<long> mother;
  mother.resize(img.getWidth(),img.getHeight(),ZEROS);
  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
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

Image<long> segmentImage::returnBlobs()
{
  return blobID;
}

Image<bool> segmentImage::returnCandidates()
{
  return candidatePixels;
}

Image<float> segmentImage::returnNormalizedCandidates()
{
  Image<float> NC;
  NC.resize(candidatePixels.getWidth(),candidatePixels.getHeight());
  for(int x = 0; x < candidatePixels.getWidth(); x++)
  {
    for(int y = 0; y < candidatePixels.getHeight(); y++)
    {
      if(candidatePixels.getVal(x,y))
        NC.setVal(x,y,255.0F);
      else
        NC.setVal(x,y,0.0F);
    }
  }
  return NC;
}

Image<PixRGB<float> > segmentImage::returnWorkImage()
{
  ASSERT((doType == 1) || (doType == 2));
  return workImage;
}

Image<float> segmentImage::returnWorkImageGREY()
{
  ASSERT(doType == 3);
  return workImageGREY;
}


int segmentImage::numberBlobs()
{
  return totalBlobs;
}

std::vector<long> segmentImage::getBlobMap()
{
  return reOrderVec;
}

void segmentImage::calcMassCenter()
{
  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
    {
      if((candidatePixels.getVal(x,y)) && (blobID.getVal(x,y) != -1))
      {
        //std::cerr << "foo " << reverseOrderVec[blobID.getVal(x,y)] << "\n";
        long *indexBlob = &reverseOrderVec[blobID.getVal(x,y)];
        if(reset[*indexBlob])
        {
          reset[*indexBlob] = false;
          Xsum[*indexBlob] = x;
          Ysum[*indexBlob] = y;
          mass[*indexBlob] = 1;
          xmin[*indexBlob] = x;
          ymin[*indexBlob] = y;
          xmax[*indexBlob] = x;
          ymax[*indexBlob] = y;
        }
        else
        {
          Xsum[*indexBlob] += x;
          Ysum[*indexBlob] += y;
          mass[*indexBlob]++;
          if(x <= xmin[*indexBlob])
            xmin[*indexBlob] = x;
          if(x >= xmax[*indexBlob])
            xmax[*indexBlob] = x;
          if(y <= ymin[*indexBlob])
            ymin[*indexBlob] = y;
          if(y >= ymax[*indexBlob])
            ymax[*indexBlob] = y;
        }
      }
    }
  }

  for(int x = 0; x < totalBlobs; x++)
  {
    //std::cerr << "MASS " << mass[x] << "\n";
    if(mass[x] > 0)
    {
      centerX[x] = Xsum[x]/mass[x];
      centerY[x] = Ysum[x]/mass[x];
    }
  }
}

float segmentImage::getCenterX(long blob)
{
  return centerX[blob];
}

float segmentImage::getCenterY(long blob)
{
  return centerY[blob];
}

long segmentImage::getMass(long blob)
{
  return mass[blob];
}

int segmentImage::getXmin(long blob)
{
  return xmin[blob];
}

//! get X max for a blob

int segmentImage::getXmax(long blob)
{
  return xmax[blob];
}

//! get Y min for a blob

int segmentImage::getYmin(long blob)
{
  return ymin[blob];
}

//! get Y max for a blob

int segmentImage::getYmax(long blob)
{
  return ymax[blob];
}

 //! get the working image size in X
int segmentImage::getImageSizeX()
{
  return candidatePixels.getWidth();
}
//! get the working image size in Y
int segmentImage::getImageSizeY()
{
  return candidatePixels.getHeight();
}

void segmentImage::getHSVvalue(long blob, float *H, float *S, float *V,
                   float *Hstd, float *Sstd, float *Vstd)
{
  float totH = 0.0F, totS = 0.0F, totV = 0.0F;
  float ssH = 0.0F, ssS = 0.0F, ssV = 0.0F;
  PixRGB<float> pix;
  for(int x = frameX1; x < frameX2; x++)
  {
    for(int y = frameY1; y < frameY2; y++)
    {
      if((candidatePixels.getVal(x,y)) && (blobID.getVal(x,y) != -1))
      {
        if(reverseOrderVec[blobID.getVal(x,y)] == blob)
        {
          float pixH,pixS,pixV;
          pix = workImage.getVal(x,y);
          PixHSV<float>(pix).getHSV(pixH,pixS,pixV);
          totH +=pixH; totS += pixS; totV += pixV;
          ssH += (pow(pixH,2))/mass[blob];
          ssS += (pow(pixS,2))/mass[blob];
          ssV += (pow(pixV,2))/mass[blob];
        }
      }
    }
  }
  if(mass[blob] > 0)
  {
    *H = totH/mass[blob];
    *S = totS/mass[blob];
    *V = totV/mass[blob];
    *Hstd = sqrt(ssH - pow(*H,2));
    *Sstd = sqrt(ssS - pow(*S,2));
    *Vstd = sqrt(ssV - pow(*V,2));
  }
}

void segmentImage::getHSVvalueMean(long blob, float *H, float *S, float *V,
                        float *Hstd, float *Sstd, float *Vstd)
{
  double massSum = 0;
  if(HSVcount == HSViter)
    HSVcount = 0;
  getHSVvalue(blob, &Havg[HSVcount], &Savg[HSVcount], &Vavg[HSVcount]
              ,&Hstdd[HSVcount], &Sstdd[HSVcount], &Vstdd[HSVcount]);
  HSVN[HSVcount] = mass[blob];
  HSVcount++;
  for(int i = 0; i < HSViter; i++)
  {
    massSum += HSVN[i];
    *H += Havg[i]*HSVN[i];
    *S += Savg[i]*HSVN[i];
    *V += Vavg[i]*HSVN[i];
    *Hstd += Hstdd[i]*HSVN[i];
    *Sstd += Sstdd[i]*HSVN[i];
    *Vstd += Vstdd[i]*HSVN[i];
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
