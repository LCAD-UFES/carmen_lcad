/*!@file SIFT/FeatureVector.C Feature vector for SIFT obj recognition */

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
// Primary maintainer for this file: James Bonaiuto <bonaiuto@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/FeatureVector.C $
// $Id: FeatureVector.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "SIFT/FeatureVector.H"
#include "Util/Assert.H"
#include "Util/Promotions.H"  // for clamped_convert<T>()
#include <cmath>
#include "Image/DrawOps.H"

// ######################################################################
FeatureVector::FeatureVector(int xSize, int ySize, int zSize, bool wrapZ) :
  itsFV(xSize*ySize*zSize, 0.0F),
  itsXSize(xSize), itsYSize(ySize), itsZSize(zSize), itsWrapZ(wrapZ)
{ }

// ######################################################################
FeatureVector::~FeatureVector()
{ }

// ########################################################################
void FeatureVector::addValue(const float x, const float y,
                             const float z, const float value)
{
  int xi0, xi1, yi0, yi1, zi0, zi1;   // bins
  float wx0, wy0, wz0, wx1, wy1, wz1; // corresponding weights

  // if close to bounds then the values go fully into the end bins,
  // otherwise they split between two adjacent bins. Note: a value of
  // 2.0 should equally split between bins 1 and 2:
  if (x <= 0.5F)
    { xi0 = 0; xi1 = 0; wx0 = 0.5F; wx1 = 0.5F; }
  else if (x >= (float)itsXSize-0.5F)
    { xi0 = itsXSize-1; xi1 = itsXSize-1; wx0 = 0.5F; wx1 = 0.5F; }
  else
    {
      const float xx = x - 0.5F;
      xi0 = int(xx); xi1 = xi0 + 1;
      wx1 = xx - float(xi0); wx0 = 1.0F - wx1;
    }

  if (y <= 0.5F)
    { yi0 = 0; yi1 = 0; wy0 = 0.5F; wy1 = 0.5F; }
  else if (y >= (float)itsYSize-0.5F)
    { yi0 = itsYSize-1; yi1 = itsYSize-1; wy0 = 0.5F; wy1 = 0.5F; }
  else
    {
      const float yy = y - 0.5F;
      yi0 = int(yy); yi1 = yi0 + 1;
      wy1 = yy - float(yi0); wy0 = 1.0F - wy1;
    }


  // the situation is different for orientation as we wrap around:
  // orientation are now labeld 'z' for more general purpose

  if (itsWrapZ){
          //Wrap the Z around the itsZSize
          if (z <= 0.5F)
          {
                  zi0 = 0; zi1 = itsZSize-1;
                  wz0 = 0.5F + z; wz1 = 1.0F - wz0;
          }
          else if (z >= itsZSize-0.5F)
          {
                  zi0 = itsZSize-1; zi1 = 0;
                  wz0 = ((float)itsZSize+0.5F) - z; wz1 = 1.0F - wz0;
          }
          else
          {
                  const float zz = z - 0.5F;
                  zi0 = int(zz); zi1 = zi0 + 1;
                  wz1 = zz - float(zi0); wz0 = 1.0F - wz1;
          }
  } else {
          //Dont wrap z bin
          if (z <= 0.5F)
          {
                  zi0 = 0; zi1 = 0;
                  wz0 = 0.5F; wz1 =0.5F;
          }
          else if (z >= (float)itsZSize-0.5F)
          {
                  zi0 = itsZSize-1; zi1 = itsZSize-1;
                  wz0 = 0.5F; wz1 = 0.5F;
          }
          else
          {
                  const float zz = z - 0.5F;
                  zi0 = int(zz); zi1 = zi0 + 1;
                  wz1 = zz - float(zi0); wz0 = 1.0F - wz1;
          }
  }

  // convention: we add 1 for each unit of o (our fastest varying
  // index), then zSize for each unit of y, finally zSize*ySize for each unit of
  // x. Let's populate our 8 bins:

  //No more opt calc unless we informace the size of the bins
  //Hopfully the compiler will optemize powers of 2
  //xi0 <<= 5; xi1 <<= 5; yi0 <<= 3; yi1 <<= 3;

  xi0 *= itsZSize*itsYSize; xi1 *= itsZSize*itsYSize;
  yi0 *= itsZSize; yi1 *= itsZSize;


  itsFV[xi0 + yi0 + zi0] += value * wx0 * wy0 * wz0;
  itsFV[xi1 + yi0 + zi0] += value * wx1 * wy0 * wz0;
  itsFV[xi0 + yi1 + zi0] += value * wx0 * wy1 * wz0;
  itsFV[xi1 + yi1 + zi0] += value * wx1 * wy1 * wz0;
  itsFV[xi0 + yi0 + zi1] += value * wx0 * wy0 * wz1;
  itsFV[xi1 + yi0 + zi1] += value * wx1 * wy0 * wz1;
  itsFV[xi0 + yi1 + zi1] += value * wx0 * wy1 * wz1;
  itsFV[xi1 + yi1 + zi1] += value * wx1 * wy1 * wz1;

  //LINFO("%f,%f,%f -> %d-%d(%f) %d-%d(%f) %d-%d(%f)",
   //     x,y,o,xi0,xi1,wx0,yi0,yi1,wy0,oi0,oi1,wo0);
}

// ########################################################################
void FeatureVector::normalize()
{
  std::vector<float>::iterator ptr = itsFV.begin(), stop = itsFV.end();

  // compute sum of squares:
  float sq = 0.0f;
  while(ptr != stop) { sq += (*ptr) * (*ptr); ++ ptr; }

  // if too small to normalize, forget it:
  if (sq < 1.0e-10) return;

  // compute normalization factor:
  sq = 1.0F / sqrtf(sq);

  // normalize it:
  ptr = itsFV.begin();
  while(ptr != stop) *ptr++ *= sq;
}

// ########################################################################
void FeatureVector::threshold(const float limit)
{
  bool changed = false;

  std::vector<float>::iterator ptr = itsFV.begin(), stop = itsFV.end();

  while(ptr != stop)
    {
      if (*ptr > limit) { *ptr = limit; changed = true; }
      ++ ptr;
    }

  if (changed) normalize();
}

// ########################################################################
float FeatureVector::getValue(const uint index) const
{
  ASSERT(index < itsFV.size());
  return itsFV[index];
}

// ######################################################################
void FeatureVector::toByteKey(std::vector<byte>& dest, float thresh, bool norm)
{
  dest.resize(itsFV.size());

  // do normalization and thresholding:

  if (norm) normalize();
  if (thresh > -1) threshold(thresh);   //TODO: is -1 a good value to check for?
  if (norm) normalize();

  std::vector<float>::const_iterator src = itsFV.begin(), stop = itsFV.end();
  std::vector<byte>::iterator dst = dest.begin();

  while (src != stop) *dst++ = clamped_convert<byte>((*src++) * 512.0F);
}

// ######################################################################
Image<PixRGB<byte> > FeatureVector::getFeatureVectorImage(std::vector<byte> &fv)
{


  int WIDTH = 256;
  int HEIGHT = 256;

  Image<PixRGB<byte> > fvImg(WIDTH, HEIGHT, NO_INIT);
  fvImg.clear(PixRGB<byte>(255, 255, 255));
  int xBins = int((float)WIDTH/4);
  int yBins = int((float)HEIGHT/4);

  drawGrid(fvImg, xBins, yBins, 1, 1, PixRGB<byte>(0, 0, 0));

  for (int xx=0; xx<4; xx++){
    for (int yy=0; yy<4; yy++){
      for (int oo=0; oo<8; oo++){
        Point2D<int> loc(xBins/2+(xBins*xx), yBins/2+(yBins*yy));
        uint fv_i = xx*32+yy*8+oo;
        byte mag = 0;
        if (fv_i > fv.size())
          LFATAL("Invalid feture vector");
        else
          mag = fv[xx*32+yy*8+oo]/4;
        drawDisk(fvImg, loc, 2, PixRGB<byte>(255, 0, 0));
        drawLine(fvImg, loc,
            Point2D<int>(int(loc.i + mag*cosf(oo*M_PI/4)),
              int(loc.j + mag*sinf(oo*M_PI/4))),
            PixRGB<byte>(255, 0, 0));
      }
    }
  }

  return fvImg;
}


double FeatureVector::getMag()
{

  double mag = 0;
  std::vector<float>::iterator ptr = itsFV.begin(), stop = itsFV.end();

  while(ptr != stop)
    {
      mag += (*ptr * *ptr);
      ++ ptr;
    }

  return sqrt(mag);

}




// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
