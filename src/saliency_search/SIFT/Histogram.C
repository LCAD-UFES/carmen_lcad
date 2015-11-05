/*!@file SIFT/Histogram.C Histogram */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/Histogram.C $
// $Id: Histogram.C 15310 2012-06-01 02:29:24Z itti $
//

#include "SIFT/Histogram.H"
#include "Image/DrawOps.H"

// #######################################################################
Histogram::Histogram(const int size) :
  itsHistogram(size)
{ clear(); }

// #######################################################################
Histogram::Histogram(const Histogram& h)
{ 
  std::vector<float> vh = h.itsHistogram;

  itsHistogram.clear();
  itsHistogram.resize(vh.size());

  std::vector<float>::iterator 
    itr = vh.begin(), stop = vh.end();
  std::vector<float>::iterator itrR = itsHistogram.begin(); 

  while(itr != stop) *itrR++ = *itr++;
}

// #######################################################################
Histogram::Histogram(std::vector<float> vh)
{ 
  itsHistogram.clear();
  itsHistogram.resize(vh.size());

  std::vector<float>::iterator 
    itr = vh.begin(), stop = vh.end();
  std::vector<float>::iterator itrR = itsHistogram.begin(); 

  while(itr != stop) *itrR++ = *itr++;
}

// #######################################################################
Histogram::Histogram(const float* input, const int size) :
  itsHistogram(size)
{
  for (int i = 0; i < size; i++) itsHistogram[i] = input[i];
}

// #######################################################################
Histogram::Histogram(std::vector<Histogram> histograms)
{
  for(uint i = 0; i < histograms.size(); i++)
    {
      for (uint j = 0; j < histograms[i].size(); j++) 
        itsHistogram.push_back(histograms[i][j]);
    }  
}

// #######################################################################
Histogram::Histogram(const Image<byte>& img) :
  itsHistogram(256)
{
  Image<float> hi1(img.getWidth(), img.getHeight(), ZEROS);

  int count1 = 0, count2 = 0;
  const int w = img.getWidth(); int h = img.getHeight();
  for (int i = 0; i < w; i++)
    for (int j = 0; j < h; j++)
      {
        if (img.getVal(i, j) < 1)
          { hi1.setVal(i, j, -255); ++ count1; }
        else if (img.getVal(i, j) > 254)
          { hi1.setVal(i, j, 255); ++ count2; }
        itsHistogram[img.getVal(i, j)] += 1.0F;
      }
}

// ######################################################################
Histogram::~Histogram()
{  }

// ######################################################################
Histogram::Histogram(Histogram h, const int sindex, const int eindex)
{
  uint si  = uint(sindex); 
  uint ei = h.size();
  if(eindex > 0) ei = uint(eindex);
  ASSERT(si < h.size() && ei < h.size() && si <= ei);
  itsHistogram.resize(ei - si + 1);

  std::vector<float> vh = h.getHistogram();
  std::vector<float>::iterator 
    itr = vh.begin()+si, stop = vh.begin() + ei;
  std::vector<float>::iterator itrR = itsHistogram.begin(); 

  while(itr != stop) *itrR++ = *itr++;
}

// #######################################################################
void Histogram::setValue(const uint index, const float value)
{
  if (index < itsHistogram.size())
    itsHistogram[index] = value;
  else
    LFATAL("Index %d out of range [0..%" ZU "]",
              index, itsHistogram.size()-1);
}

// #######################################################################
void Histogram::addValue(const uint index, const float value)
{
  if (index < itsHistogram.size())
    itsHistogram[index] += value;
  else
    LFATAL("Index %d out of range [0..%" ZU "]",
           index, itsHistogram.size()-1);
}

// #######################################################################
void Histogram::addValueInterp(const float index, const float value)
{
  ASSERT(index >= 0.0F && index < float(itsHistogram.size()));
  const uint i0 = uint(index);
  const float di = index - float(i0);
  uint i1 = i0 + 1; if (i1 >= itsHistogram.size()) i1 = 0;

  itsHistogram[i0] += value * (1.0F - di);
  itsHistogram[i1] += value * di;
}

// #######################################################################
void Histogram::fill(const Image<byte>& img)
{
  // make sure the histogram has the size 256
  itsHistogram.clear();
  itsHistogram.resize(256);

  int w = img.getWidth(); int h = img.getHeight();
  for (int i = 0; i < w; i++)
    for (int j = 0; j < h; j++)
        itsHistogram[img.getVal(i, j)] += 1.0F;
}

// #######################################################################
Image<byte> Histogram::getHistogramImage(const int w, const int h,
                                         const float mini, const float maxi)
{
  if (uint(w) < itsHistogram.size())
    LFATAL("not enough width to display the histogram properly");
  int dw = w / itsHistogram.size();

  Image<byte> res(w, h, ZEROS);
  //res.clear(255);

  // draw lines for 10% marks:
  for (int j = 0; j < 10; j++)
    drawLine(res, Point2D<int>(0, int(j * 0.1F * h)),
             Point2D<int>(w-1, int(j * 0.1F * h)), byte(64));
  drawLine(res, Point2D<int>(0, h-1), Point2D<int>(w-1, h-1), byte(64));

  float minii = mini, maxii = maxi;
  if (maxi == -1.0F) { minii = findMin(); maxii = findMax(); }

  // uniform histogram
  if (maxii == minii) minii = maxii - 1.0F;

  float range = maxii - minii;

  for (uint i = 0; i < itsHistogram.size(); i++)
    {
      int t = abs(h - int((itsHistogram[i] - minii) / range * float(h)));

      // if we have at least 1 pixel worth to draw
      if (t < h-1)
        {
          for (int j = 0; j < dw; j++)
            drawLine(res,
                     Point2D<int>(dw * i + j, t),
                     Point2D<int>(dw * i + j, h - 1),
                     byte(255));
          //drawRect(res, Rectangle::tlbrI(t,dw*i,h-1,dw*i+dw-1), byte(255));
        }
    }

  return res;
}


// #######################################################################
float Histogram::getValue(const uint index) const
{
  if (index < itsHistogram.size()) return itsHistogram[index];
  LFATAL("Index %d out of range [0..%" ZU "]",
         index, itsHistogram.size());
  return 0.0F;
}

// #######################################################################
float Histogram::operator[](const uint index) const
{
  ASSERT(index < itsHistogram.size());
  return itsHistogram[index];
}

// #######################################################################
void Histogram::smooth()
{
  std::vector<float> vect(itsHistogram.size());
  const uint siz = itsHistogram.size();

  for (uint n = 0 ; n < siz ; n++)
    {
      float val0 = itsHistogram[ (n-1+siz) % siz ];
      float val1 = itsHistogram[ (n  +siz) % siz ];
      float val2 = itsHistogram[ (n+1+siz) % siz ];

      vect[n] = 0.25F * (val0 + 2.0F*val1 + val2);
    }

  for (uint n = 0 ; n < siz ; n++) itsHistogram[n] = vect[n];
}

// #######################################################################
void Histogram::smooth(Image<double> kernel)
{
  ASSERT(kernel.getWidth() == 1 || kernel.getHeight() == 1);
  int ksize  = kernel.getSize();
  int hksize = ksize/2;
  
  int size = itsHistogram.size();
  std::vector<float> vect(size);

  // float fsumb = 0.0;
  // for (int i = 0; i < size; i++) 
  //   {
  //     fsumb+= itsHistogram[i];
  //   }
  //LINFO("ORG: fsumb: %f hksize: %d", fsumb, hksize);

  for (int i = 0; i < size; i++)
    {
      float total = 0.0; float tweight = 0.0;
      for (int j = -hksize; j <= hksize; j++)
        {
          int k = i+j; int l = j+hksize;
          if(k >= 0 && k < size)
            {
              float weight = kernel.getVal(l);
              total   += itsHistogram[k]*weight;
              tweight += weight;
            }
          vect[i] = total;///tweight;
        }
      //LINFO("v[%3d]= %15.5f == (%15.5f/%15.5f) = %15.5f", 
      //      i, itsHistogram[i], total, tweight, vect[i]);
    }
  for (int i = 0; i < size; i++) itsHistogram[i] = vect[i];

  // float fsuma = 0.0;  
  // for (int i = 0; i < size; i++) 
  //   {
  //     fsuma += itsHistogram[i];
  //   }
  //LINFO("SMO: fsuma: %f", fsuma);
}

// #######################################################################
float Histogram::findMax() const
{
  float maxi = itsHistogram[0];
  for (uint n = 1 ; n < itsHistogram.size() ; n++)
    if (itsHistogram[n] > maxi) maxi = itsHistogram[n];
  return maxi;
}

// #######################################################################
void Histogram::findMax(int &max_loc, float &max_val)
{
  max_val = itsHistogram[0];
  max_loc = 0;

  for (uint n = 1 ; n < itsHistogram.size() ; n++)
    if (itsHistogram[n] > max_val){
      max_val = itsHistogram[n];
      max_loc = n;
    }
}

// #######################################################################
float Histogram::findMin() const
{
  float mini = itsHistogram[0];
  for (uint n = 1 ; n < itsHistogram.size() ; n++)
    if (itsHistogram[n] < mini) mini = itsHistogram[n];
  return mini;
}

// #######################################################################
void Histogram::findMinMax(double &min, double &max)
{
  max = itsHistogram[0];
  min = max;
  for (uint n = 1 ; n < itsHistogram.size() ; n++)
  {
    if (itsHistogram[n] > max) max = itsHistogram[n];
    if (itsHistogram[n] < min) min = itsHistogram[n];
  }
}

// #######################################################################
void Histogram::clear()
{
  int size = itsHistogram.size();
  itsHistogram.clear();
  itsHistogram.resize(size, 0);
}

// #######################################################################
void Histogram::resize(uint size)
{
  itsHistogram.clear();
  itsHistogram.resize(size);
}

// #######################################################################
int Histogram::getSize()
{
  return itsHistogram.size();
}

uint Histogram::size() const
{
  return itsHistogram.size();
}

// #######################################################################
void Histogram::normalize(double sum)
{
  if (sum == -1)
  {
    sum = 0;
    for(uint i=0; i<itsHistogram.size(); i++)
      sum += itsHistogram[i];
  }

  for(uint i=0; i<itsHistogram.size(); i++)
    itsHistogram[i] /= sum;
}

// #######################################################################
double Histogram::getDistance(const Histogram& hist)
{
  if (hist.size() > itsHistogram.size())
    LFATAL("Incomputable histograms");
  double sum = 0;

  for(uint i=0; i<itsHistogram.size(); i++)
  {
    //euclidean distance
    sum += fabs(itsHistogram[i]-hist[i]);

    //histogram intersection
    //sum += std::min(itsHistogram[i],hist[i]);

  }

  return sum;
}

// ######################################################################
float Histogram::getChiSqDiff(Histogram hist2)
{
  ASSERT(itsHistogram.size() == hist2.size());

  // Chi-square difference 
  float diff = 0.0;
  std::vector<float>::iterator 
    itr  = itsHistogram.begin(), 
    stop = itsHistogram.end();
  std::vector<float> h2 = hist2.getHistogram();
  std::vector<float>::iterator 
    itr2 = h2.begin(); 
  //uint count = 0;
  while(itr != stop)
    {
      float dif = *itr - *itr2;
      float sum = *itr + *itr2;
      if(sum != 0.0F) diff += (dif*dif)/sum;
      
      // LINFO("[%3d] %15.5f  vs. %15.5f  ||| dif: %f sum: %f ---> %f", 
      //       count, *itr, *itr2, dif, sum, diff);
      itr++; itr2++; //count++;
    }
  diff = diff/2.0;

  // for(uint i = 0; i < hist2.size(); i++)
  //   {
  //     float dif = itsHistogram[i] - hist2[i];
  //     float sum = itsHistogram[i] + hist2[i];
  //     if(sum != 0.0F) diff += (dif*dif)/sum;
 
  //     // LINFO("[%3d] %15.5f  vs. %15.5f  ||| dif: %f sum: %f ---> %f", 
  //     //       i, itsHistogram[i], hist2[i], dif, sum, diff);
  //   }
  // diff = diff/2.0;
  return diff;
}

// #######################################################################
void Histogram::divide(float div)
{
  for(uint i = 0; i < itsHistogram.size(); i++)
    itsHistogram[i] = itsHistogram[i]/div;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
