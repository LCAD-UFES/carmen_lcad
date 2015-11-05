/*!@file SIFT/Histogram.H Header file for histograms */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/Histogram.H $
// $Id: Histogram.H 14857 2011-07-26 01:14:21Z siagian $
//

#ifndef HISTOGRAM_H_DEFINED
#define HISTOGRAM_H_DEFINED

#include "Image/Image.H"
#include <vector>

//! Histogram
class Histogram
{
public :
  //! Constructor
  Histogram(const int size = 0);

  //! Constructor
  Histogram(const Histogram& h);

  //! Constructor
  Histogram(std::vector<float> vh);

  //! Constructor
  //! create a histogram from the passed in range
  Histogram(Histogram h, const int sindex, const int eindex = -1);

  //! Constructor
  /*! Build a histogram with 256 bins from the pixel values in the image. */
  Histogram(const Image<byte>& img);

  //! Constructor
  /*! Copy the histogram from the raw input (CAUTION: we do not
    compute a histogram of the input values; we just copy the input
    histogram into our internal representation). */
  Histogram(const float* input, const int size);

  //! Constructor
  Histogram(std::vector<Histogram> histograms);

  //! Destructor
  ~Histogram();

  //! get a value from the histogram
  float getValue(const uint index) const;

  //! Access histogram through C array index interface
  float operator[](const uint index) const;

  //! set a value to the histogram
  void setValue(const uint index, const float value);

  //! add a value to the histogram
  void addValue(const uint index, const float value);

  //! add a value to the histogram, with linear interpolation
  void addValueInterp(const float index, const float value);

  //! Compute a histogram of the pixel values in an image
  void fill(const Image<byte>& img);

  //! get an image representation of the histogram
  Image<byte> getHistogramImage(const int w, const int h,
                                const float mini = -1, const float maxi = -1);

  //! smooth the histogram
  void smooth();

  //! smooth the histogram
  //! using the inputted kernel
  void smooth(Image<double> kernel);

  //! Retrieve the peak of the histogram
  float findMax() const;

  //! Retrive the peak position and value
  void findMax(int &max_loc, float &max_val);

  //! Retrieve the valley of the histogram
  float findMin() const;

  //! Retrieve the min and max
  void findMinMax(double &min, double &max);

  //!Get distance between histograms
  double getDistance(const Histogram& hist);

  //! normalize the hist by a value
  //! if sum is -1 then use the sum of all values in the histogram
  void normalize(double sum = -1);

  //! Clear the histogram
  void clear();

  //! resize the histogram
  void resize(uint size);

  //! get the number of buckets in the histogram
  int getSize();

  //! get the number of buckets in the histogram
  uint size() const;

  //! get the Chi Square difference of the two histograms
  float getChiSqDiff(Histogram hist2);

  void divide(float div);

  //! return the vector
  inline std::vector<float> getHistogram();

  //! Add two histograms
  Histogram operator+ (Histogram h);
  //Histogram operator+=(Histogram h);

  //! Substract two histograms
  Histogram operator- (Histogram h);
  //Histogram operator-=(Histogram h);

private:
  std::vector<float> itsHistogram;
};

// ######################################################################
// Implementation for Histogram inline functions
// ######################################################################
inline std::vector<float> Histogram::getHistogram()
{ return itsHistogram; }

inline Histogram Histogram::operator+(Histogram h)
{
  ASSERT(size() == h.size());

  std::vector<float>::iterator 
    itr  = itsHistogram.begin(), stop = itsHistogram.end();
  std::vector<float> vh = h.getHistogram();
  std::vector<float>::iterator itrH = vh.begin(); 
  std::vector<float> vr(size());
  std::vector<float>::iterator itrR = vr.begin(); 

  while(itr != stop) *itrR++ = (*itr++) + (*itrH++);
  Histogram res(vr);
  return res;
}

// inline Histogram Histogram::operator+=(Histogram h)
// {  
//   Histogram boo = (*this + h);

//   for(uint i = 0; i < boo.size(); i++)
//     if(boo[i] >0.0F) LINFO("<%3d> %f", i, boo[i]);

//   return boo;
// }

inline Histogram Histogram::operator-(Histogram h)
{
  ASSERT(size() == h.size());

  std::vector<float>::iterator 
    itr  = itsHistogram.begin(), stop = itsHistogram.end();
  std::vector<float> vh = h.getHistogram();
  std::vector<float>::iterator itrH = vh.begin(); 
  std::vector<float> vr(size());
  std::vector<float>::iterator itrR = vr.begin(); 

  while(itr != stop) *itrR++ = (*itr++) - (*itrH++);
  Histogram res(vr);
  return res;
}

// inline Histogram Histogram::operator-=(Histogram h)
// {  
//   return (*this - h);
// }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
