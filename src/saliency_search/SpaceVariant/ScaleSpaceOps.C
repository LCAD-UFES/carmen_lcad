/*!@file SpaceVariant/ScaleSpaceOps.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/ScaleSpaceOpss.C $

#include "SpaceVariant/ScaleSpaceOps.H"
#include "Image/ImageSet.H"
#include "Image/Pixels.H"
#include "Image/LowPass.H" //for lowpass5
#include "Image/ShapeOps.H" //for decXY
#include "Util/Assert.H"

// ######################################################################
// Local Edge implementation
// ######################################################################
LocalEdge::LocalEdge(const Dims& inp_dims, const float& stdc, const float& smult, const float& ori, const uint& length, const uint density) : onReg(), offReg()
{ 
  // If using a combination center surround scale space pixels (smult > 1.0):
  // by fitting two difference of gaussians with different centers and polarity
  // we were able to find the best params (center surround var and separation between 
  // DoGs) to sin waves of different frequencies. Then we fit a function to 
  // the surface created by the optimal paramters to capture their relationship. 
  // So, the function below computes the optimal separation given the std and surround multiplier. 

  // if using a combination of on and off responses (smult < 1.0):
  // By fitting difference of guassians (with different centers) to  sine functions,
  // a relationship between the std and the best offset between centers was found.
  const float inter = (density == 1) ? 1.0F : (float)(length-1) / float(density-1);  
  float dist = -1.0F * ((float)length - 1.0F) / 2.0F;
  
  float offset;
  if (smult < 1.0) 
    offset = 3.4110F * stdc;
  else
    {
      const float x1(-5.8399), x2(-3.1659), x3(-1.5079), x4(-0.7451), x5(3.4038);
      offset = x1 * exp(smult * x2) + x3 * exp(smult * x4) + x5 * stdc;
    }
    
  const float offset2 = offset/2.0F;
  //we will set it up as a vertical edge and then rotate the points with a 2x2 matrix  
  for (uint ii = 0; ii < density; ++ii)
    {
      //compute position along center of edge
      const float x1 = dist; const float y1 = 0.0F;
      float xon =  x1; float yon = y1 - offset2;
      float xoff = x1; float yoff = y1 + offset2;
      
      const float xonr = xon * cos(ori) - yon * sin(ori);
      const float yonr = xon * sin(ori) + yon * cos(ori);
      const float xoffr = xoff * cos(ori) - yoff * sin(ori);
      const float yoffr = xoff * sin(ori) + yoff * cos(ori);

      if ( (xonr >= 0) && (xonr < inp_dims.w()) && (yonr >= 0) && (yonr < inp_dims.h())
           && (xoffr >= 0) && (xoffr < inp_dims.w()) && (yoffr >= 0) && (yoffr < inp_dims.h()))
        {
          onReg.push_back(std::pair<float, float>(xonr, yonr));
          offReg.push_back(std::pair<float, float>(xoffr, yoffr));
        }

      dist += inter;
    }
}

// ######################################################################
LocalEdge::LocalEdge() : onReg(), offReg()
{ }

// ######################################################################
LocalEdge::~LocalEdge()
{ }

// ######################################################################
// scale space functions
// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
getScaleSpace(const Image<T_or_RGB>& input, const float& max_std)
{
  //Calculate levels in our scale space. The std will be 0 at the 0'th
  //level, then 2^(L-1), so the level for a given variance is :
  //L=log2(var)+1, with var > 1
  const float max_variance = max_std*max_std;
  const uint lev = (max_variance > 4.0) ? log2(max_variance)+1 : 3;//well need to go up to this level (first level is 0)

  //setup our image to blur, converting to float if necessary
  typedef typename promote_trait<T_or_RGB, float>::TP TP;  
  Image<TP> inp = input; 
  ImageSet<TP> pyr;

  //compute scale space

  //level 0, variance 0
  pyr.push_back(inp);
  
  //level 1, variance 2^0 = 1
  inp = lowPass5(inp);
  pyr.push_back(inp);
  
  //level 2, variance 2^1 = 2
  inp = lowPass5(inp);
  pyr.push_back(inp);
  
  //level 3, variance 2^2 = 4
  inp = lowPass5(lowPass5(inp));
  pyr.push_back(inp);

  for (uint ii = 4; ii <= lev; ii+=2)
  {
    //level ii, variance 2^(ii-1)
    inp = decXY(inp,2);
    inp = lowPass5(inp);
    pyr.push_back(inp);
  
    //level ii+1, variance 2^(ii)
    inp = lowPass5(lowPass5(inp));
    pyr.push_back(inp);
  }
  return pyr;
}

// ######################################################################
template <class T_or_RGB>
void addScale(ImageSet<T_or_RGB>& pyr, const float& max_std)
{
  ASSERT(pyr.size() >= 4);//otherwisw we didn't first call getScaleSpace
 
  //Calculate levels in our scale space. The variance will be 0 at the first
  //level, then 2^(L-1), so the level for a given variance is :
  //L=log2(var)+1, with var > 1
  const float max_variance = max_std*max_std;
  const uint lev = (max_variance > 4.0) ? log2(max_variance)+1 : 3;//well need to go up to this level (first level is 0)
  
  if (pyr.size() <= lev)
  {
    Image<T_or_RGB> inp = pyr.back();
    for (uint ii = pyr.size()-1; ii <= lev; ii+=2)
    {
      //level ii, variance 2^(ii-1)
      inp = decXY(inp,2);
      inp = lowPass5(inp);
      pyr.push_back(inp);
      
      //level ii+1, variance 2^(ii)
      inp = lowPass5(lowPass5(inp));
      pyr.push_back(inp);
    }
  }
}

// ######################################################################
template <class T_or_RGB>
T_or_RGB getScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& std)
{
  const int nlev = pyr.size();
  const float variance = std*std;
  const float lev = (variance >= 1.0F) ? log2(variance)+1 : variance;
  const int lev1 = int(floor(lev)); ASSERT(lev1 < nlev);
  // let's do a getValInterp on the two adjacent levels, then a linear
  // interpolation between these two values:

  // we will interpolate between lev and lev+1. Let's compute the
  // scaled down (x, y) coordinates for lev:
  float facx = 1.0F;
  float facy = 1.0F;
  if (lev1 >= 4) //all at original scale below 4
    {
      facx = pyr[0].getDims().w() / pyr[lev1].getDims().w();
      facy = pyr[0].getDims().h() / pyr[lev1].getDims().h();
    }

  float ii1 = x / facx, jj1 = y / facy;
  const float ww1 = float(pyr[lev1].getWidth() - 1);
  const float hh1 = float(pyr[lev1].getHeight() - 1);

  if (ii1 > ww1) ii1 = ww1;
  if (jj1 > hh1) jj1 = hh1;

  const T_or_RGB val1 = pyr[lev1].getValInterp(ii1, jj1);

  // if we are at the top level, then we cannot interpolate in the
  // scale dimension, so we just use that single level:
  if (lev1 == nlev - 1) return val1;

  // otherwise, repeat for level lev+1:
  const uint lev2 = lev1 + 1;
  facx = 1.0;
  facy = 1.0;
  if (lev2 >= 4)//all at original scale below 4
    {
      facx = pyr[0].getDims().w() / pyr[lev2].getDims().w();
      facy = pyr[0].getDims().h() / pyr[lev2].getDims().h();
    }

  float ii2 = x / facx, jj2 = y / facy;
  const float ww2 = float(pyr[lev2].getWidth() - 1);
  const float hh2 = float(pyr[lev2].getHeight() - 1);

  if (ii2 > ww2) ii2 = ww2;
  if (jj2 > hh2) jj2 = hh2;

  const T_or_RGB val2 = pyr[lev2].getValInterp(ii2, jj2);

  // now a last linear interpolation between these two values:
  const float fz = lev - float(lev1);   // fractional z value
  return T_or_RGB(val1 + (val2 - val1) * fz); // no need to clamp
}

// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP 
getDiffScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, 
                       const float& std_center, const float& std_surround, const bool on_center)
{
  typedef typename promote_trait<T_or_RGB, float>::TP TP;  
  const TP c = (TP)getScaleSpacePixel(pyr, x, y, std_center);
  const TP s = (TP)getScaleSpacePixel(pyr, x, y, std_surround);
  return (on_center) ? (c - s) : (s - c);
}

// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP 
getDivScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, 
                      const float& std_center, const float& std_surround)
{
  typedef typename promote_trait<T_or_RGB, float>::TP TP;  
  const TP c = (TP)getScaleSpacePixel(pyr, x, y, std_center);
  const TP s = (TP)getScaleSpacePixel(pyr, x, y, std_surround);
  const TP ss = (s != TP(0)) ? s : TP(1);
  return c / ss;
}

// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP
getEdgeScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& std, const LocalEdge& edgemap)
{
  typedef typename promote_trait<T_or_RGB, float>::TP TP;  
  TP sum = TP();
  for (uint ii = 0; ii < edgemap.onReg.size(); ++ii)
    {
      const float xon = edgemap.onReg[ii].first + x;
      const float yon = edgemap.onReg[ii].second + y;
      const float xoff = edgemap.offReg[ii].first + x;
      const float yoff = edgemap.offReg[ii].second + y;
      
      const TP on = (TP)getScaleSpacePixel(pyr, xon, yon, std);
      const TP off = (TP)getScaleSpacePixel(pyr, xoff, yoff, std);
      sum += on - off;
    }
  sum /= (TP)edgemap.onReg.size();
  return sum;
}

// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP
getEdgeScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& stdc, const float& stds, const LocalEdge& edgemap)
{
  typedef typename promote_trait<T_or_RGB, float>::TP TP;  
  TP sum = TP();
  for (uint ii = 0; ii < edgemap.onReg.size(); ++ii)
    {
      const float xon = edgemap.onReg[ii].first + x;
      const float yon = edgemap.onReg[ii].second + y;
      const float xoff = edgemap.offReg[ii].first + x;
      const float yoff = edgemap.offReg[ii].second + y;

      const TP on = getDiffScaleSpacePixel(pyr, xon, yon, stdc, stds, true);
      const TP off = getDiffScaleSpacePixel(pyr, xoff, yoff, stdc, stds, false);
      sum = on + off;
    }
  sum /= (TP)edgemap.onReg.size();
  return sum;
}

#include "inst/SpaceVariant/ScaleSpaceOps.I"
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
