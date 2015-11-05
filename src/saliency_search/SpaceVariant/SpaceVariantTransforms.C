/*!@file SpaceVariant/SpaceVariantTransform.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SpaceVariant/SpaceVariantTransforms.C $

#include "SpaceVariant/SpaceVariantTransforms.H"
#include "Image/MatrixOps.H" //for flipVertic,flipHoriz
#include "Image/CutPaste.H" //for concat and crop
#include "Image/LowPass.H" //for lowpass5
#include "Image/ImageSet.H" 
#include "Image/Pixels.H" 
#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include <cmath>
#include <deque>

#define PI 3.14159265F
#define PI2 PI/2.0F
#define EPS 0.001F

// ######################################################################
// implementation for SpaceVariantTransform
// ######################################################################
SpaceVariantTransform::SpaceVariantTransform() :
  itsLookup(), itsRevLookup(), itsStd(), itsMaxStd(0.0), isInitialized(false)
{
}

// ######################################################################
SpaceVariantTransform::~SpaceVariantTransform()
{
}

// ######################################################################
bool SpaceVariantTransform::initialized() const
{
  return isInitialized;
}

// ######################################################################
void SpaceVariantTransform::to(const int x, const int y, 
                               int& rings, int& wedges) const
{
  std::pair<float, float> p = itsRevLookup.getVal(x, y);
  rings = (int)round(p.first); wedges = (int)round(p.second);
}

// ######################################################################
void SpaceVariantTransform::to(const int x, const int y, 
                               float& rings, float& wedges) const
{
  std::pair<float, float> p = itsRevLookup.getVal(x, y);
  rings = p.first; wedges = p.second;
}

// ######################################################################
void SpaceVariantTransform::from(const int rings, const int wedges,
                                 int& x, int& y) const
{
  std::pair<float, float> p = itsLookup.getVal(rings, wedges);
  x = (int)round(p.first); y = (int)round(p.second);
}

// ######################################################################
void SpaceVariantTransform::from(const int rings, const int wedges,
                                 float& x, float& y) const
{
  std::pair<float,float> p = itsLookup.getVal(rings, wedges);
  x = p.first; y = p.second;
}

// ######################################################################
const Dims& SpaceVariantTransform::getCTDims() const
{
  return itsRevLookup.getDims();
}

// ######################################################################
const Dims& SpaceVariantTransform::getSVDims() const
{
  return itsLookup.getDims();
}

// ######################################################################
const float& SpaceVariantTransform::getRFSize(const uint u, const uint v) const
{
  return itsStd.getVal(u,v);
}

// ######################################################################
float SpaceVariantTransform::getRFSize(const float& u, const float& v) const
{
  return itsStd.getValInterp(u,v);
}

// ######################################################################
const float& SpaceVariantTransform::getRFSize(const uint pos) const
{
  return itsStd.getVal(pos);
}

// ######################################################################
float SpaceVariantTransform::getMaxRFSize() const
{
  return itsMaxStd;
}

// ######################################################################
SpaceVariantTransform::const_iterator SpaceVariantTransform::begin() const
{
  return itsLookup.begin();
}
  
// ######################################################################
SpaceVariantTransform::const_iterator SpaceVariantTransform::end() const
{
  return itsLookup.end();
}

// ######################################################################
SpaceVariantTransform::const_iterator SpaceVariantTransform::rbegin() const
{
  return itsRevLookup.begin();
}
  
// ######################################################################
SpaceVariantTransform::const_iterator SpaceVariantTransform::rend() const
{
  return itsRevLookup.end();
}

// #####################################################################
Image<float>::const_iterator SpaceVariantTransform::rfbegin() const
{
  return itsStd.begin();
}

// ######################################################################
Image<float>::const_iterator SpaceVariantTransform::rfend() const
{
  return itsStd.end();
}

// ######################################################################
float SpaceVariantTransform::computeDistance(const float& theta, 
                                             const uint cx, const uint cy,
                                             const uint imgw, const uint imgh)
{
  //input to this function will range from -PI/2 to 3PI/4 
  float d = 0.0F;
  
  //find the angles (from horizontal) that link cx,cy to the corners
  //of our image.
  const float angle_ru = -1.0F * atan2(cy, imgw-cx);
  const float angle_rl = atan2(imgh-cy, imgw-cx);
  const float angle_lu = 3.0F*PI2 - atan2(cx, cy);
  const float angle_ll = PI2 + atan2(cx, imgh-cy);
  
  //standard geometry to compute the length of the line from cx,cy to
  //the nearest image boundry given angle theta
  if ((theta > angle_lu) && (theta <= 3.0F*PI2))
    d = cy/cos(3.0F*PI2 - theta);
  else if ((theta >= -PI2) && (theta <= angle_ru))
    d = cy/cos(theta + PI2);
  else if ((theta > angle_ru) && (theta <= angle_rl))
    d = (imgw-cx)/cos(std::abs(theta));  
  else if ((theta > angle_rl) && (theta <= angle_ll))
    d = (imgh-cy)/cos(std::abs(theta - PI2));
  else if ((theta > angle_ll) && (theta <= angle_lu))
    d = cx/cos(std::abs(PI - theta));
  
  ASSERT(d>0.0F);
  return d;
}

// ######################################################################
// implementation for FovealTransform
// ######################################################################
FovealTransform::FovealTransform() : SpaceVariantTransform(), u_FoveaSize(0)
{ }

// ######################################################################
FovealTransform::SCALE_TYPE 
FovealTransform::toScaleType(const std::string& scale_type_string)
{
  if (scale_type_string.compare("FULL") == 0)
    return FULL;
  else if (scale_type_string.compare("CROP") == 0)
    return CROP;
  else
    return NONE;
}

// ######################################################################
void FovealTransform::setup(const uint imgw, const uint imgh,
                            const uint rings, const uint wedges,
                            const float& alpha, const float& beta,
                            const float& gain, const float& exponent, const float& offset, 
                            const float& ppdx, const float& ppdy,
                            const SCALE_TYPE& scale_type, const float& s, 
                            const float& fovea_cuttoff)
{
  //ASSERT(imgw % 2 == 0);ASSERT(imgh % 2 == 0);ASSERT(wedges/2 % 2 == 0);

  //initialize all our lookup tables to -1
  itsLookup = Image<std::pair<float,float> >(rings*2, wedges/2, NO_INIT);
  itsRevLookup = Image<std::pair<float,float> >(imgw, imgh, NO_INIT);
  itsStd = Image<float>(rings*2, wedges/2, NO_INIT);
  
  iterator iterf(itsLookup.beginw()); 
  while (iterf != itsLookup.endw())
    *iterf++ = std::pair<float,float>(-1.0F, -1.0F);

  iterf = itsRevLookup.beginw();
  while (iterf != itsRevLookup.endw())
    *iterf++ = std::pair<float,float>(-1.0F, -1.0F);
  
  Image<float>::iterator iterv(itsStd.beginw()); 
  while (iterv != itsStd.endw())
    *iterv++ = -1.0F;
  
  //some variables for our lookup tables
  const uint w2 = wedges / 2; //size of hemifield
  const uint w22 = w2/2; //visual quadrant size
  const float wstep = PI2 / (w22);//wedge step size (in radians)
  const float wstep2 = wstep/2;

  const int sx = imgw - 1;
  const int sy = imgh - 1;
  const float cx = (float)sx/2.0F; //center x of fovea
  const float cy = (float)sy/2.0F; //center y of fovea

  //////////////////////////////////////////
  //so the basic equation for our space variant transform is:
  //r=sqrt(x^2+y^2);theta=atan2(y/x);
  //u=s*ln(r*alpha + beta);v=theta;
  //
  //inverse:
  //r=[exp(u/s) - beta]/alpha
  //////////////////////////////////////////

  //find our scale factor s=umax/ln(rmax*alpha+beta)
  float scale = 0.0F;
  if (scale_type == NONE)
    scale = s;
  else if (scale_type == CROP)
    scale = (rings-1-EPS)/log( ((sx>sy) ? cy : cx) * alpha + beta);
  else if (scale_type == FULL)
    scale = (rings-1-EPS)/log( sqrt(cx*cx + cy*cy) * alpha + beta);

  /*figure out the size of our fovea. Compute the derivitive of u with
    respect to r (du/dr), and values of du/dr > c are considered in
    the fovea, where c is a user defined value. Normally c=1, as the
    fovea is usually considered the expansive area where du/dr >
    1. However, the other parameters can be chosen so that there is no
    oversampling, so we allow the user to adjust this value. For 
    numerical simplicity, we actually compute dr/du and look for dr/du 
    < 1*/
  if (fovea_cuttoff > 0.0)//ignore fovea size if the cutoff is 0 or less
    {
      u_FoveaSize = uint(2.0F * scale * log(fovea_cuttoff * alpha * scale));
      if (u_FoveaSize < 0)
        LFATAL("for the desired transform parameters and foveal cuttoff, there are no pixels "
               "in the fovea! Try increasing the fovea size (--retinasv-fovea-size) or "
               "increasing the cutoff to be considered the fovea (--retinasv-fovea-cutoff)");
    }
      
  //loop through the rings
  for (uint u = 0; u < rings; ++u)
    {
      //compute ring distance in pixels
      float r = (exp(u / scale) - beta)/alpha;
      
      //loop through all the angles storing the transform in a lookup
      //table, processing the right and left hemifields seperately so
      //that the right hemifield (all rings, wedges -pi/2 : pi/2) is
      //all on the right of the transformed image.
      float theta = -PI2 + wstep2;      
      for (uint v = 0; v < w2; ++v)//first -pi/2 : pi/2, right visual vield
        {          
          float xpos = r * cos(theta);
          float ypos = r * sin(theta);

          //compute the receptive field size for this ring as std of a guassian
          const float deg = sqrt((xpos / ppdx)*(xpos / ppdx)  + (ypos / ppdy)*(ypos / ppdy));//get in distance in deg
          const float std = (gain*pow(deg,exponent)+offset)*(ppdx+ppdy)/2;//relationship between deg and std in pixels.
          
          if (std > itsMaxStd)
            itsMaxStd = std;

          xpos += cx;
          ypos += cy;

          if (itsRevLookup.coordsOk(xpos, ypos))
            {
              itsLookup.setVal(rings - 1 + u, v, std::pair<float, float>(xpos, ypos));
              itsStd.setVal(rings - 1 + u, v, std);
            }
          theta += wstep;
        }

      for (int v = w2 - 1; v >= 0; --v) //now for pi/2 : 3pi/4, left hemifield
        {
          float xpos = r * cos(theta);
          float ypos = r * sin(theta);
          
          //compute the receptive field size for this ring as std of a guassian
          const float deg = sqrt((xpos / ppdx)*(xpos / ppdx)  + (ypos / ppdy)*(ypos / ppdy));//get in distance in deg
          const float std = (gain*pow(deg,exponent)+offset)*(ppdx+ppdy)/2;//relationship between deg and std in pixels.
          
          if (std > itsMaxStd)
            itsMaxStd = std;
          
          xpos += cx;
          ypos += cy;
          
          if (itsRevLookup.coordsOk(xpos, ypos))
            {
              itsLookup.setVal(rings - u, v, std::pair<float, float>(xpos, ypos));
              itsStd.setVal(rings - u, v, std);
            }
          theta += wstep;
        }
    }

  //create a reverse lookup table
  const float wscale = (w2 - 1 - EPS) / (PI - EPS);
  const float r2 = (rings*2-1)/2;
  for (uint x = 0; x < imgw; ++x)
    for (uint y = 0; y < imgh; ++y)
      {
        const float xx = x - cx;
        const float yy = y - cy;
        const float r = sqrt(xx*xx + yy*yy);
        
        float theta = atan2(yy,xx);
        if (theta < -PI2) 
          theta += 2.0F*PI;        
        theta += PI2 + EPS;

        //u position (ring)
        float u = scale * log(r * alpha + beta) - 0.5;        
        
        //v position (wedge)
        float v;
        if (theta > PI)
          {           
            v = (theta - PI) * wscale + EPS;
            v = w2 - 1 - v;
            u = r2 + 1 - u;
          }
        else
          {
            v = theta * wscale;
            u = r2 + u;
          }

        //clmap U and V
        if (u > (rings*2-1))
          u = rings*2-1-EPS;

        if (u < 0)
          u = 0;

        if (v > (w2-1))
          v= w2-1 - EPS;

        if (v < 0)
          v = 0;

        //save coordinate
        itsRevLookup.setVal(x, y, std::pair<float,float>(u,v));
      }
  //all done
  isInitialized = true;
}

// ######################################################################
const uint FovealTransform::getFoveaSize() const
{
  return u_FoveaSize;
}

// ######################################################################
// implementation for SCTransform
// ######################################################################
SCTransform::SCTransform() : SpaceVariantTransform()
{ }

// ######################################################################
SCTransform::RECEPTIVEFIELD_TYPE
SCTransform::toRFType(const std::string& rf_type_string)
{
  if (rf_type_string.compare("MAGNO") == 0)
    return MAGNO;
  else if (rf_type_string.compare("PARVO") == 0)
    return PARVO;
  else
  {
    LFATAL("No such RF type");
    return MAGNO;
  }
}

// ######################################################################
Point2D<float> SCTransform::pix2Deg(const float & x, const float & y, const float & ppdx, const float & ppdy, const float & imgx, const float & imgy)
{
  float const pixx = x - (imgx / 2.0F - 1.0F);
  float const pixy = y * -1.0F + (imgy / 2.0F - 1.0F);
  return Point2D<float>(pixx / ppdx, pixy / ppdy);
}

// ######################################################################
Point2D<float> SCTransform::deg2Pix(const float & x, const float & y, const float & ppdx, const float & ppdy, const float & imgx, const float & imgy)
{
  float const hdposx = x * ppdx + (imgx / 2.0F - 1.0F);
  float const hdposy = -1.0F * (y * ppdy - (imgy / 2.0F - 1.0F));
  return Point2D<float>(hdposx, hdposy);
}
// ######################################################################
float SCTransform::rf2Pix(const float & rf, const float & ppdx, const float & ppdy)
{
  return rf * (ppdx + ppdy)/2.0F;
}

// ######################################################################
void SCTransform::setup(const uint imgw, const uint imgh,
                        const uint rings, const uint wedges,
                        const float& ppdx, const float& ppdy, const float& maxDeg, SCTransform::RECEPTIVEFIELD_TYPE const type)
{
  //parameters
  float s1=0.02974;
  float s2=1.3923;
  float beta=0.81685;
  float alpha=0.32226;
  
  //magno
  float mSlope = 0.0042;
  float mOffset = 0.0519;
  
  //parvo
  float pSlope = 0.0002;
  float pExp = 1.7689;
  float pOffset = 0.0252;

  switch (type)
  {
  case MAGNO:
    setup(imgw, imgh, rings, wedges, ppdx, ppdy, maxDeg, s1, s2, beta, alpha, mSlope, 1.0, mOffset);
        break;
        
  case PARVO:
    setup(imgw, imgh, rings, wedges, ppdx, ppdy, maxDeg, s1, s2, beta, alpha, pSlope, pExp, pOffset);
    break;
    
  default:
    LFATAL("No such Receptive field type");
  }
}

// ######################################################################
void SCTransform::setup(const uint imgw, const uint imgh,
                        const uint rings, const uint wedges,
                        const float& ppdx, const float& ppdy, const float& maxDeg, 
                        const float& s1, const float& s2, const float& beta, const float& alpha,
                        const float& rf_slope, const float& rf_exp, const float& rf_offset,
                        const float& dog_factor)
{
  //initialize all our lookup tables to -1
  itsLookup = Image<std::pair<float,float> >(rings*2, wedges/2, NO_INIT);
  itsRevLookup = Image<std::pair<float,float> >(imgw, imgh, NO_INIT);
  itsStd = Image<float>(rings*2, wedges/2, NO_INIT);

  iterator iterf(itsLookup.beginw()); 
  while (iterf != itsLookup.endw())
    *iterf++ = std::pair<float,float>(-1.0F, -1.0F);

  iterf = itsRevLookup.beginw();
  while (iterf != itsRevLookup.endw())
    *iterf++ = std::pair<float,float>(-1.0F, -1.0F);
  
  Image<float>::iterator iterv(itsStd.beginw()); 
  while (iterv != itsStd.endw())
    *iterv++ = -1.0F;
  
  //////////////////////////////////////////
  // so the basic equation for our SC transform is
  // r=sqrt(x^2+y^2);
  // theta=atan2(y/x);
  // u=s*ln(r*alpha + beta);
  // v=theta;
  //
  // inverse:
  // r=[exp(u/s) - beta]/alpha
  //
  // here, we use s=s1*cos(v)+s2;
  //
  // where 'u' and 'v' are in collicular space (mm), 'theta' is in radians and 'x', 'y' and 'r' is in degrees. 
  //////////////////////////////////////////
  
  //some variables for our lookup tables
  const uint w2 = wedges / 2; //size of hemifield
  const float wstep = PI / w2;//wedge step size (in radians) from (-PI/2+wstep/2 to PI/2-wstep/2)
  const float wstep2 = wstep / 2.0F;

  //get the scale in the longest direction (x axis);
  const float sx = s1 * cos(0) + s2;
  float maxU = sx * log(alpha * maxDeg + beta);//size of colliculus given max visual angle
  const float ustep = maxU / (rings-1);

  //loop through the rings
  float u = 0;
  for (uint uind = 0; uind < rings; ++uind)
  {   
    //loop through all the angles storing the transform in a lookup
    //table, processing the right and left hemifields seperately so
    //that the right hemifield (all rings, wedges -pi/2 : pi/2) is
    //all on the right of the transformed image.
    float theta = -PI2 + wstep2;      
    for (uint vind = 0; vind < w2; ++vind)//first -pi/2 : pi/2, right visual vield
    {          
      //set the scale
      const float s = s1 * cos(theta) + s2;

      //compute ring distance in degrees
      const float r = (exp(u / s) - beta)/alpha;

      //xy pos in degrees
      float xpos = r * cos(theta);
      float ypos = r * sin(theta);

      //convert to pixels
      Point2D<float> pix = deg2Pix(xpos, ypos, ppdx, ppdy, imgw, imgh);
      xpos = pix.i;
      ypos = pix.j;

      //compute the receptive field size as a function of eccentricity. If dog_factor <= 1, the rf is represented in
      //terms of standard deviation of gaussian in degrees. if dog_factor > 1 then it is the size in degress of a disk
      //stimulus that will optimally excite a DoG filter with a given surrent factor.
      float std = 0;
      if (dog_factor <= 1)
      {
        const float stddeg = rf_slope * pow(r, rf_exp) + rf_offset;
        std = rf2Pix(stddeg, ppdx, ppdy);
      }
      else
      {
        const float diskdeg = rf_slope * pow(r, rf_exp) + rf_offset;
        const float cstddeg = (D_SQRT_2 * diskdeg * sqrt(dog_factor - 1) * sqrt(dog_factor + 1)) / (4 * dog_factor * sqrt(log(dog_factor)));
        std = rf2Pix(cstddeg, ppdx, ppdy);
      }

      if (std > itsMaxStd)
        itsMaxStd = std;      
      
      if (itsRevLookup.coordsOk(xpos, ypos))
      {
        itsLookup.setVal(rings + uind, w2 - 1 - vind, std::pair<float, float>(xpos, ypos));
        itsStd.setVal(rings + uind, w2 - 1 - vind, std);
      }

      theta += wstep;
    }

    theta = PI2+wstep2;        
    for (uint vind = 0; vind < w2; ++vind) //now for pi/2 : 3pi/2, left hemifield
    {
      //set the scale
      const float s = s1 * -1 * cos(theta) + s2;

      //compute ring distance in degrees
      const float r = (exp(u / s) - beta)/alpha;

      //xy pos in degrees
      float xpos = r * cos(theta);
      float ypos = r * sin(theta);
      
      //convert to pixels
      Point2D<float> pix = deg2Pix(xpos, ypos, ppdx, ppdy, imgw, imgh);
      xpos = pix.i;
      ypos = pix.j;

      //compute the receptive field size as a function of eccentricity. If dog_factor <= 1, the rf is represented in
      //terms of standard deviation of gaussian in degrees. if dog_factor > 1 then it is the size in degress of a disk
      //stimulus that will optimally excite a DoG filter with a given surrent factor.
      float std = 0;
      if (dog_factor <= 1)
      {
        const float stddeg = rf_slope * pow(r, rf_exp) + rf_offset;
        std = rf2Pix(stddeg, ppdx, ppdy);
      }
      else
      {
        const float diskdeg = rf_slope * pow(r, rf_exp) + rf_offset;
        const float cstddeg = (D_SQRT_2 * diskdeg * sqrt(dog_factor - 1) * sqrt(dog_factor + 1)) / (4 * dog_factor * sqrt(log(dog_factor)));
        std = rf2Pix(cstddeg, ppdx, ppdy);
      }

      if (std > itsMaxStd)
        itsMaxStd = std;
      
      if (itsRevLookup.coordsOk(xpos, ypos))
      {
        itsLookup.setVal(rings - 1 - uind, vind, std::pair<float, float>(xpos, ypos));
        itsStd.setVal(rings - 1 - uind, vind, std);
      }

      theta += wstep;
    }

    //update our position
    u += ustep;
  }

  float step = PI / (w2 - 1);
  //create a reverse lookup table
  for (uint x = 0; x < imgw; ++x)
    for (uint y = 0; y < imgh; ++y)
    {
      //get position in degrees
      Point2D<float> deg =  pix2Deg(x,y, ppdx, ppdy, imgw, imgh);
      const float xx = deg.i;
      const float yy = deg.j;
      const float r = sqrt(xx*xx + yy*yy);
      float theta = atan2(yy,xx);

      //atan2 returns such that the upper vield runs from 0-pi (starting at the right) and the lower field from -pi-0
      //(starting from left) so lets make it on the invertal 0 to 2*pi (where 0 is the bottom center of the display)
      float v = theta + PI2;
      v = (v < 0) ? v+2.0F*PI : v;

      //convert to indicies
      float uind, vind;
      if ((v >= 0) && (v < PI)) //right side of SC Transform
      {
        //set the scale
        const float s = s1 * cos(v - PI2) + s2;   
        //get the position in colicular coords
        float u = s * log(r * alpha + beta);  

        vind = v / step;
        vind = w2 - 1 - vind;
        uind = rings + u / ustep;
      }
      else                      //left side of SC Transform
      {
        //set the scale
        const float s = s1 * -1 * cos(v - PI2) + s2;   
        //get the position in colicular coords
        float u = s * log(r * alpha + beta);  

        vind = (v - PI) / step;
        uind = rings - 1 - u / ustep;
      }
   
      //save coordinate
      if (itsLookup.coordsOk(uind, vind))
        itsRevLookup.setVal(x, y, std::pair<float,float>(uind,vind));
    }
  
  //all done
  isInitialized = true;
}

// ######################################################################
// Free function implementation for SpaceVariantTransform
// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> transformTo(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, 
                            const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache, const float& scaleoffset)
{
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  
  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(input, sv.getMaxRFSize() + scaleoffset);
  
  //to hold the transformed image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space 
  Image<T_or_RGB> out(sv.getSVDims(), ZEROS);
  typename Image<T_or_RGB>::iterator iter(out.beginw());
  SpaceVariantTransform::const_iterator sviter(sv.begin());

  uint pos = 0;
  while (iter != out.endw())
    {
      const float var = sv.getRFSize(pos) + scaleoffset;
      if (sviter->first >= 0.0F) 
        *iter = clamped_convert<T_or_RGB>(getScaleSpacePixel(pyr, sviter->first, sviter->second, var));
      ++iter; ++sviter; ++pos;
    }
  return out;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> transformFrom(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input)
{
  //to hold the cartesian output image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space 
  Image<T_or_RGB> out(sv.getCTDims(), ZEROS);
  typename Image<T_or_RGB>::iterator oiter = out.beginw();
  SpaceVariantTransform::const_iterator sviter(sv.rbegin());  
  while (oiter != out.endw())
    {
      float u = sviter->first, v = sviter->second;
      if (u >= 0.0F)
        *oiter = clamped_convert<T_or_RGB>(input.getValInterp(u, v));
      ++oiter;++sviter;
    }
  return out;
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToDoG(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& smult, 
               const bool centeron, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache, const float& scaleoffset)
{
  ASSERT(smult > 1.0F);//enforce that we have a larger surround than center
  
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  
  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(input, (sv.getMaxRFSize()+scaleoffset) * smult);
  
  //to hold the transformed image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space 
  Image<TP> out(sv.getSVDims(), ZEROS);
  typename Image<TP>::iterator iter(out.beginw());
  SpaceVariantTransform::const_iterator sviter(sv.begin());
  
  uint pos = 0;
  while (iter != out.endw())
    {
      const float stdc = sv.getRFSize(pos) + scaleoffset;
      const float stds = stdc * smult;
      
      if (sviter->first >= 0.0F) 
        *iter = getDiffScaleSpacePixel(pyr, sviter->first, sviter->second, stdc, stds, centeron);
      
      ++iter; ++sviter; ++pos;
    }
  return out;
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToDivG(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& smult, 
                const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache, const float& scaleoffset)
{
  ASSERT(smult > 1.0F);//enforce that we have a larger surround than center
  
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  
  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(input, (sv.getMaxRFSize()+scaleoffset) * smult);
  
  //to hold the transformed image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space 
  Image<TP> out(sv.getSVDims(), ZEROS);
  typename Image<TP>::iterator iter(out.beginw());
  SpaceVariantTransform::const_iterator sviter(sv.begin());
  
  uint pos = 0;
  while (iter != out.endw())
    {
      const float stdc = sv.getRFSize(pos) + scaleoffset;
      const float stds = stdc * smult;
      
      if (sviter->first >= 0.0F) 
        *iter = getDivScaleSpacePixel(pyr, sviter->first, sviter->second, stdc, stds);
      
      ++iter; ++sviter; ++pos;
    }
  return out;
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToEdge(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const Image<LocalEdge>& edgemap, 
                const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache, const float& scaleoffset)
{
  //promote our image now if necessary
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  
  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(input, sv.getMaxRFSize() + scaleoffset);
  
  //to hold the transformed image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space 
  Image<TP> out(sv.getSVDims(), ZEROS);
  typename Image<TP>::iterator iter(out.beginw());
  SpaceVariantTransform::const_iterator sviter(sv.begin());
  Image<LocalEdge>::const_iterator edgeiter(edgemap.begin());  

  uint pos = 0;
  while (iter != out.endw())
    {
      const float std = sv.getRFSize(pos) + scaleoffset;
      if (sviter->first >= 0.0F) 
        *iter = getEdgeScaleSpacePixel(pyr, sviter->first, sviter->second, std, *edgeiter);
      
      ++iter; ++sviter; ++pos;++edgeiter;
    }
  return out;
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToEdge(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& smult, const Image<LocalEdge>& edgemap, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache, const float& scaleoffset)
{
  ASSERT((smult >= 1.0F) && (smult <=6.0F));//enforce that we have a larger surround than center
  
  //promote our image now if necessary
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  
  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(input, (sv.getMaxRFSize() + scaleoffset) * smult);
  
  //to hold the transformed image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space 
  Image<TP> out(sv.getSVDims(), ZEROS);
  typename Image<TP>::iterator iter(out.beginw());
  SpaceVariantTransform::const_iterator sviter(sv.begin());
  Image<LocalEdge>::const_iterator edgeiter(edgemap.begin());  

  uint pos = 0;
  while (iter != out.endw())
    {
      const float stdc = sv.getRFSize(pos) + scaleoffset;
      const float stds = stdc*smult;
      if (sviter->first >= 0.0F) 
        *iter = getEdgeScaleSpacePixel(pyr, sviter->first, sviter->second, stdc, stds, *edgeiter);
      
      ++iter; ++sviter; ++pos;++edgeiter;
    }
  return out;
}


// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToPyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL)
{
  //promote our image now if necessary
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  Image<TP> inimg = input;
  
  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(inimg, sv.getMaxRFSize() + scales.getMaxLevel());
  
  //compute each scale
  ImageSet<TP> retpyr(scales.numLevels(), sv.getSVDims(), NO_INIT);
  for (uint ii = 0; ii < scales.numLevels(); ++ii)
    retpyr[ii] = transformTo(sv, inimg,  &pyr, scales.getVariance(ii));
  
  return retpyr;
}

// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToDoGPyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& smult, const bool centeron, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache)
{ 
  //promote our image now if necessary
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  Image<TP> inimg = input;

  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(inimg, (sv.getMaxRFSize() + scales.getMaxLevel()) * smult);

  //compute each scale
  ImageSet<TP> retpyr(scales.numLevels(), sv.getSVDims(), NO_INIT);
  for (uint ii = 0; ii < scales.numLevels(); ++ii)
    retpyr[ii] = transformToDoG(sv, inimg, smult, centeron, &pyr, scales.getVariance(ii));
  
  return retpyr;
}

// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToEdgePyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const Image<LocalEdge>& edgemap, const float& orientation, const uint& length, const float& density, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache)
{

  //promote our image now if necessary
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  Image<TP> inimg = input;

  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(inimg, sv.getMaxRFSize() + scales.getMaxLevel());

  //compute each scale
  ImageSet<TP> retpyr(scales.numLevels(), sv.getSVDims(), NO_INIT);
  for (uint ii = 0; ii < scales.numLevels(); ++ii)
    retpyr[ii] = transformToEdge(sv, inimg, edgemap, &pyr, scales.getVariance(ii)); 
  
  return retpyr;
}

// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToEdgePyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& smult, const Image<LocalEdge>& edgemap, const float& orientation, const uint& length, const float& density, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache)
{
  //promote our image now if necessary
  typedef typename promote_trait<T_or_RGB, float>::TP TP;
  Image<TP> inimg = input;
  
  //compute scale space, the image may get promoted by getScaleSpace
  ImageSet<TP> pyr = (cache) ? *cache : getScaleSpace(inimg, (sv.getMaxRFSize() + scales.getMaxLevel()) * smult);

  //compute each scale
  ImageSet<TP> retpyr(scales.numLevels(), sv.getSVDims(), NO_INIT);
  for (uint ii = 0; ii < scales.numLevels(); ++ii)
    retpyr[ii] = transformToEdge(sv, inimg, smult, edgemap, &pyr, scales.getVariance(ii));
  
  return retpyr;
}

// ######################################################################
// This needs to be updated to use std instead of variance, but since
// it isn't used anywhere right now, its OK
// ######################################################################
// ######################################################################
// Free function implementation for FovealTransform
// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> transformToFoveal(const FovealTransform& sv, const Image<T_or_RGB>& input, const bool interp)
{
  //promote our image now if necessary
  Image<typename promote_trait<T_or_RGB, float>::TP> inp = input;

  //to hold the transformed image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space 
  Image<T_or_RGB> out(sv.getSVDims(), ZEROS);

  float var = 0;
  const uint w2 = sv.getSVDims().w()/2;
  const uint h = sv.getSVDims().h();

  //loop through all our rings
  for (uint r = 0; r < w2; ++r)
    {
      //lowpass our input image until it is blurred to the variance of
      //this rings receptive field variance
      const float rf = sv.getRFSize(w2 + r,0);
      while ((rf - var) > 0.25F)
        {
          inp = lowPass3(inp);
          var += 0.5F;
        }

      //right visual field
      for (uint w = 0; w < h; ++w)
        {
          const uint u = w2 + r;
          if (interp == true){
            float x, y;
            sv.from(u, w, x, y);
            if (x >= 0.0F)
              out.setVal(u, w, clamped_convert<T_or_RGB>(inp.getValInterp(x,y)));
          }
          else{
            int x, y;
            sv.from(u, w, x, y);
            if (x > -1)
              out.setVal(u, w, clamped_convert<T_or_RGB>(inp.getVal(x, y)));
          }
        }

      //left visual field
      for (uint w = 0; w < h; ++w)
        {
          const uint u = w2 - r - 1;
          if (interp == true){
            float x, y;
            sv.from(u, w, x, y);
            if (x >= 0.0F)
              out.setVal(u,w,clamped_convert<T_or_RGB>(inp.getValInterp(x,y)));
          }
          else{
            int x, y;
            sv.from(u, w, x, y);
            if (x > -1)
              out.setVal(u, w, clamped_convert<T_or_RGB>(inp.getVal(x, y)));
          }
        }
    }  
  return out;
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToDoGFoveal(const FovealTransform& sv,const Image<T_or_RGB>& input,
                     const float& smult, const bool centeron, const bool interp)
{
  ASSERT(smult > 1.0F);//enforce that we have a larger surround than center
  
  //promote our image now if necessary
  Image<typename promote_trait<T_or_RGB, float>::TP> inp = input;

  //to hold the transformed image. We need to initialize with zeros
  //in case of a transform which doesn't completely fill the space x
  Image<T_or_RGB> out(sv.getSVDims(), ZEROS);

  //to hold intermediate filter results
  std::deque<Image<typename promote_trait<T_or_RGB, float>::TP> > pyr;

  float var = 0;//image variance
  const uint rings = sv.getSVDims().w();
  const uint w2 = rings/2;//half widht
  const uint h = sv.getSVDims().h();//height
  uint rc = 0;//index of current center
  float rfc = sv.getRFSize(rc,0);//variance of current center

  //loop over our rings
  for (uint r = 0; r < w2; ++r)
    {
      //enfoce that the surround is always 0.5F greater than the
      //center, so that the subtraction just doesn't just cancel out.
      float rf = sv.getRFSize(r,0);
      if (rf*(smult - 1) >= 0.5F) rf *= smult; else rf += 0.5F;

      //The ideas here is to low pass to the surround size, but pickup
      //and store any center rf sizes on the way
      while (( rf - var) > 0.25F)
        {
          //see if we have any centers 
          while (((rfc - var) <= 0.25F) && (rc < rings))
            {
              pyr.push_back(inp);
              rfc = sv.getRFSize(++rc, 0);
            }
          inp = lowPass3(inp);
          var += 0.5F;
        }

      Image<typename promote_trait<T_or_RGB, float>::TP> diff;//store C-S 

      //calculate our center surround response if any new ones left
      if (pyr.size() > 0) {
        if (centeron)
          {
            diff = pyr.front();
            pyr.pop_front();
            diff -= inp;
          }
        else
          {
            diff = inp;
            diff -= pyr.front();
            pyr.pop_front();
          }
      }

      //right visual field
      for (uint w = 0; w < h; ++w)
        {
          const uint u = w2 + r;
          if (interp == true){
            float x, y;
            sv.from(u, w, x, y);
            if (x >= 0.0F)
              out.setVal(u, w, diff.getValInterp(x, y));
          }
          else{
            int x, y;
            sv.from(u, w, x, y);
            if (x != -1)
              out.setVal(u, w, diff.getVal(x, y));
          }
        }
      
      //left visual field
      for (uint w = 0; w < h; ++w)
        {
          const uint u = w2 - r - 1;
          if (interp == true){
            float x, y;
            sv.from(u, w, x, y);
            if (x >= 0.0F)
              out.setVal(u, w, diff.getValInterp(x, y));
          }
          else {
            int x, y;
            sv.from(u, w, x, y);
            if (x != -1)
              out.setVal(u, w, diff.getVal(x, y));
          }
        }
    }
  return out;
}

// ######################################################################
// helper function
// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> replicateHemifield(const Image<T_or_RGB>& inp, const uint pix)
{ 
  //take a small chunk from each hemiefield,flip and rotate, then
  //concatinate them horizontally, swapping sides.
  Image<T_or_RGB> t =
    concatX( flipHoriz(flipVertic( crop(inp, Point2D<int>(inp.getWidth()/2, 0),
                                        Dims(inp.getWidth()/2, pix)) )),
             flipHoriz(flipVertic( crop(inp, Point2D<int>(0, 0),
                                        Dims(inp.getWidth()/2, pix)) )) );
  
  Image<T_or_RGB> b =
    concatX( flipHoriz(flipVertic( crop(inp, Point2D<int>(inp.getWidth()/2,
                                                          inp.getHeight()-pix),
                                        Dims(inp.getWidth()/2, pix)) )),
             flipHoriz(flipVertic( crop(inp,Point2D<int>(0,inp.getHeight()-pix),
                                        Dims(inp.getWidth()/2, pix)) )) );
  
  Image<T_or_RGB> output = concatY( concatY(t,inp), b);
  return output;
}

// ######################################################################
template <class T_or_RGB>
void getFoveaPeriphery(const FovealTransform& sv, const Image<T_or_RGB>& ret, 
                       Image<T_or_RGB>& fovea, Image<T_or_RGB>& periph)
{
  ASSERT(sv.getSVDims() == ret.getDims());
    
  const uint fix = sv.getFoveaSize();
  const uint fix2 = fix / 2;
  const uint w = ret.getWidth();
  const uint w2 = w / 2;
  const uint h = ret.getHeight();

  Point2D<int> a(0,0), b(w2-fix2, 0), c(w2+fix2,0);
  Dims ad(b.i, h), bd(fix, h), cd(w - c.i, h);

  Image<T_or_RGB> lh = crop(ret, a, ad);
  Image<T_or_RGB> rh = crop(ret, c, cd);
  fovea = crop(ret, b, bd);
  periph = concatX(lh, rh);
}

// ######################################################################
Image<LocalEdge> LocalEdgeMap(const SpaceVariantTransform& sv, const float& smult, 
                              const float& orientation, const uint& length, const uint density)
{
  Image<LocalEdge> out(sv.getSVDims(), NO_INIT);
  Image<LocalEdge>::iterator ii(out.beginw());
  Image<float>::const_iterator tr(sv.rfbegin());
  while (ii != out.end())
    {
      if (*tr >= 0.0)
        *ii = LocalEdge(sv.getCTDims(), *tr, smult, orientation, length, density);
      ++ii; ++tr;
    }
  return out;
}

#include "inst/SpaceVariant/SpaceVariantTransforms.I"
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
