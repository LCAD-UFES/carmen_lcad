/*!@file Image/PixelsCommonDef.H Basic pixel types version 2.0 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PixelsCommonDef.H $
// $Id: PixelsCommonDef.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef PIXELSCOMMONDEF_H_DEFINED
#define PIXELSCOMMONDEF_H_DEFINED

#include "Image/PixelsBase.H"

// ######################################################################
// ######################################################################
// Commonly used conversion template defs
//
// Use this file to place pixels type conversions IFF you use them more
// than once. For instance HSV and RGB conversions are used in several
// places. So to reduce redundancy, they are placed here
//
// ######################################################################
// ######################################################################

// ######################################################################
/*! RGB to HSV

   RGB is converted from HSV in a simple way

   I. Get some easy work done:
     (1) If Value V = 0, then we are done, color is black set R,G,B to 0
     (2) If Saturation S = 0, no color is dominant, set to some gray color

   II. Hue is valued from 0 to 360, we chunk the space into 60 degree
       increments. At each 60 degrees we use a slightly different formula.
       In general we will assign and set R,G and B exclusively as:

       (1) We set the most dominant color:

          (a) If H is 300 -> 60 , set R = V
          (b) If H is 60  -> 180, set G = V
          (c) If H is 180 -> 300, set B = V

       (2) The least dominant color is set as:

          pv = Value * ( 1 - Saturation )

       (3) The last remaining color is set as:

          Either:

          (a) qv = Value * ( 1 - Saturation * (Hue/60) - floor(Hue/60))
          (b) tv = Value * ( 1 - Saturation * ( 1 - ((Hue/60) - floor(Hue/60))))

  III. Clean up, here we allow for i to be -1 or 6 just in case we have a
       very small floating point error otherwise, we have an undefined input.

  IV. Normalize R,G and B from 0 to 255.

  Note: S and V should be normalized between 0 and 1 prior to using this
        template. H should be its normal 0 to 360. You can do this using
        the built in limit definitions:

        const double new_v = v / HSV_V_UPPER;  // Normalize from 0 to 1
        const double new_s = s / HSV_S_UPPER;  // Normalize from 0 to 1
*/
// ######################################################################
#define PIX_HSV_TO_RGB_COMMON(H,S,V,R,G,B)                          \
if( V == 0 )                                                        \
{ R = 0; G = 0; B = 0; }                                            \
else if( S == 0 )                                                   \
{                                                                   \
  R = clamped_convert<T>(V);                                        \
  G = clamped_convert<T>(V);                                        \
  B = clamped_convert<T>(V);                                        \
}                                                                   \
else                                                                \
{                                                                   \
  const double hf = H / 60.0;                                       \
  const int    i  = (int) floor( hf );                              \
  const double f  = hf - i;                                         \
  const double pv  = V * ( 1 - S );                                 \
  const double qv  = V * ( 1 - S * f );                             \
  const double tv  = V * ( 1 - S * ( 1 - f ) );                     \
  switch( i )                                                       \
    {                                                               \
    case 0:                                                         \
      R = clamped_convert<T>(V);                                    \
      G = clamped_convert<T>(tv);                                   \
      B = clamped_convert<T>(pv);                                   \
      break;                                                        \
    case 1:                                                         \
      R = clamped_convert<T>(qv);                                   \
      G = clamped_convert<T>(V);                                    \
      B = clamped_convert<T>(pv);                                   \
      break;                                                        \
    case 2:                                                         \
      R = clamped_convert<T>(pv);                                   \
      G = clamped_convert<T>(V);                                    \
      B = clamped_convert<T>(tv);                                   \
      break;                                                        \
    case 3:                                                         \
      R = clamped_convert<T>(pv);                                   \
      G = clamped_convert<T>(qv);                                   \
      B = clamped_convert<T>(V);                                    \
      break;                                                        \
    case 4:                                                         \
      R = clamped_convert<T>(tv);                                   \
      G = clamped_convert<T>(pv);                                   \
      B = clamped_convert<T>(V);                                    \
      break;                                                        \
    case 5:                                                         \
      R = clamped_convert<T>(V);                                    \
      G = clamped_convert<T>(pv);                                   \
      B = clamped_convert<T>(qv);                                   \
      break;                                                        \
    case 6:                                                         \
      R = clamped_convert<T>(V);                                    \
      G = clamped_convert<T>(tv);                                   \
      B = clamped_convert<T>(pv);                                   \
      break;                                                        \
    case -1:                                                        \
      R = clamped_convert<T>(V);                                    \
      G = clamped_convert<T>(pv);                                   \
      B = clamped_convert<T>(qv);                                   \
      break;                                                        \
    default:                                                        \
      LFATAL("i Value error in Pixel conversion, Value is %d",i);   \
      break;                                                        \
    }                                                               \
}                                                                   \
R *= RGB_R_UPPER;                                                   \
G *= RGB_G_UPPER;                                                   \
B *= RGB_B_UPPER;

// ######################################################################
/*! HSV to RGB

   HSV is converted from RGB in a simple way

   I. First compute the base H,S and V

     (1) H - Hue if Color1 is Max then H = ( Color2 - Color3 ) / ( Max - Min )
     (2) S - Saturation is S = Max - Min / Max;
     (3) V - Value is V = max of either R,G or B

   II. Normalize the values

     General H,S,V has ranges:
     0 < Hue < 360
     0 < Sat < 100
     0 < Val < 255

     (1) H - Hue:
             if Red           H *= 60
             if Green H += 2; H *= 60
             if Blue  H += 4; H *= 60
         In essence this forces the recognizable 0-360 value seen in hue
     (2) S - May be S *= 100 , I like in some cases to keep it from 0 to 1
     (3) V - is V *= 255

   there are some other bells and whistles to take care of divide by zero
   conditions and other such singularities. For instance if Max = Min then
   we need to fudge hue

   Note: Set NORM to true for normal HSV conversions. This will make the
         values use normal HSV range. Setting NORM to false forces H,S and V
         to range between 0 and 1. This is useful if next you will convert
         to H2SV color space.

         R, G and B are assumed to be between 0 and 255.
*/
// ######################################################################
#define PIX_RGB_TO_HSV_COMMON(R,G,B,H,S,V,NORM)                        \
if((B > G) && (B > R))                                                 \
{                                                                      \
  V = clamped_convert<T>(B);                                           \
  if(V != 0)                                                           \
  {                                                                    \
    double min;                                                        \
    if(R > G) min = G;                                                 \
    else      min = R;                                                 \
    const double delta = V - min;                                      \
    if(delta != 0)                                                     \
      { S = clamped_convert<T>(delta/V); H = 4 + (R - G) / delta; }    \
    else                                                               \
      { S = clamped_convert<T>(0.0);     H = 4 + (R - G); }            \
    H *=   60; if(H < 0) H += 360;                                     \
    if(!NORM) V = clamped_convert<T>(V/255);                           \
    else      S *= clamped_convert<T>(100);                            \
  }                                                                    \
  else                                                                 \
    { S = 0; H = 0;}                                                   \
}                                                                      \
else if(G > R)                                                         \
{                                                                      \
  V = clamped_convert<T>(G);                                           \
  if(V != 0)                                                           \
  {                                                                    \
    double min;                                                        \
    if(R > B) min = B;                                                 \
    else      min = R;                                                 \
    const double delta = V - min;                                      \
    if(delta != 0)                                                     \
      { S = clamped_convert<T>(delta/V); H = 2 + (B - R) / delta; }    \
    else                                                               \
      { S = clamped_convert<T>(0.0);     H = 2 + (B - R); }            \
    H *=   60; if(H < 0) H += 360;                                     \
    if(!NORM) V = clamped_convert<T>(V/255);                           \
    else      S *= clamped_convert<T>(100);                            \
  }                                                                    \
  else                                                                 \
    { S = 0; H = 0;}                                                   \
}                                                                      \
else                                                                   \
{                                                                      \
  V = clamped_convert<T>(R);                                           \
  if(V != 0)                                                           \
  {                                                                    \
    double min;                                                        \
    if(G > B) min = B;                                                 \
    else      min = G;                                                 \
    const double delta = V - min;                                      \
    if(delta != 0)                                                     \
      { S = clamped_convert<T>(delta/V); H = (G - B) / delta; }        \
    else                                                               \
      { S = clamped_convert<T>(0.0);     H = (G - B); }                \
    H *=   60; if(H < 0) H += 360;                                     \
    if(!NORM) V = clamped_convert<T>(V/255);                           \
    else      S *= clamped_convert<T>(100);                            \
  }                                                                    \
  else                                                                 \
    { S = 0; H = 0;}                                                   \
}

// ######################################################################
/*! HSV to H2SV1
  We convert HSV to H2SV1 (or H2SV2 etc) by converting hue which is
  in radial coordinates 0 - 360 and converting it to cartesian coordinates

  The precise coordinate transform is flexable, so long as it is invertable
  here, we treat r (hue) as falling along a straight line. We could also in
  another method treat it as if it is on a circle.

  --> All final values normalize from 0 to 1
*/
// ######################################################################
#define PIX_HSV_TO_H2SV1_COMMON(H,S,H1,H2)                     \
if(S == 0)                                                     \
{                                                              \
  H1 = clamped_convert<T>(0.5);                                \
  H2 = clamped_convert<T>(0.5);                                \
}                                                              \
else                                                           \
{                                                              \
  if(H > 180)                                                  \
  {                                                            \
    H2 = clamped_convert<T>((H - 180)/180);                    \
    if(H > 270) H1 = clamped_convert<T>((H - 270)/180);        \
    else        H1 = clamped_convert<T>(1 - (H - 90)/180);     \
  }                                                            \
  else                                                         \
  {                                                            \
    H2 = clamped_convert<T>(1 - H/180);                        \
    if(H > 90) H1 = clamped_convert<T>(1 - (H - 90)/180);      \
    else       H1 = clamped_convert<T>(0.5 + H/180);           \
  }                                                            \
}

// ######################################################################
/*! HSV to H2SV2
  We convert HSV to H2SV1 (or H2SV2 etc) by converting hue which is
  in radial coordinates 0 - 360 and converting it to cartesian coordinates

  The precise coordinate transform is flexable, so long as it is invertable
  here, we treat r (hue) as falling along a straight line. We could also in
  another method treat it as if it is on a circle.

  This variant is designed to make H1 and H2 mimmic R/G B/Y opponencies
  Thus:

     (1) H1 is 0 at Blue and 1 at Yellow
     (2) H2 is 0 at Green and 1 at Red

  --> All final values normalize from 0 to 1

  Note: We can also use these templates on HSL color space since
        HSV and HSL have the exact same basis for hue.

*/
// ######################################################################
#define PIX_HSV_TO_H2SV2_COMMON(H,S,H1,H2)                                \
if(S == 0)                                                                \
{                                                                        \
  H1 = clamped_convert<T>(0.5);                                                                \
  H2 = clamped_convert<T>(0.5);                                                                \
}                                                                        \
else                                                                        \
{                                                                        \
  if(H > 120)                                                                \
  {                                                                        \
    H2 = clamped_convert<T>((H - 120)/240);                                \
    if(H > 240) H1 = clamped_convert<T>((H - 240)/180);                        \
    else        H1 = clamped_convert<T>(1 - (H - 60)/180);                \
  }                                                                        \
  else                                                                        \
  {                                                                        \
    H2 = clamped_convert<T>(1 - H/120);                                        \
    if(H > 60) H1 = clamped_convert<T>(1 - (H - 60)/180);                \
    else       H1 = clamped_convert<T>((2.0/3.0) + H/180);                \
  }                                                                        \
}

// ######################################################################
/*! H2SV1 to HSV NORMAL
    Convert H2SV1 to HSV using a simple quick method.
*/
#define PIX_H2SV1_TO_HSV_SIMPLE_COMMON(H1,H2,H)           \
if(H1 > 0.5)                                              \
  if(H2 > 0.5)  H = 180 * H1 - 90;                        \
  else          H = 90 + 180 * (1 - H1);                  \
else                                                      \
  if(H2 <= 0.5) H = 90 + 180 * (1 - H1);                  \
  else          H = 270 + 180 * H1;


// ######################################################################
/*! H2SV2 to HSV NORMAL
    Convert H2SV2 to HSV using a simple quick method.
*/
#define PIX_H2SV2_TO_HSV_SIMPLE_COMMON(H1,H2,H)           \
if(H1 > 2.0/3.0)                                          \
  if(H2 > 0.5)  H = 180 * H1 - 120;                       \
  else          H = 60 + 180 * (1 - H1);                  \
else                                                      \
  if(H2 <= 0.5) H = 60 + 180 * (1 - H1);                  \
  else          H = 240 + 180 * H1;


// ######################################################################
/*! H2SV2 to HSV ROBUST
    Convert H2SV2 to HSV using a robust method that allows us to deal with
    H1 and H2 being adjusted independantly. If they are never adjusted
    independantly, then use the simple version

    The main difference here from simple is that:
    (1) We compute xp and replace H1 with it
    (2) We compute a new saturation term S_NEW

    What makes this robust is that the H1 and H2 coordinates do not have to
    match up with the orignal unit coordinates. Instead we compute a slope
    term and figure out where it should intersect the original unit coordinates

    Saturation may be reduced if we are too far from a viable color coordinate
    and have to transform to much.


    General Computations:
    m:    The slope of the H1/H2 lines
    xp:   The Adjusted H1 (x value)
          - Useful if H1 was adjusted indepentant of H2
    yp:   The Adjusted H2 (y value)
    d:    The difference between the ideal (xp,yp) value and the real (H1,H2)
    ad:   The legth of (xp,yp) from the origen
    sfac: Adjustment to be applied to saturation if d is large
           - Useful if H1 and H2 are close to the origen, thus
             saturation should be reduced.
*/
// ######################################################################

#define PIX_H2SV2_TO_HSV_ROBUST_COMMON(H1,H2,S,H,S_NEW)         \
double sfac    = 1;                                             \
const double x = H1 - 2.0/3.0;                                  \
if(x > 0)                                                       \
{                                                               \
  const double y = H2 - 0.5;                                    \
  const double m = y/x;                                         \
  if(y > 0)                                                     \
  {                                                             \
    const double xp = 0.5/(m + 3.0/2.0);                        \
    if(xp > x)                                                  \
    {                                                           \
      const double yp = 0.5 - 0.5*xp/(1.0/3.0);                 \
      const double d  = sqrt(pow(xp - x,2) + pow(yp - y,2));    \
      const double ad = sqrt(pow(xp,2)     + pow(yp,2));        \
      sfac            = (ad - d)/ad;                            \
    }                                                           \
    H = 180 * (xp + 2.0/3.0) - 120;                             \
  }                                                             \
  else                                                          \
  {                                                             \
    const double xp = -0.5/(m - 3.0/2.0);                       \
    if(xp > x)                                                  \
    {                                                           \
      const double yp = -1.0 * (0.5 - 0.5*xp/(1.0/3.0));        \
      const double d  = sqrt(pow(xp - x,2) + pow(yp - y,2));    \
      const double ad = sqrt(pow(xp,2)     + pow(yp,2));        \
      sfac            = (ad - d)/ad;                            \
    }                                                           \
    H = 60 + 180 * (1 - (xp + 2.0/3.0));                        \
  }                                                             \
}                                                               \
else                                                            \
{                                                               \
  if(x != 0)                                                    \
  {                                                             \
    const double y = H2 - 0.5;                                  \
    const double m = y/x;                                       \
    if(y <= 0)                                                  \
    {                                                           \
      const double xp = -0.5/(m + 0.5/(2.0/3.0));               \
      if(xp < x)                                                \
      {                                                         \
        const double yp = -1.0 * (0.5 + 0.5*xp/(2.0/3.0));      \
        const double d  = sqrt(pow(xp - x,2) + pow(yp - y,2));  \
        const double ad = sqrt(pow(xp,2)     + pow(yp,2));      \
        sfac            = (ad - d)/ad;                          \
      }                                                         \
      H = 60 + 180 * (1 - (xp + 2.0/3.0));                      \
    }                                                           \
    else                                                        \
    {                                                           \
      const double xp = 0.5/(m - 0.5/(2.0/3.0));                \
      if(xp < x)                                                \
      {                                                         \
        const double yp = 0.5 + 0.5*xp/(2.0/3.0);               \
        const double d  = sqrt(pow(xp - x,2) + pow(yp - y,2));  \
        const double ad = sqrt(pow(xp,2)     + pow(yp,2));      \
        sfac            = (ad - d)/ad;                          \
      }                                                         \
      H = 240 + 180 * (xp + 2.0/3.0);                           \
    }                                                           \
  }                                                             \
  else                                                          \
    H = 240;                                                    \
}                                                               \
S_NEW = S * sfac;



// ######################################################################
/*! RGB to OpenCV XYZ

  This is the XYZ Color space from CIE with the matrix used by OpenCV.
  It is very similar to the original one developed by CIE in 1931.

*/
// ######################################################################
#define PIX_RGB_TO_XYZ_OPENCV_COMMON(R,G,B,X,Y,Z)                 \
X = R*0.412453F + G*0.357580F + B*0.180423F;                      \
Y = R*0.212671F + G*0.715160F + B*0.072169F;                      \
Z = R*0.019334F + G*0.119193F + B*0.950227F;

// ######################################################################
/*! RGB to CIE 1931 XYZ

  This is the original CIE XYZ colorspace developed in 1931.
*/
// ######################################################################
#define PIX_RGB_TO_XYZ_CIE1931_COMMON(R,G,B,X,Y,Z)                \
X = R*0.49F   + G*0.31F   + B*0.2F;                               \
Y = R*0.1769F + G*0.8124F + B*0.0107F;                            \
Z = R*0.0F    + G*0.0099F + B*0.9901F;

// ######################################################################
/*! OpenCV XYZ to RGB

  This is the XYZ Color space from CIE with the matrix used by OpenCV.
  It is very similar to the original one developed by CIE in 1931.

*/
// ######################################################################
#define PIX_XYZ_TO_RGB_OPENCV_COMMON(X,Y,Z,R,G,B)                 \
R =       X*3.240479F - Y*1.53715F  - Z*0.498535F;                \
G = -1.0F*X*0.969256F + Y*1.875991F + Z*0.041556F;                \
B =       X*0.055648F - Y*0.204043F + Z*1.057311F;

// ######################################################################
/*! XYZ to CIE Lab - OpenCV XYZ

  This is the Lab conversion using OpenCV XYZ color space as a basis.
  Use the OpenCV XYZ and Lab in conjunction to create the OpenCV version
  of Lab Color space.

  L - Is a chroma measure that encapsulates saturation and intensity.
  a - Is basically a Red/Green color opponent.
  b - Is basically a Blue/Yellow color opponent

*/
#define A_GAMUT_RANGE  186.0F
#define A_GAMUT_MIN   -87.0F
#define B_GAMUT_RANGE  203.0F
#define B_GAMUT_MIN   -108.0F

// ######################################################################
#define PIX_XYZ_TO_LAB_OPENCV_COMMON(X,Y,Z,L,A,B)                        \
const double i  = 1.0F /3.0F;                                            \
const double n  = 16.0F/116.0F;                                          \
const double t  = 0.00885645F;                                           \
const double s  = 7.78704F;                                              \
const double Xn = 255.0F*0.412453F + 255.0F*0.357580F + 255.0F*0.180423F;\
const double Yn = 255.0F*0.212671F + 255.0F*0.715160F + 255.0F*0.072169F;\
const double Zn = 255.0F*0.019334F + 255.0F*0.119193F + 255.0F*0.950227F;\
const double X1 = X/Xn;                                                  \
const double Y1 = Y/Yn;                                                  \
const double Z1 = Z/Zn;                                                  \
double X2,Y2,Z2;                                                         \
if(X1 > t) X2 = pow(X1,i);                                               \
else       X2 = s*X1 + n;                                                \
if(Y1 > t) Y2 = pow(Y1,i);                                               \
else       Y2 = s*Y1 + n;                                                \
if(Z1 > t) Z2 = pow(Z1,i);                                               \
else       Z2 = s*Z1 + n;                                                \
L = (116.0F * Y2 - 16.0F) * 255.0F/100.0F;                                 \
A = ((500.0F * (X2 - Y2) - A_GAMUT_MIN)/A_GAMUT_RANGE)*255.0F;                 \
B = ((200.0F * (Y2 - Z2) - B_GAMUT_MIN)/B_GAMUT_RANGE)*255.0F;

// ######################################################################
/*! XYZ to CIE Lab - CIE 1931

  This is the Lab conversion using CIE 1931 XYZ color space as a basis.
  Use the CIE 1931 XYZ and Lab in conjunction to create the basic CIE version
  of Lab Color space.

  L - Is intensity.
  a - Is basically a Red/Green color opponent.
  b - Is basically a Blue/Yellow color opponent
*/
// ######################################################################
#define PIX_XYZ_TO_LAB_CIE1931_COMMON(X,Y,Z,L,A,B)                         \
const double i  = 1.0F /3.0F;                                            \
const double n  = 16.0F/116.0F;                                          \
const double t  = 0.00885645F;                                           \
const double s  = 7.78704F;                                              \
const double Xn = 255.0F*0.49F   + 255.0F*0.31F   + 255.0F*0.2F;         \
const double Yn = 255.0F*0.1769F + 255.0F*0.8124F + 255.0F*0.0107F;      \
const double Zn = 255.0F*0.0F    + 255.0F*0.0099F + 255.0F*0.9901F;      \
const double X1 = X/Xn;                                                  \
const double Y1 = Y/Yn;                                                  \
const double Z1 = Z/Zn;                                                  \
double X2,Y2,Z2;                                                         \
if(X1 > t) X2 = pow(X1,i);                                               \
else       X2 = s*X1 + n;                                                \
if(Y1 > t) Y2 = pow(Y1,i);                                               \
else       Y2 = s*Y1 + n;                                                \
if(Z1 > t) Z2 = pow(Z1,i);                                               \
else       Z2 = s*Z1 + n;                                                \
L = (116.0F * Y2 - 16.0F) * 255.0F/100.0F;                                 \
A = (500.0F * (X2 - Y2)) + 128.0F;                                         \
B = (200.0F * (Y2 - Z2)) + 128.0F;
// ######################################################################
/*! XYZ to CIE Lab - Augmented to Favor Red (Experimental)

  This is the Lab conversion using OpenCV XYZ color space as a basis.
  Use the OpenCV XYZ and Lab in conjunction to create the OpenCV version
  of Lab Color space.

  L - Is a chroma measure that encapsulates saturation and intensity.
  a - Is basically a Red/Green color opponent.
  b - Is basically a Blue/Yellow color opponent

*/
// ######################################################################
#define PIX_XYZ_TO_LAB_AUGMENT_COMMON(X,Y,Z,L,A,B)                         \
const double i  = 1.0F /3.0F;                                            \
const double n  = 16.0F/116.0F;                                          \
const double t  = 0.00885645F;                                           \
const double s  = 7.78704F;                                              \
const double Xn = 255.0F*0.412453F + 255.0F*0.357580F + 255.0F*0.180423F;\
const double Yn = 255.0F*0.212671F + 255.0F*0.715160F + 255.0F*0.072169F;\
const double Zn = 255.0F*0.019334F + 255.0F*0.119193F + 255.0F*0.950227F;\
const double X1 = X/Xn;                                                  \
const double Y1 = Y/Yn;                                                  \
const double Z1 = Z/Zn;                                                  \
double X2,Y2,Z2;                                                         \
if(X1 > t) X2 = pow(X1,i);                                               \
else       X2 = s*X1 + n;                                                \
if(Y1 > t) Y2 = pow(Y1,i);                                               \
else       Y2 = s*Y1 + n;                                                \
if(Z1 > t) Z2 = pow(Z1,i);                                               \
else       Z2 = s*Z1 + n;                                                \
L = (116.0F * Y2 - 16.0F) * 255.0F/100.0F;                                 \
if(Y2 > X2) A = (500.0F * (X2 - Y2)) + 128.0F;                                 \
else        A = (500.0F * (X2 - Y2)) + 128.0F;                                 \
B = (200.0F * (Y2 - Z2)) + 128.0F;

// ######################################################################
/*! Lab to CIE XYZ - OpenCV XYZ

  This is the Lab conversion using OpenCV XYZ color space as a basis.
  Use the OpenCV XYZ and Lab in conjunction to create the OpenCV version
  of Lab Color space.

  L - Is intensity.
  a - Is basically a Red/Green color opponent.
  b - Is basically a Blue/Yellow color opponent

*/
// ######################################################################

#define PIX_LAB_TO_XYZ_OPENCV_COMMON(L,A,B,X,Y,Z)                         \
const double LL = L * 100.0F/255.0F;                                         \
const double AA = ((A/255.0F) * A_GAMUT_RANGE + A_GAMUT_MIN);                 \
const double BB = ((B/255.0F) * B_GAMUT_RANGE + B_GAMUT_MIN);                 \
const double d  = 6.0F/29.0F;                                                 \
const double ds = d*d;                                                         \
const double n  = 16.0F/116.0F;                                          \
const double Xn = 255.0F*0.412453F + 255.0F*0.357580F + 255.0F*0.180423F;\
const double Yn = 255.0F*0.212671F + 255.0F*0.715160F + 255.0F*0.072169F;\
const double Zn = 255.0F*0.019334F + 255.0F*0.119193F + 255.0F*0.950227F;\
const double Fy = (LL + 16)/116;                                         \
const double Fx = Fy + AA/500.0F;                                        \
const double Fz = Fy - BB/200.0F;                                        \
if(Fy > d) Y = Yn*pow(Fy,3);                                             \
else       Y = (Fy - n)*3*ds*Yn;                                         \
if(Fx > d) X = Xn*pow(Fx,3);                                             \
else       X = (Fx - n)*3*ds*Xn;                                         \
if(Fz > d) Z = Zn*pow(Fz,3);                                             \
else       Z = (Fz - n)*3*ds*Zn;                                         \

// ######################################################################
/*! Lab to CIE XYZ - CIE XYZ

  This is the Lab conversion using OpenCV XYZ color space as a basis.
  Use the OpenCV XYZ and Lab in conjunction to create the OpenCV version
  of Lab Color space.

  L - Is intensity.
  a - Is basically a Red/Green color opponent.
  b - Is basically a Blue/Yellow color opponent

*/
// ######################################################################
#define PIX_LAB_TO_XYZ_CIE1931_COMMON(L,A,B,X,Y,Z)                       \
const double LL  = L * 100.0F/255.0F;                                    \
const double AA  = A - 128.0F;                                           \
const double BB  = B - 128.0F;                                           \
const double d  = 6.0F/29.0F;                                            \
const double ds = d*d;                                                   \
const double n  = 16.0F/116.0F;                                          \
const double Xn = 255.0F*0.49F   + 255.0F*0.31F   + 255.0F*0.2F;         \
const double Yn = 255.0F*0.1769F + 255.0F*0.8124F + 255.0F*0.0107F;      \
const double Zn = 255.0F*0.0F    + 255.0F*0.0099F + 255.0F*0.9901F;      \
const double Fy = (LL + 16)/116;                                         \
const double Fx = Fy + AA/500.0F;                                        \
const double Fz = Fy - BB/200.0F;                                        \
if(Fy > d) Y = Yn*pow(Fy,3);                                             \
else       Y = (Fy - n)*3*ds*Yn;                                         \
if(Fx > d) X = Xn*pow(Fx,3);                                             \
else       X = (Fx - n)*3*ds*Xn;                                         \
if(Fz > d) Z = Zn*pow(Fz,3);                                             \
else       Z = (Fz - n)*3*ds*Zn;                                         \

#endif
