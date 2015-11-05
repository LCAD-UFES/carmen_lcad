 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include <carmen/logtools.h>
#include <carmen/logtools_graphics.h>

YUV
rgb_to_yuv( RGB color )
{
  YUV ret;
  ret.y =  (0.257 * color.r) + (0.504 * color.g) + (0.098 * color.b) + 16;
  ret.u = -(0.148 * color.r) - (0.291 * color.g) + (0.439 * color.b) + 128;
  ret.v =  (0.439 * color.r) - (0.368 * color.g) - (0.071 * color.b) + 128;
  return(ret);
}

RGB
yuv_to_rgb( YUV color )
{
  RGB ret;
  ret.b = 1.164*(color.y-16)                       + 2.018*(color.u-128);
  ret.g = 1.164*(color.y-16) - 0.813*(color.v-128) - 0.391*(color.u-128);
  ret.r = 1.164*(color.y-16) + 1.596*(color.v-128);
  return(ret);
}

#define NO_HUE   -1

HSV
rgb_to_hsv( RGB color )
{
  HSV    ret;
  double
    max = MAX (color.r, MAX (color.g, color.b)),
    min = MIN (color.r, MIN (color.g, color.b));
  double delta = max - min;

  ret.h = 0;
  ret.v = max;
  if (max != 0.0)
    ret.s = delta / max;
  else
    ret.s = 0.0;
  if (ret.s == 0.0)
    ret.h = NO_HUE;
  else {
    if (color.r == max)
      ret.h = (color.g - color.b) / delta;
    else if (color.g == max)
      ret.h = 2 + (color.b - color.r) / delta;
    else if (color.b == max)
      ret.h = 4 + (color.r - color.g) / delta;
    ret.h *= 60.0;
    if (ret.h < 0)
      ret.h += 360.0;
    ret.h /= 360.0;
  }
  return(ret);
}

/*
HSV
rgb_to_hsv( RGB color )
{
  HSV ret;
  double minv, maxv, delta;
  
  minv = MIN3( color.r, color.g, color.b );
  ret.v = maxv = MAX3( color.r, color.g, color.b );
  
  delta = maxv - minv;
  
  if( maxv != 0 )
    ret.s = delta / maxv; 
  else {
    ret.s = 0;
    ret.h = -1;
    return(ret);
  }

  if( color.r == maxv )
    ret.h = ( color.g - color.b ) / delta;      
  else if( color.g == maxv )
    ret.h = 2 + ( color.b - color.r ) / delta;  
  else
    ret.h = 4 + ( color.r - color.g ) / delta;  
  
  ret.h *= 60;    
  if( ret.h < 0 )
    ret.h += 360;

  return(ret);
}
*/

RGB
hsv_to_rgb( HSV color )
{
   RGB ret = { 0.0, 0.0, 0.0 };
   int i;
   double aa, bb, cc, f;

  if (color.s == 0)
    ret.r = ret.g = ret.b = color.v;
  else {
    if (color.h == 1.0)
      color.h = 0;
    color.h *= 6.0;
    i = floor (color.h);
    f = color.h - i;
    aa = color.v * (1 - color.s);
    bb = color.v * (1 - (color.s * f));
    cc = color.v * (1 - (color.s * (1 - f)));
    switch (i) {
    case 0:
      ret.r = color.v;
      ret.g = cc;
      ret.b = aa;
      break;
    case 1:
      ret.r = bb;
      ret.g = color.v;
      ret.b = aa;
      break;
    case 2:
      ret.r = aa;
      ret.g = color.v;
      ret.b = cc;
      break;
    case 3:
      ret.r = aa;
      ret.g = bb;
      ret.b = color.v;
      break;
    case 4:
      ret.r = cc;
      ret.g = aa;
      ret.b = color.v;
      break;
    case 5:
      ret.r = color.v;
      ret.g = aa;
      ret.b = bb;
      break;
    }
  }
  return(ret);
}

/*
RGB
hsv_tp_rgb( HSV color )
{
  RGB ret;
  int i;
  double f, p, q, t;
  
  if( color.s == 0 ) {
    ret.r = ret.g = ret.b = color.v;
    return(ret);
  }
  
  color.h /= 60;                    
  i = floor( color.h );
  f = color.h - i;                  
  p = color.v * ( 1 - color.s );
  q = color.v * ( 1 - color.s * f );
  t = color.v * ( 1 - color.s * ( 1 - f ) );
  
  switch( i ) {
  case 0:
    ret.r = color.v;
    ret.g = t;
    ret.b = p;
    break;
  case 1:
    ret.r = q;
    ret.g = color.v;
    ret.b = p;
    break;
  case 2:
    ret.r = p;
    ret.g = color.v;
    ret.b = t;
    break;
  case 3:
    ret.r = p;
    ret.g = q;
    ret.b = color.v;
    break;
  case 4:
    ret.r = t;
    ret.g = p;
    ret.b = color.v;
    break;
  default:             
    ret.r = color.v;
    ret.g = p;
    ret.b = q;
    break;
  }
  return(ret);
  
}
*/

RGB
val_to_rgb( double val )
{
  HSV color = {1.0, 1.0, 1.0};

  /* cut to range [0.0,1.0] */
  val = MIN( 1.0, MAX( 0.0, val ) );

  /* the gradient is done by changing hue between blue and yellow */
  if (val>0.1) {
    //    color.h = fmod(0.555*val+.66666666,1.0);
    color.h = fmod(0.5*val+.66666666,1.0);
  } else {
    /* if the val is smaller than 10% */
    color.h = .66666666;
    color.s = 1.0;
    color.v = val * 10.0;
  }
  
  return( hsv_to_rgb( color ) );
}

RGB
val_to_gray( double val )
{
  RGB color;
  
  /* cut to range [0.0,1.0] */
  val = MIN( 1.0, MAX( 0.0, val ) );

  color.r = val;
  color.g = val;
  color.b = val;

  return( color );
}
