/*********************************************************************

  EVG-THIN v1.1: A Thinning Approximation to the Extended Voronoi Graph
  
  Copyright (C) 2006 - Patrick Beeson  (pbeeson@cs.utexas.edu)


  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301
  USA

*********************************************************************/


#ifndef utils_hh 
#define utils_hh 

#include <math.h>
#include <sys/time.h>


/**
   Euclidean distance between two points.
**/
inline float dist(float ax, float ay, float bx, float by) {
  return sqrt(pow(ax-bx,2)+pow(ay-by,2));
}


/**
   Class used for timing the length of pieces of code.
**/
class Time {
public:
  struct timeval tv;
  Time(){reset();}
  void reset(){
    gettimeofday( &tv, NULL);    
  }
  float get_since(){
    struct timeval t2;
    gettimeofday( &t2, NULL);    
    return (t2.tv_sec-tv.tv_sec) + 
      0.000001*(t2.tv_usec-tv.tv_usec);
  }
};

#endif
