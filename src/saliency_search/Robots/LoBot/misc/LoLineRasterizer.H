/**
   \file  Robots/LoBot/misc/LoLineRasterizer.H
   \brief Bresenham's line rasterization algorithm.

   This file defines a function that rasterizes a line given its two end
   points. The whole thing works entirely with integer math and ought to
   be a fairly quick way of performing ray casting operations (for
   example).
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoLineRasterizer.H $
// $Id: LoLineRasterizer.H 13445 2010-05-21 06:12:57Z mviswana $
//

#ifndef LOBOT_LINE_RASTERIZER_DOT_H
#define LOBOT_LINE_RASTERIZER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/LoMath.H"

// Standard C++ headers
#include <algorithm>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \function lobot::rasterize_line
   \brief    Rasterize a line using Bresenham's algorithm.

   This function implements Bresenham's line rasterization algorithm to
   compute the pixels that lie between two points (x0, y0) and (x1, y1).
   For each pixel along the way, it will invoke the set_pixel function,
   which is to be supplied by the client. This function will be passed
   two parameters, viz., the x and y coordinates of the pixel being
   visited. As the set_pixel function is a template parameter, it can
   also be a function object rather than a function.

   This implementation of Bresenham's line algorithm also includes an
   early exit test. If the client-supplied set_pixel visitor function
   returns false, the algorithm will terminate. This feature is useful in
   a ray casting/collision detection context. Thus, in addition to taking
   two integers, the client-supplied set_pixel visitor function is also
   expected to return a bool to indicate whether the rasterization should
   continue on to the next pixel or not.

   NOTE: This implementation is a slightly modified version of the
   optimized, integer math based one described on Wikipedia. The
   modification ensures that we *always* start at (x0, y0) and rasterize
   towards (x1, y1). This is required when the algorithm is employed in a
   ray casting context (which is what it is in fact used for in
   Robolocust). The early exit test is also an addition in this version.
*/
template<typename visitor_fn>
void rasterize_line(int x0, int y0, int x1, int y1, visitor_fn set_pixel)
{
   bool steep = abs(y1 - y0) > abs(x1 - x0) ;
   if (steep) {
      std::swap(x0, y0) ;
      std::swap(x1, y1) ;
   }

   int deltax = abs(x1 - x0) ;
   int deltay = abs(y1 - y0) ;
   int error  = deltax/2 ;
   int ystep  = (y0 < y1) ? 1 : -1 ;
   int xstep ;
   if (x0 < x1)
   {
      xstep = 1 ;
      ++x1 ; ++y1 ;
   }
   else
   {
      xstep = -1 ;
      --x1 ; --y1 ;
   }

   int y = y0 ;
   for (int x = x0; x != x1; x += xstep)
   {
      bool cont = true ;
      if (steep)
         cont = set_pixel(y, x) ;
      else
         cont = set_pixel(x, y) ;
      if (! cont)
         break ;
      error -= deltay ;
      if (error < 0) {
         y += ystep ;
         error += deltax ;
      }
   }
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
