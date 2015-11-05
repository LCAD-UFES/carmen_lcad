/**
   \file  Robots/LoBot/slam/LoCoords.C
   \brief This file defines the functions in the lobot::Coords namespace.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoCoords.C $
// $Id: LoCoords.C 13560 2010-06-11 12:58:57Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoCoords.H"
#include "Robots/LoBot/slam/LoSlamParams.H"
#include "Robots/LoBot/util/LoMath.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot  {
namespace Coords {

//------------------- COORDINATE SYSTEM CONVERSIONS ---------------------

// The following function converts real or physical robot coordinates
// into grid coordinates. The occupancy grid is a discretized version of
// the underlying real/world coordinate system. The world coordinate
// system goes from left (L) to right (R) in the x direction and B
// (bottom) to T (top) in the y direction. The occupancy grid OTOH
// extends from 0 to W-1 in x and 0 to H-1 in y with the origin being at
// the top-left corner of the grid.
//
// Thus, to convert from world coordinates to grid coordinates, we apply
// the following formulae:
//
//           x - L     gx  - 0           y - B     gy - H-1
//           -----  =  -------    and    -----  =  --------
//           R - L     W-1 - 0           T - B     0  - H-1
//
// Rearranging the terms in the above two equations and simplifying gives
// us:
//
//               (x - L)(W - 1)              (T - y)(H - 1)
//          gx = --------------   and   gy = --------------
//                  (R - L)                     (T - B)
//
// If we let Sx = (W - 1)/(R - L) and Sy = (H - 1)/(T - B), then the
// above expressions become:
//          gx = (x - L) * Sx
//          gy = (T - y) * Sy
//
// The following function uses the above two equations to convert world
// coordinates (x, y) to grid coordinates (gx, gy).
//
// As an aside, we note that the grid's width W = (R - L)/S and its
// height H = (T - B)/S where S is the size of each cell. For example, if
// the map extents are [0 5000 -2500 2500] and cell size is 10mm, then
// the grid will have (5000 - 0)/10 = 500 cells in the x direction and
// (2500 - (-2500))/10 = 500 cells in the y direction.
//
// Substituting W = (R - L)/S and H = (T - B)/S in the expressions for Sx
// and Sy yields the following:
//
//                     1     1                1     1
//                Sx = - - -----   and   Sy = - - -----
//                     S   R - L              S   T - B
//
// where S is the cell size.
//
// The above expressions are used in the MapParams constructor for
// initializing Sx and Sy.
void to_grid(float x, float y, int* gx, int* gy)
{
   const int W = SlamParams::map_width() ;
   const int H = SlamParams::map_height();

   float L, T ;
   SlamParams::map_extents(&L, 0, 0, &T) ;
   const float Sx = SlamParams::Sx() ;
   const float Sy = SlamParams::Sy() ;

   *gx = clamp(round((x - L) * Sx), 0, W - 1) ;
   *gy = clamp(round((T - y) * Sy), 0, H - 1) ;
}

// This function converts grid coordinates to world coordinates. We
// simply rearrange the terms in:
//          gx = (x - L) * Sx
//          gy = (T - y) * Sy
// to get:
//           x = L + gx/Sx
//           y = T - gy/Sy
void to_real(int gx, int gy, float* x, float* y)
{
   float L, R, B, T ;
   SlamParams::map_extents(&L, &R, &B, &T) ;
   const float Sx = SlamParams::Sx() ;
   const float Sy = SlamParams::Sy() ;

   *x = clamp(L + gx/Sx, L, R) ;
   *y = clamp(T - gy/Sy, B, T) ;
}

//-----------------------------------------------------------------------

} // end of Coords namespace
} // end of lobot  namespace

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
