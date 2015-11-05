/**
   \file  Robots/LoBot/slam/LoOccGrid.C
   \brief This file defines the non-inline member functions of the
   lobot::OccGrid class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoOccGrid.C $
// $Id: LoOccGrid.C 13560 2010-06-11 12:58:57Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoOccGrid.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/slam/LoCoords.H"
#include "Robots/LoBot/slam/LoSlamParams.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/misc/singleton.hh"

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <fstream>
#include <string>
#include <algorithm>
#include <functional>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// Init "empty" occupancy grid
OccGrid::OccGrid()
{
   const int N = SlamParams::map_width() * SlamParams::map_height() ;
   m_grid.reserve(N) ;
   std::fill_n(std::back_inserter(m_grid), N, 0.0f) ;

   /*
   LERROR("created empty OccGrid [%08lX]",
          reinterpret_cast<unsigned long>(this)) ;
   // */
}

// Init occupancy grid using known obstacle map
OccGrid::OccGrid(const std::string& map_file_name)
{
   const int N = SlamParams::map_width() * SlamParams::map_height() ;
   m_grid.reserve(N) ;

   // First, mark all cells vacant
   std::fill_n(std::back_inserter(m_grid), N, prob_to_log_odds(0.01f)) ;

   // Then, read map file and mark those cells as occupied
   std::ifstream map_file(map_file_name.c_str()) ;
   const float occupied = prob_to_log_odds(0.99f) ;
   for (;;)
   {
      float x0, y0, x1, y1 ;
      map_file >> x0 >> y0 >> x1 >> y1 ;
      if (! map_file)
         break ;

      int gx0, gy0, gx1, gy1 ;
      Coords::to_grid(x0, y0, &gx0, &gy0) ;
      Coords::to_grid(x1, y1, &gx1, &gy1) ;

      if (gx0 > gx1)
         std::swap(gx0, gx1) ;
      if (gy0 > gy1)
         std::swap(gy0, gy1) ;

      const int W = SlamParams::map_width() ;
      int k = gy0 * W + gx0 ;
      for (int y = gy0; y <= gy1; ++y, k += W - (gx1 - gx0 + 1))
      for (int x = gx0; x <= gx1; ++x, ++k)
         m_grid[k] = occupied ;
   }
   /*
   LERROR("created OccGrid [%08lX] from map file \"%s\"",
          reinterpret_cast<unsigned long>(this), map_file_name.c_str()) ;
   // */
}

// Copy
OccGrid::OccGrid(const OccGrid& g)
   : m_grid(g.m_grid)
{
   /*
   LERROR("copied OccGrid [%08lX] to [%08lX]",
          reinterpret_cast<unsigned long>(&g),
          reinterpret_cast<unsigned long>(this)) ;
   // */
}

// Assignment
OccGrid& OccGrid::operator=(const OccGrid& g)
{
   if (&g != this) {
      m_grid = g.m_grid ;
      /*
      LERROR("assigned OccGrid [%08lX] to [%08lX]",
             reinterpret_cast<unsigned long>(&g),
             reinterpret_cast<unsigned long>(this)) ;
      // */
   }
   return *this ;
}

//-------------------------- MAP OPERATIONS -----------------------------

// Add two maps together and retain result in this map
OccGrid& OccGrid::operator+=(const OccGrid& M)
{
   std::transform(m_grid.begin(), m_grid.end(), M.m_grid.begin(),
                  m_grid.begin(), std::plus<float>()) ;
   return *this ;
}

// Add two maps together and return the result via a new map
OccGrid OccGrid::operator+(const OccGrid& M) const
{
   OccGrid result(*this) ;
   result += M ;
   return result ;
}

// Scale this map using the supplied weighting factor and return the
// result via a new map.
OccGrid OccGrid::operator*(float weight) const
{
   OccGrid result(*this) ;
   std::transform(result.m_grid.begin(), result.m_grid.end(),
                  result.m_grid.begin(),
                  std::bind2nd(std::multiplies<float>(), weight)) ;
   return result ;
}

//---------------------------- MAP ACCESS -------------------------------

void OccGrid::occupied(int x, int y)
{
   update(x, y, exp(-0.5f * sqr(get(x, y)))) ;
}

void OccGrid::vacant(int x, int y)
{
   update(x, y, -exp(-0.5f * sqr(get(x, y)))) ;
}

// This method increments/decrements the specified cell's log-odds value
// by the specified delta. To prevent the occupancy probability for a
// cell from becoming 0 or 1 (i.e., absolute certainty), we clamp the new
// log-odds value to some reasonable boundary, viz., [-7, 7]. A log-odds
// value of -7 corresponds to a probability of 0.00091105 and +7 to
// 0.99908895.
//
// NOTE: When we update a cell, we also update its immediate neighbours.
// A part of the delta value is added to the target cell's current
// occupancy value and the remaining part is divided equally between and
// added to the cell's immediate neighbours.
void OccGrid::update(int x, int y, float delta)
{
   const int W = SlamParams::map_width()  ;
   const int H = SlamParams::map_height() ;
   if (x < 0 || x >= W || y < 0 || y >= H)
      return ; // (x,y) out of bounds

   // Determine extents for immediate neigbours of (x,y)
   const int x_min = std::max(x - 1, 0) ;
   const int x_max = std::min(x + 1, W - 1) ;
   const int y_min = std::max(y - 1, 0) ;
   const int y_max = std::min(y + 1, H - 1) ;

   // Only a certain percentage of the delta value will be applied to
   // (x,y). The remaining percentage of the delta value will be
   // distributed equally to the immediate neigbours of (x,y).
   const float w = SlamParams::update_weight() ;
   const float delta_nbr = delta *
      (1 - w)/((x_max - x_min + 1) * (y_max - y_min + 1) - 1) ;

   // Apply delta * (1 - w)/num_nbrs to immediate neigbours of (x,y)
   for (int j = y_min; j <= y_max; ++j)
   {
      int k = j * W + x_min ;
      for (int i = x_min; i <= x_max; ++i, ++k)
         m_grid[k] = clamp(m_grid[k] + delta_nbr, -7.0f, +7.0f) ;
   }

   // The above loop would have applied delta * (1 - w)/num_nbrs to the
   // neigbours of (x,y) as well as to (x,y) itself. So, we subtract that
   // amount from the occupancy value in cell (x,y) and add the proper
   // delta value for the target cell, viz., w * delta.
   int k = y * W + x ;
   m_grid[k] = clamp(m_grid[k] - delta_nbr + w * delta, -7.0f, +7.0f) ;
}

// This function checks the occupancy value of the specified cell to see
// if it is occupied or not.
bool OccGrid::is_occupied(int x, int y) const
{
   return get(x, y) >= SlamParams::occ_threshold() ;
}

// This function returns the occupancy value of the specified cell.
//
// NOTE: The occupancy value of a cell is determined as a weighted sum of
// the values in the target cell and those of its immediate neigbours.
// The maximum weight is given to the target cell (x,y) while the others
// get lower weights. Specifically, the same weighting factor applied to
// the delta value during cell updates is used here. Thus, if the update
// weight is 0.75, then 75% of the occupancy value of cell (x,y) is used
// and the remaining 25% is divided equally between the immediate
// neighbours to yield the final occupancy value.
float OccGrid::get(int x, int y) const
{
   const int W = SlamParams::map_width()  ;
   const int H = SlamParams::map_height() ;
   if (x < 0 || x >= W || y < 0 || y >= H)
      return 0 ; // area not covered by map: 50-50 occupancy likelihood

   // Determine extents for immediate neigbours of (x,y)
   const int x_min = std::max(x - 1, 0) ;
   const int x_max = std::min(x + 1, W - 1) ;
   const int y_min = std::max(y - 1, 0) ;
   const int y_max = std::min(y + 1, H - 1) ;

   // Only a certain percentage of the occupancy value of cell (x,y) will
   // actually contribute towards its "true" occupancy value. The
   // remaining percentage of the final weighted value will come from the
   // immediate neigbours of (x,y).
   const float w = SlamParams::update_weight() ;
   const float w_nbr = (1 - w)/((x_max - x_min + 1) * (y_max - y_min + 1) - 1);

   // When determining the weighted sum for the final occupancy value
   // corresponding to cell (x,y), first apply (1 - w)/num_nbrs to the
   // immediate neigbours...
   float occ = 0 ;
   for (int j = y_min; j <= y_max; ++j)
   {
      int k = j * W + x_min ;
      for (int i = x_min; i <= x_max; ++i, ++k)
         occ += w_nbr * m_grid[k] ;
   }

   // The above loop would have applied (1 - w)/num_nbrs to the
   // neigbours of (x,y) as well as to (x,y) itself. So, we subtract that
   // amount from the occupancy value in cell (x,y) and add the proper
   // weighted value to get the final occupancy likelihood.
   int k = y * W + x ;
   occ += (w - w_nbr) * m_grid[k] ;
   return occ ;
}

//----------------------------- CLEAN-UP --------------------------------

OccGrid::~OccGrid()
{
   /*
   LERROR("cleaning up OccGrid [%08lX]",
          reinterpret_cast<unsigned long>(this)) ;
   // */
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
