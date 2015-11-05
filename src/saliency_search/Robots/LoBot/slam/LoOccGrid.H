/**
   \file  Robots/LoBot/slam/LoOccGrid.H
   \brief A log-odds occupancy grid that acts as an obstacle map.

   This file defines a class that implements an obstacle map for the
   Robolocust system. This map is an occupancy grid that holds
   probability values rather than simple 0/1 flags indicating the absence
   or presence of obstacles. Furthermore, the probabilities are stored in
   log-odds form.

   The grid is discretized so that, for example, each cell represents 10
   sq.mm of "real" space. The range sensor readings are then binned when
   determining which cell in the map they "hit."

   The occupancy grid's extents are defined by [L, R, B, T] values.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoOccGrid.H $
// $Id: LoOccGrid.H 13559 2010-06-11 10:15:15Z mviswana $
//

#ifndef LOBOT_OCCUPANCY_GRID_DOT_H
#define LOBOT_OCCUPANCY_GRID_DOT_H

//------------------------------ HEADERS --------------------------------

// Standard C++ headers
#include <string>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::OccGrid
   \brief A log-odds occupancy grid.

   This class implements an occupancy grid that is used to keep track of
   where obstacles are in the robot's environment. The occupancy grid's
   cells hold probability values rather than simple 0/1 flags to indicate
   the absence or presence of an obstacle. Furthermore, the probabilities
   are stored in log-odds form rather than directly as numbers in the
   [0,1] range.

   The extents of the occupancy grid are specified using [L, R, B, T]
   values, which are read from the Robolocust config file.
*/
class OccGrid {
   /// The occupancy grid is represented as a single dimensional array
   /// stored in row-major order.
   //@{
   typedef std::vector<float> Grid ;
   Grid m_grid ;
   //@}

public:
   /// When an "empty" occupancy grid object is created, all its
   /// probabilities are set to 50%.
   OccGrid() ;

   /// An occupancy grid may be initialized from a file containing
   /// coordinates of obstacles. In this case of a known map, the cells
   /// containing obstacles will be assigned high likelihoods (nearly
   /// 100%) and the other cells will get low likelihoods (nearly 0%).
   OccGrid(const std::string& map_file) ;

   /// Copy.
   OccGrid(const OccGrid&) ;

   /// Assignment.
   OccGrid& operator=(const OccGrid&) ;

   /// This method adds this map to the supplied map, retaining the
   /// result in this map object itself.
   OccGrid& operator+=(const OccGrid&) ;

   /// This method adds this map to the supplied map and returns the
   /// result in a new map object.
   OccGrid operator+(const OccGrid&) const ;

   /// This method multiplies the probabilities in this occupancy grid by
   /// the supplied weighting factor and returns the scaled result in a
   /// new occupancy grid object.
   OccGrid operator*(float weight) const ;

   /// Accessors.
   //@{
   Grid::const_iterator begin() const {return m_grid.begin() ;}
   Grid::const_iterator end()   const {return m_grid.end()   ;}
   //@}

   /// These methods mark the specified cell in the occupancy grid as
   /// either occupied or vacant by incrementing or decrementing the
   /// log-odds value in that cell.
   //@{
   void occupied(int x, int y) ;
   void vacant(int x, int y) ;
private:
   void update(int x, int y, float prob_delta) ;
public:
   //@}

   /// Return the log-odds occupancy value stored at the specified cell.
   ///
   /// NOTE: The occupancy value of a cell is determined as a weighted
   /// sum of the value stored in the target cell itself and the values
   /// in the immediate neighbourhood of the target cell. The target
   /// cell's occupancy value gets the maximum weight while the
   /// neigbouring cells get lower weights.
   float get(int x, int y) const ;

   /// This method checks if the specified cell of the occupancy grid is
   /// occupied.
   bool is_occupied(int x, int y) const ;

   /// Check if cell is vacant.
   bool is_vacant(int x, int y) const {return ! is_occupied(x, y) ;}

   /// Clean-up.
   ~OccGrid() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
