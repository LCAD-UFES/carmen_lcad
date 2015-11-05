/**
   \file Robots/LoBot/ui/LoLaserWindowGrid.H

   \brief A helper class for drawing the grid marking distance
   measurements on the LRF tester's main window.

   This file defines a helper class for drawing the measurements grid for
   the laser range finder. This grid shows markings at regular intervals
   indicating different distances. For example, we can have markings
   every 500, 1000 and 5000 mm. Additionally, we can arrange to have the
   markings at every 500 mm in one color, the markings at 1000 mm in
   another color, and the ones at 5000 mm in yet another color.
   Furthermore, we can also specify the zoom ranges at which the 500,
   1000 and 5000 mm markings should be visible.

   This helps make the laser range finder's distance measurements more
   readable and easier to interpret.
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindowGrid.H $
// $Id: LoLaserWindowGrid.H 11190 2009-05-16 08:45:51Z mviswana $
//

#ifndef LOBOT_LASER_WINDOW_GRID_DOT_H
#define LOBOT_LASER_WINDOW_GRID_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindowMarkings.H"
#include "Robots/LoBot/misc/factory.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LaserWindowGrid
   \brief A helper class for drawing the laser range finder's
   measurements grid.

   The laser range finder's test program depicts the distance
   measurements as rays emanating from the origin of a world coordinate
   system that is setup to match the units and range of the LRF device.
   To help make sense of the lengths of these rays, we draw a grid
   spanning the entire range of the device. This grid has markings at
   various regular intervals. Each of these intervals can have a
   different color and can be active at different zoom levels.

   For example, users can choose to have markings at 100, 500, 1000 and
   5000 mm. The markings at each of these intervals can be drawn in
   different colors. Furthermore, users can specify the zoom levels at
   which these markings should be visible. Thus, when zoomed out a lot,
   we can show only the 5000 mm markings to make the overall scene more
   legible; and at high zoom levels (zoomed in a lot), we can show the
   more fine-grained 100 mm markings.

   This class takes care of the details of drawing the grid described
   above.
*/
class LaserWindowGrid : public LaserWindowMarkings {
   // Prevent copy and assignment
   LaserWindowGrid(const LaserWindowGrid&) ;
   LaserWindowGrid& operator=(const LaserWindowGrid&) ;

   // Boilerplate code to make the factory work
   friend  class subfactory<LaserWindowGrid, LaserWindowMarkings> ;
   typedef register_factory<LaserWindowGrid, LaserWindowMarkings> my_factory ;
   static my_factory register_me ;

   // Handy type to have around in a derived class
   typedef LaserWindowMarkings base ;

   /// Private constructor because laser window markings are created with
   /// an object factory.
   LaserWindowGrid() ;

   /// Draw the measurements grid.
   void render() const ;

   /// Clean-up.
   ~LaserWindowGrid() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
