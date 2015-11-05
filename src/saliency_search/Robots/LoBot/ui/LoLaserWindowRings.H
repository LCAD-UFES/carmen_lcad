/**
   \file Robots/LoBot/ui/LoLaserWindowRings.H

   \brief A helper class for drawing concentric circles to mark distance
   measurements on the LRF tester's main window.

   This file defines a helper class for drawing concentric circles
   centered at the position of the laser range finder within the LRF test
   program's window. These circles mark distances at regular intervals.

   For example, we can have markings every 100, 500, 1000 and 5000 mm.
   Each of these markings can be drawn in a different color. Thus, the
   100 mm markings can be one color, the 500 mm markings another color,
   the 1000 mm markings yet another color, and, finally the 5000 mm
   markings in a fourth color. Furthermore, we can also specify the zoom
   ranges at which the 100, 500, 1000 and 5000 mm markings should be
   visible.

   This helps make the laser range finder's distance measurements more
   readable and easier to interpret. It is usually much easier to read
   distances off such a "contour" map rather than trying to figure them
   out on a square grid.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindowRings.H $
// $Id: LoLaserWindowRings.H 12861 2010-02-18 19:46:03Z mviswana $
//

#ifndef LOBOT_LASER_WINDOW_RINGS_DOT_H
#define LOBOT_LASER_WINDOW_RINGS_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindowMarkings.H"
#include "Robots/LoBot/misc/factory.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LaserWindowRings
   \brief A helper class for drawing the laser range finder's
   measurements rings.

   The laser range finder's test program depicts the distance
   measurements as rays emanating from the origin of a world coordinate
   system that is setup to match the units and range of the LRF device.
   To help make sense of the lengths of these rays, we draw a concentric
   circles spanning the entire range of the device. Each circle marks a
   particular distance from the laser range finder.

   We can have rings corresponding to different intervals. Each
   interval's rings can have a different color and can be active at
   different zoom levels. For example, users can choose to have markings
   at 100, 500, 1000 and 5000 mm. The markings at each of these intervals
   can be drawn in different colors. Furthermore, users can specify the
   zoom levels at which these markings should be visible. Thus, when
   zoomed out a lot, we can show only the 5000 mm markings to make the
   overall scene more legible; and at high zoom levels (zoomed in a lot),
   we can show the more fine-grained 100 mm markings.

   This class takes care of the details of drawing the contours described
   above.
*/
class LaserWindowRings : public LaserWindowMarkings {
   // Prevent copy and assignment
   LaserWindowRings(const LaserWindowRings&) ;
   LaserWindowRings& operator=(const LaserWindowRings&) ;

   // Boilerplate code to make the factory work
   friend  class subfactory<LaserWindowRings, LaserWindowMarkings> ;
   typedef register_factory<LaserWindowRings, LaserWindowMarkings> my_factory ;
   static my_factory register_me ;

   // Handy type to have around in a derived class
   typedef LaserWindowMarkings base ;

   /// This class uses an OpenGL display list to speed up drawing the
   /// concentric circles.
   mutable int m_display_list ;

   /// Private constructor because laser window markings are created with
   /// an object factory.
   LaserWindowRings() ;

   /// Draw the measurements contours.
   void render() const ;

   /// Clean-up.
   ~LaserWindowRings() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
