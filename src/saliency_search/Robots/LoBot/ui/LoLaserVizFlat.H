/**
   \file  Robots/LoBot/ui/LoLaserVizFlat.H
   \brief Visualizer that "flattens" the LRF data.

   This file defines a class that implements a drawable for visualizing
   the laser range finder's measurement data in "flat" or Cartesian mode,
   i.e., the LRF data is "unrolled" from its usual polar form and
   displayed as a 2D line graph with the y-axis showing the distance
   readings and the x-axis being the angular directions along which the
   LRF makes its measurements.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserVizFlat.H $
// $Id: LoLaserVizFlat.H 12859 2010-02-18 03:34:52Z mviswana $
//

#ifndef LOBOT_LRF_VISUALIZER_FLAT_DOT_H
#define LOBOT_LRF_VISUALIZER_FLAT_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/io/LoLaserRangeFinder.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/singleton.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LaserVizFlat
   \brief A drawable for visualizing laser range finder data.

   This class uses OpenGL to visualize the Hokuyo laser range finder's
   distance measurements. Instead of the usual polar form in which LRF
   data is often visualized, this visualizer is used to "flattens" the
   LRF measurements and show them in Cartesian form as a kind of 2D line
   graph.

   The x-axis of this graph is used for the angular directions supported
   by the LRF. The center of this axis is for zero degrees, or straight
   ahead, and the left and right of the axis's center are for angles to
   the left and right respectively.

   The y-axis of the graph is used for the distance readings.
*/
class LaserVizFlat : public Drawable {
   /// Since this drawable is used to visualize LRF data, it makes sense
   /// for it to keep handy a reference to an LRF object.
   const LaserRangeFinder* m_lrf ;

public:
   /// Initialization.
   LaserVizFlat(const LaserRangeFinder*) ;

   /// Clean-up.
   ~LaserVizFlat() ;

private:
   /// This method renders the latest measurements obtained from the
   /// laser range finder.
   void render_me() ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the LRF and behaviours'
   /// visualization.
   class Params : public singleton<Params> {
      /// The location and size of the LRF visualizer (within the
      /// Robolocust main window).
      Drawable::Geometry m_geometry ;

      /// Color for the line graph used to show the laser range finder
      /// distance measurements.
      ///
      /// NOTE: This setting actually uses the laser_viz section's
      /// measurements_color option rather than having a separate option
      /// in the laser_viz_flat section of the config file.
      GLColor m_color ;

      /// Private constructor because this is a singleton.
      Params() ;
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters
      //@{
      static const Drawable::Geometry& geometry() {
         return instance().m_geometry ;
      }
      static const GLColor& color() {return instance().m_color ;}
      //@}

      /// Clean-up
      ~Params() ;
   } ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
