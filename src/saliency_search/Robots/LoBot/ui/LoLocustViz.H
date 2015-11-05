/**
   \file  Robots/LoBot/ui/LoLocustViz.H
   \brief Locust LGMD spike rate visualizer.

   This file defines a class that implements a drawable for visualizing
   the LGMD spike rates of all the (virtual) locusts. The visualization
   resembles that of the laser range finder.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLocustViz.H $
// $Id: LoLocustViz.H 13811 2010-08-21 02:00:08Z mviswana $
//

#ifndef LOBOT_LOCUST_LGMD_VISUALIZER_DOT_H
#define LOBOT_LOCUST_LGMD_VISUALIZER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/misc/LoVector.H"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class LocustModel ;

/**
   \class lobot::LocustViz
   \brief A drawable for visualizing the LGMD spike rates.

   This class uses OpenGL to visualize the spike rates of all the
   locusts. The visualization resembles that of a range sensor such as
   the laser range finder. Each locust's spike rate is drawn as a ray
   starting at the visualizer's center. This ray's direction corresponds
   to the direction in which its associated locust is looking. The
   magnitude of the ray represents the amount of spiking activity
   associated with that locust. A set of concentric rings is used to help
   the user gauge the numerical value associated with the spike rate
   rays.
*/
class LocustViz : public Drawable {
   // Prevent copy and assignment
   LocustViz(const LocustViz&) ;
   LocustViz& operator=(const LocustViz&) ;

   /// Since this class visualizes the spike rates of all the locusts, it
   /// will need to store references to these locusts so that it can
   /// retrieve the spike rates when it comes time to perform rendering.
   const std::vector<LocustModel*>& m_locusts ;

   /// The locust model stores the direction in which a locust looks in
   /// polar form. For the rendering, we need to convert this to a
   /// Cartesian vector. Since a locust's direction remains fixed once
   /// its model interface object is created, we don't have to keep
   /// performing the same polar-to-Cartesian conversions over and over.
   /// Instead, we compute the unit vectors associated with each locust's
   /// radial direction on initialization and simply scale those vectors
   /// by the current spike rates to obtain the vectors for
   /// visualization.
   ///
   /// This data member stores the precomputed unit vectors for each of
   /// the locusts.
   std::vector<Vector> m_directions ;

   /// To properly setup the coordinate system for the visualization, we
   /// need to know the maximum spike rate. This parameter will be
   /// obtained from the locust model during initialization.
   float m_spike_max ;

   /// Once we know the maximum spike rate, we have all the information
   /// we need to draw the concentric rings mentioned above. Thus, we
   /// store the necessary rendering commands in an OpenGL display list.
   unsigned int m_display_list ;

public:
   /// Initialization: precompute the direction vectors for each of the
   /// locusts and set the max spike rate.
   LocustViz(const std::vector<LocustModel*>&) ;

private:
   /// OpenGL related initialization: create the display list for drawing
   /// the concentric rings.
   void gl_init() ;

   /// This method renders the latest spike rates obtained from all the
   /// locusts. The visualization resembles that of the laser range
   /// finder in "ray" mode.
   void render_me() ;

   /// OpenGL related clean-up: delete the display list used to render
   /// the concentric rings.
   void gl_cleanup() ;

public:
   /// Clean-up.
   ~LocustViz() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
