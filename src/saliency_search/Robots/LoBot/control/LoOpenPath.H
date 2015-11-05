/**
   \file  Robots/LoBot/control/LoOpenPath.H
   \brief A behaviour for steering the robot based on the open path
   algorithm.

   This file defines a class that implements the open path algorithm for
   steering the robot towards the longest open path that will accommodate
   the robot's width given the current set of distance readings from the
   laser range finder.

   The open path algorithm is described in the following paper:

      von Wahlde, R., Wiedenman, N., Brown, W. A., Viqueira, C.
      An Open-Path Obstacle Avoidance Algorithm Using Scanning Laser
      Range Data
      U. S. Army Research Laboratory, Report number ARL-MR-0715

   The above paper is rather abstruse and unnecessarily complicated. This
   implementation uses the open path idea but does not follow the
   description in the paper.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoOpenPath.H $
// $Id: LoOpenPath.H 13812 2010-08-21 04:04:10Z mviswana $
//

#ifndef LOBOT_OPEN_PATH_BEHAVIOUR_DOT_H
#define LOBOT_OPEN_PATH_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"

#include "Robots/LoBot/io/LoLRFData.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/util/triple.hh"

// Standard C++ headers
#include <map>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::OpenPath
   \brief A behaviour for steering the robot toward the most open path
   that can accommodate it.

   This class implements the open path algorithm for steering the robot
   toward the most open path that can accommodate the robot's width.

   The algorithm uses the current set of distance readings from the laser
   range finder and some trigonometry and vector math to build a list of
   candidate paths of some user-specified minimum length. It then picks
   the path with the maximum traversable area and issues a steering vote
   to turn the robot towards that path.
*/
class OpenPath : public Behavior {
   // Prevent copy and assignment
   OpenPath(const OpenPath&) ;
   OpenPath& operator=(const OpenPath&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<OpenPath, base> ;
   typedef register_factory<OpenPath, base> my_factory ;
   static  my_factory register_me ;

   /// Each potential open path is characterized by a length, a width and
   /// an area. This inner class serves to keep these characteristics
   /// together in one place.
   class PathInfo {
      triple<float, float, float> m_info ;
   public:
      PathInfo(float length, float width) ;

      float length() const {return m_info.first  ;}
      float width()  const {return m_info.second ;}
      float area()   const {return m_info.third  ;}

      bool operator<(const PathInfo& p) const {return area() < p.area() ;}
   } ;

   /// In order to find the most open path, the behaviour will have to
   /// first build a list of several potential candidates that meet at
   /// least the basic criteria for minimum path width and path length.
   /// Only then will it be in a position to determine which of these
   /// candidates is the best.
   ///
   /// This data structure is used to store all the candidate paths.
   /// Rather than a plain list, we use a map that stores paths
   /// corresponding to the particular directions in which they lie.
   ///
   /// This map is also used for visualization.
   //@{
   typedef std::map<int, PathInfo> Paths ;
   Paths m_paths ;
   //@}

   /// In addition to visualizing the candidate paths, this behaviour
   /// also draws its most recently issued turn vote.
   TurnArbiter::Vote m_vote ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   OpenPath() ;

   /// Some things to do before commencing regular action processing.
   void pre_run() ;

   /// This method provides the body of the behaviour's main loop.
   void action() ;

   /// This is a helper function that implements all the necessary math
   /// required to find an open path in the specified direction. The path
   /// must have the configured minimum width (to allow the robot to fit)
   /// and minimum length (to ensure a decent amount of forward motion).
   PathInfo open_path(int angle, const LRFData&) const ;

   /// Visualization routines to aid with development and debugging.
   //@{
   void render_me() ;
   static void render_paths(unsigned long client_data) ;
   void render_paths() ;
   //@}

   /// Clean-up.
   ~OpenPath() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
