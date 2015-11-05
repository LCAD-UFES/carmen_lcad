/**
   \file  Robots/LoBot/control/LoVFH.H
   \brief A behaviour for driving and steering the robot based on a
   repulsive vector field.

   This file defines a class that implements the vector field histogram
   concept for guiding the robot.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoVFH.H $
// $Id: LoVFH.H 12581 2010-01-16 01:57:40Z mviswana $
//

#ifndef LOBOT_VECTOR_FIELD_HISTOGRAM_BEHAVIOUR_DOT_H
#define LOBOT_VECTOR_FIELD_HISTOGRAM_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
//#include "Robots/LoBot/control/LoTurnArbiter.H"

#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/singleton.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::VFH
   \brief A behaviour for driving and steering the robot based on a
   repulsive vector field.

   This class implements a simple behaviour that uses the vector field
   histogram concept for robot guidance.
*/
class VFH : public Behavior {
   // Prevent copy and assignment
   VFH(const VFH&) ;
   VFH& operator=(const VFH&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<VFH, base> ;
   typedef register_factory<VFH, base> my_factory ;
   static  my_factory register_me ;

   /// To aid with development and debugging, this behaviour supports a
   /// visualization callback, which needs the most recent vote so that
   /// it can perform the proper visualization.
   //TurnArbiter::Vote m_vote ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   VFH() ;

   /// Some things to do before commencing regular action processing.
   void pre_run() ;

   /// This method provides the body of the behaviour's main loop.
   void action() ;

   /// Visualization routines to aid with development and debugging.
   void render() ;

   /// Clean-up.
   ~VFH() ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the VFH behaviour.
   class Params : public singleton<Params> {

      /// Private constructor because this is a singleton.
      Params() ;

      // Boilerplate code to make generic singleton design pattern work
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters.
      //@{
      //@}

      /// Clean-up.
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
