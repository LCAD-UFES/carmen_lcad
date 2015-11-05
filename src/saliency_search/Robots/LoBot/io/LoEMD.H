/**
   \file  Robots/LoBot/io/LoEMD.H
   \brief A generic class implementing a Reichardt Elementary Motion
   Detector.

   This file defines a class that implements an Elementary Motion
   Detector using two inputs, viz., a left and a right (though it could
   just as well be up and down instead or any other pair of opposing
   directions). The EMD expects to be given a direction vector and
   returns a scaled version of this vector to indicate the current motion
   from its inputs.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoEMD.H $
// $Id: LoEMD.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_ELEMENTARY_MOTION_DETECTOR_DOT_H
#define LOBOT_ELEMENTARY_MOTION_DETECTOR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/util/LoMath.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::EMD
   \brief A generic Elementary Motion Detector.

   This class implements the notion of an Elementary Motion Detector. It
   takes two inputs (left and right, though they could be a pair of any
   opposing directions) and performs the Reichardt EMD computation to
   assess the "amount" of motion in left or right directions (negative
   amounts mean leftward motion whereas positive amounts indicate
   rightwards motion). This scalar quantity is vectorized by scaling a
   direction vector provided to the EMD on its creation.

   Like all templates, this one too implies an interface for its type
   parameters. Specifically, the EMD's input type is required to
   implement a value() function that returns floating point number
   indicating the current level of whatever activity it measures.
*/
template<typename input_type>
class EMD {
   // Prevent copy and assignment
   EMD(const EMD&) ;
   EMD& operator=(const EMD&) ;

   /// An Elementary Motion Detector is built from a pair of adjacent
   /// inputs.
   const input_type& m_left ;
   const input_type& m_right ;

   /// For the EMD to work, we need to keep track of both the input
   /// values from the previous time step.
   float m_left_prev, m_right_prev ;

   /// The EMD is setup to point along a certain direction.
   Vector m_direction ;

public:
   /// Clients must specify the left and right (up/down, whatever) inputs
   /// and also supply a direction vector for the EMD. The direction
   /// vector is specified as an angle in degrees.
   EMD(const input_type& left, const input_type& right, float angle) ;

   /// This method uses the latest values from the EMD's left and right
   /// inputs and computes the vector representing the total motion.
   Vector update() ;

   /// Destructor
   virtual ~EMD() ;
} ;

// Initialization
template<typename input_type>
EMD<input_type>::
EMD(const input_type& left, const input_type& right, float angle)
   : m_left(left),   m_right(right),
     m_left_prev(0), m_right_prev(0),
     m_direction(cos(angle), sin(angle))
{}

// EMD update
template<typename input_type>
Vector EMD<input_type>::update()
{
   const float L = m_left.value() ;
   const float R = m_right.value() ;
   const float M = m_left_prev * R - L * m_right_prev ;

   m_left_prev  = L ;
   m_right_prev = R ;

   //return M * m_direction ; // FIXME: should this be abs(M) * m_direction?
   return abs(M) * m_direction ; // FIXME: should not be abs(M) * m_direction?
}

// Clean-up
template<typename input_type>
EMD<input_type>::~EMD()
{}

//---------------------- POINTER SPECIALIZATION -------------------------

// Full specialization for void pointers
template<>
class EMD<void*> {
   // Prevent copy and assignment
   EMD(const EMD&) ;
   EMD& operator=(const EMD&) ;

   // The left and right inputs of the EMD
   const void* m_left ;
   const void* m_right ;

   // The previous values of the two inputs
   float m_left_prev, m_right_prev ;

   // The EMD's direction vector
   Vector m_direction ;

protected:
   // Constructor
   EMD(const void* left, const void* right, float angle) ;

   // The EMD computation method
   Vector update() ;

   // Derived classes must supply this methods for retrieving the current
   // input value from the specified input.
   virtual float value(const void* input) = 0 ;

public:
   // Destructor
   virtual ~EMD() ;
} ;

// Initialization
EMD<void*>::
EMD(const void* left, const void* right, float angle)
   : m_left(left),   m_right(right),
     m_left_prev(0), m_right_prev(0),
     m_direction(cos(angle), sin(angle))
{}

// EMD update
Vector EMD<void*>::update()
{
   const float L = value(m_left) ;
   const float R = value(m_right) ;
   const float M = m_left_prev * R - L * m_right_prev ;

   m_left_prev  = L ;
   m_right_prev = R ;

   //return M * m_direction ; // FIXME: should this be abs(M) * m_direction?
   return abs(M) * m_direction ; // FIXME: should not be abs(M) * m_direction?
}

// Clean-up
EMD<void*>::~EMD()
{}

// Partial specialization for non-void pointers
template<typename input_type>
class EMD<input_type*> : private EMD<void*> {
   // Handy type to have around in a derived class
   typedef EMD<void*> base ;

   // Prevent copy and assignment
   EMD(const EMD&) ;
   EMD& operator=(const EMD&) ;

public:
   // Constructor
   EMD(const input_type* left, const input_type* right, float angle) ;

   // Destructor
   ~EMD() ;

   // EMD update
   Vector update() {return base::update() ;}

protected:
   // Override required of derived class
   float value(const void* input) ;
} ;

// Initialization
template<typename input_type>
EMD<input_type*>::
EMD(const input_type* left, const input_type* right, float angle)
   : base(reinterpret_cast<const void*>(left),
          reinterpret_cast<const void*>(right),
          angle)
{}

// EMD update
template<typename input_type>
float EMD<input_type*>::value(const void* input)
{
   return (reinterpret_cast<const input_type*>(input))->value() ;
}

// Destructor
template<typename input_type>
EMD<input_type*>::~EMD()
{}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
