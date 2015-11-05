/**
   \file  Robots/LoBot/misc/LoPID.H
   \brief Generic PID controller.

   This file defines a class that implements a simple version of the
   standard PID control algorithm.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoPID.H $
// $Id: LoPID.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_PID_CONTROLLER_DOT_H
#define LOBOT_PID_CONTROLLER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/triple.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::PID
   \brief Simple implementation of a PID controller.

   This class implements a generic PID controller that doesn't really
   know anything about the control variable. Rather, it simply computes a
   command given the current error by keeping track of the P, I and D
   components of a PID controller.

   This class is defined as a template. However, it is really meant to be
   used only with numeric types.
*/
template<typename T>
class PID {
   // Prevent copy and assignment
   PID(const PID&) ;
   PID& operator=(const PID&) ;

   /// As its name indicates, a PID controller has three components,
   /// viz., P or propoptional, I or integral, and D or derivative. The P
   /// term is computed instantaneously (i.e., from the current input);
   /// however, the I and D terms are cumulative in nature. Thus, we need
   /// to keep track of the and I and D terms.
   T m_i, m_d ;

   /// The controller needs to track the previous error so that it can
   /// compute the D term properly.
   T m_prev_error ;

   /// Each component has its own gain. To turn a component off, simply
   /// set that particular gain to zero.
   T m_p_gain, m_i_gain, m_d_gain ;

public:
   /// Re/setting the gains.
   //@{
   void p_gain(const T& g) {m_p_gain = g ;}
   void i_gain(const T& g) {m_i_gain = g ;}
   void d_gain(const T& g) {m_d_gain = g ;}
   void gains (const T& p, const T& i, const T& d) {
      p_gain(p) ;
      i_gain(i) ;
      d_gain(d) ;
   }
   void gains(const triple<T, T, T>& g) {gains(g.first, g.second, g.third) ;}
   //@}

   /// Retrieving the current gains.
   //@{
   T p_gain() const {return m_p_gain ;}
   T i_gain() const {return m_i_gain ;}
   T d_gain() const {return m_d_gain ;}
   triple<T, T, T> gains() const {
      return make_triple(p_gain(), i_gain(), d_gain()) ;
   }
   //@}

   /// Initialization. Clients should supply the gains to be used.
   //@{
   PID(const T& p_gain = T(0), const T& i_gain = T(0), const T& d_gain = T(0)) ;
   PID(const triple<T, T, T>& gains) ;
   //@}

   /// This method returns the command to be applied given the current
   /// error.
   T cmd(const T& error) ;

   /// Sometimes, clients may find it necessary to reset the controller
   /// and start over again.
   void reset() {m_i = m_d = 0 ; m_prev_error = 0 ;}

   /// Clean-up.
   ~PID() ;
} ;

//-------------------------- INITIALIZATION -----------------------------

template<typename T>
PID<T>::PID(const T& p_gain, const T& i_gain, const T& d_gain)
   : m_i(0), m_d(0),
     m_prev_error(0),
     m_p_gain(p_gain), m_i_gain(i_gain), m_d_gain(d_gain)
{}

template<typename T>
PID<T>::PID(const triple<T, T, T>& g)
   : m_i(0), m_d(0),
     m_prev_error(0),
     m_p_gain(g.first), m_i_gain(g.second), m_d_gain(g.third)
{}

//------------------------- CONTROL ALGORITHM ---------------------------

template<typename T>
T PID<T>::cmd(const T& error)
{
   m_i  += error ;
   m_d   = (error - m_prev_error) ;
   T out = m_p_gain * error + m_i_gain * m_i + m_d_gain * m_d ;

   m_prev_error = error ;

   return out ;
}

//----------------------------- CLEAN-UP --------------------------------

template<typename T>
PID<T>::~PID()
{}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
