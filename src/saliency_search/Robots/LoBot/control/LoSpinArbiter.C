/**
   \file  Robots/LoBot/control/LoSpinArbiter.C
   \brief This file defines the non-inline member functions of the
   lobot::SpinArbiter class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoSpinArbiter.C $
// $Id: LoSpinArbiter.C 13770 2010-08-08 06:49:50Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoSpinArbiter.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"
#include "Robots/LoBot/util/LoMath.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

SpinArbiter::SpinArbiter()
   : Arbiter(clamp(get_conf("spin_arbiter", "update_delay", 500), 1, 1000))
{
   start("spin_arbiter") ;
}

SpinArbiter::Vote::Vote(float spin)
   : m_spin(spin)
{}

float SpinArbiter::get_configured_priority(const std::string& behaviour) const
{
   return abs(get_conf(behaviour, "spin_priority", 0.0f)) ;
}

//-------------------------- MOTOR COMMANDS -----------------------------

void SpinArbiter::motor_cmd(const Arbiter::Votes& votes, Robot* robot)
{
   // Compute final spin amount as weighted sum of all votes
   float result = 0 ;
   Arbiter::Votes::const_iterator it = votes.begin() ;
   for (; it != votes.end(); ++it)
      result += (dynamic_cast<Vote*>((*it)->vote))->spin()
                   * priority((*it)->behavior_name) ;

   UpdateLock::begin_write() ;
      robot->spin(result) ;
   UpdateLock::end_write() ;
}

//----------------------- SPIN ARBITER CLEAN-UP -------------------------

SpinArbiter::~SpinArbiter(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
