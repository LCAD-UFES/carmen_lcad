/*!
   \file  Robots/LoBot/lgmd/gabbiani/LoGabbiani.C
   \brief Gabbiani's LGMD model.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/lgmd/gabbiani/LoGabbiani.C $
// $Id: LoGabbiani.C 13730 2010-07-29 12:26:38Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/lgmd/gabbiani/LoGabbiani.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/io/LoRobot.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/config/LoDefaults.H"

#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/util/LoMath.H"

// Standard C++ headers
#include <utility>

// Standard C headers
#include <math.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

GabbianiModel::GabbianiModel(const LocustModel::InitParams& p)
   : base(p)
{
   if (! App::lrf())
      throw io_error(LASER_RANGE_FINDER_MISSING) ;
   if (! App::robot())
      throw io_error(MOTOR_SYSTEM_MISSING) ;
}

//------------------------- LGMD COMPUTATIONS ---------------------------

// This helper function projects the robot's current velocity vector
// along the specified direction and then returns the magnitude of the
// resulting vector, i.e., the speed along the input direction.
static float project_velocity(float direction)
{
   float S = App::robot()->current_speed() ;
   float H = App::robot()->current_heading() ;

   Vector v(S * cos(H), S * sin(H)) ;
   Vector d(cos(direction), sin(direction)) ;

   float D = dot(d, v) ;
   return sign(D) * magnitude(D * d) ;
}

// This method applies the Gabbiani LGMD model to determine the current
// spike rate for this virtual locust.
void GabbianiModel::update()
{
   float speed = project_velocity(m_direction) ;
   if (is_zero(speed))
   {
      update_lgmd(0) ;
      m_distance = m_tti = -1 ;
   }
   else
   {
      m_distance = m_source->average_distance(m_lrf_range) ;
      float dist = m_distance/1000.0f ;
      float time = dist/speed ;
      update_lgmd(spike_rate(time)) ;
      m_tti = abs(time) ;
      //LERROR("%6.3f m/s, %6.3f m, %6.3f s, %8.3f spikes/s",
             //speed, dist, m_tti, get_lgmd()) ;
   }
   //LERROR("locust[%4.0f] = %8.3f spikes/second", m_direction, get_lgmd()) ;
}

/*
   Once we have an estimate of the time-to-impact t, we can determine the
   LGMD spike rate by applying the equations that Gabbiani, et al.
   developed to fit the electrophysiological data they gathered:


                f(t) = C * abs(D(t-d)) * exp(-a * T(t-d))

          where f(t) = the desired spike rate

                          -(l/v)
                D(t) = -------------      [aka theta_dot]
                       t^2 + (l/v)^2

                                 (l/v)
                T(t) = 2 * arctan-----    [aka theta]
                                   t

                  C  = a proportionality constant
                  a  = a parameter (alpha)
                  d  = a parameter (delta)
                 l/v = a parameter (the famous ell-over-vee)


   This function applies the above multiplicative model of the LGMD to
   the supplied time-to-impact and returns the resulting spike rate.
*/
float GabbianiModel::spike_rate(float t)
{
   const float C        = Params::C() ;
   const float alpha    = Params::alpha() ;
   const float delta    = Params::delta() ;
   const float l_over_v = Params::l_over_v() ;

   t -= delta ;
   float theta = -l_over_v/(sqr(t) + sqr(l_over_v)) ;
   float theta_dot  = 2 * atanf(l_over_v/t) ;
   float lgmd_ideal = C * abs(theta_dot) * exp(-alpha * theta) ;

   const float sigma = Params::sigma() ;
   return is_zero(sigma) ? lgmd_ideal //no noise
                         : lgmd_ideal + sample_tri(sigma) ;
}

//----------------------------- CLEAN-UP --------------------------------

GabbianiModel::~GabbianiModel(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Quick helper to retrieve configuration settings from the gabbiani
// section of the config file.
template<typename T>
static T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOLM_GABBIANI, key, default_value) ;
}

// Parameters initialization
GabbianiModel::Params::Params()
   : m_C(conf("C", 150)),
     m_alpha(conf("alpha", 0.75f)),
     m_delta(conf("delta", 2.5f)),
     m_l_over_v(conf("l_over_v", 1.5f)),
     m_sigma(clamp(conf("sigma", 0.0f), 0.0f, 200.0f))
{}

// Parameters clean-up
GabbianiModel::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
