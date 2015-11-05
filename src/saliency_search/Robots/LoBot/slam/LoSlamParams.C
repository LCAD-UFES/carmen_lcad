/**
   \file  Robots/LoBot/slam/LoSlamParams.C
   \brief This file defines the non-inline member functions of the
   lobot::SlamParams class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoSlamParams.C $
// $Id: LoSlamParams.C 13575 2010-06-17 01:42:18Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoSlamParams.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoMath.H"

// Standard C headers
#include <math.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Quick helper to return settings from "map" section of config file
template<typename T>
inline T map_conf(const std::string& key, T default_value)
{
   return get_conf<T>("map", key, default_value) ;
}

// Overload for returning config settings via an array
template<typename T>
inline void
map_conf(const std::string& key, T* array, const T* defaults, unsigned int n)
{
   Configuration::get<T>("map", key, array, defaults, n) ;
}

// Quick helper to return settings from "slam" section of config file
template<typename T>
inline T slam_conf(const std::string& key, T default_value)
{
   return get_conf<T>("slam", key, default_value) ;
}

// Overload for retrieving pairs
template<typename T>
inline std::pair<T, T>
slam_conf(const std::string& key, const std::pair<T, T>& default_value)
{
   return get_conf<T>("slam", key, default_value) ;
}

// Overload for retrieving triples
template<typename T>
inline triple<T, T, T>
slam_conf(const std::string& key, const triple<T, T, T>& default_value)
{
   return get_conf<T>("slam", key, default_value) ;
}

// Overload for retrieving ranges
template<typename T>
inline range<T>
slam_conf(const std::string& key, const range<T>& default_value)
{
   return get_conf<T>("slam", key, default_value) ;
}

} // end of anonymous local namespace encapsulating above helpers

// Parameter initialization
SlamParams::SlamParams()
   : m_cell_size(clamp(map_conf("cell_size", 10.0f), 1.0f, 1000.0f)),
     m_initial_pose(slam_conf("initial_pose", make_triple(0.0f, 0.0f, 0.0f))),
     m_errors(slam_conf("errors", make_triple(5.0f, 5.0f, 0.5f))),
     m_num_particles(clamp(slam_conf("num_particles", 100), 1, 50000)),
     m_ess_threshold(clamp(slam_conf("ess_threshold", m_num_particles/2),
                           0, m_num_particles + 1)),
     m_alpha(slam_conf("alpha", std::make_pair(0.85f, 0.05f))),
     m_beam_specs(slam_conf("beam_specs", make_triple(-90, 90, 15))),
     m_beam_model_sigma(clamp(slam_conf("beam_model_sigma", 75.0f),
                              1.0f, 1000.0f)),
     m_beam_model_fudge(clamp(slam_conf("beam_prob_fudge_factor", 2500.0f),
                              1.0f, 1e+12f)),
     m_beam_range(clamp(slam_conf("beam_range", make_range(60.0f, 5600.0f)),
                        make_range(30.0f, 30000.0f))),
     m_prob_rnd(1/(m_beam_range.size() + 1)),
     m_occ_range_error(clamp(slam_conf("occupancy_range_error", 50.0f),
                             5.0f, 500.0f)),
     m_occ_threshold(prob_to_log_odds(
        clamp(map_conf("occupancy_threshold", 0.75f), 0.05f, 0.95f))),
     m_update_weight(clamp(map_conf("update_weight", 0.75f), 0.5f, 0.95f)),
     m_match_scans(slam_conf("scan_matching", false)),
     m_map_file(get_conf<std::string>(LOBE_SURVEY, "map_file", "")),
     m_geometry(map_conf<std::string>("geometry", "0 0 390 390"))
{
   float default_extents[4] = {0, 5000, -2500, 2500} ;
   map_conf("extents", m_extents, default_extents, 4) ;
   m_extents[1] = std::max(m_extents[1], m_extents[0] + m_cell_size) ;
   m_extents[3] = std::max(m_extents[3], m_extents[2] + m_cell_size) ;

   const float L = m_extents[0] ;
   const float R = m_extents[1] ;
   const float B = m_extents[2] ;
   const float T = m_extents[3] ;
   const float S = m_cell_size  ;

   m_width  = static_cast<int>(ceilf((R - L)/S)) ;
   m_height = static_cast<int>(ceilf((T - B)/S)) ;
   m_max_distance = sqrtf(sqr(R - L) + sqr(T - B)) ;

   // See comment preceding Coords::to_grid() for explanation of why
   // these scale factors are computed in this manner and how they are
   // used.
   m_scale.first  = 1/S - 1/(R - L) ;
   m_scale.second = 1/S - 1/(T - B) ;

   m_initial_pose.first  = clamp(m_initial_pose.first,  L, R) ;
   m_initial_pose.second = clamp(m_initial_pose.second, B, T) ;
   m_initial_pose.third  = clamp_angle(m_initial_pose.third)  ;

   m_errors.first  = clamp(m_errors.first,  0.0f, 500.0f) ;
   m_errors.second = clamp(m_errors.second, 0.0f, 500.0f) ;
   m_errors.third  = clamp(m_errors.third,  0.0f,  90.0f) ;

   m_alpha.first  = clamp(m_alpha.first,  0.50f, 0.99f) ;
   m_alpha.second = clamp(m_alpha.second, 0.01f, 0.25f) ;

   float p = clamp(slam_conf("kmeans_percentage", 10.0f), 0.0f, 50.0f) ;
   m_k = clamp(round(p * m_num_particles/100.0f), 1, 100) ;

   m_beam_specs.first  = clamp(m_beam_specs.first, -180, 180) ;
   m_beam_specs.second =
      clamp(m_beam_specs.second,
            m_beam_specs.first + 1,
            static_cast<int>(clamp_angle(m_beam_specs.first + 359))) ;
   m_beam_specs.third  = clamp(m_beam_specs.third, 1,
                               m_beam_specs.second - m_beam_specs.first + 1) ;
}

//---------------------------- MAP EXTENTS ------------------------------

void SlamParams::map_extents(float* L, float* R, float* B, float* T)
{
   const float* ext = instance().m_extents ;
   if (L)
      *L = ext[0] ;
   if (R)
      *R = ext[1] ;
   if (B)
      *B = ext[2] ;
   if (T)
      *T = ext[3] ;
}

void SlamParams::map_extents(float x[4])
{
   if (x) {
      const float* ext = instance().m_extents ;
      x[0] = ext[0] ;
      x[1] = ext[1] ;
      x[2] = ext[2] ;
      x[3] = ext[3] ;
   }
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
