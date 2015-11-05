/**
   \file  Robots/LoBot/control/LoDangerZone.C
   \brief This file defines the non-inline member functions of the
   lobot::DangerZone class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoDangerZone.C $
// $Id: LoDangerZone.C 13780 2010-08-11 22:07:47Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoDangerZone.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/LoMath.H"

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <sstream>
#include <algorithm>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from danger zone section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("danger_zone", key, default_value) ;
}

// Return a "raw" danger zone spec z so that it lies within the bounds of
// the LRF's distance range D. If z is negative, the user probably wants
// to disable that particular angular block. So return -1 in that case.
static inline int sanify_dzone(int z, const range<int>& D)
{
   return (z <= 0) ? -1 : clamp(z, D) ;
}

// A comparator for DangerZone::Block based on danger zone settings
static bool dzone_cmp(const DangerZone::Block& a, const DangerZone::Block& b)
{
   return a.danger_zone() < b.danger_zone() ;
}

//------------------------ STATIC DATA MEMBERS --------------------------

const LaserRangeFinder* DangerZone::m_lrf ;

//-------------------------- INITIALIZATION -----------------------------

DangerZone::DangerZone()
   : m_max(0), m_lrf_data(0)
{
   // LRF better be specified at this point
   if (! m_lrf)
      throw misc_error(DANGER_ZONE_LRF_NOT_SETUP) ;

   const range<int> A = m_lrf->get_angular_range()  ;
   const range<int> D = m_lrf->get_distance_range() ;

   // Get the angular blocks specified in the config file
   std::vector<int> B(string_to_vector<int>(
      conf<std::string>("angular_blocks", to_string(A)))) ;
   int N = B.size() ;
   if (N < 2) { // user specified some crap in the config file
      B.clear() ;
      B.push_back(A.min()) ;
      B.push_back(A.max()) ;
      N = 2 ;
   }

   // Get the danger zones for above blocks from config file
   std::vector<int> Z(string_to_vector<int>(
      conf<std::string>("danger_zones", dup_string(" 350", N - 1)))) ;

   // Get the individual thresholds for above blocks from config file
   std::vector<int> T(string_to_vector<int>(
      conf<std::string>("thresholds", dup_string(" 10", N - 1)))) ;

   // Store the above settings in internal list for later use
   m_blocks.reserve(N - 1) ;
   for (int i = 0, j = 1; j < N - 1; ++i, ++j)
      if (A.in(B[i]) && A.in(B[j]))
         m_blocks.push_back(Block(B[i], B[j] - 1,
                                  sanify_dzone(Z[i], D), T[i])) ;

   int penultimate_start = clamp(B[N - 2], A.min(), A.max() - 1) ;
   m_blocks.push_back(Block(penultimate_start,
                            clamp(B[N - 1], penultimate_start + 1, A.max()),
                            sanify_dzone(Z[N - 2], D),
                            T[N - 2])) ;

   // Find the max configured danger zone
   Blocks::const_iterator max =
      std::max_element(m_blocks.begin(), m_blocks.end(), dzone_cmp) ;
   m_max = max->danger_zone() ;
}

// DangerZone::Block constructors
DangerZone::Block::Block(const range<int>& x, int z, int t)
   : m_extents(x),
     m_danger_zone(z),
     m_threshold(clamp(t, 0, m_extents.size()))
{
   m_danger_zone_readings.reserve(m_extents.size()) ;
}

DangerZone::Block::Block(int start, int end, int z, int t)
   : m_extents(start, end),
     m_danger_zone(z),
     m_threshold(clamp(t, 0, m_extents.size()))
{
   m_danger_zone_readings.reserve(m_extents.size()) ;
}

//--------------------- UPDATING THE DANGER ZONE ------------------------

DangerZone::Block::update::update(const LRFData& d)
   : lrf(d)
{}

void DangerZone::Block::update::operator()(const DangerZone::Block& B) const
{
   Block& block = const_cast<Block&>(B) ;

   block.clear() ;
   for (int angle = block.start(); angle <= block.end(); ++angle)
   {
      int distance = lrf[angle] ;
      if (distance < 0) // bad reading
         continue ;
      if (distance <= block.danger_zone())
         block.add(Reading(angle, distance)) ;
   }
   //block.dump("action_lrf_update") ;
}

void DangerZone::update()
{
   DangerZone& Z = instance() ;

   delete Z.m_lrf_data ;
   Z.m_lrf_data = new LRFData(m_lrf) ;
   std::for_each(Z.m_blocks.begin(), Z.m_blocks.end(),
                 Block::update(*Z.m_lrf_data));
}

//------------------------ DANGER ZONE QUERIES --------------------------

bool DangerZone::penetrated()
{
   const DangerZone& Z = instance() ;
   const int N = Z.m_blocks.size()  ;
   for  (int i = 0; i < N; ++i)
      if (Z.m_blocks[i].penetrated())
         return true ;
   return false ;
}

//----------------------------- CLEAN-UP --------------------------------

DangerZone::~DangerZone()
{
   delete m_lrf_data ;
}

//--------------------------- DEBUG SUPPORT -----------------------------

void DangerZone::Block::dump(const std::string& caller) const
{
   std::ostringstream str ;

   str << "from " << caller << '\n' ;

   str << "\n\tExtents: ["  << m_extents.min() << ' '
                            << m_extents.max() << "]" ;
   str << "\tDanger Zone: " << m_danger_zone ;
   str << "\tThreshold: "   << m_threshold ;

   str << "\n\tDanger zone readings: " ;
   const Readings& R = m_danger_zone_readings ; // alias to cut down on typing
   for (Readings::const_iterator it = R.begin(); it != R.end(); ++it)
      str << '(' << it->angle() << ' ' << it->distance() << ") " ;
   str << '\n' ;

   LERROR("%s", str.str().c_str()) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
