/**
   \file  Robots/LoBot/misc/LoDataset.C
   \brief This file defines the non-inline member functions of the
   lobot::Dataset class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoDataset.C $
// $Id: LoDataset.C 13934 2010-09-14 23:17:01Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoDataset.H"
#include "Robots/LoBot/metlog/LoExperiment.H"
#include "Robots/LoBot/metlog/LoPointTypes.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoSTL.H"

// Standard C++ headers
#include <algorithm>
#include <functional>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Dataset::Dataset()
{
   m_list.reserve(25) ; // just a guess
}

//---------------------------- LIST ACCESS ------------------------------

void Dataset::add(Experiment* E)
{
   AutoMutex M(m_mutex) ;
   m_list.push_back(E) ;
}

// These comparison functions are used in conjunction with std::sort to
// find the "reference experiment" in a dataset, i.e., the one that has
// the median number of points in the point list of interest.
static bool cmp_trajectory(const Experiment* a, const Experiment* b)
{
   return a->trajectory_size() < b->trajectory_size() ;
}

static bool cmp_emergency_stop(const Experiment* a, const Experiment* b)
{
   return a->emergency_stop_size() < b->emergency_stop_size() ;
}

static bool cmp_extricate(const Experiment* a, const Experiment* b)
{
   return a->extricate_size() < b->extricate_size() ;
}

static bool cmp_lgmd_extricate(const Experiment* a, const Experiment* b)
{
   return a->lgmd_extricate_size() < b->lgmd_extricate_size() ;
}

// Sort the experiment list in order to find the one with the median
// number of points in one of the experiment's point lists.
const Experiment* Dataset::find_refexp(PointListName L) const
{
   if (m_list.empty())
      throw misc_error(LOGIC_ERROR) ;

   typedef bool (*CMP)(const Experiment*, const Experiment*) ;
   const CMP cmp[] = {
      cmp_trajectory,
      cmp_emergency_stop,
      cmp_extricate,
      cmp_lgmd_extricate,
   } ;

   std::sort(m_list.begin(), m_list.end(), cmp[L]) ;
   return m_list[m_list.size()/2] ;
}

// Thread-safe sequential access to the experiment list
const Experiment* Dataset::next() const
{
   AutoMutex M(m_mutex) ;
   if (m_next == m_list.end())
      throw eol() ;
   return *m_next++ ;
}

// Resetting sequential access to the beginning of the experiment list
void Dataset::rewind()
{
   if (m_list.empty())
      throw misc_error(LOGIC_ERROR) ;
   m_next = m_list.begin() ;
}

//----------------------------- CLEAN-UP --------------------------------

Dataset::~Dataset()
{
   purge_container(m_list) ;
}

//--------------------------- DEBUG SUPPORT -----------------------------

void Dataset::dump() const
{
   std::for_each(m_list.begin(), m_list.end(),
                 std::mem_fun(&Experiment::dump)) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
