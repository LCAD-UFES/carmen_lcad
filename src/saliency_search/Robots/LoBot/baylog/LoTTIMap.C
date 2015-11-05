/**
   \file  Robots/LoBot/misc/LoTTIMap.C
   \brief This file defines the non-inline member functions of the
   lobot::TTIMap class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/baylog/LoTTIMap.C $
// $Id: LoTTIMap.C 14088 2010-10-01 19:43:37Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/baylog/LoTTIMap.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/util/LoStats.H"
#include "Robots/LoBot/util/LoMath.H"

// Standard C++ headers
#include <iomanip>
#include <sstream>
#include <utility>

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from global section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return lobot::global_conf<T>(key, default_value) ;
}

/// This inner class encapsulates various parameters that can be used to
/// tweak different aspects of the Bayesian TTI prediction analysis.
class TTIMapParams : public lobot::singleton<TTIMapParams> {
   /// The Bayesian time-to-impact state estimator is usually configured
   /// to predict the TTI within the range of zero to ten seconds in
   /// steps of a tenth of a second. However, if the Robolocust settings
   /// for the Bayesian TTI prediction experiments were different, we can
   /// specify the maximum value for the TTI predictions with this
   /// setting, which should be a floating point number whose units is
   /// seconds.
   ///
   /// When analyzing the log files for the Bayesian TTI prediction
   /// experiments, we will only consider those entries whose actual
   /// times-to-impact are less than the value of this setting because,
   /// eventually, the results files output by the lobay program will be
   /// used to produce plots showing the LGMD spike rate and TTI
   /// predictions versus the actual times-to-impact. Actual
   /// times-to-impact that are outside the range of the estimator's
   /// bounds will show up as being highly errorneous, which would be
   /// unfair. Therefore, we ignore such readings and concentrate only on
   /// those that are within the state estimation bounds.
   float m_max_tti ;

   /// Private constructor because this is a singleton.
   TTIMapParams() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class lobot::singleton<TTIMapParams> ;

public:
   /// Accessing the various parameters.
   //@{
   static float max_tti() {return instance().m_max_tti ;}
   //@}
} ;

// Parameters initialization
TTIMapParams::TTIMapParams()
   : m_max_tti(lobot::clamp(conf("max_tti", 10.0f), 1.0f, 60.0f))
{}

// Shortcut
typedef TTIMapParams Params ;

} // end of local anonymous namespace encapsulating above helpers

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

TTIMap::TTIMap(){}

void TTIMap::add(float tti, float lgmd, float predicted, float confidence)
{
   if (tti > Params::max_tti())
      return ;

   using std::right ; using std::setfill ; using std::setw ;
   using std::fixed ; using std::setprecision ;

   std::ostringstream str ;
   str << setfill('0') << setw(4) << right << fixed << setprecision(1)
       << tti ;

   std::string  key = str.str() ;
   Map::iterator it = m_map.find(key) ;
   if (it == m_map.end())
   {
      List L, P, C ;
      L.push_back(lgmd) ;
      P.push_back(predicted) ;
      C.push_back(confidence) ;
      m_map.insert(std::make_pair(key, make_triple(L, P, C))) ;
   }
   else
   {
      it->second.first.push_back(lgmd) ;
      it->second.second.push_back(predicted) ;
      it->second.third.push_back(confidence) ;
   }
}

//------------------------------ OUTPUT ---------------------------------

static std::pair<float, float> stats(const std::vector<float>& v)
{
   return mean_stdev<float>(v.begin(), v.end()) ;
}

TTIMap::dump::dump(std::ostream& s)
   : os(s)
{}

void TTIMap::dump::operator()(const TTIMap::MapEntry& E)const
{
   using std::fixed ; using std::setprecision ;
   using std::right ; using std::setw ;

   float actual_tti = from_string<float>(E.first) ;
   std::pair<float, float> lgmd = stats(E.second.first) ;
   std::pair<float, float> pred = stats(E.second.second);
   std::pair<float, float> conf = stats(E.second.third) ;
   conf.first  /= 100 ;
   conf.second /= 100 ;

   os << setw(4) << right << fixed << setprecision(1) << actual_tti  << ' '
      << setw(3) << right << fixed << setprecision(0) << lgmd.first  << ' '
      << setw(3) << right << fixed << setprecision(0) << lgmd.second << ' '
      << setw(4) << right << fixed << setprecision(1) << pred.first  << ' '
      << setw(4) << right << fixed << setprecision(1) << pred.second << ' '
      << setw(6) << right << fixed << setprecision(4) << conf.first  << ' '
      << setw(6) << right << fixed << setprecision(4) << conf.second << '\n' ;
}

std::ostream& operator<<(std::ostream& os, const TTIMap& M)
{
   std::for_each(M.m_map.begin(), M.m_map.end(), TTIMap::dump(os)) ;
   return os ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
