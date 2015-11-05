/**
   \file  Robots/LoBot/tti/LoSensorModel.C
   \brief This file defines the non-inline member functions of the
   lobot::SensorModel class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/tti/LoSensorModel.C $
// $Id: LoSensorModel.C 13120 2010-04-01 08:29:56Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/tti/LoSensorModel.H"
#include "Robots/LoBot/lgmd/gabbiani/LoGabbiani.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoDebug.H"

// Standard C++ headers
#include <numeric>
#include <algorithm>
#include <functional>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from extricate section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("tti_estimator", key, default_value) ;
}

// Overload of above function for retrieving ranges
template<typename T>
static inline range<T>
conf(const std::string& key, const range<T>& default_value)
{
   return get_conf<T>("tti_estimator", key, default_value) ;
}

// Overload of above function for retrieving triples
template<typename T>
static inline triple<T, T, T>
conf(const std::string& key, const triple<T, T, T>& default_value)
{
   return get_conf<T>("tti_estimator", key, default_value) ;
}

//-------------------------- INITIALIZATION -----------------------------

// The constructor uses the appropriate settings in the config file to
// properly set up the sensor model for the specified "phase" of the LGMD
// input signal.
SensorModel::SensorModel(const std::string& lgmd_phase)
   : m_sigma(0.0f), m_name(lgmd_phase)
{
   const range<float> lgmd_range =
      get_conf(locust_model(), "spike_range", make_range(0.0f, 800.0f)) ;

   // Get the LGMD ranges for the columns of the sensor model
   m_lgmd_ranges = string_to_deque<float>(
      conf<std::string>(lgmd_phase + "_lgmd_ranges", "0 800")) ;
   if (m_lgmd_ranges.size() < 2) { // crappy configuration!
      m_lgmd_ranges.clear() ;
      m_lgmd_ranges.push_back(lgmd_range.min()) ;
      m_lgmd_ranges.push_back(lgmd_range.max()) ;
   }
   sort(m_lgmd_ranges.begin(), m_lgmd_ranges.end()) ;
   if (m_lgmd_ranges.front() > lgmd_range.min())
      m_lgmd_ranges.push_front(lgmd_range.min()) ;
   if (m_lgmd_ranges.back() < lgmd_range.max())
      m_lgmd_ranges.push_back(lgmd_range.max()) ;

   // Figure out how many rows and columns the sensor model's probability
   // table has and allocate space for the required number of elements.
   // Initialize the probability table using a uniform distribution.
   const int C = m_lgmd_ranges.size() - 1 ;
   const int R = column_size() ;
   const int N = R * C ;
   m_prob.reserve(N) ;
   std::fill_n(std::back_inserter(m_prob), N, 1.0f/N) ;

   // Apply Gabbiani model to obtain causal probabilities and Gaussian
   // blur neighbouring bins in each row.
   update(clamp(conf(lgmd_phase + "_sigma", 1.0f),
                0.1f, static_cast<float>(row_size()))) ;
}

// This method regenerates the sensor model's probabilities using the
// Gabbiani LGMD model and the given standard deviation for the Gaussian
// blurring operation for bins near the ones actually "pointed to" by the
// [TTI, LGMD] pairs returned by the Gabbiani model.
//
// DEVNOTE: The sigma provided to this function is actually added to the
// m_sigma member variable. This allows client behaviours to increment or
// decrement the current sigma value rather than provide an actual sigma.
// The very first sigma will be read from the config file (see
// constructor).
void SensorModel::update(float dsigma)
{
   AutoMutex M(m_mutex) ;

   // Record new standard deviation
   const float R = row_size() ;
   m_sigma = clamp(m_sigma + dsigma, 0.1f, R) ;

   // Begin with a uniform distribution for each state
   const int N = m_prob.size() ;
   std::fill_n(m_prob.begin(), N, 1/R) ;

   // Apply Gabbiani LGMD model to generate causal likelihoods
   const float step = row_step()/4.0f ;
   const range<float> tti = conf(m_name + "_tti_range", Params::tti_range()) ;
   for (float t = tti.min(); t <= tti.max(); t += step)
      update_row(t, GabbianiModel::spike_rate(t), m_sigma) ;
}

// This function increments the bin "pointed" to by the given [TTI, LGMD]
// pair. It also increments the other bins in the row "pointed" to by the
// TTI using a Gaussian weighting formula to ensure that no bin in that
// row has weird likelihood values that can screw up the Bayesian TTI
// estimation. Finally, it normalizes the row to ensure that each row
// vector is a valid probability distribution.
void SensorModel::update_row(float tti, float lgmd, float sigma)
{
   const int   N = row_size() ;
   const int   C = column_size() ;
   const int   I = col_index(lgmd) ;
   const float S = 1/(2 * sqr(sigma)) ;

   Table::iterator begin = m_prob.begin() + row_index(tti) ;

   float normalizer = 0 ;
   Table::iterator it = begin ;
   for (int i = 0; i < N; ++i, it += C) {
      *it += exp(-sqr(i - I) * S) ;
      //*it = exp(-sqr(i - I) * S) ;
      normalizer += *it ;
   }

   it = begin ;
   for (int i = 0; i < N; ++i, it += C)
      *it /= normalizer ;
}

//--------------------------- TABLE ACCESS ------------------------------

// This function returns the index of the column in the sensor model's
// probability table that corresponds to a given LGMD spike rate.
int SensorModel::col_index(float lgmd) const
{
   const int N = m_lgmd_ranges.size() - 1 ;
   for  (int i = 0; i < N; ++i)
      if (m_lgmd_ranges[i] <= lgmd && lgmd < m_lgmd_ranges[i + 1])
         return i ;
   return N - 1 ;
}

// Copy column vector specified by given LGMD value
std::vector<float> SensorModel::column_vector(float lgmd) const
{
   AutoMutex M(m_mutex) ;
   const int N = column_size() ;
   Table::const_iterator begin = m_prob.begin() + col_index(lgmd) * N ;
   return std::vector<float>(begin, begin + N) ;
}

// Copy entire probability table
std::vector<float> SensorModel::table() const
{
   AutoMutex M(m_mutex) ;
   return m_prob ;
}

//----------------------------- CLEAN-UP --------------------------------

SensorModel::~SensorModel(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
SensorModel::Params::Params()
   : m_tti_discretization(conf("tti_discretization",
                               make_triple(0.0f, 10.0f, 0.1f))),
     m_tti_range(clamp(make_range(m_tti_discretization.first,
                                  m_tti_discretization.second),
                       make_range(0.0f, 1000.0f))),
     m_tti_step(clamp(m_tti_discretization.third, 0.001f, m_tti_range.max())),
     m_belief_size(round(m_tti_range.size()/m_tti_step))
{}

// Parameters clean-up
SensorModel::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
