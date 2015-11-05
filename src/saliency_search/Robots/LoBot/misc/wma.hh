/**
   \file  Robots/LoBot/misc/wma.hh
   \brief Generic weighted moving average.

   This file defines a class that implements a weighted moving average.
   From Wikipedia:

   A weighted average is any average that has multiplying factors to give
   different weights to different data points. Mathematically, the moving
   average is the convolution of the data points with a moving average
   function; in technical analysis, a weighted moving average (WMA) has
   the specific meaning of weights that decrease arithmetically. In an
   N-day WMA the latest day has weight N, the second latest N-1, etc.,
   down to zero.

                N x[n] + (N-1) x[n-1] + ... + 2 x[n-N+2] + x[n-N+1]
     WMA[n]  =  ---------------------------------------------------
                            N + (N-1) + ... + 2 + 1

   The denominator is a triangle number and can be easily computed as
   N(N+1)/2.

   When calculating the WMA across successive values, it can be noted the
   difference between the numerators of WMA[n+1] and WMA[n] is
   N x[n+1] - x[n] - ... - x[n-N+1].

   If we denote the sum x[n] + ... + x[n-N+1] by total[n], then:

               total[n+1] = total[n] + x[n+1] - x[n-N+1]

       ==> numerator[n+1] = numerator[n] + n x[n+1] - total[n]

                            numerator[n+1]
       ==>       WMA[n+1] = --------------
                               N(N+1)/2

   DEVNOTE: Unfortunately, the above optimization doesn't work very well,
   especially with floating point. One rather pernicious problem occurs
   when all the data points are zero but the total and numerator are not
   because of small floating point roundoff errors that keep accumulating
   so that the filtered value never reaches zero.

   Therefore, this implementation of the weighted moving average uses the
   brute force approach to computing the numerator.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/wma.hh $
// $Id: wma.hh 13171 2010-04-06 19:56:06Z mviswana $
//

#ifndef LOBOT_WEIGHTED_MOVING_AVERAGE_DOT_HH
#define LOBOT_WEIGHTED_MOVING_AVERAGE_DOT_HH

//------------------------------ HEADERS --------------------------------

// Standard C++ headers
#include <algorithm>
#include <deque>
#include <stdexcept>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::wma
   \brief A weighted moving average implementation.

   This class implements a generic weighted moving average calculator as
   described in Wikipedia. Although defined as a template, it is really
   meant to be used only with numeric types.
*/
template<typename T>
class wma {
   // Prevent copy and assignment
   wma(const wma&) ;
   wma& operator=(const wma&) ;

   /// To compute a weighted moving average, we need to know N, the
   /// number of previous data items to be considered. We also need to
   /// store the previous N items somewhere.
   //@{
   int m_N ;
   typedef std::deque<T> Data ;
   Data m_data ;
   //@}

   /// To make the WMA computation a wee bit faster, we note that the
   /// denominator is simply the sum of the first N integers, i.e.,
   /// N(N+1)/2.
   int m_denominator ;

public:
   /// When instantiating a WMA object, clienst must specify the weighted
   /// moving average's "data window" size. An std::runtime_error will be
   /// thrown if this size is <= zero.
   wma(int n) ;

   /// If required, clients may resize the data window after
   /// instantiation. If the window is resized after instantiation, the
   /// WMA calculator will reset itself and start over.
   ///
   ///  An std::runtime_error will be thrown if the supplied window size
   ///  is <= zero.
   void resize(int n) ;

   /// This method adds a new data item to the weighted moving average's
   /// "window" of data items. For example, if N is 10, then the
   /// lobot::wma class will compute a weighted moving average of the
   /// most recent 10 data items. When a new data item comes along, it
   /// will be added to the internal queue and the oldest one will be
   /// removed.
   void add(T data) {m_data.pop_back(); m_data.push_front(data) ;}

   /// Returns the weighted moving average of the N most recently
   /// recorded data items.
   T value() const ;

   /// Sometimes, clients may find it necessary to reset the WMA
   /// calculator and start over again.
   void reset() ;

   /// Clean-up.
   ~wma() ;
} ;

//-------------------------- INITIALIZATION -----------------------------

template<typename T>
wma<T>::wma(int n)
   : m_N(0), m_denominator(0)
{
   resize(n) ;
}

template<typename T>
void wma<T>::resize(int n)
{
   if (n <= 0)
      throw std::runtime_error("bad window size for WMA") ;

   m_N = n ;
   if (! m_data.empty())
      m_data.clear() ;
   m_data.resize(n) ;
   m_denominator = n*(n+1)/2 ;
}

template<typename T>
void wma<T>::reset()
{
   std::fill(m_data.begin(), m_data.end(), T()) ;
}

//---------------- WEIGHTED MOVING AVERAGE COMPUTATION ------------------

template<typename T>
T wma<T>::value() const
{
   T numerator = 0 ;
   typename Data::const_iterator it = m_data.begin() ;
   for (int w = m_N; it != m_data.end(); ++it, --w)
      numerator += w * *it ;
   return static_cast<T>(numerator/m_denominator) ;
}

//----------------------------- CLEAN-UP --------------------------------

template<typename T>
wma<T>::~wma()
{}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
