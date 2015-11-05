/**
   \file  Robots/LoBot/util/LoStats.H
   \brief Some statistics related functions.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoStats.H $
// $Id: LoStats.H 14285 2010-12-01 17:33:15Z mviswana $
//

#ifndef LOBOT_STATISTICS_UTILITIES_DOT_H
#define LOBOT_STATISTICS_UTILITIES_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/LoMath.H"

// Standard C++ headers
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- "FULL" STATS ------------------------------

/**
   \class lobot::generic_stats
   \brief A structure to hold several descriptive statistics together.

   In some situations, we might want to know more than just the mean and
   standard deviation of the values held by a container. The
   lobot::generic_stats structure holds several other useful values in
   addition to the mean and standard deviation.
*/
template<typename T>
struct generic_stats {
   int n  ; //< number of elements from which mean and stdev are computed
   T sum  ; //< sum of all the values from which mean and stdev are computed
   T ssq  ; //< sum of squares of all values
   T mean ; //< arithmetic mean
   T stdev; //< standard deviation

   /// Constructor.
   generic_stats(int n = 0, T sum = 0, T ssq = 0, T mean = 0, T stdev = 0) ;
} ;

template<typename T>
generic_stats<T>::generic_stats(int N, T S, T SS, T mu, T sigma)
   : n(N), sum(S), ssq(SS), mean(mu), stdev(sigma)
{}

/// This function computes the mean and standard deviation of a sequence
/// of elements contained within the sequence pointed to by the supplied
/// input iterators. Additionally, it also returns the number of elements
/// in the sequence, the sum of all the elements in the sequence and the
/// sum of the squares of all the elements. All this information is
/// returned via a lobot::generic_stats structure.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// the input_iterator can be deduced automatically. Therefore, this
/// function should be invoked as compute_stats<T>(...).
template<typename T, typename input_iterator>
generic_stats<T>
compute_stats(input_iterator begin, input_iterator end)
{
   T sum = T() ;
   T ssq = T() ; // sum of squares
   int N = 0 ;
   for (input_iterator i = begin; i != end; ++i, ++N) {
      T x  = *i ;
      sum +=  x ;
      ssq += sqr(x) ;
   }
   return generic_stats<T>(N, sum, ssq, sum/N, sqrt(N * ssq - sqr(sum))/N) ;
}

/// This function computes the mean and standard deviation of a sequence
/// of elements contained within the sequence pointed to by the supplied
/// input iterators. The type for the mean and standard deviation do not
/// have to be the same as the type of elements in the container.
/// However, it must be possible to "convert" each contained element to
/// an object of type T. This "conversion" will be performed by a helper
/// function or function object of type converter_function. Generally,
/// the converter_function will be something like the adapter returned by
/// std::mem_fun_ref.
///
/// The mean and standard deviation are returned along with the number of
/// elements in the sequence, the sum of all the elements and the sum of
/// the squares in a lobot::generic_stats<T> structure.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// types input_iterator and converter_function can be deduced
/// automatically. Therefore, this function should be invoked as
/// compute_stats<T>(...).
template<typename T, typename input_iterator, typename converter_function>
generic_stats<T>
compute_stats(input_iterator begin, input_iterator end, converter_function F)
{
   T sum = T() ;
   T ssq = T() ; // sum of squares
   int N = 0 ;
   for (input_iterator i = begin; i != end; ++i, ++N) {
      T x  = F(*i) ;
      sum += x ;
      ssq += sqr(x);
   }
   return generic_stats<T>(N, sum, ssq, sum/N, sqrt(N * ssq - sqr(sum))/N) ;
}

//-------------------- MEAN AND STANDARD DEVIATION ----------------------

/// This function computes the average of all the elements in the
/// sequence specified by the supplied iterators. It is assumed that T is
/// the type of elements contained within the sequence and that
/// dereferencing the iterators yields objects of type T. Furthermore,
/// type T must support the += and / operators. Finally, the default
/// constructor for type T must yield a T initialized to zero.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// the input_iterator type can be deduced automatically. Therefore, this
/// function should be invoked as mean<T>(...).
template<typename T, typename input_iterator>
T mean(input_iterator begin, input_iterator end)
{
   T sum = T() ;
   int N = 0 ;
   for (input_iterator i = begin; i != end; ++i, ++N)
      sum += *i ;
   return sum/N ;
}

/// This function computes the average of all the elements in the
/// sequence specified by the supplied iterators. It is not necessary
/// that T be the type of elements contained within the sequence.
/// However, there must be some way of "converting" the contained type to
/// T. Therefore, a function or function object that performs this
/// "conversion" is required by this function. This "converter" function
/// will be passed the result of dereferencing the iterator type.
/// Generally speaking, the "converter" will be something like what is
/// returned by std::mem_fun_ref.
///
/// Type T must support the += and / operators. T's default constructor
/// must yield a T initialized to zero.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// input_iterator and converter_function types can be deduced
/// automatically. Therefore, this function should be invoked as
/// mean<T>(...).
template<typename T, typename input_iterator, typename converter_function>
T mean(input_iterator begin, input_iterator end, converter_function F)
{
   T sum = T() ;
   int N = 0 ;
   for (input_iterator i = begin; i != end; ++i, ++N)
      sum += F(*i) ;
   return sum/N ;
}

/// This function computes the standard deviation of all the elements in
/// the sequence specified by the supplied iterators. It is assumed that
/// T is the type of elements contained within the sequence and that
/// dereferencing the iterators yields objects of type T. Furthermore,
/// type T must support the += and / operators and be compatible with the
/// sqrt function. Finally, the default constructor for type T must yield
/// a T initialized to zero.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// the input_iterator type can be deduced automatically. Therefore, this
/// function should be invoked as stdev<T>(...).
template<typename T, typename input_iterator>
T stdev(input_iterator begin, input_iterator end)
{
   return compute_stats<T>(begin, end).stdev ;
}

/// This function computes the standard deviation of all the elements in
/// the sequence specified by the supplied iterators. It is not necessary
/// that T be the type of elements contained within the sequence.
/// However, there must be some way of "converting" the contained type to
/// T. Therefore, a function or function object that performs this
/// "conversion" is required by this function. The "converter" function
/// will be passed the result of dereferencing the iterator type.
/// Generally speaking, the "converter" will be something like what is
/// returned by std::mem_fun_ref.
///
/// Type T must support the += and / operators. It must be compatible
/// with the sqrt function. And T's default constructor must yield a T
/// initialized to zero.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// types input_iterator and converter_function can be deduced
/// automatically. Therefore, this function should be invoked as
/// stdev<T>(...).
template<typename T, typename input_iterator, typename converter_function>
T stdev(input_iterator begin, input_iterator end, converter_function F)
{
   return compute_stats<T>(begin, end, F).stdev ;
}

/// This function computes the mean and standard deviation for a sequence
/// of elements of type T contained within the container pointed to by
/// input_iterator. The two quantities computed by this function are
/// returned via an std::pair<T,T> with the mean being the first element
/// of the pair and the standard deviation being the second element.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// the input_iterator can be deduced automatically. Therefore, this
/// function should be invoked as mean_stdev<T>(...).
template<typename T, typename input_iterator>
std::pair<T, T>
mean_stdev(input_iterator begin, input_iterator end)
{
   generic_stats<T> s = compute_stats<T>(begin, end) ;
   return std::make_pair(s.mean, s.stdev) ;
}

/// This function computes the mean and standard deviation of a sequence
/// of elements contained within the sequence pointed to by the input
/// iterators. The type for the mean and standard deviation do not have
/// to be the same as the type of elements in the container. However, it
/// must be possible to "convert" each contained element to an object of
/// type T. This "conversion" will be performed by a helper function or
/// function object of type converter_function. Generally, the
/// converter_function will be something like the adapter returned by
/// std::mem_fun_ref.
///
/// The mean and standard deviation are returned in an std::pair<T,T>
/// wherein the first element is the mean and the second one the standard
/// deviation.
///
/// NOTE: Since type T does not figure in the function's argument list,
/// automatic type deduction will not work with this function. However,
/// types input_iterator and converter_function can be deduced
/// automatically. Therefore, this function should be invoked as
/// mean_stdev<T>(...).
template<typename T, typename input_iterator, typename converter_function>
std::pair<T, T>
mean_stdev(input_iterator begin, input_iterator end, converter_function F)
{
   generic_stats<T> s = compute_stats<T>(begin, end, F) ;
   return std::make_pair(s.mean, s.stdev) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
