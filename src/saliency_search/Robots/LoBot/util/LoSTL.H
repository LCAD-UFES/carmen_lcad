/**
   \file  Robots/LoBot/util/LoSTL.H
   \brief STL helpers.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoSTL.H $
// $Id: LoSTL.H 13922 2010-09-11 03:46:14Z mviswana $
//

#ifndef LOBOT_STL_UTILITIES_DOT_H
#define LOBOT_STL_UTILITIES_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <algorithm>
#include <functional>
#include <iterator>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- PAIR FUNCTIONS -----------------------------

/// Functions to retrieve the individual elements of a pair.
//@{
template<typename P>
typename P::first_type  get_first(const P& p)  {return p.first ;}

template<typename P>
typename P::second_type get_second(const P& p) {return p.second ;}
//@}

//------------------------- DELETE FUNCTIONS ----------------------------

/// Pointer deletion (useful in conjunction with STL algorithms and
/// containers).
///
/// WARNING: P must be a pointer type (e.g., int* rather than int).
template<typename P> void delete_ptr(P p) {delete p ;}

/// Pointer deletion when the pointer is part of an std::pair
//@{
template<typename P> void delete_first (P& p) {delete p.first ;}
template<typename P> void delete_second(P& p) {delete p.second ;}
//@}

//------------------------ CONTAINER CLEAN-UP ---------------------------

/// Releasing an STL container of pointers using a specific delete
/// function (useful in cases of std::maps).
///
/// C: container type
/// D: delete function object type
///
/// WARNING: C *must* be an STL container or *must* provide an equivalent
/// begin/end iterator interface.
///
/// This function would typically be used to clean-up maps. For example,
/// an std::map<int, int*>. In this case, we cannot use delete_ptr<int*>
/// directly because the map iterators point to std::pair objects whose
/// first member is the key (int in this case) and whose second member is
/// the value (int* in this case).
///
/// Thus, we need to supply an appropriate delete function. In this case,
/// that function would be delete_second<std::pair<int, int*>>.
template<typename C, typename D>
void purge_container(C& c, D delete_fn)
{
   std::for_each(c.begin(), c.end(), delete_fn) ;
}

/// Releasing an STL container of pointers using the delete_ptr function.
/// C: container type
///
/// WARNING: C *must* be an STL container or provide an equivalent
/// begin/end iterator interface and value_type typedef.
///
/// This function would typically be used to clean-up a containers such as
/// std::vector<int*> or other such containers of pointers. In this case,
/// value_type would be int*.
template<typename C>
void purge_container(C& c)
{
   typedef typename C::value_type P ;
   purge_container(c, delete_ptr<P>) ;
}

//------------------------- FUNCTION OBJECTS ----------------------------

/// The STL already provides canned function objects for relational
/// operators. These two are for the min and max functions. They can be
/// used in conjunction with binders to quickly write calls to
/// std::transform(), etc. without having to create custom function
/// objects.
//@{
template<typename T>
struct min : std::binary_function<T, T, T> {
   T operator()(const T& a, const T& b) const {
      return std::min(a, b) ;
   }
} ;

template<typename T>
struct max : std::binary_function<T, T, T> {
   T operator()(const T& a, const T& b) const {
      return std::max(a, b) ;
   }
} ;
//@}

/// This function object is meant to be used in conjunction with the
/// std::accumulate algorithm and returns the sum of the squares of the
/// sequence to which the accumulate algorithm is applied.
template<typename T>
struct sum_of_squares : std::binary_function<T, T, T> {
   T operator()(const T& a, const T& b) const {
      return T(a + b*b) ;
   }
} ;

/// This function object is a predicate that returns true if the number
/// passed to its function call operator is out of the specified range.
template<typename T>
class out_of_range : public std::unary_function<T, bool> {
   range<T> m_range ;
public:
   out_of_range(const range<T>& R) : m_range(R) {}
   bool operator()(const T& t) const {return ! m_range.in(t) ;}
} ;

/// This object acts as an iterator that can be passed to algorithms such
/// as std::transform. However, instead of iterating across a container,
/// this iterator accumulates the source values passed to it. Typical
/// usage of this is shown below:
///
///    accumulator<int> acc =
///        std::transform(begin, end, accumulator<int>(0), f) ;
///    std::cout << "accumulated value is: " << acc.value() << '\n' ;
///
/// In the above example, f is a function or object that takes arguments
/// of the type contained between begin and end and returns an int.
///
/// Naturally, the std::accumulate algorithm would do the exact same
/// thing as the above algorithm with a lot less fuss. However,
/// accumulate cannot work with two different sequences. For instance, if
/// we have two sequences and we would like to determine a least squares
/// error between the two sequences using an STL algorithm and without
/// having to write a custom function or function object to take care of
/// the whole thing, this accumulator type along with boost::bind,
/// std::minus and a squaring function is just the ticket:
///
///    accumulator<float> acc =
///        std::transform(begin1, end1, begin2, accumulator<float>(0),
///                       boost::bind(sqr<float>,
///                                   boost::bind(std::minus<float>, _1, _2)));
///    std::cout << "square error is: " << acc.value() << '\n' ;
///
/// NOTE: The sqr used above is defined in util/LoMath.H.
template<typename T>
class accumulator :
      public std::iterator<std::output_iterator_tag, T, void, void, void> {
   T acc ;
public:
   accumulator(T init = T()) : acc(init) {}
   accumulator& operator=(T t)  {acc += t ; return *this ;}
   accumulator& operator*()     {return *this ;}
   accumulator& operator++()    {return *this ;}
   accumulator  operator++(int) {return *this ;}
   T value() const {return acc ;}
} ;

//---------------------------- MAP HELPERS ------------------------------

/// By default, STL maps compare keys when used with various STL
/// algorithms such as min_element and max_element. This function object
/// performs the comparison by comparing the values stored in the map
/// rather than its keys. It uses the < operator for comparisons.
///
/// NOTE: The type M is expected to be an std::map.
template<typename M>
class map_value_comp_less {
   typedef typename M::value_type value_type ;
public:
   map_value_comp_less(){}
   bool operator()(const value_type& A, const value_type& B) const {
      return A.second < B.second ;
   }
} ;

/// By default, STL maps compare keys when used with various STL
/// algorithms such as min_element and max_element. This function object
/// performs the comparison by comparing the values stored in the map
/// rather than its keys. It uses the supplied comparator function or
/// function object rather than the < operator to perform comparisons.
///
/// NOTE: The type M is expected to be an std::map. The type C should be
/// a comparator function object such as std::less or std::greater.
template<typename M, typename C>
class map_value_comp {
   C comp ;
   typedef typename M::value_type value_type ;
public:
   map_value_comp(const C& c) : comp(c) {}
   bool operator()(const value_type& A, const value_type& B) const {
      return comp(A.second, B.second) ;
   }
} ;

/// Helper function to return a map_value_compare_less object. This
/// helper exists to take advantage of the compiler's automatic type
/// deduction facility so as to alleviate users from having to explicitly
/// supply a type when instantiating the map_value_compare_less object.
/// Instead, at the call site, calling this function with the std::map
/// object is enough.
template<typename M>
map_value_comp_less<M>
map_value_compare(const M&)
{
   return map_value_comp_less<M>() ;
}

/// Helper function to return a map_value_comp object. This helper exists
/// to take advantage of the compiler's automatic type deduction facility
/// so as to alleviate users from having to explicitly supply the
/// necessary types when instantiating the comparator object. Instead,
/// users only need to pass this function an std::map object and a
/// comparator such std::greater.
template<typename M, typename C>
map_value_comp<M,C>
map_value_compare(const M&, const C& c)
{
   return map_value_comp<M,C>(c) ;
}

/// Unfortunately, the STL transform algorithm cannot be applied to STL
/// maps because std::map::iterator is not mutable, meaning that *i = p
/// is not a valid expression (where i is an std::map::iterator and p is
/// of type std::map::value_type). However, i->second = p is a valid
/// expression.
///
/// This function uses the above workaround to implement the transform
/// algorithm for maps.
///
/// Like the STL transform algorithm, it expects two input iterators
/// (type I), one output iterator (type O) and a unary function or
/// function object (type F). Unlike the standard transform algorithm,
/// however, the input and output iterator types are expected to be for a
/// pair associative container, i.e., a container whose value_type is an
/// std::pair.
///
/// The functional of type F will be passed the second member of the
/// iterator, which, for an std::map, is the contained data or value
/// (while the first member of the iterator is the key).
template<typename I, typename O, typename F>
O transform_map(I i, I end, O out, F f)
{
   for (; i != end; ++i, ++out)
      out->second = f(i->second) ;
   return out ;
}

/// Unfortunately, the STL transform algorithm cannot be applied to STL
/// maps because std::map::iterator is not mutable, meaning that *i = p
/// is not a valid expression (where i is an std::map::iterator and p is
/// of type std::map::value_type). However, i->second = p is a valid
/// expression.
///
/// This function uses the above workaround to implement the transform
/// algorithm for maps.
///
/// Like the STL transform algorithm, it expects two input iterators
/// (type I), one output iterator (type O) and a function or function
/// object (type F). Unlike the standard transform algorithm, however,
/// the input and output iterator types are expected to be for a pair
/// associative container, i.e., a container whose value_type is an
/// std::pair. Furthermore, the function or function object (type F) that
/// performs the transformation is expected to be a binary function
/// rather than a unary function.
///
/// The functional of type F will be passed both the first and second
/// members of the input iterator, i.e., both the map's key and value.
/// This is useful in cases where the transformation requires both the
/// key and the value (see the gaussian_weight function object for an
/// example).
template<typename I, typename O, typename F>
O transform_map2(I i, I end, O out, F f)
{
   for (; i != end; ++i, ++out)
      out->second = f(i->first, i->second) ;
   return out ;
}

/// Since STL map iterators point to STL pair objects, they can be a
/// little inconvenient to use with the STL for_each algorithm,
/// especially when the function to be performed on each element of the
/// map expects the map's contents as a parameter rather than a key-value
/// pair. This function implements the for_each algorithm assuming that
/// it is iterating over a map.
template<typename I, typename F>
F for_each_map(I i, I last, F f)
{
   for (; i != last; ++i)
      f(i->second) ;
   return f ;
}

//---------------------------- ALGORITHMS -------------------------------

/// The STL does not provide an algorithm for copying one sequence to
/// another based on some arbitrary test involving the contents of the
/// source sequence. This quick function rectifies that problem.
template<typename I, typename O, typename P>
void copy_if(I i, I last, O o, P pred)
{
   for (; i != last; ++i, ++o)
      if (pred(*i))
         *o = *i ;
}

/// This algorithm is performs a conditional copy from one sequence to
/// another. In addition to the predicate function, it takes a
/// "transformer" function that can be used to transform/convert the
/// object type pointed to by the input iterator into the object type for
/// the destination sequence.
///
/// For example, if we want to copy a vector of strings to a vector of
/// ints, we can supply a suitable function or function object that will
/// convert the source vector's strings into ints. This function will
/// then pass the source vector's contents through the supplied converter
/// function and copying this function's return value to the destination
/// int vector.
template<typename I, typename O, typename P, typename F>
void copy_if(I i, I last, O o, P pred, F func)
{
   for (; i != last; ++i, ++o)
      if (pred(*i))
         *o = func(*i) ;
}

/// Quick helper to connect whatever is in the supplied container to the
/// specified target object using push_back(). (Thus, the target object
/// must supply a push_back() method and other necessary STL glue to make
/// this work.)
template<typename C, typename T>
inline void connect(const C& container, T& target)
{
   std::copy(container.begin(), container.end(), std::back_inserter(target)) ;
}

/// Quick helper to connect whatever is in the supplied container to the
/// specified target object using push_back(). (Thus, the target object
/// must supply a push_back() method and other necessary STL glue to make
/// this work.)
template<typename C, typename T, typename F>
inline void connect_if(const C& src, T& dst, F pred)
{
   copy_if(src.begin(), src.end(), std::back_inserter(dst), pred) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
