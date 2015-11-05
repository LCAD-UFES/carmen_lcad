/**
   \file  triple.hh
   \brief A convenient structure for holding three heterogeneous objects.

   The STL pair class provides a convenient holder for two objects of
   different types. The template defined in this file serves a similar
   purpose except that it's for three objects.
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/triple.hh $
// $Id: triple.hh 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_TRIPLE_DOT_H
#define LOBOT_TRIPLE_DOT_H

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/// A convenient means of putting three things together (like an STL
/// pair, which puts two things together).
template<typename T1, typename T2, typename T3>
struct triple {
   /// The type of the triple's first component.
   typedef T1 first_type ;

   /// The type of the triple's second component.
   typedef T2 second_type ;

   /// The type of the triple's third component.
   typedef T3 third_type ;

   /// The three objects held by the triple.
   //@{
   first_type  first ;
   second_type second ;
   third_type  third ;
   //@}

   /// Default constructor. The three objects are each constructed with
   /// their respective default constructors. If they don't have default
   /// constructors, this constructor cannot be used.
   triple() ;

   /// Explicit constructor that constructs the triple using the supplied
   /// objects.
   triple(const first_type&, const second_type&, const third_type&) ;

   /// Copy constructor. Each of the three types must have corresponding
   /// copy constructors.
   triple(const triple&) ;

   /// Assignment operator. Each of the three types must have
   /// corresponding assignment operators.
   triple& operator=(const triple&) ;
} ;

//-------------------------- INITIALIZATION -----------------------------

// Default constructor
template<typename T1, typename T2, typename T3>
triple<T1, T2, T3>::triple()
   : first(T1()), second(T2()), third(T3())
{}

// Triple constructor
template<typename T1, typename T2, typename T3>
triple<T1, T2, T3>::triple(const T1& t1, const T2& t2, const T3& t3)
   : first(t1), second(t2), third(t3)
{}

// Copy constructor
template<typename T1, typename T2, typename T3>
triple<T1, T2, T3>::triple(const triple& t)
   : first(t.first), second(t.second), third(t.third)
{}

// Assignment
template<typename T1, typename T2, typename T3>
triple<T1, T2, T3>&
triple<T1, T2, T3>::
operator=(const triple& t)
{
   if (&t != this) {
      first  = t.first ;
      second = t.second ;
      third  = t.third ;
   }
   return *this ;
}

//----------------------- RELATIONAL OPERATORS --------------------------

/// Equality operator for triples. Returns true if and only if each of
/// the individual objects in the left triple equal the corresponding
/// fields in the right triple. Each of the triple's object types must
/// support the equality operator.
template<typename T1, typename T2, typename T3>
bool operator==(const triple<T1, T2, T3>& L, const triple<T1, T2, T3>& R)
{
   return L.first == R.first && L.second == R.second && L.third == R.third ;
}

/// Less-than operator for triples. Compares the first, second and third
/// objects in the left and right triples in turn and returns true if any
/// of the less-than tests succeeds and false if all the less-than tests
/// fail. Each of the triple's object types must support the less-than
/// operator.
template<typename T1, typename T2, typename T3>
bool operator<(const triple<T1, T2, T3>& L, const triple<T1, T2, T3>& R)
{
   return L.first < R.first || L.second < R.second || L.third < R.third ;
}

//----------------------- CONVENIENCE FUNCTIONS -------------------------

/// Returns a triple using the three objects supplied to it. This
/// function allows clients to construct a triple without having to
/// explicitly specify types as would be required with a class
/// instantiation. Instead, clients can take advantage of type deduction
/// for template functions and simply provide the parameters, letting the
/// compiler figure out the correct types.
template<typename T1, typename T2, typename T3>
inline triple<T1, T2, T3>
make_triple(const T1& t1, const T2& t2, const T3& t3)
{
   return triple<T1, T2, T3>(t1, t2, t3) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
