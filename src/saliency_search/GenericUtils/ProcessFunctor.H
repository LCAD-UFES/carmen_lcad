//! a data processing functor
//////////////////////////////////////////////////////////////////////////
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
//////////////////////////////////////////////////////////////////////////
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
//
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/GenericUtils/ProcessFunctor.H$

#ifdef  INVT_USE_CPP11//we need c++ 0X features for this to work
#ifndef PROCESSFUNCTOR_H_DEFINED
#define PROCESSFUNCTOR_H_DEFINED

#include "Util/log.H"
#include <typeinfo>

// ###################################################################### 
// A simple class to check if you can convert frome one type to another, and if
// so supplyies a confert function. By default everything is convertable to
// everything else, so supply your own specialization inside the definition of
// your ProcessFunctor derived class to prohibit converting or do additional
// operations besides a statoc_cast, or dynamic cast.
// ######################################################################
// general case
template<class From, class To> struct convertType {
  static const bool Convertable = 1;
  static To convert(const From& from)
  { return static_cast<To>(from); } };//this may return non-sense, but will never fail

//attempt to copy self
template<class From> struct convertType<From, From> {
  static const bool Convertable = 0;
  static From convert(const From& from)
  { return from; } };//this may return non-sense, but will never fail

//specialization for pointers, for now don't allow
template<class From, class To> struct convertType<From*, To*> {
  static const bool Convertable = 0; 
  static To convert(const From& from)
  { return dynamic_cast<To>(from); } };//never called
  
//specialization for Refs, for now don't allow.
  template<class From, class To> struct convertType<From&, To&> {
    static const bool Convertable = 0; 
  static To convert(const From& from)
  { return dynamic_cast<To>(from); } };//never called};
      
// ###################################################################### 
// ProcessFunctor
// ###################################################################### 
/*

  A ProcessFunctor embodies a computation that can occur for several different
  types. The possible types are known a priori but the the exact type that will
  be input at call point is not known by the functor. A genericItem uses this to
  performs operations on items generically, when the exact type of the item is
  not known by the Functor ahead of time. Several behaviors can be achieved. 1)
  several different actions can be performed depending on the input type (or the
  type in the generic item). 2) the same action can be performed, but with some
  type specific manipulations first or after. 3) The same action can be
  performed by converting several different types to a single type of
  interest. 4) data can be processed for some type matches and ignored for
  others, resulting in a type filtering type mechanism.

  The functor is executed by calling the function call operator. The operator
  takes a container of a homogenious type as its argument. Derived classes
  implement a process(C<T> inp) function, where C is the container type and T is
  the type held in the container. The Derived class can implement one or more
  process functions for different types T, allowing for a range of inputs to be
  handled (either by implementing with named types or as a template class). If a
  process function is not implemented for a particular T, the functor performs
  no operation, or converts to the first availible type.  Whether the functor
  performed an operation (and hence set some member variables for the result) is
  checked with a call to hasData().

  ProcessFunctor takes at least two template arguments. The first is the
  container type, and the second is a variadic template of known types.  Upon
  creation, ProcessFunctor will create virtual functions for all known types and
  implements a default non-operational function. The derived class should
  implement the process(C<T>) for all allowed types T (could use a template
  class to do this);

  By default, no conversions will take place if the functor does not know about
  the type. If the flag in the function call operator is set to true, conversion
  will be attempted. Conversion is always attempted when the function is non-op
  (ie the derived class only implements for one type that the functor has
  definitions for). This could potentially require many conversions if you
  immplement for many types. The conversion unraveling is done from last to first,
  so if you are only going to implement for one of the know types, then try to put
  that type last in the type list.x

  see test-generics.C for an example
*/
// ######################################################################
template <template <typename...> class C, class... Tail> class ProcessFunctor;

//base case in variadic template recursion
// ######################################################################
template <template <typename...> class C>
class ProcessFunctor<C>
{
public:
  //destructor
  virtual ~ProcessFunctor() { };
  
  //default copy constructor, assignment Ok
  
  //process the feature
  template <class T>
  void operator()(const C<T>& data, const bool try_convert = false) 
  { };
  
  //check to see if we have data
  bool hasData() const { return itsHasData; };  
  
protected:
  //constructor protected so that users cant create this class directly
  ProcessFunctor() : itsHasData(false) { };

  //set from that outside that we have data
  void setHasData(const bool hasdata) { itsHasData = hasdata; };
  
private:
  bool itsHasData;
};

//recursive case in variadic template
// ######################################################################
template <template <typename...> class C, class Head, class... Tail>
class ProcessFunctor<C, Head, Tail...> : private ProcessFunctor<C, Tail...>
{ 
public:
  //destructor
  virtual ~ProcessFunctor() { };
  
  //default copy constructor, assignment Ok
  
  //overload operator for matching head
  void operator()(const C<Head>& data, const bool try_convert = false) 
  { 
    setHasData(process(data)); 
    
    //If we matched but failed keep descending as we will hit base and then we
    //can pop all the way up trying to convert. Otherwise, we will return at
    //this point. The former happens when we know about a type but did not
    //implemented the virtual function in our derived.
    if (!hasData())
      ProcessFunctor<C, Tail...>::operator()(data, true); 
  };  
  
  //overload for not matching
  template <class T>
  void operator()(const C<T>& data, const bool try_convert = false) 
  { 
    //we didn't match, so keep going
    ProcessFunctor<C, Tail...>::operator()(data, try_convert); 
    
    // If we get here, we have it the bottom. As we unravel, check to see if we
    //ever set any data.  If we didn't and the flag is set, try to convert as we
    //pop back up
    if (!hasData() && try_convert)
      {
	//If we can convert, we can call the derived process and set the data
	//flag. If the operation succeeds, we will unravel with no ops. Else we
	//will attempt to convert and call for each type as we unravel.
	if (convertType<C<T>, C<Head> >::Convertable)
	  {
	    LINFO("conver");
	    C<Head> temp = convertType<C<T>, C<Head> >::convert(data);
	    setHasData(process(temp)); 
	  }
      }
  };
  
  //check to see if we have data
  bool hasData() const { return ProcessFunctor<C, Tail...>::hasData(); };  
  
protected:
  //constructor protected so that users cant create this class directly
  ProcessFunctor() : ProcessFunctor<C, Tail...>() { };
  
  //set the flag indicating data is set. 
  void setHasData(const bool hasData)
  { ProcessFunctor<C, Tail...>::setHasData(hasData); }
  
private:
  virtual bool process(const C<Head>& feature) 
  { return false; }; 
};

#endif
#endif
