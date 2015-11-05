/*! A ParamList is a statically sized array of arbitrary types */
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
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/Util/ParamList.H$

#ifndef UTIL_PARAMLIST_H_DEFINED
#define UTIL_PARAMLIST_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

//######################################################################
//a type to represent an error - useful to coerce gcc to give better
//error messages
//######################################################################
namespace paramlist_error
{
  struct OutOfBounds { };
}

//######################################################################
//declare ParamList and its base case - see ParamList below
//######################################################################
//declare the ParamList
template <typename... Params> class ParamList;

//ParamList class - base case in template recursion - see ParamList below
//######################################################################
template <> class ParamList<>
{   
public:
  //typedefs 
  typedef paramlist_error::OutOfBounds ParamType;
  typedef paramlist_error::OutOfBounds BaseType;

  //the length of our type array
  enum {length = 0};
  
  //set the current params
  void setParams() { };
  
  //assign another paramlist to this one, return true if something is assigned. 
  //This ones for when both paramlists are empty
  bool assign(const ParamList<>& rhs, int count = 0) { return (count > 0) ? true : false; };
  
  //assign another paramlist to this one, return true if something is assigned.
  //this ones for when the rhs actually has paramters
  template<typename... Params>
  bool assign(const ParamList<Params...>& rhs, int count = 0) { return false; };
  
  //defining this function and the dummy parameter are not necessary
  //but allows for more intelligable gcc errors if we attempt to
  //access outside the length of ParamList. Basically, allow
  //everything to work and then get an assignment error when we try to
  //assign our bogus paramlist_error::OutOfBounds class to an actual
  //type.
  ParamType& head() {return parameter; };
  const ParamType& head() const {return parameter; };

  //get the tail, which is just itself. 
  ParamList& tail() {return *this; };
  const ParamList& tail() const {return *this; };

private: 
  ParamType parameter;//a dummy variable for head()
};

//######################################################################
//a helper class to get the entries at locations in the
//ParamList. ParamEntry gets entry at the desired depth, and
//ParamEntryType gets the entry that matches the desired type.
//######################################################################
namespace ParamListHelper
{
  //######################################################################
  //make a parameter list
  //######################################################################
  template <class... Params> 
  ParamList<Params...> make_paramlist(const Params&... params)
  {
    return ParamList<Params...>(params...);
  }
  
  //######################################################################
  //get the parameter at the specifed location.
  //######################################################################
  template <uint depth, class... Params> struct ParamEntry;

  //get the parameter at the specifed location - error case
  //######################################################################
  template <uint depth> 
  struct ParamEntry<depth>
  {
    typedef paramlist_error::OutOfBounds ParamType;
    
    //return an error class
    static ParamType& value(ParamList<>& pfunc) 
    {
      return pfunc.head();
    };

    static const ParamType& value(const ParamList<>& pfunc) 
    {
      return pfunc.head();
    };
  };  

  //get the parameter at the specifed location - base case
  //######################################################################
  template <class Head, class... Params> 
  struct ParamEntry<0, Head, Params...>
  {
    //the type of our paramter at position 0
    typedef Head ParamType;

    //value of paramter at position 0
    static ParamType& value(ParamList<Head, Params...>& pfunc) 
    {
      return pfunc.head();
    };

    //value of paramter at position 0 - const version
    static const ParamType& value(const ParamList<Head, Params...>& pfunc)
    {
      return pfunc.head();
    };
  };
  
  //get the parameter at the specifed location - recursive case
  //######################################################################
  template <uint depth, class Head, class... Params> 
  struct ParamEntry<depth, Head, Params...>
  {
    //get the parameter type at depth-1
    typedef typename ParamEntry<depth-1, Params...>::ParamType ParamType;
    
    //return the value at depth-1
    static ParamType& value(ParamList<Head, Params...>& pfunc) 
    {
      return ParamEntry<depth-1, Params...>::value(pfunc.tail());
    };
    
    //return the value at depth-1 - const version
    static const ParamType& value(const ParamList<Head, Params...>& pfunc)
    {
      return ParamEntry<depth-1, Params...>::value(pfunc.tail());
    };
  };  
  
  //######################################################################
  //get the parameter at the first matching type
  //######################################################################
  template <typename T, typename... Params> struct ParamEntryType;

  //give a better error message if the item is not found
  //######################################################################
  template <typename T> 
  struct ParamEntryType<T>
  {
    typedef paramlist_error::OutOfBounds ParamType;
   
    //return an error class
    static ParamType& value(ParamList<>& pfunc) 
    {
      return pfunc.head();
    };

    static const ParamType& value(const ParamList<>& pfunc) 
    {
      return pfunc.head();
    };
  };

  //match case
  //######################################################################
  template <typename T, typename... Params> 
  struct ParamEntryType<T, T, Params...>
  {
    typedef T ParamType;
    
    //return the value at head
    static ParamType& value(ParamList<T, Params...>& pfunc) 
    {
      return pfunc.head();
    };
    
    //return the value at head - const version
    static const ParamType& value(const ParamList<T, Params...>& pfunc)
    {
      return pfunc.head();
    };
  };  

  //get the parameter at the first matching type - recursive case
  //######################################################################
  template <typename T, typename Head, typename... Params> 
  struct ParamEntryType<T, Head, Params...>
  {
    //typedef
    typedef T ParamType;
    
    //get the parameter value
    static ParamType& value(ParamList<Head, Params...>& pfunc) 
    {
      return ParamEntryType<T, Params...>::value(pfunc.tail()); 
    };
    
    //get the parameter value - const version
    static const ParamType& value(const ParamList<Head, Params...>& pfunc)
    {
      return ParamEntryType<T, Params...>::value(pfunc.tail()); 
    };
  };  

  //######################################################################
  //convenient access functions
  //######################################################################

  //get the parameter at the specifed location.
  //######################################################################
  template <uint depth, typename Head, typename... Params>
  typename ParamEntry<depth, Head, Params...>::ParamType& at(ParamList<Head, Params...>& paramlist)
  {
    return ParamEntry<depth, Head, Params...>::value(paramlist);
  }

  //get the parameter at the specifed location - const versin
  //######################################################################
  template <uint depth, typename Head, typename... Params>
  const typename ParamEntry<depth, Head, Params...>::ParamType& at(const ParamList<Head, Params...>& paramlist)
  {
    return ParamEntry<depth, Head, Params...>::value(paramlist);
  }

  //get the parameter with the specified type.
  //######################################################################
  template <typename T, typename Head, typename... Params>
  T& at(ParamList<Head, Params...>& paramlist)
  {
    return ParamEntryType<T, Head, Params...>::value(paramlist);
  }

  //get the parameter with the specifed type - const versin
  //######################################################################
  template <typename T, typename Head, typename... Params>
  const T& at(const ParamList<Head, Params...>& paramlist)
  {
    return ParamEntryType<T, Head, Params...>::value(paramlist);
  }
}//end namespace

//######################################################################
// ParamList class
//
// An array of arbitrary types. Types and length must be determined at
// compile-time but values can be set at runtime. Useful to pass
// paramter information around.
//
// Usage:
// 
//  using namespace ParamListHelper;
//
//  //create a paramlist and prints its values
//  ParamList<float,int,double> fun(1.1f, 2, 1.2);
//  LINFO("%3.2f, %d, %3.2f", at<0>(fun), at<1>(fun), at<2>(fun));
//
//  //reset params
//  fun.setParams(0.0f, 0, 0.0);
//  LINFO("%3.2f, %d, %3.2f", at<0>(fun), at<1>(fun), at<2>(fun));

//  //another way to create and access
//  auto fun1 = make_paramlist(14.4F,3);
//  at<0>(fun1) = 5.5F; //assign a value
//  float f = at<0>(fun1); //retrive a value by number
//  int i = at<int>(fun1); //retrive by type (first matching)
//  LINFO("%3.3f, %d", f, i);
//
// //just reassign a subset of the parameters 
// ParamList<float,int,double,float> p(0.0f,0,0.0,0.0f);
// ParamList<int,double> p1(1,1.0);
// p.assign(p1); //The second and third parameters will be set to 1 and 1.0, resp. 
//
//Notes: 
// 1) If assignment is called on two lists with no pattern shared the operation 
// will silently do nothing. The whole rhs pattern must be matched with a subpattern
// on the lhs. 

// 2)This implementation does not support references, but could
// with some simple type traits.
//######################################################################

//ParamList class - recursive case in template recursion
//######################################################################
template <typename Head, typename... Params>
class ParamList<Head, Params...> : private ParamList<Params...>
{
public:
  //typedefs    
  typedef Head ParamType;
  typedef ParamList<Params...> BaseType;

  //the length of our type array
  enum {length = sizeof...(Params) + 1};

  //empty constructor 
  ParamList() : ParamList<Params...>(), parameter(Head()) { };
  
  //constructor 
  ParamList(const Head& head, const Params&... params) 
  : ParamList<Params...>(params...), parameter(head) { };

  //default copy ctor OK
  
  //destructor
  ~ParamList() { };
  
  //set the parameters
  void setParams(const Head& head, const Params&... params)
  {
    parameter = head;
    BaseType::setParams(params...);
  };
 
  //Assign a list. The rhs may have a different number or type of arguments. 
  //The first type sub-pattern that matches the rhs type pattern will be assigned. 
  //If no match occurs the operation does nothing. Return true if something 
  //is assigned. 
  
  //base case, for when the rhs is empty, return true if something is assigned. 
  bool assign(const ParamList<>& rhs, int count = 0) { return (count > 0) ? true : false;}
  
  //match case
  template <typename... Types>
  bool assign(const ParamList<Head, Types...>& rhs, int count = 0)
  {
    bool temp = tail().assign(rhs.tail(), ++count);
    if (temp == true)
      parameter = rhs.head();
    return temp;
  }
  
  //no match case
  template <typename... Types>
  bool assign(const ParamList<Types...>& rhs, int count = 0)
  {
    if (count > 0) 
      return false;
    return tail().assign(rhs, count);
  }
  
  //get the head of the list
  ParamType& head() {return parameter; };
  const ParamType& head() const {return parameter; };
  
  //get the tail of the list (taking advantage of slicing of *this)
  BaseType& tail() {return *this; };
  const BaseType& tail() const {return *this; };
  
private:  
  Head parameter;//parameter for this level in the list
};

#endif //INVT_USE_CPP11
#endif//UTIL_PARAMLIST_H_DEFINED
