/*!@file ModelNeuron/NeuralSimUtils.H A bunch of utility classes,
   mostly some template idioms inspired from Alexandrescu, 2001*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/NeuralSimUtils.H$

#ifndef MODELNEURON_NEURALSIMUTILS_H_DEFINED
#define MODELNEURON_NEURALSIMUTILS_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "GenericUtils/GenericFactory.H"
#include "GenericUtils/CreateFunctor.H"

#include <iterator>
#include <vector>

namespace nsu
{
// ######################################################################
// Used to represent null and empty types 
// ######################################################################
  class NullType { };
  class EmptyType { };

// ######################################################################
// A simple class to 'typeify' an integral constant, useful for making
// decisions about possible control paths at compile time. From
// Alexandrescu, 2001
// ######################################################################
  template <int type>
  struct Int2Type
  {
    enum { value = type };
  };
  
// ######################################################################
// A simple class to hold type information. This is useful as a light
// weight way of choosing overloaded functions from types. From
// Alexandrescu, 2001
// ######################################################################
  template <class Type>
  struct Type2Type 
  {
    typedef Type type;
  };

  // ######################################################################
  // A simple class to choose one of two different types, from a compile
  // time decision. The type an be retrieved by: 
  //
  // Select<boolean expression, Class1, class2>::Result 
  //
  // Where if expression is true, ::Result will be of type Class1, else ::Result
  // will be of type Class2
  // From Alexandrescu, 2001
  // ######################################################################   
  template <bool expression, class C_true, class C_false>
  struct SelectType //if expression is true, Type will be of C_true type.
  {
    typedef C_true Type;
  };
  
  template <class C_true, class C_false>
  struct SelectType<false, C_true, C_false> //if false, call this specialization
  {
    typedef C_false Type;
  };
  
  // ###################################################################### 
  // A type traits class to deterimine properties of a template type
  // ######################################################################   
  template <typename T>
  class TypeTraits 
  {
  private:
    template <class U> struct PointerTraits 
    {
      enum { result = false };
      typedef U PointeeType;
    };

    template <class U> struct PointerTraits <U*>
    {
      enum { result = true };
      typedef U PointeeType;
    };

  public:
    enum { isPointer = PointerTraits<T>::result };
    typedef typename PointerTraits<T>::PointeeType PointeeType;
  };

  // ######################################################################
  //This function is used when the TypeCreator template argument of a 
  //ReusableFactory is a CreateFunctor, and one wants to set a parameter 
  //for all CreateFunctors in the the factory by the paramters type.  
  //######################################################################
  template <typename Factory, typename... Params>
  void setParameter(Factory& factory, const Params&... params)
  {
    typename Factory::iterator iter = factory.begin();
    while (iter != factory.end())
    {
      iter->second.updateParams(params...);
      ++iter;
    }
  }

  // ###################################################################### 
  //An extension to std::vector that handles polymorphic types without the user needing to double dereference the
  //pointer when using access operators and iterators (as if you had a non-pointer type) in your vector. The virtual
  //base must supply a clone() method which makes a copy of the object and returns a pointer to its base class. The
  //vector will call delete on its members when they need to be destroyed. If a non-pointer object is supplied this
  //vector works like a std::vector.

  //The const/non-const iterators work just like std::vector iterators when the object is not polyphomic. When the
  //vector contains a polymorphic type, the iterators provided have an additional pointer() function, which returns a
  //reference to the underlying pointer at the iterators position in the array. Using this function could be very
  //dangerous if you don't know what your doing and is not reccommended, put provided for advanced uses. For polymophic
  //classes, non-const iterators allow manipulation of the object or its pointer through pointer(). const iterators will
  //forbid modification of the object or its pointer through pointer(), but delete could still be called on it, so be
  //carefull.
  //######################################################################
  template<class T>
  class vector
  {
    std::vector<T> itsV;
    
  public:
    typedef typename std::vector<T>::iterator iterator;
    typedef typename std::vector<T>::const_iterator const_iterator;
    typedef T Type;
    typedef EmptyType PointerType;
    enum {isPointer = 0 };
    
    // constructor
    vector() : itsV() { };

    // constructor
    vector(size_t size, const T& value) : itsV(size, value) { };
    
    //destructor
    ~vector() { };
    
    // return a reference
    T& operator[](const int pos) { return itsV[pos]; };//not save for assignment, use set_at
    const T& operator[](const int pos) const { return itsV[pos]; };
    
    // set an element at the specified position
    void set_at(const uint pos, const T& element) { itsV[pos] = element; };

    // push an element on
    void push_back(const T& element) { itsV.push_back(element); };
    
    // pop an element
    void pop_back() { itsV.pop_back(); };
    
    // reference to element at back
    T& back() { itsV.back(); };
    const T& back() const  { itsV.back(); };
    
    // reference to element at front
    T& front() { itsV.front(); };
    const T& front() const { itsV.front(); };
    
    // get the size
    const size_t size() const {return itsV.size(); };

    // clear contents
    void clear() { itsV.clear(); };
    
    // get iterator to element at front
    iterator begin() { return itsV.begin(); };
    const_iterator begin() const { return itsV.begin(); };
    
    //get iterator to element at back
    iterator end() { return itsV.end(); };
    const_iterator end() const { return itsV.end(); };

    //return true if the element at pos is null
    const bool isNull(const uint pos) const {return false; };
  };

  // ######################################################################
  // specialization for pointer type
  // ######################################################################
  template<class T>
  class vector<T*> //in this case, T will be the pointee type
  {
    template<class U> class cp_iterator; //forward declaration
    
    //define iterator
    template <class U> //U will be the pointee type
    class p_iterator : public std::iterator<std::random_access_iterator_tag, U>
    {
      friend class cp_iterator<U>;
      typedef unsigned long diff_t;   
      typedef typename std::vector<U*>::iterator v_iterator;
      v_iterator iter;
      
    public:
      //constructors 
      p_iterator() : iter() { };
      p_iterator(const p_iterator& rhs) : iter(rhs.iter) { };
      p_iterator(const v_iterator& rhs) : iter(rhs) { };

      //default assignment OK
      
      //logical
      bool operator==(const p_iterator& rhs) const { return iter == rhs.iter; };
      bool operator!=(const p_iterator& rhs) const { return iter != rhs.iter; };
      bool operator<(const p_iterator& rhs) const { return iter < rhs.iter; };
      bool operator<=(const p_iterator& rhs) const { return iter <= rhs.iter; };
      bool operator>(const p_iterator& rhs) const { return iter > rhs.iter; };
      bool operator>=(const p_iterator& rhs) const { return iter >= rhs.iter; };
      
      //increment and decrement
      p_iterator& operator++() {
        ++iter;
        return *this;
      };
      p_iterator operator++(int) {
        p_iterator tmp = *this;
        ++iter;
        return tmp;
      };
      void operator+=(diff_t d) { 
        iter += d; };
      p_iterator& operator--() {
        --iter;
        return *this;
      };
      p_iterator operator--(int) {
        p_iterator tmp = *this;
        --iter; 
        return tmp;
      };
      void operator-=(diff_t d) { 
        iter -= d; };
      
      //access
      U& operator*() { return **iter; };
      U*& pointer() { return *iter; };
      U* operator->() { return *iter; }; //evaluated as *( *iter ).
    };

    //define const iterator
    template <class U> //U will be the pointee type
    class cp_iterator : public std::iterator<std::random_access_iterator_tag, U>
    {
      typedef unsigned long diff_t;   
      typedef typename std::vector<U*>::const_iterator v_iterator;
      v_iterator iter;
      
    public:
      //constructors 
      cp_iterator() : iter() { };
      cp_iterator(const cp_iterator& rhs) : iter(rhs.iter) { };
      cp_iterator(const p_iterator<U>& rhs) : iter(rhs.iter) { };
      cp_iterator(const v_iterator& rhs) : iter(rhs) { };

      //default assignment OK
      
      //logical
      bool operator==(const cp_iterator& rhs) const {return iter == rhs.iter; };
      bool operator!=(const cp_iterator& rhs) const {return iter != rhs.iter; };
      bool operator<(const cp_iterator& rhs) const {return iter < rhs.iter; };
      bool operator<=(const cp_iterator& rhs) const {return iter <= rhs.iter; };
      bool operator>(const cp_iterator& rhs) const {return iter > rhs.iter; };
      bool operator>=(const cp_iterator& rhs) const {return iter >= rhs.iter; };
      
      //increment and decrement
      cp_iterator& operator++() {
        ++iter;
        return *this;
      };
      cp_iterator operator++(int) {
        cp_iterator tmp = *this;
        ++iter;
        return tmp;
      };
      void operator+=(diff_t d) { 
        iter += d; };
      cp_iterator& operator--() {
        --iter;
        return *this;
      };
      cp_iterator operator--(int) {
        cp_iterator tmp = *this;
        --iter; 
        return tmp;
      };
      void operator-=(diff_t d) { 
        iter -= d; };
      
      //access
      const U& operator*() const { return **iter; };
      const U* const pointer() const { return *iter; };
      const U* operator->() const { return *iter; }; //evaluated as *( *iter ).
    };

    //private members of vector<T*>
    //our storage container
    std::vector<T*> itsV;
    
    //for our storage container iterator
    typedef typename std::vector<T*>::const_iterator v_iterator;
    
  public:
    typedef p_iterator<T> iterator;
    typedef cp_iterator<T> const_iterator;
    typedef T Type;
    typedef T* PointerType;
    enum {isPointer = 1 };
    
    // constructor
    vector() : itsV() { };

    // constructor
    vector(size_t size, const T& value) : itsV()
    { 
      for (uint ii = 0; ii < (uint)size; ++ii)
        itsV.push_back(value.clone());
    };

    //copy constructor 
    vector(const vector& rhs) 
    { 
      clear();
      v_iterator i(rhs.itsV.begin()), end(rhs.itsV.end());
      while (i != end)
        itsV.push_back( (*i++)->clone() );
    };

    //assignment operator
    vector& operator=(const vector& rhs)
    {
      if (this != &rhs)
        {
          clear();
          v_iterator i(rhs.itsV.begin()), end(rhs.itsV.end());
          while (i != end)
            itsV.push_back( (*i++)->clone() );
        }
      return *this;
    };
    
    //destructor
    ~vector()   
    {     
      clear();
    };
    
    // return a reference
    T& operator[](const int pos) { return *itsV[pos]; };//not safe for assignment, use set_at
    const T& operator[](const int pos) const { return *itsV[pos]; };

    // set an element at the specified position
    void set_at(const uint pos, const T& element) { delete itsV[pos]; itsV[pos] = element.clone(); };
       
    // push an element on, when the element is a reference
    void push_back(const T& element) { itsV.push_back(element.clone()); };

    // pop an element
    void pop_back() { delete itsV.back(); itsV.pop_back(); };
    
    // reference to element at back
    T& back() { *itsV.back(); };
    const T& back() const  { *itsV.back(); };
    
    // reference to element at front
    T& front() { *itsV.front(); };
    const T& front() const { *itsV.front(); };
    
    // get the size
    const size_t size() const {return itsV.size(); };

    // clear contents
    void clear() 
    {     
      v_iterator begin(itsV.begin()), end(itsV.end());
      while (begin != end)
        delete *begin++; 
      itsV.clear();
    };
    
    // get iterator to element at front
    iterator begin() { return iterator(itsV.begin()); };
    const_iterator begin() const { return const_iterator(itsV.begin()); };
    
    //get iterator to element at back
    iterator end() { return iterator(itsV.end()); };
    const_iterator end() const { return const_iterator(itsV.end()); };

    //check if the element at pos is null
    const bool isNull(const uint pos) const { return (itsV[pos] == NULL) ? true : false; };
  };

}//end namespace
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
