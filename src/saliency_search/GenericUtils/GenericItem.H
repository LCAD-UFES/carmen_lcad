// Primary maintainer for this file: David J. Berg

#ifdef  INVT_USE_CPP11//we need c++ 0X features for this to work
#ifndef GENERICITEM_H_DEFINED
#define GENERICITEM_H_DEFINED

#include "GenericUtils/ProcessFunctor.H"
#include <memory>

// ######################################################################
// GenericItem allows one to abtract the type away of a container and its
// contained type, so that one can have a container of containers
// of heterogenious types. 
//
// Optionally (and what makes this class useful) the template can take a number
// of additionaly types, which are the types of a ProcessFunctor which can
// operate on the generic data in the item. The ProcessFunctor is designed to
// handle the different types of data in the GenericItem. See ProcessFunctor.H
// for details, and the example in test-generics.C
// ######################################################################
template <template <typename...> class C, typename... FunctorTypes>
class GenericItemBase
{
public:
  // allow anyone to destroy this class
  virtual ~GenericItemBase() { };
  
  // default destructor, copy, and assign Ok
  
  //process an item with the functor
  virtual void runProcessFunctor(ProcessFunctor<C, FunctorTypes...>& func, const bool convert = false) = 0;
  
protected:
  //we don't want just anybody to create, or copy our generic feature
  GenericItemBase() { };
  GenericItemBase(const GenericItemBase& rhs) { };
  void operator=(const GenericItemBase& rhs) { };
};

// ######################################################################
// Lets implement a GenericItem with a template type
// ######################################################################
template <template <typename...> class C, typename T, typename... FunctorTypes>
class GenericItemImpl : public GenericItemBase<C, FunctorTypes...>
{
public:
  //construct with data, no default constructor. 
  GenericItemImpl(const C<T>& item) : itsItem(item) { };
  
  // default destructor, copy, and assign Ok
  
  //process an item with the functor
  void runProcessFunctor(ProcessFunctor<C, FunctorTypes...>& func, const bool convert = false)
  { func(itsItem, convert); }
  
private:
  C<T> itsItem;
};

// ######################################################################
// A class to represent a GenericItem
// ######################################################################
template <template <typename...> class C, typename... FunctorTypes>
class GenericItem
{
public:
  //create a feature
  template <class T>
  static GenericItem make(const C<T>& item) 
  { 
    GenericItem fs;
    fs.itsGenItem.reset(new GenericItemImpl<C, T, FunctorTypes...>(item));
    return fs;
  }
  
  //default copy constructor, destructor and assignment Ok  
  
  //process an item with the functor
  void runProcessFunctor(ProcessFunctor<C, FunctorTypes...>& func, const bool convert = false)
  {
    itsGenItem->runProcessFunctor(func, convert);
  };

private:
  //dont allow construction directly, use static create functions
  GenericItem() : itsGenItem() { };
  
  std::shared_ptr<GenericItemBase<C, FunctorTypes...> > itsGenItem;//a pointer to our GenericItem 
};

#endif
#endif
