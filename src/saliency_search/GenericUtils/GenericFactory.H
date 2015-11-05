/*!@file GenericUtils/GenericFactory.H A generic Factory*/

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
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/GenericUtils/GenericFactory.H$

#include "Util/log.H"

#include <pthread.h>
#include <map>
#include <string>

#ifndef UTIL_GENERICFACTORY_H_DEFINED
#define UTIL_GENERICFACTORY_H_DEFINED

// ######################################################################
// A default policy for error handling in a GenericFactory (see
// below). Type specific policies should be added in the type specific
// files. An ErrorPolicy type should implement the following
// functions:
//
// ReturnType* createError(); // called if 'GenericFactory::create()'
//                            // fails to find the key 
//
// void registerError(); // called if nothing is added when 'registerCreator()' 
//                       // is called
// ######################################################################
// In the default error policy, errors during object will cause LFATAL
// ######################################################################
template<class ReturnType, typename KeyType>
struct DefaultFactoryError
{
  //constructor/destructor
  DefaultFactoryError() { };
  ~DefaultFactoryError() { };
  
    //possibly do something if registerCreator fails to add an entry
  void registerError() const
  { };
  
  //return an error if create() fails to find the 'key'
  ReturnType* createError(const KeyType& key) const
  { 
    LFATAL("The Factory produced an error and no object was returned "
           "because the key could not be found.");
    return NULL; 
  };
};

// ######################################################################
// In the strict error policy, any error will cause LFATAL
// ######################################################################
template<class ReturnType, typename KeyType>
struct StrictFactoryError
{
  //constructor/destructor
  StrictFactoryError() { };
  ~StrictFactoryError() { };
  
  //possibly do something if registerCreator fails to add an entry
  void registerError() const
    { 
      LFATAL("The key/creator pair could not be registered. "
             "It is likely this key alread exists");
    };
  
  //return an error if create() fails to find the 'key'
  ReturnType* createError(const KeyType& key) const
  { 
    LFATAL("The Factory produced an error and no object was returned "
           "because the key could not be found.");
    return NULL;
  };
};

// ######################################################################
// define a do nothing error policy for factories
// ######################################################################
template<class ReturnType, typename KeyType>
struct DoNothingFactoryError
{
  //constructor/destructor
  DoNothingFactoryError() { };
  ~DoNothingFactoryError() { };
  
    //possibly do something if registerCreator fails to add an entry
  void registerError() const
  { };
  
  //return an error if create() fails to find the 'key'
  ReturnType* createError(const KeyType& key) const
  { return NULL; };
};

// ######################################################################
// A reusable factory. This class is intended to simplify the creation
// of objects from simple 'keys' (like a std::string representing the
// name of the object). The objects stored in the factory should all
// derive from a common base class. The user should call
// registerCreator() to make an association between a key and a
// pointer to a function (or a functor - especially useful for
// including parameters, see FactoryHelper namespace below) that can
// actually create the object. The pointer to function (or functor)
// should allocate memory, and configure the desired object returning
// a pointer to the objects base class. A call to
// ReusableFactory::create() will then create the appropriate object
// and return a pointer to the base class.
//
// Additionally, the createConvert<>() template function is provided
// which will attempt to create the object, and then covert it to the
// desired type before returning. The template argument can be a
// pointer or non-pointer type. If the template argument is a pointer,
// a call to createConvert<>() will call dynamic_cast and return a
// pointer to the desired type (or fail an error). If the template
// argument is a non-pointer then the object will be dynamic_cast,
// dereferenced, copied with the assignment operator, and finally the
// copy will be returned.
//
// ######################################################################
// templated types are as follows:
// ###################################################################### 
// ReturnType : the type to return from the factory (usually a base
// class), create will return a ReturnType*.
//
// KeyType : the type to be used as the unique key for each objecy
// type (any type that supports operator<() )
//
// TypeCreator : the type of the object creator mechanism. This could
// be anything that supports 'ReturnType* operator()()' (usually a
// pointer to a function, but could also be a non-pointer functor).
//
// ErrorType : the class that will handle errors in the factory. Such
// a class should implment the following functions:
//
//     ReturnType* createError(); // called if 'GenericFactory::create()'
//                                // fails to find the key 
//
//     void registerError();      // called if nothing is added when 
//                                // 'registerCreator()' is called
//
// ######################################################################
// Some example usages: Say we have a Shape base class, with derived
// classes Circle and Square, each having there own creation function. 
// ######################################################################
//  //Register a string key with pointers to functions
//  factory.add("circle", &createCircle);
//  factory.add("square", &createSquare);

//  create using factory helper function, create, to auto insert a
//  creation function
//  factory.add("rectangle", &FactoryHelper::create<Shape,Rectangle>());
//
//  //create a circle and return a pointer to the base class
//  Shape* shape = factory.create("circle");
//
//  //create a circle and return a pointer to a circle
//  Circle* shape1 = factory.createConvert<Circle*>("circle");
//
//  //create a square and return it
//  Square shape2 = factory.createConvert<Square>("square");
//
//  //will error : no registered key
//  Shape* shape = factory.create("triangle");
//
//  //will error : cannot convert Cirlce* to Square*
//  Square* shape1 = factory.createConvert<Square*>("circle");
//
// ######################################################################
// Note: This implementation uses std::map, which is not ideal for many
// situations - particularly where you need fast lookups (for example
// creating objects in the main loop of your program). Another
// implementation could have an additional template parameter for the
// internal container to use.
//######################################################################
template<class ReturnType, 
         typename KeyType = std::string,
         typename TypeCreator = ReturnType* (*)(),
         template<class, typename> class ErrorPolicy = DefaultFactoryError
         > 
class ReusableFactory : public ErrorPolicy<ReturnType, KeyType>
{
public:
  ReusableFactory() : ErrorPolicy<ReturnType, KeyType>(), itsMap() { };
  ~ReusableFactory() { };

  //a read/write iterator 
  typedef typename std::map<KeyType, TypeCreator>::iterator iterator;
  
  // ######################################################################
  // register an key/object creation function pair and return 'true'
  // if something was added. If nothing was added, createError() is
  // called and false is returned
  // ######################################################################
  const bool add(const KeyType& key, TypeCreator func)
  {
    const bool added = (itsMap.insert(make_pair(key, func))).second;
    if (added == false)
      this->registerError();
    
    return added;
  }
  
  // ######################################################################
  // unregister an object function and return whether we removed anything
  // ######################################################################
  const bool remove(const KeyType& key)
  {
    return (itsMap.erase(key) > 0);
  }
  
  // ######################################################################
  // create the appropriate object from a registered key and return a
  // pointer to the base class (ReturnType)
  // ######################################################################
  ReturnType* create(const KeyType& key) const
  {
    typename Map::const_iterator iter = itsMap.find(key);
    if (iter != itsMap.end())
      return (iter->second)();
    
    return this->createError(key);
  }

  // ###################################################################### 
  // get a read/write iterator to the first key/creator pair. Returns 
  // a std::pair where 'first' is the key and 'Second' is the value
  // ######################################################################
  iterator begin() 
  {
    return itsMap.begin();
  };

  // ###################################################################### 
  // get a read/write iterator to 1 past the last key/creator pair.
  // ######################################################################
  iterator end() 
  {
    return itsMap.end();
  };

  // ######################################################################
  // create the appropriate object from a registered key as with create(),
  // but before returning attempt to convert to the templated type.
  // ######################################################################
  template<class ConvertType> 
  ConvertType createConvert(const KeyType& key)
  {
    return convertHelper<ConvertType>(key, Int2Type<PntrTraits<ConvertType>::isPointer>());
  } 

private:
  //prevent copy and assignment
  ReusableFactory(const ReusableFactory& rhs);
  ReusableFactory& operator=(const ReusableFactory& rhs);

  // ######################################################################
  // typedefs and helper classes
  // ######################################################################
  //useful typedefs
  typedef typename std::map<KeyType, TypeCreator> Map;
  
  //a helper class to determine if ConvertType is a pointer
  template <typename T> struct PntrTraits{ enum {isPointer = false}; };
  template <typename T> struct PntrTraits<T*>{ enum {isPointer = true}; };

  //an empty helper class to 'typeify' an integer
  template <int input> class Int2Type { };
  
  // ######################################################################
  // for when ConvertType in createConvert() is a pointer
  // ######################################################################
  template<class ConvertType>
  ConvertType convertHelper(const KeyType& key, Int2Type<true>)
  {
    ReturnType* basep = this->create(key);
    
    //dynamic cast from our base type * to the convert type *
    //(ConverType will be a pointer)
    ConvertType convp = dynamic_cast<ConvertType>(basep);

    if (convp != NULL)
      return convp;

    LFATAL("Failed to covert to the desired type.");
    return NULL;
  }

  // ######################################################################
  //for when ConvertType in createConvert() is a non-pointer
  // ######################################################################
  template<class ConvertType> 
  ConvertType convertHelper(const KeyType& key, Int2Type<false>)
  {
    ReturnType* basep = this->create(key);
    
    //dynamic cast from our base type * to the convert type *
    //(ConvertType will be non-pointer)
    ConvertType* convp = dynamic_cast<ConvertType*>(basep);

    ConvertType temp = *convp; //copy the object
    delete convp;//clean up the temporary memory
    return temp;
  }
  
  //the map to hold registered key-function pairs
  Map itsMap;
};

// ######################################################################
// GenericFactory is a thread-safe singleton class that extends
// ReusableFactory. A singleton is a design pattern that supports
// global access to a unique object (only one can exist in your
// program). The main advantage of this pattern for a factory is that
// is supplies a global access point to a static structure, which
// means we can register our classes in there own .C or .H
// file. Besides being extremely convenient, it eliminates the
// dependecy bottleneck of a non-static factory, and no additional
// source files need be edited when adding a new class to the
// registry.
//
// Building on the example in ReusableFactory (above):
//
//  //Register a string key with pointers to different shape creation
//  //functions. This could in, for instance, 'Shapes.C'
//  namespace 
//  {
//    typedef GenericFactory<Shape> ShapeFactory;
//    struct RegisterShapes
//    {
//      RegisterShapes() 
//      { 
//        Using namespace FactoryHelper;
//        ShapeFactory::instance().add("Circle", &createCircle); 
//        ShapeFactory::instance().add("Square", &createSquare);
//        ShapeFactory::instance().add("Rectangle", &create<Shape,Rectangle>());
//      }
//    };
//    static RegisterShapes Register;
//  }

//  //then in our 'main' we (or someone else) can use them without
//  //ever thinking about registering them.
//  ShapeFactory::instance().add("circle", &createCircle);
//  ShapeFactory::instance().add("square", &createSquare);
//
//  //create a circle and return a pointer to the base class
//  Shape* shape = ShapeFactory::instance().create("circle");
//
//######################################################################
//Programmer Note: 
//We never 'delete' the object we create in Instance(), but this is
//not a memory leak. The object is intended to stay alive for the
//duration of the program, we never lose reference to it, no new
//memory is being created and the OS will deallocate the memory upon
//program termination.
//
//However, it would be dangerous to allow another classes destructor
//to depend on GenericFactory operations, as the order of destruction
//of static elements is not gauranteed, and the Singleton may not
//exist. 
//
//The double check and lock paradigm is used for thread safety. This
//should work on most or all systems but is not garaunteed.
//######################################################################
template<class ReturnType, 
         typename KeyType = std::string,
         typename TypeCreator = ReturnType* (*)(),
         template<class, typename> class ErrorPolicy = DefaultFactoryError
         > 
class GenericFactory
{
private:
  //a helper class to deal with the mutex
  class Lock
  {
    pthread_mutex_t tmut; //a mutex to lock creation
  public:
    Lock() { pthread_mutex_init(&tmut, NULL); };
    ~Lock() { pthread_mutex_destroy(&tmut); };
    bool lock() {return (0 == pthread_mutex_lock(&tmut)) ? 1 : 0; };
    bool unlock() {return (0 == pthread_mutex_unlock(&tmut)) ? 1 : 0; };
  };
  
  GenericFactory(); 
  ~GenericFactory(); 
  GenericFactory(const GenericFactory&); 
  GenericFactory& operator=(const GenericFactory&); 
  
  typedef ReusableFactory<ReturnType, KeyType, TypeCreator, ErrorPolicy> FactoryType;
  typedef FactoryType* InstanceType;

  static InstanceType itsInstance; // The one and only instance
  static volatile bool isCreated; // see if we are created 
  static Lock itsLocker; //a mutex to lock our instance

public:
  static FactoryType& instance() // Unique point of access
  {
    //double checked lock paradim
    if (!isCreated)
      { 
        if (!itsLocker.lock())
          LFATAL("Mutex locking error");
          
        if (!isCreated)
          {
            itsInstance = new FactoryType();
            isCreated = true;
          }

        if (!itsLocker.unlock())
          LFATAL("Mutex unlocking error");

      }
    return *itsInstance;
  }
};

//######################################################################
// FactoryHelper is a set of utilities to be used with GenericFactory
//######################################################################
namespace FactoryHelper
{
  //helper function to make a creation functions
  //######################################################################
  template <class BaseType, class DerivedType> 
  BaseType* create() {return new DerivedType; };
}

//######################################################################
// static variable initialization
//######################################################################
//initialize our itsInstance
template<class ReturnType, 
         typename KeyType,
         typename TypeCreator,
         template<class, typename> class ErrorPolicy> 
typename GenericFactory<ReturnType,KeyType,TypeCreator,ErrorPolicy>::InstanceType GenericFactory<ReturnType,KeyType,TypeCreator,ErrorPolicy>::itsInstance = NULL;

//initialize isCreated
template<class ReturnType, 
         typename KeyType,
         typename TypeCreator,
         template<class, typename> class ErrorPolicy>
volatile bool GenericFactory<ReturnType,KeyType,TypeCreator,ErrorPolicy>::isCreated = false;

//initialize the locker
template<class ReturnType, 
         typename KeyType,
         typename TypeCreator,
         template<class, typename> class ErrorPolicy>
typename GenericFactory<ReturnType,KeyType,TypeCreator,ErrorPolicy>::Lock GenericFactory<ReturnType,KeyType,TypeCreator,ErrorPolicy>::itsLocker = typename GenericFactory<ReturnType,KeyType,TypeCreator,ErrorPolicy>::Lock();

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
