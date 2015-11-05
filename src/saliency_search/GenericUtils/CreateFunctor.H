/*! A CreateFunctor is a Functor which creates an object with optional
    parameters. The parameter manipulation and object creation are
    dissacociated, so that at the point of object creation parameter
    type and value need not be known. 
*/
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
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/GenericUtils/CreateFunctor.H$

#ifndef UTIL_CREATEFUNCTOR_H_DEFINED
#define UTIL_CREATEFUNCTOR_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "GenericUtils/ParamList.H"
#include "GenericUtils/ObjCreator.H"
#include "Util/log.H"

#include <typeinfo>

//######################################################################
//CreateListHolder
//
// This is an abstract class to represent interface to various
// CreateListHolderImpl's which will hold ParamList's for object
// creation. See CreatListHolderImpl for the implementation and 
// CreateFunctor which uses a CreateListHolder. MutPlist must be 
// a ParamList<typename...> Users should not use this class directly. 
//######################################################################
template <class ReturnType, class MutPlist>
class CreateListHolder
{
public:  
  //destructor
  virtual ~CreateListHolder() { };
  
  //create a ReturnType
  virtual ReturnType* create() const = 0;
  
  //polymorphicly clone the object
  virtual CreateListHolder* clone() const 
  {
    CreateListHolder* d = doClone();
    if ( typeid(*d) != typeid(*this) )
      LFATAL("DoClone incorrectly overridden" );
    return d;
  };

  //update the mutable paramters
  template <typename... Params>
  void setMutParams(const Params&... params)
  {
    itsMutParams = ParamList<Params...>(params...);
  }
  
  //update the mutable params in the paramlist 
  virtual bool update() = 0;

protected:
  //constructors and assignment
  CreateListHolder() : itsMutParams() { };
  CreateListHolder(const MutPlist& mutplist) : itsMutParams(mutplist) { };
  CreateListHolder(const CreateListHolder& rhs) : itsMutParams(rhs.itsMutParams) { };
  CreateListHolder& operator=(const CreateListHolder& rhs) 
  { if (this != &rhs) itsMutParams = rhs.itsMutParams; return *this; };

  MutPlist itsMutParams;//our mutable parameter list
  
private:
  virtual CreateListHolder* doClone() const = 0;
};

//######################################################################
// CreateListHolderImpl 
// 
// The polymorphic implementation of CreateListHolder. This class
// stores parameter information used in CreateFunctor. Different
// template instantiations will create different derived types all
// accesable through the CreateListHolder interface. MutPlist must be 
// a ParamList<typename...> Users should not use this class directly.
// ######################################################################
template <class ReturnType, class CreateType, class MutPlist, typename... Params> class CreateListHolderImpl;

//specialization for when there are no parameters
//###################################################################### 
template <class ReturnType, class CreateType, class MutPlist>
class CreateListHolderImpl<ReturnType,CreateType,MutPlist> : public CreateListHolder<ReturnType, MutPlist>
{
public:
  //constructor
  CreateListHolderImpl() : CreateListHolder<ReturnType, MutPlist>() { };
  
  //default copy constructor and assignment OK
  
  //desctructor
  virtual ~CreateListHolderImpl() { };
  
  //create an object of type ReturnType
  ReturnType* create() const 
  { return new CreateType(); }

  //update the mutable params in the paramlist, return true if an update could be made
  virtual bool update()
  {
    return false;
  }

private:
  //actually do the work of clone()
  CreateListHolderImpl<ReturnType, CreateType, MutPlist>* doClone() const
  {
    return new CreateListHolderImpl<ReturnType, CreateType, MutPlist>(*this);
  };
};

//specialization for when there are N parameters
//###################################################################### 
template <class ReturnType, class CreateType, class MutPlist, typename... Params>
class CreateListHolderImpl : public CreateListHolder<ReturnType, MutPlist>
{
public:
  //constructor
  CreateListHolderImpl(const ParamList<Params...>& paramlist) 
    : CreateListHolder<ReturnType, MutPlist>(), itsList(paramlist) { };
  
  //constructor
  CreateListHolderImpl(const Params&... params)
    : CreateListHolder<ReturnType, MutPlist>(), itsList(params...) { };
  
  //default copy constructor and assignment OK
  
  //desctructor
  virtual ~CreateListHolderImpl() { };
  
  //create an object of type ReturnType
  ReturnType* create() const 
  { return ObjCreator<CreateType>::create(itsList); }

  //update the mutable params in the paramlist, return true if an update could be made
  virtual bool update()
  {
    return itsList.assign(CreateListHolder<ReturnType, MutPlist>::itsMutParams);
  }

private:
  //actually do the work of clone()
  CreateListHolderImpl<ReturnType, CreateType, MutPlist, Params...>* doClone() const
  {
    return new CreateListHolderImpl<ReturnType, CreateType, MutPlist, Params...>(*this);
  };
  
  ParamList<Params...> itsList;//hold our ParamList
};

//###################################################################### 
// CreateFunctor is a functor that creates an object with optional parameters. The parameter manipulation and object
// creation are dissacociated, so that at the point of object creation parameter type and value need not be known.
// Parameters can be set at functor creation and modified later if desired. A call to operator() will return the desired
// type. The is particularly useful with GenericFactory.H 'ReturnType' is the type of object to return (could be an
// abstract class). The second template must be a ParamList whose types are the types of the mutable parameters, which
// could be a subset of the parameters used to create the object. The result is some fixed and some mutable parameters.
// See UpdateParams() for more details.
//
// Usage: 
//
//Imagine we have a class Circle which can take one or two arguments and
//derives from Shape. operator() prints "I'm a Cicle". Circle has two
//public members p1, and p2 to hold the arguments.

//  //test functor components
//  ParamList<float,int> pl(1.00f, 1);
//  CreateFunctor<Circle> func(pl);
//  Circle* c = func();
//  LINFO("%3.2f, %d", c->p1, c->p2);

//  //create Functor from parameters
//  CreateFunctor<Circle> func1(2.00f, 2);
//  CreateFunctor<Circle> func2(10.10f);
//  c = func1();
//  LINFO("%3.2f, %d", c->p1, c->p2);
//  c = func2();
//  LINFO("%3.2f, %d", c->p1, c->p2);

//  //now, create a derived type of the return type
//  auto func3 = CreateFunctor<Shape>::make<Circle>(pl);
//  Shape* s = func3();
//  (*s)(); //print the message
//  c = dynamic_cast<Circle*>(s);//cast to a circle
//  LINFO("%3.2f, %d", c->p1, c->p2);

//  //make from some params instead
//  auto func4 = CreateFunctor<Shape>::make<Circle>(33.33f, 55);
//  s = func4();
//  (*s)(); //print the message
//  c = dynamic_cast<Circle*>(s);
//  LINFO("%3.2f, %d", c->p1, c->p2);
//
//  //update both parameter. see updateParams() for more details
//  CreateFunctor<Circle,ParamList<float,int> > func5(2.00f, 2);
//  func5.updateParams(0.0f,0);
//  c = func5();
//  LINFO("%3.2f, %d", c->p1, c->p2);
//  
//  //only the second paramter is able to be changed
//  CreateFunctor<Circle,ParamList<int> > func6(2);
//  func6.updateParams(0);
//  c = func6();
//  LINFO("%3.2f, %d", c->p1, c->p2);
// ######################################################################
template <class ReturnType, class MutPlist = ParamList<>>
class CreateFunctor
{
public:
  //empty creation function- particularly when we want to create a derived type and return a base class pointer.
  template<class CreateType>
  static CreateFunctor<ReturnType, MutPlist> make()
  {
    CreateListHolderImpl<ReturnType, CreateType, MutPlist> temp;
    return CreateFunctor<ReturnType, MutPlist>(temp);
  };

  //static creation function which takes a paramlist
  template<class CreateType, typename... Params>
  static CreateFunctor<ReturnType, MutPlist> make(const ParamList<Params...>& paramlist)
  {
    CreateListHolderImpl<ReturnType, CreateType, MutPlist, Params...> temp(paramlist);
    return CreateFunctor<ReturnType, MutPlist>(temp);
  };
  
  //static creation function which takes params
  template<class CreateType, typename... Params>
  static CreateFunctor<ReturnType, MutPlist> make(const Params&... params)
  {
    CreateListHolderImpl<ReturnType, CreateType, MutPlist, Params...> temp(params...);
    return CreateFunctor<ReturnType, MutPlist>(temp);
  };

  //constructor
  CreateFunctor() 
    : itsPLH(new CreateListHolderImpl<ReturnType, ReturnType, MutPlist>()) 
  { };
  
  //construct from a param list 
  template<typename... Params>
  CreateFunctor(const ParamList<Params...>& paramlist) 
    : itsPLH(new CreateListHolderImpl<ReturnType, ReturnType, MutPlist, Params...>(paramlist)) 
  { };
  
  //construct from parameters
  template<typename... Params>
  CreateFunctor(const Params&... params) 
    : itsPLH(new CreateListHolderImpl<ReturnType, ReturnType, MutPlist, Params...>(params...)) 
  { };
  
  //copy constructor
  CreateFunctor(const CreateFunctor<ReturnType, MutPlist>& rhs) 
    : itsPLH( (rhs.itsPLH) ? rhs.itsPLH->clone() : NULL ) { };
  
  //destructor
  ~CreateFunctor() {delete itsPLH; };
  
  //assignment operator
  CreateFunctor<ReturnType, MutPlist>& operator=(const CreateFunctor<ReturnType, MutPlist>& rhs)
  {
    if (this != &rhs)
      {
	if (itsPLH) delete itsPLH;
	itsPLH = (rhs.itsPLH) ? rhs.itsPLH->clone() : NULL;
      }
  };
  
  //function call operator
  ReturnType* operator()() const { return (itsPLH) ? itsPLH->create() : NULL; };
  
  //update our mutable parameters and return true if an update could be made. This relies on the 'assign' function of
  //the ParamList used as the second template type. So, only a subset of paramters can be assigned (ie Params does not
  //have to be all the possible parameters the object being created could take). See ParamList.H.
  template <typename... Params>
  bool updateParams(const Params&... params)
  {
    itsPLH->setMutParams(params...);
    return itsPLH->update();
  }

private:
  //construct from a CreateListHolderImpl
  template<class CreateType, typename... Params>
  CreateFunctor(const CreateListHolderImpl<ReturnType, CreateType, MutPlist, Params...>& clhi) : itsPLH(clhi.clone()) 
  { };
    
  //pointer to a CreateListHolderImpl
  CreateListHolder<ReturnType, MutPlist>* itsPLH;
};

#endif //INVT_USE_CPP11
#endif//UTIL_CREATEFUNCTOR_H_DEFINED
