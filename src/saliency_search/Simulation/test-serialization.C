/*!@file Simulation/test-serialization.C simple test code for serialization of SimEvent */


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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEvent.H $
// $Id: SimEvent.H 9031 2007-11-23 10:54:03Z siagian $
//

#include "Util/log.H"

#ifndef HAVE_BOOST_SERIALIZATION

int main(int argc, const char **argv)
{
  LFATAL("You need Boost::Serialization installed on your system for this program to work");
  return 1;
}

#else

// include headers that implement an archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp> 
#include <boost/serialization/config.hpp>
#include <boost/serialization/vector.hpp>

#include <fstream>

typedef unsigned char byte;

// this code derived from http://www.boost.org/doc/libs/1_42_0/libs/serialization/doc/index.html

// ######################################################################
// ######################################################################
// Straight-up code for a serializable class and a derived class:
// ######################################################################
// ######################################################################

// ######################################################################
// SimModule is used in the constructor of SimEvent, here we just fake it
class SimModule {
};

// ######################################################################
// Base class, contains no data
// this will be provided in the systemwide definition of SimEvent
class SimEvent {
public:
  SimEvent(SimModule* src = 0) { }

  virtual ~SimEvent() { }

  inline std::string toString() const { return std::string("define me as in old SimEvent def"); }

private:
  friend class boost::serialization::access;
  template<class Archive> inline void serialize(Archive& ar, const unsigned int version) { }
};

BOOST_CLASS_VERSION(SimEvent, 0); // bump this whenever you change the class' serializable data members

// ######################################################################
// Class derived directly from SimEvent, adds two data members, one int and one float
class MySimEvent : public SimEvent {
public:
  MySimEvent(SimModule* src = 0) { }

  MySimEvent(SimModule* src__, const int idata__, const float fdata__) :
    SimEvent(src__), idata_(idata__), fdata_(fdata__) { }

  virtual ~MySimEvent() { }

  int& idata() { return idata_; }
  float& fdata() { return fdata_; }

  const int& idata() const { return idata_; }
  const float& fdata() const { return fdata_; }

private:
  friend class boost::serialization::access;

  template<class Archive> void serialize(Archive& ar, const unsigned int version)
  {
    // serialize base class information:
    ar & boost::serialization::base_object<SimEvent>(*this);

    // and the new data from the derived class:
    ar & idata_;
    ar & fdata_;
  }

  int idata_; // some data member
  float fdata_; // some data member
};

BOOST_CLASS_VERSION(MySimEvent, 0); // bump this whenever you change the class' serializable data members


// ######################################################################
// Class further derived from the derived MySimEvent, adds yet more data members
class DerSimEvent : public MySimEvent {
public:
  DerSimEvent(SimModule* src = 0) { }

  DerSimEvent(SimModule* src__, const int idata__, const float fdata__,
              const std::string& sdata__, const std::vector<byte>& vdata__) :
    MySimEvent(src__, idata__, fdata__), sdata_(sdata__), vdata_(vdata__) { }

  virtual ~DerSimEvent() { }

  std::string& sdata() { return sdata_; }
  std::vector<byte>& vdata() { return vdata_; }

  const std::string& sdata() const { return sdata_; }
  const std::vector<byte>& vdata() const { return vdata_; }

private:
  friend class boost::serialization::access;

  template<class Archive> void serialize(Archive& ar, const unsigned int version)
  {
    // serialize base class information:
    ar & boost::serialization::base_object<MySimEvent>(*this);

    // and the new data from the derived class:
    ar & sdata_;
    ar & vdata_;
  }

  std::string sdata_; // note: initialization will deep copy, in reality we will use shared_ptr
  std::vector<byte> vdata_;
};

BOOST_CLASS_VERSION(DerSimEvent, 0); // bump this whenever you change the class' serializable data members

// ######################################################################
// ######################################################################
// Version with automatic creation of serialize()
// ######################################################################
// ######################################################################

// **********************************************************************
// ******************** INTERNALS:
// **********************************************************************

#include <boost/preprocessor/seq.hpp>
#include <boost/preprocessor/arithmetic/add.hpp>
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/punctuation/comma.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/punctuation/paren.hpp>
#include <boost/preprocessor/facilities/empty.hpp>

// try: g++ -include config.h -E src/Simulation/test-serialization.C to see the generated source

//! Root class for our SimEvent objects
#define INVT_PP_SIMEVENT_ROOT_CLASS SimEvent

//! Build our class variable's name from the named passed
#define INVT_PP_SIMEVENT_VAR(V) BOOST_PP_CAT(V, _)

//! Get elements of a tuple from our declaration list
#define INVT_PP_SIMEVENT_ELEM_TYP(v) BOOST_PP_TUPLE_ELEM(2, 0, v)
#define INVT_PP_SIMEVENT_ELEM_VAR(v) BOOST_PP_TUPLE_ELEM(2, 1, v)

//! Declare a variable (use with BOOST_PP_SEQ_FOR_EACH)
#define INVT_PP_SIMEVENT_DECLARE(r,x,val) INVT_PP_SIMEVENT_ELEM_TYP(val) \
  INVT_PP_SIMEVENT_VAR(INVT_PP_SIMEVENT_ELEM_VAR(val));                 \
  /**/

//! Archive (serialization) action for a variable (use with BOOST_PP_SEQ_FOR_EACH) 
#define INVT_PP_SIMEVENT_ARCHIVE(r,x,val) ar & INVT_PP_SIMEVENT_VAR(INVT_PP_SIMEVENT_ELEM_VAR(val));

//! Access function for a variable (use with BOOST_PP_SEQ_FOR_EACH)
#define INVT_PP_SIMEVENT_ACCESS(r,x,val)                                \
  inline INVT_PP_SIMEVENT_ELEM_TYP(val) & INVT_PP_SIMEVENT_ELEM_VAR(val)()     \
  { return INVT_PP_SIMEVENT_VAR(INVT_PP_SIMEVENT_ELEM_VAR(val)); }      \
  /**/

//! Access function for a variable, const version (use with BOOST_PP_SEQ_FOR_EACH)
#define INVT_PP_SIMEVENT_ACCESS_CONST(r,x,val)                          \
  inline const INVT_PP_SIMEVENT_ELEM_TYP(val)& INVT_PP_SIMEVENT_ELEM_VAR(val)() const \
  { return INVT_PP_SIMEVENT_VAR(INVT_PP_SIMEVENT_ELEM_VAR(val)); }      \
  /**/

//! Some empty implementation of a function
#define INVT_PP_SIMEVENT_EMPTY_IMPL { }

//! Default constructor
#define INVT_PP_SIMEVENT_DEFAULT_CONSTRUCTOR(CLASSNAME, BASECLASSNAME, CONSTR) \
  BOOST_PP_IF(BOOST_PP_EQUAL(CONSTR, 0), BOOST_PP_EMPTY(),              \
              CLASSNAME(SimModule* invt_pp_simevent_source = 0)         \
              BOOST_PP_IF(BOOST_PP_EQUAL(CONSTR, 1), ;,                 \
                          : BASECLASSNAME(invt_pp_simevent_source) INVT_PP_SIMEVENT_EMPTY_IMPL) ) \
/**/

//! Destructor
#define INVT_PP_SIMEVENT_DESTRUCTOR(CLASSNAME, DESTR)                   \
  BOOST_PP_IF(BOOST_PP_EQUAL(DESTR, 0), BOOST_PP_EMPTY(),               \
              BOOST_PP_IF(BOOST_PP_EQUAL(DESTR, 1), virtual ~CLASSNAME();, \
                          virtual inline ~CLASSNAME() INVT_PP_SIMEVENT_EMPTY_IMPL) ) \
/**/

//! Serialization friend, to allow the boost-serializer to access our private serialize() function
#define INVT_PP_SIMEVENT_SERIALIZATION_FRIEND friend class boost::serialization::access;

//! Serialize function declaration and definition
#define INVT_PP_SIMEVENT_SERIALIZE_FUNC(BASECLASSNAME, V)               \
  template<class Archive> inline void serialize(Archive& ar, const unsigned int version) \
  {									\
    ar & boost::serialization::base_object<BASECLASSNAME>(*this);       \
    BOOST_PP_SEQ_FOR_EACH(INVT_PP_SIMEVENT_ARCHIVE,,V)                  \
  }									\
/**/

//! toString function declaration
#define INVT_PP_SIMEVENT_TOSTRING_DECL(BASECLASSNAME, TOSTR) \
    BOOST_PP_IF(BOOST_PP_EQUAL(TOSTR, 0), BOOST_PP_EMPTY(),             \
                virtual std::string toString() const                    \
                BOOST_PP_IF(BOOST_PP_EQUAL(TOSTR, 1), ;, { return BASECLASSNAME::toString(); } ) )
/**/

//! General class declaration
#define INVT_PP_SIMEVENT_CLASSDECL(CLASSNAME, BASECLASSNAME, CLASSDECL, SERVERSION) \
  BOOST_PP_IF(BOOST_PP_EQUAL(CLASSDECL, 0), BOOST_PP_EMPTY(),           \
              class CLASSNAME;                                          \
              BOOST_CLASS_VERSION(CLASSNAME, SERVERSION);               \
              class CLASSNAME : public BASECLASSNAME { )                \
/**/

// **********************************************************************
// ******************** USER MACROS:
// **********************************************************************

//! Class declaration for a SimEvent derived class
/*! CLASSDECL = 0 (no class declaration, no opening brace), 1 (class decl + brace)
    CONSTR = 0 (no default constructor), 1 (declaration), 2 (decl + empty implementation)
    DESTR = 0 (no destructor), 1 (declaration), 2 (decl + empty implementation) 
    TOSTR = 0 (no toString() declaration), 1 (declaration), 2 (decl + empty implementation) */

#define SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, BASECLASSNAME, V, CLASSDECL, CONSTR, DESTR, TOSTR, SERVERSION) \
  INVT_PP_SIMEVENT_CLASSDECL(CLASSNAME, BASECLASSNAME, CLASSDECL, SERVERSION) \
                                                                        \
  public:                                                               \
    INVT_PP_SIMEVENT_DEFAULT_CONSTRUCTOR(CLASSNAME, BASECLASSNAME, CONSTR) \
    INVT_PP_SIMEVENT_DESTRUCTOR(CLASSNAME, DESTR)                       \
    BOOST_PP_SEQ_FOR_EACH(INVT_PP_SIMEVENT_ACCESS,, V)                  \
    BOOST_PP_SEQ_FOR_EACH(INVT_PP_SIMEVENT_ACCESS_CONST,, V)            \
    INVT_PP_SIMEVENT_TOSTRING_DECL(BASECLASSNAME, TOSTR)                \
                                                                        \
  private:								\
    BOOST_PP_SEQ_FOR_EACH(INVT_PP_SIMEVENT_DECLARE,, V)                 \
    INVT_PP_SIMEVENT_SERIALIZATION_FRIEND                               \
    INVT_PP_SIMEVENT_SERIALIZE_FUNC(BASECLASSNAME, V)                   \
  /**/

// ##### Declare the class and define a bunch of things in it. Nice but screws up auto-indent in emacs
#define SIMEVENT_CLASS_DECLARATION_TRIVIAL(CLASSNAME)                   \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, INVT_PP_SIMEVENT_ROOT_CLASS, , 1, 2, 2, 2, 0)

#define SIMEVENT_CLASS_DECLARATION_SIMPLE(CLASSNAME, V)                 \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, INVT_PP_SIMEVENT_ROOT_CLASS, V, 1, 2, 2, 1, 0)

#define SIMEVENT_CLASS_DECLARATION_BARE(CLASSNAME, BASECLASSNAME, V)    \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, BASECLASSNAME, V, 1, 0, 0, 0, 0)

#define SIMEVENT_CLASS_DECLARATION(CLASSNAME, BASECLASSNAME, V, CONSTR, DESTR, TOSTR, SERVERSION) \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, BASECLASSNAME, V, 1, CONSTR, DESTR, TOSTR, SERVERSION) \
  /**/

// ##### You declare the class manually (including opening brace), and here we just fill in the details
#define SIMEVENT_CLASS_DEFINITION_TRIVIAL(CLASSNAME)                   \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, INVT_PP_SIMEVENT_ROOT_CLASS, , 0, 2, 2, 2, 0)

#define SIMEVENT_CLASS_DEFINITION_SIMPLE(CLASSNAME, V)                 \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, INVT_PP_SIMEVENT_ROOT_CLASS, V, 0, 2, 2, 1, 0)

#define SIMEVENT_CLASS_DEFINITION_BARE(CLASSNAME, BASECLASSNAME, V)     \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, BASECLASSNAME, V, 0, 0, 0, 0, 0)

#define SIMEVENT_CLASS_DEFINITION(CLASSNAME, BASECLASSNAME, V, CONSTR, DESTR, TOSTR, SERVERSION) \
  SIMEVENT_CLASS_DECLARATION_INTERNAL(CLASSNAME, BASECLASSNAME, V, 0, CONSTR, DESTR, TOSTR, SERVERSION) \
  /**/


// **********************************************************************
// ******************** Examples:
// **********************************************************************

// ######################################################################
SIMEVENT_CLASS_DECLARATION_TRIVIAL(TrivSimEvent)
  // could declare alternate (parameterized) constructors here

  // could declare member functions here

  // you should not declare data members here, all your data members should be serializable!
};

// ######################################################################
SIMEVENT_CLASS_DECLARATION_SIMPLE( MySimEvent2,
                                   (( int, idata ))
                                   (( float, fdata )) )
public:
  // could declare alternate (parameterized) constructors here, for example:
  MySimEvent2(SimModule* src__, const int idata__, const float fdata__) :
    SimEvent(src__), idata_(idata__), fdata_(fdata__) { }


  // could declare member functions here

  // you should not declare data members here, all your data members should be serializable!

  // can access data members either directly as idata_ and fdata_ (which are private), or through the public member
  // functions idata() and fdata() which both exist in const and non-const versions
};

// we just need to implement toString() outside the class declaration:
std::string MySimEvent2::toString() const
{ return SimEvent::toString() + "MySimEvent2 Message"; }

// ######################################################################
class DerSimEvent2 : public MySimEvent2 {

  // bare definition, only our vars, serialize and access functions, no constr, destr, or toString()
  SIMEVENT_CLASS_DEFINITION_BARE( DerSimEvent2, MySimEvent2,
                                  (( std::string, sdata ))
                                  (( std::vector<byte>, vdata )) )

public:
  // could declare alternate (parameterized) constructors here, for example:
  DerSimEvent2(SimModule* src__, const int idata__, const float fdata__,
               const std::string& sdata__, const std::vector<byte>& vdata__) :
  MySimEvent2(src__, idata__, fdata__), sdata_(sdata__), vdata_(vdata__) { }

  // could declare member functions here

  // you should not declare data members here, all your data members should be serializable!

  // finally need to declare and implement constr, destr, and toString() function:

  // can both declare and implement here
  DerSimEvent2(SimModule* src = 0) : MySimEvent2(src) { /* do something cool */ }

  // can both declare and implement here
  virtual ~DerSimEvent2() { /* do something rad */ }

  // or can just declare and will implement later
  std::string toString() const;
};

// need to implement toString() outside the class declaration:
std::string DerSimEvent2::toString() const
{ return MySimEvent2::toString() + "DerSimEvent2 Message"; }

// ######################################################################
// ######################################################################
// Test program:
// ######################################################################
// ######################################################################

// choose MySimEvent or MySimEvent2, DerSimEvent or DerSimEvent2
#if 0
#  define MYSIMEVENT MySimEvent
#  define DERSIMEVENT DerSimEvent
#else
#  define MYSIMEVENT MySimEvent2
#  define DERSIMEVENT DerSimEvent2
#endif

int main(int argc, const char **argv)
{
  // create and open a character archive for output
  std::ofstream ofs("test-serialization.object");

  // create class instance
  SimModule *sm = 0;
  std::vector<byte> v; v.push_back(byte(3)); v.push_back(byte(42));
  std::string str("serialization inferno");
  const DERSIMEVENT s(sm, 5, 10.0F, str, v);

  LINFO("original data: %d %f '%s' [%d %d]", s.idata(), s.fdata(), s.sdata().c_str(), s.vdata()[0], s.vdata()[1]);

  // save data to archive
  {
    boost::archive::text_oarchive oa(ofs);
    // write class instance to archive
    oa << s;
    // archive and stream closed when destructors are called
  }

  // ... some time later restore the class instance to its orginal state
  DERSIMEVENT ss(sm);

  LINFO("new object before load: %d %f '%s' []", ss.idata(), ss.fdata(), ss.sdata().c_str());

  {
    // create and open an archive for input
    std::ifstream ifs("test-serialization.object");
    boost::archive::text_iarchive ia(ifs);
    // read class state from archive
    ia >> ss;
    // archive and stream closed when destructors are called
  }

  LINFO("re-loaded data: %d %f '%s' [%d %d]", s.idata(), s.fdata(), s.sdata().c_str(), s.vdata()[0], s.vdata()[1]);

  return 0;
}


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
