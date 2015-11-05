/**
   \file factory.hh

   \brief Defines a rudimentary framework for generic polymorphic
   factories that can churn out objects of any type.

   This file defines a couple of template classes to help clients create
   objects of any type using strings or other identifying tokens. Here is
   some illustrative code to show how this framework is meant to be used:

        #include "factory.hh"
        #include <iostream>
        #include <string>

        class base {
           std::string m_id ;
        protected:
           base(const std::string& id = "base") : m_id(id) {}
        public:
           const std::string& get() const {return m_id ;}
           virtual ~base(){}
        } ;

        class derived_one : public base {
           friend  class subfactory<derived_one, base> ;
           typedef register_factory<derived_one, base> my_factory ;
           static my_factory register_me ;
        protected:
           derived_one() : base("derived_one") {}
        public:
           ~derived_one(){}
        } ;

        class derived_two : public base {
           friend  class subfactory<derived_two, base> ;
           typedef register_factory<derived_two, base> my_factory ;
           static my_factory register_me ;
        protected:
           derived_two() : base("derived_two") {}
        public:
           ~derived_two(){}
        } ;

        derived_one::my_factory register_me("one") ;
        derived_two::my_factory register_me("two") ;

        int main()
        {
           base* a = factory<base>::create("one") ;
           base* b = factory<base>::create("two") ;

           std::cout << "a is an instance of " << a->get() << '\n' ;
           std::cout << "b is an instance of " << b->get() << '\n' ;
           return 0 ;
        }

   The framework is meant to create instances of derived classes in terms
   of base class pointers. Every derived class needs to include some
   boilerplate code to make things work as intended. Here are the steps
   to follow to set things up correctly:

        1. If the derived class has protected or private constructors,
           declare subfactory<derived_class, base_class> as a friend so
           that the factory has access to the constructor.

        2. Include a static data member of type
           register_factory<derived_class, base_class> and specify the
           identifying token in its initializer.

           NOTE: If it is important to prevent the static initialization
           dependency problem for these register_factory instances, put
           all of them in a central registry.cc file that lists them in
           the desired order of construction.

        3. To create new objects, use factory<base_class>::create() and
           pass this method the appropriate string or other identifying
           token.

   Note that it is not necessary to have any extraneous code in the base
   class's definition.

   The above examples and discussion showed how classes may be
   instantiated using their default constructors. If a particular class's
   constructor needs some parameters, they should be packaged into a
   single structure or object of some type and then passed to the
   factory's create method like so:

        #include "factory.hh"
        #include <iostream>
        #include <string>

        class base {
        protected:
           std::string m_type ;
           base(const std::string& type = "base") : m_type(type) {}

        public:
           virtual void dump(std::ostream&) const = 0 ;
           virtual ~base(){}
        } ;

        class derived_one : public base {
        public:
           struct arg_type {
              int i ;
              float f ;
           } ;

        private:
           friend  class subfactory<derived_one, base, arg_type> ;
           typedef register_factory<derived_one, base, arg_type> my_factory ;
           static my_factory register_me ;

           arg_type m_arg ;

           derived_one(const arg_type& A) : base("derived_one"), m_arg(A) {}

           void dump(std::ostream& os) const {
              os << "type = " << m_type
                 << ", arg = [" << m_arg.i << ' ' << m_arg.f << "]\n" ;
           }

           ~derived_one(){}
        } ;

        class derived_two : public base {
        public:
           struct arg_type {
              std::string s ;
              double d ;
           } ;

        private:
           friend  class subfactory<derived_one, base, arg_type> ;
           typedef register_factory<derived_one, base, arg_type> my_factory ;
           static my_factory register_me ;

           arg_type m_arg ;

           derived_two(const arg_type& A) : base("derived_one"), m_arg(A) {}

           void dump(std::ostream& os) const {
              os << "type = " << m_type
                 << ", arg = [" << m_arg.s << ' ' << m_arg.d << "]\n" ;
           }

           ~derived_two(){}
        } ;

        derived_one::my_factory register_me("one") ;
        derived_two::my_factory register_me("two") ;

        int main()
        {
           base* a = factory<base, derived_one::arg_type>::
              create("one", derived_one::arg_type(10, 100.0f)) ;
           base* b = factory<base, derived_two::arg_type>::
              create("two", derived_two::arg_type("foo", 25.0)) ;

           a->dump(std::cout) ;
           b->dump(std::cout) ;
           return 0 ;
        }
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/factory.hh $
// $Id: factory.hh 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_FACTORY_DOT_HH
#define LOBOT_FACTORY_DOT_HH

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/LoSTL.H"

// Standard C++ headers
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

//----------------------- NAMESPACE DEFINITION --------------------------

namespace lobot {

//--------------------------- THE NULL TYPE -----------------------------

/**
   \class lobot::null_type
   \brief A null type to act as a placeholder when we need template
   arguments that really don't describe any type.
*/
class null_type {} ;

//-------------------------- GENERIC FACTORY ----------------------------

/**
   \class lobot::factory<T, A, I>
   \brief A generic factory class.

   This class can be used to churn out instances of type T given
   identifiers of type I. Usually, I is a string so that this factory
   churns of objects of type T given a string identifying the desired
   derived class type.

   In addition to the identifier type I, this factory also uses an
   argument type A. When the factory creates an object of type T, it will
   pass an instance of A to T's constructor. This allows clients to pass
   arguments to the target type's constructor.

   Only a single constructor argument is supported. However, clients can
   package multiple arguments into POD structure (i.e., plain old struct)
   and pass that in to constructors that require multiple data items for
   successful object creation. This means that client classes that
   require multiple inputs during instantiation must be written in such a
   way that all those inputs can be passed via a single structure.

   It is important to note that pointer to the newly created object is
   usually returned in terms of T's base class.
*/
template<typename T, typename A = null_type, typename I = std::string>
struct factory {
   /// This method creates an object of type T given an identifier of
   /// type I. Usually, I would be a string so that the factory would
   /// produce instances of T given strings identifying classes derived
   /// from T.
   ///
   /// Any parameters required by T's constructor are passed via an
   /// argument of type A. If T's constructor needs multiple parameters,
   /// it should written so that all those parameters can be put into a
   /// single POD structure (or full-blown object) of type A.
   static T* create(const I& class_identifier, const A& arg) ;

   /// This exception is thrown when the factory is requested to create
   /// an object of an unregistered type.
   struct unknown_type : public std::runtime_error {
      unknown_type(const std::string&) ;
   } ;

   // Destructor
   virtual ~factory() ;

protected:
   /// This method is the one that implements the polymorphic factory,
   /// i.e., it creates the actual instance of the derived class. The
   /// create method only looks up the appropriate derived class factory
   /// in the registry of subfactories and then calls this method of that
   /// subfactory to do the creation.
   virtual T* subcreate(const A& arg) = 0 ;
} ;

// Generic factory destructor
template<typename T, typename A, typename I>
factory<T,A,I>::~factory()
{}

// Generic factory exception
template<typename T, typename A, typename I>
factory<T,A,I>::unknown_type::unknown_type(const std::string& type_name)
   : std::runtime_error(type_name)
{}

/**
   \class lobot::factory<T, null_type, I>
   \brief A generic factory for creating objects using default
   constructors.

   Often, client classes don't need any parameters during instantiation,
   i.e., we want to simply use their default constructors. In these
   situations, this partial specialization of the lobot::factory<T,A,I>
   class serves to provide appropriate versions of the create() and
   subcreate() methods that don't take any input arguments for the target
   type's constructor.
*/
template<typename T, typename I>
struct factory<T, null_type, I> {
   /// This method creates an object of type T given an identifier of
   /// type I. Usually, I would be a string so that the factory would
   /// produce instances of T given strings identifying classes derived
   /// from T. The instance of T is created using T's default
   /// constructor, i.e., the constructor receives no input parameters.
   static T* create(const I& class_identifier) ;

   /// This exception is thrown when the factory is requested to create
   /// an object of an unregistered type.
   struct unknown_type : public std::runtime_error {
      unknown_type(const std::string&) ;
   } ;

   // Destructor
   virtual ~factory() ;

protected:
   /// This method is the one that implements the polymorphic factory,
   /// i.e., it creates the actual instance of the derived class. The
   /// create method only looks up the appropriate derived class factory
   /// in the registry of subfactories and then calls this method of that
   /// subfactory to do the creation.
   virtual T* subcreate() = 0 ;
} ;

// Generic factory destructor
template<typename T, typename I>
factory<T, null_type, I>::~factory()
{}

// Generic factory exception
template<typename T, typename I>
factory<T, null_type, I>::unknown_type::
unknown_type(const std::string& type_name)
   : std::runtime_error(type_name)
{}

//--------------------------- SUBFACTORIES ------------------------------

/**
   \class lobot::subfactory<T, B, A, I>
   \brief Generic subfactory for objects of type T.

   This class extends lobot::factory<T,A,I> and provides the subcreate()
   factory method that actually produces the object of type T.

   The template expects to be passed not only the type T that the factory
   is meant to produce, but also the type B, i.e., the base class of T.

   WARNING: If B is not the base class of T, expect things to go wrong.

   In addition to the target type T and T's base class type B, the
   generic subfactory also requires an argument type A for any parameters
   that should be supplied to T's constructor and, of course, the
   factory's identifier type I.
*/
template<typename T, typename B,
         typename A = null_type, typename I = std::string>
struct subfactory : public factory<B,A,I> {
protected:
   /// The subfactory method that produces instances of T upcast to their
   /// base classes. This method produces objects of type T using a
   /// single argument constructor of T. If type T needs more than one
   /// parameter for proper construction, all those parameters should be
   /// packaged into a single POD structure (or full-blown object) of
   /// type A.
   B* subcreate(const A& arg) ;
} ;

// Create an instance of T using a constructor that takes an argument
template<typename T, typename B, typename A, typename I>
B* subfactory<T,B,A,I>::subcreate(const A& arg)
{
   return new T(arg) ;
}

/**
   \class lobot::subfactory<T, B, null_type, I>
   \brief Generic subfactory for objects of type T constructed using
   their default constructors.

   This partial specialization of lobot::subfactory<T,B,A,I> provides the
   subcreate() factory method that invokes the default constructor of
   target type T in order to produce an instance of T.

   The template expects to be passed not only the type T that the factory
   is meant to produce, but also the type B, i.e., the base class of T.

   WARNING: If B is not the base class of T, expect things to go wrong.
*/
template<typename T, typename B, typename I>
struct subfactory<T, B, null_type, I> : public factory<B, null_type, I> {
protected:
   /// The subfactory method that produces instances of T upcast to their
   /// base classes. This method produces objects of type T using the
   /// default constructor of T.
   B* subcreate() ;
} ;

// Create an instance of T using its default constructor
template<typename T, typename B, typename I>
B* subfactory<T, B, null_type, I>::subcreate()
{
   return new T ;
}

//----------------------------- REGISTRY --------------------------------

/**
   \class lobot::registry<T, A, I>
   \brief A registry of subfactories.

   This class holds a map that connects user-level class identifiers to
   their factories. That is, it maps identifiers of type I to
   subfactories of type T. Usually, I is a string. Therefore, the
   subfactory registry maps strings identifying derived class names to
   the appropriate subfactories that produce those objects.

   Additionally, the subfactories are expected to take a single argument
   of type A in their subcreate() methods that they pass through to T's
   constructor.
*/
template<typename T, typename A = null_type, typename I = std::string>
class registry : public singleton<registry<T,A,I> > {
   // Boilerplate code to make the singleton pattern work
   friend class singleton<registry> ;

   /// The factory registry maintains a mapping between the user-level
   /// identifiers and the corresponding factories that produce the
   /// desired objects.
   //@{
   typedef factory<T, A, I>* factory_ptr ;
   typedef std::map<I, factory_ptr> factory_map ;
   factory_map m_factories ;
   //@}

   /// A private constructor because the factory registry is a singleton
   /// object that cannot be created directly by clients.
   registry() ;

public:
   /// This method registers factories corresponding to a given
   /// identifier. If there already exists a factory for the given
   /// identifier, it will be replaced with the new one provided.
   void add(const I&, factory_ptr) ;

   /// This method return the factory corresponding to some identifier.
   /// If no such factory exists, a lobot::factory<T,A,I>::unknown_type
   /// exception will be thrown.
   factory_ptr find(const I&) const ;

   /// Delete all the factories when the singleton registry is finally
   /// cleaned up.
   ~registry() ;
} ;

// Registry constructor
template<typename T, typename A, typename I>
registry<T,A,I>::registry()
{}

// Registering subfactories with the registry
template<typename T, typename A, typename I>
void
registry<T,A,I>::
add(const I& key, registry<T,A,I>::factory_ptr F)
{
   typename factory_map::iterator it = m_factories.find(key) ;
   if (it == m_factories.end())
      m_factories.insert(std::make_pair(key, F)) ;
   else
   {
      delete it->second ;
      it->second = F ;
   }
}

// Finding subfactories in the registry
template<typename T, typename A, typename I>
typename registry<T,A,I>::factory_ptr
registry<T,A,I>::
find(const I& key) const
{
   typename factory_map::const_iterator it = m_factories.find(key) ;
   if (it == m_factories.end())
      throw typename factory<T,A,I>::unknown_type(to_string(key)) ;
   return it->second ;
}

// Registry clean-up
template<typename T, typename A, typename I>
registry<T,A,I>::~registry()
{
   typedef typename factory_map::value_type P ;
   purge_container(m_factories, delete_second<P>) ;
}

//---------------------- GENERIC FACTORY METHOD -------------------------

// DEVNOTE: Defined here rather than with generic factory class
// definition because it needs the registry class defined above. Doing it
// any other way would result in messy forward declarations and other
// unnecessary pain.

// Generic factory method for classes that take an argument in their
// constructors. If the type T needs more than one parameter for
// construction, the extra params should be packaged into a single POD
// structure (or even, if required, a full-blown object) of type A.
template<typename T, typename A, typename I>
T* factory<T,A,I>::create(const I& key, const A& arg)
{
   typedef registry<T,A,I> reg ;
   typedef factory<T,A,I>* fac ;

   const reg& R = reg::instance() ;
   fac F = R.find(key) ;
   return F->subcreate(arg) ;
}

// Partial specialization for generic factory method for classes that
// only provide default constructors.
template<typename T, typename I>
T* factory<T, null_type, I>::create(const I& key)
{
   typedef registry<T, null_type, I> reg ;
   typedef factory<T, null_type ,I>* fac ;

   const reg& R = reg::instance() ;
   fac F = R.find(key) ;
   return F->subcreate() ;
}

//---------------------- SUBFACTORY REGISTRATION ------------------------

/**
   \class lobot::register_factory<T, B, A, I>
   \brief Convenient interface for registering derived class factories.

   Instances of this class may be used to add subfactories to the
   registry. Typically, each derived class that wants to be created by a
   factory would have a static instance of this class.
*/
template<typename T, typename B,
         typename A = null_type, typename I = std::string>
struct register_factory {
   register_factory(const I& key) ;
private:
   typedef registry<B,A,I>     registry_t ;
   typedef subfactory<T,B,A,I> subfactory_t ;
} ;

template<typename T, typename B, typename A, typename I>
register_factory<T,B,A,I>::register_factory(const I& key)
{
   registry_t::instance().add(key, new subfactory_t) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
