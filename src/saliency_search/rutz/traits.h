/** @file rutz/traits.h various traits classes for determining type
    characteristics at compile time */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri May 18 16:13:27 2001
// commit: $Id: traits.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/traits.h $
//
// --------------------------------------------------------------------
//
// This file is part of GroovX.
//   [http://ilab.usc.edu/rjpeters/groovx/]
//
// GroovX is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// GroovX is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GroovX; if not, write to the Free Software Foundation,
// Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
///////////////////////////////////////////////////////////////////////

#ifndef GROOVX_RUTZ_TRAITS_H_UTC20050626084021_DEFINED
#define GROOVX_RUTZ_TRAITS_H_UTC20050626084021_DEFINED

namespace rutz
{
  /// Basic type traits class.
  template <class T>
  struct type_traits
  {
    typedef T deref_t;
    typedef T stack_t;
  };

  /// Specialization of type traits for pointers.
  template <class T>
  struct type_traits<T*>
  {
    typedef T pointee_t;
    typedef T deref_t;
  };

  /// Specialization of type traits for references.
  template <class T>
  struct type_traits<T&>
  {
    typedef T stack_t;
  };

  /// Specialization of type traits for const references.
  template <class T>
  struct type_traits<const T&>
  {
    typedef T stack_t;
  };

  /// Select between two types based on a compile-time constant boolean expression.
  template <bool test, class if_true, class if_false>
  struct select_if
  {
    typedef if_true result_t;
  };

  /// Specialization of select_if for 'false'.
  template <class if_true, class if_false>
  struct select_if<false, if_true, if_false>
  {
    typedef if_false result_t;
  };

  namespace traits
  {
    /** dummy type */ struct yes_type { char x; };
    /** dummy type */ struct no_type  { yes_type x[2]; };
  }

  /// Helper class for is_sub_super.
  template <class T>
  struct type_match
  {
    static traits::yes_type foo(T* p);
    static traits::no_type  foo(...);
  };

  /// Determine whether sub derives from super.
  template <class sub, class super>
  struct is_sub_super
  {
    enum { sz = sizeof(type_match<super>::foo(static_cast<sub*>(0))) };

    enum
      {
        result = ((sz == sizeof(traits::yes_type)) ? 1 : 0)
      };
  };

  /// Remove const/volative qualifiers

  // From boost/type_traits/is_class.hpp:
  template <class U> traits::yes_type is_class_tester(void(U::*)(void));
  template <class U> traits::no_type  is_class_tester(...);

  /// Traits class to tell us whether T is a class type or not.
  template <typename T>
  struct is_class
  {
    enum
      {
        value = (sizeof(is_class_tester<T>(0))
                 == sizeof(traits::yes_type))
      };
  };

  /// Helper struct for telling whether T is a polymorphic type or not.
  /** The implementation trick here is that, if T is NOT polymorphic,
      then if we derive a new type from T that has virtual functions,
      then its sizeof() should increase to make room for the vtable
      pointer. On the other hand, if T is already polymorphic, then it
      already has a vtable ptr and so adding a new virtual function
      won't change sizeof() the derived type. */
  template <class T>
  struct is_polymorphic_imp1
  {
    typedef T ncvT;

    struct d1 : public ncvT
    {
      d1();
      ~d1()throw();
      char padding[256];
    };

    struct d2 : public ncvT
    {
      d2();
      virtual ~d2() throw();

      struct unique{};
      virtual void unique_name_to_invt200507011541(unique*);

      char padding[256];
    };

    enum { value = (sizeof(d2) == sizeof(d1)) };
  };

  template <class T>
  struct is_polymorphic_imp2
  {
    enum { value = false };
  };

  template <bool is_class>
  struct is_polymorphic_selector
  {
    template <class T>
    struct rebind
    {
      typedef is_polymorphic_imp2<T> type;
    };
  };

  template <>
  struct is_polymorphic_selector<true>
  {
    template <class T>
    struct rebind
    {
      typedef is_polymorphic_imp1<T> type;
    };
  };

  /// Traits class to tell whether T is a polymorphic type (i.e. has virtual functions).
  template <class T>
  struct is_polymorphic
  {
    typedef is_polymorphic_selector<is_class<T>::value> selector;
    typedef typename selector::template rebind<T> binder;
    typedef typename binder::type imp_type;
    enum { value = imp_type::value };
  };

  template <class T, bool polymorphic = is_polymorphic<T>::value >
  struct full_object_caster;

  template <class T>
  struct full_object_caster<T, false>
  {
    static const void* cast(const T* p) { return static_cast<const void*>(p); }
  };

  template <class T>
  struct full_object_caster<T, true>
  {
    static const void* cast(const T* p) { return dynamic_cast<const void*>(p); }
  };

  /// Cast a pointer to the beginning of the full object.
  /** Here we select between static_cast and dynamic_cast depending on
      whether T is polymorphic. */
  template <class T>
  inline const void* full_object_cast(const T* p)
  {
    return full_object_caster<T>::cast(p);
  }

}

static const char __attribute__((used)) vcid_groovx_rutz_traits_h_utc20050626084021[] = "$Id: traits.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/traits.h $";
#endif // !GROOVX_RUTZ_TRAITS_H_UTC20050626084021_DEFINED
