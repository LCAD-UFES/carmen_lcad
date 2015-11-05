/** @file rutz/functors.h template mechanisms for determine parameter
    types and return types of functions or general functors */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Sep  7 15:07:16 2001
// commit: $Id: functors.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/functors.h $
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

#ifndef GROOVX_RUTZ_FUNCTORS_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_FUNCTORS_H_UTC20050626084020_DEFINED

namespace rutz
{
  //  #######################################################
  //  =======================================================
  //
  //  func_traits
  //
  //  =======================================================

  struct null_t;

  /// Holds typedefs for the types of a function's arguments and return value.
  template <class R = void,
            class A1 = null_t, class A2 = null_t, class A3 = null_t,
            class A4 = null_t, class A5 = null_t, class A6 = null_t,
            class A7 = null_t, class A8 = null_t>
  struct func_args
  {
    typedef R  retn_t;
    typedef A1 arg1_t;
    typedef A2 arg2_t;
    typedef A3 arg3_t;
    typedef A4 arg4_t;
    typedef A5 arg5_t;
    typedef A6 arg6_t;
    typedef A7 arg7_t;
    typedef A8 arg8_t;
  };

  /// A traits class for holding information about functions/functors.
  template <class func>
  struct func_traits
  {
    typedef typename func::retn_t retn_t;
  };

  //  =======================================================
  //
  //  func_traits specializations for free functions
  //
  //  =======================================================

  /// Specialization for free functions with no arguments.
  template <class R>
  struct func_traits<R (*)()>
    :
    public func_args<R>
  {
    enum { num_args = 0 };
  };

  /// Specialization for free functions with 1 argument.
  template <class R, class P1>
  struct func_traits<R (*)(P1)>
    :
    public func_args<R, P1>
  {
    enum { num_args = 1 };
  };

  /// Specialization for free functions with 2 arguments.
  template <class R, class P1, class P2>
  struct func_traits<R (*)(P1, P2)>
    :
    public func_args<R, P1, P2>
  {
    enum { num_args = 2 };
  };

  /// Specialization for free functions with 3 arguments.
  template <class R, class P1, class P2, class P3>
  struct func_traits<R (*)(P1, P2, P3)>
    :
    public func_args<R, P1, P2, P3>
  {
    enum { num_args = 3 };
  };

  /// Specialization for free functions with 4 arguments.
  template <class R, class P1, class P2, class P3, class P4>
  struct func_traits<R (*)(P1, P2, P3, P4)>
    :
    public func_args<R, P1, P2, P3, P4>
  {
    enum { num_args = 4 };
  };

  /// Specialization for free functions with 5 arguments.
  template <class R, class P1, class P2, class P3, class P4, class P5>
  struct func_traits<R (*)(P1, P2, P3, P4, P5)>
    :
    public func_args<R, P1, P2, P3, P4, P5>
  {
    enum { num_args = 5 };
  };

  /// Specialization for free functions with 6 arguments.
  template <class R, class P1, class P2, class P3, class P4, class P5,
            class P6>
  struct func_traits<R (*)(P1, P2, P3, P4, P5, P6)>
    :
    public func_args<R, P1, P2, P3, P3, P4, P5, P6>
  {
    enum { num_args = 6 };
  };

  //  =======================================================
  //
  //  func_traits specializations for member functions
  //
  //  We treat the "this" pointer as an implicit first-argument to the
  //  function.
  //
  //  =======================================================

  /// Specialization for member functions with "this" plus 0 arguments.
  template <class R, class C>
  struct func_traits<R (C::*)()>
    :
    public func_args<R, null_t>
  {
    enum { num_args = 1 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 0 arguments.
  template <class R, class C>
  struct func_traits<R (C::*)() const>
    :
    public func_args<R, null_t>
  {
    enum { num_args = 1 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 1 argument.
  template <class R, class C, class P1>
  struct func_traits<R (C::*)(P1)>
    :
    public func_args<R, null_t, P1>
  {
    enum { num_args = 2 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 1 argument.
  template <class R, class C, class P1>
  struct func_traits<R (C::*)(P1) const>
    :
    public func_args<R, null_t, P1>
  {
    enum { num_args = 2 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 2 arguments.
  template <class R, class C, class P1, class P2>
  struct func_traits<R (C::*)(P1, P2)>
    :
    public func_args<R, null_t, P1, P2>
  {
    enum { num_args = 3 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 2 arguments.
  template <class R, class C, class P1, class P2>
  struct func_traits<R (C::*)(P1, P2) const>
    :
    public func_args<R, null_t, P1, P2>
  {
    enum { num_args = 3 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 3 arguments.
  template <class R, class C, class P1, class P2, class P3>
  struct func_traits<R (C::*)(P1, P2, P3)>
    :
    public func_args<R, null_t, P1, P2, P3>
  {
    enum { num_args = 4 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 3 arguments.
  template <class R, class C, class P1, class P2, class P3>
  struct func_traits<R (C::*)(P1, P2, P3) const>
    :
    public func_args<R, null_t, P1, P2, P3>
  {
    enum { num_args = 4 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 4 arguments.
  template <class R, class C, class P1, class P2, class P3, class P4>
  struct func_traits<R (C::*)(P1, P2, P3, P4)>
    :
    public func_args<R, null_t, P1, P2, P3, P4>
  {
    enum { num_args = 5 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 4 arguments.
  template <class R, class C, class P1, class P2, class P3, class P4>
  struct func_traits<R (C::*)(P1, P2, P3, P4) const>
    :
    public func_args<R, null_t, P1, P2, P3, P4>
  {
    enum { num_args = 5 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 5 arguments.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5>
  struct func_traits<R (C::*)(P1, P2, P3, P4, P5)>
    :
    public func_args<R, null_t, P1, P2, P3, P4, P5>
  {
    enum { num_args = 6 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 5 arguments.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5>
  struct func_traits<R (C::*)(P1, P2, P3, P4, P5) const>
    :
    public func_args<R, null_t, P1, P2, P3, P4, P5>
  {
    enum { num_args = 6 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 6 arguments.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5, class P6>
  struct func_traits<R (C::*)(P1, P2, P3, P4, P5, P6)>
    :
    public func_args<R, null_t, P1, P2, P3, P4, P5, P6>
  {
    enum { num_args = 7 };
    typedef C class_t;
  };

  /// Specialization for member functions with "this" plus 6 arguments.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5, class P6>
  struct func_traits<R (C::*)(P1, P2, P3, P4, P5, P6) const>
    :
    public func_args<R, null_t, P1, P2, P3, P4, P5, P6>
  {
    enum { num_args = 7 };
    typedef C class_t;
  };

  //  #######################################################
  //  =======================================================

  template <class F>
  class mem_functor_base;

  /// func_traits specialization for mem_functor_base.
  template <class MF>
  struct func_traits<mem_functor_base<MF> > : public func_traits<MF>
  {};

  /// mem_functor_base adapts a member function to an ordinary operator().
  /** The "this" pointer is passed through the first argument of the
      operator() call, via a raw pointer or a smart pointer. */

  template <class mem_func>
  class mem_functor_base
    :
    public func_traits<mem_functor_base<mem_func> >
  {
  private:
    mem_func m_held_func;

  public:
    typedef typename func_traits<mem_func>::retn_t R;
    typedef typename func_traits<mem_func>::class_t C;

    mem_functor_base(mem_func f) : m_held_func(f) {}

    /// Function-call operator for object + 0 args.
    template <class ptr>
    R operator()(ptr obj)
    {
      return ((*obj).*m_held_func)();
    }

    /// Function-call operator for object + 1 arg.
    template <class ptr, class P1>
    R operator()(ptr obj, P1 p1)
    {
      return ((*obj).*m_held_func)(p1);
    }

    /// Function-call operator for object + 2 args.
    template <class ptr, class P1, class P2>
    R operator()(ptr obj, P1 p1, P2 p2)
    {
      return ((*obj).*m_held_func)(p1, p2);
    }

    /// Function-call operator for object + 3 args.
    template <class ptr, class P1, class P2, class P3>
    R operator()(ptr obj, P1 p1, P2 p2, P3 p3)
    {
      return ((*obj).*m_held_func)(p1, p2, p3);
    }

    /// Function-call operator for object + 4 args.
    template <class ptr, class P1, class P2, class P3, class P4>
    R operator()(ptr obj, P1 p1, P2 p2, P3 p3, P4 p4)
    {
      return ((*obj).*m_held_func)(p1, p2, p3, p4);
    }

    /// Function-call operator for object + 5 args.
    template <class ptr, class P1, class P2, class P3, class P4,
              class P5>
    R operator()(ptr obj, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5)
    {
      return ((*obj).*m_held_func)(p1, p2, p3, p4, p5);
    }

    /// Function-call operator for object + 6 args.
    template <class ptr, class P1, class P2, class P3, class P4,
              class P5, class P6>
    R operator()(ptr obj, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6)
    {
      return ((*obj).*m_held_func)(p1, p2, p3, p4, p5, p6);
    }
  };

  //  #######################################################
  //  =======================================================

  /// mem_functor extends mem_functor_base smart pointers for "this".

  //  =======================================================

  template <class mem_func>
  struct mem_functor : public mem_functor_base<mem_func>
  {
    mem_functor(mem_func f) : mem_functor_base<mem_func>(f) {}

    // operator()'s inherited from mem_functor_base
  };

  /// Factory function to make a mem_functor from any member function.
  template <class MF>
  inline mem_functor<MF> mem_func(MF mf)
  {
    return mf;
  }


  //  #######################################################
  //  =======================================================

  ///  Traits struct for specifying a "functor" type given a function pointer.
  template <class fptr>
  struct functor_of
  {
    typedef fptr type;
  };

  /// Specialization for zero-arg mem func.
  template <class R, class C>
  struct functor_of< R (C::*)() >
  {
    typedef rutz::mem_functor<R (C::*)()> type;
  };

  /// Specialization for zero-arg const mem func.
  template <class R, class C>
  struct functor_of< R (C::*)() const >
  {
    typedef rutz::mem_functor<R (C::*)() const> type;
  };

  /// Specialization for one-arg mem func.
  template <class R, class C, class P1>
  struct functor_of< R (C::*)(P1) >
  {
    typedef rutz::mem_functor<R (C::*)(P1)> type;
  };

  /// Specialization for one-arg const mem func.
  template <class R, class C, class P1>
  struct functor_of< R (C::*)(P1) const >
  {
    typedef rutz::mem_functor<R (C::*)(P1) const> type;
  };

  /// Specialization for two-arg mem func.
  template <class R, class C, class P1, class P2>
  struct functor_of< R (C::*)(P1, P2) >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2)> type;
  };

  /// Specialization for two-arg const mem func.
  template <class R, class C, class P1, class P2>
  struct functor_of< R (C::*)(P1, P2) const >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2) const> type;
  };

  /// Specialization for three-arg mem func.
  template <class R, class C, class P1, class P2, class P3>
  struct functor_of< R (C::*)(P1, P2, P3) >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3)> type;
  };

  /// Specialization for three-arg const mem func.
  template <class R, class C, class P1, class P2, class P3>
  struct functor_of< R (C::*)(P1, P2, P3) const >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3) const> type;
  };

  /// Specialization for four-arg mem func.
  template <class R, class C, class P1, class P2, class P3, class P4>
  struct functor_of< R (C::*)(P1, P2, P3, P4) >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3, P4)> type;
  };

  /// Specialization for four-arg const mem func.
  template <class R, class C, class P1, class P2, class P3, class P4>
  struct functor_of< R (C::*)(P1, P2, P3, P4) const >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3, P4) const> type;
  };

  /// Specialization for 5-arg mem func.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5>
  struct functor_of< R (C::*)(P1, P2, P3, P4, P5) >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3, P4, P5)> type;
  };

  /// Specialization for 5-arg const mem func.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5>
  struct functor_of< R (C::*)(P1, P2, P3, P4, P5) const >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3, P4, P5) const> type;
  };

  /// Specialization for 6-arg mem func.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5, class P6>
  struct functor_of< R (C::*)(P1, P2, P3, P4, P5, P6) >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3, P4, P5, P6)> type;
  };

  /// Specialization for 6-arg const mem func.
  template <class R, class C, class P1, class P2, class P3, class P4,
            class P5, class P6>
  struct functor_of< R (C::*)(P1, P2, P3, P4, P5, P6) const >
  {
    typedef rutz::mem_functor<R (C::*)(P1, P2, P3, P4, P5, P6) const> type;
  };


  // ########################################################
  //  =======================================================

  /// Factory function for building a "functor" from any function pointer.
  template <class fptr>
  inline typename functor_of<fptr>::type
  build_functor(fptr f)
  {
    typedef typename functor_of<fptr>::type functor_t;
    return functor_t(f);
  }

  //  #######################################################
  //  =======================================================

  template <class base_functor, class bound_t>
  class bound_first;

  /// func_traits specialization for bound_first.
  template <class base_functor, class bound_t>
  struct func_traits<bound_first<base_functor, bound_t> >
  {
    enum { num_args = (func_traits<base_functor>::num_args-1) };

    typedef typename func_traits<base_functor>::retn_t   retn_t;
    typedef typename func_traits<base_functor>::arg2_t   arg1_t;
    typedef typename func_traits<base_functor>::arg3_t   arg2_t;
    typedef typename func_traits<base_functor>::arg4_t   arg3_t;
    typedef typename func_traits<base_functor>::arg5_t   arg4_t;
    typedef typename func_traits<base_functor>::arg6_t   arg5_t;
    typedef typename func_traits<base_functor>::arg7_t   arg6_t;
    typedef typename func_traits<base_functor>::arg8_t   arg7_t;
    typedef                                     null_t   arg8_t;
  };

  /// bound_first wraps another functor type with a fixed first argument.
  /** bound_first's can be constructed with the factory function
      bind_first(). */

  template <class base_functor, class bound_t>
  class bound_first
    :
    public func_traits<bound_first<base_functor, bound_t> >
  {
  private:
    base_functor m_held_func;
    bound_t m_bound;

  public:
    bound_first(base_functor base, bound_t bound) :
      m_held_func(base),
      m_bound(bound)
    {}

    bound_first(const bound_first& other) :
      m_held_func(other.m_held_func),
      m_bound(other.m_bound)
    {}

    typedef func_traits<bound_first<base_functor, bound_t> > traits;
    typedef typename traits::retn_t   retn_t;
    typedef typename traits::arg1_t   arg1_t;
    typedef typename traits::arg2_t   arg2_t;
    typedef typename traits::arg3_t   arg3_t;
    typedef typename traits::arg4_t   arg4_t;
    typedef typename traits::arg5_t   arg5_t;
    typedef typename traits::arg6_t   arg6_t;
    typedef typename traits::arg7_t   arg7_t;
    typedef typename traits::arg8_t   arg8_t;

    //
    // All versions of operator() are provided, but only the one that
    // involves the correct call to m_held_func() will compile
    // successfully
    //

    retn_t operator()()
    {
      return m_held_func(m_bound);
    }
    retn_t operator()(arg1_t p1)
    {
      return m_held_func(m_bound, p1);
    }
    retn_t operator()(arg1_t p1, arg2_t p2)
    {
      return m_held_func(m_bound, p1, p2);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3)
    {
      return m_held_func(m_bound, p1, p2, p3);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4)
    {
      return m_held_func(m_bound, p1, p2, p3, p4);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4,
                      arg5_t p5)
    {
      return m_held_func(m_bound, p1, p2, p3, p4, p5);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4,
                      arg5_t p5, arg6_t p6)
    {
      return m_held_func(m_bound, p1, p2, p3, p4, p5, p6);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4,
                      arg5_t p5, arg6_t p6, arg7_t p7)
    {
      return m_held_func(m_bound, p1, p2, p3, p4, p5, p6, p7);
    }

  private:
    bound_first& operator=(const bound_first&);
  };

  /// Factory function for creating bound_first functors.
  template <class base_functor, class bound_t>
  bound_first<base_functor, bound_t> bind_first(base_functor base,
                                                bound_t bound)
  {
    return bound_first<base_functor, bound_t>(base, bound);
  }


  //  #######################################################
  //  =======================================================

  template <class base_functor, class bound_t>
  class bound_last;

  /// func_traits specialization for bound_last.
  template <class base_functor, class bound_t>
  struct func_traits<bound_last<base_functor, bound_t> >
  {
    enum { num_args = (func_traits<base_functor>::num_args-1) };

    typedef typename func_traits<base_functor>::retn_t   retn_t;
    typedef typename func_traits<base_functor>::arg1_t   arg1_t;
    typedef typename func_traits<base_functor>::arg2_t   arg2_t;
    typedef typename func_traits<base_functor>::arg3_t   arg3_t;
    typedef typename func_traits<base_functor>::arg4_t   arg4_t;
    typedef typename func_traits<base_functor>::arg5_t   arg5_t;
    typedef typename func_traits<base_functor>::arg6_t   arg6_t;
    typedef typename func_traits<base_functor>::arg7_t   arg7_t;
    typedef typename func_traits<base_functor>::arg8_t   arg8_t;
  };

  /// bound_last wraps another functor type with a fixed last argument.
  /** bound_last objects can be constructed with the factory function
      bind_last(). */

  template <class base_functor, class bound_t>
  class bound_last
    :
    public func_traits<bound_last<base_functor, bound_t> >
  {
  private:
    base_functor m_held_func;
    bound_t m_bound;

  public:
    bound_last(base_functor base, bound_t bound) :
      m_held_func(base),
      m_bound(bound)
    {}

    bound_last(const bound_last& other) :
      m_held_func(other.m_held_func),
      m_bound(other.m_bound)
    {}

    typedef func_traits<bound_last<base_functor, bound_t> > traits;
    typedef typename traits::retn_t   retn_t;
    typedef typename traits::arg1_t   arg1_t;
    typedef typename traits::arg2_t   arg2_t;
    typedef typename traits::arg3_t   arg3_t;
    typedef typename traits::arg4_t   arg4_t;
    typedef typename traits::arg5_t   arg5_t;
    typedef typename traits::arg6_t   arg6_t;
    typedef typename traits::arg7_t   arg7_t;
    typedef typename traits::arg8_t   arg8_t;

    //
    // Multiple versions of operator() are provided, but only the one
    // that involves the correct call to m_held_func() will compile
    // successfully
    //

    retn_t operator()()
    {
      return m_held_func(m_bound);
    }
    retn_t operator()(arg1_t p1)
    {
      return m_held_func(p1, m_bound);
    }
    retn_t operator()(arg1_t p1, arg2_t p2)
    {
      return m_held_func(p1, p2, m_bound);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3)
    {
      return m_held_func(p1, p2, p3, m_bound);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4)
    {
      return m_held_func(p1, p2, p3, p4, m_bound);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4,
                      arg5_t p5)
    {
      return m_held_func(p1, p2, p3, p4, p5, m_bound);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4,
                      arg5_t p5, arg6_t p6)
    {
      return m_held_func(p1, p2, p3, p4, p5, p6, m_bound);
    }
    retn_t operator()(arg1_t p1, arg2_t p2, arg3_t p3, arg4_t p4,
                      arg5_t p5, arg6_t p6, arg7_t p7)
    {
      return m_held_func(p1, p2, p3, p4, p5, p6, p7, m_bound);
    }

  private:
    bound_last& operator=(const bound_last&);
  };

  /// Factory function for creating bound_last functors.
  template <class base_functor, class bound_t>
  bound_last<base_functor, bound_t> bind_last(base_functor base,
                                              bound_t bound)
  {
    return bound_last<base_functor, bound_t>(base, bound);
  }

} // end namespace rutz

static const char __attribute__((used)) vcid_groovx_rutz_functors_h_utc20050626084020[] = "$Id: functors.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/functors.h $";
#endif // !GROOVX_RUTZ_FUNCTORS_H_UTC20050626084020_DEFINED
