/** @file rutz/iter.h generic iterator classes */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Aug 17 11:05:24 2001
// commit: $Id: iter.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/iter.h $
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

#ifndef GROOVX_RUTZ_ITER_H_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_ITER_H_UTC20050626084019_DEFINED

#include "rutz/shared_ptr.h"

#include <utility> // for std::pair

namespace rutz
{
  /// Symbol class for representing generic "end of iteration".
  struct iter_end_t {};

  extern const iter_end_t iter_end;

  template <class T> class fwd_iter;
  template <class T> class bidir_iter;
  template <class T> class rxs_iter;

  template <class T> class fwd_iter_ifx;
  template <class T> class bidir_iter_ifx;
  template <class T> class rxs_iter_ifx;

  template <class real_iter_t, class T> class fwd_iter_adapter;
  template <class real_iter_t, class T> class bidir_iter_adapter;
  template <class real_iter_t, class T> class rxs_iter_adapter;

  template <class T, class ifx_t> class concrete_iter;

  template <class T>
  inline T& getref(T& t) { return t; }

  template <class T1, class T2>
  inline T2& getref(std::pair<T1, T2>& p) { return p.second; }

  ///////////////////////////////////////////////////////////
  //
  // concrete_iter class
  //
  ///////////////////////////////////////////////////////////

  /// A template base class for all concrete iterator classes.
  /** concrete_iter provides a "fat" interface, but a compile-time
      error will be generated if part of the interface is used that is
      not supported by the implementation class. */
  template <class T, class ifx_t>
  class concrete_iter
  {
    shared_ptr<ifx_t> rep;

    void make_unique()
    {
      if ( !rep.unique() )
        {
          rep.reset( rep->clone() );
        }
    }

  public:
    concrete_iter(const concrete_iter& other) : rep(other.rep) {}

    concrete_iter(shared_ptr<ifx_t> impl)      : rep(impl) {}

    // Default assigment-oper OK

    void next()                     { make_unique(); rep->next(); }
    void prev()                     { make_unique(); rep->prev(); }
    void step(int n)                { make_unique(); rep->step(n); }
    concrete_iter& operator++()     { next(); return *this; }
    concrete_iter operator++(int)   { concrete_iter c(*this); next(); return c; }
    concrete_iter& operator--()     { prev(); return *this; }
    concrete_iter operator--(int)   { concrete_iter c(*this); prev(); return c; }

    concrete_iter operator+=(int n) { step(n); return *this; }
    concrete_iter operator-=(int n) { step(-n); return *this; }

    int operator-(const concrete_iter& other) const
    { return rep->minus(other.rep); }

    T*   operator->()                  const { return &(rep->get()); }
    T&   operator*()                   const { return rep->get(); }

    bool at_end()                      const { return rep->at_end(); }
    bool is_valid()                    const { return !at_end(); }
    int  from_end()                    const { return rep->from_end(); }
    bool operator==(const iter_end_t&) const { return at_end(); }
    bool operator!=(const iter_end_t&) const { return !at_end(); }
  };


  ///////////////////////////////////////////////////////////
  //
  // Forward iterators
  //
  ///////////////////////////////////////////////////////////


  /// Abstract interface class for forward iterators.
  template <class T>
  class fwd_iter_ifx
  {
  public:
    typedef T value_t;
    typedef fwd_iter_ifx<T> ifx_t;

    virtual        ~fwd_iter_ifx()  {}
    virtual ifx_t* clone()  const = 0;
    virtual void   next()         = 0;
    virtual T&     get()    const = 0;
    virtual bool   at_end() const = 0;
  };


  /// Adapts forward iterators to the fwd_iter_ifx interface.
  template <class real_iter_t, class T>
  class fwd_iter_adapter : public fwd_iter_ifx<T>
  {
    // assignment operator (not implemented; use clone() instead)
    fwd_iter_adapter<real_iter_t, T>&
    operator=(const fwd_iter_adapter<real_iter_t, T>&);

    typedef fwd_iter_ifx<T> base_t;

    real_iter_t m_iter;
    real_iter_t m_end;

    // copy constructor
    fwd_iter_adapter<real_iter_t, T>
    (const fwd_iter_adapter<real_iter_t, T>& that)
      :
      base_t(), m_iter(that.m_iter), m_end(that.m_end) {}

  public:
    /// Construct from a pair of "real" iterators.
    fwd_iter_adapter<real_iter_t, T>
    (real_iter_t iter, real_iter_t end)
      :
      base_t(), m_iter(iter), m_end(end) {}

    virtual base_t* clone() const { return new fwd_iter_adapter(*this); }
    virtual void     next()       { ++m_iter; }
    virtual T&        get() const { return getref(*m_iter); }
    virtual bool   at_end() const { return m_iter == m_end; }
  };


  /// Concrete forward iterator class.
  template <class T>
  class fwd_iter :
    public concrete_iter<T, fwd_iter_ifx<T> >
  {
    template <class It>
    shared_ptr<fwd_iter_ifx<T> >
    adapt(It iter, It end)
    {
      return shared_ptr<fwd_iter_ifx<T> >
        (new fwd_iter_adapter<It, T>(iter, end));
    }

  public:
    typedef fwd_iter_ifx<T> ifx_t;
    typedef concrete_iter<T, ifx_t> base_t;

    fwd_iter(const base_t& other) : base_t(other) {}

    fwd_iter(shared_ptr<ifx_t> impl) : base_t(impl) {}

    template <class It>
    fwd_iter(It iter, It end) : base_t(adapt(iter, end)) {}
  };

  ///////////////////////////////////////////////////////////
  //
  // Bidirectional iterators
  //
  ///////////////////////////////////////////////////////////


  /// Abstract interface class for bidirectional iterators.
  template <class T>
  class bidir_iter_ifx : public fwd_iter_ifx<T>
  {
  public:
    typedef bidir_iter_ifx<T> ifx_t;

    virtual ifx_t* clone() const = 0;
    virtual void   prev()        = 0;
  };


  /// Adapts bidirectional iterators to the bidir_iter_ifx interface.
  template <class real_iter_t, class T>
  class bidir_iter_adapter : public bidir_iter_ifx<T>
  {
    // assignment operator (not implemented; use clone() instead)
    bidir_iter_adapter<real_iter_t, T>&
    operator=(const bidir_iter_adapter<real_iter_t, T>&);

    typedef bidir_iter_ifx<T> base_t;

    real_iter_t m_iter;
    real_iter_t m_end;

    // copy constructor
    bidir_iter_adapter<real_iter_t, T>
    (const bidir_iter_adapter<real_iter_t, T>& that)
      :
      base_t(), m_iter(that.m_iter), m_end(that.m_end) {}

  public:
    /// Construct from a pair of "real" iterators.
    bidir_iter_adapter<real_iter_t, T>
    (real_iter_t iter, real_iter_t end)
      :
      base_t(), m_iter(iter), m_end(end) {}

    virtual base_t* clone() const { return new bidir_iter_adapter(*this); }
    virtual void     next()       { ++m_iter; }
    virtual void     prev()       { --m_iter; }
    virtual T&        get() const { return getref(*m_iter); }
    virtual bool   at_end() const { return m_iter == m_end; }
  };


  /// Concrete bidirectional iterator class.
  template <class T>
  class bidir_iter :
    public concrete_iter<T, bidir_iter_ifx<T> >
  {
    template <class It>
    shared_ptr<bidir_iter_ifx<T> >
    adapt(It iter, It end)
    {
      return shared_ptr<bidir_iter_ifx<T> >
        (new bidir_iter_adapter<It, T>(iter, end));
    }

  public:
    typedef bidir_iter_ifx<T> ifx_t;
    typedef concrete_iter<T, ifx_t> base_t;

    bidir_iter(const base_t& other) : base_t(other) {}

    bidir_iter(shared_ptr<ifx_t> impl) : base_t(impl) {}

    template <class It>
    bidir_iter(It iter, It end) : base_t(adapt(iter, end)) {}
  };


  ///////////////////////////////////////////////////////////
  //
  // Random Access iterators
  //
  ///////////////////////////////////////////////////////////


  /// Abstract interface class for random-access iterators.
  template <class T>
  class rxs_iter_ifx : public bidir_iter_ifx<T>
  {
  public:
    typedef rxs_iter_ifx<T> ifx_t;

    virtual ifx_t* clone()                   const = 0;
    virtual void   step(int n)                     = 0;
    virtual int    minus(const ifx_t& other) const = 0;
    virtual int    from_end()                const = 0;
  };


  /// Adapts random-access iterators to the rxs_iter_ifx interface.
  template <class real_iter_t, class T>
  class rxs_iter_adapter : public rxs_iter_ifx<T>
  {
    // assignment operator (not implemented; use clone() instead)
    rxs_iter_adapter<real_iter_t, T>&
    operator=(const rxs_iter_adapter<real_iter_t, T>&);

    typedef rxs_iter_ifx<T> base_t;

    real_iter_t m_iter;
    real_iter_t m_end;

    // copy constructor
    rxs_iter_adapter<real_iter_t, T>
    (const rxs_iter_adapter<real_iter_t, T>& that)
      :
      base_t(), m_iter(that.m_iter), m_end(that.m_end) {}

  public:
    /// Construct from a pair of "real" iterators.
    rxs_iter_adapter<real_iter_t, T>
    (real_iter_t iter, real_iter_t end)
      :
      base_t(), m_iter(iter), m_end(end) {}

    virtual base_t* clone()      const { return new rxs_iter_adapter(*this); }
    virtual void     next()            { ++m_iter; }
    virtual void     prev()            { --m_iter; }
    virtual void     step(int n)       { m_iter += n; }
    virtual T&        get()      const { return getref(*m_iter); }
    virtual bool   at_end()      const { return m_iter == m_end; }
    virtual int  from_end()      const { return m_end - m_iter; }

    virtual int minus(const rxs_iter_ifx<T>& other_) const
    {
      rxs_iter_adapter<real_iter_t, T>& other =
        dynamic_cast<const rxs_iter_adapter<real_iter_t, T>& >(other_);

      return m_iter - other.m_iter;
    }
  };


  /// Concrete random-access iterator class.
  template <class T>
  class rxs_iter :
    public concrete_iter<T, rxs_iter_ifx<T> >
  {
    template <class It>
    shared_ptr<rxs_iter_ifx<T> >
    adapt(It iter, It end)
    {
      return shared_ptr<rxs_iter_ifx<T> >
        (new rxs_iter_adapter<It, T>(iter, end));
    }

  public:
    typedef rxs_iter_ifx<T> ifx_t;
    typedef concrete_iter<T, ifx_t> base_t;

    rxs_iter(const base_t& other) : base_t(other) {}

    rxs_iter(shared_ptr<ifx_t> impl) : base_t(impl) {}

    template <class It>
    rxs_iter(It iter, It end) : base_t(adapt(iter, end)) {}
  };

} // end namespace rutz

static const char __attribute__((used)) vcid_groovx_rutz_iter_h_utc20050626084019[] = "$Id: iter.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/iter.h $";
#endif // !GROOVX_RUTZ_ITER_H_UTC20050626084019_DEFINED
