/** @file nub/ref.h smart pointers (both strong and weak) using
    intrusive ref-counting with nub::object and derivatives */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2000-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Oct 26 17:50:59 2000
// commit: $Id: ref.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/ref.h $
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

#ifndef GROOVX_NUB_REF_H_UTC20050626084019_DEFINED
#define GROOVX_NUB_REF_H_UTC20050626084019_DEFINED

#include "nub/handle.h"
#include "nub/object.h"
#include "nub/weak_handle.h"

#include "rutz/fileposition.h" // for SRC_POS macro
#include "rutz/stderror.h"     // for rutz::throw_bad_cast()
#include "rutz/traits.h"

#include <typeinfo>

namespace nub
{
  template <class T> class ref;
  template <class T> class soft_ref;
  template <class T> class floating_ref;

  //! Get the current default visibility (will control ref_vis DEFAULT).
  ref_vis get_default_ref_vis();

  //! Set the current default visibility (will control ref_vis DEFAULT).
  void set_default_ref_vis(ref_vis vis);

  namespace detail
  {
    bool is_valid_uid(nub::uid id) throw();
    nub::object* get_checked_item(nub::uid id);

    void insert_item(nub::object* obj, ref_vis vis);

    template <class T>
    inline T* get_casted_item(nub::uid id)
    {
      nub::object* obj = get_checked_item(id);
      T* t = dynamic_cast<T*>(obj);
      if (t == 0)
        rutz::throw_bad_cast(typeid(T), typeid(nub::object), SRC_POS);
      return t;
    }

    template <>
    inline nub::object* get_casted_item<nub::object>(nub::uid id)
    { return get_checked_item(id); }

    /// Policy class which unrefs objects by decrementing their ref count.
    template <class T>
    struct default_unref_policy
    {
      static void unref(T* t) throw() { t->decr_ref_count(); }
    };

    /// Policy class which decrements ref count without deletion.
    template <class T>
    struct no_delete_unref_policy
    {
      static void unref(T* t) throw() { t->decr_ref_count_no_delete(); }
    };

  } // end namespace nub::detail

} // end namespace nub

namespace rutz
{
  /// type_traits specialization for nub::ref smart pointer
  template <class T>
  struct type_traits<nub::ref<T> >
  {
    typedef T pointee_t;
  };

  /// type_traits specialization for soft_ref smart pointer.
  template <class T>
  struct type_traits<nub::soft_ref<T> >
  {
    typedef T pointee_t;
  };
}




///////////////////////////////////////////////////////////
/**
 *
 * nub::ref<T> is a ref-counted smart pointer for holding ref_counted
 * objects. A nub::ref<T> is guaranteed to always point to a valid
 * ref_counted object, and uses ref_counted's strong ref counts to
 * achieve this. In order to provide this guarantee, it is possible
 * that construction of a nub::ref<T> may throw an exception. For
 * example, a nub::ref<T> cannot be constructed for a volatlie
 * ref_counted object for which only weak references are available.
 *
 **/
///////////////////////////////////////////////////////////

template <class T>
class nub::ref
{
private:

  typedef nub::detail::handle<T, nub::detail::default_unref_policy<T> >
  handle;

  handle m_handle;

public:
  // Default destructor, copy constructor, operator=() are fine

  explicit ref(nub::uid i)
    : m_handle(detail::get_casted_item<T>(i)) {}

  explicit ref(T* ptr, ref_vis vis = DEFAULT) :
    m_handle(ptr)
  {
    detail::insert_item(ptr, vis);
  }

  template <class U>
  ref(const ref<U>& other) : m_handle(other.get()) {}

  // Will raise an exception if the soft_ref is invalid
  template <class U>
  ref(const soft_ref<U>& other);

  /// Shorthand for assignment.
  /** Given ref<T> rr and T* p, then rr.reset(p) is shorthand for
      rr=ref<T>(p). But of course, rr.reset(p) is much less typing
      if T happens to be spelt SomeLongType<WithTemplateParams>. */
  void reset(T* p) { *this = ref(p); }

  T* operator->() const throw() { return get(); }
  T& operator*()  const throw() { return *(get()); }

  T* get()        const throw() { return m_handle.get(); }

  bool operator==(const ref& other) const throw()
  { return m_handle == other.m_handle; }

  bool operator!=(const ref& other) const throw()
  { return !(operator==(other)); }

  nub::uid id() const throw()
  { return get()->id(); }

  /// Comparison operator for sorting.
  /** E.g. to allow insertion into std::map or std::set, etc. */
  bool operator<(const ref& other) const throw()
  { return get() < other.get(); }
};

template <class To, class Fr>
nub::ref<To> dyn_cast(nub::ref<Fr> p)
{
  Fr* f = p.get();
  To* t = dynamic_cast<To*>(f);
  if (t == 0)
    rutz::throw_bad_cast(typeid(To), typeid(Fr), SRC_POS);
  return nub::ref<To>(t);
}

template <class To, class Fr>
void dyn_cast_to_from(nub::ref<To>& dest, nub::ref<Fr> p)
{
  dest = dyn_cast<To, Fr>(p);
}

// Compatibility interface with alternate function-naming:

template <class To, class Fr>
nub::ref<To> dynCast(nub::ref<Fr> p)
{ return dyn_cast<To, Fr>(p); }

template <class To, class Fr>
void dynCastToFrom(nub::ref<To>& dest, nub::ref<Fr> p)
{ dyn_cast_to_from<To, Fr>(dest, p); }


///////////////////////////////////////////////////////////
/**
 *
 * nub::soft_ref<T> is a ref-counted smart pointer (like nub::ref<T>)
 * for holding ref_counted objects. Construction of a nub::soft_ref<T>
 * is guaranteed not to fail. Because of this, however, a
 * nub::soft_ref<T> is not guaranteed to always point to a valid
 * object (this must be tested with is_valid() before
 * dereferencing). With these characteristics, a nub::soft_ref<T> can
 * be used with volatile ref_counted objects for which only weak
 * references are available.
 *
 **/
///////////////////////////////////////////////////////////

template <class T>
class nub::soft_ref
{
private:
  mutable detail::weak_handle<T> m_handle;

public:
  soft_ref() : m_handle(0, STRONG) {}

  explicit soft_ref(nub::uid i, ref_type tp = STRONG)
    :
    m_handle(detail::is_valid_uid(i) ?
             detail::get_casted_item<T>(i) : 0,
             tp)
  {}

  explicit soft_ref(T* master,
                    ref_type tp = STRONG, ref_vis vis = DEFAULT)
    :
    m_handle(master,tp)
  {
    if (m_handle.is_valid())
      {
        detail::insert_item(m_handle.get(), vis);
      }
  }

  template <class U>
  soft_ref(const soft_ref<U>& other) :
    m_handle(other.is_valid() ? other.get() : 0, STRONG) {}

  template <class U>
  soft_ref(const ref<U>& other) : m_handle(other.get(), STRONG) {}

  // Default destructor, copy constructor, operator=() are fine

  /// Shorthand for assignment.
  /** Given ref<T> rr and T* p, then rr.reset(p) is shorthand for
      rr=ref<T>(p). But of course, rr.reset(p) is much less typing
      if T happens to be spelt SomeLongType<WithTemplateParams>. */
  void reset(T* p = 0) { *this = soft_ref(p); }

  /** Returns the pointee, or if throws an exception if there is not a
      valid pointee. */
  T* get()         const { return m_handle.get(); }

  T* operator->()  const { return m_handle.get(); }
  T& operator*()   const { return *(m_handle.get()); }

  /** Returns the pointee, or returns null if there is not a valid
      pointee. Will not throw an exception. */
  T* get_weak() const throw() { return m_handle.get_weak(); }

  ref_type get_ref_type() const throw() { return m_handle.get_ref_type(); }

  bool is_valid()   const throw() { return m_handle.is_valid(); }
  bool is_invalid() const throw() { return !(is_valid()); }

  bool operator==(const soft_ref& other) const throw()
  { return get_weak() == other.get_weak(); }

  bool operator!=(const soft_ref& other) const throw()
  { return get_weak() != other.get_weak(); }

  /// Comparison operator for sorting, to insert in std::map or std::set, etc.
  bool operator<(const soft_ref& other) const throw()
  { return get_weak() < other.get_weak(); }

  nub::uid id() const throw()
  { return m_handle.is_valid() ? m_handle.get()->id() : 0; }

  // Compatibility interface with alternate function-naming:

  T*   getWeak()   const throw() { return this->get_weak(); }
  bool isValid()   const throw() { return this->is_valid(); }
  bool isInvalid() const throw() { return this->is_invalid(); }
};



template <class To, class Fr>
inline nub::soft_ref<To> dyn_cast(nub::soft_ref<Fr> p)
{
  if (p.is_valid())
    {
      Fr* f = p.get();
      To* t = dynamic_cast<To*>(f);
      if (t == 0)
        rutz::throw_bad_cast(typeid(To), typeid(Fr), SRC_POS);
      return nub::soft_ref<To>(t);
    }
  return nub::soft_ref<To>(p.id());
}

template <class To, class Fr>
inline void dyn_cast_to_from(nub::soft_ref<To>& dest,
                             nub::soft_ref<Fr> p)
{
  dest = dyn_cast<To, Fr>(p);
}


template <class To, class Fr>
inline nub::soft_ref<To> dyn_cast_weak(nub::soft_ref<Fr> p)
{
  if (p.is_valid())
    {
      Fr* f = p.get();
      To* t = dynamic_cast<To*>(f);
      if (t == 0)
        return nub::soft_ref<To>(); // return a null soft_ref
      return nub::soft_ref<To>(t);
    }
  return nub::soft_ref<To>(p.id());
}

template <class To, class Fr>
inline void dyn_cast_weak_to_from(nub::soft_ref<To>& dest,
                                  nub::soft_ref<Fr> p)
{
  dest = dyn_cast<To, Fr>(p);
}


template <class To, class Fr>
inline nub::soft_ref<To> dyn_cast_weak(nub::ref<Fr> p)
{
  Fr* f = p.get();
  To* t = dynamic_cast<To*>(f);
  if (t == 0)
    return nub::soft_ref<To>(); // return a null soft_ref
  return nub::soft_ref<To>(t);
}

template <class To, class Fr>
inline void dyn_cast_weak_to_from(nub::soft_ref<To>& dest,
                                  nub::ref<Fr> p)
{
  dest = dyn_cast<To, Fr>(p);
}


// Compatibility interface with alternate function-naming:

template <class To, class Fr>
inline nub::soft_ref<To> dynCast(nub::soft_ref<Fr> p)
{ return dyn_cast<To, Fr>(p); }


template <class To, class Fr>
inline void dynCastToFrom(nub::soft_ref<To>& dest,
                          nub::soft_ref<Fr> p)
{ dyn_cast_to_from<To, Fr>(dest, p); }


template <class To, class Fr>
inline nub::soft_ref<To> dynCastWeak(nub::soft_ref<Fr> p)
{ return dyn_cast_weak<To, Fr>(p); }


template <class To, class Fr>
inline void dynCastWeakToFrom(nub::soft_ref<To>& dest,
                              nub::soft_ref<Fr> p)
{ dyn_cast_weak_to_from<To, Fr>(dest, p); }


template <class To, class Fr>
inline nub::soft_ref<To> dynCastWeak(nub::ref<Fr> p)
{ return dyn_cast_weak<To, Fr>(p); }


template <class To, class Fr>
inline void dynCastWeakToFrom(nub::soft_ref<To>& dest,
                              nub::ref<Fr> p)
{ dyn_cast_weak_to_from<To, Fr>(dest, p); }


///////////////////////////////////////////////////////////
/**
 *
 * nub::floating_ref<T> is a ref-counted smart pointer that will NOT
 * delete its pointee when exiting scope. This can be of use to
 * clients who need to allow an object to be returned to a clean
 * state (i.e. with ref count == 0), but need to ref the object
 * temporarily. Since floating_ref should only be used in restricted
 * scopes, copying and assignment are disallowed.
 *
 **/
///////////////////////////////////////////////////////////

template <class T>
class nub::floating_ref
{
private:

  typedef nub::detail::handle<T, nub::detail::no_delete_unref_policy<T> >
  handle;

  handle m_handle;

  floating_ref(const floating_ref&);
  floating_ref& operator=(const floating_ref&);

public:
  // Default dtor is fine

  explicit floating_ref(T* ptr) : m_handle(ptr) {}

  T* operator->() const throw() { return get(); }
  T& operator*()  const throw() { return *(get()); }

  T* get()        const throw() { return m_handle.get(); }
};




///////////////////////////////////////////////////////////////////////
//
// inline member definitions
//
///////////////////////////////////////////////////////////////////////

template <class T>
template <class U>
inline nub::ref<T>::ref(const soft_ref<U>& other) :
  m_handle(other.get())
{}

static const char __attribute__((used)) vcid_groovx_nub_ref_h_utc20050626084019[] = "$Id: ref.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/ref.h $";
#endif // !GROOVX_NUB_REF_H_UTC20050626084019_DEFINED
