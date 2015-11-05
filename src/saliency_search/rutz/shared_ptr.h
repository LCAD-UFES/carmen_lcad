/** @file rutz/shared_ptr.h A thread-safe shared pointer class */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Aug 16 14:39:14 2006
// commit: $Id: shared_ptr.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/shared_ptr.h $
//
// --------------------------------------------------------------------
//
// This file is part of GroovX.
//   [http://www.klab.caltech.edu/rjpeters/groovx/]
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

#ifndef GROOVX_RUTZ_SHARED_PTR_H_UTC20070412044942_DEFINED
#define GROOVX_RUTZ_SHARED_PTR_H_UTC20070412044942_DEFINED

#include "rutz/atomic.h"
#include "rutz/traits.h"

/// Auxiliary helper namespace used in implementing shared_ptr.
namespace rutz
{
  namespace shared_ptr_aux
  {
    /// Function type for checking whether a given pointer is allowed to be used in a shared_ptr.
    typedef void (ptr_check_function)(const void*);

    /// Install a particular pointer-checking function.
    /** The ptr_check_function is expected to abort or throw an
        exception if it is passed a pointer that it considers invalid,
        otherwise it does nothing. set_check_function() returns a
        pointer to the previously-installed checking function, so that
        it is possible to save and later restore the previous
        value. */
    ptr_check_function* set_check_function(ptr_check_function* func);

    /// Call the current ptr_check_function, if it is non-null, otherwise do nothing.
    void check_ptr(const void* p);
  }

  template <class T> class shared_ptr;
}

//! A thread-safe smart pointer with reference counted copy semantics.
/*! The object pointed to is deleted when the last shared_ptr pointing
    to it is destroyed or reset. Take note of the WARNING given in the
    documentation for the constructor that takes a raw pointer as its
    argument.

    Borrowed and modified from boost.org smart_ptr.hpp
    Original copyright notice:

     (C) Copyright Greg Colvin and Beman Dawes 1998, 1999. Permission
     to copy, use, modify, sell and distribute this software is
     granted provided this copyright notice appears in all
     copies. This software is provided "as is" without express or
     implied warranty, and with no claim as to its suitability for any
     purpose.
 */
template<class T>
class rutz::shared_ptr
{
public:
  //! Construct from a raw pointer. WARNING: do this only ONCE per pointer!
  /*! Bad Things (TM) will happen if this constructor is used multiple
      times to make multiple shared_ptr's from a single raw pointer;
      that's because each shared_ptr will start off thinking that it
      is the unique owner, and so they will each start their own ref
      count. If multiple shared_ptr's are needed for the same object
      (after all, that's the whole point of a shared ptr), only the
      first one should be constructed from the raw pointer, and
      subsequent ones should be constructed from the first using the
      copy constructor or assignment operator. */
  explicit inline shared_ptr(T* p =0);

  //! Copy another shared_ptr. The ref count is incremented by one.
  inline shared_ptr(const shared_ptr& r) throw();

  //! Destructor decrements the ref count by one.
  /*! If we were the last pointer to the pointee, then the pointee and
      the reference count are both delete'd. */
  inline ~shared_ptr();

  //! Assign from another shared_ptr. The ref count is incremented by one.
  inline shared_ptr& operator=(const shared_ptr& r);

  //! Copy construct from a shared_ptr of a different type.
  /*! The other type TT must be related to T by an appropriate
      inheritance relationship or by a change in
      const-qualification. */
  template<class TT>
  inline shared_ptr(const shared_ptr<TT>& r) throw();

  //! Assign from a shared_ptr of a different type.
  /*! The other type TT must be related to T by an appropriate
      inheritance relationship or by a change in
      const-qualification. */
  template<class TT>
  inline shared_ptr& operator=(const shared_ptr<TT>& r);

  //! Assign from a shared_ptr of a different type, with dynamic casting.
  /*! This is the shared_ptr equivalent of doing a dynamic_cast
      between two raw pointers. Doing 'shared_ptr<T> ptr;
      ptr.dynCastFrom(other);' with 'other' a shared_ptr<TT> will
      internally do a dynamic_cast from TT* to T* and will assign the
      result to ptr. If the dynamic_cast succeeds, ptr and other will
      share the pointee as if they were two normal shared_ptr onto it
      (even though ptr and other are shared_ptr of different
      types). If the dynamic_cast fails then ptr will point to
      NULL. Typical usage of this is to recover an expected derived
      class from a base class: for example, <PRE>

      // get our visual cortex from Brain. Brain provides a getVC()
      // function which returns a shared_ptr onto the VisualCortex
      // base class no matter which derivation of that we are actually
      // running right now:
      shared_ptr<VisualCortex> vcx = brain->getVC();

      // do something special if our VisualCortex actually happens to
      // be of type VisualCortexEyeMvt which is derived from
      // VisualCortex:
      shared_ptr<VisualCortexEyeMvt> vcem;
      vcem.dynCastFrom(vcx);

      // vcem points to NULL unless the pointee of vcx could be
      // dynamic_cast'ed to vcem's type.
      if (vcem.isValid()) {
      // yes, indeed we are running a VisualCortexEyeMvt
      ...
      }
      </PRE>

      Note: make sure you always use virtual destructors in your
      pointees, especially when you use dynCastFrom. Indeed, whichever
      shared_ptr is the last one to run out of scope will destroy the
      pointee. */
  template <class TT>
  inline shared_ptr& dyn_cast_from(const shared_ptr<TT>& r);

  template <class TT>
  inline shared_ptr& dynCastFrom(const shared_ptr<TT>& r)
  { return this->dyn_cast_from(r); }

  //! Make the shared_ptr point to a different (optionally null) pointee.
  inline void reset(T* p=0);

  //! Get a reference to the pointee.
  inline T& operator*() const throw() { return *px; }

  //! Get the pointee for accessing its members.
  inline T* operator->() const throw() { return px; }

  //! Get the pointee.
  inline T* get() const throw() { return px; }

  //! Query whether the pointee is non-null.
  bool is_valid() const throw() { return px != 0; }

  //! Query whether the pointee is non-null.
  bool is_invalid() const throw() { return px == 0; }

  //! Query how many shared_ptr's are sharing the pointee.
  inline int use_count() const throw() { return pn->atomic_get(); }

  //! Query whether the shared_ptr is the unique owner of its pointee.
  inline bool unique() const throw() { return use_count() == 1; }

  //! Swap the pointees of two shared_ptr's.
  inline void swap(shared_ptr<T>& that) throw();

private:
  T*                   px; // pointee
  rutz::atomic_int_t*  pn; // reference count with atomic incr/decr operations

  template<class TT> friend class shared_ptr;
};



// ######################################################################
// ######################################################################
// ##### INLINED FREE FUNCTIONS:
// ######################################################################
// ######################################################################

namespace rutz
{

  //! Test whether two shared_ptr's point to the same object.
  template<class T, class U>
  inline bool operator==(const shared_ptr<T>& a, const shared_ptr<U>& b)
  {
    return a.get() == b.get();
  }

  //! Test whether two shared_ptr's point to different objects.
  template<class T, class U>
  inline bool operator!=(const shared_ptr<T>& a, const shared_ptr<U>& b)
  {
    return a.get() != b.get();
  }

  //! A convenience function for making a shared_ptr out of a raw pointer.
  template <class T>
  inline shared_ptr<T> make_shared(T* t) { return shared_ptr<T>(t); }

  //! Do a dynamic cast on a shared ptr
  template <class Dst, class Src>
  inline shared_ptr<Dst> dyn_cast(const shared_ptr<Src>& src)
  {
    shared_ptr<Dst> dst;
    dst.dyn_cast_from(src);
    return dst;
  }

  //! Do a dynamic cast on a shared ptr
  template <class Dst, class Src>
  inline void dyn_cast_to_from(shared_ptr<Dst>& dst, const shared_ptr<Src>& src)
  {
    dst.dyn_cast_from(src);
  }

  //! Synonym for dyn_cast
  template <class Dst, class Src>
  inline shared_ptr<Dst> dynCast(const shared_ptr<Src>& src)
  { return dyn_cast<Dst,Src>(src); }

  //! Synonym for dyn_cast_to_from
  template <class Dst, class Src>
  inline void dynCastToFrom(shared_ptr<Dst>& dst, const shared_ptr<Src>& src)
  { dyn_cast_to_from<Dst,Src>(dst, src); }

}

// ######################################################################
// ######################################################################
// ##### INLINED MEMBER FUNCTIONS:
// ######################################################################
// ######################################################################

// ######################################################################
template <class T> inline
rutz::shared_ptr<T>::shared_ptr(T* p) :
  px(p), pn(0)
{
#if defined(GVX_MEM_DEBUG)
  // Only call check_ptr() if GVX_MEM_DEBUG has been defined, because
  // this is a relatively expensive operation and can bog things down
  // if many shared_ptr objects are being created/destroyed in an
  // inner loop.

  // We use full_object_cast() to get the address of the beginning of
  // the full object. That is slightly non-trivial, because, depending
  // on whether or not T is a polymorphic type, we might either have
  // to use a static_cast<void*> or a dynamic_cast<void*> to get the
  // full object address. That is all encapsulated by
  // rutz::full_object_cast().
  rutz::shared_ptr_aux::check_ptr(rutz::full_object_cast(p));
#endif

  // prevent leak if new throws:
  try { pn = new rutz::atomic_int_t; pn->atomic_set(1); }
  catch (...) { delete p; throw; }
}

// ######################################################################
template <class T> inline
rutz::shared_ptr<T>::shared_ptr(const shared_ptr<T>& r) throw() :
  px(r.px), pn(r.pn)
{
  pn->atomic_incr();
}

// ######################################################################
template <class T> inline
rutz::shared_ptr<T>::~shared_ptr()
{
  if (pn->atomic_decr_test_zero())
    {
      delete px; px = 0;
      delete pn; pn = 0;
    }
}

// ######################################################################
template <class T> inline
rutz::shared_ptr<T>&
rutz::shared_ptr<T>::operator=(const rutz::shared_ptr<T>& r)
{
  shared_ptr(r).swap(*this);
  return *this;
}

// ######################################################################
template <class T>
template<class TT> inline
rutz::shared_ptr<T>::shared_ptr(const rutz::shared_ptr<TT>& r) throw() :
  px(r.px), pn(r.pn)
{
  pn->atomic_incr();
}

// ######################################################################
template <class T>
template<class TT> inline
rutz::shared_ptr<T>&
rutz::shared_ptr<T>::operator=(const rutz::shared_ptr<TT>& r)
{
  shared_ptr(r).swap(*this);
  return *this;
}

// ######################################################################
template <class T>
template<class TT> inline
rutz::shared_ptr<T>&
rutz::shared_ptr<T>::dyn_cast_from(const rutz::shared_ptr<TT>& r)
{
  // if we are already initialized to something (and we always are),
  // simulate a destroy:
  if (pn->atomic_decr_test_zero())
    {
      delete px; px = 0;
      delete pn; pn = 0;
    }

  // first do the dynamic_cast so we can test whether it succeeded:
  T* const new_px = dynamic_cast<T*>(r.px);

  if (new_px == 0 && r.px != 0)
    {
      // the cast failed, so we set up a new ref count (NOTE: we DON'T
      // want to share the ref count in this case, because the
      // original shared_ptr and this shared_ptr will be pointing to
      // DIFFERENT objects -- the original has a valid pointer but we
      // have a null pointer):
      pn = new rutz::atomic_int_t; pn->atomic_set(1);
    }
  else
    {
      // share the ref count without increasing it:
      pn = r.pn;

      // increase it to account for our existence:
      pn->atomic_incr();

      // share ownership of the original pointee:
      px = new_px;
    }
  return *this;
}

// ######################################################################
template <class T> inline
void rutz::shared_ptr<T>::reset(T* p)
{
  shared_ptr(p).swap(*this);
}

// ######################################################################
template <class T> inline
void rutz::shared_ptr<T>::swap(shared_ptr<T>& that) throw()
{
  T* that_px = that.px; that.px = this->px; this->px = that_px;
  rutz::atomic_int_t* that_pn = that.pn; that.pn = this->pn; this->pn = that_pn;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

static const char __attribute__((used)) vcid_groovx_rutz_shared_ptr_h_utc20070412044942[] = "$Id: shared_ptr.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/shared_ptr.h $";
#endif // !GROOVX_RUTZ_SHARED_PTR_H_UTC20070412044942DEFINED
