/** @file nub/objdb.h singleton repository that associates each
    nub::object with its nub::uid, so that objects can be looked up by
    id -- this provides the foundation for using uids as object
    handles in a scripting language */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Sun Nov 21 00:26:29 1999
// commit: $Id: objdb.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objdb.h $
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

#ifndef GROOVX_NUB_OBJDB_H_UTC20050626084019_DEFINED
#define GROOVX_NUB_OBJDB_H_UTC20050626084019_DEFINED

#include "nub/uid.h"

#include "rutz/error.h"
#include "rutz/iter.h"

namespace nub
{
  class object;
  class objectdb;
  class invalid_uid_error;
}

/**
 *
 * nub::invalid_uid_error is an exception class that will be thrown from
 * objectdb if an attempt to use an invalid id is made in a checked
 * function.
 *
 **/

class nub::invalid_uid_error : public rutz::error
{
public:
  /// Constructor.
  invalid_uid_error(nub::uid id, const rutz::file_pos& pos);

  /// Virtual destructor.
  virtual ~invalid_uid_error() throw();
};

///////////////////////////////////////////////////////////////////////
/**
 *
 * objectdb is a database for storing nub::object objects, which can
 * be accessed by their nub::uid values. Most clients will not need to
 * use the objectdb directly, but can instead use the nub::ref and
 * nub::soft_ref smart pointers.
 *
 **/
///////////////////////////////////////////////////////////////////////

class nub::objectdb
{
protected:
  /// Default constructor makes an empty list.
  objectdb();

public:
  /// Virtual destructor.
  virtual ~objectdb();

  /// Returns the singleton instance of objectdb.
  static objectdb& instance();

  class impl;

  //
  // Iterators
  //

  typedef rutz::fwd_iter<object* const> iterator;

  iterator objects() const;

  /// A filtering iterator class; only exposes objects matching a given type.
  template <class T>
  class casting_iterator
  {
    iterator m_itr;

    void advance_to_valid()
    {
      while (!m_itr.at_end() && (dynamic_cast<T*>(*m_itr)==0))
        ++m_itr;
    }

  public:
    casting_iterator(const iterator& begin) : m_itr(begin)
    { advance_to_valid(); }

    casting_iterator& operator++() { ++m_itr; advance_to_valid(); return *this; }

    bool at_end() const { return m_itr.at_end(); }
    bool is_valid() const { return m_itr.is_valid(); }

    T* operator*() const { return &(dynamic_cast<T&>(**m_itr)); }

    T* operator->() const { return operator*(); }
  };

  //
  // Collection interface
  //

  /// Returns the number of valid objects in the database.
  int count() const throw();

  /// Returns true if 'id' is a valid uid.
  bool is_valid_uid(nub::uid id) const throw();

  /// Releases the object specified by \a id, but only if it is unshared.
  /** This causes the object to be destroyed since it was unshared. If
      the object is shared, this operation throws an exception. */
  void remove(nub::uid id);

  /// Removes reference to the object with uid \a id.
  void release(nub::uid id);

  /// Releases all unshared objects held in the database.
  /** Since the objects are unshared, they will be destroyed in the
      process. */
  void purge();

  /// Calls \c purge() repeatedly until no more items can be removed.
  /** This will get rid of items that were only referenced by other
      items in the list. */
  void clear();

  /// WARNING: should only be called during program exit.
  /** Does a full clear of all objects held by the objectdb. This breaks
      the usual semantics of objectdb, since it removes both shared and
      unshared objects. */
  void clear_on_exit();

  /// Return the \c nub::object* with the uid given by \a id.
  /** Checks first if \a id is a valid uid, and throws an \c
      nub::invalid_uid_error if it is not. */
  nub::object* get_checked_obj(nub::uid id) throw (nub::invalid_uid_error);

  /// Insert a strong reference to obj into the database.
  void insert_obj(nub::object* obj);

  /// Insert a weak reference to obj into the database.
  void insert_obj_weak(nub::object* obj);

private:
  objectdb(const objectdb&);
  objectdb& operator=(const objectdb&);

  impl* const rep;
};

static const char __attribute__((used)) vcid_groovx_nub_objdb_h_utc20050626084019[] = "$Id: objdb.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objdb.h $";
#endif // !GROOVX_NUB_OBJDB_H_UTC20050626084019_DEFINED
