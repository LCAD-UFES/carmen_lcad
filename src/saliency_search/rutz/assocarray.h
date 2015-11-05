/** @file rutz/assocarray.h generic associative arrays, implemented as
    a thin wrapper around std::map<rutz::fstring, void*> */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Oct 14 18:40:34 2004
// commit: $Id: assocarray.h 8274 2007-04-19 17:44:48Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/assocarray.h $
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

#ifndef GROOVX_RUTZ_ASSOCARRAY_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_ASSOCARRAY_H_UTC20050626084020_DEFINED

#include "rutz/fstring.h"

namespace rutz
{
  class file_pos;
  //  class fstring;

  /// A non-typesafe wrapper around std::map<string, void*>.
  /** The use must provide a pointer to a function that knows how to
      properly destroy the actual contained objects according to their
      true type. */
  class assoc_array_base
  {
  public:
    /// Function type for destroying elements.
    typedef void (kill_func_t) (void*);

    /// Default constructor.
    /** @param descr a human-readable description of what this array's
        keys represent; this is used in error messages, e.g. if descr
        is "frobnicator", then error messages would include "unknown
        frobnicator"

        @param nocase true if the array should use case-insensitive
        string comparisons (default is false, giving normal
        case-sensitive string comparisons)
    */
    assoc_array_base(kill_func_t* f, const char* descr, bool nocase);

    /// Virtual destructor.
    ~assoc_array_base();

    /// Get a list of known keys, separated by sep.
    rutz::fstring get_known_keys(const char* sep) const;

    /// Raise an exception reporting an unknown key.
    void throw_for_key(const char* key,
                       const rutz::file_pos& pos) const;

    /// Raise an exception reporting an unknown key.
    void throw_for_key(const rutz::fstring& key,
                       const rutz::file_pos& pos) const;

    /// Retrieve the object associated with the tag \a name.
    void* get_value_for_key(const rutz::fstring& name) const;

    /// Retrieve the object associated with the tag \a name.
    void* get_value_for_key(const char* name) const;

    /// Associate the object at \a ptr with the tag \a name.
    void set_value_for_key(const char* name, void* ptr);

    /// Clear all entries, calling the kill function for each.
    void clear();

  private:
    assoc_array_base(const assoc_array_base&);
    assoc_array_base& operator=(const assoc_array_base&);

    struct impl;
    impl* const rep;
  };


  /// rutz::assoc_array is a typesafe wrapper of rutz::assoc_array_base.

  template<class value_t>
  class assoc_array
  {
  public:
    /// Default constructor
    /** @param descr a human-readable description of what this array's
        keys represent; this is used in error messages, e.g. if descr
        is "frobnicator", then error messages would include "unknown
        frobnicator"

        @param nocase true if the array should use case-insensitive
        string comparisons (default is false, giving normal
        case-sensitive string comparisons)
    */
    assoc_array(const char* descr, bool nocase = false)
      : base(&delete_ptr, descr, nocase) {}

    /// Get a string listing of known keys, separated by the string sep.
    rutz::fstring get_known_keys(const char* sep) const
    { return base.get_known_keys(sep); }

    /// Raise an exception reporting an unknown key.
    void throw_for_key(const char* key,
                       const rutz::file_pos& pos) const
    { base.throw_for_key(key, pos); }

    /// Raise an exception reporting an unknown key.
    void throw_for_key(const rutz::fstring& key,
                       const rutz::file_pos& pos) const
    { base.throw_for_key(key, pos); }

    /// Get the object associated with the given key.
    value_t* get_ptr_for_key(const rutz::fstring& key) const
    { return static_cast<value_t*>(base.get_value_for_key(key)); }

    /// Get the object associated with the given key.
    value_t* get_ptr_for_key(const char* key) const
    { return static_cast<value_t*>(base.get_value_for_key(key)); }

    /// Associate the object at \a ptr with the given key.
    void set_ptr_for_key(const char* key, value_t* ptr)
    { base.set_value_for_key(key, static_cast<void*>(ptr)); }

  private:
    rutz::assoc_array_base base;

    static void delete_ptr(void* ptr)
    { delete static_cast<value_t*>(ptr); }
  };
}

static const char __attribute__((used)) vcid_groovx_rutz_assocarray_h_utc20050626084020[] = "$Id: assocarray.h 8274 2007-04-19 17:44:48Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/assocarray.h $";
#endif // !GROOVX_RUTZ_ASSOCARRAY_H_UTC20050626084020_DEFINED
