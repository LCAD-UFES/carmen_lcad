/** @file rutz/assocarray.cc generic associative arrays, implemented as
    a thin wrapper around std::map<rutz::fstring, void*> */
///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Oct 14 18:42:05 2004
// commit: $Id: assocarray.cc 8274 2007-04-19 17:44:48Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/assocarray.cc $
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

#ifndef GROOVX_RUTZ_ASSOCARRAY_CC_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_ASSOCARRAY_CC_UTC20050626084020_DEFINED

#include "assocarray.h"

#include "rutz/error.h"
#include "rutz/fstring.h"
#include "rutz/sfmt.h"

#include <algorithm> // for lexicographical_compare()
#include <map>
#include <sstream>

#include "rutz/trace.h"

struct rutz::assoc_array_base::impl
{
  impl(kill_func_t* f, const char* descr, bool nocase)
    :
    values(fstring_cmp(nocase)),
    kill_func(f),
    key_description(descr)
  {}

  static bool nocase_char_cmp(char c1, char c2)
  {
    return toupper(c1) < toupper(c2);
  }

  struct fstring_cmp
  {
    fstring_cmp(bool nocase_) : nocase(nocase_) {}

    const bool nocase;

    bool operator()(const rutz::fstring& s1,
                    const rutz::fstring& s2) const
    {
      if (nocase)
        {
          return std::lexicographical_compare
            (s1.c_str(), s1.c_str() + s1.length(),
             s2.c_str(), s2.c_str() + s2.length(),
             nocase_char_cmp);
        }

      // else...
      return (s1 < s2);
    }
  };

  typedef std::map<rutz::fstring, void*, fstring_cmp> map_t;

  map_t         values;
  kill_func_t*  kill_func;
  rutz::fstring key_description;
};

rutz::assoc_array_base::assoc_array_base(kill_func_t* f,
                                         const char* descr,
                                         bool nocase) :
  rep(new impl(f, descr, nocase))
{
GVX_TRACE("rutz::assoc_array_base::assoc_array_base");
}

rutz::assoc_array_base::~assoc_array_base()
{
GVX_TRACE("rutz::assoc_array_base::~assoc_array_base");
  clear();
}

rutz::fstring rutz::assoc_array_base::
get_known_keys(const char* sep) const
{
  std::ostringstream result;

  bool first = true;

  for (impl::map_t::iterator ii = rep->values.begin();
       ii != rep->values.end();
       ++ii)
    {
      if (ii->second != 0)
        {
          if (!first) result << sep;

          result << ii->first;

          first = false;
        }
    }

  return rutz::fstring(result.str().c_str());
}

void rutz::assoc_array_base::
throw_for_key(const char* key, const rutz::file_pos& pos) const
{
  throw rutz::error(rutz::sfmt("known keys are:\n\t%s\nunknown %s '%s'",
                               get_known_keys("\n\t").c_str(),
                               rep->key_description.c_str(),
                               key),
                    pos);
}

void rutz::assoc_array_base::
throw_for_key(const rutz::fstring& key, const rutz::file_pos& pos) const
{
  throw_for_key(key.c_str(), pos);
}

void rutz::assoc_array_base::clear()
{
GVX_TRACE("rutz::assoc_array_base::clear");
  for (impl::map_t::iterator ii = rep->values.begin();
       ii != rep->values.end();
       ++ii)
    {
      if (rep->kill_func != 0) rep->kill_func(ii->second);
      ii->second = 0;
    }

  delete rep;
}

void* rutz::assoc_array_base::get_value_for_key(const rutz::fstring& key) const
{
GVX_TRACE("rutz::assoc_array_base::get_value_for_key");
  return rep->values[key];
}

void* rutz::assoc_array_base::get_value_for_key(const char* key) const
{
  return get_value_for_key(rutz::fstring(key));
}

void rutz::assoc_array_base::set_value_for_key(const char* key, void* ptr)
{
GVX_TRACE("rutz::assoc_array_base::set_value_for_key");
  rutz::fstring skey(key);
  void*& ptr_slot = rep->values[skey];
  if (rep->kill_func != 0) rep->kill_func(ptr_slot);
  ptr_slot = ptr;
}

static const char __attribute__((used)) vcid_groovx_rutz_assocarray_cc_utc20050626084020[] = "$Id: assocarray.cc 8274 2007-04-19 17:44:48Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/assocarray.cc $";
#endif // !GROOVX_RUTZ_ASSOCARRAY_CC_UTC20050626084020_DEFINED
