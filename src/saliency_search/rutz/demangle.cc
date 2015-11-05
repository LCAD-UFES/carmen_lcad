/** @file rutz/demangle.cc get a demangled name from a std::type_info
    object */
///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Oct 13 10:41:19 1999
// commit: $Id: demangle.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle.cc $
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

#ifndef GROOVX_RUTZ_DEMANGLE_CC_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_DEMANGLE_CC_UTC20050626084020_DEFINED

#include "rutz/demangle.h"

#if defined(GVX_NO_TYPENAME_MANGLING)

namespace
{
  const char* demangle_impl(const char* in) { return in; }
}

#  define DEMANGLE_IMPL demangle_impl

#else // !defined(GVX_NO_TYPENAME_MANGLING)

// Here we pick the appropriate system-dependent header that defines a
// function for demangling mangled names with the following prototype:

//   std::string demangle_impl(const std::string& mangled)

#  if defined(HAVE_CXXABI_H)
#    include "rutz/demangle_gcc_v3.h"
#    define DEMANGLE_IMPL demangle_gcc_v3
#  elif defined(__GNUC__) && __GNUC__ < 3
#    include "rutz/demangle_gcc_v2.h"
#    define DEMANGLE_IMPL demangle_gcc_v2
#  elif defined(GVX_HAVE_PROG_CXXFILT)
#    include "rutz/demangle_cxxfilt.h"
#    define DEMANGLE_IMPL demangle_cxxfilt
#  else
// use the cxxabi demangler by default
#    include "rutz/demangle_gcc_v3.h"
#    define DEMANGLE_IMPL demangle_gcc_v3
#  endif
#endif // !defined(GVX_NO_TYPENAME_MANGLING)

#include "rutz/mutex.h"
#include "rutz/trace.h"

#include <map>
#include <pthread.h>
#include <string>

namespace
{
  // why can't we make this a map<type_info, string>?
  //   (1) gcc libstdc++ doesn't seem to have type_info::operator<()
  //   (2) gcc libstdc++ doesn't allow copying of type_info objects
  typedef std::map<std::string, std::string> cache_type;
  cache_type* g_name_cache = 0;
  pthread_once_t g_name_cache_init_once = PTHREAD_ONCE_INIT;
  pthread_mutex_t g_name_cache_mutex = PTHREAD_MUTEX_INITIALIZER;

  void name_cache_init()
  {
    GVX_ASSERT(g_name_cache == 0);
    g_name_cache = new cache_type;
  }
}

const char* rutz::demangled_name(const std::type_info& info)
{
GVX_TRACE("rutz::demangled_name");

  pthread_once(&g_name_cache_init_once, &name_cache_init);
  GVX_ASSERT(g_name_cache != 0);

  const std::string mangled = info.name();

  GVX_MUTEX_LOCK(&g_name_cache_mutex);

  cache_type::iterator itr = g_name_cache->find(mangled);

  if (itr != g_name_cache->end())
    {
      return (*itr).second.c_str();
    }

  const std::string demangled = DEMANGLE_IMPL(info.name());

  std::pair<cache_type::iterator, bool> result =
    g_name_cache->insert(cache_type::value_type(mangled, demangled));

  GVX_ASSERT(result.second == true);

  return (*result.first).second.c_str();
}

static const char __attribute__((used)) vcid_groovx_rutz_demangle_cc_utc20050626084020[] = "$Id: demangle.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle.cc $";
#endif // !GROOVX_RUTZ_DEMANGLE_CC_UTC20050626084020_DEFINED
