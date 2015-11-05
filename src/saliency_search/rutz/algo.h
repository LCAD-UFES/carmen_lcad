/** @file rutz/algo.h some trivial algos from <algorithm>, but much
    more lightweight */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Sun Jul 22 23:31:48 2001
// commit: $Id: algo.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/algo.h $
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

#ifndef GROOVX_RUTZ_ALGO_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_ALGO_H_UTC20050626084020_DEFINED

namespace rutz
{
  template <class T>
  inline const T& min(const T& a, const T& b)
  {
    /* return (a < b) ? a : b; */ // This triggers warnings in some compilers :(
    if (a < b) return a; return b;
  }

  template <class T>
  inline const T& max(const T& a, const T& b)
  {
    /* return (a > b) ? a : b; */ // This triggers warnings in some compilers :(
    if (a > b) return a; return b;
  }

  template <class T>
  inline T abs(const T& val)
  {
    return (val < 0) ? -val : val;
  }

  template <class T>
  inline T clamp(const T& val, const T& lower, const T& upper)
  {
    return rutz::max(lower, rutz::min(upper, val));
  }

  template <class T>
  inline void swap2(T& t1, T& t2)
  {
    T t2_copy = t2;
    t2 = t1;
    t1 = t2_copy;
  }
}

static const char __attribute__((used)) vcid_groovx_rutz_algo_h_utc20050626084020[] = "$Id: algo.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/algo.h $";
#endif // !GROOVX_RUTZ_ALGO_H_UTC20050626084020_DEFINED
