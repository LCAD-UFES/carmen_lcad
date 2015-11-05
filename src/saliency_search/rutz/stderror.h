/** @file rutz/stderror.h throw common exceptions with standardized
    messages */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2003-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Sat May 24 13:00:28 2003
// commit: $Id: stderror.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/stderror.h $
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

#ifndef GROOVX_RUTZ_STDERROR_H_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_STDERROR_H_UTC20050626084019_DEFINED

#include <typeinfo>

// A common interface for throwing some exceptions with standardized
// error messages.

namespace rutz
{
  class file_pos;

  void throw_bad_cast(const std::type_info& to,
                      const std::type_info& from,
                      const rutz::file_pos& pos);
}

static const char __attribute__((used)) vcid_groovx_rutz_stderror_h_utc20050626084019[] = "$Id: stderror.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/stderror.h $";
#endif // !GROOVX_RUTZ_STDERROR_H_UTC20050626084019_DEFINED
