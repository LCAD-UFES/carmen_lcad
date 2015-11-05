/** @file rutz/demangle_cxxfilt.h demangle std::type_info::name() by
    piping through /usr/bin/c++filt */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Feb 27 16:30:08 2003
// commit: $Id: demangle_cxxfilt.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle_cxxfilt.h $
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

#ifndef GROOVX_RUTZ_DEMANGLE_CXXFILT_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_DEMANGLE_CXXFILT_H_UTC20050626084020_DEFINED

#include "rutz/error.h"
#include "rutz/fstring.h"
#include "rutz/pipe.h"
#include "rutz/sfmt.h"

#include <string>

#include "rutz/debug.h"
GVX_DBG_REGISTER
#include "rutz/trace.h"

namespace
{
  std::string demangle_cxxfilt(const std::string& mangled)
  {
    GVX_TRACE("demangle_cxxfilt");

    rutz::shell_pipe pipe(rutz::sfmt("c++filt %s",
                                     mangled.c_str()).c_str(),
                          "r");

    if (pipe.is_closed())
      {
        throw rutz::error(rutz::sfmt("while demangling '%s': "
                                     "couldn't open pipe to c++filt",
                                     mangled.c_str()),
                          SRC_POS);
      }

    std::string demangled;
    std::getline(pipe.stream(), demangled);

    return demangled;
  }
}

static const char __attribute__((used)) vcid_groovx_rutz_demangle_cxxfilt_h_utc20050626084020[] = "$Id: demangle_cxxfilt.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle_cxxfilt.h $";
#endif // !GROOVX_RUTZ_DEMANGLE_CXXFILT_H_UTC20050626084020_DEFINED
