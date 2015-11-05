/** @file rutz/sfmt.h make a rutz::fstring using printf-style
    formatting */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Tue Jul  5 11:02:38 2005
// commit: $Id: sfmt.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/sfmt.h $
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

#ifndef GROOVX_RUTZ_SFMT_H_UTC20050705180238_DEFINED
#define GROOVX_RUTZ_SFMT_H_UTC20050705180238_DEFINED

#include "rutz/fstring.h"

#include <stdarg.h>

namespace rutz
{
  //! snprintf() the specified format string + varargs into a rutz::fstring.
  /*! NOTE: The CALLER is responsible for doing va_end(ap); it is not
      done internally in vsfmt(). */
  rutz::fstring vsfmt(const char* fmt, va_list ap);

  //! snprintf() the specified format string + varargs into a rutz::fstring.
  rutz::fstring sfmt(const char* fmt, ...)
    // NOTE: this __attribute__ tells gcc that it should issue
    // printf-style warnings when compiling calls to sfmt(), treating
    // the 1st argument (fmt) as the format string, and the 2nd and
    // subsequent arguments as the printf-style parameters
    __attribute__((format(__printf__, 1, 2)));
  ;
}

static const char __attribute__((used)) vcid_groovx_rutz_sfmt_h_utc20050705180238[] = "$Id: sfmt.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/sfmt.h $";
#endif // !GROOVX_RUTZ_SFMT_H_UTC20050705180238DEFINED
