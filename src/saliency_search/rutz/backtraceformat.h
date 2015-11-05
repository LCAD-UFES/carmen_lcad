/** @file rutz/backtraceformat.h generate a human-readable string from
    a rutz::backtrace object */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Thu Jun 30 14:39:14 2005
// commit: $Id: backtraceformat.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/backtraceformat.h $
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

#ifndef GROOVX_RUTZ_BACKTRACEFORMAT_H_UTC20050630213914_DEFINED
#define GROOVX_RUTZ_BACKTRACEFORMAT_H_UTC20050630213914_DEFINED

namespace rutz
{
  class backtrace;
  class fstring;

  /// Generate a human-readable string representation of the backtrace.
  /** Note: this function is not part of rutz::backtrace's interface
      so that rutz::backtrace doesn't have to depend on rutz::fstring,
      in order to break cyclic dependencies. In any case, this
      function's implementation doesn't need access to
      rutz::backtrace's privates. */
  rutz::fstring format(const rutz::backtrace& bt);
}

static const char __attribute__((used)) vcid_groovx_rutz_backtraceformat_h_utc20050630213914[] = "$Id: backtraceformat.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/backtraceformat.h $";
#endif // !GROOVX_RUTZ_BACKTRACEFORMAT_H_UTC20050630213914DEFINED
