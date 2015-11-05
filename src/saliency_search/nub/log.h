/** @file nub/log.h functions for hierarchical logging */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Jun 20 17:47:13 2001
// commit: $Id: log.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/log.h $
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

#ifndef GROOVX_NUB_LOG_H_UTC20050626084019_DEFINED
#define GROOVX_NUB_LOG_H_UTC20050626084019_DEFINED

namespace rutz
{
  class fstring;
}

namespace nub
{
  class object;

  /// Functions for hierarchical logging.
  namespace logging
  {
    void reset();

    void add_scope(const rutz::fstring& name);
    void remove_scope(const rutz::fstring& name);

    /// Add a scope named after the given object's type + id.
    void add_obj_scope(const nub::object& obj);

    /// Remove the scope named after the given object's type + id.
    void remove_obj_scope(const nub::object& obj);

    /// Specify the name of a file to which log info should be appended.
    void set_log_filename(const rutz::fstring& filename);

    /// Specify whether to copy log output to stdout (default = yes).
    void copy_to_stdout(bool shouldcopy);
  }

  void log(const char* msg);
  void log(const rutz::fstring& msg);
}

static const char __attribute__((used)) vcid_groovx_nub_log_h_utc20050626084019[] = "$Id: log.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/log.h $";
#endif // !GROOVX_NUB_LOG_H_UTC20050626084019_DEFINED
