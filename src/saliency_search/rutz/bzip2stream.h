/** @file rutz/bzip2stream.h handle bzip2-encoding through a c++ iostreams interface */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Jun 14 15:00:54 2006
// commit: $Id: bzip2stream.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/bzip2stream.h $
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

#ifndef GROOVX_RUTZ_BZIP2STREAM_H_UTC20060614220054_DEFINED
#define GROOVX_RUTZ_BZIP2STREAM_H_UTC20060614220054_DEFINED

#include <ios>
#include <iosfwd>

namespace rutz
{
  class fstring;

  template <class T> class shared_ptr;

  /** Opens a file for writing. An exception will be thrown if the
      specified file cannot be opened. The output file will be
      bzip2-compressed if the filename ends with ".bz2". */
  shared_ptr<std::ostream> obzip2open(const rutz::fstring& filename,
                                      std::ios::openmode flags =
                                      std::ios::openmode(0));

  /// Overload.
  shared_ptr<std::ostream> obzip2open(const char* filename,
                                      std::ios::openmode flags =
                                      std::ios::openmode(0));

  /** Opens a file for reading. An exception will be thrown if the
      specified file cannot be opened. The input file will be
      bzip2-decompressed if the filename ends with ".bz2". */
  shared_ptr<std::istream> ibzip2open(const rutz::fstring& filename,
                                      std::ios::openmode flags =
                                      std::ios::openmode(0));

  /// Overload.
  shared_ptr<std::istream> ibzip2open(const char* filename,
                                      std::ios::openmode flags =
                                      std::ios::openmode(0));
}

static const char vcid_groovx_rutz_bzip2stream_h_utc20060614220054[] = "$Id: bzip2stream.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/bzip2stream.h $";
#endif // !GROOVX_RUTZ_BZIP2STREAM_H_UTC20060614220054DEFINED
