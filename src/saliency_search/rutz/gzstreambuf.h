/** @file rutz/gzstreambuf.h handle gzip-encoding through a c++
    iostreams interface */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Jun 20 09:12:51 2001
// commit: $Id: gzstreambuf.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/gzstreambuf.h $
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

#ifndef GROOVX_RUTZ_GZSTREAMBUF_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_GZSTREAMBUF_H_UTC20050626084020_DEFINED

#include <istream>
#include <ostream>
#include <streambuf>
#include <zlib.h>

namespace rutz
{
  class fstring;

  template <class T> class shared_ptr;

  /// A std::streambuf implementation that handles gzip-encoded data.
  class gzstreambuf : public std::streambuf
  {
  private:
    bool m_opened;
    int m_mode;
    gzFile m_gzfile;

    gzstreambuf(const gzstreambuf&);
    gzstreambuf& operator=(const gzstreambuf&);

    static const int s_buf_size = 4092;
    static const int s_pback_size = 4;
    char m_buf[s_buf_size];

    int flushoutput();

  public:
    gzstreambuf(const char* name, int om, bool throw_exception=false);
    ~gzstreambuf() { close(); }

    bool is_open() { return m_opened; }

    void ensure_open();

    void close();

    virtual int underflow();

    virtual int overflow(int c);

    virtual int sync();
  };

  /** Opens a file for writing. An exception will be thrown if the
      specified file cannot be opened. The output file will be
      gz-compressed if the filename ends with ".gz". */
  shared_ptr<std::ostream> ogzopen(const rutz::fstring& filename,
                                   std::ios::openmode flags =
                                   std::ios::openmode(0));

  /// Overload.
  shared_ptr<std::ostream> ogzopen(const char* filename,
                                   std::ios::openmode flags =
                                   std::ios::openmode(0));

  /** Opens a file for reading. An exception will be thrown if the
      specified file cannot be opened. If the file is gz-compressed,
      this will be automagically detected regardless of the filename
      extension. */
  shared_ptr<std::istream> igzopen(const rutz::fstring& filename,
                                   std::ios::openmode flags =
                                   std::ios::openmode(0));

  /// Overload.
  shared_ptr<std::istream> igzopen(const char* filename,
                                   std::ios::openmode flags =
                                   std::ios::openmode(0));
}

static const char __attribute__((used)) vcid_groovx_rutz_gzstreambuf_h_utc20050626084020[] = "$Id: gzstreambuf.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/gzstreambuf.h $";
#endif // !GROOVX_RUTZ_GZSTREAMBUF_H_UTC20050626084020_DEFINED
