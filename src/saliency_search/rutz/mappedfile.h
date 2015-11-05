/** @file rutz/mappedfile.h c++ wrapper for mmap() for fast file i/o */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Oct  8 13:50:14 2004
// commit: $Id: mappedfile.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/mappedfile.h $
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

#ifndef GROOVX_RUTZ_MAPPEDFILE_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_MAPPEDFILE_H_UTC20050626084020_DEFINED

#include <sys/stat.h>  // for stat()

namespace rutz
{
  //! An mmap()/munmap() wrapper class for fast input file reading.
  /*! In cases where an entire file would eventually be read into
      memory anyway, it is likely to be significantly faster (2-3x
      faster at least) to use mmap-ing instead of C-style
      fopen()/fgetc()/fread()/fclose() or C++-style iostreams. This is
      because both the C and C++ libraries are likely implemented
      internally with mmap anyway, and those libraries introduce
      another layer of memory buffering on top of the raw OS mmap. */
  class mapped_infile
  {
  public:
    //! Open the named file using mmap().
    /*! The contents of the file become accessible as if the file were
        laid out continously in memory. */
    mapped_infile(const char* filename);

    //! Destructor closes and munmap()'s the file.
    ~mapped_infile();

    //! Get a pointer to the memory representing the file contents.
    const void* memory() const { return m_mem; }

    //! Get the length of the file, and of its in-memory representation.
    off_t length() const { return m_statbuf.st_size; }

    //! Get the last-modification time of the file.
    time_t mtime() const { return m_statbuf.st_mtime; }

  private:
    mapped_infile(const mapped_infile&);
    mapped_infile& operator=(const mapped_infile&);

    struct stat m_statbuf;
    int         m_fileno;
    void*       m_mem;
  };
}

static const char __attribute__((used)) vcid_groovx_rutz_mappedfile_h_utc20050626084020[] = "$Id: mappedfile.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/mappedfile.h $";
#endif // !GROOVX_RUTZ_MAPPEDFILE_H_UTC20050626084020_DEFINED
