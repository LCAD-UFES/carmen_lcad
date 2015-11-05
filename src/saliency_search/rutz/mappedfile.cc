/** @file rutz/mappedfile.cc c++ wrapper for mmap() for fast file i/o */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Oct  8 14:11:31 2004
// commit: $Id: mappedfile.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/mappedfile.cc $
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

#ifndef GROOVX_RUTZ_MAPPEDFILE_CC_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_MAPPEDFILE_CC_UTC20050626084020_DEFINED

#include "mappedfile.h"

#include "rutz/error.h"
#include "rutz/sfmt.h"

#include <cerrno>
#include <cstring>     // for strerror()
#include <fcntl.h>     // for open(), O_RDONLY
#include <sys/mman.h>  // for mmap()
#include <unistd.h>    // for close()

rutz::mapped_infile::mapped_infile(const char* filename)
  :
  m_statbuf(),
  m_fileno(0),
  m_mem(0)
{
  errno = 0;

  if (stat(filename, &m_statbuf) == -1)
    {
      throw rutz::error(rutz::sfmt("stat() failed for file '%s':\n"
                                   "%s\n", filename, strerror(errno)),
                        SRC_POS);
    }

  m_fileno = open(filename, O_RDONLY);
  if (m_fileno == -1)
    {
      throw rutz::error(rutz::sfmt("open() failed for file '%s':\n"
                                   "%s\n", filename, strerror(errno)),
                        SRC_POS);
    }

  m_mem = mmap(0, m_statbuf.st_size,
               PROT_READ, MAP_PRIVATE, m_fileno, 0);

  if (m_mem == (void*)-1)
    {
      throw rutz::error(rutz::sfmt("mmap() failed for file '%s':\n"
                                   "%s\n", filename, strerror(errno)),
                        SRC_POS);
    }
}

rutz::mapped_infile::~mapped_infile()
{
  munmap(m_mem, m_statbuf.st_size);
  close(m_fileno);
}

static const char __attribute__((used)) vcid_groovx_rutz_mappedfile_cc_utc20050626084020[] = "$Id: mappedfile.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/mappedfile.cc $";
#endif // !GROOVX_RUTZ_MAPPEDFILE_CC_UTC20050626084020_DEFINED
