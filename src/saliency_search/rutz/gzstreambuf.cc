/** @file rutz/gzstreambuf.cc handle gzip-encoding through a c++
    iostreams interface */
///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Jul 20 13:13:22 2001
// commit: $Id: gzstreambuf.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/gzstreambuf.cc $
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

#ifndef GROOVX_RUTZ_GZSTREAMBUF_CC_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_GZSTREAMBUF_CC_UTC20050626084020_DEFINED

#include "rutz/gzstreambuf.h"

#include "rutz/error.h"
#include "rutz/fstring.h"
#include "rutz/sfmt.h"
#include "rutz/shared_ptr.h"

#include <fstream>

#include "rutz/trace.h"

using rutz::fstring;
using rutz::shared_ptr;

rutz::gzstreambuf::gzstreambuf(const char* name, int om,
                               bool throw_exception)
  :
  m_opened(false),
  m_mode(0),
  m_gzfile(0)
{
  // no append nor read/write mode
  if ( (om & std::ios::ate) || (om & std::ios::app)
       || ((om & std::ios::in) && (om & std::ios::out)) )
    {
      /* do nothing -- opening fails */;
    }
  else
    {
      char fmode[10];
      char* fmodeptr = fmode;

      if (om & std::ios::in)
        {
          *fmodeptr++ = 'r';
          setg(m_buf+s_pback_size,
               m_buf+s_pback_size,
               m_buf+s_pback_size);
        }
      else if (om & std::ios::out)
        {
          *fmodeptr++ = 'w';
          setp(m_buf, m_buf+(s_buf_size-1));
        }

      *fmodeptr++ = 'b';
      *fmodeptr = '\0';

      m_gzfile = gzopen(name,fmode);

      if (m_gzfile != NULL)
        {
          m_opened = true;
          m_mode = om;
        }
    }

  if (throw_exception && !m_opened)
    {
      if (om & std::ios::in)
        {
          throw rutz::error(rutz::sfmt("couldn't open file '%s' "
                                       "for reading", name), SRC_POS);
        }
      else if (om & std::ios::out)
        {
          throw rutz::error(rutz::sfmt("couldn't open file '%s' "
                                       "for writing", name), SRC_POS);
        }
    }
}

void rutz::gzstreambuf::close()
{
  if (m_opened)
    {
      sync();
      m_opened = false;
      gzclose(m_gzfile);
    }
}

int rutz::gzstreambuf::underflow() // with help from Josuttis, p. 678
{
GVX_TRACE("rutz::gzstreambuf::underflow");
  // is read position before end of buffer?
  if (gptr() < egptr())
    return *gptr();

  int numPutback = 0;
  if (s_pback_size > 0)
    {
      // process size of putback area
      // -use number of characters read
      // -but at most four
      numPutback = gptr() - eback();
      if (numPutback > 4)
        numPutback = 4;

      // copy up to four characters previously read into the putback
      // buffer (area of first four characters)
      std::memcpy (m_buf+(4-numPutback), gptr()-numPutback,
                   numPutback);
    }

  // read new characters
  const int num =
    gzread(m_gzfile, m_buf+s_pback_size, s_buf_size-s_pback_size);

  if (num <= 0) // error (0) or end-of-file (-1)
    return EOF;

  // reset buffer pointers
  setg (m_buf+s_pback_size-numPutback,
        m_buf+s_pback_size,
        m_buf+s_pback_size+num);

  // return next character Hrmph. We have to cast to unsigned char to
  // avoid problems with eof. Problem is, -1 is a valid char value to
  // return. However, without a cast, char(-1) (0xff) gets converted
  // to int(-1), which is 0xffffffff, which is EOF! What we want is
  // int(0x000000ff), which we have to get by int(unsigned char(-1)).
  return static_cast<unsigned char>(*gptr());
}

int rutz::gzstreambuf::overflow(int c)
{
GVX_TRACE("rutz::gzstreambuf::overflow");
  if (!(m_mode & std::ios::out) || !m_opened) return EOF;

  if (c != EOF)
    {
      // insert the character into the buffer
      *pptr() = c;
      pbump(1);
    }

  if (flushoutput() == EOF)
    {
      return -1; // ERROR
    }

  return c;
}

int rutz::gzstreambuf::sync()
{
  if (flushoutput() == EOF)
    {
      return -1; // ERROR
    }
  return 0;
}

int rutz::gzstreambuf::flushoutput()
{
  if (!(m_mode & std::ios::out) || !m_opened) return EOF;

  int num = pptr()-pbase();
  if ( gzwrite(m_gzfile, pbase(), num) != num )
    {
      return EOF;
    }

  pbump(-num);
  return num;
}

namespace
{
  class gzstream : public std::iostream
  {
  private:
    rutz::gzstreambuf m_buf;
  public:
    gzstream(const char* filename_cstr,
             std::ios::openmode mode,
             bool throw_exception)
      :
      std::iostream(0),
      m_buf(filename_cstr, mode, throw_exception)
    {
      rdbuf(&m_buf);
    }
  };
}

shared_ptr<std::ostream> rutz::ogzopen(const fstring& filename,
                                       std::ios::openmode flags)
{
  static fstring gz_ext(".gz");

  if (filename.ends_with(gz_ext))
    {
      return shared_ptr<std::ostream>
        (new gzstream(filename.c_str(), std::ios::out|flags, true));
    }
  else
    {
      shared_ptr<std::ostream> result =
        make_shared(new std::ofstream(filename.c_str(), flags));
      if (result->fail())
        throw rutz::error(rutz::sfmt("couldn't open file '%s' "
                                     "for writing", filename.c_str()),
                          SRC_POS);

      return result;
    }
}

shared_ptr<std::ostream> rutz::ogzopen(const char* filename,
                                       std::ios::openmode flags)
{
  return ogzopen(fstring(filename), flags);
}

shared_ptr<std::istream> rutz::igzopen(const char* filename,
                                       std::ios::openmode flags)
{
  return shared_ptr<std::iostream>
    (new gzstream(filename, std::ios::in|flags, true));
}

shared_ptr<std::istream> rutz::igzopen(const fstring& filename,
                                       std::ios::openmode flags)
{
  return igzopen(filename.c_str(), flags);
}

// sample test code

//  #include "src/util/gzstreambuf.h"

//  int main() {
//    {
//      rutz::gzstreambuf buf("test.gz", std::ios::out);

//      std::ostream os(&buf);

//      os << "Hello, World!\n";
//    }

//    {
//      rutz::gzstreambuf buf2("test.gz", std::ios::in);

//      std::istream is(&buf2);

//      int c;
//      while ( (c=is.get()) != EOF )
//        {
//          std::cout << char(c);
//        }
//    }

//    return 0;
//  }

static const char __attribute__((used)) vcid_groovx_rutz_gzstreambuf_cc_utc20050626084020[] = "$Id: gzstreambuf.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/gzstreambuf.cc $";
#endif // !GROOVX_RUTZ_GZSTREAMBUF_CC_UTC20050626084020_DEFINED
