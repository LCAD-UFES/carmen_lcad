/** @file rutz/stdiobuf.cc wrap posix file descriptors in c++ iostreams */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Tue Feb 25 13:52:11 2003
// commit: $Id: stdiobuf.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/stdiobuf.cc $
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

#ifndef GROOVX_RUTZ_STDIOBUF_CC_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_STDIOBUF_CC_UTC20050626084020_DEFINED

#include "rutz/stdiobuf.h"

#include "rutz/error.h"

#include <ios>
#include <unistd.h>

#include "rutz/debug.h"
GVX_DBG_REGISTER
#include "rutz/trace.h"

void rutz::stdiobuf::init(int fd, int om, bool throw_exception)
{
GVX_TRACE("rutz::stdiobuf::init");
  m_filedes = fd;

  if (m_filedes >= 0)
    {
      m_mode = om;
    }

  if (throw_exception && !is_open())
    {
      throw rutz::error("couldn't open file stdiobuf", SRC_POS);
    }

  setg (buffer+s_pback_size,
        buffer+s_pback_size,
        buffer+s_pback_size);

  setp (buffer,
        buffer+s_buf_size-1);
}

rutz::stdiobuf::stdiobuf(FILE* f, int om, bool throw_exception) :
  m_mode(0), m_filedes(-1)
{
GVX_TRACE("rutz::stdiobuf::stdiobuf(FILE*)");
  init(fileno(f), om, throw_exception);
}

rutz::stdiobuf::stdiobuf(int fd, int om, bool throw_exception) :
  m_mode(0), m_filedes(-1)
{
GVX_TRACE("rutz::stdiobuf::stdiobuf(int)");

  init(fd, om, throw_exception);
}

void rutz::stdiobuf::close()
{
GVX_TRACE("rutz::stdiobuf::close");
  if (is_open())
    {
      this->sync();
      // NOTE: We don't do ::close(m_filedes) here since we leave that
      // up to whoever instantiated this stdiobuf.
      m_filedes = -1;
      m_mode = 0;
    }
}

int rutz::stdiobuf::underflow() // with help from Josuttis, p. 678
{
GVX_TRACE("rutz::stdiobuf::underflow");
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
      std::memcpy (buffer+(4-numPutback), gptr()-numPutback,
                   numPutback);
    }

  // read new characters
  const int num = ::read(m_filedes,
                         buffer+s_pback_size,
                         s_buf_size-s_pback_size);

  if (num <= 0)
    return EOF;

  // reset buffer pointers
  setg (buffer+s_pback_size-numPutback,
        buffer+s_pback_size,
        buffer+s_pback_size+num);

  // return next character Hrmph. We have to cast to unsigned char to
  // avoid problems with eof. Problem is, -1 is a valid char value to
  // return. However, without a cast, char(-1) (0xff) gets converted
  // to int(-1), which is 0xffffffff, which is EOF! What we want is
  // int(0x000000ff), which we have to get by int(unsigned char(-1)).
  return static_cast<unsigned char>(*gptr());
}

int rutz::stdiobuf::overflow(int c)
{
GVX_TRACE("rutz::stdiobuf::overflow");
  if (!(m_mode & std::ios::out) || !is_open()) return EOF;

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

int rutz::stdiobuf::sync()
{
  if (flushoutput() == EOF)
    {
      return -1; // ERROR
    }
  return 0;
}

int rutz::stdiobuf::flushoutput()
{
  if (!(m_mode & std::ios::out) || !is_open()) return EOF;

  const int num = pptr()-pbase();

  if ( ::write(m_filedes, pbase(), num) == -1 )
    {
      return EOF;
    }

  pbump(-num);
  return num;
}

// This is a hack to work around an apparently broken g++ 3.4.3
// on USC's HPCC cluster, in which libstdc++ doesn't include
// instantiations for the constructor and destructor of std::iostream
// (aka std::basic_iostream<char>); so, we force an instantiation of
// those functions here to avoid link errors. This line should be
// quickly removed as soon as we no longer need to use g++ 3.4.3.
#ifdef GVX_MISSING_IOSTREAM_INSTANTIATION
template class std::basic_iostream<char>;
#endif

static const char __attribute__((used)) vcid_groovx_rutz_stdiobuf_cc_utc20050626084020[] = "$Id: stdiobuf.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/stdiobuf.cc $";
#endif // !GROOVX_RUTZ_STDIOBUF_CC_UTC20050626084020_DEFINED
