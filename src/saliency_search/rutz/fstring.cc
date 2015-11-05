/** @file rutz/fstring.cc ref-counted string type that allows much
    faster compile times than std::string */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2000-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Oct 15 15:43:41 2004
// commit: $Id: fstring.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/fstring.cc $
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

#ifndef GROOVX_RUTZ_FSTRING_CC_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_FSTRING_CC_UTC20050626084020_DEFINED

#include "fstring.h"

#include "rutz/algo.h"
#include "rutz/compat_snprintf.h"
#include "rutz/freelist.h"
#include "rutz/mutex.h"

#include <cctype>
#include <cstring>
#include <istream>
#include <ostream>
#include <pthread.h>

#ifndef GVX_NO_PROF
#define GVX_NO_PROF
#endif

#include "rutz/trace.h"
#include "rutz/debug.h"
GVX_DBG_REGISTER

//---------------------------------------------------------------------
//
// rutz::string_rep member definitions
//
//---------------------------------------------------------------------

namespace
{
  rutz::free_list<rutz::string_rep>* g_rep_list;
  pthread_mutex_t g_rep_list_mutex = PTHREAD_MUTEX_INITIALIZER;
}

void* rutz::string_rep::operator new(size_t bytes)
{
  GVX_MUTEX_LOCK(&g_rep_list_mutex);
  if (g_rep_list == 0) g_rep_list = new rutz::free_list<rutz::string_rep>;
  return g_rep_list->allocate(bytes);
}

void rutz::string_rep::operator delete(void* space)
{
  GVX_MUTEX_LOCK(&g_rep_list_mutex);
  g_rep_list->deallocate(space);
}

rutz::string_rep::string_rep(std::size_t len, const char* txt,
                             std::size_t capac) :
  m_refcount(),
  m_capacity(rutz::max(len+1, capac)),
  m_length(0),
  m_text(new char[m_capacity])
{
  m_refcount.atomic_set(0);

  if (txt)
    uniq_append(len, txt);
  else
    add_terminator();
}

rutz::string_rep::~string_rep() throw()
{
GVX_TRACE("rutz::string_rep::~string_rep");

  delete [] m_text;
  m_text = (char*)0xdeadbeef;
}

rutz::string_rep* rutz::string_rep::make(std::size_t length,
                                         const char* text,
                                         std::size_t capacity)
{
  return new rutz::string_rep(length, text, capacity);
}

rutz::string_rep* rutz::string_rep::read_from_stream(std::istream& is)
{
  rutz::string_rep* result = rutz::string_rep::make(0, 0, 32);
  is >> std::ws;
  while ( true )
    {
      int c = is.get();
      if (c == EOF || isspace(c))
        {
          is.unget();
          break;
        }
      result->uniq_append_no_terminate(char(c));
    }
  result->add_terminator();

  return result;
}

rutz::string_rep* rutz::string_rep::readsome_from_stream(std::istream& is, unsigned int count)
{
  rutz::string_rep* result = rutz::string_rep::make(0, 0, count+2);

  if (count > 0)
    {
      unsigned int numread = is.readsome(result->m_text, count);
      result->uniq_set_length(numread); // includes add_terminator()
    }

  return result;
}

rutz::string_rep* rutz::string_rep::readline_from_stream(std::istream& is, char eol)
{
  rutz::string_rep* result = rutz::string_rep::make(0, 0, 32);

  while ( true )
    {
      int c = is.get();
      if (c == EOF || c == eol)
        break;
      result->uniq_append_no_terminate(char(c));
    }

  result->add_terminator();

  return result;
}

void rutz::string_rep::debug_dump() const throw()
{
  dbg_eval_nl(0, (const void*)this);
  dbg_eval_nl(0, m_refcount.atomic_get());
  dbg_eval_nl(0, m_length);
  dbg_eval_nl(0, (void*)m_text);
  dbg_eval_nl(0, m_text);
  for (unsigned int i = 0; i < m_length; ++i)
    dbg_print(0, (void*)(size_t)m_text[i]);
  dbg_print_nl(0, "");
}

void rutz::string_rep::uniq_append_no_terminate(char c)
{
  if (m_length + 2 <= m_capacity)
    {
      m_text[m_length++] = c;
    }
  else
    {
      uniq_realloc(m_length + 2);
      m_text[m_length++] = c;
    }
}

void rutz::string_rep::add_terminator() throw()
{
  m_text[m_length] = '\0';
}

void rutz::string_rep::uniq_set_length(std::size_t len) throw()
{
  GVX_PRECONDITION(m_refcount.atomic_get() <= 1);
  GVX_PRECONDITION(len+1 < m_capacity);
  m_length = len;
  add_terminator();
}

void rutz::string_rep::uniq_append(std::size_t len, const char* txt)
{
  GVX_PRECONDITION(m_refcount.atomic_get() <= 1);
  GVX_PRECONDITION(txt != 0);

  if (m_length + len + 1 <= m_capacity)
    {
      memcpy(m_text+m_length, txt, len);
      m_length += len;
      add_terminator();
    }
  else
    {
      uniq_realloc(m_length + len + 1);
      uniq_append(len, txt);
    }

  GVX_POSTCONDITION(m_length+1 <= m_capacity);
  GVX_POSTCONDITION(m_text[m_length] == '\0');
}

void rutz::string_rep::uniq_realloc(std::size_t capac)
{
  GVX_PRECONDITION(m_refcount.atomic_get() <= 1);

  rutz::string_rep new_rep(rutz::max(m_capacity*2 + 32, capac), 0);

  new_rep.uniq_append(this->m_length, this->m_text);

  rutz::swap2(m_capacity, new_rep.m_capacity);
  rutz::swap2(m_length, new_rep.m_length);
  rutz::swap2(m_text, new_rep.m_text);
}

//---------------------------------------------------------------------
//
// fstring member definitions
//
//---------------------------------------------------------------------

void rutz::fstring::init_empty()
{
GVX_TRACE("rutz::fstring::init_empty");
  GVX_PRECONDITION(m_rep == 0);

  m_rep = string_rep::make(0, 0);

  m_rep->incr_ref_count();
}

void rutz::fstring::init_range(char_range r)
{
GVX_TRACE("rutz::fstring::init_range");
  GVX_PRECONDITION(m_rep == 0);

  m_rep = string_rep::make(r.len, r.text);

  m_rep->incr_ref_count();
}

rutz::fstring::fstring(rutz::string_rep* r)
  :
  m_rep(r)
{
GVX_TRACE("rutz::fstring::fstring(string_rep*)");
  m_rep->incr_ref_count();
}

rutz::fstring::fstring() :
  m_rep(string_rep::make(0,0))
{
GVX_TRACE("rutz::fstring::fstring");

  m_rep->incr_ref_count();
}

rutz::fstring::fstring(const rutz::fstring& other) throw() :
  m_rep(other.m_rep)
{
GVX_TRACE("rutz::fstring::fstring(const fstring&)");
  m_rep->incr_ref_count();
}

rutz::fstring::~fstring() throw()
{
GVX_TRACE("rutz::fstring::~fstring");

  dbg_dump(7, *this);

  if (m_rep->decr_ref_count() == 0)
    m_rep = (string_rep*)0xdeadbeef;
}

void rutz::fstring::swap(rutz::fstring& other) throw()
{
GVX_TRACE("rutz::fstring::swap");

  rutz::swap2(m_rep, other.m_rep);
}

rutz::fstring& rutz::fstring::operator=(const char* text)
{
GVX_TRACE("rutz::fstring::operator=(const char*)");

  fstring copy(text);
  this->swap(copy);
  return *this;
}

rutz::fstring& rutz::fstring::operator=(const fstring& other) throw()
{
GVX_TRACE("rutz::fstring::operator=(const fstring&)");

  rutz::fstring copy(other);
  this->swap(copy);
  return *this;
}

bool rutz::fstring::ends_with(const fstring& ext) const throw()
{
GVX_TRACE("rutz::fstring::ends_with");
  if (ext.length() > this->length())
    return false;

  unsigned int skip = this->length() - ext.length();

  return ext.equals(this->c_str() + skip);
}

void rutz::fstring::clear()
{
GVX_TRACE("rutz::fstring::clear");

  fstring().swap(*this);
}

bool rutz::fstring::equals(const char* other) const throw()
{
GVX_TRACE("rutz::fstring::equals(const char*)");
  return ( c_str() == other ||
           strcmp(c_str(), other) == 0 );
}

bool rutz::fstring::equals(const fstring& other) const throw()
{
GVX_TRACE("rutz::fstring::equals(const fstring&)");

  return c_str() == other.c_str() ||
    ( length() == other.length() &&
      strcmp(c_str(), other.c_str()) == 0 );
}

bool rutz::fstring::operator<(const char* other) const throw()
{
GVX_TRACE("rutz::fstring::operator<");
  // Check if we are pointing to the same string
  if (c_str() == other) return false;
  // ...otherwise do a string compare
  return strcmp(c_str(), other) < 0;
}

bool rutz::fstring::operator>(const char* other) const throw()
{
GVX_TRACE("rutz::fstring::operator>");
  // Check if we are pointing to the same string
  if (c_str() == other) return false;
  // ...otherwise do a string compare
  return strcmp(c_str(), other) > 0;
}

//---------------------------------------------------------------------
//
// Input/Output function definitions
//
//---------------------------------------------------------------------

void rutz::fstring::read(std::istream& is)
{
GVX_TRACE("rutz::fstring::read");
  rutz::fstring(string_rep::read_from_stream(is)).swap(*this);
}

void rutz::fstring::readsome(std::istream& is, unsigned int count)
{
GVX_TRACE("rutz::fstring::readsome");
  rutz::fstring(string_rep::readsome_from_stream(is, count)).swap(*this);
}

void rutz::fstring::write(std::ostream& os) const
{
GVX_TRACE("rutz::fstring::write");
  os.write(c_str(), length());
}

void rutz::fstring::readline(std::istream& is, char eol)
{
GVX_TRACE("rutz::fstring::readline");
  rutz::fstring(string_rep::readline_from_stream(is, eol)).swap(*this);
}

void rutz::fstring::debug_dump() const throw()
{
  dbg_eval_nl(0, (const void*)this);
  m_rep->debug_dump();
}

using rutz::fstring;

fstring rutz::sconvert(char x)          { return fstring(rutz::char_range(&x, 1)); }
fstring rutz::sconvert(const char* x)   { return fstring(x); }
fstring rutz::sconvert(const fstring& x){ return x; }

#define NUM_CONVERT(fmt, val)                   \
  const int SZ = 64;                            \
  char buf[SZ];                                 \
  int n = snprintf(buf, SZ, fmt, (val));        \
  GVX_ASSERT(n > 0 && n < SZ);                  \
  return fstring(&buf[0]);

fstring rutz::sconvert(bool x)          { NUM_CONVERT("%d", int(x)) }
fstring rutz::sconvert(int x)           { NUM_CONVERT("%d", x) }
fstring rutz::sconvert(unsigned int x)  { NUM_CONVERT("%u", x) }
fstring rutz::sconvert(long x)          { NUM_CONVERT("%ld", x) }
fstring rutz::sconvert(unsigned long x) { NUM_CONVERT("%lu", x) }
fstring rutz::sconvert(double x)        { NUM_CONVERT("%g", x) }

#undef NUM_CONVERT

static const char __attribute__((used)) vcid_groovx_rutz_fstring_cc_utc20050626084020[] = "$Id: fstring.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/fstring.cc $";
#endif // !GROOVX_RUTZ_FSTRING_CC_UTC20050626084020_DEFINED
