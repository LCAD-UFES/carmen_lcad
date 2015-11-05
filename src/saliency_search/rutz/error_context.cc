/** @file rutz/error_context.cc */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Nov 17 16:14:11 2005
// commit: $Id: error_context.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/error_context.cc $
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

#ifndef GROOVX_RUTZ_ERROR_CONTEXT_CC_UTC20051118001411_DEFINED
#define GROOVX_RUTZ_ERROR_CONTEXT_CC_UTC20051118001411_DEFINED

#include "rutz/error_context.h"

#include <new>
#include <pthread.h>
#include <sstream>

#include "rutz/debug.h"
GVX_DBG_REGISTER

namespace
{
  // FIXME this "thread-local singleton" code is duplicated here and
  // in rutz/backtrace.cc; can we abstract out the common code?

  // thread-local storage (see rutz::error_context::current() below)
  pthread_key_t current_context_key;
  pthread_once_t current_context_key_once = PTHREAD_ONCE_INIT;

  void current_context_destroy(void* bt)
  {
    delete static_cast<rutz::error_context*>(bt);
  }

  void current_context_key_alloc()
  {
    pthread_key_create(&current_context_key,
                       &current_context_destroy);
  }

  // returns a mutable object (but note that
  // rutz::error_context::current() returns a const object)
  rutz::error_context* get_current_context()
  {
    pthread_once(&current_context_key_once,
                 &current_context_key_alloc);

    void* const ptr = pthread_getspecific(current_context_key);

    if (ptr != 0)
      {
        return static_cast<rutz::error_context*>(ptr);
      }

    // else...
    rutz::error_context* const c = new (std::nothrow) rutz::error_context;

    if (c == 0)
      GVX_ABORT("memory allocation failed");

    pthread_setspecific(current_context_key,
                        static_cast<void*>(c));

    return c;
  }
}

rutz::error_context::error_context() {}

rutz::error_context::~error_context() {}

const rutz::error_context& rutz::error_context::current()
{
  return *(get_current_context());
}

bool rutz::error_context::add_entry(const error_context_entry* e)
{
  return m_entries.push(e);
}

void rutz::error_context::remove_entry(const error_context_entry* e)
{
  GVX_ASSERT(m_entries.top() == e);
  m_entries.pop();
}

rutz::fstring rutz::error_context::get_text() const
{
  std::ostringstream result;

  const char* prefix = "==> while ";

  for (unsigned int i = 0; i < m_entries.size(); ++i)
    {
      result << prefix << m_entries[i]->text() << ":";
      prefix = "\n==> while ";
    }

  return rutz::fstring(result.str().c_str());
}

rutz::error_context_entry::error_context_entry(const rutz::fstring& msg)
  :
  m_text(msg),
  m_context(get_current_context())
{
  if (!m_context->add_entry(this))
    m_context = 0;
}

rutz::error_context_entry::~error_context_entry()
{
  if (m_context)
    m_context->remove_entry(this);
}

static const char __attribute__((used)) vcid_groovx_rutz_error_context_cc_utc20051118001411[] = "$Id: error_context.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/error_context.cc $";
#endif // !GROOVX_RUTZ_ERROR_CONTEXT_CC_UTC20051118001411DEFINED
