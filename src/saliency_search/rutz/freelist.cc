/** @file rutz/freelist.cc memory allocation via a free-list pool */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Jul 20 08:00:31 2001
// commit: $Id: freelist.cc 9612 2008-04-10 16:40:31Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/freelist.cc $
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

#ifndef GROOVX_RUTZ_FREELIST_CC_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_FREELIST_CC_UTC20050626084019_DEFINED

#include "rutz/freelist.h"

#include "rutz/debug.h"
GVX_DBG_REGISTER

rutz::free_list_base::free_list_base(std::size_t size_check) :
  m_node_list(0), m_num_allocations(0), m_size_check(size_check)
{
  GVX_ASSERT(m_size_check >= sizeof(node));
}

void* rutz::free_list_base::allocate(std::size_t bytes)
{
  GVX_ASSERT(bytes == m_size_check);
  if (m_node_list == 0)
    {
      ++m_num_allocations;
      return ::operator new(bytes);
    }
  node* n = m_node_list;
  m_node_list = m_node_list->next;
  return static_cast<void*>(n);
}

void rutz::free_list_base::deallocate(void* space)
{
  node* n = static_cast<node*>(space);
  n->next = m_node_list;
  m_node_list = n;
}

void rutz::free_list_base::release_free_nodes()
{
  while (m_node_list != 0)
    {
      void* p = m_node_list;
      m_node_list = m_node_list->next;
      ::operator delete(p);
      --m_num_allocations;
    }
}

static const char __attribute__((used)) vcid_groovx_rutz_freelist_cc_utc20050626084019[] = "$Id: freelist.cc 9612 2008-04-10 16:40:31Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/freelist.cc $";
#endif // !GROOVX_RUTZ_FREELIST_CC_UTC20050626084019_DEFINED
