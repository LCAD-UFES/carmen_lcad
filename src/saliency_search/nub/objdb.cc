/** @file nub/objdb.cc singleton repository that associates each
    nub::object with its nub::uid, so that objects can be looked up by
    id -- this provides the foundation for using uids as object
    handles in a scripting language */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Sun Nov 21 00:26:29 1999
// commit: $Id: objdb.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objdb.cc $
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

#ifndef GROOVX_NUB_OBJDB_CC_UTC20050626084019_DEFINED
#define GROOVX_NUB_OBJDB_CC_UTC20050626084019_DEFINED

#include "objdb.h"

#include "nub/object.h"
#include "nub/weak_handle.h"

#include "rutz/sfmt.h"

#include <typeinfo>
#include <map>

#include "rutz/trace.h"
#include "rutz/debug.h"
GVX_DBG_REGISTER

using rutz::shared_ptr;

nub::invalid_uid_error::invalid_uid_error(nub::uid id,
                                    const rutz::file_pos& pos)
  :
  rutz::error(rutz::sfmt("attempted to access invalid object '%ld'",
                         id), pos)
{}

nub::invalid_uid_error::~invalid_uid_error() throw() {}

///////////////////////////////////////////////////////////////////////
//
// nub::objectdb::impl definition
//
///////////////////////////////////////////////////////////////////////

class nub::objectdb::impl
{
private:
  impl(const impl&);
  impl& operator=(const impl&);

public:

  typedef nub::detail::weak_handle<nub::object> obj_ref;

  typedef std::map<nub::uid, obj_ref> map_type;
  mutable map_type m_obj_map;

  impl() : m_obj_map() {}

  // Check whether the iterator points to a valid spot in the map, AND
  // that it points to a still-living object. If the object has died,
  // then we erase the iterator.
  bool is_valid_itr(map_type::iterator itr) const throw()
  {
    if (itr == m_obj_map.end()) return false;

    if (!(*itr).second.is_valid())
      {
        m_obj_map.erase(itr);
        return false;
      }

    return true;
  }

  bool is_valid_uid(nub::uid id) const throw()
    {
      map_type::iterator itr = m_obj_map.find(id);
      return is_valid_itr(itr);
    }

  int count() const throw()
    { return m_obj_map.size(); }

  void release(nub::uid id)
    {
      map_type::iterator itr = m_obj_map.find(id);

      m_obj_map.erase(itr);
    }

  void remove(nub::uid id)
    {
      map_type::iterator itr = m_obj_map.find(id);
      if (!is_valid_itr(itr)) return;

      if ( (*itr).second.get()->is_shared() )
        throw rutz::error("attempted to remove a shared object", SRC_POS);

      m_obj_map.erase(itr);
    }

  // Return the number of items removed
  int purge()
    {
      map_type new_map;

      int num_removed = 0;

      for (map_type::iterator
             itr = m_obj_map.begin(),
             end = m_obj_map.end();
           itr != end;
           ++itr)
        {
          // If the object is shared, we'll be saving the object, so
          // copy it into the new_map,
          if ( is_valid_itr(itr) && (*itr).second.get()->is_shared() )
            {
              new_map.insert(*itr);
            }
          else
            {
              ++num_removed;
            }
        }

      // Now swap maps so that the old map gets cleared and everything erased
      m_obj_map.swap(new_map);
      return num_removed;
    }

  void clear_all()
    {
      m_obj_map.clear();

#if 0 // an alternate implementation for verbose debugging:
      while (!m_obj_map.empty())
        {
          map_type::iterator it = m_obj_map.begin();

          if ((*it).second.is_valid())
            {
              dbg_eval_nl(3, typeid(*(*it).second).name());
              dbg_eval_nl(3, (*it).second->id());
            }

          m_obj_map.erase(it);
        }
#endif
    }

  nub::object* get_checked_obj(nub::uid id) throw (nub::invalid_uid_error)
    {
      map_type::iterator itr = m_obj_map.find(id);
      if (!is_valid_itr(itr))
        {
          throw nub::invalid_uid_error(id, SRC_POS);
        }

      return (*itr).second.get();
    }

  void insert_obj(nub::object* ptr, bool strong)
    {
      GVX_PRECONDITION(ptr != 0);

      // Check if the object is already in the map
      map_type::iterator existing_site = m_obj_map.find(ptr->id());
      if (existing_site != m_obj_map.end())
        {
          // Make sure the existing object is the same as the object
          // that we're trying to insert
          GVX_ASSERT( (*existing_site).second.get() == ptr );
        }

      const int new_id = ptr->id();

      m_obj_map.insert
        (map_type::value_type
         (new_id, obj_ref(ptr, strong ? nub::STRONG : nub::WEAK)));
    }
};

///////////////////////////////////////////////////////////////////////
//
// nub::objectdb::iterator definitions
//
///////////////////////////////////////////////////////////////////////

namespace
{
  class iter_impl :
    public rutz::fwd_iter_ifx<nub::object* const>
  {
  public:
    typedef nub::objectdb::impl::map_type map_type;

    void advance_to_valid()
    {
      while (true)
        {
          if (m_iter == m_end)
            {
              m_obj = 0;
              return;
            }

          if ((*m_iter).second.is_valid())
            {
              m_obj = (*m_iter).second.get_weak();
              return;
            }

          map_type::iterator bad = m_iter;
          ++m_iter;

          m_map.erase(bad);
        }
    }

    iter_impl(map_type& m, map_type::iterator itr) :
      m_map(m), m_iter(itr), m_obj(0), m_end(m.end())
    {
      advance_to_valid();
    }

    map_type& m_map;
    map_type::iterator m_iter;
    nub::object* m_obj;
    const map_type::iterator m_end;

    virtual ifx_t* clone() const
    {
      return new iter_impl(m_map, m_iter);
    }

    virtual void next()
    {
      if (!at_end())
        {
          ++m_iter;
          advance_to_valid();
        }
    }

    virtual value_t& get() const
    {
      GVX_ASSERT(m_iter == m_end || (*m_iter).second.get_weak() == m_obj);
      return m_obj;
    }

    virtual bool at_end() const
    {
      return (m_iter == m_end);
    }
  };
}

///////////////////////////////////////////////////////////////////////
//
// nub::objectdb member definitions
//
///////////////////////////////////////////////////////////////////////

nub::objectdb& nub::objectdb::instance()
{
  static nub::objectdb* instance = 0;
  if (instance == 0)
    {
      instance = new nub::objectdb;
    }
  return *instance;
}

nub::objectdb::iterator nub::objectdb::objects() const
{
GVX_TRACE("nub::objectdb::children");

 return shared_ptr<nub::objectdb::iterator::ifx_t>
   (new iter_impl(rep->m_obj_map, rep->m_obj_map.begin()));
}

nub::objectdb::objectdb() :
  rep(new impl)
{
GVX_TRACE("nub::objectdb::objectdb");
}

nub::objectdb::~objectdb()
{
GVX_TRACE("nub::objectdb::~objectdb");
  delete rep;
}

int nub::objectdb::count() const throw()
{
GVX_TRACE("nub::objectdb::count");

  return rep->count();
}

bool nub::objectdb::is_valid_uid(nub::uid id) const throw()
{
GVX_TRACE("nub::objectdb::is_valid_uid");
  return rep->is_valid_uid(id);
}

void nub::objectdb::remove(nub::uid id)
{
GVX_TRACE("nub::objectdb::remove");
  rep->remove(id);
}

void nub::objectdb::release(nub::uid id)
{
GVX_TRACE("nub::objectdb::release");
  rep->release(id);
}

void nub::objectdb::purge()
{
GVX_TRACE("nub::objectdb::clear");
  dbg_eval_nl(3, typeid(*this).name());
  rep->purge();
}

void nub::objectdb::clear()
{
GVX_TRACE("nub::objectdb::clear");
  // Call purge until no more items can be removed
  while ( rep->purge() != 0 )
    { ; }
}

void nub::objectdb::clear_on_exit()
{
GVX_TRACE("nub::objectdb::clear_on_exit");
  rep->clear_all();
}

nub::object* nub::objectdb::get_checked_obj(nub::uid id) throw (nub::invalid_uid_error)
{
GVX_TRACE("nub::objectdb::get_checked_obj");
  return rep->get_checked_obj(id);
}

void nub::objectdb::insert_obj(nub::object* obj)
{
GVX_TRACE("nub::objectdb::insert_obj");
  rep->insert_obj(obj, true);
}

void nub::objectdb::insert_obj_weak(nub::object* obj)
{
GVX_TRACE("nub::objectdb::insert_obj_weak");
  rep->insert_obj(obj, false);
}

static const char __attribute__((used)) vcid_groovx_nub_objdb_cc_utc20050626084019[] = "$Id: objdb.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objdb.cc $";
#endif // !GROOVX_NUB_OBJDB_CC_UTC20050626084019_DEFINED
