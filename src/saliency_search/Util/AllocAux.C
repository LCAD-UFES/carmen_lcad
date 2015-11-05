/*!@file Util/AllocAux.C memory allocation routines for 16-byte alignment */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/AllocAux.C $
// $Id: AllocAux.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef UTIL_ALLOCAUX_C_DEFINED
#define UTIL_ALLOCAUX_C_DEFINED

#include "Util/AllocAux.H"

#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/freelist.h"
#include "rutz/mutex.h"
#include "rutz/trace.h"

#include <map>
#include <pthread.h>

namespace
{
  //! Trivial allocator that just calls operator new() and operator delete()
  struct trivial_alloc
  {
    void set_debug(bool /*do_debug*/)
    {
      // no debug settings for this allocator type
    }

    void set_allow_caching(bool /*on*/)
    {
      // no caching here in any case
    }

    void show_stats(int /*verbosity*/, const char* /*pfx*/,
                    const size_t block_size, const size_t overhead) const
    {
      // nothing to do here
    }

    void* allocate(size_t nbytes, rutz::free_list_base** source = 0)
    {
      if (source != 0)
        *source = 0;
      return ::operator new(nbytes);
    }

    void deallocate(void* space, rutz::free_list_base* source = 0)
    {
      ASSERT(source == 0);
      ::operator delete(space);
    }

    void release_free_mem()
    {
      // nothing to do here
    }
  };

  //! Caching allocator with free lists for common allocation sizes
  template <size_t cache_size>
  struct fastcache_alloc
  {
    rutz::free_list_base* cache[cache_size];
    mutable size_t num_alloc[cache_size];
    bool allow_caching;

    fastcache_alloc()
      :
      allow_caching(true)
    {
      for (size_t i = 0; i < cache_size; ++i)
        {
          this->cache[i] = 0;
          this->num_alloc[i] = 0;
        }
    }

    void set_debug(bool /*do_debug*/)
    {
      // no debug settings for this allocator type
    }

    void set_allow_caching(bool on)
    {
      if (!on && this->allow_caching)
        {
          // if we are currently caching but are being asked to turn
          // off caching, then let's first free any existing caches

          this->release_free_mem();
        }
      this->allow_caching = on;
    }

    void show_stats(int verbosity, const char* pfx,
                    const size_t block_size, const size_t overhead) const
    {
      size_t nused = 0;
      size_t bytes_allocated = 0;

      std::map<size_t, std::string> msgs;

      for (size_t i = 0; i < cache_size; ++i)
        if (this->cache[i] != 0)
          {
            ++nused;
            const size_t nb = (this->cache[i]->num_allocations()
                               * this->cache[i]->alloc_size());

            const size_t extra = (this->cache[i]->num_allocations()
                                  - this->num_alloc[i]);

            this->num_alloc[i] = this->cache[i]->num_allocations();

            bytes_allocated += nb;

            if (verbosity <= 0)
              continue;

            std::string msg =
              sformat("%s%sfastcache[%02" ZU "/%02" ZU "]: "
                      "%10.4fMB in %4" ZU " allocations of %10.4fkB",
                      pfx ? pfx : "", pfx ? ": " : "",
                      i, cache_size, nb / (1024.0*1024.0),
                      this->cache[i]->num_allocations(),
                      this->cache[i]->alloc_size() / 1024.0);

            if (block_size > 0)
              {
                if (this->cache[i]->alloc_size() - overhead >= block_size
                    || this->cache[i]->alloc_size() - overhead <= 1)
                  msg += sformat(" (%.2fkB * %7.1f + %" ZU "B)",
                                 block_size / 1024.0,
                                 (double(this->cache[i]->alloc_size() - overhead)
                                  / double(block_size)),
                                 overhead);
                else
                  msg += sformat(" (%.2fkB / %7.1f + %" ZU "B)",
                                 block_size / 1024.0,
                                 (double(block_size)
                                  / double(this->cache[i]->alloc_size() - overhead)),
                                 overhead);
              }

            if (extra > 0)
              msg += sformat(" (+%" ZU " new)", extra);

            msgs[this->cache[i]->alloc_size()] = msg;
          }

      for (std::map<size_t, std::string>::const_iterator
             itr = msgs.begin(), stop = msgs.end();
           itr != stop; ++itr)
        LINFO("%s", (*itr).second.c_str());


      std::string msg =
        sformat("%s%sfastcache_alloc<%" ZU ">: %" ZU "/%" ZU " cache table "
                "entries in use, %fMB total allocated",
                pfx ? pfx : "", pfx ? ": " : "",
                cache_size, nused, cache_size,
                bytes_allocated / (1024.0*1024.0));

      if (block_size > 0)
        msg += sformat(" (%.2fkB * %7.1f)",
                       block_size / 1024.0,
                       double(bytes_allocated) / double(block_size));

      LINFO("%s", msg.c_str());
    }

    // allocate memory block of size nbytes; also return the address
    // of the rutz::free_list_base, if any, that was used for
    // allocation
    void* allocate(size_t nbytes, rutz::free_list_base** source)
    {
      if (this->allow_caching && source != 0)
        for (size_t i = 0; i < cache_size; ++i)
          {
            if (this->cache[i] != 0)
              {
                // we found a filled slot, let's see if it matches our
                // requested size
                if (this->cache[i]->alloc_size() == nbytes)
                  {
                    *source = this->cache[i];
                    return this->cache[i]->allocate(nbytes);
                  }
                // else, continue
              }
            else // this->cache[i] == 0
              {
                // we found an empty slot, let's set up a new free
                // list for our requested size:
                this->cache[i] = new rutz::free_list_base(nbytes);
                *source = this->cache[i];
                return this->cache[i]->allocate(nbytes);
              }
          }

      *source = 0;
      return ::operator new(nbytes);
    }

    // deallocate memory from the given rutz::free_list_base,
    // otherwise free it globally
    void deallocate(void* space, rutz::free_list_base* source)
    {
      if (source != 0)
        {
          source->deallocate(space);
          if (!this->allow_caching)
            source->release_free_nodes();
        }
      else
        {
          ::operator delete(space);
        }
    }

    void release_free_mem()
    {
      for (size_t i = 0; i < cache_size; ++i)
        if (this->cache[i] != 0)
          this->cache[i]->release_free_nodes();
    }
  };

  //! Auxiliary information about an aligned memory allocation
  template <size_t N>
  struct alloc_info
  {
    struct data_s
    {
      // address of ALLOCATED memory, not the address returned to
      // the user:
      void* alloc_addr;

      // number of bytes ALLOCATED, not the number requested by the
      // user:
      size_t alloc_nbytes;

      // the freelist from which this memory was allocated, or else
      // null:
      rutz::free_list_base* source;

      size_t user_nbytes() const
      {
        return alloc_nbytes - 2*N;
      }

      unsigned long align() const
      {
        return reinterpret_cast<unsigned long>(this->alloc_addr) % N;
      }

      unsigned long adjust() const
      {
        return N-this->align();
      }

      void* user_addr() const
      {
        return
          static_cast<char*>(this->alloc_addr)
          +this->adjust()
          +N;
      }

      unsigned long user_align() const
      {
        return reinterpret_cast<unsigned long>(this->user_addr()) % N;
      }

      void print() const
      {
        LINFO("alloc: internal=[%" ZU " bytes @ %p (align%%%zu=%lu)], "
              "user=[%" ZU " bytes @ %p (align%%%" ZU "=%lu)]",
              this->alloc_nbytes, this->alloc_addr, N, this->align(),
              this->user_nbytes(), this->user_addr(), N, this->user_align());
      }

    };
    data_s data;
    char pad[N-sizeof(data_s)];
  };

  //! Allocate memory that is aligned on an N-byte boundary
  template <class src_type, size_t N>
  struct aligned_alloc
  {
    src_type src_alloc;

    bool do_debug_printing;
    double nbytes_allocated;
    size_t nallocations;
    size_t nbytes_current;
    size_t nallocations_current;

    struct assertions
    {
      // this dummy array is here as a compile-time assertion about
      // sizeof(alloc_info<N>) -- if it doesn't match our
      // expectations, then dummy_array gets a negative array size,
      // which triggers a compile error:
      char assert_sizeof_alloc_info_must_be_N[(sizeof(alloc_info<N>)
                                               == N) ? 1 : -1];
    };

    aligned_alloc()
      :
      do_debug_printing(false),
      nbytes_allocated(0.0),
      nallocations(0),
      nbytes_current(0),
      nallocations_current(0)
    {}

    void set_debug(bool do_debug)
    {
      this->do_debug_printing = do_debug;

      // also call set_debug() on our child object
      src_alloc.set_debug(do_debug);
    }

    void set_allow_caching(bool on)
    {
      src_alloc.set_allow_caching(on);
    }

    void show_stats(int verbosity, const char* pfx,
                    const size_t block_size, const size_t overhead) const
    {
      LINFO("%s%saligned_alloc<%" ZU ">: "
            "all-time: [%fMB in %" ZU " allocations], "
            "current: [%fMB in %" ZU " allocations]",
            pfx ? pfx : "", pfx ? ": " : "",
            N,
            this->nbytes_allocated/(1024.0*1024.0), this->nallocations,
            this->nbytes_current/(1024.0*1024.0), this->nallocations_current);

      // also show stats for our child object
      src_alloc.show_stats(verbosity, pfx, block_size, overhead+2*N);
    }

    void* allocate(size_t user_nbytes)
    {
    GVX_TRACE(__PRETTY_FUNCTION__);

      alloc_info<N> info;
      info.data.source = 0;

      // We request extra space beyond what the user wants -- N extra
      // bytes allow us to return an address to the user that is
      // aligned to a N-byte boundary, and sizeof(alloc_info<N>) extra
      // bytes allow us to stick some auxiliary info just ahead of the
      // address that we return to the user. Note that for convenience
      // we have set things up so that sizeof(alloc_info<N>)==N, so
      // the number of extra bytes that we need is just 2*N.
      info.data.alloc_nbytes = user_nbytes+2*N;

      info.data.alloc_addr =
        this->src_alloc.allocate(info.data.alloc_nbytes, &info.data.source);

      /* We will set things up like this:

      +- address = return value of src_alloc.allocate()
      |
      |                                      +- address = return value
      |                                      |     of this function
      v                                      v

      +------------------+-------------------+------------------------+
      | len==adjust      | len==N            | len==user_nbytes       |
      | contents:        | contents:         | contents:              |
      |   unused         |   alloc_info<N>   |   empty space for user |
      | alignment:       | alignment:        | alignment:             |
      |   unknown        |   N-byte boundary |   N-byte boundary      |
      +------------------+-------------------+------------------------+

      */

      void* const user_addr = info.data.user_addr();

      static_cast<alloc_info<N>*>(user_addr)[-1].data = info.data;

      this->nbytes_allocated += info.data.alloc_nbytes;
      ++this->nallocations;
      this->nbytes_current += info.data.alloc_nbytes;
      ++this->nallocations_current;

      if (this->do_debug_printing)
        {
          info.data.print();
          this->show_stats(0, 0, 0, 0);
        }

      return user_addr;
    }

    void deallocate(void* user_addr)
    {
    GVX_TRACE(__PRETTY_FUNCTION__);

      const alloc_info<N>* const info =
        static_cast<alloc_info<N>*>(user_addr) - 1;

      this->nbytes_current -= info->data.alloc_nbytes;
      --this->nallocations_current;

      if (this->do_debug_printing)
        {
          info->data.print();
          this->show_stats(0, 0, 0, 0);
        }

      this->src_alloc.deallocate(info->data.alloc_addr, info->data.source);
    }

    void release_free_mem()
    {
      this->src_alloc.release_free_mem();
    }
  };

  /* Here are the various macros that you can twiddle if you need to
     change the allocation strategy. Basically you can have aligned
     allocation (DO_ALIGN) at an arbitrary N-byte boundary (NALIGN),
     with optional freelist caching (DO_FASTCACHE) of (NCACHE)
     commonly-requested memory sizes.

     If you turn off both DO_ALIGN and DO_FASTCACHE, you will end up
     using trivial_alloc, which is just a bare wrapper around operator
     new() and operator delete(). By default, malloc() returns 8-byte
     aligned memory on gnu/linux/x86 machines.

     Note that certain libraries (fftw [see FourierEngine] in
     particular) require greater than 8-byte alignment, so if you are
     going to be using those parts of the code, then you'll need to
     leave DO_ALIGN set, with NALIGN>=16. Also note that NALIGN must
     be at least 4*sizeof(void*) -- in particular, 16 will be too
     small on 64-bit systems for which sizeof(void*) is 8; for those
     systems we'll need NALIGN>=32.

     DO_FASTCACHE is here primarily for performance; since our memory
     usage pattern tends to involve many many allocations of Image
     objects with only a few different Dims shapes, it helps to cache
     those memory allocations in a freelist. Profiling tests showed
     that this can give a 15-20% speedup.
  */

#define DO_ALIGN
#define DO_FASTCACHE
#define NALIGN        ( (4*sizeof(void*)) > 16 ? (4*sizeof(void*)) : 16 )
#define NCACHE        64

#ifdef DO_ALIGN
#  ifdef DO_FASTCACHE
  typedef aligned_alloc<fastcache_alloc<NCACHE>, NALIGN> g_alloc_type;
#  else
  typedef aligned_alloc<trivial_alloc, NALIGN>           g_alloc_type;
#  endif
#else // !DO_ALIGN
  typedef trivial_alloc                                  g_alloc_type;
#endif

  // Here is our global allocator object, whose type is determined by
  // the various macro settings abovve, and a corresponding mutex. For
  // now, we use a heavy-handed approach and just use the mutex to
  // lock the entire structure during each call to any of the public
  // functions. If this turns out to be a performance problem, we
  // could turn to finer-grained locking within the various allocator
  // classes themselves.
  g_alloc_type    g_alloc;
  pthread_mutex_t g_alloc_mutex = PTHREAD_MUTEX_INITIALIZER;

  size_t          g_stats_units = 0;
}

void* invt_allocate_aux(size_t user_nbytes)
{
  GVX_MUTEX_LOCK(&g_alloc_mutex);
  return g_alloc.allocate(user_nbytes);
}

void invt_deallocate_aux(void* mem)
{
  GVX_MUTEX_LOCK(&g_alloc_mutex);
  g_alloc.deallocate(mem);
}

void invt_allocation_release_free_mem()
{
  GVX_MUTEX_LOCK(&g_alloc_mutex);
  g_alloc.release_free_mem();
}

void invt_allocation_allow_caching(bool on)
{
  GVX_MUTEX_LOCK(&g_alloc_mutex);
  g_alloc.set_allow_caching(on);
}

void invt_allocation_debug_print(bool do_debug)
{
  GVX_MUTEX_LOCK(&g_alloc_mutex);
  g_alloc.set_debug(do_debug);
}

void invt_allocation_show_stats(int verbosity, const char* pfx,
                                const size_t block_size)
{
  GVX_MUTEX_LOCK(&g_alloc_mutex);
  g_alloc.show_stats(verbosity, pfx,
                     block_size ? block_size : g_stats_units, 0);
}

void invt_allocation_set_stats_units(const size_t units)
{
  g_stats_units = units;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_ALLOCAUX_C_DEFINED
