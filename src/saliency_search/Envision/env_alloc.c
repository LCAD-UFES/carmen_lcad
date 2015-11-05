/*!@file Envision/env_alloc.c memory allocation routines for 16-byte alignment */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_alloc.c $
// $Id: env_alloc.c 8338 2007-05-04 00:59:03Z rjpeters $
//

#ifndef ENVISION_ENV_ALLOC_C_DEFINED
#define ENVISION_ENV_ALLOC_C_DEFINED

#include "Envision/env_alloc.h"

#include "Envision/env_log.h"

#define NALIGN ((env_size_t)((4*sizeof(void*)) > 16 ? (4*sizeof(void*)) : 16))

static env_alloc_func* g_raw_alloc = 0;
static env_dealloc_func* g_raw_dealloc = 0;
static void* g_mutex_userdata = 0;
static env_alloc_mutex_func* g_mutex_acquire = 0;
static env_alloc_mutex_func* g_mutex_release = 0;

// ######################################################################
/// Free-node class for free-list memory pools.
struct free_list_node
{
        struct free_list_node* next;
};

// ######################################################################
/// Base class for maintaining a free-list memory pool.
struct free_list
{
        struct free_list_node* node_list;
        env_size_t num_allocations;
        env_size_t alloc_size;
        env_size_t num_active;
};

// ######################################################################
//! Auxiliary information about an aligned memory allocation
struct alloc_info
{
        // address of ALLOCATED memory, not the address returned to
        // the user:
        void* alloc_addr;

        // the freelist from which this memory was allocated, or else
        // null:
        struct free_list* source;

        // number of bytes ALLOCATED, not the number requested by the
        // user:
        env_size_t alloc_nbytes;

        char pad[NALIGN
                 - sizeof(void*)
                 - sizeof(struct free_list*)
                 - sizeof(env_size_t)];
};

// ######################################################################
//! Allocate memory that is aligned on an NALIGN-byte boundary
struct aligned_alloc
{
        struct free_list cache[ENV_NCACHE];

        unsigned long long nbytes_alltime;
        env_size_t nallocations_alltime;
        env_size_t nbytes_current;
        env_size_t nallocations_current;
};

// ######################################################################
// this dummy array is here as a compile-time assertion about
// sizeof(struct alloc_info) -- if it doesn't match our expectations,
// then dummy_array gets a negative array size, which triggers a
// compile error:
typedef char
sizeof_alloc_info_must_equal_NALIGN[(sizeof(struct alloc_info) == NALIGN)
                                    ? 1 : -1];

// ######################################################################
static void env_free_list_init(struct free_list* p, env_size_t size_check)
{
        ENV_ASSERT(p->node_list == 0);
        ENV_ASSERT(size_check >= sizeof(struct free_list_node));

        p->num_allocations = 0;
        p->alloc_size = size_check;
        p->num_active = 0;
}

// ######################################################################
/// Allocate space for a new object.
/** If there are chunks available in the free list, one of those is
    returned; otherwise new memory is allocated with
    (*g_raw_alloc). */
static void* env_free_list_allocate(struct free_list* p, env_size_t bytes)
{
        ENV_ASSERT(bytes == p->alloc_size);
        if (p->node_list == 0)
        {
                ++p->num_allocations;
                ++p->num_active;
                ENV_ASSERT(g_raw_alloc != 0);
                void* const result = (*g_raw_alloc)(bytes);
                ENV_ASSERT2(result != 0, "Memory allocation failed");
                return result;
        }
        struct free_list_node* n = p->node_list;
        p->node_list = p->node_list->next;
        ++p->num_active;
        return (void*) n;
}

// ######################################################################
/// Return an object to the free list.
static void env_free_list_deallocate(struct free_list* p, void* space)
{
        struct free_list_node* n = (struct free_list_node*) space;
        n->next = p->node_list;
        p->node_list = n;
        --p->num_active;
}

// ######################################################################
static void env_free_list_cleanup(struct free_list* p)
{
        ENV_ASSERT(p->num_active == 0);

        while (p->node_list)
        {
                void* space = p->node_list;
                p->node_list = p->node_list->next;
                ENV_ASSERT(g_raw_dealloc != 0);
                (*g_raw_dealloc)(space);
                --p->num_allocations;
        }

        ENV_ASSERT(p->num_allocations == 0);
        ENV_ASSERT(p->node_list == 0);

        // now we are back to a pristine state, so forget our size:
        p->alloc_size = 0;
}

// ######################################################################
static void env_aligned_alloc_init(struct aligned_alloc* p)
{
        p->nbytes_alltime = 0;
        p->nallocations_alltime = 0;
        p->nbytes_current = 0;
        p->nallocations_current = 0;

        for (env_size_t i = 0; i < ENV_NCACHE; ++i)
        {
                p->cache[i].node_list = 0;
                p->cache[i].num_allocations = 0;
                p->cache[i].alloc_size = 0;
                p->cache[i].num_active = 0;
        }
}

// ######################################################################
static void env_aligned_alloc_get_stats(const struct aligned_alloc* p,
                                        struct env_alloc_stats* stats)
{
        stats->nalign = NALIGN;
        stats->ncache_used = 0;
        stats->nbytes_alltime = p->nbytes_alltime;
        stats->nallocations_alltime = p->nallocations_alltime;
        stats->nbytes_current = p->nbytes_current;
        stats->nallocations_current = p->nallocations_current;
        stats->bytes_allocated = 0;
        stats->overhead = 2*NALIGN;

        for (env_size_t i = 0; i < ENV_NCACHE; ++i)
                if (p->cache[i].alloc_size > 0)
                {
                        ++stats->ncache_used;
                        const env_size_t nb =
                                (p->cache[i].num_allocations
                                 * p->cache[i].alloc_size);

                        stats->bytes_allocated += nb;

                        stats->cache[i].num_allocations =
                                p->cache[i].num_allocations;
                        stats->cache[i].alloc_size =
                                p->cache[i].alloc_size;
                        stats->cache[i].num_active =
                                p->cache[i].num_active;
                }
}

// ######################################################################
static void* env_aligned_alloc_allocate(struct aligned_alloc* p, env_size_t user_nbytes)
{
        struct alloc_info info = { 0, 0, 0, { 0 } };
        info.source = 0;

        // We request extra space beyond what the user wants -- NALIGN
        // extra bytes allow us to return an address to the user that
        // is aligned to a NALIGN-byte boundary, and sizeof(struct
        // alloc_info) extra bytes allow us to stick some auxiliary
        // info just ahead of the address that we return to the
        // user. Note that for convenience we have set things up so
        // that sizeof(struct alloc_info)==NALIGN, so the number of
        // extra bytes that we need is just 2*NALIGN.
        info.alloc_nbytes = user_nbytes+2*NALIGN;
        info.alloc_addr = 0;

        for (env_size_t i = 0; i < ENV_NCACHE; ++i)
        {
                if (p->cache[i].alloc_size > 0)
                {
                        // we found a filled slot, but if it doesn't
                        // match our requested size then we just skip
                        // over it:
                        if (p->cache[i].alloc_size != info.alloc_nbytes)
                                continue;
                }
                else
                {
                        // we found an empty slot, so let's initialize
                        // a new free list for our requested size:
                        env_free_list_init(&p->cache[i], info.alloc_nbytes);
                }

                // now, one way or the other we know that the slot
                // matches our requested size, so go ahead and request
                // an allocation:
                info.source = &p->cache[i];
                info.alloc_addr =
                        env_free_list_allocate(&p->cache[i], info.alloc_nbytes);
                break;
        }

        if (info.alloc_addr == 0)
        {
                info.source = 0;
                ENV_ASSERT(g_raw_alloc != 0);
                info.alloc_addr = (*g_raw_alloc)(info.alloc_nbytes);
                ENV_ASSERT2(info.alloc_addr != 0,
                            "Memory allocation failed");
        }

        /* We will set things up like this:

        +- address = value of info.alloc_addr
        |
        |                                        +- address = return value
        |                                        |     of this function
        v                                        v

        +---------------+------------------------+------------------------+
        | len==adjust   | len==NALIGN            | len==user_nbytes       |
        | contents:     | contents:              | contents:              |
        |   unused      |   struct alloc_info    |   empty space for user |
        | alignment:    | alignment:             | alignment:             |
        |   unknown     |   NALIGN-byte boundary |   NALIGN-byte boundary |
        +---------------+------------------------+------------------------+

        */

        void* const user_addr =
                ((char*) info.alloc_addr)
                + (2*NALIGN)
                - ((unsigned long) info.alloc_addr) % NALIGN;

        ((struct alloc_info*) user_addr)[-1] = info;

        p->nbytes_alltime += info.alloc_nbytes;
        ++p->nallocations_alltime;
        p->nbytes_current += info.alloc_nbytes;
        ++p->nallocations_current;

        return user_addr;
}

// ######################################################################
static void env_aligned_alloc_deallocate(struct aligned_alloc* p, void* user_addr)
{
        const struct alloc_info* const info =
                ((struct alloc_info*) user_addr) - 1;

        p->nbytes_current -= info->alloc_nbytes;
        --p->nallocations_current;

        // deallocate memory from the given free_list, otherwise free it
        // globally
        if (info->source != 0)
        {
                env_free_list_deallocate(info->source, info->alloc_addr);
        }
        else
        {
                ENV_ASSERT(g_raw_dealloc != 0);
                (*g_raw_dealloc)(info->alloc_addr);
        }
}

// ######################################################################
static void env_aligned_alloc_cleanup(struct aligned_alloc* p)
{
        for (env_size_t i = 0; i < ENV_NCACHE; ++i)
                env_free_list_cleanup(&p->cache[i]);
}

// ######################################################################
struct aligned_alloc g_alloc;

// ######################################################################
void* env_allocate(env_size_t user_nbytes)
{
        if (g_mutex_acquire) (*g_mutex_acquire)(g_mutex_userdata);
        void* ret = env_aligned_alloc_allocate(&g_alloc, user_nbytes);
        if (g_mutex_release) (*g_mutex_release)(g_mutex_userdata);
        return ret;
}

// ######################################################################
void env_deallocate(void* mem)
{
        if (mem)
        {
                if (g_mutex_acquire) (*g_mutex_acquire)(g_mutex_userdata);
                env_aligned_alloc_deallocate(&g_alloc, mem);
                if (g_mutex_release) (*g_mutex_release)(g_mutex_userdata);
        }
}

// ######################################################################
void env_allocation_get_stats(struct env_alloc_stats* stats)
{
        if (g_mutex_acquire) (*g_mutex_acquire)(g_mutex_userdata);
        env_aligned_alloc_get_stats(&g_alloc, stats);
        if (g_mutex_release) (*g_mutex_release)(g_mutex_userdata);
}

// ######################################################################
void env_allocation_init(env_alloc_func* alloc_func,
                         env_dealloc_func* dealloc_func)
{
        if (g_mutex_acquire) (*g_mutex_acquire)(g_mutex_userdata);

        g_raw_alloc = alloc_func;
        g_raw_dealloc = dealloc_func;

        env_aligned_alloc_init(&g_alloc);
        if (g_mutex_release) (*g_mutex_release)(g_mutex_userdata);
}

// ######################################################################
void env_allocation_cleanup(void)
{
        if (g_mutex_acquire) (*g_mutex_acquire)(g_mutex_userdata);
        env_aligned_alloc_cleanup(&g_alloc);
        if (g_mutex_release) (*g_mutex_release)(g_mutex_userdata);
}

// ######################################################################
void env_allocation_init_mutex_funcs(void* userdata,
                                     env_alloc_mutex_func* mutex_acquire,
                                     env_alloc_mutex_func* mutex_release)
{
        g_mutex_userdata = userdata;
        g_mutex_acquire = mutex_acquire;
        g_mutex_release = mutex_release;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_ALLOC_C_DEFINED
