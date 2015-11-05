/*!@file Envision/env_alloc.h memory allocation routines for 16-byte alignment */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_alloc.h $
// $Id: env_alloc.h 8338 2007-05-04 00:59:03Z rjpeters $
//

#ifndef ENVISION_ENV_ALLOC_H_DEFINED
#define ENVISION_ENV_ALLOC_H_DEFINED

#include "Envision/env_types.h"

#define ENV_NCACHE ((env_size_t)32)

struct env_alloc_stats
{
        env_size_t nalign;
        env_size_t ncache_used;
        unsigned long long nbytes_alltime;
        env_size_t nallocations_alltime;
        env_size_t nbytes_current;
        env_size_t nallocations_current;
        env_size_t bytes_allocated;
        env_size_t overhead;

        struct
        {
                env_size_t num_allocations;
                env_size_t alloc_size;
                env_size_t num_active;
        } cache[ENV_NCACHE];
};

#ifdef __cplusplus
extern "C"
{
#endif

        typedef void* (env_alloc_func)(env_size_t);
        typedef void (env_dealloc_func)(void*);

        /// Allocate nbytes of memory, throwing an exception in case of failure.
        void* env_allocate(env_size_t nbytes);

        /// Deallocate the given memory region.
        void env_deallocate(void* mem);

        /// Get current memory allocator stats
        void env_allocation_get_stats(struct env_alloc_stats* stats);

        /// Initialize memory allocator
        /** @param alloc_func pointer to some function that allocates
            raw memory (e.g. something that wraps malloc())

            @param dealloc_func pointer to some function that
            deallocates raw memory (e.g. free())
         */
        void env_allocation_init(env_alloc_func* alloc_func,
                                 env_dealloc_func* dealloc_func);

        /// Release any cached memory blocks
        void env_allocation_cleanup(void);

        typedef void (env_alloc_mutex_func)(void*);

        /// Install callbacks to provide thread-safety if desired
        /** @param mutex_acquire this function will be called at the
            beginning of every env_allocate(), env_deallocate(),
            env_allocation_get_stats(), env_allocation_init(), and
            env_allocation_cleanup() call

            @param mutex_release this function will be called at the
            end of every such call

            It is not necessary to install any callbacks in a program
            where env_allocate() and env_deallocate() are always
            called from just a single thread; but any program that
            might use those functions concurrently from multiple
            threads must install mutex functions here. */
        void env_allocation_init_mutex_funcs(void* userdata,
                                             env_alloc_mutex_func* mutex_acquire,
                                             env_alloc_mutex_func* mutex_release);

#ifdef __cplusplus
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* c-file-style: "linux" */
/* indent-tabs-mode: nil */
/* End: */

#endif // ENVISION_ENV_ALLOC_H_DEFINED
