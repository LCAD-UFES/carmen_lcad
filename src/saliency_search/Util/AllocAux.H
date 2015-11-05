/*!@file Util/AllocAux.H memory allocation routines for 16-byte alignment */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/AllocAux.H $
// $Id: AllocAux.H 9602 2008-04-10 00:16:46Z rjpeters $
//

#ifndef UTIL_ALLOCAUX_H_DEFINED
#define UTIL_ALLOCAUX_H_DEFINED

#include <cstdlib> // for size_t

/// Allocate nbytes of memory, throwing an exception in case of failure.
void* invt_allocate_aux(size_t nbytes);

/// Deallocate the given memory region.
void invt_deallocate_aux(void* mem);

/// Released any cached free memory blocks (e.g. to conserve memory)
void invt_allocation_release_free_mem();

/// Turn on/off caching of memory blocks of commonly-used sizes
void invt_allocation_allow_caching(bool on);

/// Turn on/off memory-allocation debugging statements.
/*! If on, each invt_allocate() and invt_deallocate() call will print
    some stats about the memory block being currently
    allocated/deallocated, as well as some general stats about the
    total amount of memory allocated and the amount of memory
    currently in use */
void invt_allocation_debug_print(bool print);

/// Print current stats for whatever memory allocator is in use.
/** @param verbosity If greater than 0, will request additional stats
    to be displayed

    @param pfx If non-null, will be printed at the beginning of each
    stats line.

    @param units If non-zero, allocated memory blocks will be listed
    as multiples of this number of bytes; otherwise memory blocks will
    be listed as multiples of the units specified in the most recent
    call to invt_allocation_set_stats_units().
*/
void invt_allocation_show_stats(int verbosity = 0,
                                const char* pfx = 0,
                                const size_t units = 0);

/// Set the units to be used by default in invt_allocation_show_stats()
void invt_allocation_set_stats_units(const size_t units);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_ALLOCAUX_H_DEFINED
