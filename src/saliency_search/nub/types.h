/** @file nub/types.h enum types for reference-counted handles */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Nov 16 08:57:40 2005
// commit: $Id: types.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/types.h $
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

#ifndef GROOVX_NUB_TYPES_H_UTC20051116165740_DEFINED
#define GROOVX_NUB_TYPES_H_UTC20051116165740_DEFINED

namespace nub
{
  enum ref_type { WEAK, STRONG };

  enum ref_vis
    {
      DEFAULT,   //! equivalent to result of get_default_ref_vis()
      PRIVATE,   //! nub::objectdb gets no reference to the object
      PROTECTED, //! nub::objectdb gets a weak reference to the object
      PUBLIC     //! nub::objectdb gets a strong reference to the object
    };
}

static const char __attribute__((used)) vcid_groovx_nub_types_h_utc20051116165740[] = "$Id: types.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/types.h $";
#endif // !GROOVX_NUB_TYPES_H_UTC20051116165740DEFINED
