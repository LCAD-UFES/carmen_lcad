/** @file rutz/compat_cmath.h Compatibility definitions for things missing from <cmath> */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Mon Jan 16 18:16:40 2006
// commit: $Id: compat_cmath.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/compat_cmath.h $
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

#ifndef GROOVX_RUTZ_COMPAT_CMATH_H_UTC20060117021640_DEFINED
#define GROOVX_RUTZ_COMPAT_CMATH_H_UTC20060117021640_DEFINED

#include <cmath>

namespace rutz
{
  double compat_m_pi();
  double compat_m_pi_2();
  double compat_m_e();
  double compat_m_ln2();
}


#ifdef MISSING_M_PI
#define                M_PI     (rutz::compat_m_pi())
#endif

#ifdef MISSING_M_PI_2
#define                M_PI_2   (rutz::compat_m_pi_2())
#endif

#ifdef MISSING_M_E
#define                M_E      (rutz::compat_m_e())
#endif

#ifdef MISSING_M_LN2
#define                M_LN2    (rutz::compat_m_ln2())
#endif

#ifdef MISSING_CBRT
#define                cbrt(x)  (pow(x, 1.0/3.0))
#endif

#ifdef MISSING_ISNAN
#  if defined(HAVE_STD_ISNAN)
template <class T> inline
int                    isnan(T x) { return std::isnan(x); }
#  endif
#endif

static const char __attribute__((used)) vcid_groovx_rutz_compat_cmath_h_utc20060117021640[] = "$Id: compat_cmath.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/compat_cmath.h $";
#endif // !GROOVX_RUTZ_COMPAT_CMATH_H_UTC20060117021640DEFINED
