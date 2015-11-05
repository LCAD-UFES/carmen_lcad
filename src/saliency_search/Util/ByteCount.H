/*!@file Util/ByteCount.H A class to handle byte counts, parsing them
  from strings, and converting between bytes, kibibytes, mebibytes,
  etc. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/ByteCount.H $
// $Id: ByteCount.H 15310 2012-06-01 02:29:24Z itti $
//

#ifndef UTIL_BYTECOUNT_H_DEFINED
#define UTIL_BYTECOUNT_H_DEFINED

#include "Util/sformat.H"

#include <string>

//! A class to handle parsing + converting byte counts
class ByteCount
{
public:
  //! Construct with a number of bytes
  ByteCount(size_t nbytes = 0) : itsCount(nbytes) {}

  //! Construct by parsing a string: format is "<float>[<B|KiB|MiB|GiB>]"
  /*! If there is no suffix, we assume that the value is a literal
      byte count. Otherwise, the suffix can be B, KiB, MiB, GiB to
      specify a multiplier of 1, 2^10, 2^20, or 2^30, respectively. */
  ByteCount(const std::string& str);

  //! Return a pretty-printed string using the highest possible suffix
  /*! NOTE that if you convert to string using prettyPrint() and then
      rescan that string back into a ByteCount, there will almost
      certainly be inaccuracy due to floating point roundoff. */
  std::string prettyPrint() const;

  // NOTE: there is no special function to change the number of bytes,
  // however you can use the assignment operator along with a
  // constructed temporary, e.g. ByteCount x; ... ;  x=ByteCount(10);

  //! number of bytes represented
  size_t bytes() const { return itsCount; }

  //! Returns bytes()/1024.0
  /*! To avoid confusion we use kibibytes (KiB) instead of kilobytes,
      since 'kilo' strictly means 10^3 but is often also used to refer
      to 2^10. On the other hand, 'kibi' unequivocally refers to
      2^10. */
  double kibibytes() const { return itsCount/1024.0; }

  //! Returns bytes()/(1024.0*1024.0)
  /*! To avoid confusion we use mebibytes (MiB) instead of megabytes,
      since 'mega' strictly means 10^6 but is often also used to refer
      to 2^20. On the other hand, 'mebi' unequivocally refers to
      2^20. */
  double mebibytes() const { return itsCount/(1024.0*1024.0); }

  //! Returns bytes()/(1024.0*1024.0*1024.0)
  /*! To avoid confusion we use gibibytes (GiB) instead of gigabytes,
      since 'giga' strictly means 10^9 but is often also used to refer
      to 2^30. On the other hand, 'gibi' unequivocally refers to
      2^30. */
  double gibibytes() const { return itsCount/(1024.0*1024.0*1024.0); }

  //! synonym for bytes()
  size_t B() const { return bytes(); }

  //! synonym for kibibytes()
  double KiB() const { return kibibytes(); }

  //! synonym for mebibytes()
  double MiB() const { return mebibytes(); }

  //! synonym for gibibytes()
  double GiB() const { return gibibytes(); }

  //! Equality operator
  bool operator==(const ByteCount& that) const
  { return (this->itsCount == that.itsCount); }

  //! Inequality operator
  bool operator!=(const ByteCount& that) const
  { return (this->itsCount != that.itsCount); }

private:
  size_t itsCount;
};

//! ByteCount overload: format is "<int>"
/*! Note that we don't do any abbreviations to KiB, Mib, GiB, etc. on
    output, since that would involve a conversion to floating point
    that might result in data loss after a conversion to string and
    back. If you want such an abbreviated string, you can use
    ByteCount::prettyPrint(). */
inline std::string convertToString(const ByteCount& val)
{
  return sformat("%" ZU , val.bytes());
}

//! ByteCount overload: format is "<float>[<B|KiB|MiB|GiB>]"
inline void convertFromString(const std::string& str, ByteCount& val)
{
  val = ByteCount(str);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_BYTECOUNT_H_DEFINED
