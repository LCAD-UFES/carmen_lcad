/*!@file Image/LevelSpec.H a utility class for use with SingleChannel */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LevelSpec.H $
// $Id: LevelSpec.H 5773 2005-10-20 21:45:34Z rjpeters $
//

#ifndef LEVELSPEC_H_DEFINED
#define LEVELSPEC_H_DEFINED

#include "Util/Types.H" // for uint

#include <string> // for string conversions

// ######################################################################
//! A utility class for use with SingleChannel.
/*! LevelSpec specifies a set of center/surround pairings, by tracking two
    min/max ranges: one for <i>level</a>s, and one for <i>delta</i>s. Note
    that both of these ranges are <i>inclusive</i> ranges; both the min and
    max values are valid elements. The levels refer to depth levels in a
    Pyramid, and the deltas refer to differences between the center and
    surround levels in a centerSurround() operation. LevelSpec also maps
    these ranges into a singe range 0..maxIndex(), so that all pairings of
    level/delta can be accessed in a 1-D array, for example. */
class LevelSpec
{
public:
  //! Unitialized constructor, all internal values to 0
  LevelSpec();

  //! Constructor
  LevelSpec(const uint levmin, const uint levmax,
            const uint deltmin, const uint deltmax,
            const uint maplevel);

  //! (re-)initialization
  void init(const uint levmin, const uint levmax,
            const uint deltmin, const uint deltmax,
            const uint maplevel);

  // Default copy-constructor OK

  //! Get the minimum center level (inclusive).
  uint levMin() const;
  //! Get the maximum center level (inclusive).
  uint levMax() const;
  //! Get the minimum surround minus center level difference (inclusive).
  uint delMin() const;
  //! Get the maximum surround minus center level difference (inclusive).
  uint delMax() const;

  //! Get the level at which the feature map is built.
  uint mapLevel() const;

  //! Get the maximum index that can be returned from csToIndex().
  uint maxIndex() const;
  //! Get the maximum surround level that will be used.
  uint maxDepth() const;

  //! Query if the given array index is valid for this object.
  bool indexOK(uint index) const;

  //! Query if the given center level is valid for this object.
  bool clevOK(uint centerlev) const;

  //! Query if the given surround level is valid for this object.
  bool slevOK(uint surroundlev) const;

  //! Query if the given surround-center delta is valid for this object.
  bool delOK(uint delta) const;

  //! Query if the given center/surround levels are valid for this object.
  /*! The "cs" prefix is a mnemonic to indicate that the center argument
      precedes the surround argument. */
  bool csOK(uint centerlev, uint surroundlev) const;

  //! Compute an array index for a given center/surround pairing.
  /*! Return the array index in 0..maxIndex that corresponds to the given
      center/surround levels. The "cs" prefix is a mnemonic to indicate
      that the center argument precedes the surround argument. */
  uint csToIndex(uint centerlev, uint surroundlev) const;

  //! Compute the center/surround pairing corresponding to an array index.
  void indexToCS(uint index, uint& centerlev, uint& surroundlev) const;

  //! Equality operator
  bool operator==(const LevelSpec& that) const;

  //! Inequality operator
  bool operator!=(const LevelSpec& that) const
  { return !(*this == that); }

private:
  //! levelMin <= level <= levelMax
  uint itsLevMin;
  //! levelMin <= level <= levelMax
  uint itsLevMax;
  //! deltaMin <= delta <= deltaMax
  uint itsDelMin;
  //! deltaMin <= delta <= deltaMax
  uint itsDelMax;
  //! the pyramid level at which the feature map is taken
  uint itsMapLevel;
  //! 0 <= index(.,.) < maxindex
  uint itsMaxIndex;
  //! depth of pyramids
  uint itsMaxDepth;
};

//! LevelSpec overload: format is "<int>-<int>,<int>-<int>,<int>"
/*! Format corresponds to "levmin-levmax,delmin-delmax,maplevel" */
std::string convertToString(const LevelSpec& val);

//! LevelSpec overload: format is "<int>-<int>,<int>-<int>,<int>"
void convertFromString(const std::string& str, LevelSpec& val);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !LEVELSPEC_H_DEFINED
