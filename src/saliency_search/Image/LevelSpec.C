/*!@file Image/LevelSpec.C a utility class for use with SingleChannel */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LevelSpec.C $
// $Id: LevelSpec.C 5773 2005-10-20 21:45:34Z rjpeters $
//

#include "Image/LevelSpec.H"

#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "Util/log.H"

#include <sstream>
#include <stdio.h>

// ######################################################################
// ######################################################################
// LevelSpec member definitions:
// ######################################################################
// ######################################################################

// ######################################################################
LevelSpec::LevelSpec() :
  itsLevMin(0), itsLevMax(0), itsDelMin(0), itsDelMax(0), itsMapLevel(0),
  itsMaxIndex(0), itsMaxDepth(0)
{ }

// ######################################################################
LevelSpec::LevelSpec(const uint levmin, const uint levmax,
                     const uint deltmin, const uint deltmax,
                     const uint maplevel) :
  itsLevMin(levmin),
  itsLevMax(levmax),
  itsDelMin(deltmin),
  itsDelMax(deltmax),
  itsMapLevel(maplevel),
  itsMaxIndex( (deltmax-deltmin+1) * (levmax-levmin+1) ),
  itsMaxDepth( levmax + deltmax + 1 )
{
  ASSERT(levmax >= levmin); ASSERT(deltmax >= deltmin);
}

// ######################################################################
void LevelSpec::init(const uint levmin, const uint levmax,
                     const uint deltmin, const uint deltmax,
                     const uint maplevel)
{
  ASSERT(levmax >= levmin); ASSERT(deltmax >= deltmin);

  itsLevMin = levmin; itsLevMax = levmax; itsDelMin = deltmin;
  itsDelMax = deltmax; itsMapLevel = maplevel;
  itsMaxIndex = (deltmax-deltmin+1) * (levmax-levmin+1);
  itsMaxDepth = levmax + deltmax + 1;
}

// ######################################################################
uint LevelSpec::levMin() const
{ return itsLevMin; }

// ######################################################################
uint LevelSpec::levMax() const
{ return itsLevMax; }

// ######################################################################
uint LevelSpec::delMin() const
{ return itsDelMin; }

// ######################################################################
uint LevelSpec::delMax() const
{ return itsDelMax; }

// ######################################################################
uint LevelSpec::mapLevel() const
{ return itsMapLevel; }

// ######################################################################
uint LevelSpec::maxIndex() const
{ return itsMaxIndex; }

// ######################################################################
uint LevelSpec::maxDepth() const
{ return itsMaxDepth; }

// ######################################################################
bool LevelSpec::indexOK(uint index) const
{ return (index < itsMaxIndex); }

// ######################################################################
bool LevelSpec::clevOK(uint centerlev) const
{ return (centerlev >= itsLevMin && centerlev <= itsLevMax); }

// ######################################################################
bool LevelSpec::slevOK(uint surrlev) const
{
  return (surrlev >= (itsLevMin + itsDelMin) &&
          surrlev <= (itsLevMax + itsDelMax));
}

// ######################################################################
bool LevelSpec::delOK(uint delta) const
{ return (delta >= itsDelMin && delta <= itsDelMax); }

// ######################################################################
bool LevelSpec::csOK(uint centerlev, uint surroundlev) const
{
  // Note that the delOK() call will implicitly check that
  // (centerlev<surroundlev)
//   printf("clevOK: %d, slevOK %d, delOK: %d\n",
//          clevOK(centerlev), slevOK(surroundlev),  delOK(surroundlev - centerlev));


  return clevOK(centerlev) && slevOK(surroundlev)
    && delOK(surroundlev - centerlev);
}

// ######################################################################
uint LevelSpec::csToIndex(uint centerlev, uint surroundlev) const
{
  ASSERT( csOK(centerlev, surroundlev) );

  const uint delta = surroundlev - centerlev;

  return centerlev - itsLevMin +
    (delta - itsDelMin) * (itsLevMax - itsLevMin + 1);
}

// ######################################################################
void LevelSpec::indexToCS(uint index, uint& centerlev, uint& surroundlev) const
{
  ASSERT( indexOK(index) );

  centerlev = index % (itsLevMax - itsLevMin + 1) + itsLevMin;
  uint delta = index / (itsLevMax - itsLevMin + 1) + itsDelMin;

  surroundlev  = centerlev + delta;

  ASSERT( csOK(centerlev, surroundlev) );
}

// ######################################################################
bool LevelSpec::operator==(const LevelSpec& that) const
{
  return (this->itsLevMin == that.itsLevMin
          && this->itsLevMax == that.itsLevMax
          && this->itsDelMin == that.itsDelMax
          && this->itsDelMax == that.itsDelMax
          && this->itsMapLevel == that.itsMapLevel
          && this->itsMaxIndex == that.itsMaxIndex
          && this->itsMaxDepth == that.itsMaxDepth);
}


// ######################################################################
// converters for LevelSpec
// ######################################################################

std::string convertToString(const LevelSpec& val)
{
  std::stringstream s;
  s<<val.levMin()<<'-'<<val.levMax()<<','<<val.delMin()<<'-'<<
    val.delMax()<<','<<val.mapLevel();
  return s.str();
}

void convertFromString(const std::string& str, LevelSpec& val)
{
  std::stringstream s; int lmi = -2, lma = -2, dmi = -2, dma = -2, ml = -2;
  char c; s<<str; s>>lmi>>c>>lma>>c>>dmi>>c>>dma>>c>>ml;
  if (lmi == -2 || lma == -2 || dmi == -2 || dma == -2 || ml == -2)
    conversion_error::raise<LevelSpec>(str);

  val.init(lmi, lma, dmi, dma, ml);
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
