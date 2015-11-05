/*!@file SpaceVariant/SVChanLevels.H Class to represent a location in a
   StrucureModule or Layer. */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SVChanLevels.H $

#ifndef SPACEVARIANT_SVCHANLEVELS_H_DEFINED
#define SPACEVARIANT_SVCHANLEVELS_H_DEFINED

#include "Util/log.H"
#include <vector>

// ######################################################################
// hold channel level info
// ######################################################################
class SVChanLevels 
{
public:
  SVChanLevels() { };

  //!set SVChanLevels for various levels
  SVChanLevels(const std::vector<float>& levels);

  //!destructor
  ~SVChanLevels() { };

  //operator ==
  bool operator==(const SVChanLevels& rhs);

  //!get the number of dimensions
  uint numLevels() const;

  //!get the value at N'th pos
  float getVariance(const uint pos) const;

  //!get the value at N'th pos
  float getMaxLevel() const;

private:
  std::vector<float> itsLevels;
};

// ######################################################################
// free functions
// ######################################################################
//! format is "[<float>,<float>..."
std::string convertToString(const SVChanLevels& val);

//! format is "[<float>,<float>..."
void convertFromString(const std::string& str, SVChanLevels& val);


// ######################################################################
// implementation of location
// ######################################################################
inline
SVChanLevels::SVChanLevels(const std::vector<float>& levels) : itsLevels(levels)
{ }

// ######################################################################
inline
bool SVChanLevels::operator==(const SVChanLevels& rhs)
{
  return (itsLevels == rhs.itsLevels) ? true : false;
}

// ######################################################################
inline
uint SVChanLevels::numLevels() const
{
  return (uint)itsLevels.size();
}

// ###################################################################### 
inline
float SVChanLevels::getVariance(const uint pos) const
{
  return itsLevels[pos];
}

// ###################################################################### 
inline
float SVChanLevels::getMaxLevel() const
{
  return itsLevels[numLevels()-1];
}

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
