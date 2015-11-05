/*!@file Media/FrameRange.H A range of frames at a given inter-frame delay */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FrameRange.H $
// $Id: FrameRange.H 13273 2010-04-21 22:08:00Z rand $
//

#ifndef FRAMERANGE_H_DEFINED
#define FRAMERANGE_H_DEFINED

#include "Util/Assert.H"
#include "Util/SimTime.H"
#include <string>
#include <vector>

//! Specification of a range of frames with given inter-frame delay
class FrameRange
{
public:
  //! Default-construct as "0-1-0@0"
  FrameRange();

  //! Constructor 
  FrameRange(int first, int step, int last, 
      std::vector<SimTime> delays=std::vector<SimTime>(),
      bool eventTriggered=false);

  //! Convert to string
  std::string toString() const;

  //! Pseudo-constructor; convert from string
  static FrameRange fromString(const std::string& s);

  //! Equality operator
  bool operator==(const FrameRange& that) const;

  //! Inequality operator
  bool operator!=(const FrameRange& that) const
  { return !(*this == that); }

  int getFirst() const { return itsFirst; }

  int getStep() const { return itsStep; }

  int getLast() const { return itsLast; }

  size_t numDelayTimes() const { return itsDelayTimes.size(); }

  SimTime getDelayTime(size_t idx) const
  {
    ASSERT(idx < itsDelayTimes.size());
    return itsDelayTimes[idx];
  }

  bool isEventTriggered() const { return itsEventTriggered; }

private:
  int itsFirst;             //!< number of first frame
  int itsStep;              //!< number of frames to step with each update (default=1)
  int itsLast;              //!< number of last frame
  std::vector<SimTime> itsDelayTimes; //!< inter-frame delay (single element) or elapsed time from clip onset (multiple elements)
  bool itsEventTriggered; //!< trigger on events rather than on a clock?
};

//! FrameRange overload: format is "first-step-last@delay"
inline std::string convertToString(const FrameRange& val)
{ return val.toString(); }

//! FrameRange overload: format is "[[first[-step]]-[last]]@delay"
/*! If the entire "first-step-list" range is missing, then we assume
    0-INT_MAX. If just "last" is missing (i.e., we got "first-"), then
    we assume last=INT_MAX. If just "first" is missing, then we assume
    first=0. If "step" is missing, we assume step=1. */
inline void convertFromString(const std::string& str, FrameRange& val)
{ val = FrameRange::fromString(str); }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
