/*!@file Media/FrameCounter.H frame counter based on a frame range */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FrameCounter.H $
// $Id: FrameCounter.H 15330 2012-07-18 17:54:39Z dberg $
//

#ifndef MEDIA_FRAMECOUNTER_H_DEFINED
#define MEDIA_FRAMECOUNTER_H_DEFINED

#include "Media/FrameRange.H"
#include "Media/FrameState.H"
#include "Util/Assert.H"
#include "Util/log.H"

//! Frame counting based on a given FrameRange
class FrameCounter {
public:

  //! Default constructor (first=last=0, delay=0.0)
  inline FrameCounter(const FrameRange& rng, bool w=false) :
    range(rng),
    current(-1),
    nextTime(SimTime::ZERO()),
    startTime(SimTime::ZERO()),
    wrap(w)
  {}

  //! Reset frame and time counters
  void reset(const FrameRange& rng, bool update_start_time, bool w = false)
  {
    range = rng;
    current = range.getFirst()-1;
    nextTime = startTime;
    wrap = w;

    if (update_start_time)
      {
        ASSERT(rng.numDelayTimes() > 0);
        this->setStartTime(rng.getDelayTime(0));
      }
  }

  //! Get the current frame number
  int currentFrame() const
  {
    // if current < first, then it means that we haven't yet started
    // counting, so just return the number of what the first frame
    // will be once we do start
    return current < range.getFirst() ? range.getFirst() : current;
  }

  //! Set the time at which we start incrementing frames
  void setStartTime(const SimTime& t) { startTime = t; nextTime = t; }

  //! Update our frame counter for the given simtime and new-event status.
  /*! If we are in event-triggered mode, then the counter is advanced
      if new_event is true; if we are NOT in event-triggered mode,
      then the counter is advanced if sufficient time has passed since
      our previous frame. */
  FrameState update(const SimTime& stime,
                    const bool new_event = false)
  {
    if (!range.isEventTriggered())
      {
        return this->updateByClock(stime);
      }
    else
      {
        return this->updateByEvent(new_event);
      }
  }

  //! Just increment the frame counter by one.
  FrameState updateNext();

  //! Try to force the counter to the given value
  /*! If the given value is outside of the frame range, return false. */
  bool setCurrent(int n);

  //! Get the time at which the next frame will be displayed
  const SimTime& getNextFrameTime() 
    {
      return nextTime;
    }
    
private:
  // return the proper status for the 'current' frame number, given
  // the previous frame number
  FrameState frameStatus(const int prev) const;

  FrameState updateByClock(const SimTime& stime);

  FrameState updateByEvent(const bool new_event);

  void bumpCurrent();

  FrameRange range;
  int current;        //!< current frame number
  SimTime nextTime;   //!< time at which we'll move to the next frame
  SimTime startTime;  //!< time at which we start counting frames
  bool wrap;          //!< go to the beginning of the frame if the end is reached
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_FRAMECOUNTER_H_DEFINED
