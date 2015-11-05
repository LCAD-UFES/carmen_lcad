/*!@file Media/FrameCounter.C frame counter based on a frame range */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FrameCounter.C $
// $Id: FrameCounter.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef MEDIA_FRAMECOUNTER_C_DEFINED
#define MEDIA_FRAMECOUNTER_C_DEFINED

#include "Media/FrameCounter.H"

// ######################################################################
FrameState FrameCounter::updateNext()
{
  const int prev = this->current;

  this->bumpCurrent();

  return this->frameStatus(prev);
}

// ######################################################################
FrameState FrameCounter::frameStatus(const int prev) const
{
  if      (current == prev)            return FRAME_SAME;
  else if (current == range.getLast()) return FRAME_FINAL;
  else if (current >  range.getLast()) return FRAME_COMPLETE;
  else                                 return FRAME_NEXT;
}

// ######################################################################
FrameState FrameCounter::updateByClock(const SimTime& stime)
{
  const int prev = this->current;

  // check whether we need to move on to the next frame:
  if (stime >= this->nextTime)
    {
      this->bumpCurrent();

      // compute the time after which we'll move to the next frame:
      const size_t num_delays = range.numDelayTimes();
      LDEBUG("num_delays: %" ZU , num_delays);
      if (num_delays == 1)
        {
          this->nextTime =
            this->startTime
            + ((this->current + 1 - range.getFirst())
               * range.getDelayTime(0));
        }
      else if (num_delays > 1)
        {
          // when we come to the last frame in a variable frame-rate
          // series (which has been specified from a .fl file; see
          // FrameRange::fromString() for details), we have no way of
          // giving a proper "next frame time", so instead we just use
          // SimTime::MAX()

          if (size_t(this->current) < range.numDelayTimes())
            this->nextTime = range.getDelayTime(this->current);
          else
            this->nextTime = SimTime::MAX();
        }
      else
        {
          LFATAL("FrameRange contains no delay times");
        }
      LDEBUG("simtime: %.2fms, frame %d will be uploaded at: %.2fms",
             stime.msecs(), this->current + 1, this->nextTime.msecs());
    }

  return this->frameStatus(prev);
}

// ######################################################################
FrameState FrameCounter::updateByEvent(const bool new_event)
{
  const int prev = this->current;

  if (new_event)
    this->bumpCurrent();

  return this->frameStatus(prev);
}

// ######################################################################
void FrameCounter::bumpCurrent()
{
  ASSERT(range.getStep() > 0);

  if (this->current < range.getFirst())
    // special case: in our starting condition before anything has
    // happened, we set this->current to range.first-1; so in that
    // case we don't want to bump up by range.step, but rather just
    // set this->current to range.first
    this->current = range.getFirst();
  else
    this->current += range.getStep();

  if (wrap && this->current > range.getLast())
    this->current = range.getFirst();
}

// ######################################################################
bool FrameCounter::setCurrent(int n)
{
  if(n < range.getFirst() || n > range.getLast())
    return false;

  this->current = n;
  return true;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_FRAMECOUNTER_C_DEFINED
