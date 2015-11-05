/*!@file Transport/BobDeinterlacer.C Deinterlace frames with the "bob" method */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/BobDeinterlacer.C $
// $Id: BobDeinterlacer.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef TRANSPORT_BOBDEINTERLACER_C_DEFINED
#define TRANSPORT_BOBDEINTERLACER_C_DEFINED

#include "Transport/BobDeinterlacer.H"

#include "Raster/GenericFrame.H"
#include "Util/SimTime.H"
#include <unistd.h>

// ######################################################################
BobDeinterlacer::BobDeinterlacer(OptionManager& mgr)
  :
  Deinterlacer(mgr),
  itsFieldTime(SimTime::ZERO()),
  itsFrameNumber(-1),
  itsCurFullFrame(),
  itsLastFrameTime(),
  itsLastBottomTime(),
  itsUsleepAdjustment(0.0),
  itsDeinterlacing(false),
  itsInBottomField(false)
{}

// ######################################################################
BobDeinterlacer::~BobDeinterlacer()
{}

// ######################################################################
void BobDeinterlacer::setListener(rutz::shared_ptr<FrameListener> listener)
{
  getDelegate().setListener(listener);
}

// ######################################################################
GenericFrameSpec BobDeinterlacer::peekFrameSpec()
{
  GenericFrameSpec result = getDelegate().peekFrameSpec();

  switch (result.nativeType)
    {
    case GenericFrame::NONE:
      ASSERT(0);
      break;

    case GenericFrame::RGB_U8:
    case GenericFrame::RGBD:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_RGB24;
      result.videoByteSwap = false;
      break;

    case GenericFrame::RGB_F32:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_RGB24;
      result.videoByteSwap = false;
      break;

    case GenericFrame::GRAY_U8:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_GREY;
      result.videoByteSwap = false;
      break;

    case GenericFrame::GRAY_F32:
      result.nativeType = GenericFrame::VIDEO;
      result.videoFormat = VIDFMT_GREY;
      result.videoByteSwap = false;
      break;

    case GenericFrame::RGB_U16:
      break;

    case GenericFrame::GRAY_U16:
      break;

    case GenericFrame::VIDEO:
      // no changes
      break;
    }

  return result;
}

// ######################################################################
SimTime BobDeinterlacer::getNaturalFrameTime() const
{
  return itsFieldTime;
}

// ######################################################################
bool BobDeinterlacer::setFrameNumber(int n)
{
  ASSERT(n >= 0);

  bool ret = true;

  if (itsFrameNumber < 0
      || itsFrameNumber/2 != n/2)
    ret = getDelegate().setFrameNumber(n/2);

  itsFrameNumber = n;

  return ret;
}

// ######################################################################
void BobDeinterlacer::startStream()
{
  getDelegate().startStream();

  itsInBottomField = false;
}

// ######################################################################
GenericFrame BobDeinterlacer::readFrame()
{
  if (!itsDeinterlacing)
    // we're not deinterlacing, just just return the raw buffer:
    return getDelegate().readFrame();

  // else: OK, we're deinterlacing

  if (itsInBottomField)
    {
      VideoFrame bottomfield = itsCurFullFrame.makeBobDeinterlaced(1);

      // if we have a non-zero half-field time, then let's try a
      // combination of usleep() and a busy loop to get to that time
      // exactly:
      if (itsFieldTime > SimTime::ZERO())
        {
          const double elapsed =
            (rutz::time::wall_clock_now() - itsLastFrameTime).usec();

          if (elapsed < 0)
            LFATAL("Hey, time is moving backwards!");

          const double fieldusec = itsFieldTime.usecs();

          const int sleepusec =
            int(fieldusec - elapsed - itsUsleepAdjustment - 1000);

          // we do a usleep() for most of the delay, but then finish
          // the delay with a hard busy loop to get more precise
          // timing

          // not sure why we aren't getting better timing from
          // usleep()

          if (sleepusec > 0)
            usleep(sleepusec);

          while ((rutz::time::wall_clock_now() - itsLastFrameTime).usec()
                 < fieldusec)
            {
              // hard busy loop
            }
        }

      // switch to the other field for next time:
      itsInBottomField = false;

      itsLastBottomTime = rutz::time::wall_clock_now();

      const double half_delay =
        (itsLastBottomTime - itsLastFrameTime).usec();

      itsUsleepAdjustment += 0.5 * (half_delay - itsFieldTime.usecs());

      // and return the deinterlaced bottom field:
      return GenericFrame(bottomfield);
    }

  // else, we're waiting for a top field, so we need to grab a new
  // frame:

  itsCurFullFrame = getDelegate().readFrame().asVideo();

  if (!itsCurFullFrame.initialized())
    // ok, we got an empty frame from our delegate (probably means the
    // stream is exhausted), so just return an empty frame
    return GenericFrame();

  VideoFrame topfield = itsCurFullFrame.makeBobDeinterlaced(0);

  itsLastFrameTime = rutz::time::wall_clock_now();

  // switch to the other field for next time:
  itsInBottomField = true;

  // and return the deinterlaced top field:
  return GenericFrame(topfield);
}

// ######################################################################
void BobDeinterlacer::start2()
{
  Deinterlacer::start2();

  itsCurFullFrame = VideoFrame();

  // is this a deinterlaceable format?
  VideoFormat vidfmt = getDelegate().peekFrameSpec().videoFormat;

  itsDeinterlacing = isSimplePackedMode(vidfmt) || vidfmt == VIDFMT_YUV420P;

  if (!itsDeinterlacing)
    LFATAL("bob-deinterlacing is not supported for video mode %s "
           "(bob-deinterlacing requires a simple packed pixel format)",
           convertToString(vidfmt).c_str());

  itsInBottomField = false;

  const SimTime frametime = getDelegate().getNaturalFrameTime();
  itsFieldTime = frametime * 0.5;

  // NTSC: ((1.0/29.97Hz)/2.0) * (1000000.0 usec/sec) = 16683.35usec
  LINFO("using a half-field time of %.2fus [%.2fHz] "
        "(full frame time is %.2fus [%.2fHz])",
        itsFieldTime.usecs(), itsFieldTime.hertz(),
        frametime.usecs(), frametime.hertz());
}

// ######################################################################
void BobDeinterlacer::stop2()
{
  itsCurFullFrame = VideoFrame();
  itsDeinterlacing = false;
  itsInBottomField = false;

  Deinterlacer::stop2();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_BOBDEINTERLACER_C_DEFINED
