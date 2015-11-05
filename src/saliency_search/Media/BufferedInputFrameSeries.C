/*!@file Media/BufferedInputFrameSeries.C Buffered input with frames loaded in a worker thread */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/BufferedInputFrameSeries.C $
// $Id: BufferedInputFrameSeries.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef MEDIA_BUFFEREDINPUTFRAMESERIES_C_DEFINED
#define MEDIA_BUFFEREDINPUTFRAMESERIES_C_DEFINED

#include "Media/BufferedInputFrameSeries.H"

#include "Util/log.H"
#include <unistd.h>

struct BufferedInputFrameSeries::Checkpoint
{
  Checkpoint(int f, int q, int mq) : framenum(f), qsize(q), minqsize(mq) {}

  int framenum;
  int qsize;
  int minqsize;
};

// ######################################################################
BufferedInputFrameSeries::
BufferedInputFrameSeries(OptionManager& mgr, const size_t qsize,
                         const bool forcergb)
  :
  ModelComponent(mgr, "Input Buffer", "BufferedInputFrameSeries"),
  itsSrc(new InputFrameSeries(mgr)),
  itsFrameSpec(),
  itsQ(qsize),
  itsInputDone(false),
  itsStop(false),
  itsNumFilled(),
  itsMinNumFilled(int(qsize)),
  itsFrameNum(0),
  itsCheckpoints(),
  itsForceRGB(forcergb)
{
  if (0 != pthread_mutex_init(&itsQmut, NULL))
    LFATAL("pthread_mutex_init() failed");

  this->addSubComponent(itsSrc);
}

// ######################################################################
GenericFrameSpec BufferedInputFrameSeries::peekFrameSpec() const
{
  if (!this->started())
    LFATAL("must be start()-ed before peekFrameSpec()");

  return itsFrameSpec;
}

// ######################################################################
GenericFrame BufferedInputFrameSeries::get(bool* did_underflow)
{
  GenericFrame result;

  const bool pop_ok = itsQ.pop_front(result);

  if (pop_ok)
    {
      const int n = itsNumFilled.atomic_decr_return();
      if (n < itsMinNumFilled)
        itsMinNumFilled = n;
    }

  if (did_underflow != 0)
    // we have premature underflow if the pop fails (!pop_ok) before
    // our underlying input source is finished (!itsInputDone):
    *did_underflow = (!pop_ok && !itsInputDone);

  if (++itsFrameNum % 100 == 0)
    itsCheckpoints.push_back
      (Checkpoint(itsFrameNum, itsNumFilled.atomic_get(),
                  itsMinNumFilled));

  return result;
}

// ######################################################################
void BufferedInputFrameSeries::start2()
{
  itsFrameSpec = itsSrc->peekFrameSpec();

  if (0 != pthread_create(&itsFillThread, NULL, &c_fill,
                          static_cast<void*>(this)))
    LFATAL("pthread_create() failed");

  itsMinNumFilled = int(itsQ.size());
  itsFrameNum = 0;
  itsCheckpoints.clear();
}

// ######################################################################
void BufferedInputFrameSeries::stop1()
{
  itsStop = true;

  if (0 != pthread_join(itsFillThread, NULL))
    LERROR("pthread_join() failed");

  for (std::list<Checkpoint>::const_iterator
         itr = itsCheckpoints.begin(), stop = itsCheckpoints.end();
       itr != stop; ++itr)
    {
      LINFO("checkpoint frame %06d - queue fill : %d/%d (min %d)",
            (*itr).framenum, (*itr).qsize, int(itsQ.size()),
            (*itr).minqsize);
    }
}

// ######################################################################
void* BufferedInputFrameSeries::c_fill(void* p)
{
  try
    {
      BufferedInputFrameSeries* const b =
        static_cast<BufferedInputFrameSeries*>(p);

      while (true)
        {
          if (b->itsStop)
            break;

          // get the next frame:
          const FrameState is = b->itsSrc->updateNext();
          if (is == FRAME_COMPLETE)
            {
              b->itsInputDone = true;
              return NULL;
            }

          GenericFrame f = b->itsSrc->readFrame();
          if (!f.initialized())
            {
              b->itsInputDone = true;
              return NULL;
            }

          // do we want to force the frame to have a native type of RGB?
          if (b->itsForceRGB)
            f = GenericFrame(f.asRgb());

          // now try to push it onto the queue (and just keep
          // re-trying if the push fails due to the queue being full):
          while (true)
            {
              if (b->itsStop)
                break;

              if (b->itsQ.push_back(f) == true)
                {
                  b->itsNumFilled.atomic_incr();
                  break;
                }

              usleep(20000);
            }
        }
    }
  catch (...)
    {
      REPORT_CURRENT_EXCEPTION;
      exit(1);
    }

  return NULL;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_BUFFEREDINPUTFRAMESERIES_C_DEFINED
