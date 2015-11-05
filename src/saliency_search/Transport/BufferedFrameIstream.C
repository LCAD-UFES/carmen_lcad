/*!@file Transport/BufferedFrameIstream.C Frame-buffering layer on top of other FrameIstream objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/BufferedFrameIstream.C $
// $Id: BufferedFrameIstream.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef TRANSPORT_BUFFEREDFRAMEISTREAM_C_DEFINED
#define TRANSPORT_BUFFEREDFRAMEISTREAM_C_DEFINED

#include "Transport/BufferedFrameIstream.H"

#include "Component/ModelOptionDef.H"
#include "Transport/FrameIstreamFactory.H"
#include "Transport/TransportOpts.H"
#include "Util/SimTime.H"
#include "Util/log.H"
#include "rutz/time.h"
#include <unistd.h>

// Used by: BufferedFrameIstream
static const ModelOptionDef OPT_InputBufferSize =
  { MODOPT_ARG(size_t), "InputBufferSize", &MOC_INPUT, OPTEXP_CORE,
    "Number of frames to keep in input buffer when using --in=buf",
    "input-buffer-size", '\0', "<int>", "32" };

// Used by: BufferedFrameIstream
static const ModelOptionDef OPT_UnderflowStrategy =
  { MODOPT_ARG(ErrorStrategy), "UnderflowStrategy", &MOC_INPUT, OPTEXP_CORE,
    "What to do if the input buffer is empty when trying to read "
    "a new frame when using --in=buf; ABORT=make it a fatal error, "
    "RETRY=keep polling until the buffer becomes non-empty, "
    "IGNORE=ignore the error and return an empty frame as if "
    "end-of-stream had been reached",
    "underflow-strategy", '\0', "<ABORT|RETRY|IGNORE>", "IGNORE" };

struct BufferedFrameIstream::Checkpoint
{
  Checkpoint(int f, int q, int mq) : framenum(f), qsize(q), minqsize(mq) {}

  int framenum;
  int qsize;
  int minqsize;
};

// ######################################################################
BufferedFrameIstream::BufferedFrameIstream(OptionManager& mgr)
  :
  FrameIstream(mgr, "Buffered Input", "BufferedFrameIstream"),
  itsBufSize(&OPT_InputBufferSize, this),
  itsUnderflowStrategy(&OPT_UnderflowStrategy, this),
  itsSrc(),
  itsQ(0),
  itsInputDone(false),
  itsStop(false),
  itsNumFilled(),
  itsMinNumFilled(0),
  itsInputFrameNum(-1),
  itsOutputFrameNum(0),
  itsFrameSpec(),
  itsFrameSpecValid(false),
  itsCheckpoints()
{
  if (0 != pthread_mutex_init(&itsQmut, NULL))
    LFATAL("pthread_mutex_init() failed");
}

// ######################################################################
BufferedFrameIstream::~BufferedFrameIstream()
{
  delete itsQ;
}

// ######################################################################
void BufferedFrameIstream::setConfigInfo(const std::string& cfg)
{
  if (itsSrc.is_valid())
    this->removeSubComponent(*itsSrc);
  itsSrc = makeFrameIstream(cfg);
  ASSERT(itsSrc.is_valid());
  this->addSubComponent(itsSrc);
}

// ######################################################################
bool BufferedFrameIstream::setFrameNumber(int n)
{
  if (!itsSrc.is_valid())
    LFATAL("must have a valid input source before setFrameNumber()");

  itsOutputFrameNum = n;

  return true;
}

// ######################################################################
GenericFrameSpec BufferedFrameIstream::peekFrameSpec()
{
  if (!itsFrameSpecValid)
    {
      if (!itsSrc.is_valid())
        LFATAL("must have a valid input source before peekFrameSpec()");

      if (itsInputFrameNum < 0)
        {
          itsInputFrameNum = itsOutputFrameNum;
          itsSrc->setFrameNumber(itsInputFrameNum);
        }

      itsFrameSpec = itsSrc->peekFrameSpec();
      itsFrameSpecValid = true;
    }

  return itsFrameSpec;
}

// ######################################################################
SimTime BufferedFrameIstream::getNaturalFrameTime() const
{
  if (!this->started())
    LFATAL("must be start()-ed before getNaturalFrameTime()");

  ASSERT(itsSrc.is_valid());

  return itsSrc->getNaturalFrameTime();
}

// ######################################################################
void BufferedFrameIstream::startStream()
{
  if (!this->started())
    LFATAL("must be start()-ed before startStream()");

  ASSERT(itsQ != 0);

  // wait for the input queue to get filled
  rutz::time t = rutz::time::wall_clock_now();
  while (itsNumFilled.atomic_get() < int(itsQ->size()) && !itsInputDone)
    {
      rutz::time t2 = rutz::time::wall_clock_now();
      if ((t2 - t).sec() >= 1.0)
        {
          LINFO("waiting for input queue to be filled (%d/%d)",
                itsNumFilled.atomic_get(), int(itsQ->size()));
          t = t2;
        }
      usleep(20000);
    }

  LINFO("%d/%d entries filled in input queue%s",
        itsNumFilled.atomic_get(), int(itsQ->size()),
        itsInputDone ? " [input read to completion]" : "");
}

// ######################################################################
GenericFrame BufferedFrameIstream::readFrame()
{
  if (!this->started())
    LFATAL("must be start()-ed before readFrame()");

  ASSERT(itsQ != 0);

  GenericFrame result;

  while (true)
    {
      const bool pop_ok = itsQ->pop_front(result);

      if (pop_ok)
        {
          const int n = itsNumFilled.atomic_decr_return();
          if (n < itsMinNumFilled)
            itsMinNumFilled = n;
        }

      // we have premature underflow if the pop fails (!pop_ok) before
      // our underlying input source is finished (!itsInputDone):
      const bool did_underflow = (!pop_ok && !itsInputDone);

      if (!did_underflow)
        break;
      else if (itsUnderflowStrategy.getVal() == ERR_ABORT)
        {
          LFATAL("input underflow");
        }
      else if (itsUnderflowStrategy.getVal() == ERR_RETRY)
        {
          LDEBUG("input underflow");
          usleep(20000);
          continue;
        }
      else if (itsUnderflowStrategy.getVal() == ERR_IGNORE)
        {
          // report the underflow error but ignore it and break out
          // the loop, letting the returned image be empty
          LINFO("input underflow");
          break;
        }
    }

  if (itsOutputFrameNum % 100 == 0)
    itsCheckpoints.push_back
      (Checkpoint(itsOutputFrameNum, itsNumFilled.atomic_get(),
                  itsMinNumFilled));

  return result;
}

// ######################################################################
void BufferedFrameIstream::start2()
{
  if (this->started())
    LFATAL("must be stop()-ed before start()");

  if (!itsSrc.is_valid())
    LFATAL("no underlying input source given");

  if (itsBufSize.getVal() <= 0)
    LFATAL("--%s must be greater than 0, but got --%s=%" ZU ,
           itsBufSize.getOptionDef()->longoptname,
           itsBufSize.getOptionDef()->longoptname,
           itsBufSize.getVal());

  itsInputDone = false;
  itsStop = false;
  itsMinNumFilled = int(itsBufSize.getVal());
  itsCheckpoints.clear();

  // call peekFrameSpec() at least once to ensure that itsFrameSpec
  // gets cached, so that we don't need to access
  // itsSrc->peekFrameSpec() after we start the worker thread (because
  // then we would have possibly reentrant calls into itsSrc, which
  // has no guarantee of being thread-safe)
  (void) this->peekFrameSpec();

  ASSERT(itsQ == 0);

  itsQ = new rutz::circular_queue<GenericFrame>(itsBufSize.getVal());

  if (0 != pthread_create(&itsFillThread, NULL, &c_fill,
                          static_cast<void*>(this)))
    LFATAL("pthread_create() failed");

}

// ######################################################################
void BufferedFrameIstream::stop1()
{
  if (!this->started())
    LFATAL("must be start()-ed before stop()");

  ASSERT(itsQ != 0);

  itsStop = true;

  if (0 != pthread_join(itsFillThread, NULL))
    LERROR("pthread_join() failed");

  for (std::list<Checkpoint>::const_iterator
         itr = itsCheckpoints.begin(), stop = itsCheckpoints.end();
       itr != stop; ++itr)
    {
      LINFO("checkpoint frame %06d - queue fill : %d/%d (min %d)",
            (*itr).framenum, (*itr).qsize, int(itsQ->size()),
            (*itr).minqsize);
    }

  delete itsQ;
  itsQ = 0;

  itsFrameSpecValid = false;
}

// ######################################################################
void* BufferedFrameIstream::c_fill(void* p)
{
  try
    {
      BufferedFrameIstream* const b =
        static_cast<BufferedFrameIstream*>(p);

      // convert nub::soft_ref to nub::ref to make sure that we have a
      // valid object:
      nub::ref<FrameIstream> src = b->itsSrc;

      src->startStream();

      while (true)
        {
          if (b->itsStop)
            break;

          // get the next frame:

          if (!src->setFrameNumber(b->itsInputFrameNum++))
            {
              b->itsInputDone = true;
              return NULL;
            }

          GenericFrame f = src->readFrame();
          if (!f.initialized())
            {
              b->itsInputDone = true;
              return NULL;
            }

          // now try to push it onto the queue (and just keep
          // re-trying if the push fails due to the queue being full):

          while (true)
            {
              if (b->itsStop)
                break;

              ASSERT(b->itsQ != 0);

              if (b->itsQ->push_back(f) == true)
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

#endif // TRANSPORT_BUFFEREDFRAMEISTREAM_C_DEFINED
