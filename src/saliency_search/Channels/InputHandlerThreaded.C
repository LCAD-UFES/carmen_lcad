/*!@file Channels/InputHandlerThreaded.C InputHandler subclass than can be plugged into SingleChannel to get multi-threaded computations */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/InputHandlerThreaded.C $
// $Id: InputHandlerThreaded.C 8590 2007-07-18 22:18:37Z rjpeters $
//

#ifndef CHANNELS_INPUTHANDLERTHREADED_C_DEFINED
#define CHANNELS_INPUTHANDLERTHREADED_C_DEFINED

#include "Channels/InputHandlerThreaded.H"

#include "Channels/SingleChannel.H"
#include "Util/JobServer.H"
#include "Util/JobWithSemaphore.H"
#include "Util/MainJobServer.H"

#include <unistd.h>

/// Represents a pending SingleChannel input job
struct InputHandlerThreaded::Job : public JobWithSemaphore
{
  Job(SingleChannel& chan_,
      const Image<float>& bwimg_,
      const SimTime& t_,
      const Image<byte>& clipMask_,
      const rutz::shared_ptr<PyramidCache<float> >& cache_);

  virtual ~Job();

  virtual void run();

  virtual const char* jobType() const;

  SingleChannel* const chan;
  Image<float> const bwimg;
  SimTime const t;
  Image<byte> const clipMask;
  rutz::shared_ptr<PyramidCache<float> > const cache;

private:
  Job(const Job&); // not implemented
  Job& operator=(const Job&); // not implemented
};

// ######################################################################
InputHandlerThreaded::Job::Job(SingleChannel& chan_,
                               const Image<float>& bwimg_,
                               const SimTime& t_,
                               const Image<byte>& clipMask_,
                               const rutz::shared_ptr<PyramidCache<float> >& cache_)
  :
  chan(&chan_), bwimg(bwimg_), t(t_), clipMask(clipMask_), cache(cache_)
{}

// ######################################################################
InputHandlerThreaded::Job::~Job()
{}

// ######################################################################
void InputHandlerThreaded::Job::run()
{
  // now do the job
  this->chan->setClipPyramid(this->clipMask);
  this->chan->storePyramid(this->chan->computePyramid(this->bwimg, this->cache),
                           this->t);
  // force the output to be computed and cached
  this->chan->storeOutputCache(this->chan->combineSubMaps());
  this->markFinished();
}

// ######################################################################
const char* InputHandlerThreaded::Job::jobType() const
{
  return "SingleChannelInputJob";
}

// ######################################################################
InputHandlerThreaded::
InputHandlerThreaded()
  :
  itsJob()
{}

// ######################################################################
InputHandlerThreaded::~InputHandlerThreaded()
{}

// ######################################################################
void InputHandlerThreaded::handleInput(SingleChannel& chan,
                                       const Image<float>& bwimg,
                                       const SimTime& t,
                                       const Image<byte>& clipMask,
                                       const rutz::shared_ptr<PyramidCache<float> >& cache)
{
  itsJob = rutz::make_shared(new Job(chan, bwimg, t, clipMask, cache));
  ASSERT(itsJob.is_valid());
  getMainJobServer().enqueueJob(itsJob);
}

// ######################################################################
void InputHandlerThreaded::waitForOutput(SingleChannel& chan)
{
  if (!itsJob.is_valid())
    return;

  LDEBUG("waiting for thread job to complete for channel %s",
         chan.descriptiveName().c_str());

  itsJob->wait();

  itsJob.reset(NULL);
}

// ######################################################################
rutz::shared_ptr<InputHandler> InputHandlerThreaded::makeClone() const
{
  return rutz::make_shared(new InputHandlerThreaded);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INPUTHANDLERTHREADED_C_DEFINED
