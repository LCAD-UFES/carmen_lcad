/*!@file Util/WorkThreadServer.C Generic low-level worker-thread server class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/WorkThreadServer.C $
// $Id: WorkThreadServer.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef UTIL_WORKTHREADSERVER_C_DEFINED
#define UTIL_WORKTHREADSERVER_C_DEFINED

#include "Util/WorkThreadServer.H"

#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/error_context.h"
#include "rutz/mutex.h"
#include "rutz/sfmt.h"

#include <algorithm> // for stable_sort()
#include <cstdlib>
#include <signal.h>
#include <unistd.h>

struct WorkThreadServer::ThreadData
{
  ThreadData()
    :
    srv(0), thread(), index(0), njobs(0), worktime(), waittime()
  {}

  WorkThreadServer* srv;
  pthread_t thread;
  unsigned int index;
  unsigned int njobs;
  rutz::time worktime;
  rutz::time waittime;
};

struct WorkThreadServer::Checkpoint
{
  Checkpoint(int j, size_t qs, size_t mqs) : jobnum(j), qsize(qs), maxqsize(mqs) {}

  int jobnum;
  size_t qsize;
  size_t maxqsize;
};

struct WorkThreadServer::JobStats
{
  JobStats() : nqueued(0), ndropped(0) {}

  unsigned int nqueued;
  unsigned int ndropped;
};

namespace
{
  struct JobPriorityCompare
  {
    bool operator()(const rutz::shared_ptr<JobServer::Job>& j1,
                    const rutz::shared_ptr<JobServer::Job>& j2)
    {
      return j1->priority() < j2->priority();
    }
  };
}

// ######################################################################
WorkThreadServer::WorkThreadServer(const char* name, unsigned int n, const bool verbose_logs)
  :
  itsName(name),
  itsNumThreads(0),
  itsVerboseLogs(verbose_logs),
  itsThreads(0),
  itsJobsMutex(),
  itsJobs(),
  itsSortJobs(false),
  itsJobsCounter(0u),
  itsMaxQueueSize(itsJobs.max_size()),
  itsDropPolicy(DROP_OLDEST),
  itsNumQueued(0),
  itsNumDropped(0),
  itsFlushBeforeStopping(false),
  itsJobsQuit(false),
  itsMaxObservedQueueSize(0),
  itsLastCheckpoint(-1),
  itsCheckpointPeriod(0),
  itsSleepUsecs(0),
  itsCheckpoints()
{
  // init mutex
  if (0 != pthread_mutex_init(&itsJobsMutex, NULL))
    LFATAL("pthread_mutex_init() failed");

  itsNumRun.atomic_set(0);

  this->start(n);
}

// ######################################################################
WorkThreadServer::~WorkThreadServer()
{
  this->stop();

  if (0 != pthread_mutex_destroy(&itsJobsMutex))
    // not LFATAL(), because we never want to generate an exception in
    // a destructor
    LERROR("pthread_mutex_destroy() failed");
}

// ######################################################################
void WorkThreadServer::start(unsigned int nthreads)
{
  // start threads
  ASSERT(itsThreads == 0);
  itsNumThreads = nthreads;
  itsThreads = new ThreadData[itsNumThreads];
  itsJobsQuit = false;
  for (uint i = 0; i < itsNumThreads; ++i)
    {
      itsThreads[i].srv = this;
      itsThreads[i].index = 0;
      if (0 != pthread_create(&(itsThreads[i].thread), NULL,
                              &WorkThreadServer::c_run,
                              static_cast<void*>(itsThreads+i)))
        LFATAL("pthread_create() failed for thread %u of %u",
               i+1, itsNumThreads);
    }
  itsNumQueued = 0;

  itsCheckpoints.clear();
}

// ######################################################################
void WorkThreadServer::stop()
{
  // do we want to wait for pending jobs to complete before we shut
  // down the worker threads?
  if (itsFlushBeforeStopping)
    this->flushQueue();

  // tell the worker threads to quit when they wake up next
  itsJobsQuit = true;

  // post a bunch of fake jobs to make sure the worker threads wake up
  // and realize that we want to quit
  for (uint i = 0; i < itsNumThreads; ++i)
    itsJobsCounter.post();

  if (itsVerboseLogs) {
    LINFO("%s: %u jobs were queued (%u of those dropped) into %u worker thread(s)",
          itsName.c_str(), itsNumQueued, itsNumDropped, itsNumThreads);

    for (std::map<std::string, JobStats>::iterator
           itr = itsJobStats.begin(), stop = itsJobStats.end(); itr != stop; ++itr)
      LINFO("%s: including %u jobs (%u dropped) of type %s", itsName.c_str(), (*itr).second.nqueued,
            (*itr).second.ndropped, (*itr).first.c_str());

    // dump the contents of our checkpoint cache:
    for (std::list<Checkpoint>::const_iterator
           itr = itsCheckpoints.begin(), stop = itsCheckpoints.end(); itr != stop; ++itr)
      LINFO("%s checkpoint: job #%06d - queue len: %" ZU " (max %" ZU ")",
            itsName.c_str(), (*itr).jobnum, (*itr).qsize, (*itr).maxqsize);
  }

  for (uint i = 0; i < itsNumThreads; ++i) {
      if (0 != pthread_join(itsThreads[i].thread, NULL))
        LERROR("pthread_join() failed for thread %u of %u", i+1, itsNumThreads);
      if (itsVerboseLogs)
        LINFO("%s: thread %u of %u ran %u jobs with %.3fs work time and %.3fs wait time",
              itsName.c_str(), i+1, itsNumThreads, itsThreads[i].njobs, itsThreads[i].worktime.sec(),
              itsThreads[i].waittime.sec());
  }

  delete [] itsThreads;
  itsThreads = 0;
  itsNumThreads = 0;

  if (itsJobs.size() > 0 && itsVerboseLogs)
    LINFO("%s: %" ZU " jobs were abandoned in the job queue", itsName.c_str(), itsJobs.size());

  itsJobs.resize(0);
}

// ######################################################################
void WorkThreadServer::doEnqueueJob(const rutz::shared_ptr<Job>& j)
{
  // the caller must ensure that itsJobsMutex is already locked when
  // this function is called

  itsJobs.push_back(j);

  if (itsSortJobs)
    {
      std::stable_sort(itsJobs.begin(), itsJobs.end(),
                       JobPriorityCompare());
    }

  if (itsJobs.size() <= itsMaxQueueSize)
    {
      itsJobsCounter.post();
    }
  else
    {
      ++itsNumDropped;

      ASSERT(itsJobs.size() > 1); // because itsMaxQueueSize must be >= 1

      switch (itsDropPolicy)
        {
        case DROP_OLDEST:

          // drop the job at the front of the queue
          itsJobStats[itsJobs.front()->jobType()].ndropped++;
          itsJobs.front()->drop();
          itsJobs.pop_front();
          break;

        case DROP_NEWEST:

          // drop the job at the back of the queue (this may or may
          // not be the same job that we have just added, depending on
          // whether or not we have done priority sort)
          itsJobStats[itsJobs.back()->jobType()].ndropped++;
          itsJobs.back()->drop();
          itsJobs.pop_back();
          break;

        case DROP_OLDEST_LOWEST_PRIORITY: // fall-through
        case DROP_NEWEST_LOWEST_PRIORITY:
          {
            // find the lowest priority job in the queue...
            int lowprio = itsJobs.front()->priority();

            typedef std::deque<rutz::shared_ptr<Job> >::iterator iterator;

            iterator todrop = itsJobs.begin();
            iterator itr = todrop; ++itr;
            iterator const stop = itsJobs.end();

            for ( ; itr != stop; ++itr)
              if (itsDropPolicy == DROP_OLDEST_LOWEST_PRIORITY)
                {
                  // use strictly-greater-than here, so that we get
                  // the item with lowest priority that is closest to
                  // the front of the queue (i.e., oldest)
                  if ((*itr)->priority() > lowprio)
                    todrop = itr;
                }
              else // itsDropPolicy == DROP_NEWEST_LOWEST_PRIORITY
                {
                  // use greater-than-or-equal here, so that we get
                  // the item with lowest priority that is closest to
                  // the back of the queue (i.e., newest)
                  if ((*itr)->priority() >= lowprio)
                    todrop = itr;
                }

            // and drop it:
            itsJobStats[(*todrop)->jobType()].ndropped++;
            (*todrop)->drop();
            itsJobs.erase(todrop);
          }
          break;
        }
    }
}

// ######################################################################
void WorkThreadServer::enqueueJob(const rutz::shared_ptr<Job>& j)
{
  if (itsNumThreads == 0 || itsThreads == NULL)
    LFATAL("Can't enqueue jobs into a server with no threads "
           "(jobs would block forever)");

  ASSERT(itsMaxQueueSize >= 1);

  {
    GVX_MUTEX_LOCK(&itsJobsMutex);
    this->doEnqueueJob(j);
  }

  itsJobStats[j->jobType()].nqueued++;

  ++itsNumQueued;
}

// ######################################################################
void WorkThreadServer::enqueueJobs(const rutz::shared_ptr<Job>* jobs,
                                   const size_t njobs)
{
  if (itsNumThreads == 0 || itsThreads == NULL)
    LFATAL("Can't enqueue jobs into a server with no threads "
           "(jobs would block forever)");

  ASSERT(itsMaxQueueSize >= 1);

  {
    GVX_MUTEX_LOCK(&itsJobsMutex);
    for (size_t i = 0; i < njobs; ++i)
      this->doEnqueueJob(jobs[i]);
  }

  for (size_t i = 0; i < njobs; ++i)
    itsJobStats[jobs[i]->jobType()].nqueued++;

  itsNumQueued += njobs;
}

// ######################################################################
unsigned int WorkThreadServer::getParallelismHint()
{
  return itsNumThreads;
}

// ######################################################################
void WorkThreadServer::flushQueue(uint sleepLength, bool verbose)
{
  while (true)
    {
      const size_t sz = this->size();

      if (sz == 0)
        break;

      if(verbose) LINFO("%s: flushing queue - %" ZU " remaining",
            itsName.c_str(), sz);
      usleep(sleepLength);
    }
}

// ######################################################################
size_t WorkThreadServer::size()
{
  size_t ret;
  {
    GVX_MUTEX_LOCK(&itsJobsMutex);
    ret = itsJobs.size();
  }
  return ret;
}

// ######################################################################
void WorkThreadServer::setMaxQueueSize(size_t maxsize)
{
  if (maxsize == 0)
    LFATAL("queue size must be greater than zero");

  GVX_MUTEX_LOCK(&itsJobsMutex);
  itsMaxQueueSize = maxsize;
}

// ######################################################################
void WorkThreadServer::run(ThreadData* dat)
{
  GVX_ERR_CONTEXT(rutz::sfmt("running %s worker thread %u/%u",
                             itsName.c_str(), dat->index + 1, itsNumThreads));

  rutz::time prev = rutz::time::wall_clock_now();

  while (1)
    {
      itsJobsCounter.wait();

      {
        rutz::time t = rutz::time::wall_clock_now();
        dat->waittime += (t - prev);
        prev = t;
      }

      if (itsJobsQuit == true)
        break;

      rutz::shared_ptr<JobServer::Job> job;

      size_t qsize = 0;

      {
        GVX_MUTEX_LOCK(&itsJobsMutex);

        if (itsJobs.size() == 0)
          {
            // If this happens, it probably represents an OS bug
            // (somehow the semaphore counts have gotten corrupted),
            // but we will issue an LERROR() instead of an LFATAL()
            // and try to keep limping along with the goal of not
            // unnecessarily bombing out of a long analysis job.
            LERROR("Oops! Received a semaphore but the job queue is empty!");
            continue;
          }

        ASSERT(itsJobs.size() > 0);

        if (itsJobs.size() > itsMaxObservedQueueSize)
          itsMaxObservedQueueSize = itsJobs.size();

        job = itsJobs.front();
        itsJobs.pop_front();
        qsize = itsJobs.size();

        ++(dat->njobs);
      }

      job->run();

      const int c = itsNumRun.atomic_incr_return();

      // show our queue once in a while:
      if (itsCheckpointPeriod > 0
          && c > itsLastCheckpoint
          && c % itsCheckpointPeriod == 0)
        {
          itsCheckpoints.push_back(Checkpoint(c, qsize, itsMaxObservedQueueSize));
          itsLastCheckpoint = c;
        }

      {
        rutz::time t = rutz::time::wall_clock_now();
        dat->worktime += (t - prev);
        prev = t;
      }

      if (itsSleepUsecs > 0)
        usleep(itsSleepUsecs);
    }
}

// ######################################################################
void* WorkThreadServer::c_run(void* p)
{
  try
    {
      // block all signals in this worker thread; instead of receiving
      // signals here, we rely on the main thread to catch any
      // important signals and then twiddle with the WorkThreadServer
      // object as needed (for example, destroying it to cleanly shut
      // down all worker threads)
      sigset_t ss;
      if (sigfillset(&ss) != 0)
        PLFATAL("sigfillset() failed");

      if (pthread_sigmask(SIG_SETMASK, &ss, 0) != 0)
        PLFATAL("pthread_sigmask() failed");

      ThreadData* const dat = static_cast<ThreadData*>(p);

      WorkThreadServer* const srv = dat->srv;

      srv->run(dat);
    }
  catch (...)
    {
      REPORT_CURRENT_EXCEPTION;
      abort();
    }

  return NULL;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_WORKTHREADSERVER_C_DEFINED
