/*!@file Util/WorkThreadServer.H Generic low-level worker-thread server class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/WorkThreadServer.H $
// $Id: WorkThreadServer.H 14387 2011-01-12 17:15:26Z lior $
//

#ifndef UTIL_WORKTHREADSERVER_H_DEFINED
#define UTIL_WORKTHREADSERVER_H_DEFINED

#include "Util/JobServer.H"
#include "Util/Semaphore.H"
#include "rutz/shared_ptr.h"
#include "rutz/atomic.h"
#include "rutz/time.h"

#include <deque>
#include <list>
#include <map>
#include <pthread.h>
#include <string>

/// Generic low-level thread-server class
/** To use this class, derive a job class from JobServer::Job
    whose run() function does the work that you want done. Then
    instantiate a WorkThreadServer object, start() it with your
    desired number of work threads, and start calling enqueueJob()
    with your Job objects. See DiskDataStream for an
    example of usage. */
class WorkThreadServer : public JobServer
{
public:

  /// Construct with a descriptive nameand a number of worker threads
  WorkThreadServer(const char* name, unsigned int n, const bool verbose_logs = true);

  /// Virtual destructor for safe inheritance
  virtual ~WorkThreadServer();

private:
  /// Start \a n worker threads ready to receive Job requests.
  void start(unsigned int n);

  /// Stop all worker threads (and wait for them to actually stop).
  void stop();

public:
  /// Put a Job in the queue.
  /** Worker threads process Job requests in the order they are received. */
  virtual void enqueueJob(const rutz::shared_ptr<Job>& job);

  /// Submit multiple jobs in a batch
  /** This is more efficient than multiple individual enqueueJob()
      calls, since we can lock/unlock the queue mutex just once for
      the entire batch. */
  virtual void enqueueJobs(const rutz::shared_ptr<Job>* jobs,
                           const size_t njobs);

  /// Returns the number of worker threads
  virtual unsigned int getParallelismHint();

  /// Wait now for all jobs in the queue to be completed
  void flushQueue(uint sleepLength = 250000, bool verbose = true);

  /// Get the number of jobs currently waiting in the queue.
  size_t size();

  //! Get the number of jobs ran
  int getNumRun() { return itsNumRun.atomic_get(); }

  /// How often (in number of jobs) should a checkpoint log entry be written
  /** If <= 0, then never print checkpoint entries. */
  void setCheckpointPeriod(int p) { itsCheckpointPeriod = p; }

  /// Number of microseconds to usleep() in between running each job
  /** If <= 0, then don't usleep() at all. */
  void setSleepUsecs(unsigned int us) { itsSleepUsecs = us; }

  /// Change the maximum size for the internal queue
  /** If an enqueueJob() or enqueueJobs() call would otherwise cause
      the maximum queue size to be exceeded, then jobs will be dropped
      from the queue in accordance with the current drop policy. */
  void setMaxQueueSize(size_t maxsize);

  /// Whether to wait for the job queue to become empty before stopping the worker threads
  /** If false (the default), then any jobs left in the queue will be
      abandoned when stop() or the destructor is called. */
  void setFlushBeforeStopping(bool v) { itsFlushBeforeStopping = v; }

  /// Whether to sort jobs by priority
  void setSortJobs(bool v) { itsSortJobs = v; }

  enum DropPolicy
    {
      DROP_OLDEST,  ///< drop oldest job (OR, if we are sorting, that means the highest priority job)
      DROP_NEWEST,  ///< drop newest job (OR, if we are sorting, that means the lowest priority job)
      DROP_OLDEST_LOWEST_PRIORITY, ///< drop oldest+lowest priority job regardless of whether we are sorting by priority
      DROP_NEWEST_LOWEST_PRIORITY  ///< drop newest+lowest priority job regardless of whether we are sorting by priority
    };

  /// Specify the policy to be used to drop jobs to avoid exceeding the maximum queue size
  void setDropPolicy(DropPolicy p) { itsDropPolicy = p; }

private:
  struct ThreadData;
  struct Checkpoint;
  struct JobStats;

  // For internal use only; this must be called only when itsJobsMutex
  // is already locked
  void doEnqueueJob(const rutz::shared_ptr<Job>& job);

  static void* c_run(void* p);

  void run(ThreadData* dat);

  std::string                   itsName;

  unsigned int                  itsNumThreads;
  bool                          itsVerboseLogs;
  ThreadData*                   itsThreads;

  pthread_mutex_t               itsJobsMutex;
  std::deque<rutz::shared_ptr<Job> >   itsJobs;
  bool                          itsSortJobs;
  Semaphore                     itsJobsCounter;
  size_t                        itsMaxQueueSize;
  DropPolicy                    itsDropPolicy;
  unsigned int                  itsNumQueued;
  unsigned int                  itsNumDropped;
  bool                          itsFlushBeforeStopping;
  bool                          itsJobsQuit;
  rutz::atomic_int_t            itsNumRun;
  size_t                        itsMaxObservedQueueSize;
  int                           itsLastCheckpoint;
  int                           itsCheckpointPeriod;
  unsigned int                  itsSleepUsecs;
  std::list<Checkpoint>         itsCheckpoints;
  std::map<std::string, JobStats> itsJobStats;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_WORKTHREADSERVER_H_DEFINED
