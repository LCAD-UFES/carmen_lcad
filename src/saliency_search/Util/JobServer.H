/*!@file Util/JobServer.H Generic interface for running computation jobs */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/JobServer.H $
// $Id: JobServer.H 8384 2007-05-13 16:16:54Z rjpeters $
//

#ifndef UTIL_JOBSERVER_H_DEFINED
#define UTIL_JOBSERVER_H_DEFINED

#include "rutz/shared_ptr.h"

#include <cstddef> // for size_t

/// Abstract interface for running generic computation jobs
class JobServer
{
public:

  /// Base class for jobs that can be submitted to a JobServer
  class Job
  {
  public:
    Job();

    virtual ~Job();

    virtual void run() = 0;

    virtual const char* jobType() const = 0;

    /// Default implementation returns 0 (lower value = higher priority)
    virtual int priority() const;

    /// Check if the job was dropped by the job server
    bool wasDropped() const { return itsDropped; }

    /// Mark the job as dropped (should be called by the job server as needed)
    void drop() { itsDropped = true; }

  private:
    bool itsDropped;
  };

  /// Virtual destructor for safe inheritance
  virtual ~JobServer();

  /// Submit a job to be run
  /** Depending on the JobServer subclass, the job may be run
      immediately or may be run at a later time, in a different
      thread, on a different machine, etc. */
  virtual void enqueueJob(const rutz::shared_ptr<Job>& job) = 0;

  /// Submit multiple jobs in a batch
  /** Depending on the JobServer subclass, this may be more efficient
      than a series of individual enqueueJob() calls (e.g., the
      subclass might be able to lock/unlock a mutex just once rather
      than once per submitted job).

      The default implementation just repeatedly calls enqueueJob().
  */
  virtual void enqueueJobs(const rutz::shared_ptr<Job>* jobs,
                           const size_t njobs);

  /// Return a hint as to how many jobs could naturally be run in parallel by this JobServer
  /** Clients might be able to use this hint to decide how to best
      decompose their overall algorithm into individual job
      requeusts. */
  virtual unsigned int getParallelismHint() = 0;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_JOBSERVER_H_DEFINED
