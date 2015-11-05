/*!@file Util/JobWithSemaphore.H Convenience subclass of JobServer::Job that includes a semaphore to communicate job completion */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/JobWithSemaphore.H $
// $Id: JobWithSemaphore.H 13653 2010-07-10 01:22:08Z lior $
//

#ifndef UTIL_JOBWITHSEMAPHORE_H_DEFINED
#define UTIL_JOBWITHSEMAPHORE_H_DEFINED

#include "Util/JobServer.H"
#include "Util/Semaphore.H"

/// Convenience subclass of JobServer::Job that includes a semaphore to communicate job completion
class JobWithSemaphore : public JobServer::Job
{
public:
  JobWithSemaphore();

  virtual ~JobWithSemaphore();

  /// Job clients call this to test if the job has finished running
  bool isJobDone();

  /// Job clients call this to block until their job has finished running
  void wait();

  /// Determine if job client is finished
  bool isFinished();

protected:
  /// Job subclass implementors call this at the end of their run() function
  void markFinished();

private:
  JobWithSemaphore(const JobWithSemaphore&); // not implemented
  JobWithSemaphore& operator=(const JobWithSemaphore&); // not implemented
  Semaphore finishedFlag;
  bool jobDone;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_JOBWITHSEMAPHORE_H_DEFINED
