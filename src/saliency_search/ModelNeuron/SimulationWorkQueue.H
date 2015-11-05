/*!@file ModelNeuron/SimulationWorkQueue.H */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: David J Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SimulationWorkQueue.H $

#ifndef MODELNEURON_SIMULATIONWORKQUEUE_H_DEFINED
#define MODELNEURON_SIMULATIONWORKQUEUE_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <future> 
#include <vector> 

namespace nsu
{
  // ######################################################################
  //! A 'work queue' for multithreading simulation elements
  // ######################################################################  
  class SimulationWorkQueue
  {
    public:
      //! constructor
      SimulationWorkQueue();

      //! constructor
      SimulationWorkQueue(const unsigned int numthreads);

      //! destructor
      ~SimulationWorkQueue();

      //! add a job to the queue, start the jobs with go.
      void add(std::function<void()>&& job);

      //! start processing any jobs where add was used and wait
      void go();

      //!push a job onto the queue and run instantly
      std::future<void> push(std::function<void()>&& job);

      //!push jobs onto the queue and wait
      void push(std::vector<std::function<void()> >&& job);
      
      //!wait for jobs to finish and resize the queue
      void resize(const unsigned int numthreads);
      
      //! get the number of threads
      unsigned int numThreads() const;

    private:
      //! wait for jobs
      void work();

      //! start all our threads
      void startThreads();

      //!tell worker threads to quit
      void stopThreads();

      std::vector<std::pair<std::function<void()>, std::promise<void> > > itsJobs;
      std::vector<std::function<void()> > itsQueuedJobs;
      std::mutex itsMutex;
      std::condition_variable itsConditional;
      std::vector<std::thread> itsThreads;
      volatile bool isRunning;
  };

}//namespace nsu
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
