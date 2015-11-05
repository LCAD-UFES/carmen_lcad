/*!@file ModelNeuron/SimulationWorkQueue.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SimulationWorkQueue.C $

#include "ModelNeuron/SimulationWorkQueue.H"

// ######################################################################
// SimulationWorkQueue
// ######################################################################
nsu::SimulationWorkQueue::SimulationWorkQueue() 
    : itsJobs(), itsQueuedJobs(), itsMutex(), itsConditional(), itsThreads(1), isRunning(false)
{ 
  startThreads();
}
  
// ######################################################################
nsu::SimulationWorkQueue::SimulationWorkQueue(const unsigned int numthreads) 
    : itsJobs(), itsQueuedJobs(), itsMutex(), itsConditional(), itsThreads(numthreads), isRunning(false)
{ 
  startThreads();
}

// ######################################################################
nsu::SimulationWorkQueue::~SimulationWorkQueue()
{
  stopThreads();
}

// ######################################################################
void nsu::SimulationWorkQueue::add(std::function<void()> && job)
{
  itsQueuedJobs.push_back(job);
}

// ######################################################################
void nsu::SimulationWorkQueue::go()
{
  push(std::move(itsQueuedJobs));
  itsQueuedJobs.clear();
}

// ######################################################################
std::future<void> nsu::SimulationWorkQueue::push(std::function<void()>&& job)
{
  std::lock_guard<std::mutex> lock(itsMutex);
  itsJobs.push_back(std::make_pair(job, std::promise<void>()));
  itsConditional.notify_one();
  return itsJobs.back().second.get_future();
}

// ######################################################################
void nsu::SimulationWorkQueue::push(std::vector<std::function<void()> > && jobs)
{
  std::vector<std::future<void> > futures;
  std::vector<std::function<void()> >::iterator j(jobs.begin()), end(jobs.end());

  itsMutex.lock();
  while (j != end)
  {
    itsJobs.push_back(std::make_pair(*j, std::promise<void>()));
    futures.push_back(itsJobs.back().second.get_future());
    itsConditional.notify_one();
    ++j;
  }
  itsMutex.unlock();
  
  std::vector<std::future<void> >::iterator f(futures.begin()), endf(futures.end());
  while (f != endf)
    (f++)->get();
}

// ######################################################################
void nsu::SimulationWorkQueue::resize(const unsigned int numthreads)
{
  stopThreads();
  itsThreads.resize(numthreads);
  startThreads();
}

// ######################################################################
unsigned int nsu::SimulationWorkQueue::numThreads() const
{
  return (unsigned int)itsThreads.size();
}

// ######################################################################
void nsu::SimulationWorkQueue::work()
{    
  std::unique_lock<std::mutex> lock(itsMutex);
  while (isRunning)
  {
    while (itsJobs.size() == 0)
      itsConditional.wait(lock);

    std::pair<std::function<void()>, std::promise<void> > job(std::move(itsJobs.back()));
    itsJobs.pop_back();  
    lock.unlock();

    job.first();
    job.second.set_value();

    lock.lock();
  }
}

// ######################################################################
void nsu::SimulationWorkQueue::startThreads()
{
  std::lock_guard<std::mutex> lock(itsMutex);
  if (!isRunning)
  {
    isRunning = true;
    std::vector<std::thread>::iterator thread = itsThreads.begin();
    while (thread != itsThreads.end())
      *thread++ = std::thread(&SimulationWorkQueue::work, this);
  }
}

// ######################################################################
void nsu::SimulationWorkQueue::stopThreads()
{
  itsMutex.lock();
  
  //if we are not running, bail out
  if (!isRunning)
  {
    itsJobs.clear();
    itsMutex.unlock();
    return;
  }
  
  //set state and clear jobs
  isRunning = false;
  itsJobs.clear();
  
  //push as many empty jobs as we have threads
  for (uint i = 0; i < itsThreads.size(); ++i)
    itsJobs.push_back(std::make_pair([](){}, std::promise<void>()));
  
  //wake everybody up
  itsConditional.notify_all();
  itsMutex.unlock();
  
  //join all our threads
  std::vector<std::thread>::iterator thread(itsThreads.begin()), end(itsThreads.end());
  while (thread != end)
  {
    thread->join(); 
    ++thread;
  }
  
  //make sure our job queue is empty
  itsMutex.lock();
  itsJobs.clear();
  itsMutex.unlock();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
