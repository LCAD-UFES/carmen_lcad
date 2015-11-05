/*!@file Util/SharedBuffer.H a thread-safe buffer */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/SharedBuffer.H $

#ifndef UTIL_SHAREDBUFFER_H_DEFINED
#define UTIL_SHAREDBUFFER_H_DEFINED

#include <deque>
#include "Util/Semaphore.H"

// ######################################################################
//A simple thread-safe FIFO logic buffer. This buffer can be used to
//safely read/write buffered data in a multi-threaded
//consumer-producer enviornment. Push() and pop() are gaurenteed
//atomic. The calling processing will block if pop() is called on an
//empty buffer. The Buffer is allowed to grow unbounded.
//######################################################################
template<class T>
class SharedBuffer
{
public:
  //! Construct a shared buffer, FIFO logic, possibly zero padded
  inline SharedBuffer(const uint buffsize = 0);

  //! destructor
  virtual inline ~SharedBuffer();

  //! push data on our queue
  inline void push(const T& element);

  //!pop some data off our queue, removing from the queue
  inline T pop();

  //!post a message to stop waiting on pop(), allowing threads to
  //!exit. Should only be called during cleanup.
  inline void stopWaiting();

private:
  std::deque<T> itsBuffer;
  Semaphore itsReadable,itsSISO;

};

// ######################################################################
//inline implementations
// ######################################################################
template<class T>
inline SharedBuffer<T>::SharedBuffer(const uint zerobuffer) :
  itsBuffer(zerobuffer), itsReadable(0), itsSISO(1)
{ }

// ######################################################################
template<class T>
inline SharedBuffer<T>::~SharedBuffer()
{ }

// ######################################################################
template<class T>
inline void SharedBuffer<T>::push(const T& newitem)
{
  itsSISO.wait();
  itsBuffer.push_front(newitem);
  itsSISO.post();
  itsReadable.post();
}

// ######################################################################
template<class T>
inline T SharedBuffer<T>::pop()
{
  itsReadable.wait();
  itsSISO.wait();
  //locked
  if (itsBuffer.size() > 0)
    {
      T temp = itsBuffer.back();
      itsBuffer.pop_back();
      //unlock
      itsSISO.post();
      return temp;
    }
  T temp = T();
  //unlock
  itsSISO.post();
  itsReadable.post();//return the counter to +1 so nobody blocks.
  return temp;
}

// ######################################################################
template<class T>
inline void SharedBuffer<T>::stopWaiting()
{
  itsReadable.post();
}

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
