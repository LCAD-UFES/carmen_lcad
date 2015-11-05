/*!@file Transport/BufferedFrameIstream.H Frame-buffering layer on top of other FrameIstream objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/BufferedFrameIstream.H $
// $Id: BufferedFrameIstream.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_BUFFEREDFRAMEISTREAM_H_DEFINED
#define TRANSPORT_BUFFEREDFRAMEISTREAM_H_DEFINED

#include "Component/ModelParam.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameIstream.H"
#include "Util/ErrorStrategy.H"
#include "rutz/atomic.h"
#include "rutz/circular_queue.h"

#include <list>

/// Puts a frame-buffering layer on top of any other FrameIstream object
/** Frames are read from their source in a separate worker thread
    which fills a frame buffer queue; clients of this class running in
    the main thread can pull frames out of the queue.

    To use this from the command-line with the standard --in option
    provided by InputFrameSeries, put "buf:" in front of your input
    spec, like this: --in=buf:frame#.pnm or --in=buf:foo.mpg. Then,
    the frames will actually be read in a worker thread that puts the
    frames in a FIFO queue; the main thread can retrieve frames from
    the queue. Control the number of frames in the queue with
    --input-buffer-size=N. To wait until the queue is completely
    filled before starting your main loop, put a startStream() just
    before the loop.
*/
class BufferedFrameIstream : public FrameIstream
{
public:
  BufferedFrameIstream(OptionManager& mgr);

  virtual ~BufferedFrameIstream();

  /// Interpret the config string as a specification for our underlying input source
  virtual void setConfigInfo(const std::string& cfg);

  /// Pass the frame number on to our underlying frame source
  virtual bool setFrameNumber(int n);

  /// Get the specs of our frames
  virtual GenericFrameSpec peekFrameSpec();

  /// Returns the same frame time as our underlying source
  virtual SimTime getNaturalFrameTime() const;

  /// Start filling our buffer
  virtual void startStream();

  /// Returns the next frame out of our buffer
  /** Returns an empty frame if either (1) the input source is
      exhausted, or (2) the input buffer underflowed prior to the
      input source being exhausted. It is up to the caller whether to
      treat premature underflow as a fatal error, or whether to retry
      after a delay. */
  virtual GenericFrame readFrame();

private:
  virtual void start2();

  virtual void stop1();

  static void* c_fill(void* p);

  struct Checkpoint;

  OModelParam<size_t> itsBufSize;

  /// What to do in readFrame() if the buffer is empty due to underflow
  OModelParam<ErrorStrategy> itsUnderflowStrategy;

  nub::soft_ref<FrameIstream> itsSrc;
  rutz::circular_queue<GenericFrame>* itsQ;
  pthread_mutex_t itsQmut;
  bool itsInputDone;
  bool itsStop;

  pthread_t itsFillThread;

  rutz::atomic_int_t itsNumFilled;

  int itsMinNumFilled;

  int itsInputFrameNum;
  int itsOutputFrameNum;

  GenericFrameSpec itsFrameSpec;
  bool itsFrameSpecValid;

  std::list<Checkpoint> itsCheckpoints;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_BUFFEREDFRAMEISTREAM_H_DEFINED
