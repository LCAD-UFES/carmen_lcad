/*!@file Media/BufferedInputFrameSeries.H Buffered input with frames loaded in a worker thread */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/BufferedInputFrameSeries.H $
// $Id: BufferedInputFrameSeries.H 9819 2008-06-17 05:48:53Z itti $
//

#ifndef MEDIA_BUFFEREDINPUTFRAMESERIES_H_DEFINED
#define MEDIA_BUFFEREDINPUTFRAMESERIES_H_DEFINED

#include "Component/ModelComponent.H"
#include "Media/FrameSeries.H"
#include "Raster/GenericFrame.H"
#include "rutz/atomic.h"
#include "rutz/circular_queue.h"

#include <list>

/// Buffered version of InputFrameSeries
/** Frames are read from their source in a separate worker thread
    which fills a frame buffer queue; clients of this class running in
    the main thread can frames out of the queue. */
class BufferedInputFrameSeries : public ModelComponent
{
public:
  /// Construct with buffer size qmax
  /** @param qsize size fo the buffer
      @param forcergb convert to RGB in the thread, so that the
      GenericFrame returned will have a native type of RGB instead of
      whatever it might have been from the source. This can save time
      for the recipient since asRGB() will then be a pass-through. */
  BufferedInputFrameSeries(OptionManager& mgr, const size_t qsize,
                           const bool forcergb = false);

  /// Get the specs of our frames
  GenericFrameSpec peekFrameSpec() const;

  /// Returns the next frame out of our buffer
  /** Returns an empty frame if either (1) the input source is
      exhausted, or (2) the input buffer underflowed prior to the
      input source being exhausted. If you care to distinguish between
      these, you can pass a non-null pointer did_underflow which will
      be set to false in case (1) and true in case (2). It is up to
      the caller whether to treat premature underflow as a fatal
      error, or whether to retry after a delay. */
  GenericFrame get(bool* did_underflow = 0);

private:

  virtual void start2();

  virtual void stop1();

  static void* c_fill(void* p);

  struct Checkpoint;

  nub::ref<InputFrameSeries> itsSrc;
  GenericFrameSpec itsFrameSpec;
  rutz::circular_queue<GenericFrame> itsQ;
  pthread_mutex_t itsQmut;
  bool itsInputDone;
  bool itsStop;

  pthread_t itsFillThread;

  rutz::atomic_int_t itsNumFilled;

  int itsMinNumFilled;

  int itsFrameNum;

  std::list<Checkpoint> itsCheckpoints;

  const bool itsForceRGB;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_BUFFEREDINPUTFRAMESERIES_H_DEFINED
