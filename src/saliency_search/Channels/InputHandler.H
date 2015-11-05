/*!@file Channels/InputHandler.H Hooks for tweaking individual pieces of channel functionality */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/InputHandler.H $
// $Id: InputHandler.H 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef CHANNELS_INPUTHANDLER_H_DEFINED
#define CHANNELS_INPUTHANDLER_H_DEFINED

#include "Util/Types.H"

class SimTime;
class SingleChannel;

template <class T> class Image;
template <class T> class PyramidCache;
namespace rutz { template <class T> class shared_ptr; }

//! Auxiliary class that can override parts of SingleChannel's functionality
class InputHandler
{
public:
  virtual ~InputHandler();

  /// Like h->makeClone(), but works with null pointers, too.
  /** The point of this is to allow us to pass one InputHandler to a
      ComplexChannel, and then let that ComplexChannel make multiple
      clones of the handler to pass to each of its subchannels. This
      means that there may be many clones of a given InputHandler, but
      only some of them (i.e., the ones sitting in SingleChannel
      objects) will ever see any input; the other ones (sitting in
      ComplexChannel objects) won't be used except to instantiate
      additional clones when needed. In terms of implementation, this
      means that InputHandler classes shouldn't do anything "heavy"
      until they actually receive some input. Along those lines, the
      prototypical InputHandler subclass is the beowulf
      BeoInputHandler, which doesn't do a Beowulf::requestNode() in
      its constructor, but rather waits until it gets a handleInput()
      call. That way it is safe to have more BeoInputHandler objects
      than we have Beowulf nodes, as long as only a subset of the
      objects actually request Beowulf nodes. */
  static rutz::shared_ptr<InputHandler> clone(rutz::shared_ptr<InputHandler> h);

  virtual void handleInput(SingleChannel& chan,
                           const Image<float>& bwimg,
                           const SimTime& t,
                           const Image<byte>& clipMask,
                           const rutz::shared_ptr<PyramidCache<float> >& cache) = 0;

  virtual void waitForOutput(SingleChannel& chan) = 0;

private:
  /// Make a clone of this InputHandler object.
  virtual rutz::shared_ptr<InputHandler> makeClone() const = 0;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INPUTHANDLER_H_DEFINED
