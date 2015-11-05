/*!@file Channels/InputHandlerThreaded.H InputHandler subclass than can be plugged into SingleChannel to get multi-threaded computations */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/InputHandlerThreaded.H $
// $Id: InputHandlerThreaded.H 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef CHANNELS_INPUTHANDLERTHREADED_H_DEFINED
#define CHANNELS_INPUTHANDLERTHREADED_H_DEFINED

#include "Channels/InputHandler.H"
#include "rutz/shared_ptr.h"

/// InputHandler subclass than can be plugged into SingleChannel to get multi-threaded computations
class InputHandlerThreaded : public InputHandler
{
public:
  //! Constructor
  InputHandlerThreaded();

  //! Virtual destructor ensures proper destruction of derived classes.
  virtual ~InputHandlerThreaded();

  //! Send the input image off to the processing node
  virtual void handleInput(SingleChannel& chan,
                           const Image<float>& bwimg,
                           const SimTime& t,
                           const Image<byte>& clipMask,
                           const rutz::shared_ptr<PyramidCache<float> >& cache);

  //! Have we received the results from our processing node yet?
  virtual void waitForOutput(SingleChannel& chan);

private:
  virtual rutz::shared_ptr<InputHandler> makeClone() const;

  InputHandlerThreaded(const InputHandlerThreaded&); // not implemented
  InputHandlerThreaded& operator=(const InputHandlerThreaded&); // not implemented

  struct Job;

  rutz::shared_ptr<Job> itsJob;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INPUTHANDLERTHREADED_H_DEFINED
