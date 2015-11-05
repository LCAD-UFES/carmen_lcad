/*!@file Channels/IntegerFlickerChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerFlickerChannel.H $
// $Id: IntegerFlickerChannel.H 7857 2007-02-07 21:28:59Z rjpeters $
//

#ifndef CHANNELS_INTEGERFLICKERCHANNEL_H_DEFINED
#define CHANNELS_INTEGERFLICKERCHANNEL_H_DEFINED

#include "Channels/IntegerSimpleChannel.H"

// ######################################################################
//! A temporal flicker channel.
/*! This channel just keeps a memory trace of the previous frame and
  then does a basic center-surround processing on the difference
  between current and memorized input. The time constant determines
  how the memorized input should be updated when a new input
  arrives. */
class IntegerFlickerChannel : public IntegerSimpleChannel
{
public:
  //! Construct with standard specs.
  IntegerFlickerChannel(OptionManager& mgr,
                        nub::ref<IntegerMathEngine> eng);

  //! Destructor
  virtual ~IntegerFlickerChannel();

  //! Reset IntegerFlickerChannel
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

private:
  //! IntegerFlickerChannel requires only luminance input.
  virtual void doInputInt(const IntegerInput& inp,
                          const SimTime& t,
                          PyramidCache<int>* cache,
                          const Image<byte>& clipMask);

  Image<int> itsPrevInput;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERFLICKERCHANNEL_H_DEFINED
