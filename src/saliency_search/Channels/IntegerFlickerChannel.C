/*!@file Channels/IntegerFlickerChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerFlickerChannel.C $
// $Id: IntegerFlickerChannel.C 7857 2007-02-07 21:28:59Z rjpeters $
//

#ifndef CHANNELS_INTEGERFLICKERCHANNEL_C_DEFINED
#define CHANNELS_INTEGERFLICKERCHANNEL_C_DEFINED

#include "Channels/IntegerFlickerChannel.H"

#include "Image/IntegerMathOps.H"
#include "Image/MathOps.H" // for absDiff()
#include "rutz/trace.h"

// ######################################################################
// IntegerFlickerChannel member definitions:
// ######################################################################

// ######################################################################
IntegerFlickerChannel::
IntegerFlickerChannel(OptionManager& mgr,
                      nub::ref<IntegerMathEngine> eng) :
  IntegerSimpleChannel(mgr, "Integer Flicker", "int-flicker", FLICKER,
                       rutz::make_shared(new IntgGaussianPyrBuilder
                                         (5, eng->getImath())),
                       eng),
  itsPrevInput()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsTakeAbs.setVal(true);
  itsNormalizeOutput.setVal(true);
}

// ######################################################################
IntegerFlickerChannel::~IntegerFlickerChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void IntegerFlickerChannel::doInputInt(const IntegerInput& inp,
                                       const SimTime& t,
                                       PyramidCache<int>* cache,
                                       const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(inp.grayInt().initialized());

  // If this is the first time the flicker channel has seen input,
  // then itsPrevInput will be uninitialized; obviously we can't
  // compute any flicker with only one frame, so we just store the
  // current input as the next iteration's previous input
  if (!itsPrevInput.initialized())
    {
      itsPrevInput = inp.grayInt();
    }
  else
    {
      // take simple abs difference between current and previous
      // frame:
      const Image<int> fli = absDiff(inp.grayInt(), itsPrevInput);

      IntegerSimpleChannel::doInputInt(IntegerInput::fromGrayOnly(fli),
                                       t, cache, clipMask);

      // update the previous image:
      itsPrevInput = inp.grayInt();
    }
}

// ######################################################################
void IntegerFlickerChannel::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // reset some stuff in IntegerFlickerChannel
  itsPrevInput.freeMem();

  // propagate to our base class:
  IntegerSimpleChannel::reset1();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERFLICKERCHANNEL_C_DEFINED
